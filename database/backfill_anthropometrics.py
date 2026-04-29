"""
Backfill Anthropometric Metadata into Segment HDF5 Files
=========================================================

One-time utility that reads participant anthropometric data from the
original trial HDF5 files and writes it as file-level attributes on
each segmented HDF5 file in ``database/segments/``.

Attributes written (all floats, in cm):
    total_arm_length, upper_arm_length, forearm_length,
    hand_length, upper_arm_circumference, forearm_circumference

Usage::

    python database/backfill_anthropometrics.py                       # processes all segments
    python database/backfill_anthropometrics.py --participants P01 P02 # specific participants

The script is safe to re-run; it will overwrite existing attributes.
"""

import argparse
import os
import sys
from pathlib import Path

import h5py

# ---------------------------------------------------------------------------
# Anthropometric attribute names (trial file → segment file)
# ---------------------------------------------------------------------------

#: Mapping from trial-file attribute name → segment-file attribute name
_ANTHRO_KEYS = {
    "Total Arm Length [cm]": "total_arm_length",
    "Upper Arm Length [cm]": "upper_arm_length",
    "Forearm Length [cm]":   "forearm_length",
    "Hand Length [cm]":      "hand_length",
    "Upper Arm Circumference [cm]": "upper_arm_circumference",
    "Fore Arm Circumference [cm]":  "forearm_circumference",
}


def _trigger_icloud_download(path: Path) -> None:
    """Ask macOS to start downloading an iCloud file (no-op if already local)."""
    import subprocess
    try:
        subprocess.run(
            ["brctl", "download", str(path)],
            timeout=5, capture_output=True,
        )
    except Exception:
        pass  # brctl may not exist or may fail — that's okay


def _read_anthropometrics_from_trial(trial_path: Path,
                                      max_retries: int = 3,
                                      base_wait: float = 5.0,
                                      ) -> dict[str, float]:
    """Read anthropometric file-level attrs from a raw trial HDF5 file.

    Retries with exponential backoff to handle iCloud sync delays.
    """
    import time

    _trigger_icloud_download(trial_path)

    for attempt in range(max_retries):
        try:
            result = {}
            with h5py.File(trial_path, "r") as f:
                for src_key, dst_key in _ANTHRO_KEYS.items():
                    val = f.attrs.get(src_key, None)
                    if val is not None:
                        result[dst_key] = float(val)
            return result
        except (TimeoutError, OSError) as exc:
            wait = base_wait * (2 ** attempt)
            if attempt < max_retries - 1:
                print(f"    [RETRY] {trial_path.name} timed out, "
                      f"waiting {wait:.0f}s (attempt {attempt + 1}/{max_retries})...")
                time.sleep(wait)
            else:
                print(f"    [FAIL] {trial_path.name}: {exc}")
    return {}


def _collect_participant_anthropometrics(db_dir: Path,
                                         participants: list[str] | None = None
                                         ) -> dict[str, dict[str, float]]:
    """
    Build a mapping  participant_id → {anthro_key: value, ...}
    by scanning trial files of each participant directory.

    Tries multiple trial files per participant in case some are
    not yet synced from iCloud.
    """
    anthro_map: dict[str, dict[str, float]] = {}

    part_dirs = sorted(db_dir.glob("participant_P*"))
    if participants:
        part_dirs = [p for p in part_dirs if p.name in [f"participant_{pid}" for pid in participants]]

    for pdir in part_dirs:
        pid = pdir.name.split("_")[1]  # "P01", "P02", ...
        # Find trial files inside any session
        trial_files = sorted(pdir.rglob("trial_*.h5"))
        if not trial_files:
            print(f"[WARN] No trial files found for {pid}")
            continue

        # Try multiple trial files in case some are stuck in iCloud
        anthro: dict[str, float] = {}
        for tf in trial_files[:5]:  # try up to 5 files
            anthro = _read_anthropometrics_from_trial(tf)
            if anthro:
                break

        if anthro:
            anthro_map[pid] = anthro
            print(f"  {pid}: {anthro}")
        else:
            print(f"  {pid}: SKIPPED — could not read anthropometrics "
                  f"(tried {min(len(trial_files), 5)} files)")

    return anthro_map


def backfill_segments(db_dir: Path, participants: list[str] | None = None) -> None:
    """
    Main routine: read anthropometrics from trial files and write them
    as file-level attributes on each segment HDF5 file.
    """
    segments_dir = db_dir / "segments"
    if not segments_dir.exists():
        sys.exit(f"[ERR] Segments directory not found: {segments_dir}")

    print("=== Collecting anthropometric data from trial files ===")
    anthro_map = _collect_participant_anthropometrics(db_dir, participants)

    if not anthro_map:
        sys.exit("[ERR] No anthropometric data collected. Cannot proceed.")

    print(f"\n=== Backfilling {len(anthro_map)} participant(s) into segment files ===")

    seg_files = sorted(segments_dir.glob("participant_*_segments.h5"))
    if participants:
        seg_files = [f for f in seg_files
                     if any(pid in f.name for pid in participants)]

    updated = 0
    skipped = 0

    for seg_path in seg_files:
        # Extract participant ID from filename: participant_P01_session_01_segments.h5
        parts = seg_path.stem.split("_")
        pid_idx = parts.index("participant") + 1 if "participant" in parts else -1
        if pid_idx < 0 or pid_idx >= len(parts):
            print(f"  [SKIP] Cannot parse participant ID from {seg_path.name}")
            skipped += 1
            continue

        pid = parts[pid_idx]  # "P01"
        if pid not in anthro_map:
            print(f"  [SKIP] No anthropometric data for {pid} ({seg_path.name})")
            skipped += 1
            continue

        anthro = anthro_map[pid]
        with h5py.File(seg_path, "a") as f:
            for attr_name, value in anthro.items():
                f.attrs[attr_name] = value

        print(f"  [OK] {seg_path.name} ← {pid} ({len(anthro)} attributes)")
        updated += 1

    print(f"\n=== Done: {updated} files updated, {skipped} skipped ===")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Backfill anthropometric metadata into segment HDF5 files."
    )
    parser.add_argument(
        "--db-dir",
        default=str(Path(__file__).parent),
        help="Path to the database directory (default: same dir as this script).",
    )
    parser.add_argument(
        "--participants",
        nargs="+",
        help="Specific participant IDs to process (e.g. P01 P02). Default: all.",
    )
    args = parser.parse_args()
    backfill_segments(Path(args.db_dir), args.participants)
