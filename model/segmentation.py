"""
Signal Segmentation for Trial HDF5 Database
============================================

Segments the synchronized multi-channel signals (IMU + processed EMG) into
labelled windows based on the Phase 2 state machine events stored in each
HDF5 trial file.

Segment logic
-------------
- **Only Phase 2** events are used; Phase 1 and error events are ignored.
- A pair of consecutive Phase 2 events defines one segment window:
    PHASE_2_AWAITING_PICKUP   → window until next event  →  label: "free_movement"
    PHASE_2_AWAITING_PLACEMENT → window until next event  →  label: "carrying"
- If any event in a consecutive pair is an error state the pair is DISCARDED.
- A configurable time margin (default 200 ms) is applied asymmetrically:
  it shrinks the end of a **free_movement** segment and expands the start of
  a **weight/carrying** segment (pickup), but remains 0.0 at the transition
  from carrying back to free movement (placement).
- Primary label: `weight` (kg); also stored: `state`, source/target cell,
  `t_start`, `t_end` (without margin), trial file path.

Signal sources
--------------
- IMU channels  : `synced/data`  – columns yaw1/pitch1/roll1/ax1/ay1/az1 +
                                    yaw2/pitch2/roll2/ax2/ay2/az2
- Processed EMG : `synced/emg_processed` – all ch* columns that are not
                  entirely NaN (plugged electrodes only).
  Both datasets share the same row grid (EMG sample clock) so alignment is
  done row-by-row.

Output
------
Each segment is a plain dict:
  {
    "label"   : float | str,    # numeric weight [kg] for carrying;
                                 # the string "free_movement" for free movement
    "weight"  : float,          # always the numeric weight (kg) for reference
    "state"   : str,            # "free_movement" | "carrying"
    "src_row" : int,
    "src_col" : int,
    "tgt_row" : int,
    "tgt_col" : int,
    "t_start" : float,          # event time, no margin
    "t_end"   : float,          # next event time, no margin
    "t_start_m": float,         # t_start - margin  (used for slicing)
    "t_end_m"  : float,         # t_end   + margin
    "imu"     : np.ndarray,     # shape (N, 12)
    "emg"     : np.ndarray,     # shape (N, n_emg)
    "imu_cols": list[str],
    "emg_cols": list[str],
    "emg_fs"  : float,
    "emg_fs_orig": float,
    "imu_fs"  : float,
    "imu_fs_orig": float,
    "trial_file": str,
  }

Usage
-----
As a module::

    from model.segmentation import SegmentedDataset
    ds = SegmentedDataset(margin_s=0.1)
    segments = ds.segment_database("database/")

As a script::

    python model/segmentation.py database/participant_P01/trial_1.h5 \\
        --margin 0.1 --out /tmp/segs --format hdf5
"""

import argparse
import os
import shutil
import sys
import tempfile
from pathlib import Path
from typing import Any

import h5py
import numpy as np
from tqdm import tqdm

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: IMU column names expected inside synced/data
IMU_COLUMNS = [
    "yaw1", "pitch1", "roll1", "ax1", "ay1", "az1",
    "yaw2", "pitch2", "roll2", "ax2", "ay2", "az2",
]

#: Phase 2 states that carry valid movement info
_PHASE2_STATES = {"PHASE_2_AWAITING_PICKUP", "PHASE_2_AWAITING_PLACEMENT"}

#: Error states – segments that start or end at these are discarded
_ERROR_STATES = {
    "ERROR_WRONG_PICKUP",
    "ERROR_WRONG_PLACEMENT",
    "ERROR_MISSING_WEIGHT",
}

#: Mapping from Phase 2 state → human-readable label
_STATE_TO_LABEL = {
    "PHASE_2_AWAITING_PICKUP": "free_movement",
    "PHASE_2_AWAITING_PLACEMENT": "carrying",
}


# ---------------------------------------------------------------------------
# Helper dataclass (namedtuple-like)
# ---------------------------------------------------------------------------

class _Event:
    """Lightweight container for a parsed trial event."""

    __slots__ = ("t_pc", "state", "weight", "src_row", "src_col", "tgt_row", "tgt_col")

    def __init__(self, t_pc, state, weight, src_row, src_col, tgt_row, tgt_col):
        self.t_pc = float(t_pc)
        self.state = state
        self.weight = float(weight)
        self.src_row = int(src_row)
        self.src_col = int(src_col)
        self.tgt_row = int(tgt_row)
        self.tgt_col = int(tgt_col)

    def is_phase2(self) -> bool:
        return self.state in _PHASE2_STATES

    def is_error(self) -> bool:
        return self.state in _ERROR_STATES

    def __repr__(self):
        return (
            f"_Event(t={self.t_pc:.3f}, state={self.state!r}, "
            f"weight={self.weight}, src=({self.src_row},{self.src_col}), "
            f"tgt=({self.tgt_row},{self.tgt_col}))"
        )


# ---------------------------------------------------------------------------
# Core class
# ---------------------------------------------------------------------------

class SegmentedDataset:
    """
    Segments Phase-2 multi-channel signals from HDF5 trial files.

    Parameters
    ----------
    margin_s : float
        Margin in seconds applied asymmetrically. It shrinks the end of a
        **free_movement** segment (before pickup) and expands the start of a
        **carrying** segment (at pickup). Transition at placement (carrying ->
        free_movement) uses 0.0 margin. Default is 0.2 s (200 ms).
    """

    def __init__(self, margin_s: float = 0.2):
        self.margin_s = float(margin_s)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def segment_trial(self, file_path: str | Path) -> list[dict[str, Any]]:
        """
        Segment a single trial HDF5 file.

        Returns
        -------
        list[dict]
            One dict per segment (see module docstring for field descriptions).
        """
        file_path = Path(file_path)
        if not file_path.exists():
            raise FileNotFoundError(f"HDF5 file not found: {file_path}")

        with h5py.File(file_path, "r") as f:
            events = self._load_events(f)
            segments_meta = self._build_segments(events)

            if not segments_meta:
                print(f"[SEGMENTER] No Phase 2 segments found in {file_path.name}")
                return []

            t_common, imu_data, emg_data, imu_cols, emg_cols, fs_info = self._load_signal_matrix(f)

        segments = []
        for meta in segments_meta:
            seg = self._slice_segment(meta, t_common, imu_data, emg_data,
                                      imu_cols, emg_cols, fs_info, str(file_path))
            if seg is not None:
                segments.append(seg)

        print(
            f"[SEGMENTER] {file_path.name}: {len(segments)} segments "
            f"({sum(1 for s in segments if s['state']=='free_movement')} free_movement, "
            f"{sum(1 for s in segments if s['state']=='carrying')} carrying)"
        )
        return segments

    def segment_session(
        self, session_dir: str | Path
    ) -> list[dict[str, Any]]:
        """
        Segment all HDF5 trial files in a single session directory.

        Parameters
        ----------
        session_dir : path to a ``session_NN`` directory

        Returns
        -------
        list[dict]
            Combined segments from every trial in the session.
        """
        session_dir = Path(session_dir)
        h5_files = sorted(session_dir.glob("*.h5"))

        if not h5_files:
            print(f"[SEGMENTER] No HDF5 files found in {session_dir}")
            return []

        print(f"[SEGMENTER] Session '{session_dir.name}': {len(h5_files)} trial file(s).")
        all_segments: list[dict] = []
        for fp in h5_files:
            try:
                segs = self.segment_trial(fp)
                all_segments.extend(segs)
            except Exception as exc:
                print(f"[SEGMENTER] WARNING: Skipping {fp.name} — {exc}")

        print(
            f"[SEGMENTER] Session '{session_dir.name}' total: {len(all_segments)} segments."
        )
        return all_segments

    def segment_database(
        self, database_dir: str | Path, participants: list[str] = None
    ) -> dict[str, list[dict[str, Any]]]:
        """
        Segment all HDF5 trial files grouped by session directory.

        Traverses the folder structure::

            database_dir/
              participant_PXX/
                session_NN/
                  trial_*.h5

        Returns
        -------
        dict[str, list[dict]]
            Mapping of ``"participant_PXX/session_NN"`` → list of segments.
            All sessions across all participants are included.
        """
        base = Path(database_dir)

        # Find all session_NN directories (depth 2: participant/session)
        session_dirs = sorted(
            p for p in base.rglob("session_*") if p.is_dir()
        )

        if participants:
            session_dirs = [p for p in session_dirs if p.parent.name in participants]


        # Fallback: if no session_* dirs exist, treat every participant dir as
        # a flat collection (backwards compat with old layout)
        if not session_dirs:
            print(
                "[SEGMENTER] No session_* directories found — "
                "falling back to flat scan."
            )
            h5_files = sorted(base.rglob("*.h5"))
            if participants:
                h5_files = [fp for fp in h5_files if any(p in fp.parts for p in participants)]
            if not h5_files:
                print(f"[SEGMENTER] No HDF5 files found under {database_dir}")
                return {}
            print(f"[SEGMENTER] Found {len(h5_files)} HDF5 file(s) (flat mode).")
            segs: list[dict] = []
            for fp in h5_files:
                try:
                    segs.extend(self.segment_trial(fp))
                except Exception as exc:
                    print(f"[SEGMENTER] WARNING: Skipping {fp.name} — {exc}")
            return {base.name: segs}

        print(f"[SEGMENTER] Found {len(session_dirs)} session directory(ies).")
        results: dict[str, list[dict]] = {}
        for sdir in tqdm(session_dirs, desc="Segmenting database", unit="session"):
            # Key: e.g. "participant_P01/session_01"
            participant = sdir.parent.name
            session = sdir.name
            key = f"{participant}/{session}"
            tqdm.write(f"[SEGMENTER] Processing {participant} | {session}")
            results[key] = self.segment_session(sdir)

        total = sum(len(v) for v in results.values())
        print(f"[SEGMENTER] Grand total: {total} segments across {len(results)} session(s).")
        return results

    # ------------------------------------------------------------------
    # Save helpers
    # ------------------------------------------------------------------

    def save_segments_to_hdf5(
        self, segments: list[dict], out_path: str | Path
    ) -> Path:
        """
        Save segments into a new HDF5 file.

        Layout::

            /segment_0000/
                imu          float64 (N, 12)  — attrs: column_names
                emg          float64 (N, M)   — attrs: column_names
                              group attrs: label, state, weight, src_row, src_col,
                                           tgt_row, tgt_col, t_start, t_end,
                                           t_start_margin, t_end_margin, trial_file

        Parameters
        ----------
        segments : list[dict]
        out_path : path to the output .h5 file (created/overwritten)
        """
        out_path = Path(out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # Write to a local temporary file first to avoid iCloud block timeouts
        fd, temp_path = tempfile.mkstemp(suffix=".h5")
        os.close(fd)

        try:
            with h5py.File(temp_path, "w") as f:
                f.attrs["n_segments"] = len(segments)
                f.attrs["margin_s"] = self.margin_s

                for i, seg in enumerate(segments):
                    grp = f.create_group(f"segment_{i:04d}")

                    # Signal datasets
                    ds_imu = grp.create_dataset(
                        "imu", data=seg["imu"].astype(np.float64),
                        compression="gzip", compression_opts=4
                    )
                    ds_imu.attrs["column_names"] = [c.encode() for c in seg["imu_cols"]]
                    ds_imu.attrs["fs"] = seg.get("imu_fs", 500.0)
                    ds_imu.attrs["fs_orig"] = seg.get("imu_fs_orig", 500.0)

                    ds_emg = grp.create_dataset(
                        "emg", data=seg["emg"].astype(np.float64),
                        compression="gzip", compression_opts=4
                    )
                    ds_emg.attrs["column_names"] = [c.encode() for c in seg["emg_cols"]]
                    ds_emg.attrs["fs"] = seg.get("emg_fs", 4000.0)
                    ds_emg.attrs["fs_orig"] = seg.get("emg_fs_orig", 4000.0)

                    # Metadata attributes
                    # label is either a float (carrying) or the string "free_movement"
                    grp.attrs["label"] = seg["label"]
                    grp.attrs["weight"] = seg["weight"]
                    grp.attrs["state"] = seg["state"]
                    grp.attrs["src_row"] = seg["src_row"]
                    grp.attrs["src_col"] = seg["src_col"]
                    grp.attrs["tgt_row"] = seg["tgt_row"]
                    grp.attrs["tgt_col"] = seg["tgt_col"]
                    grp.attrs["t_start"] = seg["t_start"]
                    grp.attrs["t_end"] = seg["t_end"]
                    grp.attrs["t_start_margin"] = seg["t_start_m"]
                    grp.attrs["t_end_margin"] = seg["t_end_m"]
                    grp.attrs["trial_file"] = seg["trial_file"]

            # Move completed file to the final destination (overwriting if exists)
            shutil.move(temp_path, out_path)

        except Exception:
            if os.path.exists(temp_path):
                os.remove(temp_path)
            raise

        print(f"[SEGMENTER] Saved {len(segments)} segments → {out_path}")
        return out_path

    def save_segments_to_npz(
        self, segments: list[dict], out_path: str | Path
    ) -> Path:
        """
        Save segments to a compressed NumPy archive (.npz).

        Arrays stored with keys  ``imu_NNNN`` / ``emg_NNNN``.
        A structured metadata array (one row per segment) is also stored
        under the key ``metadata`` with fields:
        label, state, src_row, src_col, tgt_row, tgt_col, t_start, t_end,
        t_start_margin, t_end_margin, trial_file.
        """
        out_path = Path(out_path)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # Write to local temp file to avoid iCloud Drive file locks
        fd, temp_path = tempfile.mkstemp(suffix=".npz")
        os.close(fd)

        arrays: dict[str, np.ndarray] = {}

        for i, seg in enumerate(segments):
            arrays[f"imu_{i:04d}"] = seg["imu"].astype(np.float64)
            arrays[f"emg_{i:04d}"] = seg["emg"].astype(np.float64)

        # Metadata as object array so mixed types can be stored
        # label is str for free_movement, float for carrying — store as U32
        meta_dtype = np.dtype([
            ("label", "U32"),   # "free_movement" or "2.25" etc.
            ("weight", "f8"),   # always numeric
            ("state", "U32"),
            ("src_row", "i4"), ("src_col", "i4"),
            ("tgt_row", "i4"), ("tgt_col", "i4"),
            ("t_start", "f8"), ("t_end", "f8"),
            ("t_start_margin", "f8"), ("t_end_margin", "f8"),
            ("trial_file", "U256"),
            ("imu_cols", "U256"),
            ("emg_cols", "U256"),
            ("emg_fs", "f8"),
            ("emg_fs_orig", "f8"),
            ("imu_fs", "f8"),
            ("imu_fs_orig", "f8"),
        ])
        meta = np.zeros(len(segments), dtype=meta_dtype)
        for i, seg in enumerate(segments):
            label_str = str(seg["label"])  # "free_movement" or e.g. "2.25"
            meta[i] = (
                label_str, seg["weight"], seg["state"],
                seg["src_row"], seg["src_col"],
                seg["tgt_row"], seg["tgt_col"],
                seg["t_start"], seg["t_end"],
                seg["t_start_m"], seg["t_end_m"],
                seg["trial_file"],
                ",".join(seg["imu_cols"]),
                ",".join(seg["emg_cols"]),
                seg.get("emg_fs", 4000.0),
                seg.get("emg_fs_orig", 4000.0),
                seg.get("imu_fs", 500.0),
                seg.get("imu_fs_orig", 500.0),
            )
        arrays["metadata"] = meta

        try:
            np.savez_compressed(temp_path, **arrays)
            shutil.move(temp_path, out_path)
        except Exception:
            if os.path.exists(temp_path):
                os.remove(temp_path)
            raise

        print(f"[SEGMENTER] Saved {len(segments)} segments → {out_path}")
        return out_path

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _load_events(self, f: h5py.File) -> list[_Event]:
        """
        Read the structured _raw/events dataset and return a sorted list of
        _Event objects, including ALL states (so we can detect error boundaries).
        """
        if "_raw/events" not in f or f["_raw/events"].shape[0] == 0:
            return []

        raw = f["_raw/events"][:]
        events: list[_Event] = []
        for row in raw:
            state = row["state"]
            if isinstance(state, (bytes, np.bytes_)):
                state = state.decode("utf-8")
            events.append(_Event(
                t_pc=row["t_pc"],
                state=state,
                weight=row["weight"],
                src_row=row["src_r"],
                src_col=row["src_c"],
                tgt_row=row["tgt_r"],
                tgt_col=row["tgt_c"],
            ))

        # Events are logged in order, but sort by time for safety
        events.sort(key=lambda e: e.t_pc)
        return events

    def _build_segments(self, events: list[_Event]) -> list[dict]:
        """
        Convert the event list into segment metadata dicts.

        Strategy
        --------
        Walk the event list forward. Each Phase 2 event starts a window that
        ends at the next event's timestamp. If the *starting* event or the
        *boundary* event is an error state, discard the window.

        The "next event" determines the t_end. For the very last Phase 2 event,
        t_end is set to the FINISHED event's t if present, otherwise the
        segment is omitted (no closed boundary).
        """
        segments: list[dict] = []

        for i, evt in enumerate(events):
            # We only start a segment on Phase 2 events
            if not evt.is_phase2():
                continue

            # Find the next event (any state) to close this window
            if i + 1 >= len(events):
                # Last event — no closing boundary; skip
                continue

            next_evt = events[i + 1]

            # If next event is an error state, this window crosses an error:
            # discard to avoid including the erroneous motion
            if next_evt.is_error():
                continue

            # If the gap skips back through an error (i.e. the event right
            # after error recovery is the next one we'd use), we still include
            # it — recovery means the participant corrected themselves and the
            # motion continues normally.

            t_start = evt.t_pc
            t_end = next_evt.t_pc

            # Sanity: skip zero-length or negative windows
            if t_end <= t_start:
                continue

            seg_state = _STATE_TO_LABEL[evt.state]

            # Primary label:
            #   carrying     → numeric weight [kg]  (what is being moved)
            #   free_movement → special string label (weight context is irrelevant)
            if seg_state == "carrying":
                label = evt.weight
            else:
                label = "free_movement"

            # Apply margin per segment type, but only at the pickup boundary.
            # Pickup boundary = start of 'carrying' / end of 'free_movement'.
            # Drop/Placement boundary = end of 'carrying' / start of 'free_movement'.
            
            if seg_state == "carrying":
                # Starts at pickup (expand by margin_s), ends at placement (0.0)
                t_start_m = t_start - self.margin_s
                t_end_m = t_end
            else:
                # Starts at placement (0.0), ends at pickup (shrink by margin_s)
                t_start_m = t_start
                t_end_m = t_end - self.margin_s

            segments.append({
                "state": seg_state,
                "label": label,
                "weight": evt.weight,   # always numeric, stored for reference
                "src_row": evt.src_row,
                "src_col": evt.src_col,
                "tgt_row": evt.tgt_row,
                "tgt_col": evt.tgt_col,
                "t_start": t_start,
                "t_end": t_end,
                "t_start_m": t_start_m,
                "t_end_m": t_end_m,
            })

        return segments

    def _load_signal_matrix(
        self, f: h5py.File
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str], list[str], dict[str, float]]:
        """
        Load the signal matrix from the HDF5 file.

        Returns
        -------
        t_common : (N,) float64
            The common PC timestamp for every row.
        imu_data : (N, 12) float64
        emg_data : (N, M) float64  — M = number of non-all-NaN EMG channels
        imu_cols : list[str]       — always == IMU_COLUMNS (12 names)
        emg_cols : list[str]       — M names
        fs_info  : dict[str, float] — Contains emg_fs, emg_fs_orig, imu_fs, imu_fs_orig
        """
        fs_info = {
            "emg_fs": 4000.0, "emg_fs_orig": 4000.0,
            "imu_fs": 500.0, "imu_fs_orig": 500.0
        }

        # ---- IMU: from synced/data ----
        if "synced/data" not in f:
            raise KeyError("'synced/data' not found in HDF5 file.")

        ds_synced = f["synced/data"]
        raw_cols = list(ds_synced.attrs.get("column_names", []))
        synced_cols = [c.decode() if isinstance(c, (bytes, np.bytes_)) else str(c)
                       for c in raw_cols]

        synced_data = ds_synced[:]  # Load entire matrix into RAM

        # t_pc_common
        if "t_pc_common" not in synced_cols:
            raise KeyError("'t_pc_common' column not found in synced/data.")
        t_common = synced_data[:, synced_cols.index("t_pc_common")]

        # ---- IMU: from synced/imu_processed (fallback to synced/data) ----
        if "synced/imu_processed" in f:
            ds_imu = f["synced/imu_processed"]
            raw_imu_cols = list(ds_imu.attrs.get("column_names", []))
            imu_cols_found = [c.decode() if isinstance(c, (bytes, np.bytes_)) else str(c) for c in raw_imu_cols]
            
            fs_info["imu_fs"] = ds_imu.attrs.get("fs", 500.0)
            fs_info["imu_fs_orig"] = ds_imu.attrs.get("fs_orig", fs_info["imu_fs"])
            
            # Ignore t_pc_common and t_tmsi from imu_processed to avoid duplicate columns
            # if we just want pure IMU fields:
            imu_indices = []
            final_imu_cols = []
            for i, c in enumerate(imu_cols_found):
                if c not in ["t_pc_common", "t_tmsi"]:
                    imu_indices.append(i)
                    final_imu_cols.append(c)
                    
            imu_data = ds_imu[:, imu_indices]
            imu_cols_found = final_imu_cols
        else:
            print("[SEGMENTER] WARNING: 'synced/imu_processed' not found. Falling back to raw IMU.")
            imu_indices = []
            imu_cols_found = []
            for col in IMU_COLUMNS:
                if col in synced_cols:
                    imu_indices.append(synced_cols.index(col))
                    imu_cols_found.append(col)
                else:
                    print(f"[SEGMENTER] WARNING: IMU column '{col}' not found in synced/data, skipped.")
            imu_data = synced_data[:, imu_indices]

        # ---- EMG: from synced/emg_processed ----
        if "synced/emg_processed" not in f:
            raise KeyError("'synced/emg_processed' not found in HDF5 file. "
                           "Run model/preprocessing.py first.")

        ds_emg = f["synced/emg_processed"]
        raw_emg_cols = list(ds_emg.attrs.get("column_names", []))
        emg_cols_all = [c.decode() if isinstance(c, (bytes, np.bytes_)) else str(c)
                        for c in raw_emg_cols]

        fs_info["emg_fs"] = ds_emg.attrs.get("fs", 4000.0)
        fs_info["emg_fs_orig"] = ds_emg.attrs.get("fs_orig", fs_info["emg_fs"])

        emg_raw = ds_emg[:]

        # If emg_processed has a different number of rows than synced/data,
        # we align by row count (both are on the same EMG sample grid).
        n_rows = synced_data.shape[0]
        if emg_raw.shape[0] != n_rows:
            # Truncate to the common length (should almost never happen)
            min_rows = min(n_rows, emg_raw.shape[0])
            print(
                f"[SEGMENTER] WARNING: synced/data has {n_rows} rows but "
                f"emg_processed has {emg_raw.shape[0]} rows — truncating to {min_rows}."
            )
            emg_raw = emg_raw[:min_rows]
            t_common = t_common[:min_rows]
            imu_data = imu_data[:min_rows]
            n_rows = min_rows

        # Select columns that are not entirely NaN and not timestamps
        ch_indices = []
        ch_names = []
        for i, col in enumerate(emg_cols_all):
            if col in ["t_pc_common", "t_tmsi"] or col.startswith("t_"):
                continue  # skip timestamp columns inside emg_processed
            if not np.all(np.isnan(emg_raw[:, i])):
                ch_indices.append(i)
                ch_names.append(col)
            else:
                print(f"[SEGMENTER] Dropping all-NaN EMG channel '{col}'")

        emg_data = emg_raw[:, ch_indices]

        return t_common, imu_data, emg_data, imu_cols_found, ch_names, fs_info

    def _slice_segment(
        self,
        meta: dict,
        t_common: np.ndarray,
        imu_data: np.ndarray,
        emg_data: np.ndarray,
        imu_cols: list[str],
        emg_cols: list[str],
        fs_info: dict[str, float],
        trial_file: str,
    ) -> dict | None:
        """
        Slice the signal arrays using the margined time window from `meta`.
        Returns None if the window contains no samples.
        """
        t_lo = meta["t_start_m"]
        t_hi = meta["t_end_m"]

        mask = (t_common >= t_lo) & (t_common <= t_hi)
        n_samples = int(np.sum(mask))

        if n_samples == 0:
            print(
                f"[SEGMENTER] WARNING: No samples in window "
                f"[{t_lo:.3f}, {t_hi:.3f}] — segment skipped."
            )
            return None

        return {
            **meta,
            "imu": imu_data[mask],
            "emg": emg_data[mask],
            "imu_cols": imu_cols,
            "emg_cols": emg_cols,
            "emg_fs": fs_info["emg_fs"],
            "emg_fs_orig": fs_info["emg_fs_orig"],
            "imu_fs": fs_info["imu_fs"],
            "imu_fs_orig": fs_info["imu_fs_orig"],
            "trial_file": trial_file,
        }


# ---------------------------------------------------------------------------
# CLI helpers
# ---------------------------------------------------------------------------

def _print_summary(segments: list, label: str, out_dir: Path):
    """Print a human-readable segment summary."""
    from collections import Counter
    # label field is mixed: "free_movement" (str) or float — sort by str key
    label_counts = Counter(str(s["label"]) for s in segments)
    state_counts = Counter(s["state"] for s in segments)
    carrying = [s for s in segments if s["state"] == "carrying"]
    weight_counts = Counter(s["weight"] for s in carrying)
    print(f"\n=== Segmentation Summary: {label} ===")
    print(f"  Total segments : {len(segments)}")
    print(f"  States         : {dict(state_counts)}")
    print(f"  Carrying weights: {dict(sorted(weight_counts.items()))}")
    print(f"  Output         : {out_dir}")


def _run_cli(args):
    target = Path(args.path)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    ds = SegmentedDataset(margin_s=args.margin)
    fmt = args.format.lower()

    def _save(segs, name):
        """Save segments under out_dir with the given base name."""
        if not segs:
            print(f"[SEGMENTER] No segments for '{name}' — skipping save.")
            return
        if fmt in ("hdf5", "h5"):
            ds.save_segments_to_hdf5(segs, out_dir / f"{name}_segments.h5")
        elif fmt == "npz":
            ds.save_segments_to_npz(segs, out_dir / f"{name}_segments.npz")
        elif fmt == "both":
            ds.save_segments_to_hdf5(segs, out_dir / f"{name}_segments.h5")
            ds.save_segments_to_npz(segs, out_dir / f"{name}_segments.npz")
        else:
            sys.exit(f"[ERR] Unknown format '{fmt}'. Use hdf5, npz, or both.")

    def _check_exists(name):
        """Check if output files for 'name' already exist."""
        if not args.skip_existing:
            return False
        
        # Check HDF5
        if fmt in ("hdf5", "h5", "both"):
            if (out_dir / f"{name}_segments.h5").exists():
                return True
        # Check NPZ
        if fmt in ("npz", "both"):
            if (out_dir / f"{name}_segments.npz").exists():
                return True
        return False

    if target.is_file():
        if _check_exists(target.stem):
            print(f"[SEGMENTER] Skipping existing trial: {target.name}")
        else:
            segments = ds.segment_trial(target)
            _save(segments, target.stem)
            _print_summary(segments, target.name, out_dir)

    elif target.is_dir():
        # Find all session_NN directories directly to allow skipping
        # (mirroring the logic in segment_database)
        session_dirs = sorted(
            p for p in target.rglob("session_*") if p.is_dir()
        )
        if args.participants:
            session_dirs = [p for p in session_dirs if p.parent.name in args.participants]

        all_segments = []
        
        if not session_dirs:
            # Fallback to flat scan (single participant directory or similar)
            h5_files = sorted(target.rglob("*.h5"))
            if args.participants:
                h5_files = [fp for fp in h5_files if any(p in fp.parts for p in args.participants)]
            if h5_files:
                name = target.name
                if not _check_exists(name):
                    segs = []
                    for fp in h5_files:
                        try:
                            segs.extend(ds.segment_trial(fp))
                        except Exception as exc:
                            print(f"[SEGMENTER] WARNING: Skipping {fp.name} — {exc}")
                    _save(segs, name)
                    all_segments = segs
                else:
                    print(f"[SEGMENTER] Skipping existing flat database/participant: {name}")
        else:
            # Process sessions one by one
            for sdir in tqdm(session_dirs, desc="Segmenting participants", unit="session"):
                # Key: e.g. "participant_P01/session_01"
                participant = sdir.parent.name
                session = sdir.name
                session_key = f"{participant}/{session}"
                safe_name = session_key.replace("/", "_").replace("\\", "_")
                
                if _check_exists(safe_name):
                    tqdm.write(f"[SEGMENTER] Skipping existing: {session_key}")
                    continue
                
                tqdm.write(f"\n[SEGMENTER] Processing {participant} | {session}")
                segs = ds.segment_session(sdir)
                _save(segs, safe_name)
                all_segments.extend(segs)

        if all_segments:
            _print_summary(all_segments, str(target), out_dir)

    else:
        sys.exit(f"[ERR] Path not found: {target}")



# ---------------------------------------------------------------------------
# Precomputed feature helpers
# ---------------------------------------------------------------------------

def save_precomputed_features(
    h5_path: str | Path,
    segment_features: dict[str, dict],
    feature_config: dict | None = None,
) -> None:
    """
    Write pre-extracted features into an existing segmented HDF5 file.

    For each segment key (e.g. ``"segment_0000"``) in *segment_features*,
    a ``"features"`` group is created (or overwritten) inside that segment
    group containing:

    * One scalar dataset per feature (float64).
    * A ``"feature_names"`` string dataset listing all feature names in order.
    * If *feature_config* is supplied its contents are serialised as JSON and
      stored as a group attribute ``"feature_config_json"`` so the extraction
      settings are permanently recorded alongside the data.

    Parameters
    ----------
    h5_path : path to the segmented .h5 file (must already exist).
    segment_features : mapping  segment_key → {feature_name: value, ...}
        ``value`` may be a scalar **or** a list/array (for sequence features).
    feature_config : optional dict from ``FEATURE_CONFIG`` to store as metadata.
    """
    import json as _json
    import tempfile as _tempfile
    import shutil as _shutil

    h5_path = Path(h5_path)
    if not h5_path.exists():
        raise FileNotFoundError(f"Segment HDF5 file not found: {h5_path}")

    config_json = _json.dumps(feature_config, default=str) if feature_config else ""

    # Write to a local temporary file first to avoid iCloud block timeouts
    fd, temp_path = _tempfile.mkstemp(suffix=".h5")
    os.close(fd)
    _shutil.copy2(h5_path, temp_path)

    try:
        with h5py.File(temp_path, "a") as f:
            for seg_key, feat_dict in segment_features.items():
                if seg_key not in f:
                    print(f"[SEGMENTER] WARNING: '{seg_key}' not found in {h5_path.name} — skipping.")
                    continue

                grp = f[seg_key]

                # Remove old features group if it exists so we can write fresh data
                if "features" in grp:
                    del grp["features"]

                feat_grp = grp.create_group("features")

                if config_json:
                    feat_grp.attrs["feature_config_json"] = config_json

                # Separate scalar features from sequence features
                scalar_names: list[str] = []
                seq_names: list[str] = []

                for name, val in feat_dict.items():
                    if isinstance(val, (list, np.ndarray)):
                        # Sequence feature: list of dicts or array
                        seq_names.append(name)
                    else:
                        scalar_names.append(name)

                # --- scalar features stored as a compact 1-D float array ----------
                if scalar_names:
                    scalar_vals = np.array(
                        [feat_dict[n] for n in scalar_names], dtype=np.float64
                    )
                    feat_grp.create_dataset(
                        "scalar_values",
                        data=scalar_vals,
                        compression="gzip",
                        compression_opts=4,
                    )
                    feat_grp.attrs["scalar_names"] = [n.encode() for n in scalar_names]

                # --- sequence features (list of window-dicts) ---------------------
                if "sequence_dicts" in feat_dict:
                    windows: list[dict] = feat_dict["sequence_dicts"]  # type: ignore[assignment]
                    if windows:
                        all_keys = list(windows[0].keys())
                        arr = np.array(
                            [[w[k] for k in all_keys] for w in windows],
                            dtype=np.float64,
                        )  # shape: (n_windows, n_features)
                        feat_grp.create_dataset(
                            "sequence_values",
                            data=arr,
                            compression="gzip",
                            compression_opts=4,
                        )
                        feat_grp.attrs["sequence_feature_names"] = [
                            k.encode() for k in all_keys
                        ]

        # Move completed file to the final destination (overwriting if exists)
        _shutil.move(temp_path, h5_path)

    except Exception:
        if os.path.exists(temp_path):
            os.remove(temp_path)
        raise

    print(
        f"[SEGMENTER] Saved precomputed features for "
        f"{len(segment_features)} segments → {h5_path}"
    )


def load_precomputed_features(
    h5_path: str | Path,
) -> dict[str, dict]:
    """
    Read precomputed features from a segmented HDF5 file written by
    :func:`save_precomputed_features`.

    Returns
    -------
    dict mapping segment_key → feature_dict.
    Scalar features are individual float entries; sequence features are
    stored under the key ``"sequence_dicts"`` as a list of window-dicts.
    Returns an empty dict if no ``"features"`` groups are found.
    """
    h5_path = Path(h5_path)
    if not h5_path.exists():
        raise FileNotFoundError(f"Segment HDF5 file not found: {h5_path}")

    result: dict[str, dict] = {}

    with h5py.File(h5_path, "r") as f:
        seg_keys = [k for k in f.keys() if k.startswith("segment_")]
        for seg_key in seg_keys:
            grp = f[seg_key]
            if "features" not in grp:
                continue  # no precomputed features for this segment

            feat_grp = grp["features"]
            feat_dict: dict = {}

            # --- scalars -------------------------------------------------------
            if "scalar_values" in feat_grp:
                scalar_vals = feat_grp["scalar_values"][:]
                raw_names = feat_grp.attrs.get("scalar_names", [])
                scalar_names = [
                    n.decode() if isinstance(n, (bytes, np.bytes_)) else str(n)
                    for n in raw_names
                ]
                for name, val in zip(scalar_names, scalar_vals):
                    feat_dict[name] = float(val)

            # --- sequences -----------------------------------------------------
            if "sequence_values" in feat_grp:
                seq_arr = feat_grp["sequence_values"][:]  # (n_windows, n_features)
                raw_names = feat_grp.attrs.get("sequence_feature_names", [])
                seq_feature_names = [
                    n.decode() if isinstance(n, (bytes, np.bytes_)) else str(n)
                    for n in raw_names
                ]
                windows = [
                    dict(zip(seq_feature_names, row)) for row in seq_arr
                ]
                feat_dict["sequence_dicts"] = windows

            result[seg_key] = feat_dict

    return result


if __name__ == "__main__":

    # Add project root to path so imports work when run as script
    _root = Path(__file__).parent.parent
    if str(_root) not in sys.path:
        sys.path.insert(0, str(_root))

    parser = argparse.ArgumentParser(
        description="Segment Phase-2 IMU+EMG signals from HDF5 trial files."
    )
    parser.add_argument(
        "path",
        nargs="?",
        default="database",
        help="Path to a single .h5 trial file or a directory of trial files. "
             "Defaults to 'database'.",
    )
    parser.add_argument(
        "--margin",
        type=float,
        default=0.2,
        help="Time margin in seconds applied asymmetrically around the "
             "weight pickup transitioning point (default: 0.2).",
    )
    parser.add_argument(
        "--out",
        default="database/segments",
        help="Output directory for the segment files (default: database/segments).",
    )
    parser.add_argument(
        "--format",
        default="hdf5",
        choices=["hdf5", "h5", "npz", "both"],
        help="Output format: hdf5, npz, or both (default: hdf5).",
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip processing if the output segment file already exists.",
    )
    parser.add_argument(
        "--participants",
        type=str,
        nargs="+",
        help="List of participants to process",
    )
    _args = parser.parse_args()
    _run_cli(_args)
