"""
Sensor-practicality ablation — par-spec ST-transformer3.

Splits the 20 baseline channels (8 EMG + 12 IMU, no diff) into 5 *practicality*
groups by physical sensor placement and runs the full pipeline over every
non-empty combination (2^5 - 1 = 31 subsets). The goal is to find which sensor
placements actually carry the load signal, so a deployable system can drop the
cumbersome ones.

Sensor groups (Sensor 1 = upper arm, Sensor 2 = forearm/underarm):
    IMUup  IMU upper arm (sensor 1)  : ax1, ay1, az1, roll_rad1, pitch_rad1, yaw_rad1
    IMUfo  IMU forearm   (sensor 2)  : ax2, ay2, az2, roll_rad2, pitch_rad2, yaw_rad2
    EMGfo  EMG forearm               : Brachioradialis, FCU, ECR
    EMGup  EMG upper arm             : Biceps Brachii, Triceps Brachii
    EMGsh  EMG shoulder              : Anterior/Lateral/Posterior Deltoid

The 6 `*_diff` IMU channels stay disabled throughout, so the 5 groups exactly
partition the baseline 20-channel set used by cross-val2's `all` config.

Mutates only the in-memory cfg / run_model module attributes — leaves
model/config_model.py untouched (mirrors run_parspec_st3_nobalance.py).

Default (par-spec) run is launched by run_delftblue.sh:
    srun $ENV_PYTHON -u -m model.run_sensor_practicality_ablation

Phase-2 generalization check (run AFTER analysing phase-1 results), e.g.:
    python -m model.run_sensor_practicality_ablation --lopo \
        --combos IMUfo-EMGfo,emg_only,IMUup-IMUfo-EMGfo
"""
import sys
import argparse
import itertools
from pathlib import Path

# Ensure project root is importable when run as a script or `-m`.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import model.config_model as cfg

# ──────────────────────────────────────────────────────────────────────────
# 1. Sensor group → channel definitions
# ──────────────────────────────────────────────────────────────────────────
# Channel names MUST match the keys in cfg.CHANNEL_CONFIG exactly.
EMG_GROUPS = {
    "EMGfo": ["Brachioradialis", "Flexor Carpi Ulnaris (FCU)", "Extensor Carpi Radialis (ECR)"],
    "EMGup": ["Biceps Brachii", "Triceps Brachii"],
    "EMGsh": ["Anterior Deltoid", "Lateral Deltoid", "Posterior Deltoid"],
}
IMU_GROUPS = {
    "IMUup": ["ax1", "ay1", "az1", "roll_rad1", "pitch_rad1", "yaw_rad1"],
    "IMUfo": ["ax2", "ay2", "az2", "roll_rad2", "pitch_rad2", "yaw_rad2"],
}
# Fixed display/order of the 5 groups (used for combo naming).
GROUP_ORDER = ["IMUup", "IMUfo", "EMGfo", "EMGup", "EMGsh"]
ALL_EMG_CH = [ch for g in EMG_GROUPS.values() for ch in g]
ALL_IMU_CH = [ch for g in IMU_GROUPS.values() for ch in g]

# Canonical names matching the cross-val2 baseline (for direct comparison).
_CANONICAL = {
    frozenset(GROUP_ORDER): "all",
    frozenset(["EMGfo", "EMGup", "EMGsh"]): "emg_only",
    frozenset(["IMUup", "IMUfo"]): "imu_only",
}


def combo_name(groups):
    """Stable, filesystem-safe name for a set of groups."""
    key = frozenset(groups)
    if key in _CANONICAL:
        return _CANONICAL[key]
    return "-".join(g for g in GROUP_ORDER if g in key)


def build_plan(groups):
    """Build a run_model CUSTOM_MODALITY_PLAN entry for a set of groups."""
    active_emg = [ch for g in EMG_GROUPS if g in groups for ch in EMG_GROUPS[g]]
    active_imu = [ch for g in IMU_GROUPS if g in groups for ch in IMU_GROUPS[g]]
    emg_map = {ch: (ch in active_emg) for ch in ALL_EMG_CH}
    imu_map = {ch: (ch in active_imu) for ch in ALL_IMU_CH}
    return {"name": combo_name(groups), "emg": emg_map, "imu": imu_map}


def all_combos():
    """All 31 non-empty subsets of the 5 groups, ordered small→large."""
    combos = []
    for r in range(1, len(GROUP_ORDER) + 1):
        for combo in itertools.combinations(GROUP_ORDER, r):
            combos.append(frozenset(combo))
    return combos


# ──────────────────────────────────────────────────────────────────────────
# 2. CLI
# ──────────────────────────────────────────────────────────────────────────
def parse_args():
    p = argparse.ArgumentParser(description="Sensor-practicality ablation grid.")
    p.add_argument("--lopo", action="store_true",
                   help="Phase-2: cross-subject LOPO instead of pooled k-fold.")
    p.add_argument("--combos", type=str, default=None,
                   help="Comma-separated combo names to restrict to (default: all 31). "
                        "Use canonical/joined names, e.g. 'IMUfo-EMGfo,emg_only'.")
    p.add_argument("--skip_baseline_combos", action="store_true",
                   help="Skip all/emg_only/imu_only (already run in cross-val2).")
    p.add_argument("--limit_folds", type=int, default=None,
                   help="Override CV_CONFIG.limit_folds (default 3 par-spec / None LOPO).")
    p.add_argument("--dev", action="store_true",
                   help="DEV_MODE smoke test (sub-sampled data, fewer epochs).")
    return p.parse_known_args()[0]


def main():
    args = parse_args()

    # ── Build the modality plan ──────────────────────────────────────────
    combos = all_combos()
    if args.skip_baseline_combos:
        combos = [c for c in combos if combo_name(c) not in ("all", "emg_only", "imu_only")]
    plan = [build_plan(c) for c in combos]
    if args.combos:
        wanted = {s.strip() for s in args.combos.split(",") if s.strip()}
        plan = [e for e in plan if e["name"] in wanted]
        missing = wanted - {e["name"] for e in plan}
        if missing:
            raise SystemExit(f"Unknown combo name(s): {sorted(missing)}\n"
                             f"Valid names: {[combo_name(c) for c in all_combos()]}")

    # ── cfg: par-spec (pooled k-fold) vs LOPO, fast-grid toggles ─────────
    cfg.CV_CONFIG['use_cross_val'] = True
    if args.lopo:
        cfg.CV_CONFIG['strategy'] = 'participant'   # leave-one-participant-out
        default_limit = None                        # full LOPO unless overridden
    else:
        cfg.CV_CONFIG['strategy'] = 'kfold'         # pooled within-subject calibration
        default_limit = 3                           # par-spec: 3 folds (24h budget)
    cfg.CV_CONFIG['limit_folds'] = args.limit_folds if args.limit_folds is not None else default_limit
    # Reuse the last-evaluated fold model; skip the extra full-dataset retrain per combo.
    cfg.CV_CONFIG['skip_final_full_train'] = True

    # Import run_model AFTER cfg dict mutations (dicts are shared by reference).
    import model.run_model as rm

    # Scalar gates are bound into run_model's namespace via `from ... import`, so they
    # must be set on the rm module, not on cfg, to take effect.
    rm.COMPUTE_FEATURE_IMPORTANCE = False     # master switch: disables perm + DeepSHAP
    rm.COMPUTE_DEEPSHAP = False
    rm.COMPUTE_PERMUTATION_CHANNEL = False
    rm.COMPUTE_PERMUTATION_FEATURE = False
    rm.COMPUTE_PERMUTATION_INDIVIDUAL = False
    if args.dev:
        rm.DEV_MODE = True

    # Hand the plan to run_model and launch.
    rm.CUSTOM_MODALITY_PLAN = plan

    print("=" * 78)
    print("SENSOR-PRACTICALITY ABLATION — par-spec ST-transformer3")
    print(f"  mode (CV strategy)   : {cfg.CV_CONFIG['strategy']}"
          f"{'  [LOPO / cross-subject]' if args.lopo else '  [pooled k-fold / calibration]'}")
    print(f"  limit_folds          : {cfg.CV_CONFIG['limit_folds']}")
    print(f"  skip_final_full_train: {cfg.CV_CONFIG['skip_final_full_train']}")
    print(f"  feature importance   : OFF (reuse cross-val2 baseline SHAP/perm)")
    print(f"  DEV_MODE             : {getattr(rm, 'DEV_MODE', False)}")
    print(f"  # combinations       : {len(plan)}")
    print(f"  combinations         : {[e['name'] for e in plan]}")
    print("=" * 78)

    rm.main()


if __name__ == "__main__":
    main()
