"""
Architecture comparison — baselines vs the ST-transformer3 final models.

Runs one baseline architecture (LSTM / GRU / CNN-LSTM / CNN-GRU / naive
Transformer) through the SAME pipeline, data, CV split, balancing and
evaluation as the st3 final runs, so the participant-class macro metrics
(model/macro_metrics.py, exported in run_data.json) are directly comparable
to final_run_lopo (generalized) and the par-spec model (specialized).

Regime is selected by THESIS_CV_STRATEGY (read by config_model at import):
    participant -> LOPO / cross-subject   (compare vs final_run_lopo)
    kfold       -> pooled within-subject   (compare vs cross-val2 / par-spec)

Input parity:
    - lstm / gru / transformer : feature sequences — identical input to st3,
      and they get the SAME participant×weight augmentation balancing
      (AUGMENTATION_CONFIG.balance_participants + balance_weights).
    - cnn_lstm / cnn_gru        : raw windowed segments (inherent to the arch);
      they use their own CNN_*_CONFIG (note: balancing is off there by default).

By default only the fused "all" model is trained (the architecture comparison
of the full fusion model) to stay within the 24 h budget; pass --all_modalities
to also run emg_only / imu_only. Feature importance (DeepSHAP/permutation) is
always OFF here — it is irrelevant to an architecture comparison and is the main
runtime cost.

Mutates only the in-memory cfg / run_model module attributes — leaves
model/config_model.py untouched (mirrors run_sensor_practicality_ablation.py).

Launched per array task by run_delftblue.sh, e.g.:
    THESIS_CV_STRATEGY=participant python -m model.run_arch_comparison --model_type lstm
"""
import sys
import argparse
from pathlib import Path

# Ensure project root is importable when run as a script or `-m`.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import model.config_model as cfg

# The 5 baseline architectures this comparison covers.
VALID_MODELS = ["lstm", "gru", "cnn_lstm", "cnn_gru", "transformer"]


def parse_args():
    p = argparse.ArgumentParser(description="Architecture comparison vs st3.")
    p.add_argument("--model_type", required=True, choices=VALID_MODELS,
                   help="Baseline architecture to train.")
    p.add_argument("--modality", default="all", choices=["all", "emg_only", "imu_only"],
                   help="Single modality to train when not using --all_modalities (default: all).")
    p.add_argument("--all_modalities", action="store_true",
                   help="Run the full modality loop (all/emg_only/imu_only) instead of just --modality.")
    return p.parse_known_args()[0]


def main():
    args = parse_args()

    # MODEL_TYPE is bound into run_model via `from config_model import MODEL_TYPE`,
    # so set it on cfg BEFORE importing run_model (dicts/scalars are read at import).
    cfg.MODEL_TYPE = args.model_type

    # ── Consistent per-regime LR schedule across all comparison models ───────
    # Match st3's recipe so the scheduler is not a confound:
    #   kfold (par-spec)      -> OneCycleLR, full 200-epoch budget (early stop off)
    #   participant (LOPO)    -> ReduceLROnPlateau (patience 3, factor 0.8), early stop 5
    # The model config dicts are shared by reference and **-expanded by the factory,
    # so mutating them here injects the schedule (each trainer reads it via build_scheduler).
    _CONFIG_BY_MODEL = {
        "lstm": cfg.LSTM_CONFIG, "gru": cfg.GRU_CONFIG,
        "cnn_lstm": cfg.CNN_LSTM_CONFIG, "cnn_gru": cfg.CNN_GRU_CONFIG,
        "transformer": cfg.TRANSFORMER_CONFIG,
    }
    model_cfg = _CONFIG_BY_MODEL[args.model_type]
    model_cfg["epochs"] = 200
    if cfg.CV_STRATEGY == "kfold":
        model_cfg["scheduler"] = {"type": "OneCycleLR", "pct_start": 0.4}
        model_cfg["early_stopping_patience"] = 200   # disabled — OneCycle runs the full budget
    else:  # participant / LOPO
        model_cfg["scheduler"] = {"type": "ReduceLROnPlateau"}
        model_cfg["scheduler_patience"] = 3
        model_cfg["scheduler_factor"] = 0.8
        model_cfg["early_stopping_patience"] = 5

    # Import run_model AFTER cfg mutation.
    import model.run_model as rm

    # Scalars are bound into run_model's namespace by `from ... import`, so set them
    # on the rm module to take effect.
    rm.MODEL_TYPE = args.model_type

    # Feature importance OFF — not needed for an architecture comparison and the
    # single biggest runtime cost (keeps each job inside the 24 h wall).
    rm.COMPUTE_FEATURE_IMPORTANCE = False
    rm.COMPUTE_DEEPSHAP = False
    rm.COMPUTE_PERMUTATION_CHANNEL = False
    rm.COMPUTE_PERMUTATION_FEATURE = False
    rm.COMPUTE_PERMUTATION_INDIVIDUAL = False

    # Modality scope: fused "all" by default, or the full loop with --all_modalities.
    if args.all_modalities:
        rm.RUN_MODALITY_ABLATION = True
        sys.argv = [sys.argv[0]]
    else:
        rm.RUN_MODALITY_ABLATION = False
        sys.argv = [sys.argv[0], "--modality", args.modality]

    print("=" * 78)
    print("ARCHITECTURE COMPARISON — baseline vs st3")
    print(f"  model_type          : {args.model_type}")
    print(f"  CV strategy (regime): {cfg.CV_STRATEGY}"
          f"{'  [LOPO / cross-subject]' if cfg.CV_STRATEGY == 'participant' else '  [pooled k-fold / par-spec]'}")
    print(f"  modalities          : {'all+emg_only+imu_only' if args.all_modalities else args.modality}")
    print(f"  scheduler           : {model_cfg['scheduler']['type']}  epochs={model_cfg['epochs']}  es_patience={model_cfg['early_stopping_patience']}")
    print(f"  feature importance  : OFF")
    print("=" * 78)

    rm.main()


if __name__ == "__main__":
    main()
