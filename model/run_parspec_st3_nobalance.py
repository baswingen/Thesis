"""
Par-spec-P01 st3 ablation — WEIGHT BALANCING OFF (comparison arm).

Identical to the standard `python model/run_model.py` par-spec-P01 st3 run
(single participant P01, kfold limit_folds=1, full modality ablation,
DeepSHAP + permutation importance) EXCEPT weight balancing is disabled.

With balance_weights=False and ST models passing participant_ids=None, the
augmenter falls through to Standard Mode (each sample augmented w/ prob p,
dataset grows ~2x) — reproducing the ORIGINAL 2026-05-07 par-spec-P01
augmentation regime (no balancing), but on the st3 architecture so the
balancing variable is isolated against the running balancing-ON arm.

Leaves model/config_model.py untouched (mutates the in-memory dict only).
Run AFTER the balancing-ON job finishes (don't run two MPS jobs in parallel).
"""
import sys
from pathlib import Path
# Ensure project root is importable when run as `python model/run_parspec_st3_nobalance.py`
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import model.config_model as cfg

# Disable weight balancing -> Standard augmentation mode for ST models.
cfg.AUGMENTATION_CONFIG['balance_weights'] = False
# (balance_participants is already a no-op for sequence models: participant_ids=None)

# Skip the redundant "final model on full dataset" retrain — we only want the
# fold evaluation results. The last evaluated fold model is reused for SHAP/saving.
cfg.CV_CONFIG['skip_final_full_train'] = True

# Scheduler: OneCycleLR (matches the original st1 par-spec-P01 reference) instead of
# WarmupCosine. OneCycleLR anneals over the full epoch budget, so bump early-stopping
# patience to 50 (reference value) to avoid stopping mid-cycle before LR comes down.
cfg.SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['scheduler'] = {
    'type': 'OneCycleLR', 'max_lr': 0.0003, 'pct_start': 0.3,
}
cfg.SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['early_stopping_patience'] = 50

print("=" * 70)
print("PAR-SPEC-P01 st3 ABLATION — WEIGHT BALANCING OFF (comparison arm)")
print(f"  balance_weights      : {cfg.AUGMENTATION_CONFIG['balance_weights']}")
print(f"  balance_participants : {cfg.AUGMENTATION_CONFIG['balance_participants']} (no-op for ST: participant_ids=None)")
print(f"  participants         : {cfg.PARTICIPANT_CONFIG['include']}")
print(f"  model / strategy     : {cfg.MODEL_TYPE} / {cfg.CV_CONFIG['strategy']} (limit_folds={cfg.CV_CONFIG['limit_folds']})")
print(f"  skip_final_full_train: {cfg.CV_CONFIG['skip_final_full_train']}")
print(f"  scheduler            : {cfg.SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['scheduler']}")
print(f"  early_stop_patience  : {cfg.SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['early_stopping_patience']}")
print(f"  modality ablation    : {cfg.RUN_MODALITY_ABLATION}")
print("=" * 70)

import model.run_model as rm

if __name__ == "__main__":
    rm.main()
