"""
Par-spec-P01 st3 retrain with CLASS-BALANCED LOSS (bias-prevention test).

Identical to run_parspec_st3_nobalance.py (single participant P01, kfold
limit_folds=1, standard aug / NO oversampling balancing, OneCycleLR, skip
final-full-train, full modality ablation, SHAP + permutation) EXCEPT it enables
inverse-frequency per-sample loss weighting (class_balanced_loss=True) to prevent
the 0kg-dominated affine gain bias at training time.

This isolates the class-balanced-loss effect vs the current canonical standard
(ST-transformer-par-spec-P01-st3), which was trained without it.

Leaves model/config_model.py untouched (mutates the in-memory dicts only).
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import model.config_model as cfg

# Match the no-balance standard's regime ...
cfg.AUGMENTATION_CONFIG['balance_weights'] = False
cfg.CV_CONFIG['skip_final_full_train'] = True
cfg.SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['scheduler'] = {
    'type': 'OneCycleLR', 'max_lr': 0.0003, 'pct_start': 0.3,
}
cfg.SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['early_stopping_patience'] = 50
# ... and the ONLY change under test:
cfg.SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['class_balanced_loss'] = True

print("=" * 70)
print("PAR-SPEC-P01 st3 — CLASS-BALANCED LOSS (bias-prevention retrain)")
print(f"  class_balanced_loss  : {cfg.SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['class_balanced_loss']}")
print(f"  balance_weights (aug): {cfg.AUGMENTATION_CONFIG['balance_weights']} (oversampling OFF; loss-weighting only)")
print(f"  participants         : {cfg.PARTICIPANT_CONFIG['include']}")
print(f"  model / strategy     : {cfg.MODEL_TYPE} / {cfg.CV_CONFIG['strategy']} (limit_folds={cfg.CV_CONFIG['limit_folds']})")
print(f"  scheduler            : {cfg.SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['scheduler']}")
print(f"  early_stop_patience  : {cfg.SPATIO_TEMPORAL_TRANSFORMER3_CONFIG['early_stopping_patience']}")
print(f"  skip_final_full_train: {cfg.CV_CONFIG['skip_final_full_train']}")
print(f"  modality ablation    : {cfg.RUN_MODALITY_ABLATION}")
print("=" * 70)

import model.run_model as rm

if __name__ == "__main__":
    rm.main()
