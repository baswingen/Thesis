import torch
import numpy as np
from pathlib import Path
from typing import Dict, List, Any, Optional

from model.config_model import (
    PARTICIPANT_CONFIG, CHANNEL_CONFIG, FEATURE_CONFIG,
    WEIGHT_INCLUDE, TRUE_WEIGHTS, DEV_MODE, DEV_FRACTION, DEV_EPOCHS,
    GLOBAL_LOSS_FUNCTION, GLOBAL_RANDOM_STATE, AUGMENTATION_CONFIG,
)
from model import performance_utils

class ReportGenerator:
    """
    Handles the generation of detailed performance reports for model training and evaluation runs.
    Designed to be modular so that new sections (like permutation importance or attention weights)
    can be added cleanly over time.
    """
    def __init__(self, run_dir: Path, timestamp: str):
        self.run_dir = Path(run_dir)
        self.timestamp = timestamp
        self.report_file = self.run_dir / "performance_report.txt"
        self.sections = []

    def _add_header(self):
        self.sections.append("=" * 55)
        self.sections.append(f"MODEL PERFORMANCE REPORT\nRun Timestamp: {self.timestamp}")
        self.sections.append("=" * 55 + "\n")

    def _add_dataset_info(self, h5_paths, X, X_test, use_cv, cv_strategy, n_folds, model):
        lines = ["--- DATASET INFO ---"]
        lines.append(f"Participants included: {PARTICIPANT_CONFIG.get('include', 'all')}")
        lines.append(f"Database segments used: {[p.name for p in h5_paths]}")
        lines.append(f"Total samples: {len(X)}")
        
        if use_cv:
            mode = f"Leave-One-Participant-Out CV ({n_folds} Participants)" if cv_strategy == 'participant' else f"{n_folds}-Fold Stratified CV"
            lines.append(f"Evaluation Mode: {mode}")
            lines.append(f"Final Model Training samples: {getattr(model, 'train_samples', len(X))}")
        else:
            lines.append("Evaluation Mode: Train/Test Split")
            lines.append(f"Testing samples: {len(X_test)}")
        
        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_balance_weights(self, y_train, use_cv, y):
        # We need to import calculate_class_weights locally to avoid circular dependency
        from model.run_model import calculate_class_weights
        lines = ["--- BALANCE WEIGHTS ---"]
        lines.append("Sample weighting: enabled (class_weight='balanced')")
        weights = calculate_class_weights(y if use_cv else y_train)
        lines.append(f"{'Weight':<12} | {'Count (Train)':<14} | {'Multiplier':<10}")
        lines.append("-" * 43)
        for cw in weights:
            lines.append(f"{cw['Weight']:<12} | {cw['Count']:<14} | {cw['Multiplier']:<10}")
        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_feature_configuration(self, is_raw_segment, ablation_modality):
        lines = ["--- FEATURE CONFIGURATION ---"]
        lines.append(f"Ablation Modality: {ablation_modality.upper()}")
        if is_raw_segment:
            lines.append("Input: Raw EMG + IMU segments (end-to-end CNN)")
        else:
            lines.append(f"EMG Window: {FEATURE_CONFIG['emg_window_size_sec']}s, IMU Window: {FEATURE_CONFIG['imu_window_size_sec']}s, Step: {FEATURE_CONFIG['window_step_sec']}s")

        # Always show enabled channels
        emg_ch = [k for k, v in CHANNEL_CONFIG.get('emg_channels', {}).items() if v]
        imu_ch = [k for k, v in CHANNEL_CONFIG.get('imu_channels', {}).items() if v]
        disabled_emg = [k for k, v in CHANNEL_CONFIG.get('emg_channels', {}).items() if not v]
        disabled_imu = [k for k, v in CHANNEL_CONFIG.get('imu_channels', {}).items() if not v]
        lines.append(f"Enabled EMG Channels ({len(emg_ch)}): {', '.join(emg_ch)}")
        lines.append(f"Enabled IMU Channels ({len(imu_ch)}): {', '.join(imu_ch)}")
        if disabled_emg:
            lines.append(f"Disabled EMG Channels ({len(disabled_emg)}): {', '.join(disabled_emg)}")
        if disabled_imu:
            lines.append(f"Disabled IMU Channels ({len(disabled_imu)}): {', '.join(disabled_imu)}")

        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_hyperparameters(self, model, model_type):
        lines = ["--- HYPERPARAMETERS ---"]
        lines.append(f"Model: {model_type.upper()}")

        # ── Architecture attributes (varies by model) ──
        arch_keys = [
            # Transformer / Spatio-Temporal Transformer
            'd_model', 'nhead', 'nhead_spatial', 'nhead_temporal',
            'num_layers', 'num_layers_spatial', 'num_layers_temporal',
            'dim_feedforward',
            # CNN / RNN
            'cnn_filters', 'cnn_kernel_sizes', 'pool_size',
            'lstm_hidden_size', 'lstm_num_layers',
            'gru_hidden_size', 'gru_num_layers',
            'hidden_size', 'num_layers',
            # TCN
            'tcn_channels', 'kernel_size',
            # General
            'use_anthropometrics', 'use_checkpointing', 'use_amp',
        ]
        arch_lines = []
        for key in arch_keys:
            val = getattr(model, key, None)
            if val is not None:
                arch_lines.append(f"  {key}: {val}")
        if arch_lines:
            lines.append("Architecture:")
            lines.extend(arch_lines)

        # ── Training / Optimisation ──
        train_keys = [
            ('learning_rate', 'Learning Rate'),
            ('weight_decay', 'Weight Decay'),
            ('batch_size', 'Batch Size'),
            ('epochs', 'Epochs'),
            ('dropout_rate', 'Dropout'),
            ('loss_type', 'Loss Function'),
            ('validation_split', 'Validation Split'),
            ('early_stopping_patience', 'Early Stopping Patience'),
            ('scheduler_patience', 'Scheduler Patience'),
            ('scheduler_factor', 'Scheduler Factor'),
        ]
        train_lines = []
        for key, label in train_keys:
            val = getattr(model, key, None)
            if val is not None:
                train_lines.append(f"  {label}: {val}")
        if train_lines:
            lines.append("Training:")
            lines.extend(train_lines)

        # ── Scheduler config (dict) ──
        sched = getattr(model, 'scheduler_config', None)
        if sched is None:
            # Try the config dict import
            from model.config_model import SPATIO_TEMPORAL_TRANSFORMER_CONFIG
            if model_type == 'spatio_temporal_transformer':
                sched = SPATIO_TEMPORAL_TRANSFORMER_CONFIG.get('scheduler', {})
        if sched:
            lines.append(f"Scheduler: {sched}")

        # ── SVR / RF specifics ──
        if model_type == 'svr':
            lines.append(f"Kernel: {getattr(model, 'kernel', 'N/A')}")
            lines.append(f"C: {getattr(model, 'C', 'N/A')}")
            lines.append(f"Epsilon: {getattr(model, 'epsilon', 'N/A')}")
        elif model_type == 'rf':
            lines.append(f"N Estimators: {getattr(model, 'n_estimators', 'N/A')}")
            lines.append(f"Max Depth: {getattr(model, 'max_depth', 'N/A')}")

        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_evaluation_metrics(self, use_cv, avg_metrics, std_metrics, metrics, n_folds, 
                                y_true_pooled, y_pred_pooled):
        from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score
        
        lines = ["--- EVALUATION METRICS ---"]
        
        if use_cv:
            lines.append("METHOD: MACRO-AVERAGE (Primary)")
            lines.append(f"Description: Unweighted mean across {n_folds} participants. Weights every person equally.")
            lines.append("(Reflects expected generalization performance to a NEW participant)")
            for k, v in avg_metrics.items():
                if 'time' not in k.lower():
                    lines.append(f"  {k}: {v:.4f} (±{std_metrics[k]:.4f})")
            
            # Add Pooled metrics for comparison (matches Regression Plot)
            if y_true_pooled is not None and y_pred_pooled is not None:
                # Apply consistent clamping
                y_pred_clamped = np.maximum(0.0, y_pred_pooled)
                p_mae = mean_absolute_error(y_true_pooled, y_pred_clamped)
                p_mse = mean_squared_error(y_true_pooled, y_pred_clamped)
                p_rmse = np.sqrt(p_mse)
                p_r2 = r2_score(y_true_pooled, y_pred_clamped)
                
                lines.append("\nMETHOD: POOLED-AVERAGE (Secondary)")
                lines.append("Description: Weighted by samples. Matches Regression Plot box-plot metrics.")
                lines.append("(Reflects overall accuracy across all recorded data points)")
                lines.append(f"  MAE: {p_mae:.4f}")
                lines.append(f"  RMSE: {p_rmse:.4f}")
                lines.append(f"  R2: {p_r2:.4f}")
        else:
            lines.append("METHOD: SINGLE TEST SPLIT")
            # Apply clamping for consistency
            y_pred_clamped = np.maximum(0.0, y_pred_pooled)
            p_mae = mean_absolute_error(y_true_pooled, y_pred_clamped)
            p_mse = mean_squared_error(y_true_pooled, y_pred_clamped)
            p_rmse = np.sqrt(p_mse)
            p_r2 = r2_score(y_true_pooled, y_pred_clamped)
            
            lines.append(f"  MAE: {p_mae:.4f}")
            lines.append(f"  RMSE: {p_rmse:.4f}")
            lines.append(f"  R2: {p_r2:.4f}")

        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_per_weight_metrics(self, y_test, y_pred, use_cv, y, oof_predictions):
        from model.run_model import calculate_per_weight_metrics
        target_y = y if use_cv else y_test
        target_pred = oof_predictions if use_cv else y_pred
        stats = calculate_per_weight_metrics(target_y, target_pred)
        
        lines = ["--- PER-WEIGHT METRICS ---"]
        lines.append(f"{'Weight':<12} | {'Count':<8} | {'MAE':<10} | {'RMSE':<10}")
        lines.append("-" * 50)
        for s in stats:
            lines.append(f"{s['Weight']:<12} | {s['Count']:<8} | {s['MAE']:<10} | {s['RMSE']:<10}")
        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_per_participant_metrics(self, participant_stats):
        if not participant_stats:
            return
            
        gen_stats = performance_utils.get_generalization_metrics(participant_stats)
        if gen_stats:
            lines = ["--- DETAILED GENERALIZATION PERFORMANCE ---"]
            lines.append(f"Mean MAE across participants: {gen_stats['mean_mae']:.4f} ± {gen_stats['std_mae']:.4f}")
            lines.append(f"Min / Max MAE:                {gen_stats['min_mae']:.4f} / {gen_stats['max_mae']:.4f}")
            lines.append(f"Mean RMSE across participants:{gen_stats['mean_rmse']:.4f} ± {gen_stats['std_rmse']:.4f}")
            lines.append(f"Min / Max RMSE:               {gen_stats['min_rmse']:.4f} / {gen_stats['max_rmse']:.4f}")
            lines.append("")
            self.sections.append("\n".join(lines))
            
        lines = ["--- PER-PARTICIPANT METRICS ---"]
        lines.append(f"{'Participant':<12} | {'Samples':<8} | {'MAE':<10} | {'RMSE':<10}")
        lines.append("-" * 50)
        for p in participant_stats:
            lines.append(f"{p['Participant']:<12} | {p['Samples']:<8} | {p['MAE']:<10.4f} | {p['RMSE']:<10.4f}")
        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_per_seqlen_metrics(self, per_seqlen_stats):
        if not per_seqlen_stats:
            return
        lines = ["--- PER-SEQUENCE-LENGTH METRICS ---"]
        for s in per_seqlen_stats:
            lines.append(f"{s['SeqLen']:<12} | {s['TimeAtPrediction']:<16} | {s['Count']:<8} | {s['MAE']:<10} | {s['RMSE']:<10}")
        lines.append("")
        self.sections.append("\n".join(lines))
        
    def _add_per_duration_metrics(self, per_duration_stats):
        if not per_duration_stats:
            return
        lines = ["--- PER-SEGMENT-DURATION METRICS (CNN-LSTM) ---"]
        lines.append("(Prediction error vs. elapsed time into lift, binned from raw segment duration)")
        lines.append(f"{'Bin':<6} | {'Time into lift (mid)':<22} | {'Count':<8} | {'MAE':<10} | {'RMSE':<10}")
        lines.append("-" * 68)
        for s in per_duration_stats:
            lines.append(f"{s['SeqLen']:<6} | {s['TimeAtPrediction']:<22} | {s['Count']:<8} | {s['MAE']:<10} | {s['RMSE']:<10}")
        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_compute_metrics(self, model, use_cv, avg_metrics, std_metrics, metrics):
        lines = ["--- COMPUTE & TIMING METRICS ---"]
        lines.append(f"Device: {performance_utils.format_device_string(performance_utils.get_device_info())}")
        if hasattr(model, 'model') and isinstance(model.model, torch.nn.Module):
            tot, train = performance_utils.count_parameters(model.model)
            lines.append(f"Total Parameters: {tot:,}\nTrainable Parameters: {train:,}")
            
        target_metrics = avg_metrics if use_cv else metrics
        target_stds = std_metrics if use_cv else {}
        
        has_timing = any('time' in k.lower() for k in target_metrics.keys())
        if has_timing:
            if use_cv:
                lines.append(f"\nAverage timings:")
                for k, v in target_metrics.items():
                    if 'time' in k.lower():
                        lines.append(f"{k}: {v:.6f}s (±{target_stds.get(k, 0):.6f}s)")
            else:
                lines.append("\nTimings:")
                for k, v in target_metrics.items():
                    if 'time' in k.lower():
                        lines.append(f"{k}: {v:.6f}s")
                        
        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_permutation_importance(self, permutation_importances):
        if not permutation_importances:
            return
        lines = ["--- PERMUTATION CHANNEL IMPORTANCE ---"]
        lines.append("(Drop in MSE when feature is shuffled. Higher is more important)")
        sorted_imp = sorted(permutation_importances.items(), key=lambda item: item[1], reverse=True)
        for k, v in sorted_imp:
            lines.append(f"{k}: {v:.6f}")
        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_deepshap_summary(self):
        shap_p = self.run_dir / "deepshap_values.npz"
        if not shap_p.exists():
            return
            
        import numpy as np
        try:
            data = np.load(shap_p, allow_pickle=True)
            grouped_names = data['grouped_names']
            grouped_vals = data['grouped_vals']
            
            lines = ["--- DEEPSHAP CHANNEL IMPORTANCE ---"]
            lines.append("(Mean absolute SHAP value per grouped channel)")
            
            # Sort by importance
            order = np.argsort(grouped_vals)[::-1]
            for idx in order:
                lines.append(f"{str(grouped_names[idx]):<15}: {grouped_vals[idx]:.6f}")
            
            lines.append("")
            self.sections.append("\n".join(lines))
        except Exception as e:
            print(f"[WARN] Failed to add DeepSHAP to report: {e}")

    def generate(self, h5_paths, X, use_cv, cv_strategy, n_folds, model, model_type,
                 X_test, X_train, y, y_train, y_test, is_raw_segment, 
                 avg_metrics, std_metrics, metrics, participant_stats, 
                 per_seqlen_stats, per_duration_stats, oof_predictions, y_pred,
                 permutation_importances=None, ablation_modality="all"):
        """Assembles all sections and writes to the report file."""
        
        self._add_header()
        self._add_dataset_info(h5_paths, X, X_test, use_cv, cv_strategy, n_folds, model)
        
        if getattr(model, 'balance_weights', False):
            self._add_balance_weights(y_train, use_cv, y)
            
        self._add_feature_configuration(is_raw_segment, ablation_modality)
        self._add_hyperparameters(model, model_type)
        self._add_pipeline_config()
        
        # Pass pooled data for secondary metric calculation
        y_true_p = y if use_cv else y_test
        y_pred_p = oof_predictions if use_cv else y_pred
        self._add_evaluation_metrics(use_cv, avg_metrics, std_metrics, metrics, n_folds, y_true_p, y_pred_p)
        self._add_per_weight_metrics(y_test, y_pred, use_cv, y, oof_predictions)
        self._add_per_participant_metrics(participant_stats)
        self._add_per_seqlen_metrics(per_seqlen_stats)
        self._add_per_duration_metrics(per_duration_stats)
        self._add_compute_metrics(model, use_cv, avg_metrics, std_metrics, metrics)
        self._add_permutation_importance(permutation_importances)
        self._add_deepshap_summary()

        with open(self.report_file, "w") as f:
            f.write("\n".join(self.sections))

        print(f"\nPerformance report saved to {self.report_file}")

    def _add_pipeline_config(self):
        """Print global pipeline settings so the report is fully reproducible."""
        lines = ["--- PIPELINE CONFIGURATION ---"]
        lines.append(f"DEV_MODE: {DEV_MODE}" + (f" (fraction={DEV_FRACTION}, epochs={DEV_EPOCHS})" if DEV_MODE else ""))
        lines.append(f"Global Loss Function: {GLOBAL_LOSS_FUNCTION}")
        lines.append(f"Random State: {GLOBAL_RANDOM_STATE}")

        # Weight-class filter
        excluded = [k for k, v in WEIGHT_INCLUDE.items() if not v]
        if excluded:
            mapped = [f"{k}kg (→ {TRUE_WEIGHTS.get(k, '?')}kg)" for k in excluded]
            lines.append(f"Excluded Weight Classes: {', '.join(mapped)}")
        else:
            lines.append("Excluded Weight Classes: none")

        # Augmentation summary
        aug_enabled = AUGMENTATION_CONFIG.get('enabled', False)
        lines.append(f"Augmentation: {'enabled' if aug_enabled else 'disabled'}")
        if aug_enabled:
            lines.append(f"  probability: {AUGMENTATION_CONFIG.get('p', 'N/A')}")
            lines.append(f"  methods: {AUGMENTATION_CONFIG.get('methods', [])}")
            lines.append(f"  noise_std: {AUGMENTATION_CONFIG.get('noise_std', 'N/A')}")
            lines.append(f"  stretch_factor_range: {AUGMENTATION_CONFIG.get('stretch_factor_range', 'N/A')}")
            lines.append(f"  channel_dropout_p: {AUGMENTATION_CONFIG.get('channel_dropout_p', 'N/A')}")
            lines.append(f"  magnitude_scale_range: {AUGMENTATION_CONFIG.get('magnitude_scale_range', 'N/A')}")

        lines.append("")
        self.sections.append("\n".join(lines))
