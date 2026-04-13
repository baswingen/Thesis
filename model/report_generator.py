import torch
import numpy as np
from pathlib import Path
from typing import Dict, List, Any, Optional

from model.config_model import PARTICIPANT_CONFIG, CHANNEL_CONFIG, FEATURE_CONFIG
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
            emg_ch = [k for k, v in CHANNEL_CONFIG.get('emg_channels', {}).items() if v]
            imu_ch = [k for k, v in CHANNEL_CONFIG.get('imu_channels', {}).items() if v]
            lines.append(f"Enabled EMG Channels ({len(emg_ch)}): {', '.join(emg_ch)}")
            lines.append(f"Enabled IMU Channels ({len(imu_ch)}): {', '.join(imu_ch)}")
        else:
            lines.append(f"EMG Window: {FEATURE_CONFIG['emg_window_size_sec']}s, IMU Window: {FEATURE_CONFIG['imu_window_size_sec']}s, Step: {FEATURE_CONFIG['window_step_sec']}s")
        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_hyperparameters(self, model, model_type):
        lines = ["--- HYPERPARAMETERS ---"]
        lines.append(f"Model: {model_type.upper()}")
        
        if model_type == "svr":
            lines.append(f"Kernel: {model.kernel}\nC: {model.C}\nEpsilon: {model.epsilon}")
        elif model_type == "rf":
            lines.append(f"N Estimators: {model.n_estimators}\nMax Depth: {model.max_depth}")
        elif model_type in ["gru", "lstm", "cnn_lstm", "tcn", "transformer"]:
            lines.append(f"Learning Rate: {getattr(model, 'learning_rate', 'N/A')}")
            lines.append(f"Batch Size: {getattr(model, 'batch_size', 'N/A')}")
            lines.append(f"Epochs: {getattr(model, 'epochs', 'N/A')}")
            
        lines.append("")
        self.sections.append("\n".join(lines))

    def _add_evaluation_metrics(self, use_cv, avg_metrics, std_metrics, metrics, n_folds):
        lines = ["--- EVALUATION METRICS ---"]
        if use_cv:
            lines.append(f"(Averaged over {n_folds} folds)")
            for k, v in avg_metrics.items():
                if 'time' not in k.lower():
                    lines.append(f"{k}: {v:.4f} (±{std_metrics[k]:.4f})")
        else:
            for k, v in metrics.items():
                if 'time' not in k.lower():
                    lines.append(f"{k}: {v:.4f}")
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
        self._add_evaluation_metrics(use_cv, avg_metrics, std_metrics, metrics, n_folds)
        self._add_per_weight_metrics(y_test, y_pred, use_cv, y, oof_predictions)
        self._add_per_participant_metrics(participant_stats)
        self._add_per_seqlen_metrics(per_seqlen_stats)
        self._add_per_duration_metrics(per_duration_stats)
        self._add_compute_metrics(model, use_cv, avg_metrics, std_metrics, metrics)
        self._add_permutation_importance(permutation_importances)

        with open(self.report_file, "w") as f:
            f.write("\n".join(self.sections))

        print(f"\nPerformance report saved to {self.report_file}")
