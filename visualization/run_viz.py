#!/usr/bin/env python3
"""
run_viz.py
==========
Generates paper-ready, premium visualizations for a Master's Thesis
styled according to the LaTeX document class `rho.cls`.

Supports modular registration of new plots and automatic output sizing 
(column, double, default) in high-quality .png format.
"""

import json
import argparse
import sys
import os
import itertools
import math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# ──────────────────────────────────────────────────────────
# DYNAMIC CUSTOM FONT REGISTRATION (rho.cls styles)
# ──────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).resolve().parent
FONTS_DIR = SCRIPT_DIR / "fonts"

custom_fonts = [
    FONTS_DIR / "FiraSans-Bold.ttf",
    FONTS_DIR / "STIXTwoText-VariableFont_wght.ttf",
    FONTS_DIR / "STIXTwoMath-Regular.ttf"
]

for font_path in custom_fonts:
    if font_path.exists():
        try:
            fm.fontManager.addfont(str(font_path))
        except Exception as e:
            print(f"Warning: Failed to dynamically register font {font_path.name}: {e}")
    else:
        print(f"Warning: Custom font file not found at {font_path}")

# ──────────────────────────────────────────────────────────
# VISUALIZATION CONFIGURATION (rho.cls style)
# ──────────────────────────────────────────────────────────
# Configure the paths to the model run folders (containing run_data.json)
# and their corresponding plot target directories.
MODEL_RUNS = {
    "generalized": {
        "run_dir": "model/model_results/final_run_lopo_2",
        "output_dir": "visualization/run_plots_gen"
    },
    "specialized": {
        # canonical par-spec standard: st3 (no-balance, OneCycleLR) full modality-ablation run.
        # Points at the multi-modality ROOT so one pass produces per-modality + modality-comparison
        # plots in run_plots_par_spec (no separate _ablation folder needed). The gen-vs-par comparison
        # collapses this root to its 'all' sub-run via _load(). Source: run_20260605_113834.
        # Previous standard: ST-transformer-par-spec-P01 (st1, kept as historical baseline).
        "run_dir": "model/model_results/final_run_par_spec",
        "output_dir": "visualization/run_plots_par_spec"
    },
    "ablation": {
        "run_dir": "model/model_results/run_modality_ablation",
        "output_dir": "visualization/run_modality_ablation"
    }
}

# Fallback values for single-run arguments
RUN_DIR = "model/model_results/final_run"
OUTPUT_DIR = "visualization/run_plots_gen"

# Physical page width scaling template: 
# - "column": single-column width (~3.46")
# - "double": double-column width (~7.0")
# - "default": standard presentation width (~8.0")
LAYOUT_WIDTH = "default"

# Specific plots to generate. 
# Options: "loss", "regression", "seqlen", "participant", "distribution", "weight_error", "ablation", "importance_channel", "importance_feature", "tukey", "deepshap_channel", "deepshap_feature_type", "deepshap_modality"
# Set to None to automatically generate all available plots, or specify a list: e.g. ["loss", "regression"]
PLOTS_TO_GENERATE = None

# Custom titles for generated plots. If set to None or an empty string,
# a default dynamic title based on the model run and metadata will be used.
# Supports formatting placeholders: {model_type}, {top_n}, {n_folds}, {total_samples}
PLOT_TITLES = {
    "loss": "{model_type}: Training Progress",
    "regression": "Participant-specialized ST-Transformer: Regression Performance",
    "seqlen": "Participant-specialized ST-Transformer: Performance by Segment Length",
    "participant": "{model_type}: Generalization per Participant",
    "distribution": "{model_type}: Dataset Composition",
    "weight_error": "Participant-specialized ST-Transformer: Performance by Weight Class",
    "ablation": "Sensor Modality Ablation Study",
    "importance_channel": "{model_type}: Channel Permutation Importance (Top {top_n})",
    "importance_feature": "{model_type}: Feature Permutation Importance (Top {top_n})",
    "tradeoff": "Generalization Accuracy vs. Inference Latency Trade-Off",
    "importance_individual": "{model_type}: Granular Feature Permutation Importance (Top {top_n})",
    "tukey": "Tukey HSD Pairwise Significance (Prediction Error)",
    "deepshap_channel": "{model_type}: DeepSHAP Channel Importance",
    "deepshap_feature_type": "{model_type}: DeepSHAP Feature Importance (Top {top_n})",
    "deepshap_modality": "{model_type}: DeepSHAP Modality Split",
    "modality_metrics": "Modality Performance Comparison",
    "modality_seqlen_mae": "MAE by Segment Length and Modality",
    "modality_seqlen_rmse": "RMSE by Segment Length and Modality",
    "modality_participant_mae": "MAE per Participant by Modality",
    "modality_participant_rmse": "RMSE per Participant by Modality",
    "modality_participant_gap": "Generalization Gap (MAE) per Participant by Modality",
    "modality_participant_ratio": "Generalization Ratio (RMSE) per Participant by Modality",
    "modality_errors": "Overall Error Distributions by Modality",
    "modality_weight_mae": "MAE by Weight Class and Modality",
    "modality_weight_rmse": "RMSE by Weight Class and Modality"
}

CURRENT_OUTPUT_DIR = None

def get_plot_title(plot_name, default_val, **kwargs):
    """Retrieves and formats the custom title from PLOT_TITLES if defined, else returns default_val."""
    global CURRENT_OUTPUT_DIR
    is_gen = False
    if CURRENT_OUTPUT_DIR and "run_plots_gen" in CURRENT_OUTPUT_DIR:
        is_gen = True

    custom_title = PLOT_TITLES.get(plot_name) if PLOT_TITLES else None
    
    if is_gen:
        # Override dynamic model_type to 'Generalized ST-Transformer' with modality suffix
        modality = kwargs.get("modality")
        mod_map = {
            "all": "Sensor-Fused",
            "emg_only": "EMG-only",
            "imu_only": "IMU-only"
        }
        mod_suffix = f" ({mod_map.get(modality, modality.replace('_', '-'))})" if modality else ""
        kwargs["model_type"] = f"Generalized ST-Transformer{mod_suffix}"
        
        # Replace Participant-specialized with Generalized ST-Transformer
        if custom_title:
            custom_title = custom_title.replace("Participant-specialized ST-Transformer", "Generalized ST-Transformer")
            custom_title = custom_title.replace("Participant-specialized", "Generalized")
        default_val = default_val.replace("Participant-specialized ST-Transformer", "Generalized ST-Transformer")
        default_val = default_val.replace("Participant-specialized", "Generalized")

    if custom_title:
        try:
            formatted_title = custom_title.format(**kwargs)
        except Exception as e:
            print(f"Warning: Failed to format custom title for '{plot_name}': {e}")
            formatted_title = custom_title
    else:
        formatted_title = default_val.format(**kwargs)

    # Force 'Generalized ST-Transformer:' to be in the title prefix if we are in run_plots_gen
    if is_gen and "Generalized ST-Transformer" not in formatted_title:
        formatted_title = f"Generalized ST-Transformer: {formatted_title}"

    return formatted_title

# ===========================================================================================
# 1. LaTeX Thesis Style Configurator (based on rho.cls)
# ===========================================================================

# ──────────────────────────────────────────────────────────
# DYNAMIC COLOR PALETTE IMPORT FROM COLOR_PICKER.PY
# ──────────────────────────────────────────────────────────
try:
    # Add script dir to sys.path to enable loading color_picker if run from root
    import sys
    from pathlib import Path
    _script_dir = str(Path(__file__).resolve().parent)
    if _script_dir not in sys.path:
        sys.path.insert(0, _script_dir)
        
    from color_picker import (
        COLOR_EMG, COLOR_IMU, COLOR_FUSION,
        COLOR_MAE, COLOR_RMSE, COLOR_R2, COLOR_GAP, COLOR_COUNT,
        COLOR_UNITY, COLOR_MEDIAN, COLOR_TARGET, COLOR_OUTLIER,
        COLOR_P_001, COLOR_P_01, COLOR_P_05, COLOR_P_NS,
        COLOR_SHAPLEY, COLOR_DEEPSHAP, COLOR_PERMUTATION,
        COLOR_HEATMAP_PRESENT, COLOR_HEATMAP_ABSENT
    )
except ImportError:
    # Fallback to local declarations if import fails
    COLOR_EMG = "#95356F"
    COLOR_IMU = "#007355"
    COLOR_FUSION = "#FF9000"
    COLOR_MAE = "#004488"
    COLOR_RMSE = "#BB5566"
    COLOR_R2 = "#774499"
    COLOR_GAP = "#C85200"
    COLOR_COUNT = "#4A5568"
    COLOR_UNITY = "#888888"
    COLOR_MEDIAN = "#DDAA33"
    COLOR_TARGET = "#228833"
    COLOR_OUTLIER = "#BBBBBB"
    COLOR_P_001 = "#CC3311"
    COLOR_P_01 = "#AA4499"
    COLOR_P_05 = "#4477AA"
    COLOR_P_NS = "#BBBBBB"
    COLOR_SHAPLEY = "#2C3E50"
    COLOR_DEEPSHAP = "#4499FF"
    COLOR_PERMUTATION = "#D55E00"
    COLOR_HEATMAP_PRESENT = "#003366"
    COLOR_HEATMAP_ABSENT = "#F3F4F6"



def clean_channel_label(raw_lbl):
    clean_lbl = raw_lbl.replace("_EMG", "").replace("_IMU", "")
    clean_lbl = clean_lbl.replace("Extensor Carpi Radialis (ECR)", "ECR").replace("Extensor Carpi Radialis", "ECR")
    clean_lbl = clean_lbl.replace("Flexor Carpi Ulnaris (FCU)", "FCU").replace("Flexor Carpi Ulnaris", "FCU")
    
    # Check if there is an _Accel suffix or similar
    suffix = ""
    if clean_lbl.endswith("_Accel"):
        clean_lbl = clean_lbl[:-6]
        suffix = " (Accel)"
    elif clean_lbl.endswith(" (Accel)"):
        clean_lbl = clean_lbl[:-8]
        suffix = " (Accel)"
        
    # Replace sensors with LaTeX math symbols
    replacements = {
        "roll_rad1": r"$\phi_1$",
        "pitch_rad1": r"$\theta_1$",
        "yaw_rad1": r"$\psi_1$",
        "ax1": r"$a_{x,1}$",
        "ay1": r"$a_{y,1}$",
        "az1": r"$a_{z,1}$",
        
        "roll_rad2": r"$\phi_2$",
        "pitch_rad2": r"$\theta_2$",
        "yaw_rad2": r"$\psi_2$",
        "ax2": r"$a_{x,2}$",
        "ay2": r"$a_{y,2}$",
        "az2": r"$a_{z,2}$",
    }
    
    for k, v in replacements.items():
        if k in clean_lbl:
            clean_lbl = clean_lbl.replace(k, v)
            break
            
    return clean_lbl + suffix


class ThesisStyle:
    # ─── 1. Paul Tol Scientific Semantic Colors ───
    COLOR_EMG = COLOR_EMG
    COLOR_IMU = COLOR_IMU
    COLOR_FUSION = COLOR_FUSION

    # ─── 2. Consistent Statistical Metric Colors ───
    COLOR_MAE = COLOR_MAE
    COLOR_RMSE = COLOR_RMSE
    COLOR_R2 = COLOR_R2
    COLOR_GAP = COLOR_GAP
    COLOR_COUNT = COLOR_COUNT

    # ─── 3. Reference and Boundary Colors ───
    COLOR_UNITY = COLOR_UNITY
    COLOR_MEDIAN = COLOR_MEDIAN
    COLOR_TARGET = COLOR_TARGET
    COLOR_OUTLIER = COLOR_OUTLIER

    # ─── 4. Pairwise Tukey HSD Significance Colors ───
    COLOR_P_001 = COLOR_P_001
    COLOR_P_01 = COLOR_P_01
    COLOR_P_05 = COLOR_P_05
    COLOR_P_NS = COLOR_P_NS

    # ─── 5. Interpretability Explanation Methods ───
    COLOR_SHAPLEY = COLOR_SHAPLEY
    COLOR_DEEPSHAP = COLOR_DEEPSHAP
    COLOR_PERMUTATION = COLOR_PERMUTATION

    # ─── 6. Modality Ablation Heatmap Grid Colors ───
    COLOR_HEATMAP_PRESENT = COLOR_HEATMAP_PRESENT
    COLOR_HEATMAP_ABSENT = COLOR_HEATMAP_ABSENT

    # Color Palette from rho.cls (for backwards compatibility)
    RHO_BLUE = "#003366"        # Deep dark blue from rho.cls (rgb: 0.0, 0.2, 0.4)
    RHO_LIGHT_BLUE = "#F0F4F8"  # Very soft Paul Tol blue tint
    ACCENT_RED = "#BB5566"      # Paul Tol High-Contrast Red
    ACCENT_BLUE = "#4477AA"     # Paul Tol Bright Blue
    NEUTRAL_GRAY = "#888888"    # Paul Tol Slate Grey
    GRID_GRAY = "#E5E7EB"       # Subtle, thin grid lines (light gray)
    
    @classmethod
    def apply(cls, layout_width="default"):
        """Applies global style parameters to Matplotlib for thesis-ready look."""
        plt.rcParams.update(plt.rcParamsDefault)
        
        # Determine sizing and margins (optimized for LaTeX one-column format)
        # We enforce a strict golden-ratio aspect ratio (~1.6:1) for consistancy across all plots
        if layout_width == "column":
            # Compact figure size (approx 1.6 aspect ratio)
            figsize = (3.5, 2.2)
            linewidth = 1.0
            markersize = 3.5
        elif layout_width == "double":
            # Large figure size (golden ratio 1.6 aspect ratio)
            figsize = (7.0, 4.3)
            linewidth = 1.4
            markersize = 4.5
        else: # "default" standard single-column width
            # Standard single-column width (golden ratio 1.6 aspect ratio)
            figsize = (6.0, 3.7)
            linewidth = 1.2
            markersize = 4.0
            
        # Unified font sizes across all layouts to match the double column format 
        # (ensuring consistent printed text and math sizing in LaTeX documents)
        fontsize = 9.0
        labelsize = 9.5
        titlesize = 10.0
        legendsize = 8.0
        ticksize = 8.0
            
        # Configure rcParams dynamically
        plt.rcParams.update({
            'figure.figsize': figsize,
            'font.size': fontsize,
            'axes.labelsize': labelsize,
            'axes.titlesize': titlesize,
            'xtick.labelsize': ticksize,
            'ytick.labelsize': ticksize,
            'legend.fontsize': legendsize,
            'lines.linewidth': linewidth,
            'lines.markersize': markersize,
            
            # Use STIX Two Text as default serif font family
            'font.family': 'serif',
            'font.serif': ['STIX Two Text', 'DejaVu Serif', 'Times New Roman', 'serif'],
            'font.sans-serif': ['Fira Sans', 'DejaVu Sans', 'Arial', 'Helvetica', 'sans-serif'],
            
            # Match math text with LaTeX's STIX font set
            'mathtext.fontset': 'stix',
            
            # Borders & Spines (Clean, professional look)
            'axes.spines.top': False,
            'axes.spines.right': False,
            'axes.spines.left': True,
            'axes.spines.bottom': True,
            'axes.edgecolor': '#2C3E50',
            'axes.linewidth': 0.6,
            'axes.axisbelow': True,
            
            # Clean Bold titles (consistent styled headers)
            'axes.titleweight': 'bold',
            'axes.titlecolor': cls.RHO_BLUE,
            
            # Thin, muted grid lines behind data
            'axes.grid': True,
            'grid.color': cls.GRID_GRAY,
            'grid.alpha': 0.7,
            'grid.linestyle': '--',
            'grid.linewidth': 0.5,
            
            # Legend styling
            'legend.frameon': True,
            'legend.facecolor': 'white',
            'legend.edgecolor': cls.GRID_GRAY,
            'legend.framealpha': 0.95,
            'legend.fancybox': True,
            
            # Ticks
            'xtick.major.size': 3.5,
            'xtick.major.width': 0.6,
            'ytick.major.size': 3.5,
            'ytick.major.width': 0.6,
            
            # Save settings
            'savefig.dpi': 300,
            'savefig.bbox': 'tight',
            'savefig.transparent': False
        })

        # Monkey patch plt.tight_layout to automatically add padding
        if not hasattr(plt, '_original_tight_layout'):
            plt._original_tight_layout = plt.tight_layout
            
        def custom_tight_layout(*args, **kwargs):
            if 'pad' not in kwargs:
                kwargs['pad'] = 1.2
            if 'h_pad' not in kwargs:
                kwargs['h_pad'] = 1.5
            if 'w_pad' not in kwargs:
                kwargs['w_pad'] = 1.5
            return plt._original_tight_layout(*args, **kwargs)
            
        plt.tight_layout = custom_tight_layout

    @classmethod
    def set_title(cls, ax, title_text):
        """Applies consistent title styling using Fira Sans Bold in deep darkblue."""
        ax.set_title(
            title_text,
            fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['axes.titlesize']),
            color=cls.RHO_BLUE,
            pad=10
        )

    @classmethod
    def set_suptitle(cls, fig, title_text):
        """Applies consistent suptitle styling using Fira Sans Bold in deep darkblue."""
        fig.suptitle(
            title_text,
            fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['axes.titlesize'] + 0.5),
            color=cls.RHO_BLUE,
            y=0.98
        )

    @classmethod
    def style_ax(cls, ax):
        """Standardizes axis label and tick fonts to STIX Two Text."""
        for label in ax.get_xticklabels() + ax.get_yticklabels():
            label.set_fontfamily('serif')
        ax.xaxis.label.set_fontfamily('serif')
        ax.yaxis.label.set_fontfamily('serif')

    @classmethod
    def save_figure(cls, fig, output_path):
        """Saves figure in both high-resolution PDF vector format and PNG format."""
        # Export PDF (academic standard vector graphic)
        fig.savefig(output_path.with_suffix(".pdf"), dpi=300, bbox_inches='tight')
        # Export PNG (preview/web format)
        fig.savefig(output_path.with_suffix(".png"), dpi=300, bbox_inches='tight')


# ===========================================================================
# 2. Modular Plotter Registry
# ===========================================================================

class PlotterRegistry:
    def __init__(self):
        self._plotters = {}

    def register(self, name):
        """Decorator to register a new Plotter class."""
        def decorator(cls):
            self._plotters[name] = cls()
            return cls
        return decorator

    def get_plotter(self, name):
        return self._plotters.get(name)

    def list_plotters(self):
        return list(self._plotters.keys())


plot_registry = PlotterRegistry()


# ===========================================================================
# 3. Core Visualizers
# ===========================================================================

@plot_registry.register("loss")
class LossPlotter:
    """Plots training and validation loss progress over epochs."""
    
    def plot(self, data, output_path, layout_width):
        histories = data.get("training_histories")
        if not histories:
            print("  [Warning] No training histories found. Skipping loss plot.")
            return False
            
        ThesisStyle.apply(layout_width)
        fig, ax = plt.subplots()
        
        is_cv = len(histories) > 1
        all_train = []
        all_val = []
        max_epochs = 0
        
        for h in histories:
            if "train" in h:
                all_train.append(h["train"])
                max_epochs = max(max_epochs, len(h["train"]))
            if "val" in h and h["val"]:
                all_val.append(h["val"])
                
        epochs = np.arange(1, max_epochs + 1)
        
        # Stack lists with NaN padding for unequal fold lengths
        def pad_and_stack(lists, target_len):
            padded = []
            for lst in lists:
                padded.append(list(lst) + [np.nan] * (target_len - len(lst)))
            return np.array(padded)
            
        if is_cv:
            train_stacked = pad_and_stack(all_train, max_epochs)
            train_mean = np.nanmean(train_stacked, axis=0)
            
            # Plot individual fold paths in thin, transparent curves
            for i, f_train in enumerate(all_train):
                ax.plot(np.arange(1, len(f_train) + 1), f_train, 
                        color=ThesisStyle.COLOR_MAE, alpha=0.15, linewidth=0.7, 
                        label="Individual Folds" if i == 0 else "")
                
            # Plot robust fold average in bold
            ax.plot(epochs, train_mean, color=ThesisStyle.COLOR_MAE, linewidth=1.8, 
                    label="Mean Training Loss")
            
            if all_val:
                val_stacked = pad_and_stack(all_val, max_epochs)
                val_mean = np.nanmean(val_stacked, axis=0)
                
                for i, f_val in enumerate(all_val):
                    ax.plot(np.arange(1, len(f_val) + 1), f_val, 
                            color=ThesisStyle.COLOR_RMSE, alpha=0.15, linewidth=0.7, linestyle='--',
                            label="Individual Folds (Val)" if i == 0 else "")
                    
                ax.plot(epochs, val_mean, color=ThesisStyle.COLOR_RMSE, linewidth=1.8, linestyle='--',
                        label="Mean Validation Loss")
        else:
            # Single train/val run
            if all_train:
                ax.plot(np.arange(1, len(all_train[0]) + 1), all_train[0], 
                        color=ThesisStyle.COLOR_MAE, linewidth=1.8, label="Training Loss")
            if all_val:
                ax.plot(np.arange(1, len(all_val[0]) + 1), all_val[0], 
                        color=ThesisStyle.COLOR_RMSE, linewidth=1.8, linestyle='--', label="Validation Loss")
                
        ax.set_yscale('log')
        ax.set_xlabel("Epoch")
        ax.set_ylabel("Loss (log scale)")
        
        # Mixed-case semibold title formatting (consistent with paper guidelines)
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        n_folds_str = f" ({len(histories)} Folds)" if is_cv else ""
        default_title = f"{{model_type}}: Training Progress{n_folds_str}"
        title = get_plot_title("loss", default_title, model_type=model_type, modality=data.get("config", {}).get("ablation_modality"))
        ThesisStyle.set_title(ax, title)
        
        ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        
        # Save standard outputs
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("regression")
class RegressionPlotter:
    """Creates an enhanced Predicted vs Actual weight box plot with premium sequential gradient aesthetics."""
    
    def plot(self, data, output_path, layout_width):
        predictions = data.get("predictions")
        if not predictions or "y_true" not in predictions or "y_pred" not in predictions:
            print("  [Warning] Predictions not found in run data. Skipping regression plot.")
            return False
            
        y_true = np.array(predictions["y_true"])
        y_pred = np.maximum(0.0, np.array(predictions["y_pred"])) # Clamp strictly at 0kg
        
        ThesisStyle.apply(layout_width)
        fig, ax = plt.subplots()
        
        actual_weights = np.sort(np.unique(y_true))
        pred_groups = [y_pred[y_true == w] for w in actual_weights]
        
        min_val = 0.0
        data_max = max(np.max(y_true), np.max(y_pred))
        max_val = min(7.5, max(6.25, data_max + 0.25))
        
        # Determine colors based on the run suffix or metadata
        out_name = output_path.name
        if "emg_only" in out_name:
            primary_color = ThesisStyle.COLOR_EMG
            fill_color = "#FAF0F5"
            accent_color = "#551133"
        elif "imu_only" in out_name:
            primary_color = ThesisStyle.COLOR_IMU
            fill_color = "#F0F7F4"
            accent_color = "#115522"
        else: # "all" (Fusion) or fallback
            primary_color = ThesisStyle.COLOR_FUSION
            fill_color = "#EBF7FA"
            accent_color = "#004D60"

        # Perfect prediction line (Ideal Unity)
        ax.plot([min_val, max_val], [min_val, max_val], 
                color=ThesisStyle.COLOR_UNITY, linestyle='--', linewidth=1.2, alpha=0.8, 
                label='Perfect Prediction', zorder=1)
        
        # Cleanly downsample outliers (especially at 0 kg where density is extremely high)
        # to avoid solid blue blocks and maintain highly readable vector-like scatter density.
        first_outlier = True
        for i, w in enumerate(actual_weights):
            preds = pred_groups[i]
            if len(preds) > 0:
                q1 = np.percentile(preds, 25)
                q3 = np.percentile(preds, 75)
                iqr = q3 - q1
                outliers = preds[(preds < q1 - 1.5 * iqr) | (preds > q3 + 1.5 * iqr)]
                
                if len(outliers) > 0:
                    # Cleanly downsample to a maximum of 100 outliers for clean visuals
                    if len(outliers) > 100:
                        np.random.seed(42) # Fixed seed for reproducible visuals
                        outliers = np.random.choice(outliers, size=100, replace=False)
                        
                    label = "Outliers" if first_outlier else ""
                    ax.scatter(np.full_like(outliers, w), outliers, 
                               alpha=0.35, s=12, color='#7F8C8D', edgecolors='none', 
                               zorder=2, label=label)
                    first_outlier = False
                    
        # Grouped box plots styled like LaTeX environments with caps-free modern aesthetic
        bp = ax.boxplot(pred_groups, positions=actual_weights, widths=0.32, 
                        patch_artist=True, showfliers=False, showcaps=False,
                        whiskerprops={'color': '#475569', 'linewidth': 1.0},
                        boxprops={'edgecolor': primary_color, 'linewidth': 1.2},
                        zorder=3)
        
        # Single uniform fill color for all boxes (soft light blue/red/purple for a clean, professional look)
        for box in bp['boxes']:
            box.set_facecolor(fill_color)
            box.set_alpha(0.85)
            
        # Vibrant Golden Amber/Red/Blue for all median lines for high contrast and clean consistency
        for idx, median in enumerate(bp['medians']):
            median.set_color(accent_color)
            median.set_linewidth(1.5)
            if idx == 0:
                median.set_label('Median')
            
        # Stats annotation styled like a premium LaTeX card box (Rounded, with RMSE added).
        # Use participant-balanced (macro) metrics as the single reported figure; fall back to
        # pooled only for single-split runs where no macro average exists.
        _ev = data.get("evaluation", {})
        cpm = _ev.get("macro_metrics", {}).get("class_participant_macro", {}) or _ev.get("class_participant_macro", {})
        eval_stats = cpm if cpm.get("MAE") is not None else (_ev.get("macro_avg") or _ev.get("pooled", {}))
        r2 = eval_stats.get("R2")
        mae = eval_stats.get("MAE")
        rmse = eval_stats.get("RMSE")
        
        if r2 is None or mae is None or rmse is None:
            from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error
            r2 = r2_score(y_true, y_pred)
            mae = mean_absolute_error(y_true, y_pred)
            rmse = np.sqrt(mean_squared_error(y_true, y_pred))
            
        stats_text = f"$\\text{{R}}^2 = {r2:.3f}$\n$\\mathrm{{MAE}} = {mae:.3f}$ kg\n$\\mathrm{{RMSE}} = {rmse:.3f}$ kg"
        
        ax.text(0.04, 0.96, stats_text, 
                transform=ax.transAxes, verticalalignment='top', fontsize=plt.rcParams['font.size'] - 1.5,
                bbox=dict(facecolor='white', alpha=0.95, 
                          edgecolor=primary_color, boxstyle='round,pad=0.5', linewidth=1.0))
        
        ax.set_xlabel("Actual Weight (kg)", labelpad=8)
        ax.set_ylabel("Predicted Weight (kg)", labelpad=8)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        if "ST-transformer-par-spec" in str(output_path) or "spec-P01" in str(output_path):
            default_title = "Participant-specialized ST-Transformer: Regression Performance"
        else:
            default_title = "{model_type}: Predicted vs. Actual Weight"
        title = get_plot_title("regression", default_title, model_type=model_type, modality=data.get("config", {}).get("ablation_modality"))
        ThesisStyle.set_title(ax, title)
        
        ax.set_xlim(min_val - 0.1, max_val)
        ax.set_ylim(min_val - 0.1, max_val)
        
        # Custom Ticks
        ax.set_xticks(actual_weights)
        ax.set_yticks(actual_weights)
        
        def format_label(x):
            return f"{x:.2f}".rstrip('0').rstrip('.')
            
        ax.set_xticklabels([format_label(w) for w in actual_weights])
        ax.set_yticklabels([format_label(w) for w in actual_weights])
        
        ax.legend(loc='lower right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        
        # Save standard outputs
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("calibration")
class CalibrationPlotter:
    """Visualizes the affine recalibration: per-weight mean prediction before vs after,
    against the identity line, with the fitted calibration transform and MAE/RMSE deltas.
    Requires a calibrated run (predictions must contain 'y_pred_uncalibrated')."""

    def plot(self, data, output_path, layout_width):
        preds = data.get("predictions") or {}
        if "y_pred_uncalibrated" not in preds or "y_true" not in preds or "y_pred" not in preds:
            print("  [Warning] No uncalibrated predictions in run data. Skipping calibration plot.")
            return False

        y_true = np.array(preds["y_true"], dtype=float).flatten()
        y_cal = np.array(preds["y_pred"], dtype=float).flatten()
        y_unc = np.array(preds["y_pred_uncalibrated"], dtype=float).flatten()

        recal = data.get("recalibration", {}) or {}
        b, a = recal.get("slope_b"), recal.get("intercept_a")
        mb, ma = recal.get("metrics_before", {}), recal.get("metrics_after", {})

        weights = np.sort(np.unique(np.round(y_true, 2)))
        mean_unc = [float(y_unc[np.abs(y_true - w) < 0.05].mean()) for w in weights]
        mean_cal = [float(y_cal[np.abs(y_true - w) < 0.05].mean()) for w in weights]

        ThesisStyle.apply(layout_width)
        fig, ax = plt.subplots()

        out_name = output_path.name
        if "emg_only" in out_name:
            cal_color = ThesisStyle.COLOR_EMG
        elif "imu_only" in out_name:
            cal_color = ThesisStyle.COLOR_IMU
        else:
            cal_color = ThesisStyle.COLOR_FUSION

        lim = float(max(weights.max(), max(mean_unc), max(mean_cal)) * 1.08)
        ax.plot([0, lim], [0, lim], color=ThesisStyle.COLOR_UNITY, linestyle="--",
                linewidth=1.2, alpha=0.8, label="Perfect prediction", zorder=1)
        ax.plot(weights, mean_unc, marker="o", linestyle="-", color="#C0392B",
                linewidth=1.6, markersize=6, label="Uncalibrated (per-weight mean)", zorder=3)
        ax.plot(weights, mean_cal, marker="s", linestyle="-", color=cal_color,
                linewidth=1.6, markersize=6, label="Calibrated (per-weight mean)", zorder=4)

        ann = []
        if b is not None and a is not None:
            ann.append(f"$\\hat{{y}}_{{\\mathrm{{cal}}}} = {b:.3f}\\,\\hat{{y}} + {a:.3f}$")
        if mb.get("MAE") is not None and ma.get("MAE") is not None:
            ann.append(f"$\\mathrm{{MAE}}$: {mb['MAE']:.3f} $\\to$ {ma['MAE']:.3f} kg")
        if mb.get("RMSE") is not None and ma.get("RMSE") is not None:
            ann.append(f"$\\mathrm{{RMSE}}$: {mb['RMSE']:.3f} $\\to$ {ma['RMSE']:.3f} kg")
        if ann:
            ax.text(0.04, 0.96, "\n".join(ann), transform=ax.transAxes, va="top",
                    bbox=dict(boxstyle="round", facecolor="white", edgecolor="#E0E0E0", alpha=0.95))

        ax.set_xlabel("Actual Weight (kg)")
        ax.set_ylabel("Mean Predicted Weight (kg)")
        ThesisStyle.set_title(ax, "Affine Recalibration: Mean Prediction per Weight")
        ax.set_xticks(weights)
        ax.set_xticklabels([f"{w:.2f}".rstrip("0").rstrip(".") for w in weights])
        ax.set_xlim(-0.1, lim)
        ax.set_ylim(-0.1, lim)
        ax.legend(loc="lower right", frameon=True, facecolor="white",
                  edgecolor="#E0E0E0", framealpha=0.95)
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("seqlen")
class SeqLenPlotter:
    """Plots error metrics vs available training context duration."""
    
    def plot(self, data, output_path, layout_width):
        per_seqlen_stats = data.get("evaluation", {}).get("per_seqlen")
        if not per_seqlen_stats:
            per_seqlen_stats = data.get("evaluation", {}).get("per_duration")
        if not per_seqlen_stats:
            print("  [Warning] No sequence length or duration stats found. Skipping seqlen plot.")
            return False
            
        min_count = 50
        filtered = [row for row in per_seqlen_stats if int(row['Count']) >= min_count]
        if not filtered:
            print(f"  [Warning] All bins dropped due to min_count={min_count}. Skipping seqlen plot.")
            return False
            
        ThesisStyle.apply(layout_width)
        
        # Check if we are running generalized model plots to limit x-axis to 3.0s
        global CURRENT_OUTPUT_DIR
        is_gen = False
        if CURRENT_OUTPUT_DIR and "run_plots_gen" in CURRENT_OUTPUT_DIR:
            is_gen = True

        times = []
        maes = []
        rmses = []
        counts = []
        for row in filtered:
            t = float(row['TimeAtPrediction'].rstrip('s'))
            if is_gen and t > 3.0:
                continue
            times.append(t)
            maes.append(float(row['MAE']))
            rmses.append(float(row['RMSE']))
            counts.append(int(row['Count']))
        
        fig, ax1 = plt.subplots()
        
        # Elegant bar chart for sample count (right y-axis) representing samples in bins
        ax2 = ax1.twinx()
        if len(times) > 1:
            bar_width = 0.5 * np.min(np.diff(times))
        else:
            bar_width = 0.1
        ax2.bar(times, counts, width=bar_width, color=ThesisStyle.RHO_LIGHT_BLUE, alpha=0.6,
                edgecolor=ThesisStyle.COLOR_UNITY, linewidth=0.5, label='Segment Count', zorder=1)
        
        ax2.set_ylabel("Segment Count", color=ThesisStyle.NEUTRAL_GRAY)
        ax2.tick_params(axis='y', labelcolor=ThesisStyle.NEUTRAL_GRAY)
        ax2.set_ylim(0, max(counts) * 3.0) # push area chart cleanly into bottom third
        ax2.spines['top'].set_visible(False)
        ax2.spines['right'].set_visible(True)
        ax2.spines['left'].set_visible(True)
        ax2.grid(False) # Disable secondary gridlines
        
        # Primary error lines (window counts text removed for clutter-free paper format)
        ax1.plot(times, maes, marker='o', color=ThesisStyle.COLOR_MAE, label='MAE', zorder=3)
        ax1.plot(times, rmses, marker='s', color=ThesisStyle.COLOR_RMSE, label='RMSE', linestyle='--', zorder=3)
        
        ax1.set_xlabel("Segment lengths (s)", labelpad=8)
        ax1.set_ylabel("Error (kg)", labelpad=8)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        title = get_plot_title("seqlen", "{model_type}: Error vs. Available Segment Length", model_type=model_type, modality=data.get("config", {}).get("ablation_modality"))
        ThesisStyle.set_title(ax1, title)
        ax1.set_ylim(bottom=0)
        
        # Setup ticks
        ax1.set_xticks(times)
        if len(times) > 15:
            labels = [f"{t:.2f}" if idx % 2 == 0 or idx == len(times)-1 else "" for idx, t in enumerate(times)]
            ax1.set_xticklabels(labels, rotation=30, ha='right')
        else:
            ax1.set_xticklabels([f"{t:.2f}" for t in times], rotation=30, ha='right')
            
        # Combine legends cleanly
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2,
                   loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        ax1.set_zorder(ax2.get_zorder() + 1)
        ax1.patch.set_visible(False)
        plt.tight_layout()
        
        # Save standard outputs
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("participant")
class ParticipantPlotter:
    """Bar chart demonstrating generalization error per left-out participant."""
    
    def plot(self, data, output_path, layout_width):
        per_participant_stats = data.get("evaluation", {}).get("per_participant")
        if not per_participant_stats:
            print("  [Warning] No participant stats found. Skipping participant performance plot.")
            return False
            
        # Sort by RMSE ascending
        sorted_stats = sorted(per_participant_stats, key=lambda x: float(x['RMSE']))
        
        participants = [stat['Participant'] for stat in sorted_stats]
        maes = [float(stat['MAE']) for stat in sorted_stats]
        rmses = [float(stat['RMSE']) for stat in sorted_stats]
        
        ThesisStyle.apply(layout_width)
        
        x = np.arange(len(participants))
        width = 0.35
        
        fig, ax = plt.subplots()
        
        # Side-by-side bars matching thesis color guidelines
        rects1 = ax.bar(x - width/2, maes, width, label='MAE', color=ThesisStyle.COLOR_MAE, alpha=0.85)
        rects2 = ax.bar(x + width/2, rmses, width, label='RMSE', color=ThesisStyle.COLOR_RMSE, alpha=0.85)
        
        # Horizontal lines mapping the mean error metrics
        _ev = data.get("evaluation", {})
        cpm = _ev.get("macro_metrics", {}).get("class_participant_macro", {}) or _ev.get("class_participant_macro", {})
        if not cpm or cpm.get("MAE") is None:
            cpm = _ev.get("class_macro_avg", {}) or _ev.get("class_macro", {})
            if not cpm:
                cpm = _ev.get("macro_metrics", {}).get("class_macro", {})
                
        mean_mae = cpm.get("MAE") if (cpm and cpm.get("MAE") is not None) else np.mean(maes)
        mean_rmse = cpm.get("RMSE") if (cpm and cpm.get("RMSE") is not None) else np.mean(rmses)
        
        ax.axhline(mean_mae, color=ThesisStyle.COLOR_MAE, linestyle='--', linewidth=1.1, alpha=0.7)
        ax.axhline(mean_rmse, color=ThesisStyle.COLOR_RMSE, linestyle='--', linewidth=1.1, alpha=0.7)
        
        # Position mean horizontal text label elegantly on the left at x=0.02 (axis relative)
        # to ensure they are 100% readable, never crop, and never overlap.
        # Add a tiny background card for crisp text rendering over columns.
        text_bbox = dict(facecolor='white', alpha=0.85, edgecolor='none', boxstyle='square,pad=0.1')
        
        ax.text(0.02, mean_mae + 0.015, f"Mean MAE: {mean_mae:.3f}", 
                transform=ax.get_yaxis_transform(), color=ThesisStyle.COLOR_MAE, 
                fontsize=plt.rcParams['font.size'] - 2.0, ha='left', va='bottom', bbox=text_bbox)
                
        ax.text(0.02, mean_rmse + 0.015, f"Mean RMSE: {mean_rmse:.3f}", 
                transform=ax.get_yaxis_transform(), color=ThesisStyle.COLOR_RMSE, 
                fontsize=plt.rcParams['font.size'] - 2.0, ha='left', va='bottom', bbox=text_bbox)
        
        ax.set_ylabel('Error (kg)', labelpad=8)
        ax.set_xlabel('Unseen Participant Left Out', labelpad=8)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        title = get_plot_title("participant", "{model_type}: Generalization per Participant", model_type=model_type, modality=data.get("config", {}).get("ablation_modality"))
        ThesisStyle.set_title(ax, title)
        
        ax.set_xticks(x)
        ax.set_xticklabels(participants, rotation=45, ha='right')
        
        max_val = max(np.max(maes), np.max(rmses))
        ax.set_ylim(0, max_val * 1.25 + 0.5) # Extra buffer for horizontal annotations
        
        ax.legend(loc='upper left', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        
        # Save standard outputs
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


# ===========================================================================
# 4. New Registered Visualizers (for Research Paper Results)
# ===========================================================================

@plot_registry.register("distribution")
class DatasetDistributionPlotter:
    """Plots Weight and Participant sample distributions side-by-side."""
    
    def plot(self, data, output_path, layout_width):
        dataset = data.get("dataset", {})
        weight_dist = dataset.get("weight_distribution")
        part_dist = dataset.get("participant_distribution")
        
        if not weight_dist or not part_dist:
            print("  [Warning] Missing distribution data in run data. Skipping distribution plot.")
            return False
            
        ThesisStyle.apply(layout_width)
        
        # Retrieve training balanced targets dynamically from config_model parameters
        aug_config = data.get("config", {}).get("augmentation_config", {})
        balance_enabled = aug_config.get("enabled", False)
        
        balance_w = aug_config.get("balance_weights", False)
        balance_p = aug_config.get("balance_participants", False)
        
        n_weights = len(weight_dist)
        n_parts = len(part_dist)
        
        weight_target = None
        part_target = None
        
        if balance_enabled:
            if balance_w and balance_p:
                target_g = aug_config.get("target_samples_per_group", 250)
                # Since cross-validation is LOPO, train fold uses (n_parts - 1) participants,
                # but conceptually for the full dataset, the leveled target represents:
                weight_target = n_parts * target_g
                part_target = n_weights * target_g
            elif balance_w:
                weight_target = aug_config.get("target_samples_per_weight", 2000)
            elif balance_p:
                part_target = aug_config.get("target_samples_per_participant", 1500)
                
        # Wide layout for double-column side-by-side presentation
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.0, 5.0) if layout_width != "column" else (8.0, 4.0))
        
        # 1. Left Subplot: Weight Distribution
        weights = sorted([float(w) for w in weight_dist.keys()])
        w_labels = [f"{w:.2f} kg" for w in weights]
        w_counts = [weight_dist[f"{w:.4f}"] for w in weights]
        total_samples = sum(w_counts)
        
        bars1 = ax1.bar(w_labels, w_counts, color=ThesisStyle.COLOR_EMG, alpha=0.85, width=0.45, label='Observed (Original)')
        for bar in bars1:
            h = bar.get_height()
            pct = (h / total_samples) * 100
            ax1.annotate(f"{h}\n({pct:.1f}%)",
                         xy=(bar.get_x() + bar.get_width() / 2, h),
                         xytext=(0, 3), textcoords="offset points",
                         ha='center', va='bottom', fontsize=plt.rcParams['font.size'] - 2.5)
                         
        if weight_target is not None:
            ax1.axhline(weight_target, color=ThesisStyle.COLOR_TARGET, linestyle='--', linewidth=1.2, 
                        label=f'Balanced Target ({weight_target})')
            ax1.text(0.98, weight_target, f"Target: {weight_target}", 
                     transform=ax1.get_yaxis_transform(), color=ThesisStyle.COLOR_TARGET, 
                     fontsize=plt.rcParams['font.size'] - 2.0, ha='right', va='bottom',
                     weight='semibold', bbox=dict(facecolor='white', alpha=0.85, edgecolor='none', pad=0.1))
            ax1.set_ylim(0, max(max(w_counts), weight_target) * 1.18)
            ax1.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        else:
            ax1.set_ylim(0, max(w_counts) * 1.18)
            
        ax1.set_xticks(range(len(w_labels)))
        ax1.set_xticklabels(w_labels)
        ax1.set_ylabel("Sample Count", labelpad=8)
        ax1.set_xlabel("Physical Weight Class", labelpad=8)
        ax1.set_title("Weight Class Distribution", fontsize=plt.rcParams['font.size'])
        
        # 2. Right Subplot: Participant Distribution
        sorted_parts = sorted(part_dist.items(), key=lambda x: x[1], reverse=True)
        p_labels = [x[0] for x in sorted_parts]
        p_counts = [x[1] for x in sorted_parts]
        
        bars2 = ax2.bar(p_labels, p_counts, color=ThesisStyle.COLOR_EMG, alpha=0.85, width=0.55, label='Observed (Original)')
        for bar in bars2:
            h = bar.get_height()
            ax2.annotate(f"{h}",
                         xy=(bar.get_x() + bar.get_width() / 2, h),
                         xytext=(0, 3), textcoords="offset points",
                         ha='center', va='bottom', fontsize=plt.rcParams['font.size'] - 3.0)
                         
        if part_target is not None:
            ax2.axhline(part_target, color=ThesisStyle.COLOR_TARGET, linestyle='--', linewidth=1.2,
                        label=f'Balanced Target ({part_target})')
            ax2.text(0.98, part_target, f"Target: {part_target}",
                     transform=ax2.get_yaxis_transform(), color=ThesisStyle.COLOR_TARGET,
                     fontsize=plt.rcParams['font.size'] - 2.0, ha='right', va='bottom',
                     weight='semibold', bbox=dict(facecolor='white', alpha=0.85, edgecolor='none', pad=0.1))
            ax2.set_ylim(0, max(max(p_counts), part_target) * 1.15)
            ax2.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        else:
            ax2.set_ylim(0, max(p_counts) * 1.15)
            
        ax2.set_ylabel("Sample Count", labelpad=8)
        ax2.set_xlabel("Participant ID", labelpad=8)
        ax2.set_title("Participant Sample Contribution", fontsize=plt.rcParams['font.size'])
        ax2.set_xticks(range(len(p_labels)))
        ax2.set_xticklabels(p_labels, rotation=45, ha='right')
        
        # Main figure title
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        default_title = "{model_type}: Dataset Composition & Augmentation Balancing" if balance_enabled else "{model_type}: Dataset Composition (Total Samples: {total_samples})"
        title_text = get_plot_title("distribution", default_title, model_type=model_type, total_samples=total_samples)
        ThesisStyle.set_suptitle(fig, title_text)
                     
        plt.tight_layout(rect=[0, 0, 1, 0.93])
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("weight_error")
class WeightErrorPlotter:
    """Plots prediction error scaling metrics across actual weight classes."""
    
    def plot(self, data, output_path, layout_width):
        per_weight = data.get("evaluation", {}).get("per_weight")
        if not per_weight:
            print("  [Warning] Missing per-weight stats. Skipping weight_error plot.")
            return False
            
        ThesisStyle.apply(layout_width)
        fig, ax1 = plt.subplots()
        
        w_labels = [row['Weight'].replace(' kg', '').replace('kg', '').strip() for row in per_weight]
        maes = [float(row['MAE']) for row in per_weight]
        rmses = [float(row['RMSE']) for row in per_weight]
        counts = [int(row['Count']) for row in per_weight]
        
        # Elegant bar chart for segment count (right y-axis) representing samples in bins
        ax2 = ax1.twinx()
        bar_width = 0.35
        x_positions = np.arange(len(w_labels))
        
        ax2.bar(x_positions, counts, width=bar_width, color=ThesisStyle.RHO_LIGHT_BLUE, alpha=0.6,
                edgecolor=ThesisStyle.COLOR_UNITY, linewidth=0.5, label='Segment Count', zorder=1)
        
        ax2.set_ylabel("Segment Count", color=ThesisStyle.NEUTRAL_GRAY)
        ax2.tick_params(axis='y', labelcolor=ThesisStyle.NEUTRAL_GRAY)
        ax2.set_ylim(0, max(counts) * 3.0) # push area chart cleanly into bottom third
        ax2.spines['top'].set_visible(False)
        ax2.spines['right'].set_visible(True)
        ax2.spines['left'].set_visible(True)
        ax2.grid(False) # Disable secondary gridlines
        
        # Dual error lines on primary axis
        ax1.plot(x_positions, maes, marker='o', color=ThesisStyle.COLOR_MAE, label='MAE', zorder=3)
        ax1.plot(x_positions, rmses, marker='s', color=ThesisStyle.COLOR_RMSE, label='RMSE', linestyle='--', zorder=3)
        
        ax1.set_xlabel("Weight Class (kg)", labelpad=8)
        ax1.set_ylabel("Prediction Error (kg)", labelpad=8)
        ax1.set_ylim(0, max(rmses) * 1.25)
        
        # Setup ticks
        ax1.set_xticks(x_positions)
        ax1.set_xticklabels(w_labels)
        
        ThesisStyle.style_ax(ax1)
        ThesisStyle.style_ax(ax2)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        if "ST-transformer-par-spec" in str(output_path) or "spec-P01" in str(output_path):
            default_title = "Participant-specialized ST-Transformer: Performance by Weight Class"
        else:
            default_title = "{model_type}: Error Scaling by Weight Class"
        title = get_plot_title("weight_error", default_title, model_type=model_type, modality=data.get("config", {}).get("ablation_modality"))
        ThesisStyle.set_title(ax1, title)
        
        # Combine legends cleanly
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2,
                   loc='upper left', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
                   
        ax1.set_zorder(ax2.get_zorder() + 1)
        ax1.patch.set_visible(False)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("ablation")
class AblationPlotter:
    """Plots a premium comparison of error metrics (MAE and RMSE) across different feature modalities (ablation groups)."""
    
    def plot(self, data, output_path, layout_width):
        is_multi = False
        if "meta" not in data:
            sub_runs = [k for k, v in data.items() if isinstance(v, dict) and "meta" in v]
            if sub_runs:
                is_multi = True
                
        if not is_multi:
            print("  [Warning] Ablation plotter requires a multi-modality ablation run dataset at root. Skipping.")
            return False
            
        ThesisStyle.apply(layout_width)
        
        modalities = []
        maes = []
        rmses = []
        
        # Clean formatting names for paper presentation
        name_map = {
            "all": "EMG + IMU\n(Sensor-Fused)",
            "emg_only": "EMG-only",
            "imu_only": "IMU-only"
        }
        
        # Prioritize predefined modality order
        for key in ["all", "emg_only", "imu_only"]:
            if key not in data or not isinstance(data[key], dict):
                continue
            run_data = data[key]
            eval_dict = run_data.get("evaluation", {})
            cpm = eval_dict.get("macro_metrics", {}).get("class_participant_macro", {}) or eval_dict.get("class_participant_macro", {})
            if not cpm or cpm.get("MAE") is None:
                cpm = eval_dict.get("class_macro_avg", {}) or eval_dict.get("class_macro", {})
                if not cpm:
                    cpm = eval_dict.get("macro_metrics", {}).get("class_macro", {})
            
            macro = eval_dict.get("macro_avg", {})
            pooled = eval_dict.get("pooled", {})
            
            mae_val = cpm.get("MAE") if (cpm and cpm.get("MAE") is not None) else (macro.get("MAE") or pooled.get("MAE", 0.0))
            rmse_val = cpm.get("RMSE") if (cpm and cpm.get("RMSE") is not None) else (macro.get("RMSE") or pooled.get("RMSE", 0.0))
            
            modalities.append(name_map.get(key, key.replace("_", " ").title()))
            maes.append(mae_val)
            rmses.append(rmse_val)
            
        # Fallback to other runs if standard ones are not present
        if not modalities:
            for key, run_data in data.items():
                if not isinstance(run_data, dict) or "evaluation" not in run_data:
                    continue
                eval_dict = run_data["evaluation"]
                cpm = eval_dict.get("macro_metrics", {}).get("class_participant_macro", {}) or eval_dict.get("class_participant_macro", {})
                if not cpm or cpm.get("MAE") is None:
                    cpm = eval_dict.get("class_macro_avg", {}) or eval_dict.get("class_macro", {})
                    if not cpm:
                        cpm = eval_dict.get("macro_metrics", {}).get("class_macro", {})
                
                macro = eval_dict.get("macro_avg", {})
                pooled = eval_dict.get("pooled", {})
                
                mae_val = cpm.get("MAE") if (cpm and cpm.get("MAE") is not None) else (macro.get("MAE") or pooled.get("MAE", 0.0))
                rmse_val = cpm.get("RMSE") if (cpm and cpm.get("RMSE") is not None) else (macro.get("RMSE") or pooled.get("RMSE", 0.0))
                
                modalities.append(name_map.get(key, key.replace("_", " ").title()))
                maes.append(mae_val)
                rmses.append(rmse_val)
                
        if not modalities:
            print("  [Warning] No modality data found with pooled metrics. Skipping.")
            return False
            
        fig, ax = plt.subplots()
        
        x = np.arange(len(modalities))
        width = 0.35
        
        rects1 = ax.bar(x - width/2, maes, width, label='MAE', color=ThesisStyle.COLOR_MAE, alpha=0.85, edgecolor=ThesisStyle.COLOR_MAE, linewidth=0.5)
        rects2 = ax.bar(x + width/2, rmses, width, label='RMSE', color=ThesisStyle.COLOR_RMSE, alpha=0.85, edgecolor=ThesisStyle.COLOR_RMSE, linewidth=0.5)
        
        # Annotate
        for rect in rects1:
            h = rect.get_height()
            ax.annotate(f"{h:.3f}",
                        xy=(rect.get_x() + rect.get_width() / 2, h),
                        xytext=(0, 3), textcoords="offset points",
                        ha='center', va='bottom', fontsize=plt.rcParams['font.size'] - 2.0, color=ThesisStyle.COLOR_MAE, weight='semibold')
                        
        for rect in rects2:
            h = rect.get_height()
            ax.annotate(f"{h:.3f}",
                        xy=(rect.get_x() + rect.get_width() / 2, h),
                        xytext=(0, 3), textcoords="offset points",
                        ha='center', va='bottom', fontsize=plt.rcParams['font.size'] - 2.0, color=ThesisStyle.COLOR_RMSE, weight='semibold')
        
        ax.set_ylabel("Error (kg)", labelpad=8)
        ax.set_xlabel("Sensor Modality Configuration", labelpad=8)
        title = get_plot_title("ablation", "Sensor Modality Ablation Study")
        ThesisStyle.set_title(ax, title)
        ax.set_xticks(x)
        ax.set_xticklabels(modalities)
        ax.set_ylim(0, max(rmses) * 1.22)
        
        ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("modality_comparison")
class ModalityComparisonPlotter:
    """Generates comparison plots across EMG+IMU, EMG only, and IMU only modalities."""
    
    def plot(self, data, output_path, layout_width):
        is_multi = False
        if "meta" not in data:
            sub_runs = [k for k, v in data.items() if isinstance(v, dict) and "meta" in v]
            if sub_runs:
                is_multi = True
                
        if not is_multi:
            print("  [Warning] Modality comparison plotter requires a multi-modality ablation run dataset at root. Skipping.")
            return False
            
        target_dir = output_path.parent
        target_dir.mkdir(parents=True, exist_ok=True)
        
        required_keys = ["all", "emg_only", "imu_only"]
        for key in required_keys:
            if key not in data or not isinstance(data[key], dict):
                print(f"  [Warning] Missing modality key '{key}' in run data. Skipping modality comparison.")
                return False
                
        prefix = output_path.name.replace("modality_comparison_plot", "")
        
        # 1. Bar Chart: metrics_comparison
        self._plot_metrics_comparison(data, target_dir / f"{prefix}metrics_comparison", layout_width)
        
        # 2. Line Charts: seqlen_mae_comparison and seqlen_rmse_comparison
        self._plot_seqlen_comparison(data, target_dir, prefix, layout_width)
        
        # 3. Grouped Bar: participant_mae_comparison and participant_rmse_comparison
        self._plot_participant_comparison(data, target_dir, prefix, layout_width)
        
        # 3.5 Grouped Bar: participant_gap_comparison and participant_ratio_comparison
        self._plot_participant_gap_comparison(data, target_dir, prefix, layout_width)
        self._plot_participant_ratio_comparison(data, target_dir, prefix, layout_width)
        
        # 4. Boxplot: error_distribution_comparison
        self._plot_error_distribution_comparison(data, target_dir / f"{prefix}error_distribution_comparison", layout_width)
        
        # 5. Line Charts: weight_mae_comparison and weight_rmse_comparison
        self._plot_weight_error_comparison(data, target_dir, prefix, layout_width)
        
        return True

    def _plot_metrics_comparison(self, data, output_path, layout_width):
        ThesisStyle.apply(layout_width)
        
        modalities = ["EMG + IMU\n(Sensor-Fused)", "EMG-only", "IMU-only"]
        keys = ["all", "emg_only", "imu_only"]
        
        maes = []
        rmses = []
        r2s = []
        gaps = []
        for k in keys:
            eval_dict = data[k].get("evaluation", {})
            cpm = eval_dict.get("macro_metrics", {}).get("class_participant_macro", {}) or eval_dict.get("class_participant_macro", {})
            macro = eval_dict.get("macro_avg", {})
            mae_val = cpm.get("MAE") if cpm.get("MAE") is not None else macro.get("MAE")
            rmse_val = cpm.get("RMSE") if cpm.get("RMSE") is not None else macro.get("RMSE")
            r2_val = cpm.get("R2") if cpm.get("R2") is not None else macro.get("R2")
            
            if mae_val is not None:
                maes.append(mae_val)
                rmses.append(rmse_val if rmse_val is not None else 0.0)
                r2s.append(r2_val if r2_val is not None else 0.0)
                
                gap_val = macro.get("Generalization Gap (MAE)")
                if gap_val is None:
                    per_fold = eval_dict.get("per_fold", [])
                    if isinstance(per_fold, list) and per_fold:
                        fold_gaps = [row.get("Generalization Gap (MAE)", None) for row in per_fold if isinstance(row, dict)]
                        fold_gaps = [g for g in fold_gaps if g is not None]
                    else:
                        fold_gaps = []
                    gap_val = float(np.mean(fold_gaps)) if fold_gaps else 0.0
                gaps.append(float(gap_val))
        
        fig, ax1 = plt.subplots()
        x = np.arange(len(modalities))
        width = 0.20
        offsets = [-1.5 * width, -0.5 * width, 0.5 * width, 1.5 * width]
        
        rects1 = ax1.bar(x + offsets[0], maes, width, label='MAE', color=ThesisStyle.COLOR_MAE, alpha=0.85, edgecolor=ThesisStyle.COLOR_MAE, linewidth=0.5)
        rects2 = ax1.bar(x + offsets[1], rmses, width, label='RMSE', color=ThesisStyle.COLOR_RMSE, alpha=0.85, edgecolor=ThesisStyle.COLOR_RMSE, linewidth=0.5)
        rects4 = ax1.bar(x + offsets[2], gaps, width, label='Gen. Gap', color=ThesisStyle.COLOR_GAP, alpha=0.85, edgecolor=ThesisStyle.COLOR_GAP, linewidth=0.5)
        
        ax2 = ax1.twinx()
        rects3 = ax2.bar(x + offsets[3], r2s, width, label=r'$\text{R}^2$', color=ThesisStyle.COLOR_R2, alpha=0.85, edgecolor=ThesisStyle.COLOR_R2, linewidth=0.5)
        
        for rect in rects1:
            h = rect.get_height()
            ax1.annotate(f"{h:.3f}",
                        xy=(rect.get_x() + rect.get_width() / 2, h),
                        xytext=(0, 3), textcoords="offset points",
                        ha='center', va='bottom', fontsize=plt.rcParams['font.size'] - 2.0, color=ThesisStyle.COLOR_MAE, weight='semibold')
                        
        for rect in rects2:
            h = rect.get_height()
            ax1.annotate(f"{h:.3f}",
                        xy=(rect.get_x() + rect.get_width() / 2, h),
                        xytext=(0, 3), textcoords="offset points",
                        ha='center', va='bottom', fontsize=plt.rcParams['font.size'] - 2.0, color=ThesisStyle.COLOR_RMSE, weight='semibold')
        
        for rect in rects4:
            h = rect.get_height()
            va = 'bottom' if h >= 0 else 'top'
            offset = 3 if h >= 0 else -3
            ax1.annotate(f"{h:.3f}",
                        xy=(rect.get_x() + rect.get_width() / 2, h),
                        xytext=(0, offset), textcoords="offset points",
                        ha='center', va=va, fontsize=plt.rcParams['font.size'] - 2.0, color=ThesisStyle.COLOR_GAP, weight='semibold')
                        
        for rect in rects3:
            h = rect.get_height()
            ax2.annotate(f"{h:.3f}",
                        xy=(rect.get_x() + rect.get_width() / 2, h),
                        xytext=(0, 3), textcoords="offset points",
                        ha='center', va='bottom', fontsize=plt.rcParams['font.size'] - 2.0, color=ThesisStyle.COLOR_R2, weight='semibold')
                        
        # Zero baseline for generalization gap (can be negative)
        ax1.axhline(y=0, color=ThesisStyle.COLOR_UNITY, linewidth=0.8, linestyle='--', alpha=0.7)
        
        ax1.set_ylabel("Error / Gen. Gap (kg)", labelpad=8)
        ax1.set_xlabel("Sensor Modality Configuration", labelpad=8)
        
        ax2.set_ylabel(r"$\text{R}^2$ Score", labelpad=8, color=ThesisStyle.COLOR_R2)
        ax2.tick_params(axis='y', labelcolor=ThesisStyle.COLOR_R2)
        
        title = get_plot_title("modality_metrics", "Modality Performance Comparison")
        ThesisStyle.set_title(ax1, title)
        
        ax1.set_xticks(x)
        ax1.set_xticklabels(modalities)
        
        # Y-axis: accommodate negative gap values
        left_min = min(min(gaps), 0) * 1.3
        left_max = max(rmses) * 1.30
        ax1.set_ylim(left_min, left_max)
        ax2.set_ylim(0, 1.05)
        
        ax1.grid(True, which='both', axis='y', linestyle='--', alpha=0.5)
        ax2.grid(False)
        
        ax1.spines['top'].set_visible(False)
        ax1.spines['right'].set_visible(True)
        ax1.spines['left'].set_visible(True)
        ax2.spines['top'].set_visible(False)
        ax2.spines['right'].set_visible(True)
        ax2.spines['left'].set_visible(True)
        
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper left', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)

    def _plot_seqlen_comparison(self, data, output_dir, prefix, layout_width):
        keys = ["all", "emg_only", "imu_only"]
        labels = ["EMG + IMU", "EMG-only", "IMU-only"]
        colors = [ThesisStyle.COLOR_FUSION, ThesisStyle.COLOR_EMG, ThesisStyle.COLOR_IMU]
        markers = ["o", "s", "^"]
        
        # Get segment counts from 'all' configuration (same across configurations)
        run_data_all = data["all"]
        per_seqlen_all = run_data_all.get("evaluation", {}).get("per_seqlen")
        if not per_seqlen_all:
            per_seqlen_all = run_data_all.get("evaluation", {}).get("per_duration", [])
        filtered_all = [row for row in per_seqlen_all if int(row.get('Count', 0)) >= 50]
        
        times_all = []
        counts_all = []
        for row in filtered_all:
            t = float(row['TimeAtPrediction'].rstrip('s'))
            if t > 3.0:
                continue
            times_all.append(t)
            counts_all.append(int(row['Count']))
            
        # 1. Generate MAE Plot
        ThesisStyle.apply(layout_width)
        fig1, ax1 = plt.subplots()
        
        # Segment counts on secondary axis
        ax1_twin = ax1.twinx()
        if times_all:
            if len(times_all) > 1:
                bar_width = 0.5 * np.min(np.diff(times_all))
            else:
                bar_width = 0.1
            ax1_twin.bar(times_all, counts_all, width=bar_width, color=ThesisStyle.RHO_LIGHT_BLUE, alpha=0.6,
                         edgecolor=ThesisStyle.COLOR_UNITY, linewidth=0.5, label='Segment Count', zorder=1)
            ax1_twin.set_ylabel("Segment Count", color=ThesisStyle.NEUTRAL_GRAY)
            ax1_twin.tick_params(axis='y', labelcolor=ThesisStyle.NEUTRAL_GRAY)
            ax1_twin.set_ylim(0, max(counts_all) * 3.0)
            ax1_twin.spines['top'].set_visible(False)
            ax1_twin.spines['right'].set_visible(True)
            ax1_twin.spines['left'].set_visible(True)
            ax1_twin.grid(False)
            
        for k, label, color, marker in zip(keys, labels, colors, markers):
            run_data = data[k]
            per_seqlen = run_data.get("evaluation", {}).get("per_seqlen")
            if not per_seqlen:
                per_seqlen = run_data.get("evaluation", {}).get("per_duration", [])
            filtered = [row for row in per_seqlen if int(row.get('Count', 0)) >= 50]
            
            times = []
            maes = []
            for row in filtered:
                t = float(row['TimeAtPrediction'].rstrip('s'))
                if t > 3.0:
                    continue
                times.append(t)
                maes.append(float(row['MAE']))
                
            if times:
                ax1.plot(times, maes, marker=marker, color=color, label=label, linewidth=1.5, markersize=4.5, zorder=3)
                
        ax1.set_xlabel("Segment Length (s)", labelpad=8)
        ax1.set_ylabel("MAE (kg)", labelpad=8)
        
        title_mae = get_plot_title("modality_seqlen_mae", "MAE by Segment Length and Modality")
        ThesisStyle.set_title(ax1, title_mae)
        ax1.set_ylim(bottom=0)
        ax1.grid(True, linestyle='--', alpha=0.5)
        ThesisStyle.style_ax(ax1)
        
        # Combine legends cleanly
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax1_twin.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        ax1.set_zorder(ax1_twin.get_zorder() + 1)
        ax1.patch.set_visible(False)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig1, output_dir / f"{prefix}seqlen_mae_comparison")
        plt.close(fig1)

        # 2. Generate RMSE Plot
        ThesisStyle.apply(layout_width)
        fig2, ax2 = plt.subplots()
        
        # Segment counts on secondary axis
        ax2_twin = ax2.twinx()
        if times_all:
            if len(times_all) > 1:
                bar_width = 0.5 * np.min(np.diff(times_all))
            else:
                bar_width = 0.1
            ax2_twin.bar(times_all, counts_all, width=bar_width, color=ThesisStyle.RHO_LIGHT_BLUE, alpha=0.6,
                         edgecolor=ThesisStyle.COLOR_UNITY, linewidth=0.5, label='Segment Count', zorder=1)
            ax2_twin.set_ylabel("Segment Count", color=ThesisStyle.NEUTRAL_GRAY)
            ax2_twin.tick_params(axis='y', labelcolor=ThesisStyle.NEUTRAL_GRAY)
            ax2_twin.set_ylim(0, max(counts_all) * 3.0)
            ax2_twin.spines['top'].set_visible(False)
            ax2_twin.spines['right'].set_visible(True)
            ax2_twin.spines['left'].set_visible(True)
            ax2_twin.grid(False)
            
        for k, label, color, marker in zip(keys, labels, colors, markers):
            run_data = data[k]
            per_seqlen = run_data.get("evaluation", {}).get("per_seqlen")
            if not per_seqlen:
                per_seqlen = run_data.get("evaluation", {}).get("per_duration", [])
            filtered = [row for row in per_seqlen if int(row.get('Count', 0)) >= 50]
            
            times = []
            rmses = []
            for row in filtered:
                t = float(row['TimeAtPrediction'].rstrip('s'))
                if t > 3.0:
                    continue
                times.append(t)
                rmses.append(float(row['RMSE']))
                
            if times:
                ax2.plot(times, rmses, marker=marker, color=color, label=label, linewidth=1.5, markersize=4.5, linestyle='--', zorder=3)
                
        ax2.set_xlabel("Segment Length (s)", labelpad=8)
        ax2.set_ylabel("RMSE (kg)", labelpad=8)
        
        title_rmse = get_plot_title("modality_seqlen_rmse", "RMSE by Segment Length and Modality")
        ThesisStyle.set_title(ax2, title_rmse)
        ax2.set_ylim(bottom=0)
        ax2.grid(True, linestyle='--', alpha=0.5)
        ThesisStyle.style_ax(ax2)
        
        # Combine legends cleanly
        lines1, labels1 = ax2.get_legend_handles_labels()
        lines2, labels2 = ax2_twin.get_legend_handles_labels()
        ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        ax2.set_zorder(ax2_twin.get_zorder() + 1)
        ax2.patch.set_visible(False)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig2, output_dir / f"{prefix}seqlen_rmse_comparison")
        plt.close(fig2)


    def _plot_participant_comparison(self, data, output_dir, prefix, layout_width):
        # Gather metrics for each participant
        part_maes = {}
        part_rmses = {}
        
        for k in ["all", "emg_only", "imu_only"]:
            per_part = data[k].get("evaluation", {}).get("per_participant", [])
            for row in per_part:
                p_id = row["Participant"]
                if p_id not in part_maes:
                    part_maes[p_id] = {}
                if p_id not in part_rmses:
                    part_rmses[p_id] = {}
                part_maes[p_id][k] = float(row["MAE"])
                part_rmses[p_id][k] = float(row["RMSE"])
                
        # 1. MAE Comparison Plot
        ThesisStyle.apply(layout_width)
        sorted_participants_mae = sorted(part_maes.keys(), key=lambda p: part_maes[p].get("all", 999.0))
        
        all_vals_mae = [part_maes[p].get("all", 0.0) for p in sorted_participants_mae]
        emg_vals_mae = [part_maes[p].get("emg_only", 0.0) for p in sorted_participants_mae]
        imu_vals_mae = [part_maes[p].get("imu_only", 0.0) for p in sorted_participants_mae]
        
        fig1, ax1 = plt.subplots()
        x = np.arange(len(sorted_participants_mae))
        width = 0.25
        
        rects1 = ax1.bar(x - width, all_vals_mae, width, label='EMG + IMU', color=ThesisStyle.COLOR_FUSION, alpha=0.85, edgecolor=ThesisStyle.COLOR_FUSION, linewidth=0.5)
        rects2 = ax1.bar(x, emg_vals_mae, width, label='EMG-only', color=ThesisStyle.COLOR_EMG, alpha=0.85, edgecolor=ThesisStyle.COLOR_EMG, linewidth=0.5)
        
        # Clip IMU only values at 1.5
        imu_plot_vals_mae = [min(v, 1.5) for v in imu_vals_mae]
        rects3 = ax1.bar(x + width, imu_plot_vals_mae, width, label='IMU-only', color=ThesisStyle.COLOR_IMU, alpha=0.85, edgecolor=ThesisStyle.COLOR_IMU, linewidth=0.5)
        
        # Annotate clipped IMU bars
        for idx, val in enumerate(imu_vals_mae):
            if val > 1.5:
                ax1.text(x[idx] + width, 1.51, f"{val:.2f}", ha='center', va='bottom',
                         color=ThesisStyle.COLOR_IMU, fontweight='bold', clip_on=False,
                         fontsize=plt.rcParams['font.size'] - 2.5)
                 
        eval_all = data["all"].get("evaluation", {})
        eval_emg = data["emg_only"].get("evaluation", {})
        eval_imu = data["imu_only"].get("evaluation", {})
        
        def get_cpm_mae(eval_dict):
            cpm = eval_dict.get("macro_metrics", {}).get("class_participant_macro", {}) or eval_dict.get("class_participant_macro", {})
            if cpm.get("MAE") is not None:
                return cpm["MAE"]
            macro = eval_dict.get("macro_avg", {})
            if macro.get("MAE") is not None:
                return macro["MAE"]
            return eval_dict.get("pooled", {}).get("MAE", 0.0)
            
        mean_all_mae = get_cpm_mae(eval_all)
        mean_emg_mae = get_cpm_mae(eval_emg)
        mean_imu_mae = get_cpm_mae(eval_imu)
        
        ax1.axhline(mean_all_mae, color=ThesisStyle.COLOR_FUSION, linestyle='--', linewidth=1.0, alpha=0.7)
        ax1.axhline(mean_emg_mae, color=ThesisStyle.COLOR_EMG, linestyle='--', linewidth=1.0, alpha=0.7)
        ax1.axhline(mean_imu_mae, color=ThesisStyle.COLOR_IMU, linestyle='--', linewidth=1.0, alpha=0.7)
        
        # Add labels next to the dashed lines on the right side of the plot
        ax1.text(0.98, mean_all_mae, f"Mean: {mean_all_mae:.3f}", transform=ax1.get_yaxis_transform(),
                 color=ThesisStyle.COLOR_FUSION, va='bottom', ha='right',
                 fontsize=plt.rcParams['font.size'] - 2.5, fontweight='semibold',
                 bbox=dict(facecolor='white', alpha=0.85, edgecolor='none', pad=1.0, boxstyle='round,pad=0.15'))
                 
        ax1.text(0.98, mean_emg_mae, f"Mean: {mean_emg_mae:.3f}", transform=ax1.get_yaxis_transform(),
                 color=ThesisStyle.COLOR_EMG, va='top', ha='right',
                 fontsize=plt.rcParams['font.size'] - 2.5, fontweight='semibold',
                 bbox=dict(facecolor='white', alpha=0.85, edgecolor='none', pad=1.0, boxstyle='round,pad=0.15'))
                 
        ax1.text(0.98, mean_imu_mae, f"Mean: {mean_imu_mae:.3f}", transform=ax1.get_yaxis_transform(),
                 color=ThesisStyle.COLOR_IMU, va='bottom', ha='right',
                 fontsize=plt.rcParams['font.size'] - 2.5, fontweight='semibold',
                 bbox=dict(facecolor='white', alpha=0.85, edgecolor='none', pad=1.0, boxstyle='round,pad=0.15'))
        
        ax1.set_ylabel('MAE (kg)', labelpad=8)
        ax1.set_xlabel('Unseen Participant Left Out', labelpad=8)
        
        title_mae = get_plot_title("modality_participant_mae", "MAE per Participant by Modality")
        ThesisStyle.set_title(ax1, title_mae)
        
        ax1.set_xticks(x)
        ax1.set_xticklabels(sorted_participants_mae, rotation=45, ha='right')
        
        ax1.set_ylim(0, 1.5)
        
        ax1.legend(loc='upper left', ncol=1, frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        ThesisStyle.save_figure(fig1, output_dir / f"{prefix}participant_mae_comparison")
        plt.close(fig1)

        # 2. RMSE Comparison Plot
        ThesisStyle.apply(layout_width)
        sorted_participants_rmse = sorted(part_rmses.keys(), key=lambda p: part_rmses[p].get("all", 999.0))
        
        all_vals_rmse = [part_rmses[p].get("all", 0.0) for p in sorted_participants_rmse]
        emg_vals_rmse = [part_rmses[p].get("emg_only", 0.0) for p in sorted_participants_rmse]
        imu_vals_rmse = [part_rmses[p].get("imu_only", 0.0) for p in sorted_participants_rmse]
        
        fig2, ax2 = plt.subplots()
        x = np.arange(len(sorted_participants_rmse))
        width = 0.25
        
        rects1 = ax2.bar(x - width, all_vals_rmse, width, label='EMG + IMU', color=ThesisStyle.COLOR_FUSION, alpha=0.85, edgecolor=ThesisStyle.COLOR_FUSION, linewidth=0.5)
        rects2 = ax2.bar(x, emg_vals_rmse, width, label='EMG-only', color=ThesisStyle.COLOR_EMG, alpha=0.85, edgecolor=ThesisStyle.COLOR_EMG, linewidth=0.5)
        
        # Clip IMU only values at 2.5
        imu_plot_vals_rmse = [min(v, 2.5) for v in imu_vals_rmse]
        rects3 = ax2.bar(x + width, imu_plot_vals_rmse, width, label='IMU-only', color=ThesisStyle.COLOR_IMU, alpha=0.85, edgecolor=ThesisStyle.COLOR_IMU, linewidth=0.5)
        
        # Annotate clipped IMU bars
        for idx, val in enumerate(imu_vals_rmse):
            if val > 2.5:
                ax2.text(x[idx] + width, 2.51, f"{val:.2f}", ha='center', va='bottom',
                         color=ThesisStyle.COLOR_IMU, fontweight='bold', clip_on=False,
                         fontsize=plt.rcParams['font.size'] - 2.5)
                 
        def get_cpm_rmse(eval_dict):
            cpm = eval_dict.get("macro_metrics", {}).get("class_participant_macro", {}) or eval_dict.get("class_participant_macro", {})
            if cpm.get("RMSE") is not None:
                return cpm["RMSE"]
            macro = eval_dict.get("macro_avg", {})
            if macro.get("RMSE") is not None:
                return macro["RMSE"]
            return eval_dict.get("pooled", {}).get("RMSE", 0.0)
            
        mean_all_rmse = get_cpm_rmse(eval_all)
        mean_emg_rmse = get_cpm_rmse(eval_emg)
        mean_imu_rmse = get_cpm_rmse(eval_imu)
        
        ax2.axhline(mean_all_rmse, color=ThesisStyle.COLOR_FUSION, linestyle='--', linewidth=1.0, alpha=0.7)
        ax2.axhline(mean_emg_rmse, color=ThesisStyle.COLOR_EMG, linestyle='--', linewidth=1.0, alpha=0.7)
        ax2.axhline(mean_imu_rmse, color=ThesisStyle.COLOR_IMU, linestyle='--', linewidth=1.0, alpha=0.7)
        
        # Add labels next to the dashed lines on the right side of the plot
        ax2.text(0.98, mean_all_rmse, f"Mean: {mean_all_rmse:.3f}", transform=ax2.get_yaxis_transform(),
                 color=ThesisStyle.COLOR_FUSION, va='bottom', ha='right',
                 fontsize=plt.rcParams['font.size'] - 2.5, fontweight='semibold',
                 bbox=dict(facecolor='white', alpha=0.85, edgecolor='none', pad=1.0, boxstyle='round,pad=0.15'))
                 
        ax2.text(0.98, mean_emg_rmse, f"Mean: {mean_emg_rmse:.3f}", transform=ax2.get_yaxis_transform(),
                 color=ThesisStyle.COLOR_EMG, va='top', ha='right',
                 fontsize=plt.rcParams['font.size'] - 2.5, fontweight='semibold',
                 bbox=dict(facecolor='white', alpha=0.85, edgecolor='none', pad=1.0, boxstyle='round,pad=0.15'))
                 
        ax2.text(0.98, mean_imu_rmse, f"Mean: {mean_imu_rmse:.3f}", transform=ax2.get_yaxis_transform(),
                 color=ThesisStyle.COLOR_IMU, va='bottom', ha='right',
                 fontsize=plt.rcParams['font.size'] - 2.5, fontweight='semibold',
                 bbox=dict(facecolor='white', alpha=0.85, edgecolor='none', pad=1.0, boxstyle='round,pad=0.15'))
        
        ax2.set_ylabel('RMSE (kg)', labelpad=8)
        ax2.set_xlabel('Unseen Participant Left Out', labelpad=8)
        
        title_rmse = get_plot_title("modality_participant_rmse", "RMSE per Participant by Modality")
        ThesisStyle.set_title(ax2, title_rmse)
        
        ax2.set_xticks(x)
        ax2.set_xticklabels(sorted_participants_rmse, rotation=45, ha='right')
        
        ax2.set_ylim(0, 2.5)
        
        ax2.legend(loc='upper left', ncol=1, frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        ThesisStyle.save_figure(fig2, output_dir / f"{prefix}participant_rmse_comparison")
        plt.close(fig2)

    def _plot_participant_gap_comparison(self, data, output_dir, prefix, layout_width):
        ThesisStyle.apply(layout_width)
        
        part_gaps = {}
        for k in ["all", "emg_only", "imu_only"]:
            per_part = data[k].get("evaluation", {}).get("per_participant", [])
            per_fold = data[k].get("evaluation", {}).get("per_fold", [])
            for idx, part_row in enumerate(per_part):
                p_id = part_row["Participant"]
                gap = 0.0
                if idx < len(per_fold):
                    fold_row = per_fold[idx]
                    gap = float(fold_row.get("Generalization Gap (MAE)", 0.0))
                if p_id not in part_gaps:
                    part_gaps[p_id] = {}
                part_gaps[p_id][k] = gap
                
        sorted_participants = sorted(part_gaps.keys(), key=lambda p: part_gaps[p].get("all", 999.0))
        
        all_vals = [part_gaps[p].get("all", 0.0) for p in sorted_participants]
        emg_vals = [part_gaps[p].get("emg_only", 0.0) for p in sorted_participants]
        imu_vals = [part_gaps[p].get("imu_only", 0.0) for p in sorted_participants]
        
        fig, ax = plt.subplots()
        
        x = np.arange(len(sorted_participants))
        width = 0.25
        
        rects1 = ax.bar(x - width, all_vals, width, label='EMG + IMU', color=ThesisStyle.COLOR_FUSION, alpha=0.85, edgecolor=ThesisStyle.COLOR_FUSION, linewidth=0.5)
        rects2 = ax.bar(x, emg_vals, width, label='EMG-only', color=ThesisStyle.COLOR_EMG, alpha=0.85, edgecolor=ThesisStyle.COLOR_EMG, linewidth=0.5)
        
        # Clip IMU only values at 1.0
        imu_plot_vals = [min(v, 1.0) for v in imu_vals]
        rects3 = ax.bar(x + width, imu_plot_vals, width, label='IMU-only', color=ThesisStyle.COLOR_IMU, alpha=0.85, edgecolor=ThesisStyle.COLOR_IMU, linewidth=0.5)
        
        # Annotate clipped IMU bars
        for idx, val in enumerate(imu_vals):
            if val > 1.0:
                ax.text(x[idx] + width, 1.01, f"{val:.2f}", ha='center', va='bottom',
                        color=ThesisStyle.COLOR_IMU, fontweight='bold', clip_on=False,
                        fontsize=plt.rcParams['font.size'] - 2.5)
        
        mean_all = np.mean(all_vals)
        mean_emg = np.mean(emg_vals)
        mean_imu = np.mean(imu_vals)
        
        # Zero baseline reference line
        ax.axhline(0, color=ThesisStyle.COLOR_UNITY, linestyle='-', linewidth=0.8, alpha=0.5)
        
        ax.axhline(mean_all, color=ThesisStyle.COLOR_FUSION, linestyle='--', linewidth=1.0, alpha=0.7, label='Mean EMG + IMU')
        ax.axhline(mean_emg, color=ThesisStyle.COLOR_EMG, linestyle='--', linewidth=1.0, alpha=0.7, label='Mean EMG-only')
        ax.axhline(mean_imu, color=ThesisStyle.COLOR_IMU, linestyle='--', linewidth=1.0, alpha=0.7, label='Mean IMU-only')
        
        ax.set_ylabel('Generalization Gap (MAE) (kg)', labelpad=8)
        ax.set_xlabel('Unseen Participant Left Out', labelpad=8)
        
        title = get_plot_title("modality_participant_gap", "Generalization Gap per Participant by Modality")
        ThesisStyle.set_title(ax, title)
        
        ax.set_xticks(x)
        ax.set_xticklabels(sorted_participants, rotation=45, ha='right')
        
        min_val = min(min(all_vals), min(emg_vals), min(imu_vals))
        y_bottom = min_val - 0.05 if min_val < 0 else -0.05
        ax.set_ylim(y_bottom, 1.0)
        
        ax.legend(loc='upper left', ncol=2, frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_dir / f"{prefix}participant_gap_comparison")
        plt.close(fig)

    def _plot_participant_ratio_comparison(self, data, output_dir, prefix, layout_width):
        ThesisStyle.apply(layout_width)
        
        part_ratios = {}
        for k in ["all", "emg_only", "imu_only"]:
            per_part = data[k].get("evaluation", {}).get("per_participant", [])
            per_fold = data[k].get("evaluation", {}).get("per_fold", [])
            for idx, part_row in enumerate(per_part):
                p_id = part_row["Participant"]
                ratio = 0.0
                if idx < len(per_fold):
                    fold_row = per_fold[idx]
                    ratio = float(fold_row.get("Overfit Ratio (RMSE)", 0.0))
                if p_id not in part_ratios:
                    part_ratios[p_id] = {}
                part_ratios[p_id][k] = ratio
                
        sorted_participants = sorted(part_ratios.keys(), key=lambda p: part_ratios[p].get("all", 999.0))
        
        all_vals = [part_ratios[p].get("all", 0.0) for p in sorted_participants]
        emg_vals = [part_ratios[p].get("emg_only", 0.0) for p in sorted_participants]
        imu_vals = [part_ratios[p].get("imu_only", 0.0) for p in sorted_participants]
        
        fig, ax = plt.subplots()
        
        x = np.arange(len(sorted_participants))
        width = 0.25
        
        rects1 = ax.bar(x - width, all_vals, width, label='EMG + IMU', color=ThesisStyle.COLOR_FUSION, alpha=0.85, edgecolor=ThesisStyle.COLOR_FUSION, linewidth=0.5)
        rects2 = ax.bar(x, emg_vals, width, label='EMG-only', color=ThesisStyle.COLOR_EMG, alpha=0.85, edgecolor=ThesisStyle.COLOR_EMG, linewidth=0.5)
        
        # Clip IMU only values at 3.0
        imu_plot_vals = [min(v, 3.0) for v in imu_vals]
        rects3 = ax.bar(x + width, imu_plot_vals, width, label='IMU-only', color=ThesisStyle.COLOR_IMU, alpha=0.85, edgecolor=ThesisStyle.COLOR_IMU, linewidth=0.5)
        
        # Annotate clipped IMU bars
        for idx, val in enumerate(imu_vals):
            if val > 3.0:
                ax.text(x[idx] + width, 3.01, f"{val:.2f}", ha='center', va='bottom',
                        color=ThesisStyle.COLOR_IMU, fontweight='bold', clip_on=False,
                        fontsize=plt.rcParams['font.size'] - 2.5)
        
        mean_all = np.mean(all_vals)
        mean_emg = np.mean(emg_vals)
        mean_imu = np.mean(imu_vals)
        
        # Baseline reference at 1.0 (perfect generalization)
        ax.axhline(1.0, color=ThesisStyle.COLOR_UNITY, linestyle=':', linewidth=0.8, alpha=0.5)
        
        ax.axhline(mean_all, color=ThesisStyle.COLOR_FUSION, linestyle='--', linewidth=1.0, alpha=0.7, label='Mean EMG + IMU')
        ax.axhline(mean_emg, color=ThesisStyle.COLOR_EMG, linestyle='--', linewidth=1.0, alpha=0.7, label='Mean EMG-only')
        ax.axhline(mean_imu, color=ThesisStyle.COLOR_IMU, linestyle='--', linewidth=1.0, alpha=0.7, label='Mean IMU-only')
        
        ax.set_ylabel('Generalization Ratio (RMSE)', labelpad=8)
        ax.set_xlabel('Unseen Participant Left Out', labelpad=8)
        
        title = get_plot_title("modality_participant_ratio", "Generalization Ratio per Participant by Modality")
        ThesisStyle.set_title(ax, title)
        
        ax.set_xticks(x)
        ax.set_xticklabels(sorted_participants, rotation=45, ha='right')
        
        ax.set_ylim(0, 3.0)
        
        ax.legend(loc='upper left', ncol=2, frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_dir / f"{prefix}participant_ratio_comparison")
        plt.close(fig)

    def _plot_error_distribution_comparison(self, data, output_path, layout_width):
        ThesisStyle.apply(layout_width)
        
        keys = ["all", "emg_only", "imu_only"]
        labels = ["EMG + IMU", "EMG-only", "IMU-only"]
        
        error_groups = []
        for k in keys:
            preds = data[k].get("predictions", {})
            y_true = np.array(preds.get("y_true", []))
            y_pred = np.maximum(0.0, np.array(preds.get("y_pred", [])))
            if len(y_true) > 0 and len(y_pred) > 0:
                abs_errors = np.abs(y_pred - y_true)
                error_groups.append(abs_errors)
            else:
                error_groups.append(np.array([]))
                
        fig, ax = plt.subplots()
        
        border_colors = [ThesisStyle.COLOR_FUSION, ThesisStyle.COLOR_EMG, ThesisStyle.COLOR_IMU]
        fill_colors = ["#EBF7FA", "#FAF0F5", "#F0F7F4"]
        median_colors = ["#004D60", "#551133", "#115522"]
        
        first_outlier = True
        positions = [1, 2, 3]
        
        for i, (preds_err, pos) in enumerate(zip(error_groups, positions)):
            if len(preds_err) > 0:
                q1 = np.percentile(preds_err, 25)
                q3 = np.percentile(preds_err, 75)
                iqr = q3 - q1
                outliers = preds_err[(preds_err < q1 - 1.5 * iqr) | (preds_err > q3 + 1.5 * iqr)]
                
                if len(outliers) > 0:
                    if len(outliers) > 100:
                        np.random.seed(42)
                        outliers = np.random.choice(outliers, size=100, replace=False)
                    label = "Outliers" if first_outlier else ""
                    ax.scatter(np.full_like(outliers, pos), outliers,
                               alpha=0.35, s=12, color='#7F8C8D', edgecolors='none',
                               zorder=2, label=label)
                    first_outlier = False
                    
        bp = ax.boxplot(error_groups, positions=positions, widths=0.4,
                        patch_artist=True, showfliers=False, showcaps=False,
                        whiskerprops={'color': '#475569', 'linewidth': 1.0},
                        zorder=3)
        
        for idx, (box, fill_c, border_c) in enumerate(zip(bp['boxes'], fill_colors, border_colors)):
            box.set_facecolor(fill_c)
            box.set_edgecolor(border_c)
            box.set_linewidth(1.2)
            box.set_alpha(0.85)
            
        for idx, (median, med_c) in enumerate(zip(bp['medians'], median_colors)):
            median.set_color(med_c)
            median.set_linewidth(1.5)
            if idx == 0:
                median.set_label('Median')
                
        ax.set_ylabel("Absolute Prediction Error (kg)", labelpad=8)
        ax.set_xlabel("Sensor Modality Configuration", labelpad=8)
        
        title = get_plot_title("modality_errors", "Overall Error Distributions by Modality")
        ThesisStyle.set_title(ax, title)
        
        ax.set_xticks(positions)
        ax.set_xticklabels(labels)
        
        ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)

    def _plot_weight_error_comparison(self, data, output_dir, prefix, layout_width):
        keys = ["all", "emg_only", "imu_only"]
        labels = ["EMG + IMU", "EMG-only", "IMU-only"]
        colors = [ThesisStyle.COLOR_FUSION, ThesisStyle.COLOR_EMG, ThesisStyle.COLOR_IMU]
        markers = ["o", "s", "^"]
        
        # Extract weight classes and their metrics for each run
        weight_data = {}
        for k in keys:
            per_weight = data[k].get("evaluation", {}).get("per_weight", [])
            for row in per_weight:
                w_val = row['Weight'].replace(' kg', '').replace('kg', '').strip()
                if w_val not in weight_data:
                    weight_data[w_val] = {}
                weight_data[w_val][k] = {
                    "MAE": float(row["MAE"]),
                    "RMSE": float(row["RMSE"])
                }
                
        # Sort weights numerically
        sorted_weights = sorted(weight_data.keys(), key=lambda w: float(w))
        x_positions = np.arange(len(sorted_weights))
        
        # 1. Generate MAE Plot
        ThesisStyle.apply(layout_width)
        fig1, ax1 = plt.subplots()
        
        for k, label, color, marker in zip(keys, labels, colors, markers):
            maes = []
            valid_x = []
            for idx, w in enumerate(sorted_weights):
                if k in weight_data[w]:
                    maes.append(weight_data[w][k]["MAE"])
                    valid_x.append(idx)
            if valid_x:
                ax1.plot(valid_x, maes, marker=marker, color=color, label=label, linewidth=1.5, markersize=4.5)
                
        ax1.set_xlabel("Weight Class (kg)", labelpad=8)
        ax1.set_ylabel("MAE (kg)", labelpad=8)
        title_mae = get_plot_title("modality_weight_mae", "MAE by Weight Class and Modality")
        ThesisStyle.set_title(ax1, title_mae)
        ax1.set_xticks(x_positions)
        ax1.set_xticklabels(sorted_weights)
        ax1.set_ylim(bottom=0)
        ax1.grid(True, linestyle='--', alpha=0.5)
        ThesisStyle.style_ax(ax1)
        ax1.legend(loc='upper left', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        ThesisStyle.save_figure(fig1, output_dir / f"{prefix}weight_mae_comparison")
        plt.close(fig1)

        # 2. Generate RMSE Plot
        ThesisStyle.apply(layout_width)
        fig2, ax2 = plt.subplots()
        
        for k, label, color, marker in zip(keys, labels, colors, markers):
            rmses = []
            valid_x = []
            for idx, w in enumerate(sorted_weights):
                if k in weight_data[w]:
                    rmses.append(weight_data[w][k]["RMSE"])
                    valid_x.append(idx)
            if valid_x:
                ax2.plot(valid_x, rmses, marker=marker, color=color, label=label, linewidth=1.5, markersize=4.5, linestyle='--')
                
        ax2.set_xlabel("Weight Class (kg)", labelpad=8)
        ax2.set_ylabel("RMSE (kg)", labelpad=8)
        title_rmse = get_plot_title("modality_weight_rmse", "RMSE by Weight Class and Modality")
        ThesisStyle.set_title(ax2, title_rmse)
        ax2.set_xticks(x_positions)
        ax2.set_xticklabels(sorted_weights)
        ax2.set_ylim(bottom=0)
        ax2.grid(True, linestyle='--', alpha=0.5)
        ThesisStyle.style_ax(ax2)
        ax2.legend(loc='upper left', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        ThesisStyle.save_figure(fig2, output_dir / f"{prefix}weight_rmse_comparison")
        plt.close(fig2)


@plot_registry.register("importance_channel")
class ImportanceChannelPlotter:
    """Plots a premium horizontal bar chart showing input channel permutation importance."""
    
    def plot(self, data, output_path, layout_width):
        feat_imp = data.get("feature_importance", {})
        channel_imp = feat_imp.get("permutation_channel")
        if not channel_imp:
            print("  [Warning] Missing channel permutation importance data. Skipping.")
            return False
            
        ThesisStyle.apply(layout_width)
        
        items = []
        for raw_lbl, val in channel_imp.items():
            clean_lbl = clean_channel_label(raw_lbl)
            
            # Modality tag
            if "_EMG" in raw_lbl or any(m in raw_lbl for m in ["Deltoid", "Brachii", "Brachioradialis", "Ulnaris", "Radialis"]):
                modality = "EMG"
            else:
                modality = "IMU"
                
            items.append((clean_lbl, val, modality))
            
        # Sort by importance magnitude
        items = sorted(items, key=lambda x: x[1], reverse=True)
        
        # Limit to top 15 for premium presentation
        top_n = min(15, len(items))
        items = items[:top_n]
        
        labels = [x[0] for x in items]
        values = [x[1] for x in items]
        modalities = [x[2] for x in items]
        
        fig, ax = plt.subplots(figsize=(6.5, 4.5) if layout_width == "default" else None)
        
        y_pos = np.arange(len(labels))
        colors = [ThesisStyle.COLOR_EMG if m == "EMG" else ThesisStyle.COLOR_IMU for m in modalities]
        
        bars = ax.barh(y_pos, values, color=colors, alpha=0.85, height=0.55, edgecolor=colors, linewidth=0.5)
        ax.axvline(0, color='#333333', linestyle='-', linewidth=0.7, alpha=0.5)
        
        # Annotate exact values
        for idx, bar in enumerate(bars):
            width = bar.get_width()
            val_str = f"{width:.3f}"
            if width >= 0:
                ax.annotate(val_str,
                            xy=(width, bar.get_y() + bar.get_height() / 2),
                            xytext=(4, 0), textcoords="offset points",
                            ha='left', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                            color='#333333')
            else:
                ax.annotate(val_str,
                            xy=(width, bar.get_y() + bar.get_height() / 2),
                            xytext=(-4, 0), textcoords="offset points",
                            ha='right', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                            color='#333333')
                
        ax.set_yticks(y_pos)
        ax.set_yticklabels(labels)
        ax.invert_yaxis()
        
        ax.set_xlabel("Permutation Importance (Mean MSE Drop)", labelpad=8)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        title = get_plot_title("importance_channel", "{model_type}: Channel Permutation Importance (Top {top_n})", model_type=model_type, top_n=top_n)
        ThesisStyle.set_title(ax, title)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor=ThesisStyle.COLOR_EMG, label='EMG Channel'),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label='IMU Channel')
        ]
        ax.legend(handles=legend_elements, loc='lower right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        max_val = max(abs(v) for v in values) if values else 1.0
        ax.set_xlim(-max_val * 1.15 if min(values) < 0 else 0, max_val * 1.18)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("importance_feature")
class ImportanceFeaturePlotter:
    """Plots a premium horizontal bar chart showing permutation importance of hand-crafted feature groups."""
    
    def plot(self, data, output_path, layout_width):
        feat_imp = data.get("feature_importance", {})
        feature_imp = feat_imp.get("permutation_feature")
        if not feature_imp:
            print("  [Warning] Missing feature permutation importance data. Skipping.")
            return False
            
        ThesisStyle.apply(layout_width)
        
        items = []
        for raw_lbl, val in feature_imp.items():
            clean_lbl = raw_lbl.replace("EMG_", "EMG ").replace("IMU_", "IMU ")
            
            full_names = {
                "MAV": "Mean Absolute Value (MAV)",
                "RMS": "Root Mean Square (RMS)",
                "WL": "Waveform Length (WL)",
                "ZC": "Zero Crossings (ZC)",
                "SSC": "Slope Sign Changes (SSC)",
                "VAR": "Variance (VAR)",
                "WAMP": "Willison Amplitude (WAMP)",
                "IEMG": "Integrated EMG (IEMG)",
                "LogDet": "Log Detector (LogDet)",
                "Skew": "Skewness",
                "Kurt": "Kurtosis",
                "HjMob": "Hjorth Mobility",
                "HjComp": "Hjorth Complexity",
                "Myopulse": "Myopulse Rate",
                "MNF": "Mean Frequency (MNF)",
                "MDF": "Median Frequency (MDF)",
                "Power": "Total Power",
                "SpecEntropy": "Spectral Entropy",
                "PeakFreq": "Peak Frequency",
                "BW": "Bandwidth",
                "DomFreq": "Dominant Frequency",
                "Jerk": "Absolute Jerk",
                "SMA": "Signal Magnitude Area (SMA)",
                "P2P": "Peak-to-Peak (P2P)",
                "SpecEnergy": "Spectral Energy",
                "SVM_Mean": "Signal Vector Magnitude (Mean)"
            }
            
            parts = clean_lbl.split(" ")
            prefix = parts[0]
            suffix = " ".join(parts[1:])
            
            refined_lbl = f"{prefix} {full_names.get(suffix, suffix)}"
            items.append((refined_lbl, val, prefix))
            
        items = sorted(items, key=lambda x: x[1], reverse=True)
        top_n = min(15, len(items))
        items = items[:top_n]
        
        labels = [x[0] for x in items]
        values = [x[1] for x in items]
        prefixes = [x[2] for x in items]
        
        fig, ax = plt.subplots(figsize=(6.5, 4.5) if layout_width == "default" else None)
        
        y_pos = np.arange(len(labels))
        colors = [ThesisStyle.COLOR_EMG if p == "EMG" else ThesisStyle.COLOR_IMU for p in prefixes]
        
        bars = ax.barh(y_pos, values, color=colors, alpha=0.85, height=0.55, edgecolor=colors, linewidth=0.5)
        ax.axvline(0, color='#333333', linestyle='-', linewidth=0.7, alpha=0.5)
        
        for idx, bar in enumerate(bars):
            width = bar.get_width()
            val_str = f"{width:.3f}"
            if width >= 0:
                ax.annotate(val_str,
                            xy=(width, bar.get_y() + bar.get_height() / 2),
                            xytext=(4, 0), textcoords="offset points",
                            ha='left', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                            color='#333333')
            else:
                ax.annotate(val_str,
                            xy=(width, bar.get_y() + bar.get_height() / 2),
                            xytext=(-4, 0), textcoords="offset points",
                            ha='right', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                            color='#333333')
                
        ax.set_yticks(y_pos)
        ax.set_yticklabels(labels)
        ax.invert_yaxis()
        
        ax.set_xlabel("Permutation Importance (Mean MSE Drop)", labelpad=8)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        title = get_plot_title("importance_feature", "{model_type}: Feature Permutation Importance (Top {top_n})", model_type=model_type, top_n=top_n)
        ThesisStyle.set_title(ax, title)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor=ThesisStyle.COLOR_EMG, label='EMG Feature Group'),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label='IMU Feature Group')
        ]
        ax.legend(handles=legend_elements, loc='lower right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        max_val = max(abs(v) for v in values) if values else 1.0
        ax.set_xlim(-max_val * 1.15 if min(values) < 0 else 0, max_val * 1.18)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("tradeoff")
class TradeoffPlotter:
    """Plots general accuracy (RMSE in kg) vs. per-sample inference latency (ms) for different modalities."""
    
    def plot(self, data, output_path, layout_width):
        sub_runs = [k for k in ["all", "emg_only", "imu_only"] if k in data and isinstance(data[k], dict)]
        if not sub_runs:
            print("  [Warning] Trade-off plotter requires multi-modality ablation runs (root dict). Skipping.")
            return False
            
        ThesisStyle.apply(layout_width)
        fig, ax = plt.subplots(figsize=(7.0, 4.8) if layout_width == "default" else None)
        
        name_map = {
            "all": "EMG + IMU\n(Sensor-Fused)",
            "emg_only": "EMG-only",
            "imu_only": "IMU-only"
        }
        
        color_map = {
            "all": ThesisStyle.COLOR_FUSION,
            "emg_only": ThesisStyle.COLOR_EMG,
            "imu_only": ThesisStyle.COLOR_IMU
        }
        
        latencies = []
        rmses = []
        maes = []
        labels = []
        colors = []
        
        # Order by logical complexity (IMU -> EMG -> ALL) to draw the frontier cleanly
        for k in ["imu_only", "emg_only", "all"]:
            if k not in data:
                continue
            run_data = data[k]
            pooled = run_data.get("evaluation", {}).get("pooled", {})
            timing = run_data.get("timing", {})
            
            rmse = pooled.get("RMSE")
            mae = pooled.get("MAE")
            latency = timing.get("avg_inference_per_sample_sec")
            
            if rmse is not None and latency is not None:
                latencies.append(latency * 1000.0) # convert to ms
                rmses.append(rmse)
                maes.append(mae if mae is not None else 0.0)
                labels.append(name_map.get(k, k))
                colors.append(color_map.get(k, ThesisStyle.COLOR_FUSION))
                
        if len(latencies) < 2:
            print("  [Warning] Not enough modality runs found for trade-off comparison. Skipping.")
            return False
            
        # Draw Pareto frontier line (dashed Slate Gray)
        ax.plot(latencies, rmses, color=ThesisStyle.COLOR_UNITY, linestyle='--', linewidth=1.2, alpha=0.7, zorder=1)
        
        # Plot scatter markers for each modality
        for lat, rms, lbl, col in zip(latencies, rmses, labels, colors):
            ax.scatter(lat, rms, color=col, s=80, edgecolor='white', linewidth=1.0, zorder=3)
            # Annotate with names offset
            ax.annotate(lbl.replace('\n', ' '), xy=(lat, rms), xytext=(10, -2),
                        textcoords='offset points', ha='left', va='center',
                        fontsize=plt.rcParams['font.size'] - 1.5, weight='semibold',
                        color=col,
                        bbox=dict(facecolor='white', alpha=0.8, edgecolor='none', pad=1))
            
            # Print latency and error value below marker
            label_detail = f"RMSE: {rms:.3f} kg\nLatency: {lat:.2f} ms"
            ax.annotate(label_detail, xy=(lat, rms), xytext=(10, -18),
                        textcoords='offset points', ha='left', va='top',
                        fontsize=plt.rcParams['font.size'] - 2.5,
                        color='#555555')
            
        ax.set_xlabel("Inference Latency per Sample (ms)", labelpad=8)
        ax.set_ylabel("Prediction Error (RMSE, kg)", labelpad=8)
        title = get_plot_title("tradeoff", "Generalization Accuracy vs. Inference Latency Trade-Off")
        ThesisStyle.set_title(ax, title)
        
        ax.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        
        # Expand xlim and ylim slightly to accommodate text annotations
        ax.set_xlim(min(latencies) * 0.8, max(latencies) * 1.5)
        ax.set_ylim(min(rmses) * 0.7, max(rmses) * 1.25)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("importance_individual")
class ImportanceIndividualPlotter:
    """Plots a premium horizontal bar chart showing individual feature permutation importance (top 15)."""
    
    def plot(self, data, output_path, layout_width):
        feat_imp = data.get("feature_importance", {})
        indiv_imp = feat_imp.get("permutation_individual")
        if not indiv_imp:
            print("  [Warning] Missing individual permutation importance data. Skipping.")
            return False
            
        ThesisStyle.apply(layout_width)
        
        items = []
        for raw_lbl, val in indiv_imp.items():
            clean_lbl = raw_lbl
            
            muscle_map = {
                "Anterior Deltoid": "AD",
                "Lateral Deltoid": "LD",
                "Posterior Deltoid": "PD",
                "Triceps Brachii": "TB",
                "Biceps Brachii": "BB",
                "Brachioradialis": "BR",
                "Flexor Carpi Ulnaris (FCU)": "FCU",
                "Flexor Carpi Ulnaris": "FCU",
                "Extensor Carpi Radialis (ECR)": "ECR",
                "Extensor Carpi Radialis": "ECR"
            }
            
            for m_full, m_short in muscle_map.items():
                clean_lbl = clean_lbl.replace(m_full, m_short)
                
            clean_lbl = clean_lbl.replace("_EMG_", " EMG ").replace("_IMU_", " IMU ")
            
            full_names = {
                "MAV": "MAV",
                "RMS": "RMS",
                "WL": "WL",
                "ZC": "ZC",
                "SSC": "SSC",
                "VAR": "Variance",
                "WAMP": "WAMP",
                "IEMG": "Integrated EMG",
                "LogDet": "Log Detector",
                "Skew": "Skewness",
                "Kurt": "Kurtosis",
                "HjMob": "Hjorth Mobility",
                "HjComp": "Hjorth Complexity",
                "Myopulse": "Myopulse",
                "MNF": "Mean Freq",
                "MDF": "Median Freq",
                "Power": "Total Power",
                "SpecEntropy": "Spectral Entropy",
                "PeakFreq": "Peak Freq",
                "BW": "Bandwidth",
                "Mean": "Mean",
                "Var": "Variance",
                "Std": "Std Dev",
                "Max": "Max",
                "Min": "Min",
                "SMA": "SMA",
                "P2P": "Peak-to-Peak",
                "IQR": "IQR",
                "Jerk": "Jerk",
                "Energy": "Energy",
                "DomFreq": "Dominant Freq",
                "SpecEnergy": "Spectral Energy",
                "SVM_Mean": "SVM Mean",
                "SVM_Std": "SVM Std"
            }
            
            for f_short, f_full in full_names.items():
                if clean_lbl.endswith(f_short):
                    clean_lbl = clean_lbl[:-len(f_short)] + f_full
                    break
                elif " " + f_short + "_" in clean_lbl:
                    clean_lbl = clean_lbl.replace(" " + f_short + "_", " " + f_full + "_")
                    
            if "_EMG" in raw_lbl or any(m in raw_lbl for m in ["Deltoid", "Brachii", "Brachioradialis", "Ulnaris", "Radialis"]):
                modality = "EMG"
            else:
                modality = "IMU"
                
            items.append((clean_lbl, val, modality))
            
        items = sorted(items, key=lambda x: x[1], reverse=True)
        top_n = min(15, len(items))
        items = items[:top_n]
        
        labels = [x[0] for x in items]
        values = [x[1] for x in items]
        modalities = [x[2] for x in items]
        
        fig, ax = plt.subplots(figsize=(7.0, 5.0) if layout_width == "default" else None)
        y_pos = np.arange(len(labels))
        colors = [ThesisStyle.COLOR_EMG if m == "EMG" else ThesisStyle.COLOR_IMU for m in modalities]
        
        bars = ax.barh(y_pos, values, color=colors, alpha=0.85, height=0.55, edgecolor=colors, linewidth=0.5)
        ax.axvline(0, color='#333333', linestyle='-', linewidth=0.7, alpha=0.5)
        
        for idx, bar in enumerate(bars):
            width = bar.get_width()
            val_str = f"{width:.4f}"
            if width >= 0:
                ax.annotate(val_str,
                            xy=(width, bar.get_y() + bar.get_height() / 2),
                            xytext=(4, 0), textcoords="offset points",
                            ha='left', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                            color='#333333')
            else:
                ax.annotate(val_str,
                            xy=(width, bar.get_y() + bar.get_height() / 2),
                            xytext=(-4, 0), textcoords="offset points",
                            ha='right', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                            color='#333333')
                            
        ax.set_yticks(y_pos)
        ax.set_yticklabels(labels)
        ax.invert_yaxis()
        
        ax.set_xlabel("Permutation Importance (Mean MSE Drop)", labelpad=8)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        title = get_plot_title("importance_individual", "{model_type}: Granular Feature Permutation Importance (Top {top_n})", model_type=model_type, top_n=top_n)
        ThesisStyle.set_title(ax, title)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor=ThesisStyle.COLOR_EMG, label='EMG Individual Feature'),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label='IMU Individual Feature')
        ]
        ax.legend(handles=legend_elements, loc='lower right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        max_val = max(abs(v) for v in values) if values else 1.0
        ax.set_xlim(-max_val * 1.15 if min(values) < 0 else 0, max_val * 1.18)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("tukey")
class TukeyPlotter:
    """Plots a premium pairwise Tukey HSD confidence/difference chart for statistical significance."""
    
    def plot(self, data, output_path, layout_width):
        anova = data.get("evaluation", {}).get("anova")
        if not anova or not isinstance(anova, dict) or "err" not in anova or not isinstance(anova.get("err"), dict) or "tukey" not in anova["err"] or not isinstance(anova["err"]["tukey"], list):
            print("  [Warning] Missing Tukey HSD data in run data. Skipping tukey plot.")
            return False
            
        ThesisStyle.apply(layout_width)
        fig, ax = plt.subplots()
        
        tukey_results = anova["err"]["tukey"]
        
        comparisons = []
        diffs = []
        p_vals = []
        
        for idx, row in enumerate(tukey_results):
            w1 = row.get("w_high") or row.get("g2")
            w2 = row.get("w_low") or row.get("g1")
            diff = row.get("diff")
            p = row.get("p")
            
            if w1 is not None and w2 is not None and diff is not None and p is not None:
                comparisons.append(f"{float(w1):.2f}kg vs {float(w2):.2f}kg".replace(".00kg", "kg"))
                diffs.append(diff)
                p_vals.append(p)
                
        if not comparisons:
            print("  [Warning] No valid Tukey comparisons found. Skipping.")
            return False
            
        y_pos = np.arange(len(comparisons))
        
        colors = []
        labels = []
        for p in p_vals:
            if p < 0.001:
                colors.append(ThesisStyle.COLOR_P_001)
                labels.append("*** (p < 0.001)")
            elif p < 0.01:
                colors.append(ThesisStyle.COLOR_P_01)
                labels.append("** (p < 0.01)")
            elif p < 0.05:
                colors.append(ThesisStyle.COLOR_P_05)
                labels.append("* (p < 0.05)")
            else:
                colors.append(ThesisStyle.COLOR_P_NS)
                labels.append("n.s. (p >= 0.05)")
                
        bars = ax.barh(y_pos, diffs, color=colors, alpha=0.85, height=0.45, edgecolor=colors, linewidth=0.5)
        
        for idx, (bar, label) in enumerate(zip(bars, labels)):
            width = bar.get_width()
            if width >= 0:
                ax.annotate(label,
                            xy=(width, bar.get_y() + bar.get_height() / 2),
                            xytext=(5, 0), textcoords="offset points",
                            ha='left', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                            weight='semibold' if "p <" in label else 'normal',
                            color=colors[idx])
            else:
                ax.annotate(label,
                            xy=(width, bar.get_y() + bar.get_height() / 2),
                            xytext=(-5, 0), textcoords="offset points",
                            ha='right', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                            weight='semibold' if "p <" in label else 'normal',
                            color=colors[idx])
                
        ax.set_yticks(y_pos)
        ax.set_yticklabels(comparisons)
        ax.invert_yaxis()
        
        ax.set_xlabel("Mean Absolute Error Difference (kg)", labelpad=8)
        title = get_plot_title("tukey", "Tukey HSD Pairwise Significance (Prediction Error)")
        ThesisStyle.set_title(ax, title)
        ax.axvline(0, color='#333333', linestyle='-', linewidth=0.8, alpha=0.5)
        
        max_diff = max(abs(d) for d in diffs) if diffs else 1.0
        ax.set_xlim(-max_diff * 1.35 if min(diffs) < 0 else 0, max_diff * 1.35)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("deepshap_channel")
class DeepShapChannelPlotter:
    """Plots a premium horizontal bar chart showing input channel DeepSHAP attribution."""
    
    def plot(self, data, output_path, layout_width):
        feat_imp = data.get("feature_importance", {})
        channel_imp = feat_imp.get("deepshap_channel")
        if not channel_imp:
            print("  [Warning] Missing DeepSHAP channel attribution data. Skipping.")
            return False
            
        ThesisStyle.apply(layout_width)
        
        items = []
        for raw_lbl, val in channel_imp.items():
            clean_lbl = clean_channel_label(raw_lbl)
            
            # Modality tag
            if "_EMG" in raw_lbl or any(m in raw_lbl for m in ["Deltoid", "Brachii", "Brachioradialis", "Ulnaris", "Radialis"]):
                modality = "EMG"
            else:
                modality = "IMU"
                
            items.append((clean_lbl, val, modality))
            
        # Sort by importance magnitude
        items = sorted(items, key=lambda x: x[1], reverse=True)
        
        # No longer limit to top 15 as requested: show all channels
        top_n = len(items)
        
        labels = [x[0] for x in items]
        values = [x[1] for x in items]
        modalities = [x[2] for x in items]
        
        # Dynamically scale figure height based on the number of channels to prevent overlapping
        fig_height = max(4.5, len(labels) * 0.22)
        fig, ax = plt.subplots(figsize=(6.5, fig_height) if layout_width == "default" else None)
        
        y_pos = np.arange(len(labels))
        colors = [ThesisStyle.COLOR_EMG if m == "EMG" else ThesisStyle.COLOR_IMU for m in modalities]
        
        bars = ax.barh(y_pos, values, color=colors, alpha=0.85, height=0.55, edgecolor=colors, linewidth=0.5)
        ax.axvline(0, color='#333333', linestyle='-', linewidth=0.7, alpha=0.5)
        
        # Annotate exact values
        for idx, bar in enumerate(bars):
            width = bar.get_width()
            val_str = f"{width:.3f}"
            ax.annotate(val_str,
                        xy=(width, bar.get_y() + bar.get_height() / 2),
                        xytext=(4, 0), textcoords="offset points",
                        ha='left', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                        color='#333333')
                
        ax.set_yticks(y_pos)
        ax.set_yticklabels(labels)
        ax.invert_yaxis()
        
        ax.set_xlabel("Mean |SHAP Value|", labelpad=8)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        title = get_plot_title("deepshap_channel", "{model_type}: DeepSHAP Channel Importance", model_type=model_type)
        ThesisStyle.set_title(ax, title)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor=ThesisStyle.COLOR_EMG, label='EMG Channel'),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label='IMU Channel')
        ]
        ax.legend(handles=legend_elements, loc='lower right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        max_val = max(values) if values else 1.0
        ax.set_xlim(0, max_val * 1.18)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("deepshap_feature_type")
class DeepShapFeatureTypePlotter:
    """Plots a premium horizontal bar chart showing DeepSHAP attribution of feature types."""
    
    def plot(self, data, output_path, layout_width):
        feat_imp = data.get("feature_importance", {})
        feature_imp = feat_imp.get("deepshap_feature_type")
        if not feature_imp:
            print("  [Warning] Missing DeepSHAP feature type data. Skipping.")
            return False
            
        ThesisStyle.apply(layout_width)
        
        items = []
        for raw_lbl, val in feature_imp.items():
            clean_lbl = raw_lbl.replace("EMG_", "EMG ").replace("IMU_", "IMU ")
            
            full_names = {
                "MAV": "Mean Absolute Value (MAV)",
                "RMS": "Root Mean Square (RMS)",
                "WL": "Waveform Length (WL)",
                "ZC": "Zero Crossings (ZC)",
                "SSC": "Slope Sign Changes (SSC)",
                "VAR": "Variance (VAR)",
                "WAMP": "Willison Amplitude (WAMP)",
                "IEMG": "Integrated EMG (IEMG)",
                "LogDet": "Log Detector (LogDet)",
                "Skew": "Skewness",
                "Kurt": "Kurtosis",
                "HjMob": "Hjorth Mobility",
                "HjComp": "Hjorth Complexity",
                "Myopulse": "Myopulse Rate",
                "MNF": "Mean Frequency (MNF)",
                "MDF": "Median Frequency (MDF)",
                "Power": "Total Power",
                "SpecEntropy": "Spectral Entropy",
                "PeakFreq": "Peak Frequency",
                "BW": "Bandwidth",
                "DomFreq": "Dominant Frequency",
                "Jerk": "Absolute Jerk",
                "SMA": "Signal Magnitude Area (SMA)",
                "P2P": "Peak-to-Peak (P2P)",
                "SpecEnergy": "Spectral Energy",
                "SVM_Mean": "Signal Vector Magnitude (Mean)"
            }
            
            parts = clean_lbl.split(" ")
            prefix = parts[0]
            suffix = " ".join(parts[1:])
            
            refined_lbl = f"{prefix} {full_names.get(suffix, suffix)}"
            items.append((refined_lbl, val, prefix))
            
        items = sorted(items, key=lambda x: x[1], reverse=True)
        top_n = min(15, len(items))
        items = items[:top_n]
        
        labels = [x[0] for x in items]
        values = [x[1] for x in items]
        prefixes = [x[2] for x in items]
        
        fig, ax = plt.subplots(figsize=(6.5, 4.5) if layout_width == "default" else None)
        
        y_pos = np.arange(len(labels))
        colors = [ThesisStyle.COLOR_EMG if p == "EMG" else ThesisStyle.COLOR_IMU for p in prefixes]
        
        bars = ax.barh(y_pos, values, color=colors, alpha=0.85, height=0.55, edgecolor=colors, linewidth=0.5)
        ax.axvline(0, color='#333333', linestyle='-', linewidth=0.7, alpha=0.5)
        
        for idx, bar in enumerate(bars):
            width = bar.get_width()
            val_str = f"{width:.3f}"
            ax.annotate(val_str,
                        xy=(width, bar.get_y() + bar.get_height() / 2),
                        xytext=(4, 0), textcoords="offset points",
                        ha='left', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                        color='#333333')
                
        ax.set_yticks(y_pos)
        ax.set_yticklabels(labels)
        ax.invert_yaxis()
        
        ax.set_xlabel("Mean |SHAP Value|", labelpad=8)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        title = get_plot_title("deepshap_feature_type", "{model_type}: DeepSHAP Feature Importance (Top {top_n})", model_type=model_type, top_n=top_n)
        ThesisStyle.set_title(ax, title)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor=ThesisStyle.COLOR_EMG, label='EMG Feature Group'),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label='IMU Feature Group')
        ]
        ax.legend(handles=legend_elements, loc='lower right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        max_val = max(values) if values else 1.0
        ax.set_xlim(0, max_val * 1.18)
        
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True


@plot_registry.register("deepshap_modality")
class DeepShapModalityPlotter:
    """Plots DeepSHAP Modality Split (Pie chart of EMG/IMU/Anthro shares + grouped bar chart)."""
    
    def plot(self, data, output_path, layout_width):
        feat_imp = data.get("feature_importance", {})
        mod_imp = feat_imp.get("deepshap_modality")
        chan_imp = feat_imp.get("deepshap_channel")
        
        if not mod_imp or not chan_imp:
            print("  [Warning] Missing DeepSHAP modality or channel data. Skipping.")
            return False
            
        ThesisStyle.apply(layout_width)
        
        # 1. Prepare modality colors
        modality_colors = {
            "EMG": ThesisStyle.COLOR_EMG,
            "IMU": ThesisStyle.COLOR_IMU,
            "Anthro": ThesisStyle.COLOR_MEDIAN  # High-contrast yellow/gold
        }
        
        # 2. Gather and group channel items by Modality: EMG first, then IMU, then Anthro
        items = []
        for raw_lbl, val in chan_imp.items():
            clean_lbl = clean_channel_label(raw_lbl)
            
            if "_EMG" in raw_lbl or any(m in raw_lbl for m in ["Deltoid", "Brachii", "Brachioradialis", "Ulnaris", "Radialis"]):
                modality = "EMG"
            elif "_IMU" in raw_lbl or any(m in raw_lbl for m in ["ax", "ay", "az", "roll", "pitch", "yaw", "diff", "$"]):
                modality = "IMU"
            else:
                modality = "Anthro"
                
            items.append((clean_lbl, val, modality))
            
        grouped_items = []
        for mod in ["EMG", "IMU", "Anthro"]:
            mod_items = [x for x in items if x[2] == mod]
            # Sort within group descending
            mod_items = sorted(mod_items, key=lambda x: x[1], reverse=True)
            grouped_items.extend(mod_items)
            
        # No longer limit to top 15 grouped features: show all channels
        
        # 3. Draw 1 row, 2 columns layout with dynamic height based on the number of channels
        fig_height = max(5.0, len(grouped_items) * 0.22)
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.0, fig_height) if layout_width != "column" else (8.0, fig_height * 0.8))
        
        # 4. Left: Pie Chart
        shares = []
        labels = []
        colors = []
        
        for k in ["EMG", "IMU", "Anthro"]:
            val = mod_imp.get(k, 0.0)
            if val > 0:
                shares.append(val)
                labels.append(k)
                colors.append(modality_colors.get(k, ThesisStyle.COLOR_UNITY))
                
        if not shares:
            print("  [Warning] No positive modality shares found. Skipping.")
            plt.close(fig)
            return False
            
        total = sum(shares)
        pct_labels = [f"{l}\n({(s/total)*100:.1f}%)" for l, s in zip(labels, shares)]
        
        ax1.pie(
            shares,
            labels=pct_labels,
            colors=colors,
            startangle=90,
            autopct="%1.1f%%",
            wedgeprops=dict(edgecolor="white", linewidth=1.5, antialiased=True),
            textprops=dict(fontfamily='serif', size=plt.rcParams['font.size'] - 1.0)
        )
        ax1.set_title("Modality Share of Total |SHAP|", fontsize=plt.rcParams['font.size'])
        
        # 5. Right: Grouped Horizontal Bar Chart of Channels
        ch_labels = [x[0] for x in grouped_items]
        ch_vals = [x[1] for x in grouped_items]
        ch_mods = [x[2] for x in grouped_items]
        
        y_pos = np.arange(len(ch_labels))
        ch_colors = [modality_colors.get(m, ThesisStyle.COLOR_UNITY) for m in ch_mods]
        
        bars = ax2.barh(y_pos, ch_vals, color=ch_colors, alpha=0.85, height=0.55, edgecolor=ch_colors, linewidth=0.5)
        ax2.axvline(0, color='#333333', linestyle='-', linewidth=0.7, alpha=0.5)
        
        for idx, bar in enumerate(bars):
            width = bar.get_width()
            val_str = f"{width:.3f}"
            ax2.annotate(val_str,
                        xy=(width, bar.get_y() + bar.get_height() / 2),
                        xytext=(4, 0), textcoords="offset points",
                        ha='left', va='center', fontsize=plt.rcParams['font.size'] - 2.5,
                        color='#333333')
                        
        ax2.set_yticks(y_pos)
        ax2.set_yticklabels(ch_labels)
        ax2.invert_yaxis()
        ax2.set_xlabel("Mean |SHAP| Value", labelpad=8)
        ax2.set_title("Importance per Grouped Feature", fontsize=plt.rcParams['font.size'])
        ax2.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        title_text = get_plot_title("deepshap_modality", "{model_type}: DeepSHAP Modality Split", model_type=model_type)
        ThesisStyle.set_suptitle(fig, title_text)
        
        plt.tight_layout(rect=[0, 0, 1, 0.93])
        ThesisStyle.save_figure(fig, output_path)
        plt.close(fig)
        return True



# ===========================================================================
# 4b. Cross-Run Comparison Plotter (Generalized vs Participant-Specific)
# ===========================================================================

class RunComparisonPlotter:
    """Generates side-by-side comparison plots between two model run configurations
    (e.g. Generalized LOPO model vs Participant-Specific model).

    Gen run  → uses macro_avg (mean ± std across LOPO folds) for all metrics.
    Par-spec → uses pooled values (single split).
    """

    LABEL_GEN = "Generalized\n(LOPO)"
    LABEL_PAR = "Participant-Specific\n(Single Split)"
    COLOR_GEN = "#4477AA"    # Paul Tol Bright Blue  – generalized model
    COLOR_PAR = "#95356F"    # Paul Tol Muted Purple – participant-specific model

    def _load(self, run_dir_raw):
        """Resolve and load run_data.json from a run directory (supports multi-run root)."""
        path = Path(run_dir_raw)
        if not path.is_absolute():
            script_dir = Path(__file__).resolve().parent
            for base in [Path.cwd(), script_dir.parent, script_dir]:
                candidate = (base / run_dir_raw)
                if (candidate / "run_data.json").exists():
                    path = candidate
                    break
        json_path = path / "run_data.json"
        if json_path.exists():
            with open(json_path) as f:
                data = json.load(f)
            # Consolidated multi-modality root: collapse to the 'all' (fused) sub-run
            # so callers expecting a single-run schema (e.g. the par-spec comparison) work.
            if isinstance(data, dict) and 'all' in data and set(data.keys()) <= {'all', 'emg_only', 'imu_only'}:
                return data['all']
            return data
        # Multi-run root: load the "all" sub-run
        all_path = path / "all" / "run_data.json"
        if all_path.exists():
            with open(all_path) as f:
                return json.load(f)
        raise FileNotFoundError(f"Cannot find run_data.json in {run_dir_raw}")

    def _get_gen_metrics(self, data):
        """Extract participant-class-macro metrics (mean ± std) from a LOPO run."""
        eval_dict = data.get("evaluation", {})
        cpm = eval_dict.get("macro_metrics", {}).get("class_participant_macro", {}) or eval_dict.get("class_participant_macro", {})
        macro = eval_dict.get("macro_avg", {})
        
        mae = cpm.get("MAE") if cpm.get("MAE") is not None else macro.get("MAE", 0.0)
        rmse = cpm.get("RMSE") if cpm.get("RMSE") is not None else macro.get("RMSE", 0.0)
        r2 = cpm.get("R2") if cpm.get("R2") is not None else macro.get("R2", 0.0)
        
        return {
            "MAE":  (mae,  macro.get("MAE_std", 0.0)),
            "RMSE": (rmse, macro.get("RMSE_std", 0.0)),
            "R2":   (r2,   macro.get("R2_std", 0.0)),
            "Gap":  (macro.get("Generalization Gap (MAE)", 0.0),
                     macro.get("Generalization Gap (MAE)_std", 0.0)),
        }

    def _get_par_metrics(self, data):
        """Extract participant-class-macro or class-macro metrics from a single-subject or single-split run."""
        eval_dict = data.get("evaluation", {})
        cpm = eval_dict.get("macro_metrics", {}).get("class_participant_macro", {}) or eval_dict.get("class_participant_macro", {})
        if not cpm or cpm.get("MAE") is None:
            cpm = eval_dict.get("class_macro_avg", {}) or eval_dict.get("class_macro", {})
            if not cpm:
                cpm = eval_dict.get("macro_metrics", {}).get("class_macro", {})
                
        mae = cpm.get("MAE")
        rmse = cpm.get("RMSE")
        r2 = cpm.get("R2")
        
        if mae is None:
            macro = eval_dict.get("macro_avg", {})
            mae = macro.get("MAE") or eval_dict.get("pooled", {}).get("MAE", 0.0)
        if rmse is None:
            macro = eval_dict.get("macro_avg", {})
            rmse = macro.get("RMSE") or eval_dict.get("pooled", {}).get("RMSE", 0.0)
        if r2 is None:
            macro = eval_dict.get("macro_avg", {})
            r2 = macro.get("R2") or eval_dict.get("pooled", {}).get("R2", 0.0)
            
        return {
            "MAE": mae,
            "RMSE": rmse,
            "R2": r2,
        }

    # ------------------------------------------------------------------
    # Plot 1a: 6-Grid Regression Plot Comparison
    # ------------------------------------------------------------------
    def _get_regression_metrics(self, data, is_specialized=False):
        """Extract MAE, RMSE, and R2 for regression plotting using participant-class-macro averages."""
        eval_dict = data.get("evaluation", {})
        cpm = eval_dict.get("macro_metrics", {}).get("class_participant_macro", {}) or eval_dict.get("class_participant_macro", {})
        macro = eval_dict.get("macro_avg", {})
        
        mae = cpm.get("MAE")
        rmse = cpm.get("RMSE")
        r2 = cpm.get("R2")
        
        # Fallbacks
        if mae is None or rmse is None or r2 is None:
            per_part = eval_dict.get("per_participant", [])
            if per_part:
                if mae is None:
                    mae = float(np.mean([float(p["MAE"]) for p in per_part]))
                if rmse is None:
                    rmse = float(np.mean([float(p["RMSE"]) for p in per_part]))
                if r2 is None:
                    r2_vals = [p.get("R2") for p in per_part if p.get("R2") is not None and not np.isnan(p.get("R2"))]
                    r2 = float(np.mean(r2_vals)) if r2_vals else None
            
        if mae is None:
            mae = macro.get("MAE") or eval_dict.get("pooled", {}).get("MAE")
        if rmse is None:
            rmse = macro.get("RMSE") or eval_dict.get("pooled", {}).get("RMSE")
        if r2 is None:
            r2 = macro.get("R2") or macro.get("R2_score") or eval_dict.get("pooled", {}).get("R2")
            
        # Fallback calculations if anything is still None
        if mae is None or rmse is None or r2 is None:
            predictions = data.get("predictions", {})
            if "y_true" in predictions and "y_pred" in predictions:
                y_true = np.array(predictions["y_true"])
                y_pred = np.maximum(0.0, np.array(predictions["y_pred"]))
                from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error
                if mae is None:
                    mae = mean_absolute_error(y_true, y_pred)
                if rmse is None:
                    rmse = np.sqrt(mean_squared_error(y_true, y_pred))
                if r2 is None:
                    r2 = r2_score(y_true, y_pred)
        
        return {"MAE": mae, "RMSE": rmse, "R2": r2}

    def _plot_regression_6grid(self, gen_all, gen_emg, gen_imu, par_all, par_emg, par_imu, output_dir, layout_width):
        ThesisStyle.apply(layout_width)
        
        # Grid dimensions: 3 rows (Modalities) x 2 columns (Specialized vs Generalized)
        # Made more compact vertically to ensure plots are clearly rectangular (aspect ratio ~1.6:1)
        if layout_width == "column":
            figsize = (3.5, 3.4)
        elif layout_width == "double":
            figsize = (7.0, 6.2)
        else: # "default"
            figsize = (8.0, 7.1)
            
        fig, axes = plt.subplots(nrows=3, ncols=2, figsize=figsize, sharex=True, sharey=True)
        
        # Map runs to their grid slots
        grid_runs = [
            # Row 0: Sensor-Fused (EMG + IMU)
            [(par_all, "Participant-Specialized", True, ThesisStyle.COLOR_FUSION, "#EBF7FA", "#004D60"),
             (gen_all, "Generalized (LOPO)", False, ThesisStyle.COLOR_FUSION, "#EBF7FA", "#004D60")],
            # Row 1: EMG-only
            [(par_emg, "Participant-Specialized", True, ThesisStyle.COLOR_EMG, "#FAF0F5", "#551133"),
             (gen_emg, "Generalized (LOPO)", False, ThesisStyle.COLOR_EMG, "#FAF0F5", "#551133")],
            # Row 2: IMU-only
            [(par_imu, "Participant-Specialized", True, ThesisStyle.COLOR_IMU, "#F0F7F4", "#115522"),
             (gen_imu, "Generalized (LOPO)", False, ThesisStyle.COLOR_IMU, "#F0F7F4", "#115522")]
        ]
        
        row_labels = ["Sensor-Fused (EMG + IMU)", "EMG-only", "IMU-only"]
        
        # Determine shared actual weights from all available predictions
        all_y_true = []
        for row in grid_runs:
            for run, _, _, _, _, _ in row:
                if run and "predictions" in run and "y_true" in run["predictions"]:
                    all_y_true.extend(run["predictions"]["y_true"])
        if len(all_y_true) > 0:
            actual_weights = np.sort(np.unique(all_y_true))
        else:
            actual_weights = np.array([0.0, 0.98, 1.97, 2.95, 4.15, 5.93])
            
        # Determine dynamic y_pred max for axes limits
        max_val = 6.5
        for row in grid_runs:
            for run, _, _, _, _, _ in row:
                if run and "predictions" in run and "y_pred" in run["predictions"]:
                    max_val = max(max_val, np.max(run["predictions"]["y_pred"]))
        max_val = min(7.5, max_val + 0.25)
        min_val = -0.2
        
        for row_idx, cols in enumerate(grid_runs):
            for col_idx, (run, label, is_spec, primary_color, fill_color, accent_color) in enumerate(cols):
                ax = axes[row_idx, col_idx]
                
                # Plot perfect prediction diagonal
                ax.plot([min_val, max_val], [min_val, max_val], 
                        color=ThesisStyle.COLOR_UNITY, linestyle='--', linewidth=1.0, alpha=0.7, 
                        label='Perfect Prediction' if (row_idx == 0 and col_idx == 0) else '', zorder=1)
                
                if not run or "predictions" not in run:
                    ax.text(0.5, 0.5, "Data Not Available", ha='center', va='center', transform=ax.transAxes,
                            fontproperties=fm.FontProperties(family='serif', style='italic'))
                    continue
                
                preds = run["predictions"]
                y_true = np.array(preds["y_true"])
                y_pred = np.maximum(0.0, np.array(preds["y_pred"]))
                
                pred_groups = [y_pred[y_true == w] for w in actual_weights]
                
                # Downsample and plot outliers
                first_outlier = True
                for w_idx, w in enumerate(actual_weights):
                    g_preds = pred_groups[w_idx]
                    if len(g_preds) > 0:
                        q1 = np.percentile(g_preds, 25)
                        q3 = np.percentile(g_preds, 75)
                        iqr = q3 - q1
                        outliers = g_preds[(g_preds < q1 - 1.5 * iqr) | (g_preds > q3 + 1.5 * iqr)]
                        
                        if len(outliers) > 0:
                            if len(outliers) > 50:
                                np.random.seed(42)
                                outliers = np.random.choice(outliers, size=50, replace=False)
                                
                            label_out = "Outliers" if (first_outlier and row_idx == 0 and col_idx == 0) else ""
                            ax.scatter(np.full_like(outliers, w), outliers, 
                                       alpha=0.3, s=8, color=ThesisStyle.COLOR_OUTLIER, edgecolors='none', 
                                       zorder=2, label=label_out)
                            if label_out:
                                first_outlier = False
                                
                # Create styled boxplots
                bp = ax.boxplot(pred_groups, positions=actual_weights, widths=0.28, 
                                patch_artist=True, showfliers=False, showcaps=False,
                                whiskerprops={'color': '#475569', 'linewidth': 0.8},
                                boxprops={'edgecolor': primary_color, 'linewidth': 1.0},
                                zorder=3)
                
                for box in bp['boxes']:
                    box.set_facecolor(fill_color)
                    box.set_alpha(0.8)
                    
                for idx, median in enumerate(bp['medians']):
                    median.set_color(accent_color)
                    median.set_linewidth(1.2)
                    if idx == 0 and row_idx == 0 and col_idx == 0:
                        median.set_label('Median')
                        
                # Extract metrics
                metrics = self._get_regression_metrics(run, is_specialized=is_spec)
                mae = metrics["MAE"]
                rmse = metrics["RMSE"]
                r2 = metrics["R2"]
                
                stats_text = f"$\\text{{R}}^2 = {r2:.3f}$\n$\\mathrm{{MAE}} = {mae:.3f}$ kg\n$\\mathrm{{RMSE}} = {rmse:.3f}$ kg"
                
                ax.text(0.04, 0.96, stats_text, 
                        transform=ax.transAxes, verticalalignment='top', fontsize=plt.rcParams['font.size'] - 1.5,
                        bbox=dict(facecolor='white', alpha=0.92, 
                                  edgecolor=primary_color, boxstyle='round,pad=0.4', linewidth=0.8))
                
                # Column titles at the top row (row_idx == 0) - styled in bold black sans-serif
                if row_idx == 0:
                    ax.set_title(
                        label,
                        fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['axes.titlesize']),
                        color='black',
                        pad=10
                    )
                
                # Row titles and Y-axis label on the leftmost plots (col_idx == 0)
                if col_idx == 0:
                    # Standard y-axis label in Serif font (with a very tight labelpad)
                    ax.set_ylabel("Predicted Weight (kg)", labelpad=4)
                    
                    # Distinct bold Fira Sans header label for the modality row, colored in black (very tight margin)
                    ax.annotate(row_labels[row_idx], xy=(-0.24, 0.5), xycoords='axes fraction',
                                rotation=90, ha='center', va='center',
                                fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['axes.labelsize']),
                                color='black')
                else:
                    ax.set_ylabel("", labelpad=0)
                    
                # X-axis label on the bottom row (row_idx == 2)
                if row_idx == 2:
                    ax.set_xlabel("Actual Weight (kg)", labelpad=8)
                else:
                    ax.set_xlabel("", labelpad=0)
                    
        # Apply unified ticks and limits
        for ax in axes.flatten():
            ax.set_xlim(min_val, max_val)
            ax.set_ylim(min_val, max_val)
            ax.set_xticks(actual_weights)
            ax.set_yticks(actual_weights)
            
            def format_label(x):
                return f"{x:.2f}".rstrip('0').rstrip('.')
                
            ax.set_xticklabels([format_label(w) for w in actual_weights])
            ax.set_yticklabels([format_label(w) for w in actual_weights])
            ThesisStyle.style_ax(ax)
            
        # Add single global legend at the bottom of the grid
        from matplotlib.lines import Line2D
        handles, labels = axes[0, 0].get_legend_handles_labels()
        legend_handles = []
        for h, l in zip(handles, labels):
            if l == "Median":
                # Use a dark grey proxy handle for the Median to represent the median of all plots generally
                legend_handles.append(Line2D([0], [0], color="#475569", linewidth=1.5))
            else:
                legend_handles.append(h)
                
        fig.legend(handles=legend_handles, labels=labels, loc='lower center', ncol=3,
                   bbox_to_anchor=(0.5, 0.01),
                   frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        plt.tight_layout(rect=[0.01, 0.04, 1, 1.0])
        ThesisStyle.save_figure(fig, output_dir / "comp_regression_6grid")
        plt.close(fig)

    def _plot_participant_bars_subplot(self, ax, participants, vals_all, vals_emg, vals_imu, mean_all, mean_emg, mean_imu, clip_val=1.0):
        x = np.arange(len(participants))
        width = 0.25
        
        # Clip values at clip_val for plotting
        plot_all = [min(v, clip_val) for v in vals_all]
        plot_emg = [min(v, clip_val) for v in vals_emg]
        plot_imu = [min(v, clip_val) for v in vals_imu]
        
        rects1 = ax.bar(x - width, plot_all, width, label='Sensor-Fused (EMG + IMU)', color=ThesisStyle.COLOR_FUSION, alpha=0.85, edgecolor=ThesisStyle.COLOR_FUSION, linewidth=0.5)
        rects2 = ax.bar(x, plot_emg, width, label='EMG-only', color=ThesisStyle.COLOR_EMG, alpha=0.85, edgecolor=ThesisStyle.COLOR_EMG, linewidth=0.5)
        rects3 = ax.bar(x + width, plot_imu, width, label='IMU-only', color=ThesisStyle.COLOR_IMU, alpha=0.85, edgecolor=ThesisStyle.COLOR_IMU, linewidth=0.5)
        
        # Annotate any clipped bars vertically to prevent adjacent overlap
        for idx, val in enumerate(vals_all):
            if val > clip_val:
                ax.text(x[idx] - width, clip_val * 1.02, f"{val:.2f}", ha='center', va='bottom',
                         color=ThesisStyle.COLOR_FUSION, fontweight='bold', clip_on=False,
                         fontsize=plt.rcParams['font.size'] - 2.8, rotation=90)
        for idx, val in enumerate(vals_emg):
            if val > clip_val:
                ax.text(x[idx], clip_val * 1.02, f"{val:.2f}", ha='center', va='bottom',
                         color=ThesisStyle.COLOR_EMG, fontweight='bold', clip_on=False,
                         fontsize=plt.rcParams['font.size'] - 2.8, rotation=90)
        for idx, val in enumerate(vals_imu):
            if val > clip_val:
                ax.text(x[idx] + width, clip_val * 1.02, f"{val:.2f}", ha='center', va='bottom',
                         color=ThesisStyle.COLOR_IMU, fontweight='bold', clip_on=False,
                         fontsize=plt.rcParams['font.size'] - 2.8, rotation=90)
                          
        # Add mean lines
        ax.axhline(mean_all, color=ThesisStyle.COLOR_FUSION, linestyle='--', linewidth=1.0, alpha=0.7)
        ax.axhline(mean_emg, color=ThesisStyle.COLOR_EMG, linestyle='--', linewidth=1.0, alpha=0.7)
        ax.axhline(mean_imu, color=ThesisStyle.COLOR_IMU, linestyle='--', linewidth=1.0, alpha=0.7)
        
        # Horizontally stagger the mean labels to prevent overlap
        means_data = [
            (mean_all, ThesisStyle.COLOR_FUSION),
            (mean_emg, ThesisStyle.COLOR_EMG),
            (mean_imu, ThesisStyle.COLOR_IMU)
        ]
        # Sort them by value ascending
        means_sorted = sorted(means_data, key=lambda val: val[0])
        
        # Stagger positions across left, center, right to avoid overlap
        for idx, (val, color) in enumerate(means_sorted):
            if idx == 0:
                x_pos = 0.02
                ha = 'left'
            elif idx == 1:
                x_pos = 0.50
                ha = 'center'
            else:
                x_pos = 0.98
                ha = 'right'
            ax.text(x_pos, val, f"Mean: {val:.3f}", transform=ax.get_yaxis_transform(),
                     color=color, va='bottom', ha=ha,
                     fontsize=plt.rcParams['font.size'] - 2.5, fontweight='semibold',
                     bbox=dict(facecolor='white', alpha=0.85, edgecolor='none', pad=1.0, boxstyle='round,pad=0.15'))
                  
        ylim_val = clip_val * 1.15
        ax.set_ylim(0, ylim_val)
        if clip_val == 3.0:
            ticks = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
            tick_labels = ["0.0", "0.5", "1.0", "1.5", "2.0", "2.5", "3.0"]
        elif clip_val == 2.0:
            ticks = [0.0, 0.5, 1.0, 1.5, 2.0]
            tick_labels = ["0.0", "0.5", "1.0", "1.5", "2.0"]
        elif clip_val == 1.5:
            ticks = [0.0, 0.5, 1.0, 1.5]
            tick_labels = ["0.0", "0.5", "1.0", "1.5"]
        elif clip_val == 1.25:
            ticks = [0.0, 0.25, 0.5, 0.75, 1.0, 1.25]
            tick_labels = ["0.0", "0.25", "0.5", "0.75", "1.0", "1.25"]
        elif clip_val == 1.0:
            ticks = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]
            tick_labels = ["0.0", "0.2", "0.4", "0.6", "0.8", "1.0"]
        else:
            ticks = list(np.linspace(0, clip_val, 6))
            tick_labels = [f"{t:.1f}" for t in ticks]
            
        ax.set_yticks(ticks)
        ax.set_yticklabels(tick_labels)
        ax.set_xticks(x)
        ax.set_xticklabels(participants, rotation=45, ha='right', fontsize=plt.rcParams['font.size'] - 1.5)
        ax.grid(True, which='both', axis='y', linestyle='--', alpha=0.5)
        ThesisStyle.style_ax(ax)

    def _plot_generalization_6grid(self, gen_all, gen_emg, gen_imu, par_all, par_emg, par_imu, output_dir, layout_width, metric="MAE"):
        ThesisStyle.apply(layout_width)
        
        # Grid dimensions: 3 rows x 2 columns (Left: Specialized, Right: LOPO)
        # Sized to match comp_regression_6grid exactly
        if layout_width == "column":
            figsize = (3.5, 3.4)
        elif layout_width == "double":
            figsize = (7.0, 6.2)
        else: # "default"
            figsize = (8.0, 7.1)
            
        fig, axes = plt.subplots(nrows=3, ncols=2, figsize=figsize)

        # Retrieve modalities mapping
        keys = ["all", "emg_only", "imu_only"]
        labels = ["Sensor-Fused (EMG + IMU)", "EMG-only", "IMU-only"]
        colors = [ThesisStyle.COLOR_FUSION, ThesisStyle.COLOR_EMG, ThesisStyle.COLOR_IMU]
        markers = ["o", "s", "^"]

        # Column Titles (Row 0 subplots) - Black in Fira Sans Bold
        axes[0, 0].set_title("Participant-Specialized", fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['font.size']), color='black', pad=8)
        axes[0, 1].set_title("Generalized (LOPO)", fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['font.size']), color='black', pad=8)

        # --------------------------------------------------
        # 1. Row 1: Participant Comparison
        # --------------------------------------------------
        par_part_errs = {}
        par_runs = {"all": par_all, "emg_only": par_emg, "imu_only": par_imu}
        for k, run in par_runs.items():
            if run:
                per_part = run.get("evaluation", {}).get("per_participant", [])
                for row in per_part:
                    p_id = row["Participant"]
                    if p_id not in par_part_errs:
                        par_part_errs[p_id] = {}
                    par_part_errs[p_id][k] = float(row[metric])

        gen_part_errs = {}
        gen_runs = {"all": gen_all, "emg_only": gen_emg, "imu_only": gen_imu}
        for k, run in gen_runs.items():
            if run:
                per_part = run.get("evaluation", {}).get("per_participant", [])
                for row in per_part:
                    p_id = row["Participant"]
                    if p_id not in gen_part_errs:
                        gen_part_errs[p_id] = {}
                    gen_part_errs[p_id][k] = float(row[metric])

        # Order the participants in ascending error, averaged over both plots for consistency
        def get_avg_error(p):
            par_err = par_part_errs.get(p, {}).get("all", 0.0)
            gen_err = gen_part_errs.get(p, {}).get("all", 0.0)
            return (par_err + gen_err) / 2.0

        all_participants = sorted(list(set(par_part_errs.keys()) | set(gen_part_errs.keys())), key=get_avg_error)
        
        par_vals_all = [par_part_errs.get(p, {}).get("all", 0.0) for p in all_participants]
        par_vals_emg = [par_part_errs.get(p, {}).get("emg_only", 0.0) for p in all_participants]
        par_vals_imu = [par_part_errs.get(p, {}).get("imu_only", 0.0) for p in all_participants]

        gen_vals_all = [gen_part_errs.get(p, {}).get("all", 0.0) for p in all_participants]
        gen_vals_emg = [gen_part_errs.get(p, {}).get("emg_only", 0.0) for p in all_participants]
        gen_vals_imu = [gen_part_errs.get(p, {}).get("imu_only", 0.0) for p in all_participants]

        def get_mean_metric(run_data):
            if not run_data: return 0.0
            eval_dict = run_data.get("evaluation", {})
            cpm = eval_dict.get("macro_metrics", {}).get("class_participant_macro", {}) or eval_dict.get("class_participant_macro", {})
            if cpm.get(metric) is not None:
                return cpm[metric]
            if "macro_avg" in eval_dict and eval_dict["macro_avg"] is not None:
                return eval_dict["macro_avg"].get(metric, 0.0)
            return eval_dict.get("pooled", {}).get(metric, 0.0)

        par_mean_all = float(np.mean(par_vals_all))
        par_mean_emg = float(np.mean(par_vals_emg))
        par_mean_imu = float(np.mean(par_vals_imu))

        gen_mean_all = float(np.mean(gen_vals_all))
        gen_mean_emg = float(np.mean(gen_vals_emg))
        gen_mean_imu = float(np.mean(gen_vals_imu))

        clip_val = 1.25

        # Plot Left (Specialized)
        ax_r1_l = axes[0, 0]
        self._plot_participant_bars_subplot(ax_r1_l, all_participants, par_vals_all, par_vals_emg, par_vals_imu, par_mean_all, par_mean_emg, par_mean_imu, clip_val=clip_val)
        ax_r1_l.set_ylabel(f"{metric} (kg)", labelpad=8)
        ax_r1_l.set_xlabel("Participant ID", labelpad=4)

        # Plot Right (Generalized)
        ax_r1_r = axes[0, 1]
        self._plot_participant_bars_subplot(ax_r1_r, all_participants, gen_vals_all, gen_vals_emg, gen_vals_imu, gen_mean_all, gen_mean_emg, gen_mean_imu, clip_val=clip_val)
        ax_r1_r.set_xlabel("Participant ID", labelpad=4)
        ax_r1_r.yaxis.set_tick_params(labelleft=False)

        # --------------------------------------------------
        # 2. Row 2: Seqlen Comparison
        # --------------------------------------------------
        def get_seqlen_data(run_data):
            if not run_data: return [], [], []
            per_seqlen = run_data.get("evaluation", {}).get("per_seqlen") or run_data.get("evaluation", {}).get("per_duration", [])
            filtered = [row for row in per_seqlen if int(row.get('Count', 0)) >= 50]
            times = []
            vals = []
            counts = []
            for row in filtered:
                t = float(row['TimeAtPrediction'].rstrip('s'))
                if t > 3.0:
                    continue
                times.append(t)
                vals.append(float(row[metric]))
                counts.append(int(row['Count']))
            return times, vals, counts

        all_seqlen_vals = []
        all_seqlen_counts = []
        
        par_seqlen_runs = {"all": par_all, "emg_only": par_emg, "imu_only": par_imu}
        par_seqlen_data = {}
        for k, run in par_seqlen_runs.items():
            if run:
                times, vals, counts = get_seqlen_data(run)
                par_seqlen_data[k] = (times, vals, counts)
                all_seqlen_vals.extend(vals)
                all_seqlen_counts.extend(counts)

        gen_seqlen_runs = {"all": gen_all, "emg_only": gen_emg, "imu_only": gen_imu}
        gen_seqlen_data = {}
        for k, run in gen_seqlen_runs.items():
            if run:
                times, vals, counts = get_seqlen_data(run)
                gen_seqlen_data[k] = (times, vals, counts)
                all_seqlen_vals.extend(vals)
                all_seqlen_counts.extend(counts)

        ylim_seqlen_val = max(0.8, max(all_seqlen_vals) * 1.15) if all_seqlen_vals else 0.8
        ylim_count_val = max(1000, max(all_seqlen_counts) * 3.0) if all_seqlen_counts else 3000

        # Plot Left (Specialized)
        ax_r2_l = axes[1, 0]
        ax_r2_l_twin = ax_r2_l.twinx()
        
        if "all" in par_seqlen_data and par_seqlen_data["all"][0]:
            t_all, _, c_all = par_seqlen_data["all"]
            bar_width = 0.5 * np.min(np.diff(t_all)) if len(t_all) > 1 else 0.1
            ax_r2_l_twin.bar(t_all, c_all, width=bar_width, color=ThesisStyle.RHO_LIGHT_BLUE, alpha=0.6,
                             edgecolor=ThesisStyle.COLOR_UNITY, linewidth=0.5, zorder=1)
        ax_r2_l_twin.set_ylim(0, ylim_count_val)
        ax_r2_l_twin.yaxis.set_tick_params(labelright=False)
        ax_r2_l_twin.spines['top'].set_visible(False)
        ax_r2_l_twin.spines['right'].set_visible(True)
        ax_r2_l_twin.spines['left'].set_visible(True)
        ax_r2_l_twin.grid(False)

        for k, label, color, marker in zip(keys, labels, colors, markers):
            if k in par_seqlen_data:
                t, m, _ = par_seqlen_data[k]
                ax_r2_l.plot(t, m, marker=marker, color=color, label=label, linewidth=1.0, markersize=3.0, zorder=3)
        ax_r2_l.set_xlabel("Segment Length (s)", labelpad=4)
        ax_r2_l.set_ylabel(f"{metric} (kg)", labelpad=8)
        ax_r2_l.set_ylim(0, ylim_seqlen_val)
        ax_r2_l.grid(True, linestyle='--', alpha=0.5)
        ThesisStyle.style_ax(ax_r2_l)
        ax_r2_l.set_zorder(ax_r2_l_twin.get_zorder() + 1)
        ax_r2_l.patch.set_visible(False)

        # Plot Right (Generalized)
        ax_r2_r = axes[1, 1]
        ax_r2_r_twin = ax_r2_r.twinx()
        
        if "all" in gen_seqlen_data and gen_seqlen_data["all"][0]:
            t_all, _, c_all = gen_seqlen_data["all"]
            bar_width = 0.5 * np.min(np.diff(t_all)) if len(t_all) > 1 else 0.1
            ax_r2_r_twin.bar(t_all, c_all, width=bar_width, color=ThesisStyle.RHO_LIGHT_BLUE, alpha=0.6,
                             edgecolor=ThesisStyle.COLOR_UNITY, linewidth=0.5, label='Segment Count', zorder=1)
        ax_r2_r_twin.set_ylabel("Segment Count", color=ThesisStyle.NEUTRAL_GRAY, labelpad=8)
        ax_r2_r_twin.tick_params(axis='y', labelcolor=ThesisStyle.NEUTRAL_GRAY)
        ax_r2_r_twin.set_ylim(0, ylim_count_val)
        ax_r2_r_twin.spines['top'].set_visible(False)
        ax_r2_r_twin.spines['right'].set_visible(True)
        ax_r2_r_twin.spines['left'].set_visible(True)
        ax_r2_r_twin.grid(False)

        for k, label, color, marker in zip(keys, labels, colors, markers):
            if k in gen_seqlen_data:
                t, m, _ = gen_seqlen_data[k]
                ax_r2_r.plot(t, m, marker=marker, color=color, label=label, linewidth=1.0, markersize=3.0, zorder=3)
        ax_r2_r.set_xlabel("Segment Length (s)", labelpad=4)
        ax_r2_r.yaxis.set_tick_params(labelleft=False)
        ax_r2_r.set_ylim(0, ylim_seqlen_val)
        ax_r2_r.grid(True, linestyle='--', alpha=0.5)
        ThesisStyle.style_ax(ax_r2_r)
        ax_r2_r.set_zorder(ax_r2_r_twin.get_zorder() + 1)
        ax_r2_r.patch.set_visible(False)

        # --------------------------------------------------
        # 3. Row 3: Weight Class Comparison
        # --------------------------------------------------
        def get_weight_data(run_data):
            if not run_data: return {}
            per_weight = run_data.get("evaluation", {}).get("per_weight", [])
            weight_vals = {}
            for row in per_weight:
                w_val = row['Weight'].replace(' kg', '').replace('kg', '').strip()
                weight_vals[w_val] = float(row[metric])
            return weight_vals

        par_weight_vals = {k: get_weight_data(run) for k, run in par_runs.items()}
        gen_weight_vals = {k: get_weight_data(run) for k, run in gen_runs.items()}

        all_weights_set = set()
        for k in keys:
            all_weights_set.update(par_weight_vals.get(k, {}).keys())
            all_weights_set.update(gen_weight_vals.get(k, {}).keys())
        sorted_weights = sorted(list(all_weights_set), key=lambda w: float(w))
        x_positions = np.arange(len(sorted_weights))

        all_weight_vals = []
        for k in keys:
            all_weight_vals.extend(par_weight_vals.get(k, {}).values())
            all_weight_vals.extend(gen_weight_vals.get(k, {}).values())
        ylim_weight_val = max(0.8, max(all_weight_vals) * 1.15) if all_weight_vals else 0.8

        # Plot Left (Specialized)
        ax_r3_l = axes[2, 0]
        for k, label, color, marker in zip(keys, labels, colors, markers):
            vals = []
            valid_x = []
            for idx, w in enumerate(sorted_weights):
                if w in par_weight_vals.get(k, {}):
                    vals.append(par_weight_vals[k][w])
                    valid_x.append(idx)
            if valid_x:
                ax_r3_l.plot(valid_x, vals, marker=marker, color=color, label=label, linewidth=1.0, markersize=3.0)
        ax_r3_l.set_xlabel("Weight Class (kg)", labelpad=4)
        ax_r3_l.set_ylabel(f"{metric} (kg)", labelpad=8)
        ax_r3_l.set_xticks(x_positions)
        ax_r3_l.set_xticklabels(sorted_weights)
        ax_r3_l.set_ylim(0, ylim_weight_val)
        ax_r3_l.grid(True, linestyle='--', alpha=0.5)
        ThesisStyle.style_ax(ax_r3_l)

        # Plot Right (Generalized)
        ax_r3_r = axes[2, 1]
        for k, label, color, marker in zip(keys, labels, colors, markers):
            vals = []
            valid_x = []
            for idx, w in enumerate(sorted_weights):
                if w in gen_weight_vals.get(k, {}):
                    vals.append(gen_weight_vals[k][w])
                    valid_x.append(idx)
            if valid_x:
                ax_r3_r.plot(valid_x, vals, marker=marker, color=color, label=label, linewidth=1.0, markersize=3.0)
        ax_r3_r.set_xlabel("Weight Class (kg)", labelpad=4)
        ax_r3_r.yaxis.set_tick_params(labelleft=False)
        ax_r3_r.set_xticks(x_positions)
        ax_r3_r.set_xticklabels(sorted_weights)
        ax_r3_r.set_ylim(0, ylim_weight_val)
        ax_r3_r.grid(True, linestyle='--', alpha=0.5)
        ThesisStyle.style_ax(ax_r3_r)

        # Style all axes spines (hide top and right) & Add rotated Row Labels on the left edge of first column
        row_labels = ["Participant", "Segment Length", "Weight Class"]
        for row_idx in range(3):
            for col_idx in range(2):
                ax = axes[row_idx, col_idx]
                ax.spines['top'].set_visible(False)
                ax.spines['right'].set_visible(False)
                
                # Row labels on the left edge of the first column
                if col_idx == 0:
                    ax.annotate(row_labels[row_idx], xy=(-0.24, 0.5), xycoords='axes fraction',
                                rotation=90, ha='center', va='center',
                                fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['axes.labelsize']),
                                color='black')
            
        # Add single global legend at the bottom of the grid
        lines_mae, labels_mae = ax_r3_l.get_legend_handles_labels()
        lines_twin, labels_twin = ax_r2_r_twin.get_legend_handles_labels()
        
        fig.legend(handles=lines_mae + lines_twin, labels=labels_mae + labels_twin, loc='lower center', ncol=4,
                   bbox_to_anchor=(0.5, 0.01),
                   frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)

        plt.tight_layout(rect=[0.01, 0.04, 1, 1.0])
        ThesisStyle.save_figure(fig, output_dir / f"comp_generalization_{metric.lower()}_6grid")
        plt.close(fig)

    # ------------------------------------------------------------------
    # Plot 1: Summary bar chart – MAE, RMSE, R², Gen Gap
    # ------------------------------------------------------------------
    def _plot_metrics_summary(self, gen_data, par_data, output_dir, layout_width):
        ThesisStyle.apply(layout_width)
        gen = self._get_gen_metrics(gen_data)
        par = self._get_par_metrics(par_data)

        metrics_left  = ["MAE", "RMSE"]          # left y-axis (kg)
        metrics_right = ["R2"]                    # right y-axis (score)

        fig, ax1 = plt.subplots()
        ax2 = ax1.twinx()

        x = np.arange(len(metrics_left + ["Gen Gap"]))
        width = 0.30

        # ── Left axis: MAE, RMSE, Gen Gap ──
        # Generalized bars
        gen_left_vals  = [gen["MAE"][0],  gen["RMSE"][0],  gen["Gap"][0]]
        gen_left_errs  = [gen["MAE"][1],  gen["RMSE"][1],  gen["Gap"][1]]
        par_left_vals  = [par["MAE"],     par["RMSE"],     None]  # No Gen Gap for par-spec

        rects_gen = ax1.bar(
            x - width / 2, gen_left_vals, width,
            label=self.LABEL_GEN, color=self.COLOR_GEN, alpha=0.85,
            edgecolor=self.COLOR_GEN, linewidth=0.5,
            yerr=gen_left_errs, capsize=3,
            error_kw={"elinewidth": 1.0, "ecolor": self.COLOR_GEN, "alpha": 0.7}
        )

        par_bars = [par["MAE"], par["RMSE"], 0.0]  # placeholder 0 for gap
        rects_par = ax1.bar(
            x + width / 2, par_bars, width,
            label=self.LABEL_PAR, color=self.COLOR_PAR, alpha=0.85,
            edgecolor=self.COLOR_PAR, linewidth=0.5
        )

        # Annotate gen bars (left axis)
        for rect, val, err in zip(rects_gen, gen_left_vals, gen_left_errs):
            ax1.annotate(f"{val:.3f}",
                         xy=(rect.get_x() + rect.get_width() / 2, val + err),
                         xytext=(0, 4), textcoords="offset points",
                         ha='center', va='bottom',
                         fontsize=plt.rcParams['font.size'] - 2.0,
                         color=self.COLOR_GEN, weight='semibold')

        # Annotate par bars (only MAE and RMSE, skip gap)
        for i, (rect, val) in enumerate(zip(rects_par, [par["MAE"], par["RMSE"], None])):
            if val is None:
                # Mark "N/A" for Gen Gap on par-spec bar
                ax1.text(rect.get_x() + rect.get_width() / 2, 0.01,
                         "N/A", ha='center', va='bottom',
                         fontsize=plt.rcParams['font.size'] - 2.5,
                         color=self.COLOR_PAR, weight='semibold', style='italic')
                rect.set_visible(False)
            else:
                ax1.annotate(f"{val:.3f}",
                             xy=(rect.get_x() + rect.get_width() / 2, val),
                             xytext=(0, 4), textcoords="offset points",
                             ha='center', va='bottom',
                             fontsize=plt.rcParams['font.size'] - 2.0,
                             color=self.COLOR_PAR, weight='semibold')

        # ── Right axis: R² ──
        r2_x = np.array([len(x)])   # position after Gen Gap
        r2_gen_val, r2_gen_err = gen["R2"]
        r2_par_val = par["R2"]

        rect_r2_gen = ax2.bar(
            r2_x - width / 2, [r2_gen_val], width,
            color=self.COLOR_GEN, alpha=0.85, edgecolor=self.COLOR_GEN, linewidth=0.5,
            yerr=[r2_gen_err], capsize=3,
            error_kw={"elinewidth": 1.0, "ecolor": self.COLOR_GEN, "alpha": 0.7}
        )
        rect_r2_par = ax2.bar(
            r2_x + width / 2, [r2_par_val], width,
            color=self.COLOR_PAR, alpha=0.85, edgecolor=self.COLOR_PAR, linewidth=0.5
        )
        ax2.annotate(f"{r2_gen_val:.3f}",
                     xy=(r2_x[0] - width / 2, r2_gen_val + r2_gen_err),
                     xytext=(0, 4), textcoords="offset points",
                     ha='center', va='bottom',
                     fontsize=plt.rcParams['font.size'] - 2.0,
                     color=self.COLOR_GEN, weight='semibold')
        ax2.annotate(f"{r2_par_val:.3f}",
                     xy=(r2_x[0] + width / 2, r2_par_val),
                     xytext=(0, 4), textcoords="offset points",
                     ha='center', va='bottom',
                     fontsize=plt.rcParams['font.size'] - 2.0,
                     color=self.COLOR_PAR, weight='semibold')

        # Zero baseline for gen gap (can be negative)
        ax1.axhline(0, color=ThesisStyle.COLOR_UNITY, linewidth=0.8, linestyle='--', alpha=0.6)

        # Axes labels
        ax1.set_ylabel("Error / Gen. Gap (kg)", labelpad=8)
        ax2.set_ylabel(r"$\text{R}^2$ Score", labelpad=8, color=ThesisStyle.COLOR_R2)
        ax2.tick_params(axis='y', labelcolor=ThesisStyle.COLOR_R2)
        ax1.set_xlabel("Metric", labelpad=8)

        # x-ticks across both axes
        all_x = np.append(x, r2_x)
        ax1.set_xticks(all_x)
        ax1.set_xticklabels(["MAE", "RMSE", "Gen Gap", r"$\text{R}^2$"])

        # Y limits
        left_vals_all = gen_left_vals + [gen_left_errs[i] + gen_left_vals[i] for i in range(3)]
        left_max = max(v for v in left_vals_all if v is not None) * 1.35
        left_min = min(min(gen_left_vals), 0) * 1.3
        ax1.set_ylim(left_min, left_max)
        ax2.set_ylim(0, 1.1)

        ax1.grid(True, axis='y', linestyle='--', alpha=0.5)
        ax2.grid(False)
        ax1.spines['top'].set_visible(False)
        ax2.spines['top'].set_visible(False)

        lines1, labels1 = ax1.get_legend_handles_labels()
        ax1.legend(lines1[:2], labels1[:2], loc='upper right', frameon=True,
                   facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)

        ThesisStyle.set_title(ax1, "Generalized vs. Participant-Specific: Performance Metrics")
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_dir / "comp_metrics_summary")
        plt.close(fig)

    # ------------------------------------------------------------------
    # Plot 2: Per-participant MAE (gen LOPO folds) + par-spec reference
    # ------------------------------------------------------------------
    def _plot_per_participant_mae(self, gen_data, par_data, output_dir, layout_width):
        ThesisStyle.apply(layout_width)

        per_part = gen_data.get("evaluation", {}).get("per_participant", [])
        if not per_part:
            return
        participants = [r["Participant"] for r in per_part]
        gen_maes = [r["MAE"] for r in per_part]
        par_pooled_mae = par_data.get("evaluation", {}).get("pooled", {}).get("MAE", None)

        sorted_pairs = sorted(zip(gen_maes, participants))
        gen_maes_sorted = [v for v, _ in sorted_pairs]
        participants_sorted = [p for _, p in sorted_pairs]

        fig, ax = plt.subplots()
        x = np.arange(len(participants_sorted))

        ax.bar(x, gen_maes_sorted, color=self.COLOR_GEN, alpha=0.85,
               edgecolor=self.COLOR_GEN, linewidth=0.5, label=self.LABEL_GEN)

        cpm = gen_data.get("evaluation", {}).get("macro_metrics", {}).get("class_participant_macro", {}) or gen_data.get("evaluation", {}).get("class_participant_macro", {})
        gen_mean = cpm.get("MAE") if cpm.get("MAE") is not None else gen_data.get("evaluation", {}).get("macro_avg", {}).get("MAE", float(np.mean(gen_maes_sorted)))
        ax.axhline(gen_mean, color=self.COLOR_GEN, linestyle='--', linewidth=1.0,
                   alpha=0.8, label=f"Mean Gen. (μ={gen_mean:.3f})")

        if par_pooled_mae is not None:
            ax.axhline(par_pooled_mae, color=self.COLOR_PAR, linestyle='--', linewidth=1.0,
                       alpha=0.8, label=f"Par-Spec Pooled MAE (μ={par_pooled_mae:.3f})")

        ax.set_ylabel("MAE (kg)", labelpad=8)
        ax.set_xlabel("Unseen Participant (LOPO fold)", labelpad=8)
        ax.set_xticks(x)
        ax.set_xticklabels(participants_sorted, rotation=45, ha='right')
        ax.set_ylim(0, max(gen_maes_sorted) * 1.30 + 0.5)
        ax.grid(True, axis='y', linestyle='--', alpha=0.5)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.legend(frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)

        ThesisStyle.set_title(ax, "Per-Participant MAE: Generalized LOPO vs. Participant-Specific Reference")
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_dir / "comp_per_participant_mae")
        plt.close(fig)

    # ------------------------------------------------------------------
    # Plot 3: Per-participant RMSE (gen LOPO folds) + par-spec reference
    # ------------------------------------------------------------------
    def _plot_per_participant_rmse(self, gen_data, par_data, output_dir, layout_width):
        ThesisStyle.apply(layout_width)

        per_part = gen_data.get("evaluation", {}).get("per_participant", [])
        if not per_part:
            return
        participants = [r["Participant"] for r in per_part]
        gen_rmses = [r["RMSE"] for r in per_part]
        par_pooled_rmse = par_data.get("evaluation", {}).get("pooled", {}).get("RMSE", None)

        sorted_pairs = sorted(zip(gen_rmses, participants))
        gen_rmses_sorted = [v for v, _ in sorted_pairs]
        participants_sorted = [p for _, p in sorted_pairs]

        fig, ax = plt.subplots()
        x = np.arange(len(participants_sorted))

        ax.bar(x, gen_rmses_sorted, color=self.COLOR_GEN, alpha=0.85,
               edgecolor=self.COLOR_GEN, linewidth=0.5, label=self.LABEL_GEN)

        cpm = gen_data.get("evaluation", {}).get("macro_metrics", {}).get("class_participant_macro", {}) or gen_data.get("evaluation", {}).get("class_participant_macro", {})
        gen_mean = cpm.get("RMSE") if cpm.get("RMSE") is not None else gen_data.get("evaluation", {}).get("macro_avg", {}).get("RMSE", float(np.mean(gen_rmses_sorted)))
        ax.axhline(gen_mean, color=self.COLOR_GEN, linestyle='--', linewidth=1.0,
                   alpha=0.8, label=f"Mean Gen. (μ={gen_mean:.3f})")

        if par_pooled_rmse is not None:
            ax.axhline(par_pooled_rmse, color=self.COLOR_PAR, linestyle='--', linewidth=1.0,
                       alpha=0.8, label=f"Par-Spec Pooled RMSE (μ={par_pooled_rmse:.3f})")

        ax.set_ylabel("RMSE (kg)", labelpad=8)
        ax.set_xlabel("Unseen Participant (LOPO fold)", labelpad=8)
        ax.set_xticks(x)
        ax.set_xticklabels(participants_sorted, rotation=45, ha='right')
        ax.set_ylim(0, max(gen_rmses_sorted) * 1.30 + 0.5)
        ax.grid(True, axis='y', linestyle='--', alpha=0.5)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.legend(frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)

        ThesisStyle.set_title(ax, "Per-Participant RMSE: Generalized LOPO vs. Participant-Specific Reference")
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_dir / "comp_per_participant_rmse")
        plt.close(fig)

    # ------------------------------------------------------------------
    # Plot 4: Generalization Gap per participant (gen only) + mean line
    # ------------------------------------------------------------------
    def _plot_gen_gap(self, gen_data, output_dir, layout_width):
        ThesisStyle.apply(layout_width)

        per_fold = gen_data.get("evaluation", {}).get("per_fold", [])
        if not per_fold:
            return

        participants = [row["val_participants"][0] if isinstance(row.get("val_participants"), list)
                        else str(row.get("val_participants", f"fold_{i}"))
                        for i, row in enumerate(per_fold)]
        gaps = [row.get("Generalization Gap (MAE)", 0.0) for row in per_fold]

        sorted_pairs = sorted(zip(gaps, participants))
        gaps_sorted = [v for v, _ in sorted_pairs]
        participants_sorted = [p for _, p in sorted_pairs]

        fig, ax = plt.subplots()
        x = np.arange(len(participants_sorted))

        bar_colors = [ThesisStyle.COLOR_GAP if g >= 0 else ThesisStyle.COLOR_RMSE
                      for g in gaps_sorted]
        ax.bar(x, gaps_sorted, color=bar_colors, alpha=0.85, linewidth=0.5)

        mean_gap = float(np.mean(gaps_sorted))
        ax.axhline(0, color=ThesisStyle.COLOR_UNITY, linewidth=0.8, linestyle='-', alpha=0.5)
        ax.axhline(mean_gap, color=ThesisStyle.COLOR_GAP, linestyle='--', linewidth=1.2,
                   alpha=0.85, label=f"Mean Gap (μ={mean_gap:.3f} kg)")

        ax.set_ylabel("Generalization Gap — MAE (kg)\n(Test MAE − Train MAE)", labelpad=8)
        ax.set_xlabel("Unseen Participant (LOPO fold)", labelpad=8)
        ax.set_xticks(x)
        ax.set_xticklabels(participants_sorted, rotation=45, ha='right')

        y_min = min(min(gaps_sorted), 0) * 1.3
        y_max = max(max(gaps_sorted), 0) * 1.35
        ax.set_ylim(y_min, y_max)
        ax.grid(True, axis='y', linestyle='--', alpha=0.5)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.legend(frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)

        ThesisStyle.set_title(ax, "Generalization Gap per LOPO Fold (Generalized Model)")
        plt.tight_layout()
        ThesisStyle.save_figure(fig, output_dir / "comp_gen_gap_per_fold")
        plt.close(fig)

    # ------------------------------------------------------------------
    # Importance helpers
    # ------------------------------------------------------------------
    def _load_subrun(self, base_run_dir, sub):
        """Load a specific sub-run (e.g. 'emg_only') from a multi-modality root dir."""
        path = Path(base_run_dir)
        if not path.is_absolute():
            script_dir = Path(__file__).resolve().parent
            for base in [Path.cwd(), script_dir.parent, script_dir]:
                candidate = base / base_run_dir
                if candidate.exists():
                    path = candidate
                    break
        json_path = path / sub / "run_data.json"
        if json_path.exists():
            with open(json_path) as f:
                return json.load(f)
        raise FileNotFoundError(f"Cannot find {sub}/run_data.json in {base_run_dir}")

    def _top_n(self, d, n=10):
        """Return top-n items from a dict sorted by value descending."""
        if not d:
            return [], []
        sorted_items = sorted(d.items(), key=lambda x: abs(x[1]), reverse=True)[:n]
        labels = [k for k, _ in sorted_items]
        values = [v for _, v in sorted_items]
        return labels, values

    # ------------------------------------------------------------------
    # Plot 5: DeepSHAP modality split – all 4 runs grouped
    # ------------------------------------------------------------------
    def _plot_deepshap_modality(self, all_data, output_dir, layout_width):
        ThesisStyle.apply(layout_width)

        # Retrieve the generalized (EMG + IMU) and participant-specialized data
        gen_data = all_data.get("EMG + IMU (Gen)")
        par_data = all_data.get("Par-Specific")

        if not gen_data or not par_data:
            print("  [Warning] Missing required runs for combined DeepSHAP modality comparison. Skipping.")
            return

        gen_mod = gen_data.get("feature_importance", {}).get("deepshap_modality", {})
        gen_chan = gen_data.get("feature_importance", {}).get("deepshap_channel", {})
        par_mod = par_data.get("feature_importance", {}).get("deepshap_modality", {})
        par_chan = par_data.get("feature_importance", {}).get("deepshap_channel", {})
        
        gen_perm = gen_data.get("feature_importance", {}).get("permutation_channel", {})
        par_perm = par_data.get("feature_importance", {}).get("permutation_channel", {})

        if not gen_mod or not gen_chan or not par_mod or not par_chan or not gen_perm or not par_perm:
            print("  [Warning] Missing deepshap_modality, deepshap_channel or permutation_channel in run data. Skipping.")
            return

        # Determine the size based on double the active layout size for double-column format (significantly less tall):
        if layout_width == "column":
            figsize = (3.5, 2.0)
        elif layout_width == "double":
            figsize = (7.0, 4.0)
        else: # "default"
            figsize = (8.0, 4.5)

        # Setup 3-subplot horizontal layout:
        # ax1: stacked bars of Modality Share (%)
        # ax2: Combined channel importance (all channels) using solid/dashed lines for DeepSHAP
        # ax3: Combined channel importance (all channels) using solid/dashed lines for Permutation Importance
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=figsize,
                                             gridspec_kw={'width_ratios': [1.6, 2.2, 2.2]})

        # --------------------------------------------------
        # 1. Left Panel: Stacked Modality Share (%)
        # --------------------------------------------------
        models = ["Participant-\nSpecialized", "Generalized\n(LOPO)"]
        
        gen_total = sum(gen_mod.values()) or 1.0
        par_total = sum(par_mod.values()) or 1.0
        
        emg_shares = [
            par_mod.get("EMG", 0.0) / par_total * 100,
            gen_mod.get("EMG", 0.0) / gen_total * 100
        ]
        imu_shares = [
            par_mod.get("IMU", 0.0) / par_total * 100,
            gen_mod.get("IMU", 0.0) / gen_total * 100
        ]
        anthro_shares = [
            par_mod.get("Anthro", 0.0) / par_total * 100,
            gen_mod.get("Anthro", 0.0) / gen_total * 100
        ]
        
        x_pos = np.arange(len(models))
        bar_width = 0.55
        
        bars_emg = ax1.bar(x_pos, emg_shares, bar_width, label="EMG", color=ThesisStyle.COLOR_EMG, alpha=0.85, edgecolor=ThesisStyle.COLOR_EMG, linewidth=0.5)
        bars_imu = ax1.bar(x_pos, imu_shares, bar_width, bottom=emg_shares, label="IMU", color=ThesisStyle.COLOR_IMU, alpha=0.85, edgecolor=ThesisStyle.COLOR_IMU, linewidth=0.5)
        
        bottom_anthro = [e + i for e, i in zip(emg_shares, imu_shares)]
        if sum(anthro_shares) > 0:
            bars_anthro = ax1.bar(x_pos, anthro_shares, bar_width, bottom=bottom_anthro, label="Anthro", color=ThesisStyle.COLOR_MEDIAN, alpha=0.85, edgecolor=ThesisStyle.COLOR_MEDIAN, linewidth=0.5)
            
        ax1.set_ylabel("Modality Share (%)", labelpad=8)
        ax1.set_xticks(x_pos)
        ax1.set_xticklabels(models, fontsize=plt.rcParams['font.size'] - 2.5, rotation=0, ha='center')
        ax1.set_ylim(0, 100)
        ax1.set_title("Modality Share of Total |SHAP|", fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['font.size']), color='black', pad=10)
        ax1.spines['top'].set_visible(False)
        ax1.spines['right'].set_visible(False)
        ax1.grid(True, axis='y', linestyle='--', alpha=0.5)
        ThesisStyle.style_ax(ax1)
        
        # Annotate percentages
        for idx in range(len(models)):
            if emg_shares[idx] > 5:
                ax1.text(idx, emg_shares[idx] / 2, f"{emg_shares[idx]:.1f}%", ha='center', va='center', color='white', fontweight='bold', fontsize=plt.rcParams['font.size'] - 1.0, rotation=90)
            if imu_shares[idx] > 5:
                ax1.text(idx, emg_shares[idx] + imu_shares[idx] / 2, f"{imu_shares[idx]:.1f}%", ha='center', va='center', color='white', fontweight='bold', fontsize=plt.rcParams['font.size'] - 1.0, rotation=90)
            if anthro_shares[idx] > 5:
                ax1.text(idx, bottom_anthro[idx] + anthro_shares[idx] / 2, f"{anthro_shares[idx]:.1f}%", ha='center', va='center', color='white', fontweight='bold', fontsize=plt.rcParams['font.size'] - 1.0, rotation=90)
                
        # Legend removed (moved to global legend)

        # --------------------------------------------------
        # 2. Middle Panel: Combined Channel Importance (DeepSHAP Relative Share %)
        # --------------------------------------------------
        # Compute relative values for DeepSHAP (Middle Panel)
        gen_chan_total = sum(gen_chan.values()) or 1.0
        par_chan_total = sum(par_chan.values()) or 1.0
        gen_rel = {k: v / gen_chan_total * 100 for k, v in gen_chan.items()}
        par_rel = {k: v / par_chan_total * 100 for k, v in par_chan.items()}
        
        # Compute relative values for Permutation (Right Panel)
        gen_perm_total = sum(gen_perm.values()) or 1.0
        par_perm_total = sum(par_perm.values()) or 1.0
        gen_perm_rel = {k: v / gen_perm_total * 100 for k, v in gen_perm.items()}
        par_perm_rel = {k: v / par_perm_total * 100 for k, v in par_perm.items()}

        # Sort channels by their average relative importance across both plots (metrics) and models
        all_channels_set = set(gen_rel.keys()) | set(par_rel.keys()) | set(gen_perm_rel.keys()) | set(par_perm_rel.keys())
        mean_scores = {}
        for ch in all_channels_set:
            mean_scores[ch] = (
                gen_rel.get(ch, 0.0) + 
                par_rel.get(ch, 0.0) + 
                gen_perm_rel.get(ch, 0.0) + 
                par_perm_rel.get(ch, 0.0)
            ) / 4.0
        sorted_channels = [x[0] for x in sorted(mean_scores.items(), key=lambda x: x[1], reverse=True)]
        
        # Build labels, values, and colors for Middle Panel (DeepSHAP)
        labels = []
        vals_gen = []
        vals_par = []
        colors = []
        for ch in sorted_channels:
            clean_lbl = clean_channel_label(ch)
            labels.append(clean_lbl)
            vals_gen.append(gen_rel.get(ch, 0.0))
            vals_par.append(par_rel.get(ch, 0.0))
            if "_EMG" in ch or any(m in ch for m in ["Deltoid", "Brachii", "Brachioradialis", "Ulnaris", "Radialis"]):
                colors.append(ThesisStyle.COLOR_EMG)
            else:
                colors.append(ThesisStyle.COLOR_IMU)
                
        y_pos = np.arange(len(labels))
        offset = 0.12
        
        # Plot Participant-Specialized (dashed lines + hollow circles)
        ax2.hlines(y_pos - offset, 0, vals_par, colors=colors, linestyles='--', linewidth=1.0, alpha=0.85)
        ax2.scatter(vals_par, y_pos - offset, facecolors='white', edgecolors=colors, marker='o', zorder=3, s=12, linewidths=1.0, alpha=0.85)
        
        # Plot Generalized (solid lines + solid circles)
        ax2.hlines(y_pos + offset, 0, vals_gen, colors=colors, linestyles='-', linewidth=1.0, alpha=0.85)
        ax2.scatter(vals_gen, y_pos + offset, color=colors, marker='o', zorder=3, s=12, alpha=0.85)
        
        ax2.set_yticks(y_pos)
        ax2.set_yticklabels(labels, fontsize=plt.rcParams['font.size'] - 1.5)
        ax2.invert_yaxis()
        ax2.set_xlabel("Relative Importance", labelpad=4)
        ax2.set_title("DeepSHAP Channel Importance", fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['font.size']), color='black', pad=10)
        ax2.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        ax2.spines['top'].set_visible(False)
        ax2.spines['right'].set_visible(False)
        ThesisStyle.style_ax(ax2)
        
        # Add custom legend representing the different model lines and channel modalities
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        custom_legend = [
            Line2D([0], [0], color='#555555', linestyle='-', marker='o', markerfacecolor='#555555', markersize=6, label='Generalized Model'),
            Line2D([0], [0], color='#555555', linestyle='--', marker='o', markerfacecolor='white', markeredgecolor='#555555', markeredgewidth=1.5, markersize=6, label='Participant-Specialized Model'),
            Patch(facecolor=ThesisStyle.COLOR_EMG, label='EMG Channel'),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label='IMU Channel')
        ]
        # Legend removed (moved to global legend)
        
        max_val = max(max(vals_gen), max(vals_par)) if vals_gen else 1.0
        ax2.set_xlim(0, max_val * 1.1)

        # --------------------------------------------------
        # 3. Right Panel: Combined Permutation Channel Importance (Relative Share %)
        # --------------------------------------------------
        # (Relative share dictionaries gen_perm_rel and par_perm_rel are already computed above)
        sorted_perm_channels = sorted_channels
        
        labels_perm = []
        vals_gen_perm = []
        vals_par_perm = []
        colors_perm = []
        
        for ch in sorted_perm_channels:
            clean_lbl = clean_channel_label(ch)
            labels_perm.append(clean_lbl)
            
            vals_gen_perm.append(gen_perm_rel.get(ch, 0.0))
            vals_par_perm.append(par_perm_rel.get(ch, 0.0))
            
            # Determine modality for coloring
            if "_EMG" in ch or any(m in ch for m in ["Deltoid", "Brachii", "Brachioradialis", "Ulnaris", "Radialis"]):
                colors_perm.append(ThesisStyle.COLOR_EMG)
            else:
                colors_perm.append(ThesisStyle.COLOR_IMU)
                
        y_pos_perm = np.arange(len(labels_perm))
        
        # Plot Participant-Specialized (dashed lines + hollow circles)
        ax3.hlines(y_pos_perm - offset, 0, vals_par_perm, colors=colors_perm, linestyles='--', linewidth=1.0, alpha=0.85)
        ax3.scatter(vals_par_perm, y_pos_perm - offset, facecolors='white', edgecolors=colors_perm, marker='o', s=12, linewidths=1.0, alpha=0.85, zorder=3)
        
        # Plot Generalized (solid lines + solid circles)
        ax3.hlines(y_pos_perm + offset, 0, vals_gen_perm, colors=colors_perm, linestyles='-', linewidth=1.0, alpha=0.85)
        ax3.scatter(vals_gen_perm, y_pos_perm + offset, color=colors_perm, marker='o', s=12, alpha=0.85, zorder=3)
        
        ax3.set_yticks(y_pos_perm)
        ax3.set_yticklabels(labels_perm, fontsize=plt.rcParams['font.size'] - 1.5)
        ax3.invert_yaxis()
        ax3.set_xlabel("Relative Importance", labelpad=4)
        ax3.set_title("Permutation Channel Importance", fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['font.size']), color='black', pad=10)
        ax3.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        ax3.spines['top'].set_visible(False)
        ax3.spines['right'].set_visible(False)
        ThesisStyle.style_ax(ax3)
        
        # Legend removed (moved to global legend)
        ax3.axvline(0, color='#333333', linestyle='-', linewidth=0.8, alpha=0.5)
        
        min_val_perm = min(min(vals_gen_perm), min(vals_par_perm)) if vals_gen_perm else 0.0
        max_val_perm = max(max(vals_gen_perm), max(vals_par_perm)) if vals_gen_perm else 1.0
        if min_val_perm < 0:
            range_val = max_val_perm - min_val_perm
            x_min = min_val_perm - range_val * 0.05
            x_max = max_val_perm + range_val * 0.05
        else:
            x_min = 0.0
            x_max = max_val_perm * 1.1
        ax3.set_xlim(x_min, x_max)

        # Add global legend at the bottom
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        global_legend_handles = [
            Line2D([0], [0], color='#555555', linestyle='--', marker='o', markerfacecolor='white', markeredgecolor='#555555', markeredgewidth=1.5, markersize=6, label='Participant-Specialized Model'),
            Line2D([0], [0], color='#555555', linestyle='-', marker='o', markerfacecolor='#555555', markersize=6, label='Generalized Model'),
            Patch(facecolor=ThesisStyle.COLOR_EMG, label='EMG'),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label='IMU')
        ]
        fig.legend(handles=global_legend_handles, loc='lower center', ncol=4,
                   bbox_to_anchor=(0.5, 0.03),
                   frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)

        # Title and layout (reserve bottom space for global legend and right side for labels)
        plt.tight_layout(rect=[0.02, 0.09, 0.94, 0.98])
        ThesisStyle.save_figure(fig, output_dir / "comp_deepshap_modality")
        plt.close(fig)

    def _plot_deepshap_modality_2(self, all_data, output_dir, layout_width):
        # This specific plot variant is designed to fit exactly one column in the LaTeX template
        # (width of 3.46 inches), so we force the single-column layout style.
        ThesisStyle.apply("column")

        # Retrieve the generalized (EMG + IMU) and participant-specialized data
        gen_data = all_data.get("EMG + IMU (Gen)")
        par_data = all_data.get("Par-Specific")

        if not gen_data or not par_data:
            print("  [Warning] Missing required runs for combined DeepSHAP modality comparison 2. Skipping.")
            return

        gen_chan = gen_data.get("feature_importance", {}).get("deepshap_channel", {})
        par_chan = par_data.get("feature_importance", {}).get("deepshap_channel", {})
        
        gen_perm = gen_data.get("feature_importance", {}).get("permutation_channel", {})
        par_perm = par_data.get("feature_importance", {}).get("permutation_channel", {})

        if not gen_chan or not par_chan or not gen_perm or not par_perm:
            print("  [Warning] Missing deepshap_channel or permutation_channel in run data. Skipping.")
            return

        # Explicitly define sizes for single-column layout (3.46 in wide, 4.5 in high)
        figsize = (3.46, 4.5)
        marker_size = 12
        ncol = 2
        bottom_margin = 0.13

        ytick_labelsize = plt.rcParams['ytick.labelsize']
        title_fontsize = plt.rcParams['axes.titlesize']
        label_fontsize = plt.rcParams['axes.labelsize']
        legend_fontsize = plt.rcParams['legend.fontsize']

        # Setup 2-subplot horizontal layout (sharing y-axis):
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=figsize, sharey=True)

        # --------------------------------------------------
        # Compute relative values for DeepSHAP (Left Panel)
        # --------------------------------------------------
        gen_chan_total = sum(gen_chan.values()) or 1.0
        par_chan_total = sum(par_chan.values()) or 1.0
        gen_rel = {k: v / gen_chan_total * 100 for k, v in gen_chan.items()}
        par_rel = {k: v / par_chan_total * 100 for k, v in par_chan.items()}
        
        # Compute relative values for Permutation (Right Panel)
        gen_perm_total = sum(gen_perm.values()) or 1.0
        par_perm_total = sum(par_perm.values()) or 1.0
        gen_perm_rel = {k: v / gen_perm_total * 100 for k, v in gen_perm.items()}
        par_perm_rel = {k: v / par_perm_total * 100 for k, v in par_perm.items()}

        # Sort channels by their average relative importance across both plots and models
        all_channels_set = set(gen_rel.keys()) | set(par_rel.keys()) | set(gen_perm_rel.keys()) | set(par_perm_rel.keys())
        mean_scores = {}
        for ch in all_channels_set:
            mean_scores[ch] = (
                gen_rel.get(ch, 0.0) + 
                par_rel.get(ch, 0.0) + 
                gen_perm_rel.get(ch, 0.0) + 
                par_perm_rel.get(ch, 0.0)
            ) / 4.0
        sorted_channels = [x[0] for x in sorted(mean_scores.items(), key=lambda x: x[1], reverse=True)]
        
        # Build labels, values, and colors for Left Panel (DeepSHAP)
        labels = []
        vals_gen = []
        vals_par = []
        colors = []
        for ch in sorted_channels:
            clean_lbl = clean_channel_label(ch)
            labels.append(clean_lbl)
            vals_gen.append(gen_rel.get(ch, 0.0))
            vals_par.append(par_rel.get(ch, 0.0))
            if "_EMG" in ch or any(m in ch for m in ["Deltoid", "Brachii", "Brachioradialis", "Ulnaris", "Radialis"]):
                colors.append(ThesisStyle.COLOR_EMG)
            else:
                colors.append(ThesisStyle.COLOR_IMU)
                
        y_pos = np.arange(len(labels))
        offset = 0.12
        
        # Plot Participant-Specialized (dashed lines + hollow circles)
        ax1.hlines(y_pos - offset, 0, vals_par, colors=colors, linestyles='--', linewidth=1.0, alpha=0.85)
        ax1.scatter(vals_par, y_pos - offset, facecolors='white', edgecolors=colors, marker='o', zorder=3, s=marker_size, linewidths=1.0, alpha=0.85)
        
        # Plot Generalized (solid lines + solid circles)
        ax1.hlines(y_pos + offset, 0, vals_gen, colors=colors, linestyles='-', linewidth=1.0, alpha=0.85)
        ax1.scatter(vals_gen, y_pos + offset, color=colors, marker='o', zorder=3, s=marker_size, alpha=0.85)
        
        # Configure Left Panel
        ax1.set_yticks(y_pos)
        ax1.set_yticklabels(labels, fontsize=ytick_labelsize, rotation=30, ha='right')
        ax1.invert_yaxis()
        ax1.set_title("DeepSHAP", fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=title_fontsize), color='black', pad=8)
        ax1.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        ax1.spines['top'].set_visible(False)
        ax1.spines['right'].set_visible(False)
        ax1.tick_params(axis='x', labelsize=ytick_labelsize)
        ThesisStyle.style_ax(ax1)
        
        max_val = max(max(vals_gen), max(vals_par)) if vals_gen else 1.0
        ax1.set_xlim(0, max_val * 1.1)

        # --------------------------------------------------
        # Plot Permutation (Right Panel)
        # --------------------------------------------------
        vals_gen_perm = []
        vals_par_perm = []
        colors_perm = []
        for ch in sorted_channels:
            vals_gen_perm.append(gen_perm_rel.get(ch, 0.0))
            vals_par_perm.append(par_perm_rel.get(ch, 0.0))
            if "_EMG" in ch or any(m in ch for m in ["Deltoid", "Brachii", "Brachioradialis", "Ulnaris", "Radialis"]):
                colors_perm.append(ThesisStyle.COLOR_EMG)
            else:
                colors_perm.append(ThesisStyle.COLOR_IMU)
                
        # Plot Participant-Specialized (dashed lines + hollow circles)
        ax2.hlines(y_pos - offset, 0, vals_par_perm, colors=colors_perm, linestyles='--', linewidth=1.0, alpha=0.85)
        ax2.scatter(vals_par_perm, y_pos - offset, facecolors='white', edgecolors=colors_perm, marker='o', s=marker_size, linewidths=1.0, alpha=0.85, zorder=3)
        
        # Plot Generalized (solid lines + solid circles)
        ax2.hlines(y_pos + offset, 0, vals_gen_perm, colors=colors_perm, linestyles='-', linewidth=1.0, alpha=0.85)
        ax2.scatter(vals_gen_perm, y_pos + offset, color=colors_perm, marker='o', s=marker_size, alpha=0.85, zorder=3)
        
        # Configure Right Panel
        ax2.set_title("Permutation", fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=title_fontsize), color='black', pad=8)
        ax2.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        ax2.spines['top'].set_visible(False)
        ax2.spines['right'].set_visible(False)
        ax2.tick_params(axis='x', labelsize=ytick_labelsize)
        ax2.yaxis.set_tick_params(labelleft=False) # explicitly hide shared y-axis labels on the right plot
        ThesisStyle.style_ax(ax2)
        
        ax2.axvline(0, color='#333333', linestyle='-', linewidth=0.8, alpha=0.5)
        
        min_val_perm = min(min(vals_gen_perm), min(vals_par_perm)) if vals_gen_perm else 0.0
        max_val_perm = max(max(vals_gen_perm), max(vals_par_perm)) if vals_gen_perm else 1.0
        if min_val_perm < 0:
            range_val = max_val_perm - min_val_perm
            x_min = min_val_perm - range_val * 0.05
            x_max = max_val_perm + range_val * 0.05
        else:
            x_min = 0.0
            x_max = max_val_perm * 1.1
        ax2.set_xlim(x_min, x_max)

        # Add global legend at the bottom
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        global_legend_handles = [
            Line2D([0], [0], color='#555555', linestyle='--', marker='o', markerfacecolor='white', markeredgecolor='#555555', markeredgewidth=1.0, markersize=5, label='Participant-Specialized Model'),
            Line2D([0], [0], color='#555555', linestyle='-', marker='o', markerfacecolor='#555555', markersize=5, label='Generalized Model'),
            Patch(facecolor=ThesisStyle.COLOR_EMG, label='EMG'),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label='IMU')
        ]
        # Adjust margins to fit all labels and the legend perfectly (left margin reduced since labels are diagonalized)
        plt.tight_layout(rect=[0.02, bottom_margin, 0.98, 0.98])

        # Force a canvas draw to calculate the final layout coordinates of subplots
        fig.canvas.draw()

        # Get subplot boundaries to find the exact center of the plotting area (excluding y-axis labels on the left)
        bbox1 = ax1.get_position()
        bbox2 = ax2.get_position()
        x_center = (bbox1.x0 + bbox2.x1) / 2.0

        # Add global legend at the bottom centered under the plotting area
        fig.legend(handles=global_legend_handles, loc='upper center', ncol=ncol,
                   bbox_to_anchor=(x_center, 0.12),
                   frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95,
                   fontsize=legend_fontsize)

        # Add a single centered shared x-axis label centered under the plotting area
        fig.text(x_center, 0.135, "Normalized Importance", ha='center', va='center', fontsize=label_fontsize)

        # Save figure
        ThesisStyle.save_figure(fig, output_dir / "comp_deepshap_modality_2")
        plt.close(fig)

        # Restore original style configuration to prevent affecting other plots in the loop
        ThesisStyle.apply(layout_width)

    def _plot_deepshap_feature(self, all_data, output_dir, layout_width):
        ThesisStyle.apply(layout_width)

        # Retrieve the generalized (EMG + IMU) and participant-specialized data
        gen_data = all_data.get("EMG + IMU (Gen)")
        par_data = all_data.get("Par-Specific")

        if not gen_data or not par_data:
            print("  [Warning] Missing required runs for combined DeepSHAP feature comparison. Skipping.")
            return

        gen_mod = gen_data.get("feature_importance", {}).get("deepshap_modality", {})
        par_mod = par_data.get("feature_importance", {}).get("deepshap_modality", {})

        gen_feat = gen_data.get("feature_importance", {}).get("deepshap_feature_type", {})
        par_feat = par_data.get("feature_importance", {}).get("deepshap_feature_type", {})
        
        gen_perm = gen_data.get("feature_importance", {}).get("permutation_feature", {})
        par_perm = par_data.get("feature_importance", {}).get("permutation_feature", {})

        if not gen_mod or not gen_feat or not par_mod or not par_feat or not gen_perm or not par_perm:
            print("  [Warning] Missing deepshap_modality, deepshap_feature_type or permutation_feature in run data. Skipping.")
            return

        # Determine the size based on double the active layout size for double-column format:
        if layout_width == "column":
            figsize = (7.0, 3.2)
        elif layout_width == "double":
            figsize = (14.0, 6.0)
        else: # "default"
            figsize = (12.0, 5.2)

        # Setup 3-subplot horizontal layout:
        # ax1: stacked bars of Modality Share (%)
        # ax2: DeepSHAP Feature-Type Importance
        # ax3: Permutation Feature-Type Importance
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=figsize,
                                             gridspec_kw={'width_ratios': [1.6, 2.2, 2.2]})

        # --------------------------------------------------
        # 1. Left Panel: Stacked Modality Share (%)
        # --------------------------------------------------
        models = ["Participant-\nSpecialized", "Generalized\n(LOPO)"]
        
        gen_total = sum(gen_mod.values()) or 1.0
        par_total = sum(par_mod.values()) or 1.0
        
        emg_shares = [
            par_mod.get("EMG", 0.0) / par_total * 100,
            gen_mod.get("EMG", 0.0) / gen_total * 100
        ]
        imu_shares = [
            par_mod.get("IMU", 0.0) / par_total * 100,
            gen_mod.get("IMU", 0.0) / gen_total * 100
        ]
        anthro_shares = [
            par_mod.get("Anthro", 0.0) / par_total * 100,
            gen_mod.get("Anthro", 0.0) / gen_total * 100
        ]
        
        x_pos = np.arange(len(models))
        bar_width = 0.55
        
        bars_emg = ax1.bar(x_pos, emg_shares, bar_width, label="EMG", color=ThesisStyle.COLOR_EMG, alpha=0.85, edgecolor=ThesisStyle.COLOR_EMG, linewidth=0.5)
        bars_imu = ax1.bar(x_pos, imu_shares, bar_width, bottom=emg_shares, label="IMU", color=ThesisStyle.COLOR_IMU, alpha=0.85, edgecolor=ThesisStyle.COLOR_IMU, linewidth=0.5)
        
        bottom_anthro = [e + i for e, i in zip(emg_shares, imu_shares)]
        if sum(anthro_shares) > 0:
            bars_anthro = ax1.bar(x_pos, anthro_shares, bar_width, bottom=bottom_anthro, label="Anthro", color=ThesisStyle.COLOR_MEDIAN, alpha=0.85, edgecolor=ThesisStyle.COLOR_MEDIAN, linewidth=0.5)
            
        ax1.set_ylabel("Modality Share (%)", labelpad=8)
        ax1.set_xticks(x_pos)
        ax1.set_xticklabels(models, fontsize=plt.rcParams['font.size'] - 1.5, rotation=0, ha='center')
        ax1.set_ylim(0, 100)
        ThesisStyle.set_title(ax1, "Modality Share of Total |SHAP|")
        ax1.spines['top'].set_visible(False)
        ax1.spines['right'].set_visible(False)
        ax1.grid(True, axis='y', linestyle='--', alpha=0.5)
        
        # Annotate percentages
        for idx in range(len(models)):
            if emg_shares[idx] > 5:
                ax1.text(idx, emg_shares[idx] / 2, f"{emg_shares[idx]:.1f}%", ha='center', va='center', color='white', fontweight='bold', fontsize=plt.rcParams['font.size'] - 1.0, rotation=90)
            if imu_shares[idx] > 5:
                ax1.text(idx, emg_shares[idx] + imu_shares[idx] / 2, f"{imu_shares[idx]:.1f}%", ha='center', va='center', color='white', fontweight='bold', fontsize=plt.rcParams['font.size'] - 1.0, rotation=90)
            if anthro_shares[idx] > 5:
                ax1.text(idx, bottom_anthro[idx] + anthro_shares[idx] / 2, f"{anthro_shares[idx]:.1f}%", ha='center', va='center', color='white', fontweight='bold', fontsize=plt.rcParams['font.size'] - 1.0, rotation=90)

        # --------------------------------------------------
        # Helper for Feature Cleaning & Parsing
        # --------------------------------------------------
        def clean_feature_label(raw_lbl):
            clean_lbl = raw_lbl.replace("EMG_", "EMG ").replace("IMU_", "IMU ")
            full_names = {
                "MAV": "Mean Absolute Value (MAV)",
                "RMS": "Root Mean Square (RMS)",
                "WL": "Waveform Length (WL)",
                "ZC": "Zero Crossings (ZC)",
                "SSC": "Slope Sign Changes (SSC)",
                "VAR": "Variance (VAR)",
                "WAMP": "Willison Amplitude (WAMP)",
                "IEMG": "Integrated EMG (IEMG)",
                "LogDet": "Log Detector (LogDet)",
                "Skew": "Skewness",
                "Kurt": "Kurtosis",
                "HjMob": "Hjorth Mobility",
                "HjComp": "Hjorth Complexity",
                "Myopulse": "Myopulse Rate",
                "MNF": "Mean Frequency (MNF)",
                "MDF": "Median Frequency (MDF)",
                "Power": "Total Power",
                "SpecEntropy": "Spectral Entropy",
                "PeakFreq": "Peak Frequency",
                "BW": "Bandwidth",
                "DomFreq": "Dominant Frequency",
                "Jerk": "Absolute Jerk",
                "SMA": "Signal Magnitude Area (SMA)",
                "P2P": "Peak-to-Peak (P2P)",
                "SpecEnergy": "Spectral Energy",
                "SVM_Mean": "Signal Vector Magnitude (Mean)"
            }
            parts = clean_lbl.split(" ")
            prefix = parts[0]
            suffix = " ".join(parts[1:])
            return f"{prefix} {full_names.get(suffix, suffix)}", prefix

        # --------------------------------------------------
        # 2. Middle Panel: Combined Feature-Type Importance (DeepSHAP Relative Share %)
        # --------------------------------------------------
        gen_feat_total = sum(gen_feat.values()) or 1.0
        par_feat_total = sum(par_feat.values()) or 1.0
        gen_feat_rel = {k: v / gen_feat_total * 100 for k, v in gen_feat.items()}
        par_feat_rel = {k: v / par_feat_total * 100 for k, v in par_feat.items()}
        
        gen_perm_total = sum(gen_perm.values()) or 1.0
        par_perm_total = sum(par_perm.values()) or 1.0
        gen_perm_rel = {k: v / gen_perm_total * 100 for k, v in gen_perm.items()}
        par_perm_rel = {k: v / par_perm_total * 100 for k, v in par_perm.items()}

        # Sort features by their average relative importance across both metrics and models
        all_features_set = set(gen_feat_rel.keys()) | set(par_feat_rel.keys()) | set(gen_perm_rel.keys()) | set(par_perm_rel.keys())
        mean_scores = {}
        for feat in all_features_set:
            mean_scores[feat] = (
                gen_feat_rel.get(feat, 0.0) + 
                par_feat_rel.get(feat, 0.0) + 
                gen_perm_rel.get(feat, 0.0) + 
                par_perm_rel.get(feat, 0.0)
            ) / 4.0
        sorted_features = [x[0] for x in sorted(mean_scores.items(), key=lambda x: x[1], reverse=True)]
        
        # Keep top 15 features
        top_n = min(15, len(sorted_features))
        sorted_features = sorted_features[:top_n]
        
        # DeepSHAP Panel
        labels = []
        vals_gen = []
        vals_par = []
        colors = []
        for feat in sorted_features:
            refined_lbl, prefix = clean_feature_label(feat)
            labels.append(refined_lbl)
            vals_gen.append(gen_feat_rel.get(feat, 0.0))
            vals_par.append(par_feat_rel.get(feat, 0.0))
            colors.append(ThesisStyle.COLOR_EMG if prefix == "EMG" else ThesisStyle.COLOR_IMU)
                
        y_pos = np.arange(len(labels))
        offset = 0.15
        
        # Plot Participant-Specialized (dashed lines + hollow circles)
        ax2.hlines(y_pos - offset, 0, vals_par, colors=colors, linestyles='--', linewidth=1.5, alpha=0.85)
        ax2.scatter(vals_par, y_pos - offset, facecolors='white', edgecolors=colors, marker='o', zorder=3, s=30, linewidths=1.5, alpha=0.85)
        
        # Plot Generalized (solid lines + solid circles)
        ax2.hlines(y_pos + offset, 0, vals_gen, colors=colors, linestyles='-', linewidth=1.5, alpha=0.85)
        ax2.scatter(vals_gen, y_pos + offset, color=colors, marker='o', zorder=3, s=30, alpha=0.85)
        
        ax2.set_yticks(y_pos)
        ax2.set_yticklabels(labels, fontsize=plt.rcParams['font.size'] - 1.5)
        ax2.invert_yaxis()
        ax2.set_xlabel("Relative Importance", labelpad=4)
        ThesisStyle.set_title(ax2, "DeepSHAP Feature-Type Importance")
        ax2.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        ax2.spines['top'].set_visible(False)
        ax2.spines['right'].set_visible(False)
        
        max_val = max(max(vals_gen), max(vals_par)) if vals_gen else 1.0
        ax2.set_xlim(0, max_val * 1.1)

        # --------------------------------------------------
        # 3. Right Panel: Combined Permutation Feature Importance (Relative Share %)
        # --------------------------------------------------
        labels_perm = []
        vals_gen_perm = []
        vals_par_perm = []
        colors_perm = []
        
        for feat in sorted_features:
            refined_lbl, prefix = clean_feature_label(feat)
            labels_perm.append(refined_lbl)
            vals_gen_perm.append(gen_perm_rel.get(feat, 0.0))
            vals_par_perm.append(par_perm_rel.get(feat, 0.0))
            colors_perm.append(ThesisStyle.COLOR_EMG if prefix == "EMG" else ThesisStyle.COLOR_IMU)
                
        y_pos_perm = np.arange(len(labels_perm))
        
        # Plot Participant-Specialized (dashed lines + hollow circles)
        ax3.hlines(y_pos_perm - offset, 0, vals_par_perm, colors=colors_perm, linestyles='--', linewidth=1.5, alpha=0.85)
        ax3.scatter(vals_par_perm, y_pos_perm - offset, facecolors='white', edgecolors=colors_perm, marker='o', s=30, linewidths=1.5, alpha=0.85, zorder=3)
        
        # Plot Generalized (solid lines + solid circles)
        ax3.hlines(y_pos_perm + offset, 0, vals_gen_perm, colors=colors_perm, linestyles='-', linewidth=1.5, alpha=0.85)
        ax3.scatter(vals_gen_perm, y_pos_perm + offset, color=colors_perm, marker='o', s=30, alpha=0.85, zorder=3)
        
        ax3.set_yticks(y_pos_perm)
        ax3.set_yticklabels(labels_perm, fontsize=plt.rcParams['font.size'] - 1.5)
        ax3.invert_yaxis()
        ax3.set_xlabel("Relative Importance", labelpad=4)
        ThesisStyle.set_title(ax3, "Permutation Feature-Type Importance")
        ax3.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        ax3.spines['top'].set_visible(False)
        ax3.spines['right'].set_visible(False)
        
        ax3.axvline(0, color='#333333', linestyle='-', linewidth=0.8, alpha=0.5)
        
        min_val_perm = min(min(vals_gen_perm), min(vals_par_perm)) if vals_gen_perm else 0.0
        max_val_perm = max(max(vals_gen_perm), max(vals_par_perm)) if vals_gen_perm else 1.0
        if min_val_perm < 0:
            range_val = max_val_perm - min_val_perm
            x_min = min_val_perm - range_val * 0.05
            x_max = max_val_perm + range_val * 0.05
        else:
            x_min = 0.0
            x_max = max_val_perm * 1.1
        ax3.set_xlim(x_min, x_max)

        # Add global legend at the bottom
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        global_legend_handles = [
            Line2D([0], [0], color='#555555', linestyle='--', marker='o', markerfacecolor='white', markeredgecolor='#555555', markeredgewidth=1.5, markersize=6, label='Participant-Specialized Model'),
            Line2D([0], [0], color='#555555', linestyle='-', marker='o', markerfacecolor='#555555', markersize=6, label='Generalized Model'),
            Patch(facecolor=ThesisStyle.COLOR_EMG, label='EMG Feature'),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label='IMU Feature')
        ]
        fig.legend(handles=global_legend_handles, loc='lower center', ncol=4,
                   bbox_to_anchor=(0.5, 0.01),
                   frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)

        # Title and layout (reserve bottom space for global legend and right side for labels)
        plt.tight_layout(rect=[0.02, 0.10, 0.95, 0.98])
        ThesisStyle.save_figure(fig, output_dir / "comp_deepshap_feature")
        plt.close(fig)

    def _plot_deepshap_all(self, gen_all, gen_emg, gen_imu, par_all, par_emg, par_imu, output_dir, layout_width):
        ThesisStyle.apply(layout_width)

        # Retrieve modality shares
        gen_all_mod = gen_all.get("feature_importance", {}).get("deepshap_modality", {}) if gen_all else {}
        par_all_mod = par_all.get("feature_importance", {}).get("deepshap_modality", {}) if par_all else {}
        
        def get_mod_share(run, modality):
            if not run: return {}
            m = run.get("feature_importance", {}).get("deepshap_modality", {})
            if m: return m
            c = run.get("feature_importance", {}).get("deepshap_channel", {})
            if c: return {modality: sum(c.values())}
            return {modality: 1.0}
            
        gen_emg_mod = get_mod_share(gen_emg, "EMG")
        par_emg_mod = get_mod_share(par_emg, "EMG")
        gen_imu_mod = get_mod_share(gen_imu, "IMU")
        par_imu_mod = get_mod_share(par_imu, "IMU")

        # Set up fig size based on layout width (height increased to give more vertical channel spacing)
        if layout_width == "column":
            figsize = (7.0, 4.0)
        elif layout_width == "double":
            figsize = (14.0, 8.0)
        else: # "default"
            figsize = (12.0, 7.0)

        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=figsize,
                                             gridspec_kw={'width_ratios': [1.6, 2.2, 2.2]})

        # 1. Left Panel: Stacked Modality Share (%) for Fused Model only
        models = ["Participant-\nSpecialized", "Generalized\n(LOPO)"]
        
        gen_total = sum(gen_all_mod.values()) or 1.0
        par_total = sum(par_all_mod.values()) or 1.0
        
        emg_shares = [
            par_all_mod.get("EMG", 0.0) / par_total * 100,
            gen_all_mod.get("EMG", 0.0) / gen_total * 100
        ]
        imu_shares = [
            par_all_mod.get("IMU", 0.0) / par_total * 100,
            gen_all_mod.get("IMU", 0.0) / gen_total * 100
        ]
        anthro_shares = [
            par_all_mod.get("Anthro", 0.0) / par_total * 100,
            gen_all_mod.get("Anthro", 0.0) / gen_total * 100
        ]
        
        x_pos = np.arange(len(models))
        bar_width = 0.55
        
        bars_emg = ax1.bar(x_pos, emg_shares, bar_width, label="EMG", color=ThesisStyle.COLOR_EMG, alpha=0.85, edgecolor=ThesisStyle.COLOR_EMG, linewidth=0.5)
        bars_imu = ax1.bar(x_pos, imu_shares, bar_width, bottom=emg_shares, label="IMU", color=ThesisStyle.COLOR_IMU, alpha=0.85, edgecolor=ThesisStyle.COLOR_IMU, linewidth=0.5)
        
        bottom_anthro = [e + i for e, i in zip(emg_shares, imu_shares)]
        if sum(anthro_shares) > 0:
            bars_anthro = ax1.bar(x_pos, anthro_shares, bar_width, bottom=bottom_anthro, label="Anthro", color=ThesisStyle.COLOR_MEDIAN, alpha=0.85, edgecolor=ThesisStyle.COLOR_MEDIAN, linewidth=0.5)
            
        ax1.set_ylabel("Modality Share (%)", labelpad=8)
        ax1.set_xticks(x_pos)
        ax1.set_xticklabels(models, fontsize=plt.rcParams['font.size'] - 1.5, rotation=0, ha='center')
        ax1.set_ylim(0, 100)
        ThesisStyle.set_title(ax1, "Modality Share of Total |SHAP|")
        ax1.spines['top'].set_visible(False)
        ax1.spines['right'].set_visible(False)
        ax1.grid(True, axis='y', linestyle='--', alpha=0.5)
        
        for idx in range(len(models)):
            if emg_shares[idx] > 5:
                ax1.text(idx, emg_shares[idx] / 2, f"{emg_shares[idx]:.1f}%", ha='center', va='center', color='white', fontweight='bold', fontsize=plt.rcParams['font.size'] - 1.5, rotation=90)
            if imu_shares[idx] > 5:
                ax1.text(idx, emg_shares[idx] + imu_shares[idx] / 2, f"{imu_shares[idx]:.1f}%", ha='center', va='center', color='white', fontweight='bold', fontsize=plt.rcParams['font.size'] - 1.5, rotation=90)
            if anthro_shares[idx] > 5:
                ax1.text(idx, bottom_anthro[idx] + anthro_shares[idx] / 2, f"{anthro_shares[idx]:.1f}%", ha='center', va='center', color='white', fontweight='bold', fontsize=plt.rcParams['font.size'] - 1.5, rotation=90)

        # 2. Extract Relative Importances for Channel Plotting
        def to_rel(d):
            if not d: return {}
            tot = sum(d.values()) or 1.0
            return {k: v / tot * 100 for k, v in d.items()}
            
        gen_all_rel = to_rel(gen_all.get("feature_importance", {}).get("deepshap_channel", {})) if gen_all else {}
        par_all_rel = to_rel(par_all.get("feature_importance", {}).get("deepshap_channel", {})) if par_all else {}
        gen_emg_rel = to_rel(gen_emg.get("feature_importance", {}).get("deepshap_channel", {})) if gen_emg else {}
        par_emg_rel = to_rel(par_emg.get("feature_importance", {}).get("deepshap_channel", {})) if par_emg else {}
        gen_imu_rel = to_rel(gen_imu.get("feature_importance", {}).get("deepshap_channel", {})) if gen_imu else {}
        par_imu_rel = to_rel(par_imu.get("feature_importance", {}).get("deepshap_channel", {})) if par_imu else {}
        
        gen_all_perm_rel = to_rel(gen_all.get("feature_importance", {}).get("permutation_channel", {})) if gen_all else {}
        par_all_perm_rel = to_rel(par_all.get("feature_importance", {}).get("permutation_channel", {})) if par_all else {}
        gen_emg_perm_rel = to_rel(gen_emg.get("feature_importance", {}).get("permutation_channel", {})) if gen_emg else {}
        par_emg_perm_rel = to_rel(par_emg.get("feature_importance", {}).get("permutation_channel", {})) if par_emg else {}
        gen_imu_perm_rel = to_rel(gen_imu.get("feature_importance", {}).get("permutation_channel", {})) if gen_imu else {}
        par_imu_perm_rel = to_rel(par_imu.get("feature_importance", {}).get("permutation_channel", {})) if par_imu else {}

        # Get all channels from Fused model
        all_channels_set = set(gen_all_rel.keys()) | set(par_all_rel.keys())
        mean_scores = {}
        for ch in all_channels_set:
            mean_scores[ch] = (gen_all_rel.get(ch, 0.0) + par_all_rel.get(ch, 0.0)) / 2.0
        sorted_channels = [x[0] for x in sorted(mean_scores.items(), key=lambda x: x[1], reverse=True)]

        labels = []
        colors = []
        
        # DeepSHAP values lists
        ds_fused_gen = []
        ds_fused_spec = []
        ds_mod_gen = []
        ds_mod_spec = []
        
        # Permutation values lists
        perm_fused_gen = []
        perm_fused_spec = []
        perm_mod_gen = []
        perm_mod_spec = []
        
        for ch in sorted_channels:
            clean_lbl = clean_channel_label(ch)
            labels.append(clean_lbl)
            
            is_emg = "_EMG" in ch or any(m in ch for m in ["Deltoid", "Brachii", "Brachioradialis", "Ulnaris", "Radialis"])
            color = ThesisStyle.COLOR_EMG if is_emg else ThesisStyle.COLOR_IMU
            colors.append(color)
            
            # DeepSHAP
            ds_fused_gen.append(gen_all_rel.get(ch, 0.0))
            ds_fused_spec.append(par_all_rel.get(ch, 0.0))
            if is_emg:
                ds_mod_gen.append(gen_emg_rel.get(ch, 0.0))
                ds_mod_spec.append(par_emg_rel.get(ch, 0.0))
            else:
                ds_mod_gen.append(gen_imu_rel.get(ch, 0.0))
                ds_mod_spec.append(par_imu_rel.get(ch, 0.0))
                
            # Permutation
            perm_fused_gen.append(gen_all_perm_rel.get(ch, 0.0))
            perm_fused_spec.append(par_all_perm_rel.get(ch, 0.0))
            if is_emg:
                perm_mod_gen.append(gen_emg_perm_rel.get(ch, 0.0))
                perm_mod_spec.append(par_emg_perm_rel.get(ch, 0.0))
            else:
                perm_mod_gen.append(gen_imu_perm_rel.get(ch, 0.0))
                perm_mod_spec.append(par_imu_perm_rel.get(ch, 0.0))

        y_pos = np.arange(len(labels))
        off1, off2, off3, off4 = -0.18, -0.06, 0.06, 0.18

        # 3. Middle Panel: DeepSHAP Channel Importance
        # Fused Spec
        ax2.hlines(y_pos + off1, 0, ds_fused_spec, colors=colors, linestyles='--', linewidth=1.2, alpha=0.8)
        ax2.scatter(ds_fused_spec, y_pos + off1, facecolors='white', edgecolors=colors, marker='o', zorder=3, s=20, linewidths=1.2, alpha=0.8)
        # Fused Gen
        ax2.hlines(y_pos + off2, 0, ds_fused_gen, colors=colors, linestyles='-', linewidth=1.2, alpha=0.8)
        ax2.scatter(ds_fused_gen, y_pos + off2, color=colors, marker='o', zorder=3, s=20, alpha=0.8)
        # Modality-only Spec
        ax2.hlines(y_pos + off3, 0, ds_mod_spec, colors=colors, linestyles='-.', linewidth=1.2, alpha=0.8)
        ax2.scatter(ds_mod_spec, y_pos + off3, facecolors='white', edgecolors=colors, marker='s', zorder=3, s=18, linewidths=1.2, alpha=0.8)
        # Modality-only Gen
        ax2.hlines(y_pos + off4, 0, ds_mod_gen, colors=colors, linestyles=':', linewidth=1.2, alpha=0.8)
        ax2.scatter(ds_mod_gen, y_pos + off4, color=colors, marker='s', zorder=3, s=18, alpha=0.8)

        ax2.set_yticks(y_pos)
        ax2.set_yticklabels(labels, fontsize=plt.rcParams['font.size'] - 1.5)
        ax2.invert_yaxis()
        ax2.set_xlabel("Relative Importance", labelpad=4)
        ThesisStyle.set_title(ax2, "DeepSHAP Channel Importance")
        ax2.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        ax2.spines['top'].set_visible(False)
        ax2.spines['right'].set_visible(False)
        max_val_ds = max(max(ds_fused_gen + ds_fused_spec + ds_mod_gen + ds_mod_spec), 1.0)
        ax2.set_xlim(0, max_val_ds * 1.1)

        # 4. Right Panel: Permutation Channel Importance
        # Fused Spec
        ax3.hlines(y_pos + off1, 0, perm_fused_spec, colors=colors, linestyles='--', linewidth=1.2, alpha=0.8)
        ax3.scatter(perm_fused_spec, y_pos + off1, facecolors='white', edgecolors=colors, marker='o', zorder=3, s=20, linewidths=1.2, alpha=0.8)
        # Fused Gen
        ax3.hlines(y_pos + off2, 0, perm_fused_gen, colors=colors, linestyles='-', linewidth=1.2, alpha=0.8)
        ax3.scatter(perm_fused_gen, y_pos + off2, color=colors, marker='o', zorder=3, s=20, alpha=0.8)
        # Modality-only Spec
        ax3.hlines(y_pos + off3, 0, perm_mod_spec, colors=colors, linestyles='-.', linewidth=1.2, alpha=0.8)
        ax3.scatter(perm_mod_spec, y_pos + off3, facecolors='white', edgecolors=colors, marker='s', zorder=3, s=18, linewidths=1.2, alpha=0.8)
        # Modality-only Gen
        ax3.hlines(y_pos + off4, 0, perm_mod_gen, colors=colors, linestyles=':', linewidth=1.2, alpha=0.8)
        ax3.scatter(perm_mod_gen, y_pos + off4, color=colors, marker='s', zorder=3, s=18, alpha=0.8)

        ax3.set_yticks(y_pos)
        ax3.set_yticklabels(labels, fontsize=plt.rcParams['font.size'] - 1.5)
        ax3.invert_yaxis()
        ax3.set_xlabel("Relative Importance", labelpad=4)
        ThesisStyle.set_title(ax3, "Permutation Channel Importance")
        ax3.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        ax3.spines['top'].set_visible(False)
        ax3.spines['right'].set_visible(False)
        ax3.axvline(0, color='#333333', linestyle='-', linewidth=0.8, alpha=0.5)
        
        min_val_perm = min(min(perm_fused_gen + perm_fused_spec + perm_mod_gen + perm_mod_spec), 0.0)
        max_val_perm = max(max(perm_fused_gen + perm_fused_spec + perm_mod_gen + perm_mod_spec), 1.0)
        if min_val_perm < 0:
            range_val = max_val_perm - min_val_perm
            x_min = min_val_perm - range_val * 0.05
            x_max = max_val_perm + range_val * 0.05
        else:
            x_min = 0.0
            x_max = max_val_perm * 1.1
        ax3.set_xlim(x_min, x_max)

        # 5. Global Legend
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        global_legend_handles = [
            Line2D([0], [0], color='#555555', linestyle='--', marker='o', markerfacecolor='white', markeredgecolor='#555555', markeredgewidth=1.2, markersize=6, label='Specialized (Fused)'),
            Line2D([0], [0], color='#555555', linestyle='-', marker='o', markerfacecolor='#555555', markersize=6, label='Generalized (Fused)'),
            Line2D([0], [0], color='#555555', linestyle='-.', marker='s', markerfacecolor='white', markeredgecolor='#555555', markeredgewidth=1.2, markersize=6, label='Specialized (Modality-only)'),
            Line2D([0], [0], color='#555555', linestyle=':', marker='s', markerfacecolor='#555555', markersize=6, label='Generalized (Modality-only)'),
            Patch(facecolor=ThesisStyle.COLOR_EMG, label='EMG Channel'),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label='IMU Channel')
        ]
        fig.legend(handles=global_legend_handles, loc='lower center', ncol=3,
                   bbox_to_anchor=(0.5, 0.01),
                   frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)

        # Main figure title and layout
        fig.suptitle(
            "Channel and Modality Importance: Fused vs Modality-Only Comparisons",
            fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['axes.titlesize'] + 0.5),
            color=ThesisStyle.RHO_BLUE,
            y=0.985
        )

        plt.tight_layout(rect=[0, 0.09, 1, 0.94])
        ThesisStyle.save_figure(fig, output_dir / "comp_deepshap_all")
        plt.close(fig)

    # ------------------------------------------------------------------
    # Plot 6: DeepSHAP channel importance – 2×2 panel, top-10 per run
    # ------------------------------------------------------------------
    def _plot_importance_panels(self, all_data, fi_key, xlabel, filename,
                                title, output_dir, layout_width, top_n=12):
        ThesisStyle.apply(layout_width)

        run_items = list(all_data.items())
        n_runs = len(run_items)
        ncols = 2
        nrows = (n_runs + 1) // ncols

        fig, axes = plt.subplots(nrows, ncols,
                                 figsize=(plt.rcParams['figure.figsize'][0] * 2,
                                          plt.rcParams['figure.figsize'][1] * nrows))
        axes = np.array(axes).flatten()

        run_colors = {
            "EMG + IMU (Gen)":   self.COLOR_GEN,
            "EMG-only (Gen)":    ThesisStyle.COLOR_EMG,
            "IMU-only (Gen)":    ThesisStyle.COLOR_IMU,
            "Par-Specific":      self.COLOR_PAR,
        }

        for ax, (rname, rdata) in zip(axes, run_items):
            fi = rdata.get("feature_importance", {}).get(fi_key, {})
            labels, values = self._top_n(fi, n=top_n)
            if fi_key in ["deepshap_channel", "permutation_channel"]:
                labels = [clean_channel_label(lbl) for lbl in labels]
            if not labels:
                ax.text(0.5, 0.5, "No data", ha='center', va='center',
                        transform=ax.transAxes)
                ax.set_title(rname)
                continue

            color = run_colors.get(rname, "#4477AA")
            y = np.arange(len(labels))
            ax.barh(y, values, color=color, alpha=0.85, edgecolor=color, linewidth=0.4)
            ax.axvline(0, color='#333333', linestyle='-', linewidth=0.7, alpha=0.5)
            ax.set_yticks(y)
            ax.set_yticklabels(labels, fontsize=plt.rcParams['font.size'] - 1.5)
            ax.invert_yaxis()
            ax.set_xlabel(xlabel, labelpad=6, fontsize=plt.rcParams['font.size'] - 1.0)
            ax.set_title(rname, fontsize=plt.rcParams['font.size'],
                         color=color, weight='semibold')
            ax.grid(True, axis='x', linestyle='--', alpha=0.5)
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)

        # Hide any unused panels
        for ax in axes[n_runs:]:
            ax.set_visible(False)

        ThesisStyle.set_suptitle(fig, title)
        plt.tight_layout(rect=[0, 0, 1, 0.93])
        ThesisStyle.save_figure(fig, output_dir / filename)
        plt.close(fig)

    def plot(self, gen_run_dir, par_run_dir, output_dir, layout_width):
        """Main entry point. Loads all run variants and generates all comparison plots."""
        # Load generalized sub-runs
        try:
            gen_all  = self._load_subrun(gen_run_dir, "all")
        except FileNotFoundError:
            try:
                gen_all = self._load(gen_run_dir)
            except FileNotFoundError as e:
                print(f"  [ERROR] Could not load gen all run data: {e}")
                return 0
        try:
            gen_emg = self._load_subrun(gen_run_dir, "emg_only")
        except FileNotFoundError:
            gen_emg = None
            print("  [Warning] gen emg_only sub-run not found, skipping.")

        try:
            gen_imu = self._load_subrun(gen_run_dir, "imu_only")
        except FileNotFoundError:
            gen_imu = None
            print("  [Warning] gen imu_only sub-run not found, skipping.")

        # Load par-spec sub-runs
        try:
            par_all  = self._load_subrun(par_run_dir, "all")
        except FileNotFoundError:
            try:
                par_all = self._load(par_run_dir)
            except FileNotFoundError as e:
                print(f"  [ERROR] Could not load participant-specific 'all' run data: {e}")
                return 0

        try:
            par_emg = self._load_subrun(par_run_dir, "emg_only")
        except FileNotFoundError:
            par_emg = None
            print("  [Warning] par emg_only sub-run not found, skipping.")

        try:
            par_imu = self._load_subrun(par_run_dir, "imu_only")
        except FileNotFoundError:
            par_imu = None
            print("  [Warning] par imu_only sub-run not found, skipping.")

        par_data = par_all

        # Dict of all 4 runs used for importance comparison (ordered)
        all_runs = {"EMG + IMU (Gen)": gen_all}
        if gen_emg:  all_runs["EMG-only (Gen)"]  = gen_emg
        if gen_imu:  all_runs["IMU-only (Gen)"]  = gen_imu
        all_runs["Par-Specific"] = par_data

        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        count = 0

        # ── Performance metrics ──
        print("  Plotting comp_metrics_summary...")
        self._plot_metrics_summary(gen_all, par_data, output_dir, layout_width)
        count += 1

        print("  Plotting comp_regression_6grid...")
        self._plot_regression_6grid(gen_all, gen_emg, gen_imu, par_all, par_emg, par_imu, output_dir, layout_width)
        count += 1

        print("  Plotting comp_generalization_mae_6grid...")
        self._plot_generalization_6grid(gen_all, gen_emg, gen_imu, par_all, par_emg, par_imu, output_dir, layout_width, metric="MAE")
        count += 1

        print("  Plotting comp_generalization_rmse_6grid...")
        self._plot_generalization_6grid(gen_all, gen_emg, gen_imu, par_all, par_emg, par_imu, output_dir, layout_width, metric="RMSE")
        count += 1

        print("  Plotting comp_per_participant_mae...")
        self._plot_per_participant_mae(gen_all, par_data, output_dir, layout_width)
        count += 1

        print("  Plotting comp_per_participant_rmse...")
        self._plot_per_participant_rmse(gen_all, par_data, output_dir, layout_width)
        count += 1

        # ── Importance comparisons ──
        print("  Plotting comp_deepshap_modality...")
        self._plot_deepshap_modality(all_runs, output_dir, layout_width)
        count += 1

        print("  Plotting comp_deepshap_modality_2...")
        self._plot_deepshap_modality_2(all_runs, output_dir, layout_width)
        count += 1

        print("  Plotting comp_deepshap_feature...")
        self._plot_deepshap_feature(all_runs, output_dir, layout_width)
        count += 1

        print("  Plotting comp_deepshap_all...")
        self._plot_deepshap_all(gen_all, gen_emg, gen_imu, par_all, par_emg, par_imu, output_dir, layout_width)
        count += 1

        print("  Plotting comp_deepshap_channel...")
        self._plot_importance_panels(
            all_runs, "deepshap_channel",
            xlabel="Mean |SHAP| Value",
            filename="comp_deepshap_channel",
            title="DeepSHAP Channel Importance — Top 12 per Model",
            output_dir=output_dir, layout_width=layout_width, top_n=12
        )
        count += 1

        print("  Plotting comp_deepshap_feature_type...")
        self._plot_importance_panels(
            all_runs, "deepshap_feature_type",
            xlabel="Mean |SHAP| Value",
            filename="comp_deepshap_feature_type",
            title="DeepSHAP Feature-Type Importance — All Models",
            output_dir=output_dir, layout_width=layout_width, top_n=20
        )
        count += 1

        print("  Plotting comp_permutation_channel...")
        self._plot_importance_panels(
            all_runs, "permutation_channel",
            xlabel="Permutation Importance (MAE increase)",
            filename="comp_permutation_channel",
            title="Permutation Channel Importance — Top 12 per Model",
            output_dir=output_dir, layout_width=layout_width, top_n=12
        )
        count += 1

        print("  Plotting comp_permutation_feature_type...")
        self._plot_importance_panels(
            all_runs, "permutation_feature",
            xlabel="Permutation Importance (MAE increase)",
            filename="comp_permutation_feature_type",
            title="Permutation Feature-Type Importance — All Models",
            output_dir=output_dir, layout_width=layout_width, top_n=20
        )
        count += 1

        return count



def get_class_participant_macro_metric_val(run_data, metric_name="RMSE"):
    """Calculate class-participant-macro metric value from predictions if not present in run_data."""
    eval_dict = run_data.get("evaluation", {}) or {}
    cpm = eval_dict.get("macro_metrics", {}).get("class_participant_macro", {}) or eval_dict.get("class_participant_macro", {})
    if cpm and cpm.get(metric_name) is not None:
        return float(cpm[metric_name])
        
    preds = run_data.get("predictions", {})
    if preds and "y_true" in preds and "y_pred" in preds:
        y_true = np.array(preds["y_true"])
        y_pred = np.maximum(0.0, np.array(preds["y_pred"]))
        parts = np.array(preds.get("participant", []))
        
        if len(parts) == len(y_true) and len(parts) > 0:
            part_metrics = []
            for p in np.unique(parts):
                m_p = parts == p
                w_metrics = []
                for w in np.unique(y_true):
                    m_w = (y_true == w) & m_p
                    if m_w.any():
                        if metric_name == "MAE":
                            w_metrics.append(np.mean(np.abs(y_true[m_w] - y_pred[m_w])))
                        else: # RMSE
                            w_metrics.append(np.sqrt(np.mean((y_true[m_w] - y_pred[m_w])**2)))
                if w_metrics:
                    part_metrics.append(np.mean(w_metrics))
            if part_metrics:
                return float(np.mean(part_metrics))
        else:
            class_metrics = []
            for w in np.unique(y_true):
                m_w = y_true == w
                if m_w.any():
                    if metric_name == "MAE":
                        class_metrics.append(np.mean(np.abs(y_true[m_w] - y_pred[m_w])))
                    else: # RMSE
                        class_metrics.append(np.sqrt(np.mean((y_true[m_w] - y_pred[m_w])**2)))
            if class_metrics:
                return float(np.mean(class_metrics))
                
    macro = eval_dict.get("macro_avg", {}) or {}
    pooled = eval_dict.get("pooled", {}) or {}
    val = macro.get(metric_name) or pooled.get(metric_name)
    return float(val) if val is not None else 0.0


@plot_registry.register("modality_ablation_study")
class ModalityAblationStudyPlotter:
    """Generates premium modality ablation study plots for a 31-combo grid.
    Combines:
    1. Presence/absence combination heatmap sorted by metric performance
    2. Per-group importance comparison (Shapley vs DeepSHAP vs Permutation)
    into a single unified figure.
    """
    
    def plot(self, data, output_path, layout_width):
        # Verify that this is the modality ablation study (requires combinations like EMGfo, IMUfo, etc.)
        required_combos = ["EMGfo", "EMGup", "EMGsh", "IMUfo", "IMUup", "all"]
        is_ablation_study = all(k in data for k in required_combos)
        if not is_ablation_study:
            print("  [Warning] Modality ablation study plotter requires a 32-combo ablation grid dataset. Skipping.")
            return False
            
        target_dir = output_path.parent
        target_dir.mkdir(parents=True, exist_ok=True)
        
        # 1. Combined plot (RMSE Variant)
        self._plot_combined(data, target_dir, "RMSE", layout_width)
        
        # 2. Combined plot (MAE Variant)
        self._plot_combined(data, target_dir, "MAE", layout_width)
        
        return True

    def _compute_shapley_importance(self, data, metric):
        CANONICAL_ORDER = ["IMUup", "IMUfo", "EMGfo", "EMGup", "EMGsh"]
        
        def all_combos():
            combos = []
            for r in range(1, len(CANONICAL_ORDER) + 1):
                for combo in itertools.combinations(CANONICAL_ORDER, r):
                    combos.append(frozenset(combo))
            return combos

        def combo_name(groups):
            _CANONICAL = {
                frozenset(CANONICAL_ORDER): "all",
                frozenset(["EMGfo", "EMGup", "EMGsh"]): "emg_only",
                frozenset(["IMUup", "IMUfo"]): "imu_only",
            }
            key = frozenset(groups)
            if key in _CANONICAL:
                return _CANONICAL[key]
            return "-".join(g for g in CANONICAL_ORDER if g in key)

        combos = all_combos()
        combo_metrics = {}
        NAME_TO_GROUPS = {combo_name(c): frozenset(c) for c in combos}
        for combo_name_str, run_data in data.items():
            groups = NAME_TO_GROUPS.get(combo_name_str)
            if not groups:
                continue
            val = get_class_participant_macro_metric_val(run_data, metric)
            combo_metrics[groups] = val
            
        v_empty = 1.0
        all_run = data.get("all")
        if all_run and "predictions" in all_run and "y_true" in all_run["predictions"]:
            y_true = np.asarray(all_run["predictions"]["y_true"], dtype=float)
            v_empty = float(np.std(y_true))
            
        def c(S):
            S = frozenset(S)
            if len(S) == 0:
                return v_empty
            return combo_metrics.get(S, v_empty)
            
        n = len(CANONICAL_ORDER)
        phi = {}
        for g in CANONICAL_ORDER:
            others = [x for x in CANONICAL_ORDER if x != g]
            total = 0.0
            for s in range(len(others) + 1):
                w = math.factorial(s) * math.factorial(n - s - 1) / math.factorial(n)
                for S in itertools.combinations(others, s):
                    cs = c(S)
                    csg = c(set(S) | {g})
                    total += w * (cs - csg)
            phi[g] = total

        def normalize_dict(d):
            s = sum(d.values())
            return {k: (v / s if s else 0.0) for k, v in d.items()}

        shap_norm = normalize_dict({k: max(v, 0.0) for k, v in phi.items()})
        return shap_norm

    def _plot_combined(self, data, target_dir, metric, layout_width):
        import matplotlib.gridspec as gridspec
        
        # Apply style configuration
        ThesisStyle.apply(layout_width)
        
        # Compute Shapley values for the specific metric (RMSE or MAE)
        shap_norm = self._compute_shapley_importance(data, metric)
        
        # Sort groups by descending Shapley importance
        CANONICAL_ORDER = ["IMUup", "IMUfo", "EMGfo", "EMGup", "EMGsh"]
        sorted_groups = sorted(CANONICAL_ORDER, key=lambda g: shap_norm.get(g, 0.0), reverse=True)
        
        # Determine figure size depending on layout_width
        if layout_width == "column":
            figsize = (3.46, 6.5)
        else:
            figsize = (7.0, 7.8)
            
        fig = plt.figure(figsize=figsize)
        
        # Use gridspec for layout:
        # Row 0: combo heatmap (matrix & bar) -> height ratio 5.0
        # Row 1: group importance bar chart -> height ratio 2.5
        gs = gridspec.GridSpec(2, 1, height_ratios=[5.0, 2.5], hspace=0.32)
        
        # Row 0 subplots
        if layout_width == "column":
            width_ratios = [1.0, 1.6]
        else:
            width_ratios = [1.0, 2.2]
            
        gs_top = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=gs[0], width_ratios=width_ratios, wspace=0.12)
        ax_mat = fig.add_subplot(gs_top[0, 0])
        ax_bar = fig.add_subplot(gs_top[0, 1])
        
        # Row 1 subplot
        ax_imp = fig.add_subplot(gs[1])
        
        # Plot Heatmap component with sorted groups
        self._plot_combo_heatmap_ax(data, ax_mat, ax_bar, metric, sorted_groups, layout_width)
        
        # Plot Group Importance component with precomputed values
        self._plot_group_importance_ax(data, ax_imp, sorted_groups, shap_norm, layout_width)
        
        # Entire figure suptitle
        fig.suptitle("Sensor Group Ablation Study Heat Map",
                     fontproperties=fm.FontProperties(family='Fira Sans', weight='bold', size=plt.rcParams['axes.titlesize'] + 0.5),
                     color="black", y=0.98)
        
        # Adjust layout spacing to avoid overlaps
        plt.tight_layout()
        
        # Subplots adjust to accommodate suptitle at the top and legend at the bottom
        bottom_margin = 0.11 if layout_width == "column" else 0.09
        fig.subplots_adjust(top=0.94, bottom=bottom_margin)
        
        # Save figure
        file_suffix = f"ablation_both_{metric.lower()}"
        ThesisStyle.save_figure(fig, target_dir / file_suffix)
        plt.close(fig)

    def _plot_group_importance_ax(self, data, ax, sorted_groups, shap_norm, layout_width):
        GROUP_LABELS = {
            "IMUup": "IMU-1",
            "IMUfo": "IMU-2",
            "EMGfo": "EMG-F",
            "EMGup": "EMG-U",
            "EMGsh": "EMG-S"
        }
        
        # Locate baseline directory from MODEL_RUNS
        baseline_cfg = MODEL_RUNS.get("specialized", {})
        baseline_dir = baseline_cfg.get("run_dir", "model/model_results/ST-transformer-par-spec-cross-val")
        
        baseline_path = Path(baseline_dir)
        if not baseline_path.is_absolute():
            script_dir = Path(__file__).resolve().parent
            project_root = script_dir.parent
            for base in [Path.cwd(), project_root, script_dir]:
                candidate = base / baseline_dir
                if (candidate / "run_data.json").exists() or (candidate / "all" / "run_data.json").exists():
                    baseline_path = candidate
                    break
                    
        # Load baseline run data
        baseline_data = None
        for json_cand in [baseline_path / "run_data.json", baseline_path / "all" / "run_data.json"]:
            if json_cand.exists():
                try:
                    with open(json_cand) as f:
                        baseline_data = json.load(f)
                    break
                except Exception as e:
                    print(f"Error loading baseline {json_cand}: {e}")
                    
        def channel_to_group(key):
            EMG_GROUPS = {
                "EMGfo": ["Brachioradialis", "Flexor Carpi Ulnaris (FCU)", "Extensor Carpi Radialis (ECR)"],
                "EMGup": ["Biceps Brachii", "Triceps Brachii"],
                "EMGsh": ["Anterior Deltoid", "Lateral Deltoid", "Posterior Deltoid"],
            }
            IMU_GROUPS = {
                "IMUup": ["ax1", "ay1", "az1", "roll_rad1", "pitch_rad1", "yaw_rad1"],
                "IMUfo": ["ax2", "ay2", "az2", "roll_rad2", "pitch_rad2", "yaw_rad2"],
            }
            name = key.replace("_EMG", "").replace("_IMU", "")
            name = name.split(" (")[0]
            if "Flexor Carpi Ulnaris" in name: name = "Flexor Carpi Ulnaris (FCU)"
            if "Extensor Carpi Radialis" in name: name = "Extensor Carpi Radialis (ECR)"
            
            for g, chans in EMG_GROUPS.items():
                if name in chans:
                    return g
            for g, chans in IMU_GROUPS.items():
                if name in chans:
                    return g
            return None

        def aggregate_attribution(run_data, fi_key):
            if not run_data:
                return {}
            fi = run_data.get("feature_importance", {}) or {}
            if not fi and "all" in run_data:
                fi = run_data["all"].get("feature_importance", {}) or {}
            chan = fi.get(fi_key)
            if not chan:
                return {}
            out = {g: 0.0 for g in sorted_groups}
            for k, v in chan.items():
                g = channel_to_group(k)
                if g is not None:
                    out[g] += max(float(v), 0.0)
            return out

        shap_g = aggregate_attribution(baseline_data, "deepshap_channel")
        perm_g = aggregate_attribution(baseline_data, "permutation_channel")
        
        def normalize_dict(d):
            s = sum(d.values())
            return {k: (v / s if s else 0.0) for k, v in d.items()}

        ds_norm = normalize_dict(shap_g) if shap_g else {}
        perm_norm = normalize_dict(perm_g) if perm_g else {}

        # Plotting onto the given ax
        x = np.arange(len(sorted_groups))
        
        methods = []
        if ds_norm: methods.append(("DeepSHAP", ds_norm, ThesisStyle.COLOR_DEEPSHAP))
        if perm_norm: methods.append(("Permutation", perm_norm, ThesisStyle.COLOR_PERMUTATION))
        
        series_labels = ["Ablation"] + [m[0] for m in methods]
        series_dicts = [shap_norm] + [m[1] for m in methods]
        series_colors = [ThesisStyle.COLOR_SHAPLEY] + [m[2] for m in methods]
        
        num_series = len(series_labels)
        width = 0.8 / num_series
        
        def darken_hex_color(hex_str, amount=0.25):
            hex_str = hex_str.lstrip('#')
            try:
                r, g, b = int(hex_str[0:2], 16), int(hex_str[2:4], 16), int(hex_str[4:6], 16)
                r = max(0, int(r * (1.0 - amount)))
                g = max(0, int(g * (1.0 - amount)))
                b = max(0, int(b * (1.0 - amount)))
                return f"#{r:02x}{g:02x}{b:02x}"
            except Exception:
                return "#000000"
                
        hatches = ["//", "\\\\", "xx"]
        for i in range(num_series):
            offset = (i - (num_series - 1) / 2) * width
            vals = [series_dicts[i].get(g, 0.0) for g in sorted_groups]
            c_fill = series_colors[i]
            c_edge = darken_hex_color(c_fill, amount=0.25)
            hatch_pattern = hatches[i % len(hatches)]
            ax.bar(x + offset, vals, width, label=series_labels[i],
                   color=c_fill, alpha=0.85, edgecolor=c_edge, hatch=hatch_pattern, linewidth=0.8)
                            
        ax.set_ylabel("Normalized Importance", labelpad=4)
        
        title = "Sensor Group Importance"
        ThesisStyle.set_title(ax, title)
        ax.title.set_color("black")
        
        ax.set_xticks(x)
        ax.set_xticklabels([GROUP_LABELS[g] for g in sorted_groups])
        
        max_val = max([max(s.values()) for s in series_dicts if s]) if any(series_dicts) else 1.0
        ax.set_ylim(0, max_val * 1.25)
        
        # Position legend closely at the bottom of this subplot
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=3, frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95, fontsize=8.0)

    def _plot_combo_heatmap_ax(self, data, ax_mat, ax_bar, metric, sorted_groups, layout_width):
        GROUP_LABELS = {
            "IMUup": "IMU-1",
            "IMUfo": "IMU-2",
            "EMGfo": "EMG-F",
            "EMGup": "EMG-U",
            "EMGsh": "EMG-S"
        }
        EMG_GROUPS = {
            "EMGfo": ["Brachioradialis", "Flexor Carpi Ulnaris (FCU)", "Extensor Carpi Radialis (ECR)"],
            "EMGup": ["Biceps Brachii", "Triceps Brachii"],
            "EMGsh": ["Anterior Deltoid", "Lateral Deltoid", "Posterior Deltoid"],
        }
        IMU_GROUPS = {
            "IMUup": ["ax1", "ay1", "az1", "roll_rad1", "pitch_rad1", "yaw_rad1"],
            "IMUfo": ["ax2", "ay2", "az2", "roll_rad2", "pitch_rad2", "yaw_rad2"],
        }
        
        CANONICAL_ORDER = ["IMUup", "IMUfo", "EMGfo", "EMGup", "EMGsh"]
        
        def all_combos():
            combos = []
            for r in range(1, len(CANONICAL_ORDER) + 1):
                for combo in itertools.combinations(CANONICAL_ORDER, r):
                    combos.append(frozenset(combo))
            return combos

        def combo_name(groups):
            _CANONICAL = {
                frozenset(CANONICAL_ORDER): "all",
                frozenset(["EMGfo", "EMGup", "EMGsh"]): "emg_only",
                frozenset(["IMUup", "IMUfo"]): "imu_only",
            }
            key = frozenset(groups)
            if key in _CANONICAL:
                return _CANONICAL[key]
            return "-".join(g for g in CANONICAL_ORDER if g in key)

        combos = all_combos()
        
        rows = []
        for combo in combos:
            name = combo_name(combo)
            run_data = data.get(name)
            if not run_data:
                continue
            rmse = get_class_participant_macro_metric_val(run_data, "RMSE")
            mae = get_class_participant_macro_metric_val(run_data, "MAE")
            row = {
                "combo": name,
                "RMSE": rmse,
                "MAE": mae,
                "n_channels": sum(len(IMU_GROUPS.get(g, EMG_GROUPS.get(g, []))) for g in combo)
            }
            for g in sorted_groups:
                row[g] = int(g in combo)
            rows.append(row)
            
        import pandas as pd
        df = pd.DataFrame(rows)
        df = df.sort_values(metric, ascending=True).reset_index(drop=True)
        
        nrows, ncols = df[sorted_groups].values.shape
        
        # Draw background grid squares (soft gray)
        for r in range(nrows):
            for c in range(ncols):
                rect = plt.Rectangle((c - 0.5, r - 0.5), 1.0, 1.0,
                                     facecolor=ThesisStyle.COLOR_HEATMAP_ABSENT, edgecolor="white",
                                     linewidth=1.5, zorder=1)
                ax_mat.add_patch(rect)
                
        # Draw present blocks on top
        for r in range(nrows):
            for c in range(ncols):
                if df[sorted_groups].values[r, c] == 1:
                    rect = plt.Rectangle((c - 0.5, r - 0.5), 1.0, 1.0,
                                         facecolor=ThesisStyle.COLOR_HEATMAP_PRESENT, edgecolor="white",
                                         linewidth=1.5, zorder=2)
                    ax_mat.add_patch(rect)
                    
        ax_mat.set_xlim(-0.5, ncols - 0.5)
        ax_mat.set_ylim(nrows - 0.5, -0.5)
        ax_mat.set_yticks([])
        ax_mat.set_xticks(np.arange(ncols))
        ax_mat.set_xticklabels([GROUP_LABELS[g] for g in sorted_groups], rotation=45, ha="right", fontsize=8.0)
        
        # Remove spines
        for spine in ax_mat.spines.values():
            spine.set_visible(False)
            
        # Draw horizontal bar chart on right side
        metric_vals = df[metric].values
        y_pos = np.arange(nrows)
        height = 0.5
        
        color = ThesisStyle.COLOR_HEATMAP_PRESENT
        ax_bar.barh(y_pos, metric_vals, height=height, color=color, alpha=0.85, zorder=2)
        
        ax_bar.set_ylim(nrows - 0.5, -0.5)
        ax_bar.set_yticks([])
        ax_bar.grid(True, axis='x', linestyle='--', color=ThesisStyle.GRID_GRAY, alpha=0.5, zorder=0)
        ax_bar.set_xlabel(f"{metric} (kg)", labelpad=4)
        
        ax_bar.spines['top'].set_visible(False)
        ax_bar.spines['right'].set_visible(False)


def run_comparison_pipeline(gen_run_dir, par_run_dir, output_dir_raw, width_style):
    """Runs the cross-run comparison pipeline for generalized vs participant-specific models."""
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parent

    # Resolve output directory
    out_path = Path(output_dir_raw)
    if not out_path.is_absolute():
        for base in [Path.cwd(), project_root, script_dir]:
            candidate = base / output_dir_raw
            out_path = candidate
            break

    out_path.mkdir(parents=True, exist_ok=True)

    plotter = RunComparisonPlotter()
    n = plotter.plot(gen_run_dir, par_run_dir, out_path, width_style)
    print(f"Completed comparison run! Generated {n} plot formats in {out_path}.\n")


# ===========================================================================
# 5. Command Line Orchestration
# ===========================================================================

def run_viz_pipeline(run_dir_raw, output_dir_raw, active_plots, width_style):
    # 1. Resolve run_data.json path using a multi-step search strategy
    run_dir_path = Path(run_dir_raw)
    
    if not run_dir_path.is_absolute():
        # Try resolving relative to:
        # A. Current working directory
        cwd_resolved = run_dir_path.resolve()
        # B. Project root directory (parent of the script's directory)
        script_dir = Path(__file__).resolve().parent
        project_root = script_dir.parent
        proj_resolved = (project_root / run_dir_raw).resolve()
        # C. Script's parent directory
        script_resolved = (script_dir / run_dir_raw).resolve()
        
        if (cwd_resolved / "run_data.json").exists():
            run_dir_path = cwd_resolved
        elif (proj_resolved / "run_data.json").exists():
            run_dir_path = proj_resolved
        elif (script_resolved / "run_data.json").exists():
            run_dir_path = script_resolved
        else:
            # Fallback to standard resolve relative to cwd
            run_dir_path = cwd_resolved
    else:
        run_dir_path = run_dir_path.resolve()
        
    json_path = run_dir_path / "run_data.json"
    data = None
    
    if not json_path.exists():
        # Check if there are subdirectories with run_data.json
        sub_runs_data = {}
        if run_dir_path.exists():
            for p in run_dir_path.iterdir():
                if p.is_dir() and (p / "run_data.json").exists():
                    try:
                        with open(p / "run_data.json", 'r') as f:
                            sub_runs_data[p.name] = json.load(f)
                    except Exception as e:
                        print(f"Error loading {p / 'run_data.json'}: {e}")
        if sub_runs_data:
            data = sub_runs_data
            print(f"Dynamically loaded {len(data)} sub-runs from subdirectories in {run_dir_path}")
        else:
            print(f"Error: run_data.json not found at: {json_path} and no subdirectories containing run_data.json found.")
            print("Please check the configuration or command line arguments.")
            return False
    else:
        print(f"Loading run data from: {json_path}")
        with open(json_path, 'r') as f:
            try:
                data = json.load(f)
            except Exception as e:
                print(f"Error reading JSON: {e}")
                return False
            
    # 2. Setup output folder
    output_dir_path = Path(output_dir_raw)
    if not output_dir_path.is_absolute():
        # Resolve relative to project root
        script_dir = Path(__file__).resolve().parent
        project_root = script_dir.parent
        output_dir_path = (project_root / output_dir_raw).resolve()
    else:
        output_dir_path = output_dir_path.resolve()
        
    global CURRENT_OUTPUT_DIR
    CURRENT_OUTPUT_DIR = str(output_dir_path)
    
    output_dir_path.mkdir(parents=True, exist_ok=True)
    print(f"Saving figures to folder: {output_dir_path}")
    
    # Detect if we have a multi-modality ablation or flat run
    is_multi_ablation = False
    sub_runs = []
    if "meta" not in data:
        sub_runs = [k for k, v in data.items() if isinstance(v, dict) and "meta" in v]
        if sub_runs:
            is_multi_ablation = True

    # If this is a multi-modality ablation study run, we ONLY run 'modality_ablation_study' plotter
    # to prevent generating 180+ individual sub-run plots.
    if is_multi_ablation:
        active_plots = [p for p in active_plots if p == "modality_ablation_study"]

    # 4. Run plotters
    successful_count = 0
    
    # Define runs to process for standard individual plotters
    if is_multi_ablation:
        runs_to_process = [(k, data[k], f"_{k}") for k in sub_runs]
    else:
        runs_to_process = [("default", data, "")]
        
    PLOT_CATEGORIES = {
        "loss": "performance",
        "regression": "performance",
        "calibration": "calibration",
        "seqlen": "performance",
        "participant": "performance",
        "weight_error": "performance",
        "ablation": "performance",
        "tradeoff": "performance",
        "modality_comparison": "modality comparison",
        "modality_ablation_study": "",
        
        "distribution": "dataset",
        
        "tukey": "statistics",
        
        "importance_channel": "importance",
        "importance_feature": "importance",
        "importance_individual": "importance",
        "deepshap_channel": "importance",
        "deepshap_feature_type": "importance",
        "deepshap_modality": "importance"
    }

    # Determine prefix from folder name
    folder_name = output_dir_path.name
    if folder_name.startswith("run_plots_"):
        prefix = folder_name.replace("run_plots_", "") + "_"
    else:
        prefix = folder_name + "_"

    for name in active_plots:
        plotter = plot_registry.get_plotter(name)
        if not plotter:
            continue
            
        category = PLOT_CATEGORIES.get(name, "")
        target_subfolder = output_dir_path / category
        target_subfolder.mkdir(parents=True, exist_ok=True)
            
        if name in ["ablation", "tradeoff", "modality_comparison", "modality_ablation_study"]:
            # These plotters run once on the root multi-run dictionary
            print(f"Running '{name}' plotter (on root multi-run data)...")
            out_file = target_subfolder / f"{prefix}{name}_plot"
            try:
                success = plotter.plot(data, out_file, width_style)
                if success:
                    successful_count += 1
            except Exception as e:
                import traceback
                print(f"  [ERROR] Plotter '{name}' failed with exception: {e}")
                traceback.print_exc()
        elif name == "distribution":
            # Run once on single/sub-run dictionary
            print(f"Running '{name}' plotter (once)...")
            out_file = target_subfolder / f"{prefix}{name}_plot"
            try:
                run_dict = data[sub_runs[0]] if is_multi_ablation else data
                success = plotter.plot(run_dict, out_file, width_style)
                if success:
                    successful_count += 1
            except Exception as e:
                import traceback
                print(f"  [ERROR] Plotter '{name}' failed with exception: {e}")
                traceback.print_exc()
        else:
            # Individual plotters run for each sub-run/flat-run
            for run_name, run_dict, suffix in runs_to_process:
                run_desc = f" ({run_name})" if is_multi_ablation else ""
                print(f"Running '{name}' plotter{run_desc}...")
                out_file = target_subfolder / f"{prefix}{name}_plot{suffix}"
                try:
                    success = plotter.plot(run_dict, out_file, width_style)
                    if success:
                        successful_count += 1
                except Exception as e:
                    import traceback
                    print(f"  [ERROR] Plotter '{name}' failed{run_desc} with exception: {e}")
                    traceback.print_exc()
                
    print(f"Completed run! Generated {successful_count} visual formats successfully in {output_dir_path}.\n")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Generates premium, modular, and rho.cls styled LaTeX thesis plots from run_data.json.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--run-dir", "-r",
        type=str,
        default=None,
        help="Path to the model run folder containing run_data.json. If omitted, runs configured in MODEL_RUNS are processed."
    )
    parser.add_argument(
        "--output-dir", "-o",
        type=str,
        default=None,
        help="Target folder to save generated figures. If omitted, runs configured in MODEL_RUNS are processed."
    )
    parser.add_argument(
        "--width", "-w",
        type=str,
        choices=["column", "double", "default"],
        default=LAYOUT_WIDTH,
        help="Physical page width scaling template: 'column' (3.5\"), 'double' (7.0\"), or 'default' (8.0\")."
    )
    parser.add_argument(
        "--plots", "-p",
        type=str,
        nargs="+",
        default=PLOTS_TO_GENERATE,
        help=f"Specific plots to generate. Available options: {', '.join(plot_registry.list_plotters())}. If unspecified, all are generated."
    )
    
    # Use parse_known_args to ignore unexpected command-line arguments (e.g. from IDE runners)
    args, unknown = parser.parse_known_args()
    if unknown:
        print(f"Note: Ignoring unrecognized command-line arguments: {unknown}")
        
    # Determine which plotters to run
    active_plots = args.plots
    if active_plots is None:
        active_plots = plot_registry.list_plotters()
    else:
        # Validate selected plotters
        invalid = [p for p in active_plots if p not in plot_registry.list_plotters()]
        if invalid:
            print(f"Error: Selected invalid plotters: {invalid}")
            print(f"Choose from: {plot_registry.list_plotters()}")
            sys.exit(1)
            
    # Determine width style to use: by default, single plots are single-column ('column')
    # and comparison plots are double-column ('double').
    # If the user explicitly requested a width style choice, respect it for both.
    if args.width == "default":
        single_width = "column"
        comp_width = "double"
    else:
        single_width = args.width
        comp_width = args.width

    # If run-dir or output-dir is specified via command line arguments, process that single run.
    if args.run_dir is not None or args.output_dir is not None:
        run_dir = args.run_dir if args.run_dir is not None else RUN_DIR
        output_dir = args.output_dir if args.output_dir is not None else OUTPUT_DIR
        run_viz_pipeline(run_dir, output_dir, active_plots, single_width)
    else:
        # Default: Process both generalized and specialized runs configured in MODEL_RUNS
        for run_name, config in MODEL_RUNS.items():
            print(f"\n==========================================================================")
            print(f" PROCESSING MODEL RUN CONFIGURATION: {run_name.upper()}")
            print(f"==========================================================================")
            run_viz_pipeline(config["run_dir"], config["output_dir"], active_plots, single_width)

        # Cross-run comparison: generalized vs participant-specific
        gen_cfg  = MODEL_RUNS.get("generalized", {})
        par_cfg  = MODEL_RUNS.get("specialized", {})
        if gen_cfg and par_cfg:
            print(f"\n==========================================================================")
            print(f" PROCESSING CROSS-RUN COMPARISON: GENERALIZED vs PARTICIPANT-SPECIFIC")
            print(f"==========================================================================")
            run_comparison_pipeline(
                gen_run_dir  = gen_cfg["run_dir"],
                par_run_dir  = par_cfg["run_dir"],
                output_dir_raw = "visualization/run_plots_comp",
                width_style  = comp_width
            )



if __name__ == "__main__":
    main()
