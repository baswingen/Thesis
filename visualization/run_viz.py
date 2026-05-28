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
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# ──────────────────────────────────────────────────────────
# VISUALIZATION CONFIGURATION (rho.cls style)
# ──────────────────────────────────────────────────────────
# Path to the model run folder containing run_data.json.
# Can be absolute, or relative to the project root.
RUN_DIR = "model/model_results/run_20260521_100751"

# Target folder to save generated figures.
OUTPUT_DIR = "/Users/baswingen/Library/Mobile Documents/com~apple~CloudDocs/Master Thesis/python/Thesis/visualization/run_plots"

# Physical page width scaling template: 
# - "column": single-column width (~3.46")
# - "double": double-column width (~7.0")
# - "default": standard presentation width (~8.0")
LAYOUT_WIDTH = "default"

# Specific plots to generate. 
# Options: "loss", "regression", "seqlen", "participant", "distribution", "weight_error", "ablation", "importance_channel", "importance_feature", "tukey"
# Set to None to automatically generate all available plots, or specify a list: e.g. ["loss", "regression"]
PLOTS_TO_GENERATE = None

# ===========================================================================================
# 1. LaTeX Thesis Style Configurator (based on rho.cls)
# ===========================================================================

class ThesisStyle:
    # ─── 1. Paul Tol Scientific Semantic Colors ───
    COLOR_EMG = "#BB5566"       # Paul Tol High-Contrast Red (Swapped: EMG)
    COLOR_IMU = "#004488"       # Paul Tol High-Contrast Blue (Swapped: IMU)
    COLOR_FUSION = "#AA3377"    # Paul Tol Bright Purple (Combined/Fusion blend of EMG + IMU)

    # ─── 2. Consistent Statistical Metric Colors ───
    COLOR_MAE = "#004488"       # Paul Tol High-Contrast Blue (MAE)
    COLOR_RMSE = "#BB5566"      # Paul Tol High-Contrast Red (RMSE)
    COLOR_COUNT = "#4A5568"     # Muted Charcoal (sample count bars/lines)

    # ─── 3. Reference and Boundary Colors ───
    COLOR_UNITY = "#888888"     # Paul Tol Slate Grey (dashed perfect-prediction line)
    COLOR_MEDIAN = "#DDAA33"    # Paul Tol High-Contrast Yellow (pop-out median line in boxplots)
    COLOR_TARGET = "#228833"    # Paul Tol Bright Green (balanced dataset target threshold)
    COLOR_OUTLIER = "#BBBBBB"   # Paul Tol Light Grey (scatter points)

    # ─── 4. Pairwise Tukey HSD Significance Colors ───
    COLOR_P_001 = "#CC3311"     # Paul Tol Vibrant Red (*** p < 0.001)
    COLOR_P_01 = "#AA4499"      # Paul Tol Muted Purple (** p < 0.01)
    COLOR_P_05 = "#4477AA"      # Paul Tol Bright Blue (* p < 0.05)
    COLOR_P_NS = "#BBBBBB"      # Paul Tol Light Grey (n.s. p >= 0.05)

    # Color Palette from rho.cls (for backwards compatibility)
    RHO_BLUE = "#004488"        # Paul Tol High-Contrast Blue (Primary brand color)
    RHO_LIGHT_BLUE = "#F0F4F8"  # Very soft Paul Tol blue tint
    ACCENT_RED = "#BB5566"      # Paul Tol High-Contrast Red
    ACCENT_BLUE = "#4477AA"     # Paul Tol Bright Blue
    NEUTRAL_GRAY = "#888888"    # Paul Tol Slate Grey
    GRID_GRAY = "#F0F0F0"       # Soft background grid lines
    
    @classmethod
    def apply(cls, layout_width="default"):
        """Applies global style parameters to Matplotlib for thesis-ready look."""
        plt.rcParams.update(plt.rcParamsDefault)
        
        # Base clean style
        try:
            plt.style.use('seaborn-v0_8-paper')
        except:
            plt.style.use('ggplot')
            
        # Determine sizing and margins (to avoid scaling figures in LaTeX)
        if layout_width == "column":
            # Fits perfectly in a single column (~3.46 inches wide)
            figsize = (3.46, 2.8)
            fontsize = 8
            labelsize = 8
            titlesize = 9
            legendsize = 7
            ticksize = 7
            linewidth = 1.0
            markersize = 3
        elif layout_width == "double":
            # Fits across a two-column layout (~7.0 inches wide)
            figsize = (7.0, 4.2)
            fontsize = 9
            labelsize = 9.5
            titlesize = 10.5
            legendsize = 8
            ticksize = 8
            linewidth = 1.2
            markersize = 4
        else: # "default" or full screen representation
            figsize = (8.0, 5.0)
            fontsize = 10
            labelsize = 11
            titlesize = 12
            legendsize = 9
            ticksize = 9
            linewidth = 1.5
            markersize = 5
            
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
            
            # Use Fira Sans if available, falling back to other clean sans-serif families
            'font.family': 'sans-serif',
            'font.sans-serif': ['Fira Sans', 'DejaVu Sans', 'Arial', 'Helvetica', 'sans-serif'],
            
            # Match math text with LaTeX's STIX/Times font
            'mathtext.fontset': 'stix',
            
            # Borders & Spines (Clean, professional look)
            'axes.spines.top': False,
            'axes.spines.right': False,
            'axes.spines.left': True,
            'axes.spines.bottom': True,
            'axes.edgecolor': '#333333',
            'axes.linewidth': 0.7,
            
            # Clean Semibold mixed-case titles (consistent styled headers)
            'axes.titleweight': 'semibold',
            'axes.titlecolor': cls.RHO_BLUE,
            
            # Thin, muted grid lines behind data
            'axes.grid': True,
            'grid.color': cls.GRID_GRAY,
            'grid.alpha': 0.5,
            'grid.linestyle': '--',
            'grid.linewidth': 0.5,
            
            # Ticks
            'xtick.major.size': 3,
            'xtick.major.width': 0.7,
            'ytick.major.size': 3,
            'ytick.major.width': 0.7,
            
            # Save settings
            'savefig.dpi': 300,
            'savefig.bbox': 'tight',
            'savefig.transparent': False
        })


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
        title = f"{model_type}: Training Progress"
        if is_cv:
            title += f" ({len(histories)} Folds)"
        ax.set_title(title, pad=12)
        
        ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        
        # Export PNG only
        fig.savefig(output_path.with_suffix(".png"))
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
            fill_color = "#FDF3F4"
            accent_color = "#004488"
        elif "imu_only" in out_name:
            primary_color = ThesisStyle.COLOR_IMU
            fill_color = ThesisStyle.RHO_LIGHT_BLUE
            accent_color = "#BB5566"
        else: # "all" or fallback
            primary_color = ThesisStyle.COLOR_FUSION
            fill_color = "#FAF0F5"
            accent_color = "#DDAA33"

        # Perfect prediction line (Ideal Unity)
        ax.plot([min_val, max_val], [min_val, max_val], 
                color=ThesisStyle.COLOR_UNITY, linestyle='--', linewidth=1.8, alpha=0.8, 
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
                               alpha=0.18, s=10, color=ThesisStyle.COLOR_OUTLIER, edgecolors='none', 
                               zorder=2, label=label)
                    first_outlier = False
                    
        # Grouped box plots styled like LaTeX environments with caps-free modern aesthetic
        bp = ax.boxplot(pred_groups, positions=actual_weights, widths=0.35, 
                        patch_artist=True, showfliers=False, showcaps=False,
                        whiskerprops={'color': ThesisStyle.NEUTRAL_GRAY, 'linewidth': 1.6},
                        boxprops={'edgecolor': primary_color, 'linewidth': 2.0},
                        zorder=3)
        
        # Single uniform fill color for all boxes (soft light blue/red/purple for a clean, professional look)
        for box in bp['boxes']:
            box.set_facecolor(fill_color)
            box.set_alpha(0.9)
            
        # Vibrant Golden Amber/Red/Blue for all median lines for high contrast and clean consistency
        for median in bp['medians']:
            median.set_color(accent_color)
            median.set_linewidth(2.5)
            
        # Stats annotation styled like a premium LaTeX card box (Rounded, with RMSE added)
        eval_pooled = data.get("evaluation", {}).get("pooled", {})
        r2 = eval_pooled.get("R2")
        mae = eval_pooled.get("MAE")
        rmse = eval_pooled.get("RMSE")
        
        if r2 is None or mae is None or rmse is None:
            from sklearn.metrics import r2_score, mean_absolute_error, mean_squared_error
            r2 = r2_score(y_true, y_pred)
            mae = mean_absolute_error(y_true, y_pred)
            rmse = np.sqrt(mean_squared_error(y_true, y_pred))
            
        stats_text = f"POOLED METRICS\n$R^2 = {r2:.3f}$\n$MAE = {mae:.3f}$ kg\n$RMSE = {rmse:.3f}$ kg"
        
        ax.text(0.04, 0.96, stats_text, 
                transform=ax.transAxes, verticalalignment='top', fontsize=plt.rcParams['font.size'] - 1.5,
                bbox=dict(facecolor='white', alpha=0.95, 
                          edgecolor=primary_color, boxstyle='round,pad=0.6', linewidth=1.2))
        
        ax.set_xlabel("Actual Weight (kg)", labelpad=8)
        ax.set_ylabel("Predicted Weight (kg)", labelpad=8)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        ax.set_title(f"{model_type}: Predicted vs. Actual Weight", pad=12)
        
        ax.set_xlim(min_val - 0.1, max_val)
        ax.set_ylim(min_val - 0.1, max_val)
        
        # Custom ticks
        ax.set_xticks(actual_weights)
        ax.set_yticks(actual_weights)
        
        def format_label(x):
            return f"{x:.2f}".rstrip('0').rstrip('.')
            
        ax.set_xticklabels([format_label(w) for w in actual_weights])
        ax.set_yticklabels([format_label(w) for w in actual_weights])
        
        ax.legend(loc='lower right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        
        # Export PNG only
        fig.savefig(output_path.with_suffix(".png"))
        plt.close(fig)
        return True


@plot_registry.register("seqlen")
class SeqLenPlotter:
    """Plots error metrics vs available training context duration."""
    
    def plot(self, data, output_path, layout_width):
        per_seqlen_stats = data.get("evaluation", {}).get("per_seqlen")
        if not per_seqlen_stats:
            print("  [Warning] No sequence length stats found. Skipping seqlen plot.")
            return False
            
        min_count = 10
        filtered = [row for row in per_seqlen_stats if int(row['Count']) >= min_count]
        if not filtered:
            print(f"  [Warning] All bins dropped due to min_count={min_count}. Skipping seqlen plot.")
            return False
            
        ThesisStyle.apply(layout_width)
        
        times = [float(row['TimeAtPrediction'].rstrip('s')) for row in filtered]
        maes = [float(row['MAE']) for row in filtered]
        rmses = [float(row['RMSE']) for row in filtered]
        counts = [int(row['Count']) for row in filtered]
        
        fig, ax1 = plt.subplots()
        
        # Elegant bar chart for sample count (right y-axis) representing samples in bins
        ax2 = ax1.twinx()
        if len(times) > 1:
            bar_width = 0.5 * np.min(np.diff(times))
        else:
            bar_width = 0.1
        ax2.bar(times, counts, width=bar_width, color=ThesisStyle.RHO_LIGHT_BLUE, alpha=0.6,
                edgecolor=ThesisStyle.COLOR_UNITY, linewidth=0.5, label='Sample Count', zorder=1)
        
        ax2.set_ylabel("Sample Count", color=ThesisStyle.NEUTRAL_GRAY)
        ax2.tick_params(axis='y', labelcolor=ThesisStyle.NEUTRAL_GRAY)
        ax2.set_ylim(0, max(counts) * 3.0) # push area chart cleanly into bottom third
        ax2.spines['top'].set_visible(False)
        ax2.spines['right'].set_visible(True)
        ax2.spines['left'].set_visible(True)
        ax2.grid(False) # Disable secondary gridlines
        
        # Primary error lines (window counts text removed for clutter-free paper format)
        ax1.plot(times, maes, marker='o', color=ThesisStyle.COLOR_MAE, label='MAE', zorder=3)
        ax1.plot(times, rmses, marker='s', color=ThesisStyle.COLOR_RMSE, label='RMSE', zorder=3)
        
        ax1.set_xlabel("Elapsed time into lift at prediction (s)", labelpad=8)
        ax1.set_ylabel("Error (kg)", labelpad=8)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        ax1.set_title(f"{model_type}: Error vs. Available Segment Length", pad=12)
        ax1.set_ylim(bottom=0)
        
        # Setup ticks
        ax1.set_xticks(times)
        if len(times) > 15:
            labels = [f"{t:.2f}s" if idx % 2 == 0 or idx == len(times)-1 else "" for idx, t in enumerate(times)]
            ax1.set_xticklabels(labels, rotation=30, ha='right')
        else:
            ax1.set_xticklabels([f"{t:.2f}s" for t in times], rotation=30, ha='right')
            
        # Combine legends cleanly
        lines1, labels1 = ax1.get_legend_handles_labels()
        lines2, labels2 = ax2.get_legend_handles_labels()
        ax1.legend(lines1 + lines2, labels1 + labels2,
                   loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        
        ax1.set_zorder(ax2.get_zorder() + 1)
        ax1.patch.set_visible(False)
        
        n_dropped = len(per_seqlen_stats) - len(filtered)
        if n_dropped > 0:
            footnote = f"Bins with < {min_count} samples omitted ({n_dropped} bin(s) excluded)."
            fig.text(0.5, -0.05, footnote, ha='center', va='top',
                     fontsize=plt.rcParams['font.size'] - 2, color=ThesisStyle.NEUTRAL_GRAY, style='italic')
            
        plt.tight_layout()
        
        # Export PNG only
        fig.savefig(output_path.with_suffix(".png"))
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
        mean_mae = np.mean(maes)
        mean_rmse = np.mean(rmses)
        
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
        ax.set_title(f"{model_type}: Generalization per Participant", pad=12)
        
        ax.set_xticks(x)
        ax.set_xticklabels(participants, rotation=45, ha='right')
        
        max_val = max(np.max(maes), np.max(rmses))
        ax.set_ylim(0, max_val * 1.25) # Extra buffer for horizontal annotations
        
        ax.legend(loc='upper left', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        
        # Export PNG only
        fig.savefig(output_path.with_suffix(".png"))
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
        title_text = f"{model_type}: Dataset Composition & Augmentation Balancing" if balance_enabled else f"{model_type}: Dataset Composition (Total Samples: {total_samples})"
        fig.suptitle(title_text, 
                     fontsize=plt.rcParams['axes.titlesize'] + 0.5, fontweight='semibold', 
                     color=ThesisStyle.RHO_BLUE, y=0.98)
                     
        plt.tight_layout(rect=[0, 0, 1, 0.93])
        fig.savefig(output_path.with_suffix(".png"))
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
        fig, ax = plt.subplots()
        
        w_labels = [row['Weight'] for row in per_weight]
        maes = [float(row['MAE']) for row in per_weight]
        rmses = [float(row['RMSE']) for row in per_weight]
        
        # Dual error lines
        ax.plot(w_labels, maes, marker='o', color=ThesisStyle.COLOR_MAE, label='MAE', zorder=3)
        ax.plot(w_labels, rmses, marker='s', color=ThesisStyle.COLOR_RMSE, label='RMSE', zorder=3)
        
        # Annotate values above/below markers
        for idx, (mae, rmse) in enumerate(zip(maes, rmses)):
            # Draw MAE slightly below, RMSE slightly above
            ax.annotate(f"{mae:.3f}", xy=(idx, mae), xytext=(0, -13),
                        textcoords='offset points', ha='center', va='top',
                        fontsize=plt.rcParams['font.size'] - 2, color=ThesisStyle.COLOR_MAE)
            ax.annotate(f"{rmse:.3f}", xy=(idx, rmse), xytext=(0, 6),
                        textcoords='offset points', ha='center', va='bottom',
                        fontsize=plt.rcParams['font.size'] - 2, color=ThesisStyle.COLOR_RMSE)
                        
        ax.set_xlabel("Physical Weight Class", labelpad=8)
        ax.set_ylabel("Prediction Error (kg)", labelpad=8)
        ax.set_ylim(0, max(rmses) * 1.25)
        
        model_type = data.get("meta", {}).get("model_type", "Model").replace("_", " ").title()
        ax.set_title(f"{model_type}: Error Scaling by Weight Class", pad=12)
        
        ax.legend(loc='upper left', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        
        fig.savefig(output_path.with_suffix(".png"))
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
            "all": "EMG + IMU\n(Full Model)",
            "emg_only": "EMG Only",
            "imu_only": "IMU Only"
        }
        
        # Prioritize predefined modality order
        for key in ["all", "emg_only", "imu_only"]:
            if key not in data or not isinstance(data[key], dict):
                continue
            run_data = data[key]
            pooled = run_data.get("evaluation", {}).get("pooled", {})
            if not pooled:
                continue
                
            modalities.append(name_map.get(key, key.replace("_", " ").title()))
            maes.append(pooled.get("MAE", 0.0))
            rmses.append(pooled.get("RMSE", 0.0))
            
        # Fallback to other runs if standard ones are not present
        if not modalities:
            for key, run_data in data.items():
                if not isinstance(run_data, dict) or "evaluation" not in run_data:
                    continue
                pooled = run_data["evaluation"].get("pooled", {})
                if not pooled:
                    continue
                modalities.append(name_map.get(key, key.replace("_", " ").title()))
                maes.append(pooled.get("MAE", 0.0))
                rmses.append(pooled.get("RMSE", 0.0))
                
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
        ax.set_title("Sensor Modality Ablation Study", pad=12)
        ax.set_xticks(x)
        ax.set_xticklabels(modalities)
        ax.set_ylim(0, max(rmses) * 1.22)
        
        ax.legend(loc='upper right', frameon=True, facecolor='white', edgecolor='#E0E0E0', framealpha=0.95)
        plt.tight_layout()
        fig.savefig(output_path.with_suffix(".png"))
        plt.close(fig)
        return True


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
            clean_lbl = raw_lbl.replace("_EMG", "").replace("_IMU", "")
            # Sensors
            clean_lbl = clean_lbl.replace("ax1", "S1 Accel X").replace("ay1", "S1 Accel Y").replace("az1", "S1 Accel Z")
            clean_lbl = clean_lbl.replace("ax2", "S2 Accel X").replace("ay2", "S2 Accel Y").replace("az2", "S2 Accel Z")
            clean_lbl = clean_lbl.replace("roll_rad1", "S1 Roll").replace("pitch_rad1", "S1 Pitch").replace("yaw_rad1", "S1 Yaw")
            clean_lbl = clean_lbl.replace("roll_rad2", "S2 Roll").replace("pitch_rad2", "S2 Pitch").replace("yaw_rad2", "S2 Yaw")
            
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
        ax.set_title(f"{model_type}: Channel Permutation Importance (Top {top_n})", pad=12)
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
        fig.savefig(output_path.with_suffix(".png"))
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
        ax.set_title(f"{model_type}: Feature Permutation Importance (Top {top_n})", pad=12)
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
        fig.savefig(output_path.with_suffix(".png"))
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
            "all": "EMG + IMU\n(Full Model)",
            "emg_only": "EMG Only",
            "imu_only": "IMU Only"
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
        ax.set_title("Generalization Accuracy vs. Inference Latency Trade-Off", pad=12)
        
        ax.grid(True, which='both', linestyle='--', linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
        
        # Expand xlim and ylim slightly to accommodate text annotations
        ax.set_xlim(min(latencies) * 0.8, max(latencies) * 1.5)
        ax.set_ylim(min(rmses) * 0.7, max(rmses) * 1.25)
        
        plt.tight_layout()
        fig.savefig(output_path.with_suffix(".png"))
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
        ax.set_title(f"{model_type}: Granular Feature Permutation Importance (Top {top_n})", pad=12)
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
        fig.savefig(output_path.with_suffix(".png"))
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
        ax.set_title("Tukey HSD Pairwise Significance (Prediction Error)", pad=12)
        ax.axvline(0, color='#333333', linestyle='-', linewidth=0.8, alpha=0.5)
        
        max_diff = max(abs(d) for d in diffs) if diffs else 1.0
        ax.set_xlim(-max_diff * 1.35 if min(diffs) < 0 else 0, max_diff * 1.35)
        
        plt.tight_layout()
        fig.savefig(output_path.with_suffix(".png"))
        plt.close(fig)
        return True


# ===========================================================================
# 5. Command Line Orchestration
# ===========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Generates premium, modular, and rho.cls styled LaTeX thesis plots from run_data.json.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        "--run-dir", "-r",
        type=str,
        default=RUN_DIR,
        help="Path to the model run folder containing run_data.json."
    )
    parser.add_argument(
        "--output-dir", "-o",
        type=str,
        default=OUTPUT_DIR,
        help="Target folder to save generated figures. Grouped cleanly in 'visualization/run_plots'."
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
    
    args = parser.parse_args()
    
    # 1. Resolve run_data.json path
    run_dir_path = Path(args.run_dir).resolve()
    json_path = run_dir_path / "run_data.json"
    
    if not json_path.exists():
        print(f"Error: run_data.json not found at: {json_path}")
        print("Please point --run-dir to a valid run directory.")
        sys.exit(1)
        
    print(f"Loading run data from: {json_path}")
    with open(json_path, 'r') as f:
        try:
            data = json.load(f)
        except Exception as e:
            print(f"Error reading JSON: {e}")
            sys.exit(1)
            
    # 2. Setup output folder
    output_dir_path = Path(args.output_dir).resolve()
    output_dir_path.mkdir(parents=True, exist_ok=True)
    print(f"Saving figures to folder: {output_dir_path}")
    
    # 3. Determine which plotters to run
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
            
    # Detect if we have a multi-modality ablation or flat run
    is_multi_ablation = False
    sub_runs = []
    if "meta" not in data:
        sub_runs = [k for k, v in data.items() if isinstance(v, dict) and "meta" in v]
        if sub_runs:
            is_multi_ablation = True

    # 4. Run plotters
    successful_count = 0
    
    # Define runs to process for standard individual plotters
    if is_multi_ablation:
        runs_to_process = [(k, data[k], f"_{k}") for k in sub_runs]
    else:
        runs_to_process = [("default", data, "")]
        
    for name in active_plots:
        plotter = plot_registry.get_plotter(name)
        if not plotter:
            continue
            
        if name in ["ablation", "tradeoff"]:
            # These plotters run once on the root multi-run dictionary
            print(f"Running '{name}' plotter (on root multi-run data)...")
            out_file = output_dir_path / f"{name}_plot"
            try:
                success = plotter.plot(data, out_file, args.width)
                if success:
                    successful_count += 1
            except Exception as e:
                import traceback
                print(f"  [ERROR] Plotter '{name}' failed with exception: {e}")
                traceback.print_exc()
        elif name == "distribution":
            # Run once on single/sub-run dictionary
            print(f"Running '{name}' plotter (once)...")
            out_file = output_dir_path / f"{name}_plot"
            try:
                run_dict = data[sub_runs[0]] if is_multi_ablation else data
                success = plotter.plot(run_dict, out_file, args.width)
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
                out_file = output_dir_path / f"{name}_plot{suffix}"
                try:
                    success = plotter.plot(run_dict, out_file, args.width)
                    if success:
                        successful_count += 1
                except Exception as e:
                    import traceback
                    print(f"  [ERROR] Plotter '{name}' failed{run_desc} with exception: {e}")
                    traceback.print_exc()
                
    print(f"\nCompleted! Generated {successful_count} visual formats successfully in {output_dir_path}.")

if __name__ == "__main__":
    main()
