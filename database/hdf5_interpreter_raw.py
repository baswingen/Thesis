"""
HDF5 Trial Interpreter
======================
Plots every selected column of a dataset inside a trial .h5 file.
Edit the CONFIG block below — nothing else needs to change.

Run:
    python database/hdf5_interpreter.py
"""

# =============================================================================
# CONFIG — edit these
# =============================================================================

# Path to the .h5 file you want to inspect
# Layout: database/participant_PXX/session_NN/trial_N_TIMESTAMP.h5
FILE = r"database\participant_P09\session_01\trial_2_20260408_164859.h5"

# Dataset path inside the file.
# Examples:
#   "_raw/stm32"     – raw STM32 data  (22 cols)
#   "_raw/emg"       – raw EMG data    (35 cols)
#   "_raw/metrics"   – sync metrics    (3 cols)
#   "synced/data"    – full synced matrix (56 cols)
DATASET = "synced/data"

# Name of the column that goes on the X-axis.
# Set to None to use the row index instead.
X_COLUMN = "t_pc_common"

# Use relative time (start at 0) or absolute time.
RELATIVE_TIME = True

# Columns to INCLUDE.  Set to [] (empty list) to include ALL columns.
# Example: ["yaw1", "pitch1", "roll1", "prbs_lvl"]
INCLUDE_COLUMNS = []

# Columns to EXCLUDE (ignored if INCLUDE_COLUMNS is set).
# Useful for dropping time/index columns from the signal plots.
# Example: ["t_pc", "t_stm32", "imu1_ok", "imu2_ok"]
EXCLUDE_COLUMNS = ["t_pc_common", "t_pc", "t_stm32", "t_tmsi"]

# Maximum number of rows to load (None = all rows).
# Useful for large datasets during quick inspection.
MAX_ROWS = None

# Plot layout
COLS_PER_ROW   = 3      # subplots per row
FIG_HEIGHT_PER_ROW = 2  # inches per row of subplots
FIG_WIDTH      = 18     # total figure width in inches
MAX_VISIBLE_ROWS = 5    # show max N rows at once, scroll for the rest
STYLE          = "dark_background"   # matplotlib style, e.g. "dark_background", "seaborn-v0_8"
LINE_ALPHA     = 0.85
LINE_WIDTH     = 0.8

# =============================================================================
# Script — no edits needed below this line
# =============================================================================

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider
import h5py

# Resolve path relative to this script's parent directory
root = Path(__file__).parent.parent
h5_path = root / FILE

if not h5_path.exists():
    sys.exit(f"[ERR] File not found: {h5_path}")


def load_dataset(path: Path, dataset: str, max_rows=None):
    with h5py.File(path, "r") as f:
        if dataset not in f:
            available = []
            f.visititems(lambda n, o: available.append(n) if hasattr(o, "shape") else None)
            sys.exit(
                f"[ERR] Dataset '{dataset}' not found in file.\n"
                f"Available datasets:\n  " + "\n  ".join(available)
            )
        dset = f[dataset]
        col_names = list(dset.attrs.get("column_names", [f"col_{i}" for i in range(dset.shape[1])]))
        data = dset[:max_rows] if max_rows else dset[:]
    return data, col_names


def resolve_columns(col_names, include, exclude, x_col):
    """Return (x_idx_or_None, [(idx, name), ...]) for the columns to plot."""
    # X column
    x_idx = None
    if x_col and x_col in col_names:
        x_idx = col_names.index(x_col)

    if include:
        plot_cols = [(col_names.index(c), c) for c in include if c in col_names]
        missing = [c for c in include if c not in col_names]
        if missing:
            print(f"[WARN] INCLUDE_COLUMNS not found in dataset, skipping: {missing}")
    else:
        plot_cols = [
            (i, c) for i, c in enumerate(col_names)
            if c not in exclude and c != x_col
        ]

    return x_idx, plot_cols


def plot(data, col_names, x_idx, plot_cols, dataset_path, file_path):
    n_plots = len(plot_cols)
    if n_plots == 0:
        sys.exit("[ERR] No columns to plot after applying INCLUDE/EXCLUDE filters.")

    # Determine window size
    max_cells = MAX_VISIBLE_ROWS * COLS_PER_ROW
    n_visible_plots = min(n_plots, max_cells)
    n_rows_visible = int(np.ceil(n_visible_plots / COLS_PER_ROW))
    
    total_potential_rows = int(np.ceil(n_plots / COLS_PER_ROW))

    plt.style.use(STYLE)
    
    # Create figure with extra space for slider on the right
    fig = plt.figure(figsize=(FIG_WIDTH, FIG_HEIGHT_PER_ROW * n_rows_visible + 0.8))
    fig.suptitle(
        f"{Path(file_path).name}  →  {dataset_path}   "
        f"({data.shape[0]} rows × {data.shape[1]} cols)",
        fontsize=11, color="white" if "dark" in STYLE else "black", y=0.99
    )

    # Adjust layout to make room for a vertical slider on the right
    # GridSpec for subplots
    gs = gridspec.GridSpec(n_rows_visible, COLS_PER_ROW, figure=fig,
                           hspace=0.55, wspace=0.3)
    plt.subplots_adjust(right=0.94)

    # Build X axis
    if x_idx is not None:
        x = data[:, x_idx]
        x_label = col_names[x_idx]
        if RELATIVE_TIME:
            # Shift so the first valid value is 0
            valid_x = x[~np.isnan(x)]
            if len(valid_x) > 0:
                x = x - valid_x[0]
            x_label = f"relative {x_label} [s]"
    else:
        x = np.arange(data.shape[0])
        x_label = "row index"

    cmap = plt.colormaps["tab20"].resampled(n_plots)
    
    # Store axes and line objects for updating
    axes = []
    lines = []
    titles = []

    # Shared X-axis
    base_ax = None

    for k in range(n_visible_plots):
        row_g, col_g = divmod(k, COLS_PER_ROW)
        ax = fig.add_subplot(gs[row_g, col_g], sharex=base_ax)
        if base_ax is None:
            base_ax = ax
            
        line, = ax.plot([], [], lw=LINE_WIDTH, alpha=LINE_ALPHA)
        
        ax.set_xlabel(x_label, fontsize=6, labelpad=1)
        ax.tick_params(labelsize=6)
        ax.margins(x=0.01)
        
        axes.append(ax)
        lines.append(line)
        titles.append(ax.set_title("", fontsize=8, pad=3))

    # Explicitly set X range once (shared by all)
    if len(x) > 0:
        valid_x = x[~np.isnan(x)]
        if len(valid_x) > 0:
            base_ax.set_xlim(np.min(valid_x), np.max(valid_x))

    # Add slider axis [left, bottom, width, height]
    ax_slider = plt.axes([0.96, 0.15, 0.015, 0.7], facecolor='#333333' if "dark" in STYLE else '#DDDDDD')
    
    # Slider value is the index of the first Row shown
    slider = Slider(
        ax_slider, 'Row', 0, max(0, total_potential_rows - MAX_VISIBLE_ROWS), 
        valinit=0, valstep=1, orientation='vertical'
    )
    slider.label.set_visible(False)

    def update_view(val):
        start_row = int(slider.val)
        start_plot_idx = start_row * COLS_PER_ROW
        
        for i in range(n_visible_plots):
            plot_idx = start_plot_idx + i
            ax = axes[i]
            line = lines[i]
            
            if plot_idx < n_plots:
                col_idx, col_name = plot_cols[plot_idx]
                y = data[:, col_idx]
                
                ax.set_visible(True)
                line.set_data(x, y)
                line.set_color(cmap(plot_idx))
                ax.set_title(col_name, fontsize=8, pad=3)
                
                # Rescale Y
                if not np.all(np.isnan(y)):
                    ymin, ymax = np.nanmin(y), np.nanmax(y)
                    if ymin == ymax:
                        ymin -= 0.1
                        ymax += 0.1
                    else:
                        margin = (ymax - ymin) * 0.05
                        ymin -= margin
                        ymax += margin
                    ax.set_ylim(ymin, ymax)
            else:
                ax.set_visible(False)
        
        fig.canvas.draw_idle()

    slider.on_changed(update_view)
    
    # Initial call to populate data
    update_view(0)

    # Mouse wheel support
    def on_scroll(event):
        if event.button == 'up':
            slider.set_val(max(slider.valmin, slider.val - 1))
        elif event.button == 'down':
            slider.set_val(min(slider.valmax, slider.val + 1))

    fig.canvas.mpl_connect('scroll_event', on_scroll)

    plt.show()


if __name__ == "__main__":
    print(f"[INFO] Loading  : {h5_path}")
    print(f"[INFO] Dataset  : {DATASET}")

    data, col_names = load_dataset(h5_path, DATASET, MAX_ROWS)
    print(f"[INFO] Shape    : {data.shape}")
    print(f"[INFO] Columns  : {col_names}")

    x_idx, plot_cols = resolve_columns(col_names, INCLUDE_COLUMNS, EXCLUDE_COLUMNS, X_COLUMN)
    print(f"[INFO] Plotting {len(plot_cols)} column(s) vs {X_COLUMN or 'row index'}")

    plot(data, col_names, x_idx, plot_cols, DATASET, h5_path)
