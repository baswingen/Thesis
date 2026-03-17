"""
HDF5 Segment Interpreter
========================
Visualises labelled segments from a *_segments.h5 file produced by
model/segmentation.py.

The slider (or mouse-wheel) scrolls through segments one by one.
Each segment shows its IMU and EMG channels in separate subplot grids.
The figure title shows the segment index, label, state, weight, and
source/target cell.

Edit the CONFIG block — nothing else needs changing.

Run:
    python database/hdf5_interpreter_seg.py
"""

# =============================================================================
# CONFIG — edit these
# =============================================================================

# Path to a *_segments.h5 file produced by model/segmentation.py
FILE = r"database/segments/participant_P01_session_02_segments.h5"

# Which signal blocks to show: "imu", "emg", or "both"
SHOW = "both"

# Columns to EXCLUDE from the plots (applies to both imu and emg)
EXCLUDE_COLUMNS: list[str] = []

# Use row index (0…N-1) or seconds on the X-axis.
# Set to True to compute seconds from sample index (useful since segments
# don't all have a t_pc_common stored in the segment file itself).
X_AS_SECONDS = True
EMG_FS  = 2000   # EMG sample rate — used only when X_AS_SECONDS is True
IMU_FS  = 2000   # IMU sample rate (same grid as EMG)

# EMG channel → muscle name mapping (used for backward compatibility with older segment files)
# For newer segment files, the column names will already be the muscle names.
import sys
from pathlib import Path
root = Path(__file__).parent.parent
if str(root) not in sys.path:
    sys.path.insert(0, str(root))

from model.config_model import EMG_CHANNEL_CONFIG

CHANNEL_LABELS: dict[str, str] = {}
for muscle, channels in EMG_CHANNEL_CONFIG.items():
    if isinstance(channels, tuple):
        for ch in channels:
            CHANNEL_LABELS[ch] = muscle
    else:
        CHANNEL_LABELS[channels] = muscle

# IMU mapped names for visualization
IMU_CHANNEL_LABELS: dict[str, str] = {
    "ax1": "Accel X (Upper Arm)",
    "ay1": "Accel Y (Upper Arm)",
    "az1": "Accel Z (Upper Arm)",
    "roll_rad1": "Roll (Upper Arm)",
    "pitch_rad1": "Pitch (Upper Arm)",
    "yaw_rad1": "Yaw (Upper Arm)",
    "ax2": "Accel X (Forearm)",
    "ay2": "Accel Y (Forearm)",
    "az2": "Accel Z (Forearm)",
    "roll_rad2": "Roll (Forearm)",
    "pitch_rad2": "Pitch (Forearm)",
    "yaw_rad2": "Yaw (Forearm)",
    "ax_diff": "Rel. Accel X (Elbow)",
    "ay_diff": "Rel. Accel Y (Elbow)",
    "az_diff": "Rel. Accel Z (Elbow)",
    "roll_rad_diff": "Rel. Roll (Elbow)",
    "pitch_rad_diff": "Rel. Pitch (Elbow)",
    "yaw_rad_diff": "Rel. Yaw (Elbow)",
}

CHANNEL_LABELS.update(IMU_CHANNEL_LABELS)

# Plot layout
COLS_PER_ROW        = 4
FIG_HEIGHT_PER_ROW  = 2.0   # inches per subplot row
FIG_WIDTH           = 20    # total figure width
MAX_VISIBLE_ROWS    = 4     # rows of subplots visible at once
STYLE               = "dark_background"
LINE_ALPHA          = 0.85
LINE_WIDTH          = 0.7

# =============================================================================
# Script — no edits needed below this line
# =============================================================================

import sys
from pathlib import Path

import h5py
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider, Button
import numpy as np

root = Path(__file__).parent.parent
h5_path = root / FILE

if not h5_path.exists():
    sys.exit(f"[ERR] File not found: {h5_path}")


# ---------------------------------------------------------------------------
# Load all segments from the HDF5 file
# ---------------------------------------------------------------------------

def load_all_segments(path: Path) -> list[dict]:
    """Read every segment_NNNN group into a list of dicts."""
    segments = []
    with h5py.File(path, "r") as f:
        keys = sorted(k for k in f.keys() if k.startswith("segment_"))
        for key in keys:
            grp = f[key]

            # --- signal arrays ---
            imu = grp["imu"][:] if "imu" in grp else np.empty((0, 0))
            emg = grp["emg"][:] if "emg" in grp else np.empty((0, 0))

            imu_cols = _decode_cols(grp["imu"].attrs.get("column_names", [])) if "imu" in grp else []
            emg_cols = _decode_cols(grp["emg"].attrs.get("column_names", [])) if "emg" in grp else []

            # --- metadata attrs ---
            def _a(name, default=""):
                v = grp.attrs.get(name, default)
                return v.decode() if isinstance(v, bytes) else v

            segments.append({
                "key":      key,
                "label":    _a("label", "?"),
                "weight":   float(grp.attrs.get("weight", -1)),
                "state":    _a("state"),
                "src_row":  int(grp.attrs.get("src_row", -1)),
                "src_col":  int(grp.attrs.get("src_col", -1)),
                "tgt_row":  int(grp.attrs.get("tgt_row", -1)),
                "tgt_col":  int(grp.attrs.get("tgt_col", -1)),
                "t_start":  float(grp.attrs.get("t_start", 0)),
                "t_end":    float(grp.attrs.get("t_end", 0)),
                "trial_file": _a("trial_file"),
                "imu":      imu,
                "emg":      emg,
                "imu_cols": imu_cols,
                "emg_cols": emg_cols,
            })
    return segments


def _decode_cols(raw) -> list[str]:
    return [c.decode() if isinstance(c, bytes) else str(c) for c in raw]


# ---------------------------------------------------------------------------
# Build the per-segment subplot data list
# ---------------------------------------------------------------------------

def _build_channel_list(seg: dict) -> list[tuple[np.ndarray, np.ndarray, str, str]]:
    """
    Return a flat list of (x, y, col_name, block_tag) tuples for the segment,
    filtered by SHOW and EXCLUDE_COLUMNS.

    x is in seconds (relative to segment start) when X_AS_SECONDS is True,
    otherwise it's sample index.
    """
    channels = []

    def _make_x(n: int, fs: float) -> np.ndarray:
        if X_AS_SECONDS:
            return np.arange(n) / fs
        return np.arange(n)

    if SHOW in ("imu", "both") and seg["imu"].size > 0:
        imu = seg["imu"]
        x = _make_x(imu.shape[0], IMU_FS)
        for i, name in enumerate(seg["imu_cols"]):
            if name not in EXCLUDE_COLUMNS and not np.all(np.isnan(imu[:, i])):
                channels.append((x, imu[:, i], name, "IMU"))

    if SHOW in ("emg", "both") and seg["emg"].size > 0:
        emg = seg["emg"]
        x = _make_x(emg.shape[0], EMG_FS)
        for i, name in enumerate(seg["emg_cols"]):
            if name not in EXCLUDE_COLUMNS and not np.all(np.isnan(emg[:, i])):
                channels.append((x, emg[:, i], name, "EMG"))

    return channels


def compute_emg_global_max(segments: list[dict]) -> dict[str, float]:
    """
    Compute the global maximum value of each EMG channel across ALL segments.
    Used to fix the y-axis so comparisons between segments are on the same scale.
    Returns a dict {channel_name: global_max}.
    """
    maxes: dict[str, float] = {}
    for seg in segments:
        if seg["emg"].size == 0:
            continue
        for i, name in enumerate(seg["emg_cols"]):
            col = seg["emg"][:, i]
            if not np.all(np.isnan(col)):
                val = float(np.nanmax(col))
                if val > maxes.get(name, 0.0):
                    maxes[name] = val
    return maxes


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_segments(segments: list[dict]):
    if not segments:
        sys.exit("[ERR] No segments found in the file.")

    plt.style.use(STYLE)
    is_dark = "dark" in STYLE
    txt_color = "white" if is_dark else "black"
    slider_fc  = "#333333" if is_dark else "#DDDDDD"
    carrying_color  = "#2ecc71"   # green
    free_color      = "#3498db"   # blue
    error_color     = "#e74c3c"   # red for unknown states

    # ---- Build figure layout ----------------------------------------
    max_cells = MAX_VISIBLE_ROWS * COLS_PER_ROW

    # Count max channels across all segments so the figure is stable
    max_channels = max(len(_build_channel_list(s)) for s in segments)
    n_visible_plots = min(max_channels, max_cells)
    n_rows_visible  = max(1, int(np.ceil(n_visible_plots / COLS_PER_ROW)))
    total_pot_rows  = max(1, int(np.ceil(max_channels / COLS_PER_ROW)))

    fig = plt.figure(figsize=(FIG_WIDTH, FIG_HEIGHT_PER_ROW * n_rows_visible + 1.6))
    # Reserve bottom strip for navigation controls
    plt.subplots_adjust(bottom=0.12, right=0.93, top=0.88, left=0.04,
                        hspace=0.55, wspace=0.3)

    gs = gridspec.GridSpec(
        n_rows_visible, COLS_PER_ROW, figure=fig,
        bottom=0.12, top=0.88, right=0.93, left=0.04,
        hspace=0.55, wspace=0.3
    )

    # ---- Build subplot axes -----------------------------------------
    axes  = []
    lines = []
    base_ax = None
    for k in range(n_visible_plots):
        r, c = divmod(k, COLS_PER_ROW)
        ax = fig.add_subplot(gs[r, c], sharex=base_ax)
        if base_ax is None:
            base_ax = ax
        line, = ax.plot([], [], lw=LINE_WIDTH, alpha=LINE_ALPHA)
        ax.tick_params(labelsize=6)
        ax.margins(x=0.01)
        axes.append(ax)
        lines.append(line)

    # ---- Channel-scroll slider (right) ------------------------------
    ax_ch_slider = plt.axes([0.945, 0.12, 0.012, 0.76], facecolor=slider_fc)
    ch_slider = Slider(
        ax_ch_slider, "", 0, max(0, total_pot_rows - MAX_VISIBLE_ROWS),
        valinit=0, valstep=1, orientation="vertical"
    )
    ch_slider.label.set_visible(False)

    # ---- Segment navigation slider (bottom) -------------------------
    ax_seg_slider = plt.axes([0.10, 0.04, 0.65, 0.025], facecolor=slider_fc)
    seg_slider = Slider(
        ax_seg_slider, "Segment", 0, len(segments) - 1,
        valinit=0, valstep=1,
    )
    seg_slider.label.set_color(txt_color)

    # ---- Prev / Next buttons ----------------------------------------
    ax_prev = plt.axes([0.78, 0.03, 0.07, 0.045])
    ax_next = plt.axes([0.86, 0.03, 0.07, 0.045])
    btn_prev = Button(ax_prev, "◀  Prev", color="#555555", hovercolor="#888888")
    btn_next = Button(ax_next, "Next  ▶", color="#555555", hovercolor="#888888")

    # ---- State -------------------------------------------------------
    state = {"seg_idx": 0, "ch_row": 0}

    color_map = {
        "free_movement": free_color,
        "carrying":      carrying_color,
    }

    x_label = "time [s]" if X_AS_SECONDS else "sample index"
    cmap = plt.colormaps["tab20"]

    def _render():
        seg_idx = state["seg_idx"]
        ch_row  = state["ch_row"]
        seg     = segments[seg_idx]

        channels = _build_channel_list(seg)
        n_ch     = len(channels)
        total_ch_rows = max(1, int(np.ceil(n_ch / COLS_PER_ROW)))
        start_ch = ch_row * COLS_PER_ROW

        # Update channel-scroll slider range
        ch_slider.valmax = max(0, total_ch_rows - MAX_VISIBLE_ROWS)
        ch_slider.ax.set_ylim(ch_slider.valmin, ch_slider.valmax)

        # Figure title
        dur = seg["t_end"] - seg["t_start"]
        src = f"({chr(65+seg['src_row'])}{seg['src_col']+1})" if seg["src_row"] >= 0 else "(–)"
        tgt = f"({chr(65+seg['tgt_row'])}{seg['tgt_col']+1})" if seg["tgt_row"] >= 0 else "(–)"
        tc  = color_map.get(seg["state"], error_color)

        fig.suptitle(
            f"Segment {seg_idx+1}/{len(segments)}   "
            f"[{seg['key']}]   "
            f"label = {seg['label']}   "
            f"state = {seg['state']}   "
            f"weight = {seg['weight']} kg   "
            f"src {src} → tgt {tgt}   "
            f"dur = {dur:.2f} s   "
            f"n = {seg['imu'].shape[0]} samp",
            fontsize=9, color=tc, y=0.97, wrap=True
        )

        for k in range(n_visible_plots):
            ch_idx = start_ch + k
            ax   = axes[k]
            line = lines[k]

            if ch_idx < n_ch:
                x_data, y_data, col_name, block = channels[ch_idx]
                ax.set_visible(True)
                line.set_data(x_data, y_data)
                line.set_color(cmap(ch_idx % 20))
                label_str = CHANNEL_LABELS.get(col_name, "")
                title_str = f"[{block}] {col_name}" + (f" — {label_str}" if label_str else "")
                ax.set_title(title_str, fontsize=7, pad=2, color=txt_color)
                ax.set_xlabel(x_label, fontsize=6, labelpad=1)
                if len(x_data):
                    ax.set_xlim(x_data[0], x_data[-1])
                if not np.all(np.isnan(y_data)):
                    ymin, ymax = np.nanmin(y_data), np.nanmax(y_data)
                    if ymin == ymax:
                        ymin -= 0.1; ymax += 0.1
                    else:
                        m = (ymax - ymin) * 0.05
                        ymin -= m; ymax += m
                    ax.set_ylim(ymin, ymax)
            else:
                ax.set_visible(False)

        fig.canvas.draw_idle()

    def _on_seg_change(val):
        state["seg_idx"] = int(seg_slider.val)
        state["ch_row"]  = 0
        ch_slider.set_val(0)
        _render()

    def _on_ch_change(val):
        state["ch_row"] = int(ch_slider.val)
        _render()

    def _prev(_):
        new = max(0, state["seg_idx"] - 1)
        seg_slider.set_val(new)   # triggers _on_seg_change

    def _next(_):
        new = min(len(segments) - 1, state["seg_idx"] + 1)
        seg_slider.set_val(new)

    seg_slider.on_changed(_on_seg_change)
    ch_slider.on_changed(_on_ch_change)
    btn_prev.on_clicked(_prev)
    btn_next.on_clicked(_next)

    def _on_scroll(event):
        # Scroll over main plots → navigate channel rows
        # Scroll over seg slider → navigate segments
        if event.inaxes == ax_seg_slider:
            step = -1 if event.button == "up" else 1
            seg_slider.set_val(
                int(np.clip(seg_slider.val + step, 0, len(segments) - 1))
            )
        else:
            step = -1 if event.button == "up" else 1
            ch_slider.set_val(
                int(np.clip(ch_slider.val + step, 0, ch_slider.valmax))
            )

    def _on_key(event):
        if event.key in ("right", "n"):
            _next(None)
        elif event.key in ("left", "p"):
            _prev(None)

    fig.canvas.mpl_connect("scroll_event", _on_scroll)
    fig.canvas.mpl_connect("key_press_event", _on_key)

    _render()
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print(f"[INFO] Loading segments from : {h5_path}")
    segments = load_all_segments(h5_path)
    print(f"[INFO] Segments loaded        : {len(segments)}")
    if segments:
        s0 = segments[0]
        print(f"[INFO] IMU shape (seg 0)      : {s0['imu'].shape}  cols: {s0['imu_cols']}")
        print(f"[INFO] EMG shape (seg 0)      : {s0['emg'].shape}  cols: {s0['emg_cols'][:5]}…")

    plot_segments(segments)
