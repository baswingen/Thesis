"""
comp_deepshap_modality_3.pdf — adds a third "Attention" lollipop column to the
existing DeepSHAP | Permutation channel-attribution comparison
(visualization.run_viz._plot_deepshap_modality_2), in the same ThesisStyle.

Left panel reproduces DeepSHAP channel importance; the middle panel shows the
model's own attention-pooling weights; the right panel shows the permutation channel
importance. Channels are ordered by the mean of all three methods (DeepSHAP, Attention, 
and Permutation) across both the Participant-Specialized and Generalized models.

Run attention_viz.py for BOTH regimes first, then:
    python -m visualization.attention_modality_3col
"""
import json
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from visualization.run_viz import ThesisStyle, clean_channel_label

ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = ROOT / "visualization" / "run_plots_comp"
ATTN_DIR = ROOT / "visualization" / "run_plots_attention"
PAR_RUN = ROOT / "model/model_results/final_run_par_spec/run_data.json"
GEN_RUN = ROOT / "model/model_results/final_run_lopo_2/run_data.json"


def node(d):
    return d["all"] if "all" in d else d


def chan_imp(path, key):
    return node(json.load(open(path)))["feature_importance"].get(key, {})


def rel(d):
    tot = sum(d.values()) or 1.0
    return {k: v / tot * 100 for k, v in d.items()}


def is_emg(ch):
    return "_EMG" in ch or any(m in ch for m in ["Deltoid", "Brachii", "Brachioradialis", "Ulnaris", "Radialis"])


def main():
    for f in (PAR_RUN, GEN_RUN):
        if not f.exists():
            sys.exit(f"[3col] Missing run data: {f}")
    npz = {r: ATTN_DIR / f"attention_data_{r}.npz" for r in ("kfold", "participant")}
    for r, f in npz.items():
        if not f.exists():
            sys.exit(f"[3col] Missing {f}. Run: THESIS_CV_STRATEGY={r} python -m visualization.attention_viz ...")

    # --- DeepSHAP + permutation (normalized %) ---
    par_shap = rel(chan_imp(PAR_RUN, "deepshap_channel"))
    gen_shap = rel(chan_imp(GEN_RUN, "deepshap_channel"))
    par_perm = rel(chan_imp(PAR_RUN, "permutation_channel"))
    gen_perm = rel(chan_imp(GEN_RUN, "permutation_channel"))

    # --- Attention pooling (normalized %) ---
    def load_attn(r):
        d = np.load(npz[r], allow_pickle=True)
        return rel(dict(zip([str(c) for c in d["channels"]], d["pooling"])))
    par_attn = load_attn("kfold")
    gen_attn = load_attn("participant")

    # --- channel order: mean of the 6 series (DeepSHAP, Permutation, and Attention across both models) ---
    chans = set(par_shap) | set(gen_shap) | set(par_perm) | set(gen_perm) | set(par_attn) | set(gen_attn)
    mean6 = {c: (gen_shap.get(c, 0) + par_shap.get(c, 0) + gen_perm.get(c, 0) + par_perm.get(c, 0) + gen_attn.get(c, 0) + par_attn.get(c, 0)) / 6 for c in chans}
    order = sorted(chans, key=lambda c: mean6[c], reverse=True)

    # Abbreviate muscle names to create extra horizontal space
    def clean_channel_label_abbrev(c):
        lbl = clean_channel_label(c)
        abbrevs = {
            "Brachioradialis": "BR",
            "Lateral Deltoid": "LD",
            "Biceps Brachii": "BB",
            "Anterior Deltoid": "AD",
            "Triceps Brachii": "TB",
            "Posterior Deltoid": "PD",
        }
        for k, v in abbrevs.items():
            if k in lbl:
                lbl = lbl.replace(k, v)
        return lbl

    def plot_variant(abbrev=True):
        suffix = "" if abbrev else "_full"
        labels = [clean_channel_label_abbrev(c) if abbrev else clean_channel_label(c) for c in order]
        colors = [ThesisStyle.COLOR_EMG if is_emg(c) else ThesisStyle.COLOR_IMU for c in order]

        # --- figure (ThesisStyle single-column/column layout) ---
        ThesisStyle.apply("column")
        marker_size, offset = 12, 0.12
        yts = plt.rcParams["ytick.labelsize"]
        tfs = plt.rcParams["axes.titlesize"]
        lfs = plt.rcParams["axes.labelsize"]
        legfs = plt.rcParams["legend.fontsize"]

        # We squeeze the three subplots by setting figsize to (3.46, 4.5)
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(3.46, 4.5), sharey=True)
        y = np.arange(len(order))

        def panel(ax, par, gen, title, show_axvline=False):
            vp = [par.get(c, 0.0) for c in order]
            vg = [gen.get(c, 0.0) for c in order]
            ax.hlines(y - offset, 0, vp, colors=colors, linestyles="--", linewidth=1.0, alpha=0.85)
            ax.scatter(vp, y - offset, facecolors="white", edgecolors=colors, marker="o", zorder=3, s=marker_size, linewidths=1.0, alpha=0.85)
            ax.hlines(y + offset, 0, vg, colors=colors, linestyles="-", linewidth=1.0, alpha=0.85)
            ax.scatter(vg, y + offset, color=colors, marker="o", zorder=3, s=marker_size, alpha=0.85)
            ax.set_title(title, fontproperties=fm.FontProperties(family="Fira Sans", weight="bold", size=tfs), color="black", pad=8)
            ax.grid(True, which="both", linestyle="--", linewidth=0.5, color=ThesisStyle.GRID_GRAY, alpha=0.5)
            ax.spines["top"].set_visible(False)
            ax.spines["right"].set_visible(False)
            ax.tick_params(axis="x", labelsize=yts)
            if show_axvline:
                ax.axvline(0, color="#333333", linestyle="-", linewidth=0.8, alpha=0.5)
            ThesisStyle.style_ax(ax)
            v_min = min(min(vg), min(vp)) if vg else 0.0
            v_max = max(max(vg), max(vp)) if vg else 1.0
            if v_min < 0:
                r_val = v_max - v_min
                ax.set_xlim(v_min - r_val * 0.05, v_max + r_val * 0.05)
            else:
                ax.set_xlim(0, v_max * 1.1)

        panel(ax1, par_shap, gen_shap, "DeepSHAP")
        ax1.set_yticks(y)
        ax1.set_yticklabels(labels, fontsize=yts, rotation=0, ha="right")
        ax1.invert_yaxis()
        panel(ax2, par_attn, gen_attn, "Attention")
        ax2.yaxis.set_tick_params(labelleft=False)
        panel(ax3, par_perm, gen_perm, "Permutation", show_axvline=True)
        ax3.yaxis.set_tick_params(labelleft=False)

        legend_handles = [
            Line2D([0], [0], color="#555555", linestyle="--", marker="o", markerfacecolor="white", markeredgecolor="#555555", markeredgewidth=1.0, markersize=5, label="Participant-Specialized Model"),
            Line2D([0], [0], color="#555555", linestyle="-", marker="o", markerfacecolor="#555555", markersize=5, label="Generalized Model (LOPO)"),
            Patch(facecolor=ThesisStyle.COLOR_EMG, label="EMG"),
            Patch(facecolor=ThesisStyle.COLOR_IMU, label="IMU"),
        ]
        
        # Adjust margins: give more space on the left if we have long full labels
        left_margin = 0.12 if abbrev else 0.24
        fig.subplots_adjust(left=left_margin, right=0.98, bottom=0.22, top=0.90, wspace=0.12)
        fig.canvas.draw()
        x_center = (ax1.get_position().x0 + ax3.get_position().x1) / 2.0
        fig.legend(handles=legend_handles, loc="upper center", ncol=2, bbox_to_anchor=(x_center, 0.11),
                   frameon=True, facecolor="white", edgecolor="#E0E0E0", framealpha=0.95, fontsize=legfs)
        fig.text(x_center, 0.145, "Normalized Importance", ha="center", va="center", fontsize=lfs)

        out_name = f"comp_deepshap_modality_3{suffix}"
        ThesisStyle.save_figure(fig, OUT_DIR / out_name)
        plt.close(fig)
        print(f"[3col] Wrote {OUT_DIR / out_name}.pdf / .png / .svg")

    # Generate both variants
    plot_variant(abbrev=True)
    plot_variant(abbrev=False)


if __name__ == "__main__":
    main()
