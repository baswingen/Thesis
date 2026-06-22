"""
Attention-map visualization for the ST-Transformer3 (Aksan-style dual attention).

Loads an ALREADY-TRAINED st3 model from its `.joblib` and extracts attention by
running a forward pass on real segments with hooks — NO retraining. The attention
is already computed inside the model (the `_` in `spatial_out, _ = self.spatial_attn(...)`);
we just capture it.

Three attention sites, three questions:
  * SPATIAL (channel x channel) — which sensors attend to which (incl. EMG<->IMU).
  * POOLING (per channel)       — which channels drive the final readout (cross-checks SHAP).
  * TEMPORAL (time x time, per modality group) — which time points within a lift the
                                  model integrates (causal -> lower-triangular).

Single-regime run (extract + per-regime plots + save .npz):
    THESIS_CV_STRATEGY=kfold       python -m visualization.attention_viz \
        --model model/model_results/final_run_par_spec/all/spatio_temporal_transformer3_model.joblib
    THESIS_CV_STRATEGY=participant  python -m visualization.attention_viz \
        --model model/model_results/final_run_lopo_2/all/spatio_temporal_transformer3_model.joblib

Specialized-vs-generalized comparison (after both runs above):
    python -m visualization.attention_viz --compare

THESIS_CV_STRATEGY must match the model so the loader extracts the same feature set
the model was trained on. Outputs go to visualization/run_plots_attention/.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from model.model_archs.spatio_temporal_transformer3 import SpatioTemporalTransformerRegressor3
from model.data_loader import DataLoader
from model.config_model import DATABASE_CONFIG, USE_PRECOMPUTED_FEATURES, CV_STRATEGY

OUT_DIR = Path(__file__).resolve().parent / "run_plots_attention"
DEFAULT_MODEL = {
    "kfold": "model/model_results/final_run_par_spec/all/spatio_temporal_transformer3_model.joblib",
    "participant": "model/model_results/final_run_lopo_2/all/spatio_temporal_transformer3_model.joblib",
}
REGIME_LABEL = {"kfold": "Specialized (k-fold)", "participant": "Generalized (LOPO)"}

_EMG_ABBR = {
    "Anterior Deltoid": "A.Delt", "Lateral Deltoid": "L.Delt", "Posterior Deltoid": "P.Delt",
    "Triceps Brachii": "Triceps", "Biceps Brachii": "Biceps", "Brachioradialis": "Brachiorad",
    "Flexor Carpi Ulnaris (FCU)": "FCU", "Extensor Carpi Radialis (ECR)": "ECR",
}
ATTN_CMAP = LinearSegmentedColormap.from_list("attn", ["#f7fbff", "#2c497f", "#0a1f44"])
MOD_COLORS = {"EMG": "#b83838", "IMU-Acc": "#137054", "IMU-Ori": "#2c497f", "Other": "#888888"}


def modality_of(ch):
    if "_EMG" in ch:
        return "EMG"
    if "_IMU" in ch and any(k in ch for k in ["ax", "ay", "az", "Accel", "SVM"]):
        return "IMU-Acc"
    if "_IMU" in ch and any(k in ch for k in ["roll", "pitch", "yaw", "rad", "Orient"]):
        return "IMU-Ori"
    return "Other"


def short_label(ch):
    return _EMG_ABBR.get(ch.replace("_EMG", "").replace("_IMU", ""), ch.replace("_EMG", "").replace("_IMU", ""))


def channel_layout(channels):
    """Return ordering (by modality), short labels, modality list and group boundaries."""
    order = sorted(range(len(channels)), key=lambda i: (modality_of(channels[i]), channels[i]))
    labels = [short_label(channels[i]) for i in order]
    mods = [modality_of(channels[i]) for i in order]
    bounds = [k for k in range(1, len(channels)) if mods[k] != mods[k - 1]]
    return order, labels, mods, bounds


# ──────────────────────────────────────────────────────────────────────────
# Extraction
# ──────────────────────────────────────────────────────────────────────────
def load_segments(reg, n_segments, max_files, full_length_only=False):
    seg_dir = DATABASE_CONFIG["segments_dir"]
    h5_paths = sorted(p for p in seg_dir.glob("*.h5") if not p.name.startswith("._"))
    if not h5_paths:
        sys.exit(f"[attention_viz] No segment files in {seg_dir}. Mount the database.")
    loader = DataLoader()
    df = loader.load_and_extract_features(h5_paths[:max_files], is_sequence=True,
                                          use_precomputed=USE_PRECOMPUTED_FEATURES)
    if df.empty:
        sys.exit("[attention_viz] No data loaded.")
    X, _ = loader.prepare_for_ml(df, target_col="weight")
    if full_length_only:
        L = getattr(reg, "max_seq_len", None) or 24
        keep = [i for i in range(len(X)) if len(X.iloc[i]["sequence_dicts"]) >= L]
        X = X.iloc[keep] if keep else X
    if len(X) > n_segments:
        X = X.sample(n=n_segments, random_state=0)
    return X


def extract(reg, X, want_temporal=False):
    """Run inference with hooks; return per-layer spatial (CxC), pooling (C), temporal (group->TxT)."""
    spatial_acc, pool_acc, temporal_acc = {}, [], {}

    def spatial_hook(name):
        def hook(_m, _i, out):
            w = out[1] if isinstance(out, tuple) and len(out) > 1 else None
            if w is None:
                return
            w = w.detach().float().cpu().mean(0).numpy()
            if name not in spatial_acc:
                spatial_acc[name] = [w.copy(), 1]
            else:
                spatial_acc[name][0] += w
                spatial_acc[name][1] += 1
        return hook

    def pool_hook(_m, _i, out):
        w = out[1] if isinstance(out, tuple) and len(out) > 1 else None
        if w is not None:
            pool_acc.append(w.detach().float().cpu().squeeze(-1).mean(0).numpy())

    def temporal_hook(group):
        def hook(_m, _i, out):
            w = out[1] if isinstance(out, tuple) and len(out) > 1 else None
            if w is None:
                return
            w = w.detach().float().cpu().mean(0).numpy()  # [T, T]
            if group not in temporal_acc:
                temporal_acc[group] = [w.copy(), 1]
            else:
                if temporal_acc[group][0].shape == w.shape:   # same T (full-length pass)
                    temporal_acc[group][0] += w
                    temporal_acc[group][1] += 1
        return hook

    handles = []
    for name, module in reg.model.named_modules():
        if name.endswith("spatial_attn") and isinstance(module, nn.MultiheadAttention):
            handles.append(module.register_forward_hook(spatial_hook(name)))
        elif name.endswith("attention_pooling"):
            handles.append(module.register_forward_hook(pool_hook))
        elif want_temporal and ".temporal_attns." in name and isinstance(module, nn.MultiheadAttention):
            handles.append(module.register_forward_hook(temporal_hook(name.split(".")[-1])))
    with torch.no_grad():
        reg.predict(X)
    for h in handles:
        h.remove()

    layer_names = sorted(spatial_acc.keys())
    spatial = np.stack([spatial_acc[n][0] / spatial_acc[n][1] for n in layer_names])  # [L, C, C]
    pooling = np.mean(pool_acc, axis=0) if pool_acc else None
    temporal = {g: a[0] / a[1] for g, a in temporal_acc.items()} if temporal_acc else {}
    return spatial, pooling, temporal


# ──────────────────────────────────────────────────────────────────────────
# Single-regime plots
# ──────────────────────────────────────────────────────────────────────────
def _heat(ax, mat, labels, bounds, title):
    im = ax.imshow(mat, cmap=ATTN_CMAP, aspect="equal")
    C = len(labels)
    ax.set_xticks(range(C)); ax.set_xticklabels(labels, rotation=90, fontsize=6)
    ax.set_yticks(range(C)); ax.set_yticklabels(labels, fontsize=6)
    for b in bounds:
        ax.axhline(b - 0.5, color="#cc3333", lw=0.8); ax.axvline(b - 0.5, color="#cc3333", lw=0.8)
    ax.set_title(title, fontsize=9)
    ax.set_xlabel("Key channel", fontsize=7); ax.set_ylabel("Query channel", fontsize=7)
    return im


def plot_single(channels, spatial, pooling, temporal, regime, tag):
    order, labels, mods, bounds = channel_layout(channels)
    mean_attn = spatial.mean(0)[np.ix_(order, order)]

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    im = _heat(ax, mean_attn, labels, bounds, f"Spatial attention — {REGIME_LABEL[regime]}")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="attention weight")
    fig.tight_layout(); fig.savefig(OUT_DIR / f"attention_spatial_mean_{tag}.png", dpi=200, bbox_inches="tight"); plt.close(fig)

    nL = spatial.shape[0]; cols = min(nL, 4); rows = int(np.ceil(nL / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4.0 * cols, 3.7 * rows), squeeze=False)
    for k in range(nL):
        _heat(axes[k // cols][k % cols], spatial[k][np.ix_(order, order)], labels, bounds, f"Block {k + 1}")
    for k in range(nL, rows * cols):
        axes[k // cols][k % cols].axis("off")
    fig.tight_layout(); fig.savefig(OUT_DIR / f"attention_spatial_per_layer_{tag}.png", dpi=200, bbox_inches="tight"); plt.close(fig)

    if pooling is not None:
        p = pooling[order]
        fig, ax = plt.subplots(figsize=(9, 4))
        ax.bar(range(len(p)), p, color=[MOD_COLORS[m] for m in mods])
        ax.set_xticks(range(len(p))); ax.set_xticklabels(labels, rotation=90, fontsize=7)
        ax.set_ylabel("Attention-pooling weight", fontsize=9)
        ax.set_title(f"Per-channel importance — {REGIME_LABEL[regime]}", fontsize=10)
        ax.legend([plt.Rectangle((0, 0), 1, 1, color=c) for c in MOD_COLORS.values()],
                  MOD_COLORS.keys(), fontsize=7, ncol=4, loc="upper right")
        fig.tight_layout(); fig.savefig(OUT_DIR / f"attention_pooling_{tag}.png", dpi=200, bbox_inches="tight"); plt.close(fig)

    if temporal:
        groups = list(temporal.keys())
        fig, axes = plt.subplots(1, len(groups), figsize=(3.6 * len(groups), 3.3), squeeze=False)
        for k, g in enumerate(groups):
            T = temporal[g]
            im = axes[0][k].imshow(T, cmap=ATTN_CMAP, aspect="equal")
            axes[0][k].set_title(g, fontsize=9)
            axes[0][k].set_xlabel("Key time step"); axes[0][k].set_ylabel("Query time step")
        fig.suptitle(f"Temporal attention (causal) — {REGIME_LABEL[regime]}", fontsize=10)
        fig.tight_layout(); fig.savefig(OUT_DIR / f"attention_temporal_{tag}.png", dpi=200, bbox_inches="tight"); plt.close(fig)


# ──────────────────────────────────────────────────────────────────────────
# Comparison (specialized vs generalized)
# ──────────────────────────────────────────────────────────────────────────
def plot_compare():
    files = {r: OUT_DIR / f"attention_data_{r}.npz" for r in ("kfold", "participant")}
    for r, f in files.items():
        if not f.exists():
            sys.exit(f"[attention_viz] Missing {f}. Run the {r} extraction first (see header).")
    data = {r: np.load(f, allow_pickle=True) for r, f in files.items()}
    ch_k = list(data["kfold"]["channels"]); ch_p = list(data["participant"]["channels"])
    common = [c for c in ch_k if c in ch_p]
    if not common:
        sys.exit("[attention_viz] No common channels between the two models.")
    ik = [ch_k.index(c) for c in common]; ip = [ch_p.index(c) for c in common]
    order, labels, mods, bounds = channel_layout(common)

    sk = data["kfold"]["spatial_mean"][np.ix_(ik, ik)][np.ix_(order, order)]
    sp = data["participant"]["spatial_mean"][np.ix_(ip, ip)][np.ix_(order, order)]
    diff = sp - sk

    # Spatial: spec | gen | difference
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.4))
    vmax = max(sk.max(), sp.max())
    for ax, mat, title in [(axes[0], sk, REGIME_LABEL["kfold"]), (axes[1], sp, REGIME_LABEL["participant"])]:
        im = ax.imshow(mat, cmap=ATTN_CMAP, vmin=0, vmax=vmax, aspect="equal")
        ax.set_xticks(range(len(labels))); ax.set_xticklabels(labels, rotation=90, fontsize=6)
        ax.set_yticks(range(len(labels))); ax.set_yticklabels(labels, fontsize=6)
        for b in bounds:
            ax.axhline(b - 0.5, color="#cc3333", lw=0.8); ax.axvline(b - 0.5, color="#cc3333", lw=0.8)
        ax.set_title(f"Spatial — {title}", fontsize=10); ax.set_xlabel("Key"); ax.set_ylabel("Query")
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    dlim = np.abs(diff).max()
    im = axes[2].imshow(diff, cmap="RdBu_r", vmin=-dlim, vmax=dlim, aspect="equal")
    axes[2].set_xticks(range(len(labels))); axes[2].set_xticklabels(labels, rotation=90, fontsize=6)
    axes[2].set_yticks(range(len(labels))); axes[2].set_yticklabels(labels, fontsize=6)
    for b in bounds:
        axes[2].axhline(b - 0.5, color="#333", lw=0.6); axes[2].axvline(b - 0.5, color="#333", lw=0.6)
    axes[2].set_title("Difference (Generalized − Specialized)", fontsize=10); axes[2].set_xlabel("Key"); axes[2].set_ylabel("Query")
    fig.colorbar(im, ax=axes[2], fraction=0.046, pad=0.04)
    fig.tight_layout(); fig.savefig(OUT_DIR / "attention_spatial_compare.png", dpi=200, bbox_inches="tight"); plt.close(fig)

    # Pooling: grouped bars
    pk = data["kfold"]["pooling"][ik][order]; pp = data["participant"]["pooling"][ip][order]
    x = np.arange(len(labels)); w = 0.4
    fig, ax = plt.subplots(figsize=(11, 4.2))
    ax.bar(x - w / 2, pk, w, label=REGIME_LABEL["kfold"], color=[MOD_COLORS[m] for m in mods])
    ax.bar(x + w / 2, pp, w, label=REGIME_LABEL["participant"],
           color=[MOD_COLORS[m] for m in mods], alpha=0.55, hatch="//")
    ax.set_xticks(x); ax.set_xticklabels(labels, rotation=90, fontsize=7)
    ax.set_ylabel("Attention-pooling weight", fontsize=9)
    ax.set_title("Per-channel importance: Specialized (solid) vs Generalized (hatched)", fontsize=10)
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout(); fig.savefig(OUT_DIR / "attention_pooling_compare.png", dpi=200, bbox_inches="tight"); plt.close(fig)
    print(f"[attention_viz] Wrote comparison plots to {OUT_DIR}")


def main():
    ap = argparse.ArgumentParser(description="ST-Transformer3 attention maps (no retraining).")
    ap.add_argument("--model", default=None, help="Trained st3 .joblib (default per THESIS_CV_STRATEGY).")
    ap.add_argument("--n_segments", type=int, default=512)
    ap.add_argument("--max_files", type=int, default=8)
    ap.add_argument("--device", default="cpu", help="cpu (default; MPS can skip attn-weight return).")
    ap.add_argument("--no_temporal", action="store_true", help="Skip temporal-attention pass.")
    ap.add_argument("--compare", action="store_true", help="Build specialized-vs-generalized comparison from saved .npz.")
    args = ap.parse_args()
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    if args.compare:
        plot_compare()
        return

    regime = CV_STRATEGY if CV_STRATEGY in DEFAULT_MODEL else "kfold"
    model_path = Path(args.model or DEFAULT_MODEL[regime])
    if not model_path.exists():
        sys.exit(f"[attention_viz] Model not found: {model_path}")
    print(f"[attention_viz] Regime={regime} ({REGIME_LABEL[regime]})  model={model_path.name}")
    reg = SpatioTemporalTransformerRegressor3.load(model_path)
    reg.device = torch.device(args.device); reg.model.to(reg.device); reg.model.eval()
    channels = list(reg.model.channel_names)
    print(f"[attention_viz] {len(channels)} channels, {reg.model.num_layers} blocks")

    X = load_segments(reg, args.n_segments, args.max_files)
    print(f"[attention_viz] Spatial/pooling on {len(X)} segments...")
    spatial, pooling, _ = extract(reg, X, want_temporal=False)

    temporal = {}
    if not args.no_temporal:
        Xt = load_segments(reg, args.n_segments, args.max_files, full_length_only=True)
        print(f"[attention_viz] Temporal on {len(Xt)} full-length segments...")
        _, _, temporal = extract(reg, Xt, want_temporal=True)

    tag = regime
    plot_single(channels, spatial, pooling, temporal, regime, tag)
    np.savez(OUT_DIR / f"attention_data_{tag}.npz",
             channels=np.array(channels, dtype=object),
             spatial_mean=spatial.mean(0), pooling=pooling if pooling is not None else np.array([]))
    print(f"[attention_viz] Wrote {regime} plots + attention_data_{tag}.npz to {OUT_DIR}")


if __name__ == "__main__":
    main()
