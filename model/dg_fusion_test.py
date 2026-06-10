"""
Quick local test (MPS-friendly) of CONDITIONAL DOMAIN-GENERALIZATION for EMG+IMU fusion.

Question it answers
-------------------
Under leave-one-participant-out (LOPO), plain fusion (EMG+IMU) loses to EMG-only because
the load-bearing IMU magnitude features (SMA/Mean/Max ~ a=F/m) are entangled with
per-subject calibration. Does adding a CLASS-CONDITIONAL feature-alignment loss
(conditional CORAL: align embedding 1st+2nd moments across participants *within each
weight class*) let IMU contribute so fusion beats EMG-only?

It compares three conditions on identical LOPO folds:
  (A) EMG-only            -- the baseline fusion must beat
  (B) Fusion ERM          -- plain EMG+IMU, expected to ~tie or lose to (A) under LOPO
  (C) Fusion + cond-align -- EMG+IMU with conditional moment matching across participants

Success = (C) beats (A) on the LOADED classes (>0 kg) while (B) does not.

Design choices for SPEED (this is a mechanism probe, not the final model):
  * Per-segment features are mean+std pooled over the window sequence -> one vector/segment.
    This deliberately concentrates the magnitude features where the subject-confound lives,
    which is exactly the failure mode we want to test. It is NOT the temporal st3 model.
  * Small fusion MLP, fixed epoch budget (same for all conditions -> fair comparison).
  * Pooled feature matrix is cached to a local .npz so reruns don't touch the SSD.

Usage:
  python3 model/dg_fusion_test.py                 # defaults (8 participants, 4 folds)
  python3 model/dg_fusion_test.py --participants P01 P02 P03 P04 P05 P06 --folds 3 --lam 1.0
"""
from __future__ import annotations
import argparse
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

CANON_WEIGHTS = [0.0, 0.98, 1.97, 2.95, 4.15, 5.93]  # drop excluded 0.9 / 2.24
CACHE = Path(__file__).parent / "model_results" / "dg_fusion_test_cache.npz"


# ----------------------------------------------------------------------------- data
def load_pooled(participants, force=False):
    """Load precomputed window-sequence features, mean+std pool per segment.

    Returns: Xemg (N,De), Ximu (N,Di), y (N,), subj (N,), emg_names, imu_names
    """
    key = "|".join(sorted(participants))
    if CACHE.exists() and not force:
        z = np.load(CACHE, allow_pickle=True)
        if str(z["key"]) == key:
            print(f"[cache] loaded pooled features from {CACHE.name}")
            return (z["Xemg"], z["Ximu"], z["y"], z["subj"],
                    list(z["emg_names"]), list(z["imu_names"]))

    import model.config_model as cfg
    cfg.PARTICIPANT_CONFIG["include"] = list(participants)
    from model.data_loader import DataLoader

    dl = DataLoader()
    paths = [p for p in sorted(cfg.DATABASE_CONFIG["segments_dir"].glob("*.h5"))
             if not p.name.startswith("._")]
    df = dl.load_precomputed_features(paths)
    if df.empty:
        raise RuntimeError("No features loaded -- is the SSD mounted?")

    # canonical feature key order (consistent across all segments/participants)
    w0 = df["sequence_dicts"].iloc[0][0]
    emg_names = sorted(k for k in w0 if "_EMG_" in k)
    imu_names = sorted(k for k in w0 if "_IMU_" in k)

    Xemg, Ximu, y, subj = [], [], [], []
    for _, row in df.iterrows():
        w = float(row["weight"])
        if w not in CANON_WEIGHTS:
            continue
        seq = row["sequence_dicts"]
        emg_arr = np.array([[win.get(k, 0.0) for k in emg_names] for win in seq], dtype=np.float32)
        imu_arr = np.array([[win.get(k, 0.0) for k in imu_names] for win in seq], dtype=np.float32)
        # mean + std pooling over the window axis
        e = np.concatenate([emg_arr.mean(0), emg_arr.std(0)])
        i = np.concatenate([imu_arr.mean(0), imu_arr.std(0)])
        Xemg.append(e); Ximu.append(i); y.append(w); subj.append(str(row["subject"]))

    Xemg = np.asarray(Xemg, np.float32); Ximu = np.asarray(Ximu, np.float32)
    y = np.asarray(y, np.float32); subj = np.asarray(subj)
    Xemg = np.nan_to_num(Xemg); Ximu = np.nan_to_num(Ximu)

    CACHE.parent.mkdir(parents=True, exist_ok=True)
    np.savez(CACHE, key=key, Xemg=Xemg, Ximu=Ximu, y=y, subj=subj,
             emg_names=np.array(emg_names, dtype=object),
             imu_names=np.array(imu_names, dtype=object))
    print(f"[cache] wrote {CACHE.name}  (Xemg {Xemg.shape}, Ximu {Ximu.shape})")
    return Xemg, Ximu, y, subj, emg_names, imu_names


# ----------------------------------------------------------------------------- model
class FusionMLP(nn.Module):
    def __init__(self, de, di, use_imu=True, emb=32, hid=128, p=0.3):
        super().__init__()
        self.use_imu = use_imu
        self.emg = nn.Sequential(nn.Linear(de, hid), nn.ReLU(), nn.Dropout(p))
        if use_imu:
            self.imu = nn.Sequential(nn.Linear(di, hid), nn.ReLU(), nn.Dropout(p))
        trunk_in = hid * (2 if use_imu else 1)
        self.trunk = nn.Sequential(nn.Linear(trunk_in, emb), nn.ReLU(), nn.Dropout(p))
        self.head = nn.Linear(emb, 1)

    def forward(self, xe, xi):
        h = self.emg(xe)
        if self.use_imu:
            h = torch.cat([h, self.imu(xi)], dim=1)
        z = self.trunk(h)              # embedding aligned by the DG loss
        return self.head(z).squeeze(1), z


def _cov(x):
    x = x - x.mean(0, keepdim=True)
    return (x.t() @ x) / max(x.shape[0] - 1, 1)


def conditional_align_loss(z, dom, wcls):
    """Conditional CORAL: align per-participant embedding 1st+2nd moments WITHIN each
    weight class to that class's pooled moments. Removes subject identity but keeps load."""
    total = z.new_tensor(0.0); n = 0
    for w in torch.unique(wcls):
        mw = wcls == w
        zw, dw = z[mw], dom[mw]
        doms = torch.unique(dw)
        if doms.numel() < 2 or zw.shape[0] < 6:
            continue
        mu_pool, cov_pool = zw.mean(0), _cov(zw)
        for d in doms:
            zd = zw[dw == d]
            if zd.shape[0] < 3:
                continue
            total = total + ((zd.mean(0) - mu_pool) ** 2).mean() + ((_cov(zd) - cov_pool) ** 2).mean()
            n += 1
    return total / max(n, 1)


# ----------------------------------------------------------------------------- train/eval
def standardize(tr, te):
    mu, sd = tr.mean(0, keepdims=True), tr.std(0, keepdims=True) + 1e-6
    return (tr - mu) / sd, (te - mu) / sd


def calibrate_imu(Ximu, y, subj):
    """Per-subject IMU calibration: standardize each participant's IMU features by their
    OWN mean/std (computed over all of that subject's segments). Deployment-realistic
    'few-shot calibration' -- needs a short sample of the new user's movement, no load
    labels. Removes the multiplicative subject-gain (a=F/m) that breaks cross-subject
    transfer. Std is floored by the global per-feature std so near-constant features
    don't explode (the bug in the 0kg-only baseline version)."""
    Xc = Ximu.copy()
    global_sd = Ximu.std(0) + 1e-6
    floor = 0.1 * global_sd
    for s in np.unique(subj):
        m = subj == s
        mu = Ximu[m].mean(0)
        sd = np.maximum(Ximu[m].std(0), floor)
        Xc[m] = (Ximu[m] - mu) / sd
    return Xc


def metrics(y_true, y_pred):
    err = np.abs(y_pred - y_true)
    loaded = y_true > 0.0
    return {
        "MAE": float(err.mean()),
        "MAE_0kg": float(err[~loaded].mean()) if (~loaded).any() else float("nan"),
        "MAE_loaded": float(err[loaded].mean()) if loaded.any() else float("nan"),
    }


def run_condition(name, use_imu, lam, Xemg, Ximu, y, subj, test_pid, dev, epochs, seed=0):
    torch.manual_seed(seed); np.random.seed(seed)

    te_mask = subj == test_pid
    tr_mask = ~te_mask
    Xe_tr, Xe_te = standardize(Xemg[tr_mask], Xemg[te_mask])
    Xi_tr, Xi_te = standardize(Ximu[tr_mask], Ximu[te_mask])
    y_tr, y_te = y[tr_mask], y[te_mask]

    # integer domain + weight-class codes for the conditional loss
    tr_subj = subj[tr_mask]
    dom_codes = {s: i for i, s in enumerate(sorted(set(tr_subj)))}
    dom = np.array([dom_codes[s] for s in tr_subj])
    wcode = np.array([CANON_WEIGHTS.index(round(float(w), 2)) for w in y_tr])

    t = lambda a: torch.tensor(a, device=dev)
    Xe_tr, Xi_tr, y_tr_t = t(Xe_tr), t(Xi_tr), t(y_tr)
    dom_t, wcode_t = t(dom).long(), t(wcode).long()
    Xe_te, Xi_te = t(Xe_te), t(Xi_te)

    model = FusionMLP(Xemg.shape[1], Ximu.shape[1], use_imu=use_imu).to(dev)
    opt = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-4)
    mse = nn.MSELoss()

    N = Xe_tr.shape[0]
    bs = min(1024, N)  # large batch so each (participant x class) group is populated
    for ep in range(epochs):
        model.train()
        perm = torch.randperm(N, device=dev)
        for s in range(0, N, bs):
            idx = perm[s:s + bs]
            opt.zero_grad()
            pred, z = model(Xe_tr[idx], Xi_tr[idx])
            loss = mse(pred, y_tr_t[idx])
            if use_imu and lam > 0:
                loss = loss + lam * conditional_align_loss(z, dom_t[idx], wcode_t[idx])
            loss.backward()
            opt.step()

    model.eval()
    with torch.no_grad():
        pred, _ = model(Xe_te, Xi_te)
    return metrics(y_te, pred.cpu().numpy())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--participants", nargs="+",
                    default=["P01", "P02", "P03", "P04", "P06", "P08", "P09", "P11"])
    ap.add_argument("--folds", type=int, default=4, help="how many test participants to rotate over")
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--lam", type=float, default=1.0, help="conditional-align loss weight")
    ap.add_argument("--force", action="store_true", help="rebuild feature cache")
    args = ap.parse_args()

    dev = torch.device("mps" if torch.backends.mps.is_available()
                       else "cuda" if torch.cuda.is_available() else "cpu")
    print(f"device: {dev}")

    Xemg, Ximu, y, subj, en, inn = load_pooled(args.participants, force=args.force)
    print(f"segments: {len(y)} | EMG dim {Xemg.shape[1]} | IMU dim {Ximu.shape[1]} | "
          f"participants {sorted(set(subj))}")
    Ximu_cal = calibrate_imu(Ximu, y, subj)  # per-subject 0kg-baseline calibration

    test_pids = sorted(set(subj))[:args.folds]
    # (name, use_imu, lam, imu_array)
    conditions = [
        ("A_EMG_only",         False, 0.0,      Ximu),
        ("B_Fusion_ERM",       True,  0.0,      Ximu),
        ("C_Fusion_condDG",    True,  args.lam, Ximu),
        ("D_Fusion_calib",     True,  0.0,      Ximu_cal),
        ("E_Fusion_calib+DG",  True,  args.lam, Ximu_cal),
    ]

    agg = {c[0]: {"MAE": [], "MAE_0kg": [], "MAE_loaded": []} for c in conditions}
    t0 = time.time()
    for pid in test_pids:
        line = [f"test={pid}"]
        for cname, use_imu, lam, imu_arr in conditions:
            m = run_condition(cname, use_imu, lam, Xemg, imu_arr, y, subj, pid, dev, args.epochs)
            for k in agg[cname]:
                agg[cname][k].append(m[k])
            line.append(f"{cname}: load={m['MAE_loaded']:.3f}")
        print("  " + " | ".join(line))

    print(f"\n==== MEAN OVER {len(test_pids)} LOPO FOLDS  (lambda={args.lam}, {args.epochs} ep, {time.time()-t0:.0f}s) ====")
    print(f"{'condition':<20}{'MAE_all':>10}{'MAE_0kg':>10}{'MAE_loaded':>12}")
    res = {}
    for cname, *_ in conditions:
        a = {k: float(np.mean(v)) for k, v in agg[cname].items()}
        res[cname] = a
        print(f"{cname:<20}{a['MAE']:>10.4f}{a['MAE_0kg']:>10.4f}{a['MAE_loaded']:>12.4f}")

    emg = res["A_EMG_only"]["MAE_loaded"]
    print("\n--- VERDICT (loaded-class MAE vs EMG-only baseline; <0% = fusion wins) ---")
    print(f"  A EMG-only         : {emg:.4f}")
    for cname in ["B_Fusion_ERM", "C_Fusion_condDG", "D_Fusion_calib", "E_Fusion_calib+DG"]:
        v = res[cname]["MAE_loaded"]
        print(f"  {cname:<19}: {v:.4f}  ({'+' if v > emg else ''}{(v - emg) / emg * 100:.1f}% vs EMG)")
    best = min(["B_Fusion_ERM", "C_Fusion_condDG", "D_Fusion_calib", "E_Fusion_calib+DG"],
               key=lambda c: res[c]["MAE_loaded"])
    if res[best]["MAE_loaded"] < emg:
        print(f"  => {best} makes fusion BEAT EMG-only on loaded classes. Fusion rescued.")
    else:
        print(f"  => best fusion ({best}) still trails EMG-only; gap not closed at this setting.")


if __name__ == "__main__":
    main()
