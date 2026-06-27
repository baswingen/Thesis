import json, os
import numpy as np

ROOT = "model/model_results"

PAR = {  # Participant-specific (k-fold)
    "ST-Transformer": f"{ROOT}/final_run_par_spec/all/run_data.json",
    "LSTM":           f"{ROOT}/arch_comp/run_20260621_115216_kfold_t5/run_data.json",
    "GRU":            f"{ROOT}/arch_comp/run_20260621_115711_kfold_t6/run_data.json",
    "Naive-Trans":    f"{ROOT}/arch_comp/run_20260621_123754_kfold_t9/run_data.json",
    "CNN-GRU":        f"{ROOT}/arch_comp/run_20260621_123754_kfold_t8/run_data.json",
    "CNN-LSTM":       f"{ROOT}/arch_comp/run_20260621_121031_kfold_t7/run_data.json",
}
LOPO = {  # Generalized (leave-one-participant-out)
    "ST-Transformer": f"{ROOT}/final_run_lopo_2/all/run_data.json",
    "LSTM":           f"{ROOT}/arch_comp/run_20260621_110726_participant_t0/run_data.json",
    "GRU":            f"{ROOT}/arch_comp/run_20260621_110726_participant_t1/run_data.json",
    "Naive-Trans":    f"{ROOT}/arch_comp/run_20260621_113022_participant_t4/run_data.json",
    "CNN-GRU":        f"{ROOT}/arch_comp/run_20260621_113022_participant_t3/run_data.json",
    "CNN-LSTM":       f"{ROOT}/arch_comp/run_20260621_110726_participant_t2/run_data.json",
}

def load(p):
    d = json.load(open(p))
    if isinstance(d, dict) and 'all' in d and set(d.keys()) <= {'all','emg_only','imu_only'}:
        d = d['all']
    return d['predictions']

# Establish class centroids from union of y_true (par-spec)
all_true = []
for name, p in PAR.items():
    all_true.extend(load(p)['y_true'])
classes = np.sort(np.unique(np.round(all_true, 2)))
gaps = np.diff(classes)
print("Weight classes:", classes, "\n")

def analyse(runs, title):
    print(f"=== {title} ===")
    hdr = f"{'Model':<14} {'MAE':>6} {'snap%':>7} {'valley%':>8}"
    print(hdr); print("-"*len(hdr))
    out = {}
    for name, p in runs.items():
        pr = load(p)
        yt = np.array(pr['y_true']); yp = np.maximum(0.0, np.array(pr['y_pred']))
        mae = np.mean(np.abs(yp-yt))
        tol = 0.10 * np.median(gaps)
        dist = np.min(np.abs(yp[:,None]-classes[None,:]), axis=1)
        snap = np.mean(dist < tol)
        mid = 0
        for a,b in zip(classes[:-1],classes[1:]):
            lo,hi = a+0.4*(b-a), a+0.6*(b-a)
            mid += np.sum((yp>=lo)&(yp<hi))
        valley = mid/len(yp)
        out[name] = (snap, valley)
        print(f"{name:<14} {mae:>6.3f} {100*snap:>6.1f}% {100*valley:>7.1f}%")
    print()
    return out

p = analyse(PAR,  "PARTICIPANT-SPECIFIC")
l = analyse(LOPO, "GENERALIZED (LOPO)")

print("snap%   = % preds within 10% of a class gap of nearest training-weight centroid (on-rail)")
print("valley% = % preds in the central 20% band between adjacent classes (genuine interpolation)")
