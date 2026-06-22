import json, numpy as np
from pathlib import Path
from scipy.stats import spearmanr

ROOT = Path(".")
ATTN = ROOT/"visualization/run_plots_attention"
PAR = ROOT/"model/model_results/final_run_par_spec/run_data.json"
GEN = ROOT/"model/model_results/final_run_lopo_2/run_data.json"

def node(d): return d["all"] if "all" in d else d
def chan_imp(p,k): return node(json.load(open(p)))["feature_importance"].get(k,{})
def rel(d):
    tot=sum(d.values()) or 1.0
    return {k:v/tot*100 for k,v in d.items()}

def is_emg(ch):
    return "_EMG" in ch or any(m in ch for m in ["Deltoid","Brachii","Brachioradialis","Ulnaris","Radialis"])

def load_attn(r):
    d=np.load(ATTN/f"attention_data_{r}.npz",allow_pickle=True)
    return rel(dict(zip([str(c) for c in d["channels"]], d["pooling"])))

par_shap=rel(chan_imp(PAR,"deepshap_channel"))
gen_shap=rel(chan_imp(GEN,"deepshap_channel"))
par_perm=rel(chan_imp(PAR,"permutation_channel"))
gen_perm=rel(chan_imp(GEN,"permutation_channel"))
par_attn=load_attn("kfold")
gen_attn=load_attn("participant")

def emg_share(d): return sum(v for k,v in d.items() if is_emg(k))

print("=== Attention EMG share ===")
print(f"specialized EMG {emg_share(par_attn):.1f}%  IMU {100-emg_share(par_attn):.1f}%")
print(f"generalized EMG {emg_share(gen_attn):.1f}%  IMU {100-emg_share(gen_attn):.1f}%")

def top(d,n=5):
    return sorted(d.items(),key=lambda x:-x[1])[:n]
print("\n=== Attention top channels (specialized) ===")
for k,v in top(par_attn): print(f"  {v:5.1f}%  {k}")
print("=== Attention top channels (generalized) ===")
for k,v in top(gen_attn): print(f"  {v:5.1f}%  {k}")

# correlations of attention vs deepshap/perm, and spec vs gen attention
chans=set(par_attn)|set(par_shap)|set(par_perm)
def vec(d): return [d.get(c,0) for c in sorted(chans)]
print("\n=== Spearman (specialized) ===")
print("  attn vs deepshap:", round(spearmanr(vec(par_attn),vec(par_shap)).correlation,3))
print("  attn vs perm:    ", round(spearmanr(vec(par_attn),vec(par_perm)).correlation,3))
print("=== Spearman (generalized) ===")
print("  attn vs deepshap:", round(spearmanr(vec(gen_attn),vec(gen_shap)).correlation,3))
print("  attn vs perm:    ", round(spearmanr(vec(gen_attn),vec(gen_perm)).correlation,3))
print("=== Spearman attention spec vs gen ===")
ca=sorted(set(par_attn)&set(gen_attn))
print("  ", round(spearmanr([par_attn[c] for c in ca],[gen_attn[c] for c in ca]).correlation,3))

# max single channel share
print("\n=== Max single-channel share ===")
print(f"  attn spec: {max(par_attn.values()):.1f}%  attn gen: {max(gen_attn.values()):.1f}%")
print(f"  deepshap spec: {max(par_shap.values()):.1f}%  perm spec: {max(par_perm.values()):.1f}%")
