import json, numpy as np
from pathlib import Path
from scipy.stats import spearmanr
ROOT=Path(".")
ATTN=ROOT/"visualization/run_plots_attention"
PAR=ROOT/"model/model_results/final_run_par_spec/run_data.json"
GEN=ROOT/"model/model_results/final_run_lopo_2/run_data.json"
def node(d): return d["all"] if "all" in d else d
def chan(p,k): return node(json.load(open(p)))["feature_importance"].get(k,{})
def load_attn(r):
    d=np.load(ATTN/f"attention_data_{r}.npz",allow_pickle=True)
    return dict(zip([str(c) for c in d["channels"]], d["pooling"]))

def cross(a,b,name):
    keys=sorted(set(a)&set(b))
    rho,p=spearmanr([a[k] for k in keys],[b[k] for k in keys])
    print(f"{name:12} rho={rho:.4f}  p={p:.3e}  n={len(keys)}")

cross(chan(PAR,"deepshap_channel"),    chan(GEN,"deepshap_channel"),    "DeepSHAP")
cross(chan(PAR,"permutation_channel"), chan(GEN,"permutation_channel"), "Permutation")
cross(load_attn("kfold"),              load_attn("participant"),        "Attention")
