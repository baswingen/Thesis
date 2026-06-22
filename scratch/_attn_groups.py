import json, numpy as np
from pathlib import Path
ROOT=Path(".")
ATTN=ROOT/"visualization/run_plots_attention"
PAR=ROOT/"model/model_results/final_run_par_spec/run_data.json"

EMG_GROUPS={"EMG-F":["Brachioradialis","Flexor Carpi Ulnaris (FCU)","Extensor Carpi Radialis (ECR)"],
            "EMG-U":["Biceps Brachii","Triceps Brachii"],
            "EMG-S":["Anterior Deltoid","Lateral Deltoid","Posterior Deltoid"]}
IMU_GROUPS={"IMU-1":["ax1","ay1","az1","roll_rad1","pitch_rad1","yaw_rad1"],
            "IMU-2":["ax2","ay2","az2","roll_rad2","pitch_rad2","yaw_rad2"]}
def grp(key):
    name=key.replace("_EMG","").replace("_IMU","").split(" (")[0]
    if "Flexor Carpi Ulnaris" in name: name="Flexor Carpi Ulnaris (FCU)"
    if "Extensor Carpi Radialis" in name: name="Extensor Carpi Radialis (ECR)"
    for g,c in EMG_GROUPS.items():
        if name in c: return g
    for g,c in IMU_GROUPS.items():
        if name in c: return g
    return None
groups=["EMG-F","EMG-U","EMG-S","IMU-2","IMU-1"]

def node(d): return d["all"] if "all" in d else d
def agg(path,k):
    chan=node(json.load(open(path)))["feature_importance"].get(k,{})
    o={g:0.0 for g in groups}
    for kk,v in chan.items():
        g=grp(kk)
        if g: o[g]+=max(float(v),0)
    s=sum(o.values()) or 1
    return {g:o[g]/s for g in groups}

d=np.load(ATTN/"attention_data_kfold.npz",allow_pickle=True)
attn=dict(zip([str(c) for c in d["channels"]],d["pooling"]))
o={g:0.0 for g in groups}
for kk,v in attn.items():
    g=grp(kk)
    if g: o[g]+=max(float(v),0)
s=sum(o.values())
attn_g={g:o[g]/s for g in groups}

shap_g=agg(PAR,"deepshap_channel")
perm_g=agg(PAR,"permutation_channel")
print(f"{'group':8}{'DeepSHAP':>10}{'Perm':>10}{'Attention':>11}")
for g in groups:
    print(f"{g:8}{shap_g[g]*100:9.1f}%{perm_g[g]*100:9.1f}%{attn_g[g]*100:10.1f}%")
print("\nAttention group ranking:", sorted(groups,key=lambda g:-attn_g[g]))
print("Perm group ranking:     ", sorted(groups,key=lambda g:-perm_g[g]))
print("DeepSHAP group ranking: ", sorted(groups,key=lambda g:-shap_g[g]))
