import json, math, itertools, numpy as np
from pathlib import Path
from scipy.stats import spearmanr

ROOT=Path(".")
ABL=ROOT/"model/model_results/run_modality_ablation_spec"
PAR=ROOT/"model/model_results/final_run_par_spec/run_data.json"
METRIC="mae"  # figure shown is MAE v2

CANON=["IMUup","IMUfo","EMGfo","EMGup","EMGsh"]

# replicate get_class_participant_macro_metric_val: need to find it
import sys
sys.path.insert(0,str(ROOT))
from visualization.run_viz import get_class_participant_macro_metric_val

# Load all combo run_data
def combo_name(groups):
    CANSET={frozenset(CANON):"all",
            frozenset(["EMGfo","EMGup","EMGsh"]):"emg_only",
            frozenset(["IMUup","IMUfo"]):"imu_only"}
    key=frozenset(groups)
    if key in CANSET: return CANSET[key]
    return "-".join(g for g in CANON if g in key)

combos=[frozenset(c) for r in range(1,6) for c in itertools.combinations(CANON,r)]
data={}
for c in combos:
    name=combo_name(c)
    for cand in [ABL/name/"run_data.json", ABL/name/"all"/"run_data.json"]:
        if cand.exists():
            data[name]=json.load(open(cand)); break

NAME_TO_GROUPS={combo_name(c):frozenset(c) for c in combos}
combo_metrics={}
for name,rd in data.items():
    g=NAME_TO_GROUPS.get(name)
    combo_metrics[g]=get_class_participant_macro_metric_val(rd,METRIC)

v_empty=1.0
allrun=data.get("all")
if allrun and "predictions" in allrun and "y_true" in allrun["predictions"]:
    v_empty=float(np.std(np.asarray(allrun["predictions"]["y_true"],dtype=float)))

def c(S):
    S=frozenset(S)
    return v_empty if len(S)==0 else combo_metrics.get(S,v_empty)

n=len(CANON); phi={}
for g in CANON:
    others=[x for x in CANON if x!=g]; total=0.0
    for s in range(len(others)+1):
        w=math.factorial(s)*math.factorial(n-s-1)/math.factorial(n)
        for S in itertools.combinations(others,s):
            total+=w*(c(S)-c(set(S)|{g}))
    phi[g]=total
ssum=sum(max(v,0) for v in phi.values())
shap={k:max(v,0)/ssum for k,v in phi.items()}

# permutation per group from final_run_par_spec
EMG_GROUPS={"EMGfo":["Brachioradialis","Flexor Carpi Ulnaris (FCU)","Extensor Carpi Radialis (ECR)"],
            "EMGup":["Biceps Brachii","Triceps Brachii"],
            "EMGsh":["Anterior Deltoid","Lateral Deltoid","Posterior Deltoid"]}
IMU_GROUPS={"IMUup":["ax1","ay1","az1","roll_rad1","pitch_rad1","yaw_rad1"],
            "IMUfo":["ax2","ay2","az2","roll_rad2","pitch_rad2","yaw_rad2"]}
def grp(key):
    name=key.replace("_EMG","").replace("_IMU","").split(" (")[0]
    if "Flexor Carpi Ulnaris" in name: name="Flexor Carpi Ulnaris (FCU)"
    if "Extensor Carpi Radialis" in name: name="Extensor Carpi Radialis (ECR)"
    for gg,cc in {**EMG_GROUPS,**IMU_GROUPS}.items():
        if name in cc: return gg
    return None
def node(d): return d["all"] if "all" in d else d
perm_chan=node(json.load(open(PAR)))["feature_importance"].get("permutation_channel",{})
perm={g:0.0 for g in CANON}
for k,v in perm_chan.items():
    g=grp(k)
    if g: perm[g]+=max(float(v),0)
psum=sum(perm.values()); perm={g:perm[g]/psum for g in CANON}

order_s=sorted(CANON,key=lambda g:-shap[g])
order_p=sorted(CANON,key=lambda g:-perm[g])
print("Shapley:    ",[(g,round(shap[g]*100,1)) for g in order_s])
print("Permutation:",[(g,round(perm[g]*100,1)) for g in order_p])
rho,p=spearmanr([shap[g] for g in CANON],[perm[g] for g in CANON])
print(f"\nSpearman Shapley vs Permutation (n=5): rho={rho:.4f}  p={p:.4f}")
