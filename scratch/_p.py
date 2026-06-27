import itertools
from scipy.stats import spearmanr
base=[1,2,3,4,5]
cnt=tot=0
for perm in itertools.permutations(base):
    rho,_=spearmanr(base,perm); tot+=1
    if abs(rho)>=0.9999: cnt+=1
print(f"exact two-sided p for rho=1, n=5: {cnt}/{tot} = {cnt/tot:.4f}")
