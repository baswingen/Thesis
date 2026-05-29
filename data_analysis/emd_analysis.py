"""Measure the electromechanical delay (EMD): lag between EMG activation envelope
and IMU kinematic response, per segment. Tests whether the lag is load-dependent
(=> needs adaptive cross-modal attention) or roughly constant (=> a fixed window
stagger at feature-extraction time would suffice).
"""
import h5py, numpy as np, glob, os, csv
from collections import defaultdict

FS = 2000.0
DECIM = 10                      # -> 200 Hz, 5 ms lag resolution
FS_D = FS / DECIM
LAG_MIN_MS, LAG_MAX_MS = -50, 350   # search EMG-leads window (positive = accel follows EMG)
EMG_SMOOTH_MS = 30
HP_WIN_MS = 300                 # high-pass window for removing gravity/slow drift
MIN_PEAK_CORR = 0.30            # keep only segments with a clear coupling peak
MIN_DUR_S = 0.7

IMU_COLS = ['ax1','ay1','az1','roll_rad1','pitch_rad1','yaw_rad1',
            'ax2','ay2','az2','roll_rad2','pitch_rad2','yaw_rad2',
            'ax_diff','ay_diff','az_diff','roll_rad_diff','pitch_rad_diff','yaw_rad_diff']

def moving_avg(x, w):
    if w < 2: return x
    k = np.ones(w)/w
    return np.convolve(x, k, mode='same')

def block_mean(x, f):
    n = (len(x)//f)*f
    return x[:n].reshape(-1, f).mean(1)

def envelope_emg(emg):
    # emg: (N,8) MVC-normalized unrectified -> total rectified drive, smoothed
    drive = np.abs(emg).sum(1)
    return moving_avg(drive, int(EMG_SMOOTH_MS/1000*FS))

def response_accel(imu, idx):
    # dynamic acceleration magnitude of one IMU (gravity/drift removed via high-pass)
    a = imu[:, idx]                      # (N,3)
    mag = np.sqrt((a**2).sum(1))
    hp = mag - moving_avg(mag, int(HP_WIN_MS/1000*FS))
    return np.abs(hp)

def best_lag(emg_env, resp):
    # downsample
    e = block_mean(emg_env, DECIM); r = block_mean(resp, DECIM)
    n = min(len(e), len(r)); e, r = e[:n], r[:n]
    if n < int(FS_D*MIN_DUR_S): return None
    e = (e - e.mean())/(e.std()+1e-9); r = (r - r.mean())/(r.std()+1e-9)
    lmin = int(LAG_MIN_MS/1000*FS_D); lmax = int(LAG_MAX_MS/1000*FS_D)
    best, bestc = None, -2
    for L in range(lmin, lmax+1):
        if L >= 0:
            a, b = e[:n-L], r[L:]            # accel shifted earlier => accel follows emg by L
        else:
            a, b = e[-L:], r[:n+L]
        if len(a) < int(FS_D*0.4): continue
        c = float(np.mean(a*b))              # both z-scored => ~correlation
        if c > bestc: bestc, best = c, L
    return best/FS_D*1000.0, bestc          # ms, peak corr

def main():
    files = sorted(glob.glob('/Volumes/Laurens SSD/BasData/segments/*_segments.h5'))
    a_ax = [IMU_COLS.index(c) for c in ['ax2','ay2','az2']]   # distal sensor
    rows = []
    for fp in files:
        pid = os.path.basename(fp).split('_session')[0].replace('participant_','')
        sess = os.path.basename(fp).split('session_')[1][:2]
        try:
            f = h5py.File(fp,'r')
        except Exception as ex:
            print('skip', fp, ex); continue
        for k in f.keys():
            g = f[k]; at = g.attrs
            N = g['emg'].shape[0]
            if N/FS < MIN_DUR_S: continue
            emg = g['emg'][:]; imu = g['imu'][:]
            env = envelope_emg(emg); resp = response_accel(imu, a_ax)
            res = best_lag(env, resp)
            if res is None: continue
            lag, corr = res
            if corr < MIN_PEAK_CORR: continue
            rows.append(dict(pid=pid, sess=sess, seg=k,
                             state=str(at.get('state','?')),
                             weight=round(float(at.get('weight',-1)),2),
                             lag_ms=lag, corr=corr, dur_s=N/FS))
        f.close()
        print(f'{pid} s{sess}: kept {sum(1 for r in rows if r["pid"]==pid and r["sess"]==sess)}')

    if not rows:
        print('NO ROWS'); return
    lags = np.array([r['lag_ms'] for r in rows])
    print(f'\n=== OVERALL (n={len(rows)} segments, corr>{MIN_PEAK_CORR}) ===')
    print(f'lag ms: median {np.median(lags):.0f}  mean {lags.mean():.0f}  '
          f'IQR [{np.percentile(lags,25):.0f}, {np.percentile(lags,75):.0f}]  '
          f'std {lags.std():.0f}')

    print('\n=== BY STATE ===')
    for st in sorted(set(r['state'] for r in rows)):
        l = np.array([r['lag_ms'] for r in rows if r['state']==st])
        print(f'  {st:14s} n={len(l):4d}  median {np.median(l):4.0f} ms  IQR [{np.percentile(l,25):.0f},{np.percentile(l,75):.0f}]')

    print('\n=== BY WEIGHT (carrying only) ===')
    cw = defaultdict(list)
    for r in rows:
        if r['state']=='carrying': cw[r['weight']].append(r['lag_ms'])
    for w in sorted(cw):
        l=np.array(cw[w]); print(f'  {w:5.2f} kg  n={len(l):4d}  median {np.median(l):4.0f} ms  mean {l.mean():4.0f}  IQR [{np.percentile(l,25):.0f},{np.percentile(l,75):.0f}]')

    print('\n=== BY PARTICIPANT (median lag) ===')
    for p in sorted(set(r['pid'] for r in rows)):
        l=np.array([r['lag_ms'] for r in rows if r['pid']==p]); print(f'  {p}: {np.median(l):4.0f} ms (n={len(l)})')

    out='data_analysis/emd_results.csv'
    with open(out,'w',newline='') as fo:
        wr=csv.DictWriter(fo, fieldnames=list(rows[0].keys())); wr.writeheader(); wr.writerows(rows)
    print(f'\nsaved {out}')

if __name__=='__main__':
    main()
