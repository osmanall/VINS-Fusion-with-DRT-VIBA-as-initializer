import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

base = '/home/aze-pc-0331/Downloads/machine_hall'
out  = '/home/aze-pc-0331/output'
gt = {'MH01': f'{base}/MH_01_easy/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv',
      'MH02': f'{base}/MH_02_easy/MH_02_easy/mav0/state_groundtruth_estimate0/data.csv',
      'MH03': f'{base}/MH_03_medium/MH_03_medium/mav0/state_groundtruth_estimate0/data.csv',
      'MH04': f'{base}/MH_04_difficult/MH_04_difficult/mav0/state_groundtruth_estimate0/data.csv',
      'MH05': f'{base}/MH_05_difficult/MH_05_difficult/mav0/state_groundtruth_estimate0/data.csv'}
seqs = ['MH01','MH02','MH03','MH04','MH05']

def load(path):
    d = np.loadtxt(path, delimiter=',', comments='#', usecols=(0,1,2,3))
    return d[:,0]/1e9, d[:,1:4]

def align(st, sp, rt, rp):
    idx = np.clip(np.searchsorted(rt, st), 1, len(rt)-1)
    idx = np.where(st-rt[idx-1] < rt[idx]-st, idx-1, idx)
    m = np.abs(rt[idx]-st) < 0.02
    S, D = sp[m], rp[idx[m]]
    mu_s, mu_d = S.mean(0), D.mean(0)
    U,_,Vt = np.linalg.svd((S-mu_s).T @ (D-mu_d))
    dd = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1,1,dd]) @ U.T
    return (R @ sp.T).T + (mu_d - R @ mu_s)

def save_one(xy, color, title, fname, xlim, ylim):
    plt.figure(figsize=(7,6))
    plt.plot(xy[:,0], xy[:,1], color=color, lw=0.9)
    plt.xlim(xlim); plt.ylim(ylim); plt.gca().set_aspect('equal')
    plt.xlabel('x (m)'); plt.ylabel('y (m)'); plt.title(title)
    plt.tight_layout(); plt.savefig(fname, dpi=130); plt.close()
    print('saved', fname)

for s in seqs:
    gt_t, gt_p = load(gt[s])
    st_a = align(*load(f'{out}/vio_{s}_stock.csv'), gt_t, gt_p)
    dr_a = align(*load(f'{out}/vio_{s}_drt.csv'),   gt_t, gt_p)
    ax = np.concatenate([gt_p[:,0], st_a[:,0], dr_a[:,0]])
    ay = np.concatenate([gt_p[:,1], st_a[:,1], dr_a[:,1]])
    xlim = (ax.min()-0.5, ax.max()+0.5); ylim = (ay.min()-0.5, ay.max()+0.5)
    save_one(gt_p,  'black', f'{s} - ground truth', f'{out}/{s}_gt.png',    xlim, ylim)
    save_one(st_a,  'red',   f'{s} - stock VINS',   f'{out}/{s}_stock.png', xlim, ylim)
    save_one(dr_a,  'blue',  f'{s} - DRT+VI-BA',    f'{out}/{s}_drt.png',   xlim, ylim)