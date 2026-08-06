import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

base = '/home/aze-pc-0331/Downloads/machine_hall'
gt = {'MH01': f'{base}/MH_01_easy/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv',
      'MH02': f'{base}/MH_02_easy/MH_02_easy/mav0/state_groundtruth_estimate0/data.csv',
      'MH03': f'{base}/MH_03_medium/MH_03_medium/mav0/state_groundtruth_estimate0/data.csv',
      'MH04': f'{base}/MH_04_difficult/MH_04_difficult/mav0/state_groundtruth_estimate0/data.csv',
      'MH05': f'{base}/MH_05_difficult/MH_05_difficult/mav0/state_groundtruth_estimate0/data.csv'}
seqs = ['MH01','MH02','MH03','MH04','MH05']

def load(path):
    d = np.loadtxt(path, delimiter=',', comments='#', usecols=(0,1,2,3))
    return d[:,0]/1e9, d[:,1:4]          # time(s), xyz

def align(st, sp, rt, rp):               # SE(3) umeyama: align est(st,sp) to ref(rt,rp)
    idx = np.clip(np.searchsorted(rt, st), 1, len(rt)-1)
    idx = np.where(st-rt[idx-1] < rt[idx]-st, idx-1, idx)
    m = np.abs(rt[idx]-st) < 0.02
    S, D = sp[m], rp[idx[m]]
    mu_s, mu_d = S.mean(0), D.mean(0)
    U,_,Vt = np.linalg.svd((S-mu_s).T @ (D-mu_d))
    dd = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1,1,dd]) @ U.T
    return (R @ sp.T).T + (mu_d - R @ mu_s)

fig, axes = plt.subplots(1, 5, figsize=(24,5))
for ax, s in zip(axes, seqs):
    gt_t, gt_p = load(gt[s])
    for f, c, l in [('stock','red','stock VINS'), ('drt','blue','DRT+VI-BA')]:
        et, ep = load(f'/home/aze-pc-0331/output/vio_{s}_{f}.csv')
        a = align(et, ep, gt_t, gt_p)
        ax.plot(a[:,0], a[:,1], color=c, lw=0.9, label=l)
    ax.plot(gt_p[:,0], gt_p[:,1], 'k', lw=1.1, label='ground truth')
    ax.set_title(s); ax.set_aspect('equal'); ax.legend(fontsize=8)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
plt.tight_layout()
plt.savefig('/home/aze-pc-0331/output/vins_traj_all.png', dpi=130)
print('saved vins_traj_all.png')