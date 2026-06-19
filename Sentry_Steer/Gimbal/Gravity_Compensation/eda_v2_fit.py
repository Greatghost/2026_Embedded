"""
Gravity Compensation Fit v2 — Hysteresis-aware
Pair up/down sweep segments, average to cancel friction.
"""
import csv, os, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

os.chdir(os.path.dirname(os.path.abspath(__file__)))
CSV = 'Ozone_DataGraph_26061902.csv'

# ===== Load & Segment =====
with open(CSV, 'r') as f:
    reader = csv.reader(f, delimiter=';')
    header = next(reader)
    rows = [[float(x) for x in r] for r in reader]

time = np.array([r[0]/1e6 for r in rows])
gyro = np.array([r[1] for r in rows])
current = np.array([r[2] for r in rows])
target = np.array([r[3] for r in rows])

changes = [0]
for i in range(1, len(target)):
    if abs(target[i] - target[i-1]) > 0.3:
        changes.append(i)
changes.append(len(target)-1)

segments = []
for i in range(len(changes)-1):
    s, e = changes[i], changes[i+1]
    if e - s < 50: continue
    dur = time[e] - time[s]
    if dur < 1.0: continue
    ss_s = s + (e - s) * 8 // 10
    gs = np.mean(gyro[ss_s:e])
    cs = np.mean(current[ss_s:e])
    ts = np.mean(target[ss_s:e])
    saturated = abs(cs) > 580
    segments.append({'target': ts, 'gyro': gs, 'current': cs, 'saturated': saturated})

# Remove power-up (first segment)
segments = segments[1:]

# ===== Split into down-sweep and up-sweep =====
# Down: target decreasing (-21 → +11), Up: target increasing (+10 → -23)
# The first half of segments (target going more positive) = down sweep
# The second half (target going more negative) = up sweep

# Actually, let's use target direction: find the point where target starts going back
# Down sweep: segments where target is monotonically increasing (downward motor motion)
# Up sweep: segments where target is monotonically decreasing (upward motor motion)

# Simpler: split at the midpoint. First batch of unique targets going UP (down sweep)
# then targets going DOWN (up sweep).
n = len(segments)
mid = n // 2

# Actually, let's pair by target value
# Down sweep: segments with increasing target (first half, target from -21→11)
# Up sweep: segments with decreasing target (second half, target from 10→-23)

# Pair by closest target
down_sweep = []  # first pass, target increasing
up_sweep = []    # second pass, target decreasing

# Find the turning point: where target direction reverses
for i in range(1, n):
    if segments[i]['target'] < segments[i-1]['target'] - 0.5:
        # target started decreasing — this is the up sweep start
        down_sweep = segments[:i]
        up_sweep = segments[i:]
        break

print(f"Down sweep: {len(down_sweep)} segments (target {down_sweep[0]['target']:.0f} -> {down_sweep[-1]['target']:.0f})")
print(f"Up sweep:   {len(up_sweep)} segments (target {up_sweep[0]['target']:.0f} -> {up_sweep[-1]['target']:.0f})")

# ===== Pair by target and average =====
pairs = []
for ds in down_sweep:
    if ds['saturated']: continue
    # Find closest up-sweep segment by target
    best = None
    best_dt = 999
    for us in up_sweep:
        if us['saturated']: continue
        dt = abs(ds['target'] - us['target'])
        if dt < best_dt:
            best_dt = dt
            best = us
    if best and best_dt < 1.5:
        g_avg = (ds['gyro'] + best['gyro']) / 2
        c_avg = (ds['current'] + best['current']) / 2
        pairs.append({
            'gyro_avg': g_avg,
            'current_avg': c_avg,
            'gyro_down': ds['gyro'],
            'current_down': ds['current'],
            'gyro_up': best['gyro'],
            'current_up': best['current'],
            'target': ds['target'],
            'friction': abs(ds['current'] - best['current']) / 2
        })

print(f"Paired points: {len(pairs)}")

# Remove outliers: points where friction is > 3x median
frictions = [p['friction'] for p in pairs]
med_fric = np.median(frictions)
clean_pairs = [p for p in pairs if p['friction'] < 3 * med_fric]
print(f"After friction outlier removal: {len(clean_pairs)}")

# ===== Fit =====
gyro_fit = np.array([p['gyro_avg'] for p in clean_pairs])
curr_fit = np.array([p['current_avg'] for p in clean_pairs])

def poly4(x, p1, p2, p3, p4, p5):
    return p1*x**4 + p2*x**3 + p3*x**2 + p4*x + p5

def poly3(x, p1, p2, p3, p4):
    return p1*x**3 + p2*x**2 + p3*x + p4

fits = {}
gyro_fine = np.linspace(gyro_fit.min()-2, gyro_fit.max()+2, 200)

for order, poly_fn, p0 in [
    (3, poly3, [-0.5, -1.5, 20, 500]),
    (4, poly4, [0.001, -0.02, -1.5, 25, 510])
]:
    try:
        popt, pcov = curve_fit(poly_fn, gyro_fit, curr_fit, p0=p0, maxfev=10000)
        perr = np.sqrt(np.diag(pcov))
    except:
        deg = 3 if order==3 else 4
        c = np.polyfit(gyro_fit, curr_fit, deg)[::-1]
        popt = c[:order+1]
        perr = [np.nan]*(order+1)
    pred = poly_fn(gyro_fit, *popt)
    rmse = np.sqrt(np.mean((curr_fit - pred)**2))
    r2 = 1 - np.sum((curr_fit - pred)**2) / np.sum((curr_fit - np.mean(curr_fit))**2)
    fits[order] = {'opt': popt, 'err': perr, 'rmse': rmse, 'r2': r2,
                   'curve': poly_fn(gyro_fine, *popt), 'pred': pred}

# ===== Print =====
print()
for order in [3, 4]:
    f = fits[order]
    names = ['p1','p2','p3','p4','p5'][:order+1]
    print(f"=== {order}rd-order: RMSE={f['rmse']:.1f}, R²={f['r2']:.4f} ===")
    for name, p, e in zip(names, f['opt'], f['err']):
        sig = '✓' if (np.isnan(e) or abs(p) > 2*e) else '✗ 不显著'
        print(f"  {name:>4s} = {p:12.6f} ± {e:10.6f}  {sig}")

# Pick best
best = 3 if fits[3]['rmse'] <= fits[4]['rmse'] * 1.05 else 4
bf = fits[best]
print(f"\n>>> Selected: {best}rd-order")

# ===== C code =====
names_c = ['p1','p2','p3','p4','p5'][:best+1]
if best == 3:
    c_code = f"""float GimbalPitchComp()
{{
    // Poly3拟合, 上下行平均消摩擦, RMSE={bf['rmse']:.1f}, R²={bf['r2']:.4f}
    const static float p1 = {bf['opt'][0]:.6f}f;  // x^3
    const static float p2 = {bf['opt'][1]:.6f}f;  // x^2
    const static float p3 = {bf['opt'][2]:.6f}f;  // x
    const static float p4 = {bf['opt'][3]:.6f}f;  // const

    float x = gimbal_controller.gyro_pitch_angle;
    float x2 = x * x;
    float x3 = x2 * x;
    float comp_current = p1 * x3 + p2 * x2 + p3 * x + p4;

    iir(&gimbal_controller.comp_pitch_current, comp_current, 0.7f);
    return gimbal_controller.comp_pitch_current;
}}"""
else:
    c_code = f"""float GimbalPitchComp()
{{
    // Poly4拟合, 上下行平均消摩擦, RMSE={bf['rmse']:.1f}, R²={bf['r2']:.4f}
    const static float p1 = {bf['opt'][0]:.6f}f;  // x^4
    const static float p2 = {bf['opt'][1]:.6f}f;  // x^3
    const static float p3 = {bf['opt'][2]:.6f}f;  // x^2
    const static float p4 = {bf['opt'][3]:.6f}f;  // x
    const static float p5 = {bf['opt'][4]:.6f}f;  // const

    float x = gimbal_controller.gyro_pitch_angle;
    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float comp_current = p1 * x4 + p2 * x3 + p3 * x2 + p4 * x + p5;

    iir(&gimbal_controller.comp_pitch_current, comp_current, 0.7f);
    return gimbal_controller.comp_pitch_current;
}}"""

with open('gimbal_pitch_comp_v2.c', 'w', encoding='utf-8') as f:
    f.write(c_code)

# ===== Save results =====
with open('fit_results_v2.txt', 'w', encoding='utf-8') as f:
    f.write(f"Gravity Compensation Fit v2 — Hysteresis-averaged\n")
    f.write(f"Paired: {len(pairs)}, Clean: {len(clean_pairs)}\n\n")
    for order in [3, 4]:
        ff = fits[order]
        f.write(f"{order}rd: RMSE={ff['rmse']:.1f}, R²={ff['r2']:.4f}\n")
        for name, p, e in zip(['p1','p2','p3','p4','p5'][:order+1], ff['opt'], ff['err']):
            f.write(f"  {name} = {p:.6f} ± {e:.6f}\n")
        f.write("\n")
    f.write(f"Selected: {best}rd-order\n")
    f.write(f"Data points:\n")
    for p in sorted(clean_pairs, key=lambda x: x['gyro_avg']):
        f.write(f"  gyro={p['gyro_avg']:8.3f}  I_avg={p['current_avg']:8.1f}  "
                f"I_down={p['current_down']:7.1f}  I_up={p['current_up']:7.1f}  "
                f"fric={p['friction']:6.1f}\n")

print(f"Saved: gimbal_pitch_comp_v2.c, fit_results_v2.txt")

# ===== Plots =====
fig, axes = plt.subplots(2, 3, figsize=(20, 11))

# 1. Down vs Up sweep scatter
ax = axes[0,0]
for ds in down_sweep:
    if not ds['saturated']:
        ax.scatter(ds['gyro'], ds['current'], c='#2196F3', s=20, alpha=0.6)
for us in up_sweep:
    if not us['saturated']:
        ax.scatter(us['gyro'], us['current'], c='#FF9800', s=20, alpha=0.6)
ax.scatter([], [], c='#2196F3', label='Down sweep')
ax.scatter([], [], c='#FF9800', label='Up sweep')
ax.set_xlabel('gyro (deg)'); ax.set_ylabel('current')
ax.set_title('Raw: Down vs Up Sweep'); ax.legend(); ax.grid(alpha=0.3)

# 2. Averaged points + fit
ax = axes[0,1]
ax.scatter(gyro_fit, curr_fit, c='#4CAF50', s=40, edgecolors='black', lw=0.8,
           zorder=5, label=f'Averaged (n={len(clean_pairs)})')
ax.plot(gyro_fine, fits[3]['curve'], '--', color='#2196F3', lw=2,
        label=f"3rd: RMSE={fits[3]['rmse']:.1f}")
ax.plot(gyro_fine, fits[4]['curve'], '-', color='#E91E63', lw=2,
        label=f"4th: RMSE={fits[4]['rmse']:.1f}")
ax.set_xlabel('gyro (deg)'); ax.set_ylabel('current')
ax.set_title(f'Fit Comparison (best={best}rd)'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

# 3. Residuals (best fit)
ax = axes[0,2]
r = curr_fit - bf['pred']
ax.axhline(0, color='black', lw=0.5)
ax.scatter(gyro_fit, r, c='#673AB7', s=30, edgecolors='black', lw=0.5)
ax.axhline(2*bf['rmse'], color='red', alpha=0.4, ls='--')
ax.axhline(-2*bf['rmse'], color='red', alpha=0.4, ls='--')
ax.set_xlabel('gyro (deg)'); ax.set_ylabel('residual')
ax.set_title(f'Residuals ({best}rd, RMSE={bf["rmse"]:.1f})'); ax.grid(alpha=0.3)

# 4. Friction vs angle
ax = axes[1,0]
fric_gyro = [p['gyro_avg'] for p in clean_pairs]
fric_val = [p['friction'] for p in clean_pairs]
ax.scatter(fric_gyro, fric_val, c='#FF5722', s=30, edgecolors='black', lw=0.5)
ax.axhline(med_fric, color='red', alpha=0.4, ls='--', label=f'Median={med_fric:.0f}')
ax.set_xlabel('gyro (deg)'); ax.set_ylabel('|I_up - I_down|/2')
ax.set_title(f'Friction Torque Estimate'); ax.legend(); ax.grid(alpha=0.3)

# 5. Pair connections (show hysteresis loops)
ax = axes[1,1]
for p in clean_pairs:
    ax.plot([p['gyro_down'], p['gyro_up']], [p['current_down'], p['current_up']],
            '-', color='gray', alpha=0.3, lw=0.5)
    ax.scatter(p['gyro_down'], p['current_down'], c='#2196F3', s=15)
    ax.scatter(p['gyro_up'], p['current_up'], c='#FF9800', s=15)
ax.set_xlabel('gyro (deg)'); ax.set_ylabel('current')
ax.set_title('Hysteresis Loops'); ax.grid(alpha=0.3)

# 6. Target error analysis
ax = axes[1,2]
for s in segments[1:]:
    c = 'red' if s['saturated'] else '#4CAF50'
    ax.scatter(s['gyro'], s['target']-s['gyro'], c=c, s=15, alpha=0.6)
ax.axhline(0, color='black', lw=0.5)
ax.set_xlabel('gyro (deg)'); ax.set_ylabel('target - gyro (deg)')
ax.set_title('Tracking Error'); ax.grid(alpha=0.3)

plt.suptitle(f'Gravity Compensation v2 — Hysteresis-Averaged Fit (best={best}rd-order)',
             fontsize=13, fontweight='bold')
plt.tight_layout(rect=[0,0,1,0.96])
plt.savefig('fit_v2_report.png', dpi=150, bbox_inches='tight')
plt.savefig('fit_v2_report.pdf', dpi=150, bbox_inches='tight')

# Simple plot
fig2, ax2 = plt.subplots(figsize=(12,7))
ax2.scatter(gyro_fit, curr_fit, c='#4CAF50', s=50, edgecolors='black', lw=0.8,
            zorder=5, label=f'Averaged data (n={len(clean_pairs)})')
ax2.plot(gyro_fine, fits[3]['curve'], '--', color='#2196F3', lw=2,
         label=f"3rd: RMSE={fits[3]['rmse']:.1f}, R²={fits[3]['r2']:.3f}")
ax2.plot(gyro_fine, fits[4]['curve'], '-', color='#E91E63', lw=2.5,
         label=f"4th: RMSE={fits[4]['rmse']:.1f}, R²={fits[4]['r2']:.3f}")
ax2.fill_between(gyro_fine, bf['curve']-2*bf['rmse'], bf['curve']+2*bf['rmse'],
                 alpha=0.12, color='#E91E63')
for g, c in zip(gyro_fit, curr_fit):
    ax2.annotate(f'{c:.0f}', (g,c), fontsize=7, alpha=0.6, textcoords="offset points", xytext=(0,6))
b_rmse2 = bf['rmse']; b_r22 = bf['r2']
coeff_str = '  '.join([f'{n}={v:.4f}' for n,v in zip(names_c, bf['opt'])])
ax2.set_xlabel('gyro_pitch_angle (deg)', fontsize=12)
ax2.set_ylabel('Compensation Current', fontsize=12)
ax2.set_title(f'Gravity Compensation v2 — {best}rd-order (Hysteresis-Averaged)\n{coeff_str}  |  RMSE={b_rmse2:.1f}  R²={b_r22:.4f}', fontsize=10)
ax2.legend(fontsize=10); ax2.grid(alpha=0.3)
plt.tight_layout(); plt.savefig('fit_v2_simple.png', dpi=150, bbox_inches='tight')

print("Plots saved.")
print("DONE")
