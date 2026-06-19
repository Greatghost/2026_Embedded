"""
Gravity Compensation Fitting for DM Pitch Motor
================================================
Input:  Ozone_DataGraph_260619.csv
Output: Polynomial coefficients for GimbalPitchComp()
Author: Claude Code + EDA
Date:   2026-06-19
"""
import csv
import math
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.stats import zscore

# Work in script's own directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))
print(f"Working dir: {os.getcwd()}")

# ============================================================
# 1. Load Data
# ============================================================
with open('Ozone_DataGraph_260619.csv', 'r') as f:
    reader = csv.reader(f, delimiter=';')
    header = next(reader)
    rows = [[float(x) for x in r] for r in reader]

time = np.array([r[0]/1e6 for r in rows])      # us -> s
gyro = np.array([r[1] for r in rows])
current = np.array([r[2] for r in rows])
target = np.array([r[3] for r in rows])

print(f"Data loaded: {len(rows)} samples, {time[-1]-time[0]:.0f}s")
print(f"Gyro range:  [{gyro.min():.2f}, {gyro.max():.2f}] deg")
print(f"Current range: [{current.min():.0f}, {current.max():.0f}]")
print(f"Target range:  [{target.min():.0f}, {target.max():.0f}] deg")

# ============================================================
# 2. Segment Extraction (EDA)
# ============================================================
changes = [0]
for i in range(1, len(target)):
    if abs(target[i] - target[i-1]) > 0.3:
        changes.append(i)
changes.append(len(target)-1)

segments = []
for i in range(len(changes)-1):
    s, e = changes[i], changes[i+1]
    if e - s < 50:
        continue
    dur = time[e] - time[s]
    if dur < 1.0:
        continue

    # Last 20% as steady state
    ss_s = s + (e - s) * 8 // 10
    gyro_ss = np.mean(gyro[ss_s:e])
    curr_ss = np.mean(current[ss_s:e])
    curr_std = np.std(current[ss_s:e])
    tgt_ss = np.mean(target[ss_s:e])
    tracking_err = tgt_ss - gyro_ss
    saturated = abs(curr_ss) > 580

    segments.append({
        'target': tgt_ss, 'gyro': gyro_ss, 'current': curr_ss,
        'std': curr_std, 'dur': dur, 'saturated': saturated,
        'tracking_err': tracking_err
    })

print(f"\nSegments extracted: {len(segments)}")

# ============================================================
# 3. Outlier Detection
# ============================================================
# 3a: Initial power-up point (gyro≈0, current≈0, target=0 at start)
powerup = [s for s in segments if abs(s['gyro']) < 0.01 and abs(s['current']) < 1]
print(f"\nPower-up initial: {len(powerup)} point(s)")

# 3b: Saturated
sat = [s for s in segments if s['saturated']]
print(f"Saturated (|I|>580): {len(sat)} point(s)")
for s in sat:
    print(f"  gyro={s['gyro']:.2f}, I={s['current']:.0f}, target={s['target']:.1f}")

# 3c: Large tracking error (>3 deg)
big_err = [s for s in segments if abs(s['tracking_err']) > 3.0 and not s['saturated']]
print(f"Large tracking error (>3 deg): {len(big_err)} point(s)")
for s in big_err:
    print(f"  gyro={s['gyro']:.2f}, I={s['current']:.0f}, err={s['tracking_err']:.2f}")

# 3d: Flag hysteresis points - near -19 deg with large current variance
# These are near the mechanical limit where the motor can't actually reach the target
low_end_anomaly = [s for s in segments
                   if s['gyro'] < -18.5 and abs(s['tracking_err']) > 1.5 and not s['saturated']]
print(f"Bottom-edge anomaly (gyro<-18.5, err>1.5): {len(low_end_anomaly)} point(s)")
for s in low_end_anomaly:
    print(f"  gyro={s['gyro']:.2f}, I={s['current']:.0f}, target={s['target']:.1f}, err={s['tracking_err']:.2f}")

# 3e: Isolation anomaly — single point at positive extreme with outlier current
high_end_isolated = [s for s in segments
                     if s['gyro'] > 10 and s['current'] < 100 and not s['saturated']]
print(f"High-end isolated anomaly (gyro>10, I<100): {len(high_end_isolated)} point(s)")
for s in high_end_isolated:
    print(f"  gyro={s['gyro']:.2f}, I={s['current']:.0f}, target={s['target']:.1f}")

# ============================================================
# 4. Build Clean Dataset
# ============================================================
excluded_pts = list(powerup) + list(sat) + list(low_end_anomaly) + list(high_end_isolated)
exclude_ids = set(id(s) for s in excluded_pts)

clean = [s for s in segments if id(s) not in exclude_ids]

gyro_clean = np.array([s['gyro'] for s in clean])
curr_clean = np.array([s['current'] for s in clean])

print(f"\nClean points for fitting: {len(clean)} (excluded {len(segments)-len(clean)})")



# ============================================================
# 5. Polynomial Fit — Compare 3rd vs 4th order
# ============================================================
def poly4(x, p1, p2, p3, p4, p5):
    return p1*x**4 + p2*x**3 + p3*x**2 + p4*x + p5

def poly3(x, p1, p2, p3, p4):
    return p1*x**3 + p2*x**2 + p3*x + p4

fits = {}
gyro_fine = np.linspace(gyro_clean.min()-2, gyro_clean.max()+2, 200)

# --- 4th order ---
try:
    p4_opt, p4_cov = curve_fit(poly4, gyro_clean, curr_clean,
                                p0=[0.0001, 0.01, -1.5, 50, 450], maxfev=10000)
    p4_err = np.sqrt(np.diag(p4_cov))
    p4_fit = poly4(gyro_fine, *p4_opt)
    p4_pred = poly4(gyro_clean, *p4_opt)
    p4_rmse = np.sqrt(np.mean((curr_clean - p4_pred)**2))
    p4_r2 = 1 - np.sum((curr_clean - p4_pred)**2) / np.sum((curr_clean - np.mean(curr_clean))**2)
except Exception as e:
    print(f"4th-order fit failed: {e}, falling back to polyfit")
    p4_opt = np.polyfit(gyro_clean, curr_clean, 4)  # highest power first
    p4_opt = p4_opt[::-1]  # reverse to [p1..p5] for poly4
    p4_err = [np.nan]*5
    p4_fit = poly4(gyro_fine, *p4_opt)
    p4_pred = poly4(gyro_clean, *p4_opt)
    p4_rmse = np.sqrt(np.mean((curr_clean - p4_pred)**2))
    p4_r2 = 1 - np.sum((curr_clean - p4_pred)**2) / np.sum((curr_clean - np.mean(curr_clean))**2)

fits['4th'] = {'opt': p4_opt, 'err': p4_err, 'rmse': p4_rmse, 'r2': p4_r2,
               'fit': p4_fit, 'pred': p4_pred,
               'names': ['p1 (x⁴)', 'p2 (x³)', 'p3 (x²)', 'p4 (x)', 'p5 (const)']}

# --- 3rd order ---
try:
    p3_opt, p3_cov = curve_fit(poly3, gyro_clean, curr_clean,
                                p0=[-0.5, -1.5, 20, 500], maxfev=10000)
    p3_err = np.sqrt(np.diag(p3_cov))
    p3_fit = poly3(gyro_fine, *p3_opt)
    p3_pred = poly3(gyro_clean, *p3_opt)
    p3_rmse = np.sqrt(np.mean((curr_clean - p3_pred)**2))
    p3_r2 = 1 - np.sum((curr_clean - p3_pred)**2) / np.sum((curr_clean - np.mean(curr_clean))**2)
except Exception as e:
    print(f"3rd-order fit failed: {e}, falling back to polyfit")
    p3_opt = np.polyfit(gyro_clean, curr_clean, 3)
    p3_opt = p3_opt[::-1]
    p3_err = [np.nan]*4
    p3_fit = poly3(gyro_fine, *p3_opt)
    p3_pred = poly3(gyro_clean, *p3_opt)
    p3_rmse = np.sqrt(np.mean((curr_clean - p3_pred)**2))
    p3_r2 = 1 - np.sum((curr_clean - p3_pred)**2) / np.sum((curr_clean - np.mean(curr_clean))**2)

fits['3rd'] = {'opt': p3_opt, 'err': p3_err, 'rmse': p3_rmse, 'r2': p3_r2,
               'fit': p3_fit, 'pred': p3_pred,
               'names': ['p1 (x³)', 'p2 (x²)', 'p3 (x)', 'p4 (const)']}

# --- Print comparison ---
print()
for label in ['4th', '3rd']:
    f = fits[label]
    print(f"\n=== {label}-order Polynomial Fit ===")
    print(f"  Equation: I = {' + '.join(f['names'])}")
    for i, (name, p, e) in enumerate(zip(f['names'], f['opt'], f['err'])):
        note = '  ⚠️ 不显著' if (not np.isnan(e) and abs(p) < 2*e) else ''
        print(f"    {name:>14s} = {p:12.6f}  ± {e:10.6f}{note}")
    print(f"    {'':>14s}   RMSE = {f['rmse']:.2f}")
    print(f"    {'':>14s}   R²   = {f['r2']:.6f}")

# --- Select best fit (lower RMSE wins, prefer 3rd if tie) ---
if fits['3rd']['rmse'] <= fits['4th']['rmse'] * 1.05:
    best = '3rd'
    print("\n>>> 3rd-order selected: simpler model, statistically equivalent fit")
else:
    best = '4th'
    print("\n>>> 4th-order selected: significant improvement over 3rd")

best_fit = fits[best]

# ============================================================
# 6. Generate C Code
# ============================================================
if best == '4th':
    c_code = f"""float GimbalPitchComp()
{{
    // 多项式系数 (Poly4拟合 - 2026/06/19, DM电机标定, RMSE={best_fit['rmse']:.1f}, R²={best_fit['r2']:.4f})
    const static float p1 = {best_fit['opt'][0]:.6f}f;
    const static float p2 = {best_fit['opt'][1]:.6f}f;
    const static float p3 = {best_fit['opt'][2]:.6f}f;
    const static float p4 = {best_fit['opt'][3]:.6f}f;
    const static float p5 = {best_fit['opt'][4]:.6f}f;

    float x = gimbal_controller.gyro_pitch_angle;

    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float comp_current = p1 * x4 + p2 * x3 + p3 * x2 + p4 * x + p5;

    iir(&gimbal_controller.comp_pitch_current, comp_current, 0.7f);
    return gimbal_controller.comp_pitch_current;
}}"""
else:
    c_code = f"""float GimbalPitchComp()
{{
    // 多项式系数 (Poly3拟合 - 2026/06/19, DM电机标定, RMSE={best_fit['rmse']:.1f}, R²={best_fit['r2']:.4f})
    const static float p1 = {best_fit['opt'][0]:.6f}f;
    const static float p2 = {best_fit['opt'][1]:.6f}f;
    const static float p3 = {best_fit['opt'][2]:.6f}f;
    const static float p4 = {best_fit['opt'][3]:.6f}f;

    float x = gimbal_controller.gyro_pitch_angle;

    float x2 = x * x;
    float x3 = x2 * x;
    float comp_current = p1 * x3 + p2 * x2 + p3 * x + p4;

    iir(&gimbal_controller.comp_pitch_current, comp_current, 0.7f);
    return gimbal_controller.comp_pitch_current;
}}"""

with open('gimbal_pitch_comp.c', 'w', encoding='utf-8') as f:
    f.write(c_code)
print("\nC code saved to gimbal_pitch_comp.c")

# ============================================================
# 7. Save Results Table
# ============================================================
with open('fit_results.txt', 'w', encoding='utf-8') as f:
    f.write("Gravity Compensation Fit Results\n")
    f.write("================================\n\n")
    f.write(f"Data file: Ozone_DataGraph_260619.csv\n")
    f.write(f"Total samples: {len(rows)}, Duration: {time[-1]-time[0]:.0f}s\n")
    f.write(f"Segments: {len(segments)}, Clean points: {len(clean)}, Excluded: {len(segments)-len(clean)}\n\n")
    f.write("Excluded points:\n")
    for s in excluded_pts:
        reason = []
        if s in powerup: reason.append('power-up initial')
        if s in sat: reason.append('saturated')
        if s in low_end_anomaly: reason.append('bottom-edge anomaly')
        if s in high_end_isolated: reason.append('high-end isolated')
        f.write(f"  gyro={s['gyro']:.3f}, I={s['current']:.1f}, target={s['target']:.1f}  [{', '.join(reason)}]\n")

    f.write(f"\n=== Comparison: 3rd vs 4th order ===\n")
    for label in ['3rd', '4th']:
        fk = fits[label]
        f.write(f"\n{label}-order: RMSE={fk['rmse']:.2f}, R²={fk['r2']:.6f}\n")
        for name, p, e in zip(fk['names'], fk['opt'], fk['err']):
            f.write(f"  {name:>14s} = {p:12.6f}  ± {e:10.6f}\n")

    f.write(f"\nSelected: {best}-order polynomial\n")
    f.write(f"RMSE = {best_fit['rmse']:.2f}\n")
    f.write(f"R²   = {best_fit['r2']:.6f}\n")

    f.write(f"\nClean data points used for fitting:\n")
    for s in sorted(clean, key=lambda s: s['gyro']):
        f.write(f"  gyro={s['gyro']:8.3f}  current={s['current']:8.1f}  target={s['target']:6.1f}  "
                f"err={s['tracking_err']:6.2f}  std={s['std']:6.1f}\n")

print("Summary saved to fit_results.txt")

# ============================================================
# 8. Plotting
# ============================================================
fig, axes = plt.subplots(2, 3, figsize=(20, 12))

# --- Plot 1: Raw Time Series ---
ax = axes[0, 0]
ax.plot(time, gyro, alpha=0.7, linewidth=0.5, label='gyro_pitch_angle', color='#2196F3')
ax.plot(time, target, alpha=0.7, linewidth=0.5, label='target', color='#FF9800', linestyle='--')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (deg)')
ax.set_title('Raw Time Series: Gyro & Target')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# --- Plot 2: Current Time Series ---
ax = axes[0, 1]
ax.plot(time, current, alpha=0.7, linewidth=0.5, label='set_pitch_current', color='#4CAF50')
ax.axhline(y=580, color='red', alpha=0.4, linestyle='--', linewidth=0.8, label='saturation ±580')
ax.axhline(y=-580, color='red', alpha=0.4, linestyle='--', linewidth=0.8)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Current (t_ff units)')
ax.set_title('Raw Time Series: set_pitch_current')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# --- Plot 3: Segments Overview ---
ax = axes[0, 2]
seg_gyros = [s['gyro'] for s in segments]
seg_currs = [s['current'] for s in segments]
colors = []
for s in segments:
    if s in powerup: colors.append('gray')
    elif s in sat: colors.append('red')
    elif s in low_end_anomaly: colors.append('orange')
    elif s in high_end_isolated: colors.append('purple')
    else: colors.append('#4CAF50')

ax.scatter(seg_gyros, seg_currs, c=colors, s=30, edgecolors='black', linewidth=0.5, zorder=5)
ax.axhline(y=580, color='red', alpha=0.3, linestyle='--', linewidth=0.8)
ax.axhline(y=-580, color='red', alpha=0.3, linestyle='--', linewidth=0.8)
ax.set_xlabel('gyro_pitch_angle (deg)')
ax.set_ylabel('set_pitch_current')
ax.set_title('Segment Steady-State Values')
ax.grid(True, alpha=0.3)

from matplotlib.patches import Patch
legend_elements = [
    Patch(facecolor='#4CAF50', label='Clean'),
    Patch(facecolor='red', label='Saturated'),
    Patch(facecolor='orange', label='Bottom anomaly'),
    Patch(facecolor='purple', label='High-end isolated'),
    Patch(facecolor='gray', label='Power-up init'),
]
ax.legend(handles=legend_elements, fontsize=7)

# --- Plot 4: Tracking Error ---
ax = axes[1, 0]
seg_errs = [s['tracking_err'] for s in segments]
ax.axhline(y=0, color='black', linewidth=0.5)
for i, s in enumerate(segments):
    c = 'red' if s in sat else ('orange' if s in low_end_anomaly else ('gray' if s in powerup else '#4CAF50'))
    ax.scatter(s['gyro'], s['tracking_err'], c=c, s=30, edgecolors='black', linewidth=0.5)
ax.set_xlabel('gyro_pitch_angle (deg)')
ax.set_ylabel('Tracking Error = target - gyro (deg)')
ax.set_title('Steady-State Tracking Error')
ax.grid(True, alpha=0.3)

# --- Plot 5: Fitting Result (both orders) ---
ax = axes[1, 1]
# Clean data
ax.scatter(gyro_clean, curr_clean, c='#4CAF50', s=40, edgecolors='black',
           linewidth=0.8, zorder=5, label=f'Clean data (n={len(clean)})')
# Excluded data
exc_gyros = [s['gyro'] for s in excluded_pts]
exc_currs = [s['current'] for s in excluded_pts]
for eg, ec, s in zip(exc_gyros, exc_currs, excluded_pts):
    c = 'red' if s in sat else ('orange' if s in low_end_anomaly else ('purple' if s in high_end_isolated else 'gray'))
    ax.scatter(eg, ec, c=c, s=60, marker='x', linewidth=1.5, zorder=6)
# Both fit curves
ax.plot(gyro_fine, fits['4th']['fit'], '-', color='#E91E63', linewidth=2, zorder=4,
        label=f"4th: RMSE={fits['4th']['rmse']:.1f}, R²={fits['4th']['r2']:.3f}")
ax.plot(gyro_fine, fits['3rd']['fit'], '--', color='#2196F3', linewidth=2, zorder=4,
        label=f"3rd: RMSE={fits['3rd']['rmse']:.1f}, R²={fits['3rd']['r2']:.3f}")
ax.set_xlabel('gyro_pitch_angle (deg)')
ax.set_ylabel('Compensation Current')
ax.set_title(f'Gravity Compensation Fit (best={best} order)')
ax.legend(fontsize=7)
ax.grid(True, alpha=0.3)

# --- Plot 6: Residuals (best fit) ---
ax = axes[1, 2]
rmse_best = best_fit['rmse']
r_best = curr_clean - best_fit['pred']
ax.axhline(y=0, color='black', linewidth=0.5)
r_sorted = r_best[np.argsort(gyro_clean)]
gyro_sorted = np.sort(gyro_clean)
ax.scatter(gyro_sorted, r_sorted, c='#673AB7', s=40, edgecolors='black', linewidth=0.5, zorder=5)
ax.axhline(y=2*rmse_best, color='red', alpha=0.4, linestyle='--', linewidth=0.8)
ax.axhline(y=-2*rmse_best, color='red', alpha=0.4, linestyle='--', linewidth=0.8)
ax.fill_between([gyro_sorted.min()-1, gyro_sorted.max()+1],
                -2*rmse_best, 2*rmse_best, alpha=0.1, color='red')
ax.set_xlabel('gyro_pitch_angle (deg)')
ax.set_ylabel(f'Residual ({best}-order fit)')
ax.set_title(f'Residuals ({best}-order, RMSE={rmse_best:.1f})')
ax.grid(True, alpha=0.3)
for g, r in zip(gyro_sorted, r_sorted):
    if abs(r) > 1.5 * rmse_best:
        ax.annotate(f'{r:.0f}', (g, r), fontsize=7, alpha=0.7,
                    textcoords="offset points", xytext=(0, 8))

plt.suptitle(f'Gravity Compensation Calibration — Pitch DM Motor (best={best}-order, 2026-06-19)',
             fontsize=13, fontweight='bold', y=0.98)
plt.tight_layout(rect=[0, 0, 1, 0.96])

# Save plots
for fmt in ['png', 'pdf']:
    plt.savefig(f'fitting_report.{fmt}', dpi=150, bbox_inches='tight')
print(f"Plots saved to fitting_report.png and fitting_report.pdf")

# Also save a simpler version
plt.close()

# Simple fit-only plot (best fit + both curves)
fig2, ax2 = plt.subplots(figsize=(12, 7))
ax2.scatter(gyro_clean, curr_clean, c='#4CAF50', s=50, edgecolors='black',
            linewidth=0.8, zorder=5, label=f'Measured data (n={len(clean)})')
ax2.plot(gyro_fine, fits['4th']['fit'], '-', color='#E91E63', linewidth=1.5, alpha=0.6, zorder=4,
         label=f"4th: RMSE={fits['4th']['rmse']:.1f}")
ax2.plot(gyro_fine, fits['3rd']['fit'], '--', color='#2196F3', linewidth=2.5, zorder=4,
         label=f"3rd: RMSE={fits['3rd']['rmse']:.1f}")
ax2.fill_between(gyro_fine, best_fit['fit'] - 2*best_fit['rmse'], best_fit['fit'] + 2*best_fit['rmse'],
                 alpha=0.15, color='#E91E63')
for g, c in zip(gyro_clean, curr_clean):
    ax2.annotate(f'{c:.0f}', (g, c), fontsize=7, alpha=0.6,
                 textcoords="offset points", xytext=(0, 6))
ax2.set_xlabel('gyro_pitch_angle (deg)', fontsize=12)
ax2.set_ylabel('Compensation Current (t_ff units)', fontsize=12)
coeff_str = '  '.join([f'{n}={v:.3f}' for n, v in zip(best_fit['names'], best_fit['opt'])])
b_rmse = best_fit['rmse']
b_r2 = best_fit['r2']
ax2.set_title(f'Gravity Compensation — {best}-order Fit\n{coeff_str}  |  RMSE={b_rmse:.1f}  R²={b_r2:.4f}',
              fontsize=10)
ax2.legend(fontsize=10)
ax2.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('fit_simple.png', dpi=150, bbox_inches='tight')
print("Simple plot saved to fit_simple.png")

print("\n=== DONE ===")
print(f"All outputs saved to Gravity_Compensation/")
