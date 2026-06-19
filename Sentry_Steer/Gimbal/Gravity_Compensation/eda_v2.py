"""
EDA for Ozone_DataGraph_26061902.csv (v2 calibration)
"""
import csv, os, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

os.chdir(os.path.dirname(os.path.abspath(__file__)))
CSV = 'Ozone_DataGraph_26061902.csv'

with open(CSV, 'r') as f:
    reader = csv.reader(f, delimiter=';')
    header = next(reader)
    rows = [[float(x) for x in r] for r in reader]

time = np.array([r[0]/1e6 for r in rows])
gyro = np.array([r[1] for r in rows])
current = np.array([r[2] for r in rows])
target = np.array([r[3] for r in rows])

print(f"Data: {len(rows)} samples, {time[-1]-time[0]:.0f}s")
print(f"Gyro: [{gyro.min():.2f}, {gyro.max():.2f}]")
print(f"Current: [{current.min():.0f}, {current.max():.0f}]")
print(f"Target: [{target.min():.0f}, {target.max():.0f}]")

# Segment by target changes
changes = [0]
for i in range(1, len(target)):
    if abs(target[i] - target[i-1]) > 0.3:
        changes.append(i)
changes.append(len(target)-1)
print(f"\nSegments: {len(changes)-2}")

# Per-segment analysis
print(f"\n{'Seg':>4s} {'Target':>7s} {'Dur(s)':>7s} {'Gyro_ss':>9s} {'Curr_ss':>9s} {'Std':>7s} {'dErr':>7s} {'Note':>10s}")
print('-'*75)

segments = []
for i in range(len(changes)-1):
    s, e = changes[i], changes[i+1]
    if e - s < 50: continue
    dur = time[e] - time[s]
    if dur < 1.0: continue

    ss_s = s + (e - s) * 8 // 10
    gs = np.mean(gyro[ss_s:e])
    cs = np.mean(current[ss_s:e])
    cs_std = np.std(current[ss_s:e])
    ts = np.mean(target[ss_s:e])
    err = ts - gs
    saturated = abs(cs) > 580
    note = 'SAT' if saturated else ('noisy' if cs_std > 50 else '')

    print(f'{i:4d} {ts:7.1f} {dur:7.1f} {gs:9.3f} {cs:9.1f} {cs_std:7.1f} {err:7.2f} {note:>10s}')

    segments.append({
        'target': ts, 'gyro': gs, 'current': cs, 'std': cs_std,
        'dur': dur, 'saturated': saturated, 'err': err
    })

# Outlier categories
powerup = [s for s in segments if abs(s['gyro'])<0.01 and abs(s['current'])<1]
sat = [s for s in segments if s['saturated']]
print(f"\nPower-up: {len(powerup)}, Saturated: {len(sat)}")

# Hysteresis detection
valid = [s for s in segments if not s['saturated'] and s not in powerup]
valid_sorted = sorted(valid, key=lambda s: s['gyro'])
print(f"\nHysteresis check (valid: {len(valid)}):")
for i in range(len(valid_sorted)-1):
    for j in range(i+1, len(valid_sorted)):
        dg = abs(valid_sorted[i]['gyro'] - valid_sorted[j]['gyro'])
        dc = abs(valid_sorted[i]['current'] - valid_sorted[j]['current'])
        if dg < 1.5 and dc > 80:
            print(f"  dGyro={dg:.2f} dCurr={dc:.0f}: gyro={valid_sorted[i]['gyro']:.2f}(I={valid_sorted[i]['current']:.0f}) vs gyro={valid_sorted[j]['gyro']:.2f}(I={valid_sorted[j]['current']:.0f})")

# Coverage gaps
valid_gyros = sorted([s['gyro'] for s in valid])
print(f"\nCoverage gaps (>2 deg):")
for i in range(len(valid_gyros)-1):
    gap = valid_gyros[i+1] - valid_gyros[i]
    if gap > 2.0:
        print(f"  {valid_gyros[i]:.2f} -> {valid_gyros[i+1]:.2f}  gap={gap:.2f}")

# ---- PLOTS ----
fig, axes = plt.subplots(2, 2, figsize=(16, 10))

# 1. Raw time series
ax = axes[0,0]
ax.plot(time, gyro, alpha=0.7, lw=0.5, label='gyro')
ax.plot(time, target, alpha=0.7, lw=0.5, label='target', ls='--')
ax.legend(fontsize=8); ax.set_xlabel('Time(s)'); ax.set_ylabel('Angle(deg)')
ax.set_title('Time Series: Gyro & Target'); ax.grid(alpha=0.3)

# 2. Current time series
ax = axes[0,1]
ax.plot(time, current, alpha=0.7, lw=0.5, color='#4CAF50')
ax.axhline(580, color='red', alpha=0.4, ls='--', lw=0.8)
ax.axhline(-580, color='red', alpha=0.4, ls='--', lw=0.8)
ax.set_xlabel('Time(s)'); ax.set_ylabel('Current'); ax.set_title('set_pitch_current')
ax.grid(alpha=0.3)

# 3. Segments scatter
ax = axes[1,0]
colors = ['gray' if s in powerup else ('red' if s in sat else '#4CAF50') for s in segments]
ax.scatter([s['gyro'] for s in segments], [s['current'] for s in segments],
           c=colors, s=30, edgecolors='black', lw=0.5)
ax.axhline(580, color='red', alpha=0.3, ls='--')
ax.axhline(-580, color='red', alpha=0.3, ls='--')
ax.set_xlabel('gyro (deg)'); ax.set_ylabel('current')
ax.set_title('Steady-State Points'); ax.grid(alpha=0.3)

# 4. Tracking error
ax = axes[1,1]
ax.axhline(0, color='black', lw=0.5)
for s in segments:
    c = 'red' if s in sat else ('gray' if s in powerup else '#4CAF50')
    ax.scatter(s['gyro'], s['err'], c=c, s=30, edgecolors='black', lw=0.5)
ax.set_xlabel('gyro (deg)'); ax.set_ylabel('target - gyro (deg)')
ax.set_title('Tracking Error'); ax.grid(alpha=0.3)

plt.suptitle('EDA v2 — Pitch DM Motor Calibration Data', fontsize=13, fontweight='bold')
plt.tight_layout()
plt.savefig('eda_v2.png', dpi=150, bbox_inches='tight')
print("\nPlot saved to eda_v2.png")
print("DONE")
