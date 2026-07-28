import csv
import statistics

rows = []
with open(r'd:\HUST\Study\Robot\LangYa\Electic_control\2026_Embedded-Sentry\Sentry_Steer\Chassis\Ozone_DataGraph_260722.csv', 'r') as f:
    reader = csv.reader(f, delimiter=';')
    header = next(reader)
    for r in reader:
        if len(r) == 3 and r[0] != '':
            try:
                t = int(r[0]); m = float(r[1]); ref = float(r[2])
                rows.append((t, m, ref))
            except: pass

start_idx = 0
for i, (t, m, ref) in enumerate(rows):
    if abs(ref) > 10:
        start_idx = i; break

active = rows[start_idx:]
times = [t for t, m, ref in active]
measures = [m for t, m, ref in active]
refs = [ref for t, m, ref in active]
errors = [ref - m for t, m, ref in active]

# Focus on Phase 0: first continuous movement (rows 0~246 of active)
# This is the first 2.87s of motion
print('=== Phase 0 Detailed (first 2.87s) ===')
phase0_end = 246
for i in range(0, min(phase0_end, len(active)), 10):
    print(f'  t={active[i][0]/1e6:.3f}s  meas={measures[i]:>8.1f}  ref={refs[i]:>8.1f}  err={errors[i]:>8.1f}')

# Compute peak-to-peak oscillation in phase 0
# Find first steady ref
print('\n=== Phase 0 Stats (excluding reversal at row ~40) ===')
# Only look at rows 0-35 where ref is stable around 720
stable_end = 35
m_vals = measures[:stable_end]
r_vals = refs[:stable_end]
print(f'Stable section (rows 0-34):')
print(f'  Ref: mean={statistics.mean(r_vals):.1f}, stdev={statistics.stdev(r_vals):.1f}')
print(f'  Meas: mean={statistics.mean(m_vals):.1f}, stdev={statistics.stdev(m_vals):.1f}')
print(f'  Peak-to-peak Meas: {max(m_vals)-min(m_vals):.1f}')

# Analyze settling after reversal
print('\n=== Settling after reversal (rows 45~150) ===')
# Measure overshoot & settling
settle_start = 45
settle_end = min(150, len(active))
m_settle = measures[settle_start:settle_end]
r_settle = refs[settle_start:settle_end]
# Find when error stays within 10% of ref
ref_target = -720
settled_idx = None
for i in range(settle_start, settle_end):
    if abs(errors[i]) < 72:  # 10% of 720
        if settled_idx is None:
            settled_idx = i
    else:
        settled_idx = None
if settled_idx:
    print(f'  Settled at row {settled_idx}, time={active[settled_idx][0]/1e6:.3f}s')
else:
    print(f'  Not settled within rows {settle_start}-{settle_end}')

# Peak after reversal
peak_idx = max(range(settle_start, settle_end), key=lambda i: abs(measures[i]))
print(f'  Peak: meas={measures[peak_idx]:.1f} at row {peak_idx} (t={active[peak_idx][0]/1e6:.3f}s)')
print(f'  Overshoot vs ref_target({ref_target}): {(abs(measures[peak_idx])-abs(ref_target))/abs(ref_target)*100:.1f}%')

# Check oscillation frequency in steady-state portions
# Find periods where ref is stable (>500ms at same value)
print('\n=== Oscillation in stable ref periods ===')
stable_periods = []
i = 0
while i < len(refs):
    if abs(refs[i]) > 100:
        # Start of potential stable period
        start = i
        while i < len(refs) and abs(refs[i] - refs[start]) < 50:
            i += 1
        if i - start > 40:  # at least ~0.5s
            stable_periods.append((start, i))
    else:
        i += 1

print(f'Found {len(stable_periods)} stable ref periods:')
for idx, (s, e) in enumerate(stable_periods):
    period_meas = measures[s:e]
    period_ref = refs[s:e]
    period_err = [r-m for r,m in zip(period_ref, period_meas)]
    # Count oscillations in measure
    osc = 0
    for j in range(1, len(period_meas)):
        if (period_meas[j-1] > period_meas[j] + 10 and j+1 < len(period_meas) and period_meas[j] < period_meas[j+1] - 10):
            osc += 1
    dur = (times[e-1] - times[s]) / 1e6
    p2p = max(period_meas) - min(period_meas)
    print(f'  Period {idx}: rows {s}~{e}, dur={dur:.3f}s, ref_mean={statistics.mean(period_ref):.1f}')
    print(f'    Meas: mean={statistics.mean(period_meas):.1f}, stdev={statistics.stdev(period_meas):.1f}, p2p={p2p:.1f}')
    print(f'    Err: mean={statistics.mean(period_err):.1f}, stdev={statistics.stdev(period_err):.1f}')
    print(f'    Oscillations: {osc}, freq~{osc/dur:.1f}Hz')

# Sampling rate analysis
print('\n=== Sampling Analysis ===')
diffs = [times[i+1]-times[i] for i in range(len(times)-1)]
print(f'Sampling: mean={statistics.mean(diffs):.0f}us, stdev={statistics.stdev(diffs):.0f}us')
print(f'Min={min(diffs)}us, Max={max(diffs)}us')
print(f'Median={statistics.median(diffs):.0f}us')
