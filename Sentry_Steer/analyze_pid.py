import csv
import statistics

rows = []
with open(r'd:\HUST\Study\Robot\LangYa\Electic_control\2026_Embedded-Sentry\Sentry_Steer\Chassis\Ozone_DataGraph_260722.csv', 'r') as f:
    reader = csv.reader(f, delimiter=';')
    header = next(reader)
    for r in reader:
        if len(r) == 3 and r[0] != '':
            try:
                t = int(r[0])
                m = float(r[1])
                ref = float(r[2])
                rows.append((t, m, ref))
            except:
                pass

print('=== Basic Info ===')
print(f'Total rows: {len(rows)}')
print(f'Time range: {rows[0][0]} ~ {rows[-1][0]} us = {rows[-1][0]/1e6:.3f} s')

# Find where Ref becomes non-zero
start_idx = 0
for i, (t, m, ref) in enumerate(rows):
    if abs(ref) > 10:
        start_idx = i
        break
print(f'Movement starts at row {start_idx}, time={rows[start_idx][0]} us = {rows[start_idx][0]/1e6:.3f} s')

active = rows[start_idx:]
print(f'Active rows: {len(active)}')

times = [t for t, m, ref in active]
measures = [m for t, m, ref in active]
refs = [ref for t, m, ref in active]

if len(times) > 1:
    diffs = [times[i+1]-times[i] for i in range(min(100, len(times)-1))]
    avg_dt = statistics.mean(diffs)
    print(f'Avg sampling interval: {avg_dt:.0f} us = {avg_dt/1000:.2f} ms')
    print(f'Sampling rate: {1e6/avg_dt:.0f} Hz')

print(f'\n=== Active Region Stats ===')
print(f'Measure: min={min(measures):.1f}, max={max(measures):.1f}, mean={statistics.mean(measures):.1f}')
print(f'Ref:     min={min(refs):.1f}, max={max(refs):.1f}, mean={statistics.mean(refs):.1f}')

errors = [ref - m for t, m, ref in active]
print(f'Error:   min={min(errors):.1f}, max={max(errors):.1f}, mean={statistics.mean(errors):.1f}, stdev={statistics.stdev(errors):.1f}')

# Steady-state (last 30%)
ss_start = int(len(active) * 0.7)
ss_measures = measures[ss_start:]
ss_refs = refs[ss_start:]
ss_errors = [r - m for r, m in zip(ss_refs, ss_measures)]
print(f'\n=== Steady-State (last 30%) ===')
print(f'Measure: min={min(ss_measures):.1f}, max={max(ss_measures):.1f}, mean={statistics.mean(ss_measures):.1f}, stdev={statistics.stdev(ss_measures):.1f}')
print(f'Ref:     min={min(ss_refs):.1f}, max={max(ss_refs):.1f}, mean={statistics.mean(ss_refs):.1f}')
print(f'Error:   min={min(ss_errors):.1f}, max={max(ss_errors):.1f}, mean={statistics.mean(ss_errors):.1f}, stdev={statistics.stdev(ss_errors):.1f}')

# Oscillation
osc_count = 0
for i in range(1, len(ss_errors)):
    if (ss_errors[i-1] > 0 and ss_errors[i] < 0) or (ss_errors[i-1] < 0 and ss_errors[i] > 0):
        osc_count += 1
print(f'Error zero crossings in steady-state: {osc_count}')
if len(ss_errors) > 1:
    ss_duration = (times[ss_start + len(ss_errors) - 1] - times[ss_start]) / 1e6
    print(f'Steady-state duration: {ss_duration:.3f} s')
    if ss_duration > 0:
        print(f'Oscillation frequency: {osc_count / 2 / ss_duration:.1f} Hz')

# Rising edge
print(f'\n=== Rising Edge Analysis (first 200 rows) ===')
edge_end = min(200, len(active))
edge_measures = measures[:edge_end]
edge_refs = refs[:edge_end]
ref_final = edge_refs[-1]
peak_idx = max(range(len(edge_measures)), key=lambda i: abs(edge_measures[i]))
print(f'Ref target: {ref_final:.1f}')
print(f'Peak measure: {edge_measures[peak_idx]:.1f} at row {peak_idx} (time={active[peak_idx][0]/1e6:.3f}s)')
if ref_final != 0:
    overshoot = (abs(edge_measures[peak_idx]) - abs(ref_final)) / abs(ref_final) * 100
    print(f'Overshoot: {overshoot:.1f}%')

# Rise time
if ref_final != 0:
    target_10 = 0.1 * abs(ref_final)
    target_90 = 0.9 * abs(ref_final)
    t10 = None
    t90 = None
    for i, m in enumerate(edge_measures):
        if t10 is None and abs(m) >= target_10:
            t10 = active[i][0]
        if t90 is None and abs(m) >= target_90:
            t90 = active[i][0]
            break
    if t10 and t90:
        print(f'Rise time (10%-90%): {(t90-t10)/1000:.1f} ms')

# Check for high-freq oscillation pattern
print(f'\n=== High-freq oscillation (consecutive sign changes) ===')
hf_count = 0
for i in range(1, len(measures)):
    if (measures[i-1] > measures[i]+5 and measures[i] < measures[i+1]-5 if i+1 < len(measures) else False):
        hf_count += 1
print(f'High-freq oscillation points: {hf_count}')

# Print sample data around peak
print(f'\n=== Sample around peak (rows {max(0,peak_idx-5)} ~ {min(len(active),peak_idx+10)}) ===')
for i in range(max(0,peak_idx-5), min(len(active),peak_idx+10)):
    print(f'  t={active[i][0]/1e6:.4f}s  meas={measures[i]:.1f}  ref={refs[i]:.1f}  err={errors[i]:.1f}')

# Count distinct movement phases
print(f'\n=== Movement Phases ===')
phases = []
in_phase = False
phase_start = 0
for i, ref in enumerate(refs):
    if abs(ref) > 10 and not in_phase:
        in_phase = True
        phase_start = i
    elif abs(ref) <= 10 and in_phase:
        in_phase = False
        phases.append((phase_start, i))
if in_phase:
    phases.append((phase_start, len(refs)))
print(f'Detected {len(phases)} movement phases:')
for idx, (s, e) in enumerate(phases):
    dur = (times[e-1] - times[s]) / 1e6
    print(f'  Phase {idx}: rows {s}~{e}, dur={dur:.3f}s, ref_mean={statistics.mean(refs[s:e]):.1f}')
