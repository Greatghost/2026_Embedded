"""
Pitch Step Response Analysis
=============================
Analyzes square wave test data: rise time, settling time, overshoot
"""
import csv, os, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

os.chdir(os.path.dirname(os.path.abspath(__file__)))
CSV = '../Pitch轴响应速度调试数据1.csv'

with open(CSV, 'r') as f:
    reader = csv.reader(f, delimiter=';')
    header = next(reader)
    rows = np.array([[float(x) for x in r] for r in reader])

time = rows[:,0] / 1e6  # us -> s
measure = rows[:,1]      # gyro_pitch_angle
ref = rows[:,2]           # target_pitch_angle

print(f"Data: {len(rows)} samples, {time[-1]-time[0]:.1f}s")
print(f"Measure range: [{measure.min():.2f}, {measure.max():.2f}] deg")
print(f"Ref range: [{ref.min():.0f}, {ref.max():.0f}] deg")

# Find step edges (ref changes > 1 deg)
edges = []
for i in range(1, len(ref)):
    d = abs(ref[i] - ref[i-1])
    if d > 1.0:
        edges.append((i, time[i], ref[i-1], ref[i], d))

print(f"\nFound {len(edges)} step edges:")
for i, (idx, t, fr, to, d) in enumerate(edges):
    print(f"  [{i}] T={t:.3f}s  {fr:.0f}° -> {to:.0f}°  (delta={d:.0f}°)")

# Analyze each step response
print(f"\n=== Step Response Analysis ===")
print(f"{'Step':>5s} {'Dir':>5s} {'Delta':>7s} {'RiseT':>8s} {'SettleT':>8s} {'Overshoot':>10s} {'SteadyErr':>10s}")
print('-' * 65)

for step_i, (edge_idx, edge_t, ref_before, ref_after, step_size) in enumerate(edges):
    # Search window: from edge to next edge or end of data
    next_edge = len(time)-1
    for j in range(step_i+1, len(edges)):
        next_edge = edges[j][0]
        break
    if step_i == len(edges)-1:
        next_edge = len(time)-1

    # Slice data for this step
    start = edge_idx
    end = min(next_edge, edge_idx + 800)  # max ~1.6s window

    t_slice = time[start:end] - edge_t
    m_slice = measure[start:end]
    r_slice = ref[start:end]

    # Find steady state: mean of last 0.3s before next step
    ss_start = max(0, len(t_slice) - int(0.3 / (t_slice[1]-t_slice[0] if len(t_slice)>1 else 0.002)))
    if ss_start < len(t_slice):
        steady_val = np.mean(m_slice[ss_start:])
    else:
        steady_val = m_slice[-1]

    # Rise time: time from 10% to 90% of final value
    initial_val = m_slice[0]
    target_val = ref_after
    low_thresh = initial_val + 0.1 * (target_val - initial_val)
    high_thresh = initial_val + 0.9 * (target_val - initial_val)

    t_rise_start = None
    t_rise_end = None
    for j, m in enumerate(m_slice):
        if t_rise_start is None and abs(m - initial_val) > abs(low_thresh - initial_val):
            t_rise_start = t_slice[j]
        if t_rise_end is None and t_rise_start is not None and abs(m - initial_val) > abs(high_thresh - initial_val):
            t_rise_end = t_slice[j]
            break

    rise_time = (t_rise_end - t_rise_start) if (t_rise_start and t_rise_end) else float('nan')

    # Settling time: time to stay within 5% of final value
    settle_band = abs(target_val - initial_val) * 0.05
    t_settle = float('nan')
    settled_count = 0
    settle_needed = int(0.1 / (t_slice[1]-t_slice[0] if len(t_slice)>1 else 0.002))  # 100ms stable
    if settle_needed < 1: settle_needed = 1
    for j, m in enumerate(m_slice):
        if j > 0 and t_slice[j] > 0.01:  # skip first sample
            if abs(m - target_val) < settle_band:
                settled_count += 1
                if settled_count >= settle_needed:
                    t_settle = t_slice[j]
                    break
            else:
                settled_count = 0

    # Overshoot
    direction = 1 if target_val > initial_val else -1
    if direction > 0:
        overshoot = max(m_slice) - target_val
    else:
        overshoot = target_val - min(m_slice)
    overshoot_pct = overshoot / abs(target_val - initial_val) * 100 if abs(target_val - initial_val) > 0.1 else 0

    # Steady-state error
    steady_err = steady_val - target_val

    direction_str = 'UP' if direction > 0 else 'DN'
    print(f'{step_i:5d} {direction_str:>5s} {step_size:7.1f}° {rise_time if not math.isnan(rise_time) else 0:8.3f}s '
          f'{t_settle if not math.isnan(t_settle) else 0:8.3f}s {overshoot_pct:9.1f}% {steady_err:10.3f}°')

# ===== Plots =====
fig, axes = plt.subplots(2, 1, figsize=(14, 8))

# Full time series
ax = axes[0]
ax.plot(time, ref, '--', color='#FF9800', lw=1.5, label='target_pitch_angle (ref)', alpha=0.8)
ax.plot(time, measure, '-', color='#2196F3', lw=1, label='gyro_pitch_angle (measure)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle (deg)')
ax.set_title('Pitch Square Wave Response — Full Time Series')
ax.legend(); ax.grid(alpha=0.3)

# Zoom on first 2-3 steps
ax = axes[1]
zoom_end = min(20, time[-1])
mask = time <= zoom_end
ax.plot(time[mask], ref[mask], '--', color='#FF9800', lw=1.5, label='target')
ax.plot(time[mask], measure[mask], '-', color='#2196F3', lw=1.5, label='measure')

# Annotate edges
for i, (idx, t, fr, to, d) in enumerate(edges):
    if t <= zoom_end:
        ax.axvline(t, color='red', alpha=0.3, lw=0.5)
        ax.annotate(f'{fr:.0f}→{to:.0f}°', (t, to), fontsize=7)

ax.set_xlabel('Time (s)'); ax.set_ylabel('Angle (deg)')
ax.set_title(f'Zoom: First {zoom_end:.0f}s')
ax.legend(); ax.grid(alpha=0.3)

plt.tight_layout()
plt.savefig('pitch_step_response.png', dpi=150)
print("\nPlot saved to pitch_step_response.png")
print("DONE")
