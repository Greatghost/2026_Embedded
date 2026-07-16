"""
Pitch Limit Oscillation Analysis
=================================
Analyzes angle & speed loop data during limit-bound oscillation.
"""
import csv, os, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

os.chdir(os.path.dirname(os.path.abspath(__file__)))

# ========== Load Data ==========
def load(csv_name):
    with open(csv_name, 'r', encoding='utf-8-sig') as f:
        reader = csv.reader(f, delimiter=';')
        header = next(reader)
        rows = np.array([[float(x) for x in r] for r in reader])
    t = rows[:,0] / 1e6  # us -> s
    return t, rows, header

t_ang, r_ang, h_ang = load('Pitch上下限位处震荡角度环数据.csv')
t_spd, r_spd, h_spd = load('Pitch上下限位处震荡速度环数据.csv')

print(f"Angle data: {len(r_ang)} samples, {t_ang[-1]-t_ang[0]:.1f}s, cols={h_ang}")
print(f"Speed data: {len(r_spd)} samples, {t_spd[-1]-t_spd[0]:.1f}s, cols={h_spd}")

# Extract columns
gyro       = r_ang[:,1]  # angle measure
ang_out    = r_ang[:,2]  # angle PID output (= speed_ref before FF)
ang_ref    = r_ang[:,3]  # angle Ref (= target including FF contribution)

spd_meas   = r_spd[:,1]  # speed measure (gyro_speed)
spd_out    = r_spd[:,2]  # speed PID output (= t_ff before FF+gravity)
spd_ref    = r_spd[:,3]  # speed Ref (= set_pitch_speed, incl FF)

# ========== Find oscillation zones ==========
# Oscillation = gyro oscillates around some value
# Detect where gyro crosses its local mean frequently
window = 100  # samples
crossings = []
for i in range(window, len(gyro)-window):
    local_mean = np.mean(gyro[i-window:i+window])
    # Detect sign change of (gyro - local_mean)
    if i > 0 and (gyro[i]-local_mean) * (gyro[i-1]-local_mean) < 0:
        crossings.append(t_ang[i])

print(f"\nGyro mean-crossings (oscillation indicator): {len(crossings)}")
if len(crossings) > 5:
    # Group crossings into oscillation bursts
    bursts = []
    burst_start = crossings[0]
    last_t = crossings[0]
    for c in crossings[1:]:
        if c - last_t > 0.5:  # gap > 500ms = new burst
            bursts.append((burst_start, last_t))
            burst_start = c
        last_t = c
    bursts.append((burst_start, last_t))
    print(f"Oscillation bursts detected: {len(bursts)}")
    for i, (s, e) in enumerate(bursts):
        dur = e - s
        # Count crossings per second in this burst
        n_cross = sum(1 for c in crossings if s <= c <= e)
        freq = n_cross / dur if dur > 0 else 0
        print(f"  Burst {i}: {s:.2f}s - {e:.2f}s, dur={dur:.2f}s, freq={freq:.1f}Hz, crossings={n_cross}")

# ========== Key metrics in oscillation ==========
# Average oscillation amplitude
if len(gyro) > 1000:
    # Simple detrend: subtract moving average
    window = min(100, len(gyro)//10)
    gyro_smooth = np.convolve(gyro, np.ones(window)/window, mode='same')
    gyro_detrend = gyro - gyro_smooth
    osc_amplitude = np.sqrt(np.mean(gyro_detrend**2))
    print(f"\nOscillation RMS amplitude: {osc_amplitude:.2f} deg")

# Speed oscillation
spd_rms = np.sqrt(np.mean(spd_meas**2))
print(f"Speed RMS: {spd_rms:.1f} deg/s")

# Angle output saturation
ang_sat_hi = np.sum(ang_out > 115) / len(ang_out) * 100
ang_sat_lo = np.sum(ang_out < -115) / len(ang_out) * 100
print(f"Angle PID saturation: +{ang_sat_hi:.1f}%  -{ang_sat_lo:.1f}%")

# Speed output saturation
spd_sat_hi = np.sum(np.abs(spd_out) > 15000) / len(spd_out) * 100
print(f"Speed PID saturation: {spd_sat_hi:.1f}%")

# ========== Diagnostic: limitPitchAngle clamping effect ==========
# The Ref (target+FF) should follow measure closely in normal tracking
# If Ref - Measure has large spikes with sign flips, it's limitPitchAngle clamping
ref_err = ang_ref - gyro
print(f"\nRef-Measure error: mean={np.mean(ref_err):.2f}, std={np.std(ref_err):.2f}")
print(f"  max={np.max(ref_err):.2f}, min={np.min(ref_err):.2f}")

# ========== Plots ==========
fig, axes = plt.subplots(3, 1, figsize=(16, 12), sharex=True)

# 1. Angle loop
ax = axes[0]
ax.plot(t_ang, ang_ref, '--', color='#FF9800', lw=1, alpha=0.7, label='Ref (target+FF)')
ax.plot(t_ang, gyro, '-', color='#2196F3', lw=1, label='Measure (gyro)')
ax.set_ylabel('Angle (deg)')
ax.set_title('Angle Loop — Ref vs Measure')
ax.legend(loc='upper right', fontsize=8)
ax.grid(alpha=0.3)

# 2. Angle PID output (speed reference)
ax = axes[1]
ax.plot(t_ang, ang_out, '-', color='#4CAF50', lw=0.8, label='Angle PID Out (speed ref)')
ax.axhline(120, color='red', alpha=0.4, ls='--', lw=0.8, label='MaxOut ±120')
ax.axhline(-120, color='red', alpha=0.4, ls='--', lw=0.8)
ax.set_ylabel('Output (deg/s)')
ax.set_title('Angle PID Output')
ax.legend(loc='upper right', fontsize=8)
ax.grid(alpha=0.3)

# 3. Speed loop
ax = axes[2]
ax.plot(t_spd, spd_meas, '-', color='#2196F3', lw=0.8, alpha=0.7, label='Speed Measure')
ax.plot(t_spd, spd_ref, '--', color='#FF9800', lw=0.8, alpha=0.7, label='Speed Ref')
ax.axhline(0, color='black', lw=0.5)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Speed (deg/s)')
ax.set_title('Speed Loop — Ref vs Measure')
ax.legend(loc='upper right', fontsize=8)
ax.grid(alpha=0.3)

plt.tight_layout()
plt.savefig('oscillation_overview.png', dpi=150)
print("\nPlot saved to oscillation_overview.png")

# ========== Zoom on worst oscillation ==========
if len(bursts) > 0:
    # Find the burst with highest frequency (most oscillation)
    best_burst = max(bursts, key=lambda b: b[1]-b[0])
    t0, t1 = best_burst
    t0 = max(0, t0 - 0.5)
    t1 = min(t_ang[-1], t1 + 0.5)

    fig2, axes2 = plt.subplots(4, 1, figsize=(16, 12), sharex=True)

    mask_ang = (t_ang >= t0) & (t_ang <= t1)
    mask_spd = (t_spd >= t0) & (t_spd <= t1)

    ax = axes2[0]
    ax.plot(t_ang[mask_ang], ang_ref[mask_ang], '--', color='#FF9800', lw=1.5, label='Ref')
    ax.plot(t_ang[mask_ang], gyro[mask_ang], '-', color='#2196F3', lw=1.5, label='Measure')
    ax.set_ylabel('Angle (deg)')
    ax.set_title(f'Zoom: Angle Loop [{t0:.1f}s - {t1:.1f}s]')
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes2[1]
    ax.plot(t_ang[mask_ang], ang_out[mask_ang], '-', color='#4CAF50', lw=1)
    ax.axhline(120, color='red', alpha=0.4, ls='--', lw=0.8)
    ax.axhline(-120, color='red', alpha=0.4, ls='--', lw=0.8)
    ax.set_ylabel('AngleOut')
    ax.set_title('Angle PID Output'); ax.grid(alpha=0.3)

    ax = axes2[2]
    ax.plot(t_spd[mask_spd], spd_ref[mask_spd], '--', color='#FF9800', lw=1.5, label='Spd Ref')
    ax.plot(t_spd[mask_spd], spd_meas[mask_spd], '-', color='#2196F3', lw=1, label='Spd Meas')
    ax.set_ylabel('Speed (deg/s)')
    ax.set_title('Speed Loop'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = axes2[3]
    ax.plot(t_spd[mask_spd], spd_out[mask_spd], '-', color='#E91E63', lw=1, label='Spd Out (t_ff)')
    ax.axhline(0, color='black', lw=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('t_ff')
    ax.set_title('Speed PID Output'); ax.legend(fontsize=8); ax.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig('oscillation_zoom.png', dpi=150)
    print(f"Zoom plot saved to oscillation_zoom.png (burst at {t0:.1f}s)")

# ========== Tuning Recommendations ==========
print("\n" + "="*60)
print("ANALYSIS & RECOMMENDATIONS")
print("="*60)

# Check if angle PID is saturating
if ang_sat_hi > 10 or ang_sat_lo > 10:
    print("[ISSUE] Angle PID heavily saturated near limits")
    print("  -> Consider increasing PITCH_ANGLE_MAXOUT or reducing speed FF")
    print("  -> Current MaxOut=120 deg/s is being hit hard")

# Check if speed oscillation frequency indicates instability
if len(crossings) > 0 and len(gyro) > 100:
    avg_freq = len(crossings) / (t_ang[-1] - t_ang[0])
    print(f"\nAverage oscillation frequency: {avg_freq:.1f} Hz")
    if avg_freq > 3:
        print("  High frequency oscillation (>3Hz) -> likely PID gain too high near limit")
        print("  -> Try adding PITCH_ANGLE_KD = 0.3~0.5 for damping")
    elif avg_freq > 1:
        print("  Medium frequency (1-3Hz) -> limit cycle, limitPitchAngle clamping may trigger")
        print("  -> Check if limitPitchAngle causes target sudden jump")

# Check FF contribution
spd_ref_max = np.max(np.abs(spd_ref))
print(f"\nMax speed reference: {spd_ref_max:.1f} deg/s")
if spd_ref_max > 100:
    print("  Speed Ref > 100 deg/s -> FF may be pushing too hard into limit")
    print("  -> Try reducing PITCH_SPEED_FF_VEL from current value")

# General recommendations
print("\n--- RECOMMENDED ACTIONS ---")
print("1. Add PITCH_ANGLE_KD = 0.3 (angle derivative → damping near limits)")
print("2. Reduce PITCH_SPEED_FF_VEL slightly (e.g. 7→5) to soften limit approach")
print("3. Widen soft limits temporarily (GIMBAL_ANGLE_MIN/MAX) to test if clamping is the root cause")
print("4. If oscillation only at one limit (not both), check mechanical asymmetry")

print("\nDONE")
