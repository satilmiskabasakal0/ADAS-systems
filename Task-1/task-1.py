import matplotlib.pyplot as plt
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
# Constants
initial_speed    = 25.0       # Initial speed (m/s)
gravity          = 9.81       # Gravitational acceleration (m/s²)
friction_coeffs  = [0.15, 0.5,0.51,0.85,]
demanded_accel   = -5.0       # Desired braking acceleration (m/s²)
lateral_distance = 3.0        # Required lateral avoidance distance (m)

# Create a 2×2 grid of subplots
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
axes = axes.flatten()

for ax, mu in zip(axes, friction_coeffs):
    # --- compute actual braking accel ---
    max_brake_accel = -mu * gravity
    actual_accel    = demanded_accel if abs(demanded_accel) <= mu * gravity else max_brake_accel

    # --- braking trajectory ---
    t_stop = initial_speed / abs(actual_accel)
    t      = np.linspace(0, t_stop, 200)
    s      = initial_speed * t + 0.5 * actual_accel * t**2
    s_stop = 0.5 * initial_speed**2 / abs(actual_accel)

    # --- steering avoidance distance ---
    if mu * gravity > abs(actual_accel):
        max_lat_accel = np.sqrt((mu * gravity)**2 - actual_accel**2)
        s_steer      = initial_speed * np.sqrt(2 * lateral_distance / max_lat_accel)
        steer_label  = f"s_steer={s_steer:.1f} m"
    else:
        s_steer     = np.nan
        steer_label = "s_steer = not possible"

    # --- plot trajectory ---
    ax.plot(t, s, label=f"a={actual_accel:.2f} m/s²")
    # braking distance line
    ax.hlines(s_stop, 0, t_stop, colors='gray', linestyles='dotted')
    ax.text(t_stop + 0.2, s_stop, f"s_stop={s_stop:.1f} m", color='gray', va='bottom', fontsize=9)
    # steering distance line (if valid)
    if not np.isnan(s_steer):
        ax.hlines(s_steer, 0, t_stop, colors='black', linestyles='--')
        ax.text(t_stop + 0.2, s_steer, steer_label, color='black', va='bottom', fontsize=9)

    # --- styling each subplot ---
    ax.set_title(f"μ = {mu:.2f}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (m)")
    ax.grid(True)
    ax.legend(title="Params", fontsize=8, loc='upper right')

plt.tight_layout()
plt.show()
