import matplotlib.pyplot as plt
import numpy as np


def _use_compatible_backend():
    """Select an interactive Matplotlib backend that's actually available on
    this machine (macOS often ships without Tkinter), falling back to the
    non-interactive 'Agg' backend so the script still runs headless / in CI.
    Set the MPLBACKEND environment variable to force a specific backend."""
    import os, sys
    if os.environ.get("MPLBACKEND"):
        return  # respect an explicit user choice
    candidates = (["MacOSX"] if sys.platform == "darwin" else []) + ["QtAgg", "TkAgg", "Agg"]
    for backend in candidates:
        try:
            plt.switch_backend(backend)
            return
        except Exception:
            continue


_use_compatible_backend()
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

    # --- steering avoidance distances ---
    # Pure steering (d2): no braking -> full friction available laterally (μ·g)
    a_lat_pure   = mu * gravity
    s_steer_pure = initial_speed * np.sqrt(2 * lateral_distance / a_lat_pure)
    # Combined braking + steering (d3): remaining friction after braking
    if mu * gravity > abs(actual_accel):
        a_lat_comb   = np.sqrt((mu * gravity)**2 - actual_accel**2)
        s_steer_comb = initial_speed * np.sqrt(2 * lateral_distance / a_lat_comb)
        comb_label   = f"s_steer(comb)={s_steer_comb:.1f} m"
    else:
        s_steer_comb = np.nan
        comb_label   = "s_steer(comb) = not possible"

    # --- plot trajectory ---
    ax.plot(t, s, label=f"a={actual_accel:.2f} m/s²")
    # braking distance line
    ax.hlines(s_stop, 0, t_stop, colors='gray', linestyles='dotted')
    ax.text(t_stop + 0.2, s_stop, f"s_stop={s_stop:.1f} m", color='gray', va='bottom', fontsize=9)
    # pure-steering distance line
    ax.hlines(s_steer_pure, 0, t_stop, colors='green', linestyles='-.')
    ax.text(t_stop + 0.2, s_steer_pure, f"s_steer(pure)={s_steer_pure:.1f} m", color='green', va='bottom', fontsize=9)
    # combined-maneuver distance line (if valid)
    if not np.isnan(s_steer_comb):
        ax.hlines(s_steer_comb, 0, t_stop, colors='black', linestyles='--')
        ax.text(t_stop + 0.2, s_steer_comb, comb_label, color='black', va='bottom', fontsize=9)

    # --- styling each subplot ---
    ax.set_title(f"μ = {mu:.2f}")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (m)")
    ax.grid(True)
    ax.legend(title="Params", fontsize=8, loc='upper right')

plt.tight_layout()
plt.show()
