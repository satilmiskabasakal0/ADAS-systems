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
friction_coeffs  = [0.15, 0.5, 0.85]
demanded_accel   = -5.0       # Desired braking acceleration (m/s²)
lateral_distance = 3.0        # Required lateral avoidance distance (m)

plt.figure(figsize=(14, 8))

for mu in friction_coeffs:
    # Determine the maximum braking allowed by friction
    max_brake_accel = -mu * gravity
    # Apply the demanded accel if within friction limit, otherwise limit it
    actual_accel = demanded_accel if abs(demanded_accel) <= mu * gravity else max_brake_accel

    # Braking trajectory calculations
    t_stop = initial_speed / abs(actual_accel)
    t      = np.linspace(0, t_stop, 200)
    s      = initial_speed * t + 0.5 * actual_accel * t**2
    s_stop = 0.5 * initial_speed**2 / abs(actual_accel)

    # --- Pure-steering avoidance distance (d2): no braking, so the full
    #     friction circle is available for lateral acceleration (a_lat = μ·g) ---
    a_lat_pure = mu * gravity
    s_steer_pure = initial_speed * np.sqrt(2 * lateral_distance / a_lat_pure)

    # --- Combined braking + steering distance (d3): brake at actual_accel and
    #     use the remaining friction laterally (friction circle) ---
    if mu * gravity > abs(actual_accel):
        a_lat_comb   = np.sqrt((mu * gravity)**2 - actual_accel**2)
        s_steer_comb = initial_speed * np.sqrt(2 * lateral_distance / a_lat_comb)
        comb_label   = f"s_steer(combined)={s_steer_comb:.1f}m"
    else:
        s_steer_comb = np.nan
        comb_label   = "s_steer(combined)=Not possible"

    # Build a comprehensive legend label
    label = (f"μ={mu:.2f}, a={actual_accel:.2f} m/s², "
             f"s_stop={s_stop:.1f}m, s_steer(pure)={s_steer_pure:.1f}m, {comb_label}")

    # Plot the braking trajectory
    plt.plot(t, s, label=label)

    # Horizontal line for stopping distance
    plt.hlines(s_stop, 0, t_stop,
               colors='gray', linestyles='dotted', alpha=0.7)
    # Pure-steering avoidance distance (always feasible while μ > 0)
    plt.hlines(s_steer_pure, 0, t_stop,
               colors='green', linestyles='-.', alpha=0.7)
    # Combined-maneuver avoidance distance (if any friction remains for steering)
    if not np.isnan(s_steer_comb):
        plt.hlines(s_steer_comb, 0, t_stop,
                   colors='black', linestyles='--', alpha=0.7)

plt.title("Collision Avoidance: Braking vs. Steering", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Distance (m)", fontsize=12)
plt.grid(True)
plt.legend(fontsize=9, loc='upper right', framealpha=0.9)
plt.tight_layout()
plt.show()
