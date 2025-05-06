import matplotlib.pyplot as plt
import numpy as np
import matplotlib
matplotlib.use('TkAgg')

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

    # Compute pure-steering avoidance distance if possible
    if mu * gravity > abs(actual_accel):
        max_lat_accel = np.sqrt((mu * gravity)**2 - actual_accel**2)
        s_steer      = initial_speed * np.sqrt(2 * lateral_distance / max_lat_accel)
        steer_label  = f"s_steer={s_steer:.1f}m"
    else:
        s_steer      = np.nan
        steer_label  = "s_steer=Not possible"

    # Build a comprehensive legend label
    label = (f"μ={mu:.2f}, a={actual_accel:.2f} m/s², "
             f"s_stop={s_stop:.1f}m, {steer_label}")

    # Plot the braking trajectory
    plt.plot(t, s, label=label)

    # Horizontal line for stopping distance
    plt.hlines(s_stop, 0, t_stop,
               colors='gray', linestyles='dotted', alpha=0.7)
    # Horizontal line for steering distance (if valid)
    if not np.isnan(s_steer):
        plt.hlines(s_steer, 0, t_stop,
                   colors='black', linestyles='--', alpha=0.7)

plt.title("Collision Avoidance: Braking vs. Steering", fontsize=14)
plt.xlabel("Time (s)", fontsize=12)
plt.ylabel("Distance (m)", fontsize=12)
plt.grid(True)
plt.legend(fontsize=9, loc='upper right', framealpha=0.9)
plt.tight_layout()
plt.show()
