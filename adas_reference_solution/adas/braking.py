"""Task 1: distinguish distance travelled, remaining stopping distance and avoidance.

SI units throughout. Ax is a signed acceleration; b is a positive deceleration.
The worksheet's steering formula assumes constant longitudinal speed, zero
initial lateral velocity and constant lateral acceleration, not a complete lane change.
"""
import numpy as np


def scenario(mu, v0=25.0, demand=5.0, clearance=3.0, g=9.81):
    if min(mu, v0, demand, clearance, g) <= 0:
        raise ValueError("All Task-1 parameters must be positive magnitudes.")
    b = min(demand, mu * g)
    stop_time = v0 / b
    t = np.linspace(0, stop_time, 301)
    v = np.maximum(v0 - b * t, 0)
    travelled = v0 * t - 0.5 * b * t**2
    remaining = v**2 / (2 * b)
    lateral_capacity = np.sqrt(max((mu * g)**2 - b**2, 0))
    pure = v * np.sqrt(2 * clearance / (mu * g))
    # This is the worksheet's frozen-speed estimate using the remaining friction.
    combined_frozen = (v * np.sqrt(2 * clearance / lateral_capacity)
                       if lateral_capacity > 1e-9 else np.full_like(v, np.nan))
    lateral_time = (np.sqrt(2 * clearance / lateral_capacity)
                    if lateral_capacity > 1e-9 else None)
    # Optional extension under constant Cartesian accelerations, before stopping.
    combined_braking = (v0 * lateral_time - 0.5 * b * lateral_time**2
                        if lateral_time is not None and lateral_time <= stop_time else None)
    summary = dict(mu=mu, brake_mps2=b, stop_time_s=stop_time,
                   stop_distance_m=v0**2 / (2*b), lateral_capacity_mps2=lateral_capacity,
                   pure_steer_m=float(pure[0]),
                   combined_frozen_m=float(combined_frozen[0]) if lateral_time else None,
                   combined_braking_m=combined_braking,
                   combined_before_stop=combined_braking is not None)
    return summary, dict(t_s=t, speed_mps=v, travelled_m=travelled,
                         remaining_stop_m=remaining, pure_steer_m=pure,
                         combined_frozen_m=combined_frozen)
