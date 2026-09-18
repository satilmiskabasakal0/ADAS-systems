"""Task 4: spatial speed envelope derived from v_next^2 = v^2 + 2*a*ds.

Backward propagation of all future limits is eq. 27 generalized beyond a fixed
lookahead. Forward propagation enforces acceleration. Infeasible initial states
are returned explicitly, never changed to a lower initial speed. Road geometry
is independent of speed: there is no lateral tracking claim or path RMSE here.
"""
import numpy as np

BOUNDARIES = np.array([0., 400., 1000., 1800., 2400., 3000.])
CURVATURE = np.array([0., .005, 0., .0025, 0.])
TRAFFIC = np.array([90., 50., 70., 50., 90.])/3.6


def segment_index(s):
    # Worksheet uses the old value AT a transition. Planning additionally checks
    # the one-sided upcoming limit so a point convention cannot postpone braking.
    return np.clip(np.searchsorted(BOUNDARIES[1:-1], s, side='left'), 0, 4)


def limits(s, ay):
    if ay < 0 or not np.isfinite(ay):
        raise ValueError("Lateral acceleration magnitude must be finite and nonnegative")
    i = segment_index(np.asarray(s))
    k = abs(CURVATURE[i])
    road = np.sqrt(np.divide(ay, k, out=np.full_like(k, np.inf, dtype=float), where=k>0))
    return road, TRAFFIC[i], np.minimum(road, TRAFFIC[i])


def geometry(s):
    """Exact integration of piecewise constant curvature, with continuous heading."""
    s = np.atleast_1d(s).astype(float)
    x = np.zeros_like(s); y = np.zeros_like(s)
    heading = 0.
    for start, end, k in zip(BOUNDARIES[:-1], BOUNDARIES[1:], CURVATURE):
        distance = np.clip(s-start, 0, end-start)
        if k == 0:
            x += distance*np.cos(heading); y += distance*np.sin(heading)
        else:
            x += (np.sin(heading+k*distance)-np.sin(heading))/k
            y += (np.cos(heading)-np.cos(heading+k*distance))/k
        heading += k*(end-start)
    return x, y


def plan(brake=2., ay=1.5, v0=25., accel=1., ds=1.):
    if not all(np.isfinite([brake, ay, v0, accel, ds])) or min(brake,ay,v0) < 0 or min(accel,ds) <= 0:
        raise ValueError("Use finite nonnegative brake/ay/v0 and positive accel/ds")
    s = np.unique(np.r_[np.arange(0, 3000, ds), BOUNDARIES])
    delta = np.diff(s)
    road, traffic, node_limit = limits(s, ay)
    interval_limit = limits((s[:-1]+s[1:])/2, ay)[2]
    # Each endpoint must respect both adjacent constant-limit intervals.
    caps = node_limit.copy()
    caps[:-1] = np.minimum(caps[:-1], interval_limit)
    caps[1:] = np.minimum(caps[1:], interval_limit)
    envelope = caps.copy()
    for i in range(len(s)-2,-1,-1):
        envelope[i] = min(envelope[i], np.sqrt(envelope[i+1]**2+2*brake*delta[i]))
    summary = dict(brake_mps2=brake, lateral_limit_mps2=ay, initial_speed_mps=v0,
                   admissible_initial_speed_mps=float(envelope[0]), ds_m=ds)
    if v0 > envelope[0]+1e-9:
        return dict(summary, status='infeasible_initial_speed', travel_time_s=None), None
    v = envelope.copy(); v[0] = v0
    for i, step in enumerate(delta):
        v[i+1] = min(v[i+1], np.sqrt(v[i]**2+2*accel*step))
    if np.any(v[:-1]+v[1:] < 1e-12):
        return dict(summary, status='blocked_zero_speed_interval', travel_time_s=None), None
    a = np.diff(v*v)/(2*delta)
    dt = 2*delta/(v[:-1]+v[1:])
    t = np.r_[0.,np.cumsum(dt)]
    # In an interval, v^2 is linear. Its extrema occur at the endpoints.
    excess = np.maximum(v[:-1],v[1:])-interval_limit
    k = abs(CURVATURE[segment_index((s[:-1]+s[1:])/2)])
    lateral_peak = np.max(np.maximum(v[:-1]**2,v[1:]**2)*k)
    summary.update(status='feasible', travel_time_s=float(t[-1]),
                   max_speed_excess_mps=float(max(0,np.max(excess))),
                   max_lateral_accel_mps2=float(lateral_peak),
                   min_accel_mps2=float(a.min()), max_accel_mps2=float(a.max()))
    x,y = geometry(s)
    return summary, dict(s_m=s,t_s=t,speed_mps=v,road_limit_mps=road,
                         traffic_limit_mps=traffic,combined_limit_mps=node_limit,
                         admissible_speed_mps=envelope,accel_mps2=np.r_[a,a[-1]],x_m=x,y_m=y)
