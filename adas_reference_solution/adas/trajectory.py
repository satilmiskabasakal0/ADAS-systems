"""Task 2: boundary-conditioned quintic candidates, exact jerk and extrema.

The optimization variables are the FINITE grid of (T, delta_s), as in eq. 6.
Weights apply to the original dimensional objective, not a speed-tracking proxy.
An explicitly separate filtered selection enforces user-chosen physical limits.
"""
from dataclasses import dataclass
import numpy as np
from numpy.polynomial import Polynomial as P


@dataclass(frozen=True)
class Config:
    s0: float = 0.0
    v0: float = 25.0
    a0: float = 0.0
    lead_s0: float = 50.0
    lead_v: float = 20.0
    d0: float = 10.0
    headway: float = 1.5
    min_accel: float = -5.0
    max_accel: float = 2.0
    max_jerk: float = 5.0


def quintic(T, initial, final):
    """Return q(u)=s(T*u), u in [0,1], satisfying six endpoint conditions."""
    if not np.isfinite(T) or T <= 0:
        raise ValueError("T must be finite and positive.")
    s0, v0, a0 = initial
    s1, v1, a1 = final
    fixed = np.array([s0, T*v0, T*T*a0/2])
    matrix = np.array([[1, 1, 1], [3, 4, 5], [6, 12, 20]], dtype=float)
    rhs = [s1 - sum(fixed), T*v1-fixed[1]-2*fixed[2], T*T*a1-2*fixed[2]]
    return P(np.r_[fixed, np.linalg.solve(matrix, rhs)])


def extrema(poly):
    """Continuous min/max on [0,1], including every real stationary point."""
    roots = poly.deriv().roots()
    points = [0., 1.] + [float(r.real) for r in roots
                         if abs(r.imag) < 1e-8 and 0 < r.real < 1]
    values = poly(np.array(points))
    return float(np.min(values)), float(np.max(values))


def candidate(T, delta, cfg=Config()):
    target = cfg.lead_s0 + cfg.lead_v*T - (cfg.d0 + cfg.headway*cfg.lead_v)
    q = quintic(T, (cfg.s0, cfg.v0, cfg.a0), (target+delta, cfg.lead_v, 0))
    velocity, acceleration, jerk = [q.deriv(n)/T**n for n in (1, 2, 3)]
    gap = P([cfg.lead_s0, cfg.lead_v*T]) - q
    margin = gap - cfg.d0 - cfg.headway*velocity
    integral = (jerk*jerk).integ()
    J = float(T*(integral(1)-integral(0)))
    vmin, vmax = extrema(velocity)
    amin, amax = extrema(acceleration)
    jmin, jmax = extrema(jerk)
    gap_min, _ = extrema(gap)
    margin_min, _ = extrema(margin)
    feasible = (vmin >= -1e-8 and amin >= cfg.min_accel-1e-8
                and amax <= cfg.max_accel+1e-8 and max(abs(jmin), abs(jmax)) <= cfg.max_jerk+1e-8
                and margin_min >= -1e-8)
    return dict(T_s=float(T), delta_m=float(delta), jerk_integral=J,
                rms_jerk_mps3=float(np.sqrt(max(J/T, 0))),
                target_error_m=float(q(1)-target), min_gap_m=gap_min,
                min_margin_m=margin_min, min_speed_mps=vmin, max_speed_mps=vmax,
                min_accel_mps2=amin, max_accel_mps2=amax,
                max_abs_jerk_mps3=max(abs(jmin), abs(jmax)), feasible=bool(feasible),
                coefficients=q.coef.tolist())


def candidates(cfg=Config(), time_step=0.25, offset_step=1.0):
    # T in [2,12] s and terminal offsets in [-20,10] m are declared design choices.
    return [candidate(T, delta, cfg)
            for T in np.arange(2, 12+1e-9, time_step)
            for delta in np.arange(-20, 10+1e-9, offset_step)]


def select(pool, weights=(1., 1., 1.), filtered=False):
    kj, kt, ks = weights
    if min(weights) < 0 or not all(np.isfinite(weights)):
        raise ValueError("Weights must be finite and nonnegative.")
    valid = [c for c in pool if not filtered or c['feasible']]
    if not valid:
        return None  # explicit infeasibility: never silently return an invalid plan
    def cost(c):
        return kj*c['jerk_integral'] + kt*c['T_s'] + ks*c['delta_m']**2
    best = min(valid, key=lambda c: (cost(c), c['T_s'], c['delta_m']))
    return dict(best, cost=float(cost(best)), weights=list(weights), filtered=filtered)


def sample(c, cfg=Config(), n=501):
    T = c['T_s']
    t = np.linspace(0, T, n)
    q = P(c['coefficients'])
    s, v, a, j = [q.deriv(i)(t/T)/T**i for i in range(4)]
    lead = cfg.lead_s0 + cfg.lead_v*t
    return dict(t_s=t, position_m=s, speed_mps=v, accel_mps2=a, jerk_mps3=j,
                lead_position_m=lead, gap_m=lead-s, required_gap_m=cfg.d0+cfg.headway*v,
                margin_m=lead-s-cfg.d0-cfg.headway*v)
