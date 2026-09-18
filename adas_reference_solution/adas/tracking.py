"""Task 3: worksheet yaw-rate lag model and two curvature controllers.

This is not a validated tire/steering model. Curvature saturation is an explicitly
declared numerical actuator bound, not a friction guarantee. Errors are measured
against the nearest point on a polyline, not equal time indices.
"""
import numpy as np


def reference(x):
    x = np.asarray(x)
    z1, z2 = 2.4/25*(x-60)-1.2, 2.4/25*(x-130)-1.2
    t1, t2 = np.tanh(z1), np.tanh(z2)
    y = 1.75*(t1-t2)
    slope = 1.75*2.4/25*((1-t1*t1)-(1-t2*t2))
    return y, np.arctan(slope)


def project(point, path):
    d = np.diff(path, axis=0)
    f = np.clip(np.sum((point-path[:-1])*d, axis=1)/np.sum(d*d, axis=1), 0, 1)
    feet = path[:-1] + f[:, None]*d
    dist2 = np.sum((feet-point)**2, axis=1)
    i = int(np.argmin(dist2))
    heading = np.arctan2(d[i, 1], d[i, 0])
    # Desired-minus-actual error, projected onto the path's left normal.
    error = np.dot(feet[i]-point, [-np.sin(heading), np.cos(heading)])
    return i, feet[i], float(error), float(np.sqrt(dist2[i])), heading


def pursuit_target(point, path, i, foot, lookahead):
    if lookahead <= 0:
        raise ValueError("lookahead must be positive")
    ahead = np.vstack((foot, path[i+1:]))
    dist = np.linalg.norm(ahead-point, axis=1)
    crossings = np.where((dist[:-1] <= lookahead) & (dist[1:] >= lookahead))[0]
    if len(crossings):
        k = int(crossings[0]); start = ahead[k]; d = ahead[k+1]-start
        z = start-point
        A, B, C = d@d, 2*(z@d), z@z-lookahead**2
        for r in sorted(np.roots([A, B, C])):
            if np.isreal(r) and -1e-8 <= r.real <= 1+1e-8:
                return start+float(r.real)*d
    # Recovery if far off path, or finite-path endpoint. Never reset the state.
    k = int(np.argmin(abs(dist-lookahead)))
    return ahead[k]


def simulate(controller='feedback', ky=.01, kpsi=2., lookahead=20., dt=.01,
             duration=12., speed=20., tau=.5, spacing=.25):
    if controller not in ('feedback', 'pure_pursuit'):
        raise ValueError("Unknown controller")
    if min(dt, duration, speed, tau, spacing, lookahead) <= 0:
        raise ValueError("Simulation scales must be positive")
    xref = np.arange(0, 450+spacing/2, spacing)
    path = np.column_stack((xref, reference(xref)[0]))
    state = np.zeros(4)  # X, Y, psi, r
    records = []
    status = 'completed'
    def dynamics(z, curvature):
        return np.array([speed*np.cos(z[2]), speed*np.sin(z[2]), z[3],
                         (-z[3]+speed*curvature)/tau])
    times = np.linspace(0, duration, int(np.ceil(duration/dt))+1)
    for n, t in enumerate(times):
        i, foot, signed, distance, heading = project(state[:2], path)
        heading_error = np.arctan2(np.sin(heading-state[2]), np.cos(heading-state[2]))
        if controller == 'feedback':
            raw = ky*signed + kpsi*heading_error
        else:
            target = pursuit_target(state[:2], path, i, foot, lookahead)
            delta = target-state[:2]
            L2 = delta@delta
            raw = (2*(-np.sin(state[2])*delta[0]+np.cos(state[2])*delta[1])/L2
                   if L2 > 1e-12 else 0.)
        command = float(np.clip(raw, -.12, .12))
        records.append([t, *state, signed, distance, heading_error, command, speed*state[3]])
        if i == len(path)-2 and np.dot(state[:2]-path[-1], path[-1]-path[-2]) >= 0:
            status = 'path_end'; break
        if n == len(times)-1:
            break
        h = times[n+1]-t
        k1 = dynamics(state, command); k2 = dynamics(state+h*k1/2, command)
        k3 = dynamics(state+h*k2/2, command); k4 = dynamics(state+h*k3, command)
        state += h*(k1+2*k2+2*k3+k4)/6
    matrix = np.array(records)
    names = ['t_s','x_m','y_m','heading_rad','yaw_rate_radps','signed_error_m',
             'distance_error_m','heading_error_rad','curvature_command_pm','lateral_accel_mps2']
    data = dict(zip(names, matrix.T))
    t = data['t_s']; error = data['distance_error_m']
    summary = dict(controller=controller, ky=ky, kpsi=kpsi, lookahead_m=lookahead,
                   dt_s=dt, path_spacing_m=spacing, status=status,
                   rmse_m=float(np.sqrt(np.trapezoid(error**2,t)/(t[-1]-t[0]))),
                   max_error_m=float(np.max(error)),
                   max_abs_lateral_accel_mps2=float(np.max(abs(data['lateral_accel_mps2']))),
                   saturation_fraction=float(np.mean(abs(data['curvature_command_pm']) >= .12-1e-9)),
                   final_x_m=float(data['x_m'][-1]))
    return summary, data, path
