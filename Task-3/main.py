import numpy as np
import matplotlib.pyplot as plt


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


class PathTrackingSimulator:
    def __init__(self, lookahead_dist=15.0, k_y=0.1, k_psi=2.0):
        self.lookahead_dist = lookahead_dist
        self.k_y = k_y
        self.k_psi = k_psi

        # Vehicle parameters
        self.L = 2.8  # Vehicle Wheelbase
        self.tau = 0.5  # Time Constant
        self.vx = 20  # Vehicle Speed
        self.dt = 0.01  # Time Step
        self.T = 20  # Total Time
        self.t_vals = np.arange(0, self.T, self.dt)  # Time Vector
        self.N = len(self.t_vals)  # Number of Time Steps

        # Lane change parameters
        self.dy1 = 3.5
        self.dy2 = 3.5
        self.dx1 = 25
        self.dx2 = 25
        self.Xs1 = 60
        self.Xs2 = 130
        self.alpha = 2.4

        self._generate_reference_path()

    def _generate_reference_path(self):
        X_ref = self.vx * self.t_vals
        z1 = (self.alpha / self.dx1) * (X_ref - self.Xs1) - (self.alpha / 2)
        z2 = (self.alpha / self.dx2) * (X_ref - self.Xs2) - (self.alpha / 2)
        tanh_z1 = np.tanh(z1)
        tanh_z2 = np.tanh(z2)
        sech_sq_z1 = 1 - tanh_z1 ** 2
        sech_sq_z2 = 1 - tanh_z2 ** 2

        self.X_ref = X_ref
        self.Y_ref = (self.dy1 / 2) * (1 + tanh_z1) - (self.dy2 / 2) * (1 + tanh_z2)
        term1 = self.dy1 * sech_sq_z1 * (self.alpha / (2 * self.dx1))
        term2 = self.dy2 * sech_sq_z2 * (self.alpha / (2 * self.dx2))
        self.psi_ref = np.arctan(term1 - term2)
        dpsi_dt = np.gradient(self.psi_ref, self.dt)
        self.kappa_des = dpsi_dt / self.vx

    def run_simulation(self):
        N, dt, vx, tau = self.N, self.dt, self.vx, self.tau

        # Feedback controller states
        self.X_fb, self.Y_fb, self.psi_fb, r_fb = np.zeros(N), np.zeros(N), np.zeros(N), np.zeros(N)
        self.e_y_fb = np.zeros(N)

        # Pure pursuit controller states
        self.X_pp, self.Y_pp, self.psi_pp, r_pp = np.zeros(N), np.zeros(N), np.zeros(N), np.zeros(N)
        self.e_y_pp = np.zeros(N)

        for i in range(N - 1):
            # Feedback controller
            dist_sq_fb = (self.X_ref - self.X_fb[i]) ** 2 + (self.Y_ref - self.Y_fb[i]) ** 2
            ref_idx_fb = np.argmin(dist_sq_fb)
            ahead_idx_fb = ref_idx_fb
            remaining_dist_fb = 0

            while ahead_idx_fb < N - 1 and remaining_dist_fb < self.lookahead_dist:
                dx_seg = self.X_ref[ahead_idx_fb + 1] - self.X_ref[ahead_idx_fb]
                dy_seg = self.Y_ref[ahead_idx_fb + 1] - self.Y_ref[ahead_idx_fb]
                remaining_dist_fb += np.hypot(dx_seg, dy_seg)
                ahead_idx_fb += 1

            local_psi_fb = self.psi_fb[i]
            dx_fb = self.X_ref[ref_idx_fb] - self.X_fb[i]
            dy_fb = self.Y_ref[ref_idx_fb] - self.Y_fb[i]
            cos_psi_fb = np.cos(local_psi_fb)
            sin_psi_fb = np.sin(local_psi_fb)
            self.e_y_fb[i] = -sin_psi_fb * dx_fb + cos_psi_fb * dy_fb
            e_psi_fb = np.arctan2(np.sin(self.psi_ref[ahead_idx_fb] - local_psi_fb),
                                  np.cos(self.psi_ref[ahead_idx_fb] - local_psi_fb))
            kappa_cmd_fb = np.clip(self.k_y * self.e_y_fb[i] + self.k_psi * e_psi_fb, -0.12, 0.12)

            r_fb[i + 1] = r_fb[i] + dt * (-r_fb[i] / tau + vx * kappa_cmd_fb / tau)
            self.psi_fb[i + 1] = self.psi_fb[i] + dt * r_fb[i]
            self.X_fb[i + 1] = self.X_fb[i] + dt * vx * cos_psi_fb
            self.Y_fb[i + 1] = self.Y_fb[i] + dt * vx * sin_psi_fb

            # Pure pursuit
            '''
                Vehicle location: (local_x, local_y) = (10, 5)
                Vehicle direction:   = 45° = π/4 rad (right up)
                Target (reference) point: (X_ref, Y_ref) = (13, 9)

                | x_local |  = | cos(psi)    sin(psi) | * | dx |
                | y_local |  = | -sin(psi)   cos(psi) | * | dy |
            '''
            local_x_pp, local_y_pp, local_psi_pp = self.X_pp[i], self.Y_pp[i], self.psi_pp[i]
            dx_pp, dy_pp = self.X_ref - local_x_pp, self.Y_ref - local_y_pp
            cos_psi_pp, sin_psi_pp = np.cos(local_psi_pp), np.sin(local_psi_pp)
            x_local_pp = dx_pp * cos_psi_pp + dy_pp * sin_psi_pp
            y_local_pp = -dx_pp * sin_psi_pp + dy_pp * cos_psi_pp
            mask = x_local_pp > 0

            if not np.any(mask):
                continue

            L2_dist_pp = x_local_pp[mask] ** 2 + y_local_pp[mask] ** 2
            target_idx_pp = np.where(mask)[0][np.argmin(np.abs(np.sqrt(L2_dist_pp) - self.lookahead_dist))]

            alpha_pp = np.arctan2(y_local_pp[target_idx_pp], x_local_pp[target_idx_pp])
            raw_kappa_pp = 2 * np.sin(alpha_pp) / np.hypot(x_local_pp[target_idx_pp], y_local_pp[target_idx_pp])
            kappa_cmd_pp = np.clip(raw_kappa_pp, -0.12, 0.12)

            r_pp[i + 1] = r_pp[i] + dt * (-r_pp[i] / tau + vx * kappa_cmd_pp / tau)
            self.psi_pp[i + 1] = self.psi_pp[i] + dt * r_pp[i]
            self.X_pp[i + 1] = self.X_pp[i] + dt * vx * cos_psi_pp
            self.Y_pp[i + 1] = self.Y_pp[i] + dt * vx * sin_psi_pp
            self.e_y_pp[i] = y_local_pp[target_idx_pp]

    def plot_results(self):
        fig, axs = plt.subplots(3, 1, figsize=(12, 12))
        axs[0].plot(self.X_ref, self.Y_ref, 'k--', label='Reference')
        axs[0].plot(self.X_fb, self.Y_fb, 'b-', label='Feedback')
        axs[0].plot(self.X_pp, self.Y_pp, 'r-', label='Pure Pursuit')
        axs[0].set_title(f'Tracking Comparison\nLookahead={self.lookahead_dist}, k_y={self.k_y}, k_psi={self.k_psi}')
        axs[0].set_xlabel('X [m]')
        axs[0].set_ylabel('Y [m]')
        axs[0].legend()
        axs[0].grid(True)

        # Yaw angle plot including reference
        axs[1].plot(self.X_ref, np.degrees(self.psi_ref), 'k--', label='Yaw Reference')
        axs[1].plot(self.X_fb, np.degrees(self.psi_fb), 'b-', label='Feedback')
        axs[1].plot(self.X_pp, np.degrees(self.psi_pp), 'r-', label='Pure Pursuit')
        axs[1].set_title('Yaw Angle')
        axs[1].set_xlabel('X [m]')
        axs[1].set_ylabel('Yaw [deg]')
        axs[1].legend()
        axs[1].grid(True)

        # Lateral (cross-track) error = perpendicular distance from the vehicle
        # to the reference path. Comparing X/Y at the same time index would mix
        # longitudinal lag into the error; the true tracking error is the
        # shortest distance to the path, so we take the nearest reference point.
        def cross_track_error(X, Y):
            e = np.empty(len(X))
            for i in range(len(X)):
                e[i] = np.min(np.hypot(self.X_ref - X[i], self.Y_ref - Y[i]))
            return e

        e_lat_fb = cross_track_error(self.X_fb, self.Y_fb)
        e_lat_pp = cross_track_error(self.X_pp, self.Y_pp)
        rms_fb = np.sqrt(np.mean(e_lat_fb ** 2))
        rms_pp = np.sqrt(np.mean(e_lat_pp ** 2))

        axs[2].plot(self.X_fb, e_lat_fb, 'b-', label=f'Feedback (RMSE={rms_fb:.3f} m)')
        axs[2].plot(self.X_pp, e_lat_pp, 'r-', label=f'Pure Pursuit (RMSE={rms_pp:.3f} m)')
        axs[2].set_title('Lateral Error')
        axs[2].set_xlabel('X [m]')
        axs[2].set_ylabel('Lateral Error [m]')
        axs[2].legend()
        axs[2].grid(True)

        plt.tight_layout()
        plt.show()


sim1 = PathTrackingSimulator(lookahead_dist=20.0, k_y=0.01, k_psi=2.0)
sim2 = PathTrackingSimulator(lookahead_dist=20.0, k_y=1, k_psi=2.0)
sim3 = PathTrackingSimulator(lookahead_dist=20.0, k_y=1e-6, k_psi=2.0)
sim4 = PathTrackingSimulator(lookahead_dist=20.0, k_y=0.01, k_psi=5.0)
sim5 = PathTrackingSimulator(lookahead_dist=20.0, k_y=0.01, k_psi=0.5)
sim6 = PathTrackingSimulator(lookahead_dist=10,k_y=0.01, k_psi=2.0)
sim7 = PathTrackingSimulator(lookahead_dist=55.0, k_y=0.01, k_psi=2.0)

sim1.run_simulation() 
sim1.plot_results()
sim2.run_simulation()
sim2.plot_results()
sim3.run_simulation()
sim3.plot_results()
sim4.run_simulation()
sim4.plot_results()
sim5.run_simulation()
sim5.plot_results()
sim6.run_simulation()
sim6.plot_results()
sim7.run_simulation()
sim7.plot_results()
