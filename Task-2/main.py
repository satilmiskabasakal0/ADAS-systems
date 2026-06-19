import os
import csv
import uuid
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import quad
from scipy.optimize import minimize
from matplotlib.gridspec import GridSpec


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

class TrajectoryOptimizer:
    '''
    Vehicle trajectory will be adjusted using a fifth-degree polynomial
    '''

    def __init__(self,ego_state,lead_state,T,D_0,tau):

        self.s0,self.v0,self.ac0=ego_state
        self.s_lv0,self.v_lv,self.ac_vl = lead_state
        self.T=T
        self.D_0=D_0
        self.tau=tau


        # Target Position
        self.s_lv_T = self.s_lv0+self.v_lv*self.T
        self.s_target = self.s_lv_T - (self.D_0+self.tau*self.v_lv)  # [7]


    def generate_trajectory(self,a,t):
        '''
        To compute the trajectory, use a 5th-degree equation:
        trajectory = f(x)                               '''
        return sum(a[i]*t**i for i in range(6))

    def generate_velocity(self,a,t):
        '''
        Compute the velocity of the vehicles
        f'(x)                      '''
        return a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4

    def generate_acceleration(self,a,t):
        ''' Compute the acceleration of the vehicles
            f''(X)                       '''
        return 2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3

    def generate_jerk(self,a,t):
        """
        Derivative of acceleration measures ride comfort (jerk)
        f'''(x)
        """
        return 6*a[3] + 24*a[4]*t + 60*a[5]*t**2

    def jerk_squared_integral(self,a,t0,t1):    # [9]
        '''
        Jerk karesi integralini hesapla
        '''
        jerk_func = lambda t:(6 * a[3] + 24 * a[4] * t + 60 * a[5] * t**2)**2
        return quad(jerk_func,t0,t1)[0]  # quad (result,error)


    def cost_function(self,a_free,k_j,k_t,k_s):
        '''

        trajectory = a0 +a1*t +a2*t^2 +..+ a5*t^5      t=0  f(x)=a0      a0  = s0
        velocity = a1+ 2a2*t + 3a3*t^2 +..+ 5a5*t^4    t=0  f(x)=a1      a1  = v0
        acceleration  2a2 + 6a3*t + ..+ 20a5*t^3       t=0  f(x)=2a2     2a2 = ac0    a2 = ac0/2
        jerk =   6a3 + 24a4*t+ 60a5*t^2                t=0  f(x)=6a3

       # Compute the integral of squared jerk
        k_j = Importance given to jerk
        k_t = Importance given to minimizing time
        k_s = Importance given to reaching the target position

        Compute the cost function C     [8]
        '''

        # # Constant coefficients and initial estimates

        a_full =[self.s0,self.v0,self.ac0/2]+list(a_free)  #  [6] Trajectory generation

        # Characteristic scales used to non-dimensionalise every cost term, so
        # that k_j / k_t / k_s are the ONLY weights that decide the trade-off.
        # (Previously the terms had mismatched units and were divided by the
        # arbitrary constants 1.0 / 2.0 / 1000.0, which hid extra weighting.)
        L_char = max(abs(self.s_target - self.s0), 1.0)   # characteristic length [m]
        v_char = max(self.v_lv, 1.0)                       # characteristic speed [m/s]
        jerk_char = L_char / self.T**3                     # characteristic jerk  [m/s^3]
        accel_char = L_char / self.T**2                    # characteristic accel [m/s^2]

        t_samples = np.linspace(0,self.T,20)

        # --- Jerk cost (comfort) ---
        jerk_cost = self.jerk_squared_integral(a_full,0,self.T)   # ∫ jerk² dt  [m²/s⁵]
        jerk_values = self.generate_jerk(a_full,t_samples)
        max_jerk_penalty = np.max(np.abs(jerk_values))**2         # peak jerk²  [m²/s⁶]

        # Non-dimensionalise both pieces before combining (∫jerk²dt by jerk_char²·T,
        # peak jerk² by jerk_char²) so the sum is unit-consistent.
        norm_jerk_integral = jerk_cost / (jerk_char**2 * self.T)
        norm_max_jerk = max_jerk_penalty / jerk_char**2
        norm_jerk_cost = norm_jerk_integral + 0.1 * norm_max_jerk

        # --- Time cost (speed tracking + smoothness) ---
        velocities = self.generate_velocity(a_full,t_samples)
        # The lead vehicle's speed is used as the target; it could potentially be
        # updated to maintain a safe following distance.
        desired_speed = self.v_lv
        speed_deviation = np.mean((velocities - desired_speed)**2)   # [m²/s²]
        accelerations = self.generate_acceleration(a_full,t_samples)
        accel_cost = np.mean(accelerations**2)                       # [m²/s⁴]

        # Each piece divided by its own characteristic scale -> dimensionless.
        norm_time_cost = speed_deviation / v_char**2 + 0.1 * accel_cost / accel_char**2

        # --- Position cost (reach target gap + stay safe) ---
        s_T = self.generate_trajectory(a_full,self.T)
        position_error = (s_T - self.s_target)**2                    # [m²]

        lead_pos = self.s_lv0 + self.v_lv*t_samples  # Moving with constant velocity
        ego_pos = self.generate_trajectory(a_full,t_samples)
        distances = lead_pos - ego_pos
        safe_distances = self.D_0+self.tau*self.generate_velocity(a_full,t_samples)
        safety_violations = np.maximum(0,safe_distances-distances)
        safety_cost = np.sum(safety_violations**2)                   # [m²]

        position_error_weight = 0.5
        safety_weight = 0.1
        # Both pieces are lengths², normalise by L_char² -> dimensionless.
        norm_position_cost = (position_error_weight * position_error
                              + safety_weight * safety_cost) / L_char**2

        # Total Weighted Cost

        total_cost = k_j*norm_jerk_cost+k_t*norm_time_cost+k_s*norm_position_cost  # Total cost function  [8]

        # Handling numerical issues
        if np.isnan(total_cost) or np.isinf(total_cost):
            print(f"Warning : Invalid cosr value detected in guessing. a_free =  {a_free}")
            total_cost = 1e4

        # Return total cost and individual component costs for analysis
        return total_cost, k_j * norm_jerk_cost, k_t * norm_time_cost, k_s * norm_position_cost


    def optimize_trajectory(self,k_j,k_t,k_s,initial_guess=None):
        '''
        Find optimal trajectory coefficient by minimizing the cost function
        '''
        if initial_guess is None:
            # Closed-form quintic boundary-value initial guess.
            # a0=s0, a1=v0, a2=ac0/2 are fixed; solve for a3,a4,a5 so that at
            # t=T the trajectory reaches the target position with the lead
            # vehicle's speed and zero acceleration. Earlier this was computed
            # with T multiplied (instead of divided) into a4/a5, which produced
            # huge, dimensionally-inconsistent starting coefficients.
            T = self.T
            s_T_target = self.s_target          # desired position at T
            v_T_target = self.v_lv              # match lead-vehicle speed
            a_T_target = 0.0                    # settle to zero acceleration

            # Contribution of the fixed (a0,a1,a2) terms evaluated at t=T.
            c_pos = self.s0 + self.v0 * T + (self.ac0 / 2) * T**2
            c_vel = self.v0 + self.ac0 * T
            c_acc = self.ac0

            M = np.array([
                [T**3,     T**4,      T**5],
                [3 * T**2, 4 * T**3,  5 * T**4],
                [6 * T,    12 * T**2, 20 * T**3],
            ])
            rhs = np.array([
                s_T_target - c_pos,
                v_T_target - c_vel,
                a_T_target - c_acc,
            ])
            initial_guess = list(np.linalg.solve(M, rhs))


        def objective(a_free):
            return self.cost_function(a_free,k_j,k_t,k_s)[0]

        # Trying multiple optimization methods and starting points if needed
        # Setup to track the best optimization result
        best_result = None
        best_cost = float('inf')

        # Trying different optimization methods
        '''
        Optimization Methods:
        - BFGS: A quasi-Newton method using gradient approximations for smooth problems.
        - Nelder-Mead: A derivative-free method based on simplex search. Good for noisy or non-smooth objectives.
        - Powell: A derivative-free method that performs directional line searches. Reliable for small-dimensional problems.
        - SLSQP (fallback): Supports constraints and bounds; used if others fail.
        '''
        methods = ["BFGS","Nelder-Mead","Powell"]
        os.makedirs("opt_logs",exist_ok=True)

        for method in methods:
            cost_history=[]

            def callback(xk):
                cost_history.append(xk.copy())

            try:
                # Add small random noise to initial guess to explore other local minimum points
                jittered_guess = [x+np.random.uniform(-0.1,0.1) for x in initial_guess]

                result = minimize(
                    lambda x: self.cost_function(x,k_j,k_t,k_s)[0],
                    jittered_guess,
                    method=method,
                    callback=callback,
                    options={"maxiter": 100}
                )

                with open(f"opt_logs/{method}_{k_j}_{k_t}_{k_s}_{uuid.uuid4().hex[:6]}.csv","w") as f:
                    writer = csv.writer(f)
                    writer.writerow(["iteration", "total_cost", "jerk_cost", "time_cost", "position_cost"])
                    for i, xk in enumerate(cost_history):
                        total, jerk, time, pos = self.cost_function(xk, k_j, k_t, k_s)
                        writer.writerow([i, total, jerk, time, pos])


                current_cost = objective(result.x)
                if current_cost < best_cost:
                    best_cost = current_cost
                    best_result = result
                    best_method = method

            except Exception as e:
                print(f"Optimization with {method} Failed: 1{e}")


        if best_result is None:
            def fallback_callback(xk):
                cost_history.append(xk.copy())

            cost_history=[]
            best_method = 'SLSQP'
            best_result = minimize(
                    lambda x: self.cost_function(x, k_j, k_t, k_s)[0],
                    initial_guess,
                    callback=fallback_callback,
                    method=best_method)
            with open(f"opt_logs/{best_method}_kJ{k_j}_kT{k_t}_kS{k_s}_{uuid.uuid4().hex[:6]}.csv", "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["iteration", "total_cost", "jerk_cost", "time_cost", "position_cost"])
                for i, a_free in enumerate(cost_history):
                    total, jerk, time, pos = self.cost_function(a_free, k_j, k_t, k_s)
                    writer.writerow([i, total, jerk, time, pos])


        a_full = [self.s0,self.v0,self.ac0/2]+ list(best_result.x)
        total_cost,jerk_cost,time_cost,position_cost = self.cost_function(best_result.x,k_j,k_t,k_s)
        print(f"Optimized with {k_j=}, {k_t=}, {k_s=}")
        print(f"Coefficients: a = {[round(a, 4) for a in a_full]}")
        print(f"Costs - Total: {total_cost:.4f}, Jerk: {jerk_cost:.4f}, Time: {time_cost:.4f}, Position: {position_cost:.4f}")

        return a_full,total_cost,jerk_cost,time_cost,position_cost


    def evaluate_trajectory(self,a_full,t_values,method_name = None):
        '''
        Evaluate trajectory characeristic for analysis and safety checks
        '''
        position = self.generate_trajectory(a_full, t_values)
        velocity = self.generate_velocity(a_full, t_values)
        acceleration = self.generate_acceleration(a_full, t_values)
        jerk = self.generate_jerk(a_full, t_values)

        s_T = position[-1]
        v_T = velocity[-1]
        ac_T = acceleration[-1]

        rms_jerk = np.sqrt(np.mean(jerk**2))
        lead_pos = self.s_lv0 + self.v_lv* t_values
        distances = lead_pos - position
        min_distance = np.min(distances)

        required_distance = self.D_0+ self.tau*velocity
        is_safe_vector = distances >= required_distance
        is_safe = np.all(is_safe_vector)

        return {
        'method': method_name,
        'position': position,
        'velocity': velocity,
        'acceleration': acceleration,
        'jerk': jerk,
        's_T': s_T,
        'v_T': v_T,
        'ac_T': ac_T,
        'target_error': s_T - self.s_target,
        'rms_jerk': rms_jerk,
        'min_distance': min_distance,
        'is_safe': is_safe,
        'is_safe_vector': is_safe_vector,
        'distances': distances,
        'required_distance': required_distance
        }


def plot_trajectory_comparison(results, t_values, label_key="param", title_prefix="", target_pos=None, lead_pos=None):
    '''
    Plot comparison of different trajectories
    results = [(parameter_value, a_opt, eval_data, jerk_cost, time_cost, position_cost), ...]
    '''
    plt.figure(figsize=(18, 14))
    gs = GridSpec(3, 2, figure=plt.gcf(), height_ratios=[1, 1, 0.8])

    # Use different colors for better distinction
    colors = ['blue', 'green', 'red', 'purple', 'orange', 'cyan', 'magenta', 'brown', 'pink']

    # Create labels with parameter values
    labels = [f"{label_key}={val:.1f}" for val, *_ in results]

    # Position plot
    ax1 = plt.subplot(gs[0, 0])

    # Plot lead vehicle trajectory
    if lead_pos is not None:
        ax1.plot(t_values, lead_pos, 'k--', linewidth=2, label="Lead Vehicle")

    # Plot target position line
    if target_pos is not None:
        ax1.axhline(y=target_pos, color="r", linestyle="--", linewidth=2, label="Target Position")

    # Plot each optimized trajectory with distinct line styles
    for i, ((label_val, _, eval_data, *_), lbl) in enumerate(zip(results, labels)):
        ax1.plot(t_values, eval_data['position'], color=colors[i % len(colors)],
                 linewidth=2, label=lbl)

    ax1.set_title(f"{title_prefix} - Position Profiles", fontsize=14)
    ax1.set_xlabel("Time (s)", fontsize=10)
    ax1.set_ylabel("Position (m)", fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=10)

    # Velocity plot
    ax2 = plt.subplot(gs[0, 1])

    # Plot lead vehicle velocity
    ax2.axhline(y=results[0][2]['velocity'][0], color='gray', linestyle=':', label="Initial Velocity")

    # Plot lead vehicle velocity as reference
    if lead_pos is not None:
        lead_vel = np.gradient(lead_pos, t_values)
        ax2.axhline(y=lead_vel[0], color='k', linestyle='--', label="Lead Vehicle Velocity")

    # Plot each optimized velocity profile
    for i, ((_, _, eval_data, *_), lbl) in enumerate(zip(results, labels)):
        ax2.plot(t_values, eval_data['velocity'], color=colors[i % len(colors)],
                 linewidth=2, label=lbl)

    ax2.set_title(f"{title_prefix} - Velocity Profiles", fontsize=10)
    ax2.set_xlabel("Time (s)", fontsize=10)
    ax2.set_ylabel("Velocity (m/s)", fontsize=10)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=10)

    # Acceleration plot
    ax3 = plt.subplot(gs[1, 0])

    # Add zero acceleration reference line
    ax3.axhline(y=0, color='gray', linestyle=':', label="Zero Acceleration")

    for i, ((_, _, eval_data, *_), lbl) in enumerate(zip(results, labels)):
        ax3.plot(t_values, eval_data['acceleration'], color=colors[i % len(colors)],
                 linewidth=2, label=lbl)

    ax3.set_title(f"{title_prefix} - Acceleration Profiles", fontsize=10)
    ax3.set_xlabel("Time (s)", fontsize=10)
    ax3.set_ylabel("Acceleration (m/s²)", fontsize=10)
    ax3.grid(True, alpha=0.3)
    ax3.legend(fontsize=10)

    # Jerk plot
    ax4 = plt.subplot(gs[1, 1])

    # Add zero jerk reference line
    ax4.axhline(y=0, color='gray', linestyle=':', label="Zero Jerk")

    for i, ((_, _, eval_data, *_), lbl) in enumerate(zip(results, labels)):
        ax4.plot(t_values, eval_data['jerk'], color=colors[i % len(colors)],
                 linewidth=2, label=lbl)

    ax4.set_title(f"{title_prefix} - Jerk Profiles", fontsize=10)
    ax4.set_xlabel("Time (s)", fontsize=10)
    ax4.set_ylabel("Jerk (m/s³)", fontsize=10)
    ax4.grid(True, alpha=0.3)
    ax4.legend(fontsize=10)

    # Cost components plot - use log scale for better comparison
    ax5 = plt.subplot(gs[2, 0])

    # Extract cost components
    x = np.arange(len(labels))
    width = 0.25

    jerk_costs = np.array([jc for *_, jc, _, _ in results])
    time_costs = np.array([tc for *_, _, tc, _ in results])
    pos_costs = np.array([pc for *_, _, _, pc in results])

    # Ensure positive values for log scale
    jerk_costs = np.maximum(jerk_costs, 1e-10)
    time_costs = np.maximum(time_costs, 1e-10)
    pos_costs = np.maximum(pos_costs, 1e-10)

    # Create stacked bars for better visualization
    ax5.bar(x, jerk_costs, width, label='Jerk Cost', color='skyblue')
    ax5.bar(x, time_costs, width, bottom=jerk_costs, label='Time Cost', color='lightgreen')
    ax5.bar(x, pos_costs, width, bottom=jerk_costs+time_costs, label='Position Cost', color='salmon')

    # Add total cost values as text
    for i, (j, t, p) in enumerate(zip(jerk_costs, time_costs, pos_costs)):
        total = j + t + p
        ax5.text(i, total + 0.1*total, f'{total:.1f}', ha='center', fontsize=9)

    ax5.set_title("Cost Component Comparison", fontsize=10)
    ax5.set_xticks(x)
    ax5.set_xticklabels(labels, fontsize=10)
    ax5.set_ylabel("Cost Value", fontsize=10)
    ax5.legend(fontsize=10)

    # Safety distance comparison
    ax6 = plt.subplot(gs[2, 1])

    # Plot the minimum required safety distance
    req_dist = results[0][2]['required_distance'][0]  # Just use the initial required distance as reference
    ax6.axhline(y=req_dist, color='red', linestyle='--', linewidth=2, label="Min Required Distance")

    # Plot the vehicle gap for each parameter value
    for i, ((_, _, eval_data, *_), lbl) in enumerate(zip(results, labels)):
        # Calculate vehicle gap (distance between vehicles)
        gap = eval_data['distances']

        # Plot gaps
        ax6.plot(t_values, gap, color=colors[i % len(colors)], linewidth=2,
                 label=f"Gap ({lbl})")

    ax6.set_title("Vehicle Gap Comparison", fontsize=10)
    ax6.set_xlabel("Time (s)", fontsize=10)
    ax6.set_ylabel("Distance (m)", fontsize=10)
    ax6.grid(True, alpha=0.3)
    ax6.legend(fontsize=10)

    plt.tight_layout()
    plt.show()


def run_parameter_sweep():
    '''
    Run parameter sweep to analyze the effect of weight parameters
    '''
    # Vehicle initial states [position, velocity, acceleration]
    ego_car = [0, 25, 0,]      # Ego vehicle: starting at 0m with 25m/s (90km/h)  [5]
    lead_car = [50, 20, 0]    # Lead vehicle: 50m ahead at 20m/s (72km/h)  [5]

    # Planning parameters
    T = 8                     # Time horizon of 8 seconds
    D_0 = 10                  # Minimum standstill distance
    tau = 1.5                 # Time headway for safety

    optimizer = TrajectoryOptimizer(ego_car, lead_car, T, D_0, tau)
    t_values = np.linspace(0, T, 100)

    # Lead vehicle position at each time step
    lead_pos = optimizer.s_lv0 + optimizer.v_lv * t_values

    # Use wider parameter ranges to see more distinctive behaviors
    k_j_values = [0.1, 1.0, 10.0, 50.0, 100.0]  # Wider range for comfort weight
    k_t_values = [0.1, 1.0, 10.0, 50.0, 100.0]  # Wider range for time efficiency
    k_s_values = [0.1, 1.0, 10.0, 50.0, 100.0]  # Wider range for position accuracy

    # 1. Analyze the effect of k_j (jerk weight)
    print("\n== Effect of k_j Parameter (k_t=1.0, k_s=1.0) ==")
    k_j_results = []
    for k_j in k_j_values:
        k_t = 1.0
        k_s = 1.0

        #print(f"\nOptimization: k_j={k_j:.2f}, k_t={k_t:.2f}, k_s={k_s:.2f}")
        a_opt, total_cost, jerk_cost, time_cost, position_cost = optimizer.optimize_trajectory(k_j, k_t, k_s)
        eval_data = optimizer.evaluate_trajectory(a_opt, t_values)

        k_j_results.append((k_j, a_opt, eval_data, jerk_cost, time_cost, position_cost))

    plot_trajectory_comparison(
        k_j_results, t_values,
        label_key="k_j",
        title_prefix="Effect of k_j (Comfort Weight)",
        target_pos=optimizer.s_target,
        lead_pos=lead_pos
    )

    # 2. Analyze the effect of k_t (time weight)
    print("\n== Effect of k_t Parameter (k_j=1.0, k_s=1.0) ==")
    k_t_results = []
    for k_t in k_t_values:
        k_j = 1.0
        k_s = 1.0

        #print(f"\nOptimization: k_j={k_j:.2f}, k_t={k_t:.2f}, k_s={k_s:.2f}")
        a_opt, total_cost, jerk_cost, time_cost, position_cost = optimizer.optimize_trajectory(k_j, k_t, k_s)
        eval_data = optimizer.evaluate_trajectory(a_opt, t_values)

        k_t_results.append((k_t, a_opt, eval_data, jerk_cost, time_cost, position_cost))

    plot_trajectory_comparison(
        k_t_results, t_values,
        label_key="k_t",
        title_prefix="Effect of k_t (Time Weight)",
        target_pos=optimizer.s_target,
        lead_pos=lead_pos
    )

    # 3. Analyze the effect of k_s (position weight)
    print("\n== Effect of k_s Parameter (k_j=1.0, k_t=1.0) ==")
    k_s_results = []
    for k_s in k_s_values:
        k_j = 1.0
        k_t = 1.0

        #print(f"\nOptimization: k_j={k_j:.2f}, k_t={k_t:.2f}, k_s={k_s:.2f}")
        a_opt, total_cost, jerk_cost, time_cost, position_cost = optimizer.optimize_trajectory(k_j, k_t, k_s)
        eval_data = optimizer.evaluate_trajectory(a_opt, t_values)

        k_s_results.append((k_s, a_opt, eval_data, jerk_cost, time_cost, position_cost))

    plot_trajectory_comparison(
        k_s_results, t_values,
        label_key="k_s",
        title_prefix="Effect of k_s (Position Weight)",
        target_pos=optimizer.s_target,
        lead_pos=lead_pos
    )


if __name__ == "__main__":
    run_parameter_sweep()



