import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from sklearn.metrics import mean_squared_error
import matplotlib
matplotlib.use('TkAgg')
class SmartSpeedAssistant:
    def __init__(self, a_x_comfort=-2.0, a_y_comfort=1.5, v_init=25.0, road_length=3000, segment_length=50):
        """
        Initialize the Smart Speed Assistant with given parameters.
        
        Args:
            a_x_comfort: Comfortable longitudinal deceleration (m/s²)
            a_y_comfort: Comfortable lateral acceleration (m/s²)
            v_init: Initial vehicle speed (m/s)
            road_length: Total road length (m)
            segment_length: Length of each road segment (m)
        """
        self.a_x_comfort = a_x_comfort
        self.a_y_comfort = a_y_comfort
        self.v_init = v_init
        self.road_length = road_length
        self.segment_length = segment_length
        
        # Store results
        self.time = None
        self.position = None
        self.velocity = None
        self.acceleration = None
        self.curvature = None
        self.v_lim_road = None
        self.v_lim_traffic = None
        self.v_lim_combined = None
        self.road_x = None
        self.road_y = None
    
    def road_curvature(self, s):
        """
        Calculate road curvature κ based on position s.
        
        Args:
            s: Position along the road (m)
            
        Returns:
            κ: Road curvature at position s (1/m)
        """
        if s <= 400:
            return 0
        elif s <= 1000:
            return 0.005
        elif s <= 1800:
            return 0
        elif s <= 2400:
            return 0.0025
        elif s <= 3000:
            return 0
        else:
            return 0
    
    def speed_limit_road(self, s):
        """
        Calculate speed limit due to road curvature.
        
        Args:
            s: Position along the road (m)
            
        Returns:
            v_lim_road: Speed limit due to road curvature (m/s)
        """
        kappa = self.road_curvature(s)
        if kappa > 0:
            return np.sqrt(self.a_y_comfort / kappa)
        else:
            return float('inf')  # No curvature means no speed limit
    
    def speed_limit_traffic(self, s):
        """
        Calculate speed limit due to traffic regulations.
        
        Args:
            s: Position along the road (m)
            
        Returns:
            v_lim_traffic: Speed limit due to traffic regulations (m/s)
        """
        if s <= 400:
            return 90 / 3.6  # Convert from km/h to m/s
        elif s <= 1000:
            return 50 / 3.6
        elif s <= 1800:
            return 70 / 3.6
        elif s <= 2400:
            return 50 / 3.6
        elif s <= 3000:
            return 90 / 3.6
        else:
            return 90 / 3.6
    
    def speed_limit_combined(self, s):
        """
        Calculate combined speed limit.
        
        Args:
            s: Position along the road (m)
            
        Returns:
            v_lim: Combined speed limit (m/s)
        """
        v_road = self.speed_limit_road(s)
        v_traffic = self.speed_limit_traffic(s)
        return min(v_road, v_traffic)
    
    def trigger_distance(self, v_curr, v_lim):
        """
        Calculate the trigger distance for deceleration.
        
        Args:
            v_curr: Current vehicle speed (m/s)
            v_lim: Speed limit (m/s)
            
        Returns:
            d_trig: Trigger distance (m)
        """
        if v_curr <= v_lim:
            return 0  # No need to decelerate
        else:
            return (v_curr**2 - v_lim**2) / (2 * abs(self.a_x_comfort))
    
    def desired_acceleration(self, s, v):
        """
        Calculate the desired acceleration based on current state.
        
        Args:
            s: Current position (m)
            v: Current velocity (m/s)
            
        Returns:
            a_des: Desired acceleration (m/s²)
        """
        # Look ahead for speed limits
        look_ahead_distance = 200  # metres to look ahead
        min_accel = 0  # Default acceleration
        
        # Check multiple points ahead to find required deceleration
        for distance in range(int(s), int(s + look_ahead_distance), 10):
            if distance >= self.road_length:
                continue
                
            v_lim = self.speed_limit_combined(distance)
            d_trig = self.trigger_distance(v, v_lim)
            
            # If we're within deceleration distance to a speed limit
            if distance - s <= d_trig:
                # Calculate required acceleration to meet speed limit
                if v > v_lim:
                    # Using basic acceleration formula: a = (v_final^2 - v_initial^2) / (2 * distance)
                    accel = (v_lim**2 - v**2) / (2 * (distance - s))
                    
                    # Get the maximum required deceleration (most negative)
                    min_accel = min(min_accel, accel)
        
        # Cap at comfortable deceleration
        min_accel = max(min_accel, self.a_x_comfort)
        
        # Don't exceed speed limit at current position
        current_limit = self.speed_limit_combined(s)
        if v > current_limit:
            return self.a_x_comfort  # Apply comfortable deceleration immediately
        elif v < current_limit - 1 and min_accel == 0:
            return 1.0  # Accelerate if below speed limit and no deceleration needed
        
        return min_accel
    
    def vehicle_dynamics(self, t, y):
        """
        Define vehicle dynamics for numerical integration.
        
        Args:
            t: Current time (s)
            y: Current state [s, v]
            
        Returns:
            dydt: Rate of change of state [v, a]
        """
        s, v = y
        a = self.desired_acceleration(s, v)
        return [v, a]
    
    def generate_road_geometry(self):
        """
        Generate road geometry using segment-based approach.
        
        Returns:
            X, Y: Arrays of X and Y coordinates of the road
        """
        # Number of segments
        N = int(self.road_length / self.segment_length)
        
        # Initialize arrays
        s = np.zeros(N+1)
        psi = np.zeros(N+1)
        X = np.zeros(N+1)
        Y = np.zeros(N+1)
        
        # Initial values
        s[0] = 0
        psi[0] = 0
        X[0] = 0
        Y[0] = 0
        
        # Generate road geometry
        for i in range(N):
            kappa = self.road_curvature(s[i])
            L = self.segment_length
            
            s[i+1] = s[i] + L
            psi[i+1] = psi[i] + L * kappa
            X[i+1] = X[i] + L * np.cos(psi[i] + kappa * L/2)
            Y[i+1] = Y[i] + L * np.sin(psi[i] + kappa * L/2)
        
        return X, Y
    def compute_vehicle_path(self):
        """
        Compute actual vehicle path based on curvature and traveled distance.
        Returns X, Y arrays representing the vehicle's path.
        """
        X = [0]
        Y = [0]
        psi = [0]  # heading angle

        for i in range(1, len(self.position)):
            ds = self.position[i] - self.position[i - 1]
            dpsi = self.curvature[i - 1] * ds
            psi.append(psi[-1] + dpsi)

            dx = ds * np.cos(psi[-1])
            dy = ds * np.sin(psi[-1])
            X.append(X[-1] + dx)
            Y.append(Y[-1] + dy)

        return np.array(X), np.array(Y)
    def compute_rmse_path_error(self, X_actual, Y_actual):
        """
        Compute RMSE between actual vehicle path and planned road geometry.
        """
        N = min(len(self.road_x), len(X_actual))
        errors = np.sqrt((self.road_x[:N] - X_actual[:N])**2 + (self.road_y[:N] - Y_actual[:N])**2)
        rmse = np.sqrt(np.mean(errors**2))
        return rmse


    def simulate(self, max_time=200):
        """
        Simulate vehicle dynamics along the road.
        
        Args:
            max_time: Maximum simulation time (s)
        """
        # Initial conditions [s0, v0]
        y0 = [0, self.v_init]
        
        # Time span for integration
        t_span = [0, max_time]
        
        # Stop integration when vehicle reaches the end of the road
        def event(t, y):
            return y[0] - self.road_length
        event.terminal = True
        
        # Solve ODE
        sol = solve_ivp(
            self.vehicle_dynamics, 
            t_span, 
            y0, 
            method='RK45', 
            events=event,
            max_step=0.5
        )
        
        # Store results
        self.time = sol.t
        self.position = sol.y[0]
        self.velocity = sol.y[1]
        
        # Calculate acceleration, curvature, and speed limits at each position
        self.acceleration = np.zeros_like(self.time)
        self.curvature = np.zeros_like(self.time)
        self.v_lim_road = np.zeros_like(self.time)
        self.v_lim_traffic = np.zeros_like(self.time)
        self.v_lim_combined = np.zeros_like(self.time)
        
        for i, (s, v) in enumerate(zip(self.position, self.velocity)):
            self.acceleration[i] = self.desired_acceleration(s, v)
            self.curvature[i] = self.road_curvature(s)
            self.v_lim_road[i] = self.speed_limit_road(s)
            self.v_lim_traffic[i] = self.speed_limit_traffic(s)
            self.v_lim_combined[i] = self.speed_limit_combined(s)
        
        # Generate road geometry
        self.road_x, self.road_y = self.generate_road_geometry()
    
    def plot_results(self):
        """
        Plot simulation results.
        """
        X_actual, Y_actual = self.compute_vehicle_path()
        rmse_error = self.compute_rmse_path_error(X_actual, Y_actual)
        # Create figure with subplots
        fig = plt.figure(figsize=(15, 12))
        
        # Plot road geometry
        ax1 = fig.add_subplot(4, 1, 1)
        ax1.plot(self.road_x, self.road_y, '--', label='Planned Road')
        ax1.plot(X_actual, Y_actual, label='Actual Vehicle Path')
        ax1.plot([], [], ' ', label=f'RMSE Path Error: {rmse_error:.3f} m')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title(
            f'Ax Comfort : {self.a_x_comfort}  Ay Comfort : {self.a_y_comfort}\n'
            f'Actual Vehicle Path vs Road Geometry'
        )
        ax1.grid(True)
        ax1.axis('equal')
        ax1.legend()
        
        # Plot speed vs position
        ax2 = fig.add_subplot(4, 1, 2)
        ax2.plot(self.position, self.velocity * 3.6, label='Vehicle Speed')
        ax2.plot(self.position, self.v_lim_road * 3.6, 'r--', label='Road Curvature Limit')
        ax2.plot(self.position, self.v_lim_traffic * 3.6, 'g--', label='Traffic Regulation Limit')
        ax2.plot(self.position, self.v_lim_combined * 3.6, 'k--', label='Combined Limit')
        ax2.set_xlabel('Position (m)')
        ax2.set_ylabel('Speed (km/h)')
        ax2.set_title('Vehicle Speed vs Position')
        ax2.grid(True)
        ax2.legend()
        
        # Plot acceleration vs position
        ax3 = fig.add_subplot(4, 1, 3)
        ax3.plot(self.position, self.acceleration)
        ax3.set_xlabel('Position (m)')
        ax3.set_ylabel('Acceleration (m/s²)')
        ax3.set_title('Vehicle Acceleration vs Position')
        ax3.grid(True)
        
        # Plot curvature vs position
        ax4 = fig.add_subplot(4, 1, 4)
        ax4.plot(self.position, self.curvature)
        ax4.set_xlabel('Position (m)')
        ax4.set_ylabel('Curvature (1/m)')
        ax4.set_title('Road Curvature vs Position')
        ax4.grid(True)





        plt.tight_layout()
        plt.show()

# Run simulation
if __name__ == "__main__":
    assistant = SmartSpeedAssistant(
        a_x_comfort=-2.0,  # Comfortable deceleration (m/s²)
        a_y_comfort=1.5,   # Comfortable lateral acceleration (m/s²)
        v_init=25.0,       # Initial speed (m/s) = 90 km/h
        road_length=3000,  # Road length (m)
        segment_length=50  # Segment length (m)
    )

    assistant2 = SmartSpeedAssistant(
        a_x_comfort=-5.0,
        a_y_comfort=3.0,
        v_init=25.0,
        road_length=3000,
        segment_length=50
    )
    assistant3 = SmartSpeedAssistant(
        a_x_comfort=-0.01,
        a_y_comfort=0.01,
        v_init=25.0,
        road_length=3000,
        segment_length=50
    )
    assistant4 = SmartSpeedAssistant(
        a_x_comfort=-1.0,
        a_y_comfort=0.8,
        v_init=25.0,
        road_length=3000,
        segment_length=50
    )
    assistant5 = SmartSpeedAssistant(
        a_x_comfort=0,
        a_y_comfort=0,
        v_init=45.0,
        road_length=3000,
        segment_length=50
    )
    # Run the simulation
    assistant5.simulate()
    assistant5.plot_results()
    assistant.simulate()
    assistant2.simulate()
    assistant3.simulate()
    assistant4.simulate()
    # Plot results
    assistant.plot_results()
    assistant2.plot_results()
    assistant3.plot_results()
    assistant4.plot_results()
