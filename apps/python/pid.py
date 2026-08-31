import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

# --- System Parameters ---
THRUST = 0.405 * 9.81
MASS = 0.405     # Kilograms
DT = 0.01      # Simulation time step (seconds)
SIM_TIME = 5.0 # Total simulation time (seconds)

def simulate_system(pid_gains, plot=False):
    Kp, Ki, Kd = pid_gains
    
    steps = int(SIM_TIME / DT)
    
    vx = 0.5 # initial horizontal velocity
    
    integral_e = 0.0
    prev_e = 0.0
    total_cost = 0.0
    
    # Arrays for plotting
    time_data = []
    vx_data = []
    theta_data = []
    
    for step in range(steps):
        # 1. Calculate Error (Target horizontal velocity is 0)
        error = 0.0 - vx
        
        # 2. PID Calculations
        integral_e += error * DT
        derivative_e = (error - prev_e) / DT
        
        # Output is the angle theta in radians
        theta = (Kp * error) + (Ki * integral_e) + (Kd * derivative_e)
        
        # Constrain the angle to realistic physical limits (-pi/2 to pi/2)
        theta = np.clip(theta, -np.pi/2, np.pi/2)
        
        # 3. Physics Model
        # Horizontal force is Thrust * sin(theta)
        # Acceleration is Force / Mass
        ax = (THRUST / MASS) * np.sin(theta)
        
        # Update velocity
        vx += ax * DT
        
        # 4. Calculate Cost (Integral Square Error to penalize large deviations)
        total_cost += (error ** 2) * DT
        
        # Store for plotting
        if plot:
            time_data.append(step * DT)
            vx_data.append(vx)
            theta_data.append(theta)
            
        prev_e = error
        
    if plot:
        return time_data, vx_data, theta_data
        
    return total_cost

def find_ideal_pid():
    # Initial guess for [Kp, Ki, Kd]
    initial_guess = [0.5, 0.1, 0.1]
    
    bounds = [(0, None), (0, None), (0, None)]
    
    print(f"Optimizing PID values for Thrust = {THRUST}N, Mass = {MASS}kg...")
    
    # Run the optimization
    result = minimize(
        simulate_system, 
        initial_guess, 
        method='L-BFGS-B', 
        bounds=bounds
    )
    
    if result.success:
        best_Kp, best_Ki, best_Kd = result.x
        print("\nOptimization Successful!")
        print(f"Ideal Kp: {best_Kp:.4f}")
        print(f"Ideal Ki: {best_Ki:.4f}")
        print(f"Ideal Kd: {best_Kd:.4f}")
        return result.x
    else:
        print("Optimization failed.")
        return initial_guess

# --- Main Execution ---
if __name__ == "__main__":
    # 1. Find the ideal PID values
    ideal_gains = find_ideal_pid()
    
    # 2. Run a final simulation with the ideal gains to visualize the result
    time_history, vx_history, theta_history = simulate_system(ideal_gains, plot=True)
    
    # 3. Plot the results
    plt.figure(figsize=(10, 6))
    
    plt.subplot(2, 1, 1)
    plt.plot(time_history, vx_history, label="Horizontal Velocity (m/s)", color="blue", linewidth=2)
    plt.axhline(0, color='red', linestyle='--', label="Target Velocity (0 m/s)")
    plt.title("System Response with Ideal PID Gains")
    plt.ylabel("Velocity (m/s)")
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(time_history, theta_history, label="Stick Angle (Radians)", color="green", linewidth=2)
    plt.ylabel("Angle (rad)")
    plt.xlabel("Time (s)")
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()