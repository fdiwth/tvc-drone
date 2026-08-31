import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import LinearNDInterpolator

# ==========================================
# 1. SAMPLE DATA (Simulating your file/input)
# ==========================================
# Replace this string with your actual file parsing or data stream
data_input = """
12.54	0.028	10
12.48	0.077	20
12.38	0.143	30
12.24	0.212	40
12.06	0.28	50
11.82	0.339	60
11.6	0.399	70
11.36	0.45	80
11.2	0.495	90
12.15	0.027	10
12.12	0.075	20
12.05	0.136	30
11.95	0.203	40
11.8	0.271	50
11.61	0.335	60
11.4	0.39	70
11.2	0.44	80
11	0.484	90
11.7	0.025	10
11.67	0.066	20
11.63	0.124	30
11.53	0.188	40
11.4	0.251	50
11.25	0.316	60
11.1	0.376	70
10.9	0.429	80
10.7	0.463	90
11.4	0.023	10
11.39	0.066	20
11.32	0.12	30
11.24	0.18	40
11.13	0.245	50
11	0.307	60
10.82	0.361	70
10.65	0.417	80
10.46	0.453	90
"""
# Parse lines containing 3 space-separated numbers
x, y, z = [], [], []
for line in data_input.strip().split('\n'):
    parts = line.split()
    if len(parts) == 3:
        x.append(float(parts[0]))
        y.append(float(parts[1]))
        z.append(float(parts[2]))

x = np.array(x)
y = np.array(y)
z = np.array(z)

# ==========================================
# 2. SURFACE FITTING & EQUATION EXTRACTION
# ==========================================
A = np.c_[np.ones_like(x), x, y, x**2, x*y, y**2]
coefficients, _, _, _ = np.linalg.lstsq(A, z, rcond=None)
a, b, c, d, e, f = coefficients

print("interpolated equation")
print(f"z = {a:.8f} + ({b:.8f})*x + ({c:.8f})*y + ({d:.8f})*x^2 + ({e:.8f})*x*y + ({f:.8f})*y^2\n")

# ==========================================
# 3. LOWER-RESOLUTION SPATIAL INTERPOLATION
# ==========================================
# Lowered resolution to a 30x30 grid to eliminate lag/rendering bugs
grid_x, grid_y = np.meshgrid(np.linspace(min(x), max(x), 50),
                             np.linspace(min(y), max(y), 50))

# Linear spatial interpolation
interp_func = LinearNDInterpolator(list(zip(x, y)), z)
grid_z_linear = interp_func(grid_x, grid_y)

# Polynomial fallback for outer bounds
grid_z_poly = a + b*grid_x + c*grid_y + d*grid_x**2 + e*grid_x*grid_y + f*grid_y**2
grid_z = np.where(np.isnan(grid_z_linear), grid_z_poly, grid_z_linear)

# ==========================================
# 4. 3D VISUALIZATION (Standard Native Box)
# ==========================================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(projection='3d')

# Plot the surface (antialiased=True adds smoothness to the lower res grid)
surf = ax.plot_surface(grid_x, grid_y, grid_z, cmap='viridis', alpha=0.7, antialiased=True)

# Plot original data points
ax.scatter(x, y, z, color='red', s=50, label='Parsed Points', depthshade=True)

# Reverted to native Matplotlib axis handling (auto-adjusts dynamically based on data)
ax.set_xlabel('X Axis', fontsize=11, fontweight='bold')
ax.set_ylabel('Y Axis', fontsize=11, fontweight='bold')
ax.set_zlabel('Z Axis', fontsize=11, fontweight='bold')

plt.title("voltage vs. thrust vs. pwm", fontsize=14)
plt.legend()
plt.show()