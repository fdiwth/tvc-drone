import numpy as np
import scipy.linalg as linalg
import scipy.signal as signal
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# ============================================================
# Physical parameters
# ============================================================
mass = 0.430
g    = 9.81
thrust_n = mass * g
arm  = 0.205
I_xx = 0.01802468421
I_yy = 0.01802468421
B_rz_aero = 0.392   # rad/s^2 per kg

B_rx = (thrust_n * arm) / I_xx
B_ry = (thrust_n * arm) / I_yy
B_pz = 1.0 / mass

dt = 0.01

# ============================================================
# 12-state continuous plant: 
# [rx, ry, rz, pz, drx, dry, drz, dpz, irx, iry, irz, ipz]
# ============================================================
A_c = np.array([
    # rx, ry, rz, pz,  drx, dry, drz, dpz,  irx, iry, irz, ipz
    [ 0,  0,  0,  0,    1,   0,   0,   0,    0,   0,   0,   0 ], # d(rx)/dt  = drx
    [ 0,  0,  0,  0,    0,   1,   0,   0,    0,   0,   0,   0 ], # d(ry)/dt  = dry
    [ 0,  0,  0,  0,    0,   0,   1,   0,    0,   0,   0,   0 ], # d(rz)/dt  = drz
    [ 0,  0,  0,  0,    0,   0,   0,   1,    0,   0,   0,   0 ], # d(pz)/dt  = dpz
    [ 0,  0,  0,  0,    0,   0,   0,   0,    0,   0,   0,   0 ], # d(drx)/dt = (from B)
    [ 0,  0,  0,  0,    0,   0,   0,   0,    0,   0,   0,   0 ], # d(dry)/dt = (from B)
    [ 0,  0,  0,  0,    0,   0,   0,   0,    0,   0,   0,   0 ], # d(drz)/dt = (from B)
    [ 0,  0,  0,  0,    0,   0,   0,   0,    0,   0,   0,   0 ], # d(dpz)/dt = (from B)
    [ 1,  0,  0,  0,    0,   0,   0,   0,    0,   0,   0,   0 ], # d(irx)/dt = rx
    [ 0,  1,  0,  0,    0,   0,   0,   0,    0,   0,   0,   0 ], # d(iry)/dt = ry
    [ 0,  0,  1,  0,    0,   0,   0,   0,    0,   0,   0,   0 ], # d(irz)/dt = rz
    [ 0,  0,  0,  1,    0,   0,   0,   0,    0,   0,   0,   0 ], # d(ipz)/dt = pz
], dtype=float)

B_c = np.array([
    # u0,      u1,       u2,          u3
    [ 0,       0,        0,           0    ], # rx
    [ 0,       0,        0,           0    ], # ry
    [ 0,       0,        0,           0    ], # rz
    [ 0,       0,        0,           0    ], # pz
    [-B_rx,    0,        0,           0    ], # drx
    [ 0,       B_ry,     0,           0    ], # dry
    [ 0,       0,        B_rz_aero,   0    ], # drz
    [ 0,       0,        0,           B_pz ], # dpz
    [ 0,       0,        0,           0    ], # irx
    [ 0,       0,        0,           0    ], # iry
    [ 0,       0,        0,           0    ], # irz
    [ 0,       0,        0,           0    ], # ipz
], dtype=float)

# Discretize the fully augmented 12-state system
discrete_sys = signal.StateSpace(A_c, B_c, np.eye(12), np.zeros((12, 4))).to_discrete(dt)
A_d = discrete_sys.A
B_d = discrete_sys.B

# ============================================================
# Controllability check
# ============================================================
ctrb = np.hstack([np.linalg.matrix_power(A_d, i) @ B_d for i in range(A_d.shape[0])])
rank = np.linalg.matrix_rank(ctrb)
print(f"Controllability rank: {rank} / {A_d.shape[0]}  {'OK' if rank == A_d.shape[0] else 'WARNING: NOT FULLY CONTROLLABLE'}")

# ============================================================
# LQI Unified Calculation
# ============================================================
Q12 = np.diag([
    # PD States
    130., # rx
    130., # ry
    25, # rz
    15.,  # pz
    4.,   # drx
    4.,   # dry
    0.00001, # drz
    0.1,  # dpz
    10.,  # irx
    10.,  # iry
    0.1,  # irz
    0.01  # ipz
])

R = np.diag([
    100., # servo-x in rad
    100., # servo-y in rad
    17500., # differential thrust in kg
    5.    # thrust in newtons
])

# Solve the single DARE for all 12 states simultaneously
P12 = linalg.solve_discrete_are(A_d, B_d, Q12, R)
K_full = linalg.inv(R + B_d.T @ P12 @ B_d) @ B_d.T @ P12 @ A_d

# ============================================================
# Verification and Output
# ============================================================
print("\nK_full integral columns (signs now inherently mapped to plant physics):")
for ch in range(4):
    print(f"  K[{ch}][{8+ch}] = {K_full[ch, 8+ch]:.8f}")

print("\nconst float K[LQR_MAX_INPUTS][LQR_MAX_STATES] = {")
for row in K_full:
    print("    {" + ", ".join(f"{v:.8f}f" for v in row) + "},")
print("};")

# Closed-loop DISCRETE poles of the full 12-state system
Acl_d = A_d - B_d @ K_full
eig_d = np.linalg.eigvals(Acl_d)
print("\nClosed-loop DISCRETE poles (magnitude must be < 1.0 for stability):")
for e in sorted(eig_d, key=abs, reverse=True):
    print(f"  {e:+.4f} (Mag: {abs(e):.4f})")

# ============================================================
# ZOH simulation — mirrors firmware exactly
# ============================================================
u_min = np.array([-0.149066, -0.149066, -0.1, -0.1 * g])
u_max = np.array([ 0.149066,  0.149066,  0.1,  0.05 * g])

INTEGRAL_MAX = np.array([1.0, 1.0, 0.2, 0.1])
x_ref12 = np.zeros(12)

def zoh_simulate(x0_8, integ0, T):
    n_steps = int(T / dt)
    x  = x0_8.copy()
    ig = integ0.copy()

    xs  = [x.copy()]
    igs = [ig.copy()]
    us  = []
    
    # Extract the 8-state physical dynamics for continuous IVP simulation
    A_c_phys = A_c[:8, :8]
    B_c_phys = B_c[:8, :]

    for _ in range(n_steps):
        x12  = np.concatenate([x, ig])
        err12 = x12 - x_ref12

        u = -K_full @ err12
        u_clamped = np.clip(u, u_min, u_max)

        # Firmware-style integral accumulation (with anti-windup conceptually applied to all)
        for ch in range(4):
            saturated = (u[ch] <= u_min[ch] * 0.99) or (u[ch] >= u_max[ch] * 0.99)
            if not saturated:
                ig[ch] += err12[ch] * dt
            ig[ch] = np.clip(ig[ch], -INTEGRAL_MAX[ch], INTEGRAL_MAX[ch])

        us.append(u_clamped.copy())

        # Forward simulate continuous physics via IVP solver using only the 8 physical states
        def f(t, xx): return A_c_phys @ xx + B_c_phys @ u_clamped
        sol = solve_ivp(f, (0, dt), x, method='RK45', max_step=dt / 4)
        x = sol.y[:, -1]

        xs.append(x.copy())
        igs.append(ig.copy())

    return np.array(xs), np.array(igs), np.array(us)

x0 = np.array([np.radians(15), np.radians(10), np.radians(20), -0.5,
               0., 0., 0., 0.])

T  = 8.0
xs, igs, us = zoh_simulate(x0, np.zeros(4), T)
t  = np.linspace(0, T, len(xs))
tu = np.linspace(0, T - dt, len(us))

# ============================================================
# Plot
# ============================================================
fig, axes = plt.subplots(4, 3, figsize=(14, 10))
fig.suptitle('LQI (12-state) | ZOH @ 100Hz\n'
             'Yaw: aero-drag torque only', fontsize=11)

axes[0,0].plot(t, np.degrees(xs[:,0]), 'b', lw=2); axes[0,0].axhline(0,color='r',ls='--'); axes[0,0].set_title('Angle X (deg)'); axes[0,0].grid(True)
axes[0,1].plot(t, np.degrees(xs[:,1]), 'b', lw=2); axes[0,1].axhline(0,color='r',ls='--'); axes[0,1].set_title('Angle Y (deg)'); axes[0,1].grid(True)
axes[0,2].plot(t, np.degrees(xs[:,2]), 'b', lw=2); axes[0,2].axhline(0,color='r',ls='--'); axes[0,2].set_title('Angle Z / Yaw (deg)'); axes[0,2].grid(True)

axes[1,0].plot(t, np.degrees(xs[:,4]), 'g', lw=2); axes[1,0].axhline(0,color='r',ls='--'); axes[1,0].set_title('Rate X (deg/s)'); axes[1,0].grid(True)
axes[1,1].plot(t, np.degrees(xs[:,5]), 'g', lw=2); axes[1,1].axhline(0,color='r',ls='--'); axes[1,1].set_title('Rate Y (deg/s)'); axes[1,1].grid(True)
axes[1,2].plot(t, np.degrees(xs[:,6]), 'g', lw=2); axes[1,2].axhline(0,color='r',ls='--'); axes[1,2].set_title('Rate Z (deg/s)'); axes[1,2].grid(True)

axes[2,0].plot(t, xs[:,3], 'purple', lw=2); axes[2,0].axhline(0,color='r',ls='--'); axes[2,0].set_title('Altitude Error (m)'); axes[2,0].grid(True)
axes[2,1].plot(t, xs[:,7], 'purple', lw=2); axes[2,1].axhline(0,color='r',ls='--'); axes[2,1].set_title('Altitude Rate (m/s)'); axes[2,1].grid(True)
axes[2,2].plot(t, igs[:,0], 'b', lw=2, label='irx')
axes[2,2].plot(t, igs[:,1], 'r', lw=2, label='iry')
axes[2,2].plot(t, igs[:,2], 'g', lw=1.5, ls='--', label='irz')
axes[2,2].set_title('Integrals (rad·s)'); axes[2,2].legend(); axes[2,2].grid(True)

axes[3,0].plot(tu, np.degrees(us[:,0]), 'r',      lw=2, label='Servo X')
axes[3,0].plot(tu, np.degrees(us[:,1]), 'orange',  lw=2, label='Servo Y')
axes[3,0].axhline(np.degrees(u_max[0]), color='k', ls=':')
axes[3,0].axhline(np.degrees(u_min[0]), color='k', ls=':')
axes[3,0].set_title('Servo Output (deg)'); axes[3,0].legend(); axes[3,0].grid(True)

axes[3,1].plot(tu, us[:,2], 'darkblue', lw=2); axes[3,1].axhline(0,color='r',ls='--'); axes[3,1].set_title('Yaw Torque (Nm)'); axes[3,1].grid(True)
axes[3,2].plot(tu, us[:,3], 'darkred',  lw=2); axes[3,2].axhline(0,color='r',ls='--'); axes[3,2].set_title('Thrust Delta (N)'); axes[3,2].grid(True)

plt.tight_layout()
plt.show()