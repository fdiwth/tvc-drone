"""
TVC LQR 3D Simulation
=====================
Based on the linearised 4-state LQR:
  state = [roll, pitch, roll_rate, pitch_rate]
  control = [servo_x, servo_y]  (radians, clipped to ±MAX_SERVO_RAD)

Configure everything in the PHYSICS / LQR sections below.

Dependencies:
    pip install pygame-ce PyOpenGL PyOpenGL_accelerate numpy scipy

Controls:
    Mouse drag   orbit camera
    Scroll       zoom
    SPACE        reset
    D            toggle LQR on/off
    ESC          quit
"""

import sys, math, os, time
import numpy as np
from scipy.linalg import solve_continuous_are
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

# ── System matrices (match your LQR script exactly) ───────────────────────────

m   = 0.399       # kg
g   = 9.81      # m/s²
L   = 0.22      # TVC_ARM — pivot to COM
Ixx = 0.01804885602     # moment of inertia

grav = (m * g * L) / Ixx   # ≈ 70.0 rad/s²  — quite significant

A = np.array([
    [0, 0,     1, 0    ],
    [0, 0,     0, 1    ],
    [grav, 0,  0, 0    ],   # roll_rate accelerates with roll angle
    [0, grav,  0, 0    ],   # pitch_rate accelerates with pitch angle
])

B = np.array([
    [0,           0          ],
    [0,           0          ],
    [-26.90447525, 0          ],
    [0,            26.90447525],
], dtype=float)

Q = np.diag([10.0, 10.0, 1.0, 1.0])
R = np.diag([10.0, 10.0])

# ── Servo limits ──────────────────────────────────────────────────────────────
MAX_SERVO_DEG = 15.0                          # physical travel limit
MAX_SERVO_RAD = math.radians(MAX_SERVO_DEG)  # = 0.26180 rad (your clip used 0.18438)

# ── Initial conditions ────────────────────────────────────────────────────────
INITIAL_ROLL_DEG  = 15.0
INITIAL_PITCH_DEG = 15.0

# ── OBJ model (optional) ──────────────────────────────────────────────────────
OBJ_PATH = None   # e.g. "rocket.obj"

# ═══════════════════════════════════════════════════════════════════════════════
#  END OF CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

# ── Solve LQR ─────────────────────────────────────────────────────────────────
P_lqr = solve_continuous_are(A, B, Q, R)
K_LQR = np.linalg.inv(R) @ B.T @ P_lqr
print("K =", K_LQR)

DT = 1.0 / 120.0   # physics timestep

# ── Quaternion helpers ────────────────────────────────────────────────────────
def qmul(q1, q2):
    w1,x1,y1,z1 = q1;  w2,x2,y2,z2 = q2
    return np.array([
        w1*w2-x1*x2-y1*y2-z1*z2,
        w1*x2+x1*w2+y1*z2-z1*y2,
        w1*y2-x1*z2+y1*w2+z1*x2,
        w1*z2+x1*y2-y1*x2+z1*w2,
    ])

def qnorm(q):  return q / np.linalg.norm(q)

def quat_to_mat4(q):
    w,x,y,z = q
    return np.array([
        [1-2*(y*y+z*z),  2*(x*y-w*z),   2*(x*z+w*y), 0],
        [  2*(x*y+w*z),1-2*(x*x+z*z),   2*(y*z-w*x), 0],
        [  2*(x*z-w*y),  2*(y*z+w*x), 1-2*(x*x+y*y), 0],
        [0,             0,             0,             1],
    ], dtype=np.float32)

def euler_to_quat(roll, pitch, yaw=0.0):
    cr,sr = math.cos(roll/2),  math.sin(roll/2)
    cp,sp = math.cos(pitch/2), math.sin(pitch/2)
    cy,sy = math.cos(yaw/2),   math.sin(yaw/2)
    return qnorm(np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ]))

def quat_integrate(q, omega, dt):
    omag = np.linalg.norm(omega)
    if omag < 1e-12: return q
    axis  = omega / omag
    angle = omag * dt
    dq = np.array([math.cos(angle/2), *(axis * math.sin(angle/2))])
    return qnorm(qmul(q, dq))

# ── OBJ loader ────────────────────────────────────────────────────────────────
def load_obj(path):
    verts, tris = [], []
    with open(path) as f:
        for line in f:
            t = line.strip().split()
            if not t: continue
            if t[0] == 'v':
                verts.append((float(t[1]), float(t[2]), float(t[3])))
            elif t[0] == 'f':
                idx = [int(s.split('/')[0])-1 for s in t[1:]]
                for i in range(1, len(idx)-1):
                    tris.append((verts[idx[0]], verts[idx[i]], verts[idx[i+1]]))
    return tris

def centre_and_scale_tris(tris, target_height=1.4):
    all_v = np.array([v for tri in tris for v in tri])
    mn, mx = all_v.min(0), all_v.max(0)
    centre = (mn + mx) / 2
    scale  = target_height / max(mx[1]-mn[1], 1e-9)
    return [tuple((np.array(v)-centre)*scale for v in tri) for tri in tris]

# ── Procedural rocket ─────────────────────────────────────────────────────────
def _cyl(r, h, n, y0):
    out = []
    for i in range(n):
        a0,a1 = 2*math.pi*i/n, 2*math.pi*(i+1)/n
        x0,z0 = math.cos(a0)*r, math.sin(a0)*r
        x1,z1 = math.cos(a1)*r, math.sin(a1)*r
        out += [((x0,y0,z0),(x1,y0,z1),(x1,y0+h,z1)),
                ((x0,y0,z0),(x1,y0+h,z1),(x0,y0+h,z0)),
                ((0,y0,0),(x0,y0,z0),(x1,y0,z1)),
                ((0,y0+h,0),(x1,y0+h,z1),(x0,y0+h,z0))]
    return out

def _cone(r, h, n, y0):
    out = []
    for i in range(n):
        a0,a1 = 2*math.pi*i/n, 2*math.pi*(i+1)/n
        x0,z0 = math.cos(a0)*r, math.sin(a0)*r
        x1,z1 = math.cos(a1)*r, math.sin(a1)*r
        out += [((x0,y0,z0),(x1,y0,z1),(0,y0+h,0)),
                ((0,y0,0),(x0,y0,z0),(x1,y0,z1))]
    return out

def build_procedural_rocket():
    tris  = _cyl(0.08, 1.4, 20, -0.6) + _cone(0.08, 0.35, 20, 0.8) + _cone(0.055, -0.14, 16, -0.6)
    for angle in [0, 90, 180, 270]:
        ca, sa = math.cos(math.radians(angle)), math.sin(math.radians(angle))
        for tri in _cyl(0.005, 0.18, 4, -0.62):
            tris.append(tuple(
                (v[0]*ca - v[2]*sa + 0.085*ca, v[1], v[0]*sa + v[2]*ca + 0.085*sa)
                for v in tri))
    return tris

# ── OpenGL helpers ────────────────────────────────────────────────────────────
def compile_rocket_dl(tris):
    dl = glGenLists(1)
    glNewList(dl, GL_COMPILE)
    glColor4f(0.14, 0.14, 0.18, 1.0)
    glBegin(GL_TRIANGLES)
    for tri in tris:
        for v in tri: glVertex3f(*v)
    glEnd()
    # COM ring
    glColor4f(1.0, 0.9, 0.2, 1.0); glLineWidth(2.0)
    glBegin(GL_LINE_LOOP)
    for i in range(32):
        a = 2*math.pi*i/32; glVertex3f(math.cos(a)*0.09, 0, math.sin(a)*0.09)
    glEnd()
    glEndList()
    return dl

def compile_grid():
    dl = glGenLists(1)
    glNewList(dl, GL_COMPILE)
    glColor4f(0.20, 0.20, 0.26, 1.0); glLineWidth(1.0)
    glBegin(GL_LINES)
    for i in range(-12, 13):
        glVertex3f(i*0.5, 0, -6.0); glVertex3f(i*0.5, 0,  6.0)
        glVertex3f(-6.0,  0, i*0.5); glVertex3f( 6.0,  0, i*0.5)
    glEnd()
    glEndList()
    return dl

def draw_world_axes():
    glLineWidth(2.5)
    glBegin(GL_LINES)
    glColor3f(.9,.2,.2); glVertex3f(0,0,0); glVertex3f(.5,0,0)
    glColor3f(.2,.9,.2); glVertex3f(0,0,0); glVertex3f(0,.5,0)
    glColor3f(.2,.2,.9); glVertex3f(0,0,0); glVertex3f(0,0,.5)
    glEnd()

def draw_nozzle(servo_x_rad, servo_y_rad, thrust_scale=1.0):
    """
    servo_x deflects around X axis  → pitch torque
    servo_y deflects around Z axis  → roll torque
    The nozzle is drawn at the base of the rocket (y = -0.6).
    """
    glPushMatrix()
    glTranslatef(0, -0.6, 0)
    glRotatef(math.degrees(servo_x_rad), 1, 0, 0)
    glRotatef(math.degrees(servo_y_rad), 0, 0, 1)

    # Nozzle bell
    glColor4f(0.55, 0.55, 0.60, 1.0)
    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(0, 0, 0)
    for i in range(13):
        a = 2*math.pi*i/12
        glVertex3f(math.cos(a)*0.055, -0.07, math.sin(a)*0.055)
    glEnd()

    # Flame
    fl = 0.18 + thrust_scale * 0.016
    glBegin(GL_TRIANGLE_FAN)
    glColor4f(1.0, 0.65, 0.1, 0.95); glVertex3f(0, -fl, 0)
    for i in range(13):
        a = 2*math.pi*i/12
        glColor4f(1.0, 0.1, 0.0, 0.0)
        glVertex3f(math.cos(a)*0.035, -0.07, math.sin(a)*0.035)
    glEnd()

    glPopMatrix()

# ── Servo angle bar (HUD element drawn in OpenGL 2D) ─────────────────────────
def draw_servo_bars(sx_deg, sy_deg, W, H, limit):
    """Small gimbal indicator cross in bottom-right corner."""
    cx, cy = W - 80, H - 80
    size   = 55

    glLineWidth(1.5)
    # background circle
    glColor4f(0.1, 0.1, 0.15, 0.8)
    glBegin(GL_TRIANGLE_FAN)
    glVertex2f(cx, cy)
    for i in range(33):
        a = 2*math.pi*i/32
        glVertex2f(cx + math.cos(a)*size, cy + math.sin(a)*size)
    glEnd()

    # crosshair
    glColor4f(0.3, 0.3, 0.4, 1.0)
    glBegin(GL_LINES)
    glVertex2f(cx-size, cy); glVertex2f(cx+size, cy)
    glVertex2f(cx, cy-size); glVertex2f(cx, cy+size)
    glEnd()

    # dot position
    nx = cx + (sx_deg / limit) * size
    ny = cy + (sy_deg / limit) * size
    glColor4f(0.2, 1.0, 0.5, 1.0)
    glBegin(GL_TRIANGLE_FAN)
    glVertex2f(nx, ny)
    for i in range(17):
        a = 2*math.pi*i/16
        glVertex2f(nx + math.cos(a)*5, ny + math.sin(a)*5)
    glEnd()

# ── Pygame HUD ────────────────────────────────────────────────────────────────
def render_hud(surf, fsm, flg, state, servo, lqr_on, elapsed, W, H):
    roll_d  = math.degrees(state['roll'])
    pitch_d = math.degrees(state['pitch'])
    rr_d    = math.degrees(state['roll_rate'])
    pr_d    = math.degrees(state['pitch_rate'])
    sx_d    = math.degrees(servo[0])
    sy_d    = math.degrees(servo[1])

    pw, ph = 250, 330
    panel  = pygame.Surface((pw, ph), pygame.SRCALPHA)
    panel.fill((8, 10, 20, 210))

    def lbl(t, y, c=(100,120,145)): panel.blit(fsm.render(t, True, c), (12, y))
    def val(t, y, c=(55,215,135)):
        s = fsm.render(t, True, c); panel.blit(s, (pw - s.get_width() - 12, y))

    title = flg.render("TVC LQR", True, (170, 210, 255))
    panel.blit(title, (pw//2 - title.get_width()//2, 10))

    lc = (55, 220, 90) if lqr_on else (220, 70, 70)
    lt = fsm.render(f"LQR {'ACTIVE' if lqr_on else 'OFF'}  [D]", True, lc)
    panel.blit(lt, (pw//2 - lt.get_width()//2, 34))

    y = 62
    lbl("── ATTITUDE ──", y, (150,150,175)); y += 20
    lbl("Roll",  y); val(f"{roll_d:+7.2f} °", y);  y += 18
    lbl("Pitch", y); val(f"{pitch_d:+7.2f} °", y); y += 22

    lbl("── RATES ──", y, (150,150,175)); y += 20
    lbl("Roll rate",  y); val(f"{rr_d:+7.1f} °/s", y); y += 18
    lbl("Pitch rate", y); val(f"{pr_d:+7.1f} °/s", y); y += 22

    lbl("── SERVO ──", y, (150,150,175)); y += 20
    sc = (220, 100, 60) if abs(sx_d) > MAX_SERVO_DEG*0.9 else (55,215,135)
    lbl("Servo X", y); val(f"{sx_d:+6.2f} °", y, sc); y += 18
    sc = (220, 100, 60) if abs(sy_d) > MAX_SERVO_DEG*0.9 else (55,215,135)
    lbl("Servo Y", y); val(f"{sy_d:+6.2f} °", y, sc); y += 22

    lbl("Time", y); val(f"{elapsed:.2f} s", y, (180,180,200)); y += 18

    # K matrix summary
    k_str = f"K=[{K_LQR[0,0]:.1f}, {K_LQR[0,2]:.2f} | {K_LQR[1,1]:.1f}, {K_LQR[1,3]:.2f}]"
    ks = fsm.render(k_str, True, (70, 80, 100))
    panel.blit(ks, (pw//2 - ks.get_width()//2, y))

    surf.blit(panel, (10, 10))

    hints = ["SPACE  reset", "D  toggle LQR", "drag  orbit", "scroll  zoom"]
    for i, h in enumerate(hints):
        s = fsm.render(h, True, (65, 65, 85))
        surf.blit(s, (14, H - 20 - (len(hints)-i-1)*16))

# ═══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    pygame.init()
    W, H   = 1100, 720
    screen = pygame.display.set_mode((W, H), DOUBLEBUF | OPENGL)
    pygame.display.set_caption("TVC LQR Simulation")
    hud    = pygame.Surface((W, H), pygame.SRCALPHA)
    fsm    = pygame.font.SysFont("monospace", 13)
    flg    = pygame.font.SysFont("monospace", 16, bold=True)

    glEnable(GL_DEPTH_TEST); glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    glClearColor(0.05, 0.05, 0.09, 1.0)
    glMatrixMode(GL_PROJECTION); glLoadIdentity()
    gluPerspective(45, W/H, 0.05, 200.0)
    glMatrixMode(GL_MODELVIEW)

    if OBJ_PATH and os.path.exists(OBJ_PATH):
        tris = centre_and_scale_tris(load_obj(OBJ_PATH))
        print(f"Loaded OBJ: {len(tris)} triangles")
    else:
        if OBJ_PATH: print(f"OBJ not found at '{OBJ_PATH}', using procedural model.")
        tris = build_procedural_rocket()

    rocket_dl = compile_rocket_dl(tris)
    grid_dl   = compile_grid()

    def make_state():
        max_rad = math.radians(max(INITIAL_ROLL_DEG, INITIAL_PITCH_DEG))
        angle   = np.random.uniform(0, max_rad)
        direction = np.random.uniform(0, 2 * math.pi)
        return {
            'roll':       angle * math.cos(direction),
            'pitch':      angle * math.sin(direction),
            'roll_rate':  0.0,
            'pitch_rate': 0.0,
        }

    state   = make_state()
    lqr_on  = True
    servo   = np.zeros(2)
    t_start = time.time()
    acc_dt  = 0.0

    # Camera
    cam_yaw, cam_pitch, cam_dist = 35.0, -18.0, 4.0
    mouse_dn, mouse_last = False, (0, 0)
    clock = pygame.time.Clock()

    while True:
        frame_dt = clock.tick(120) / 1000.0
        acc_dt  += frame_dt

        for ev in pygame.event.get():
            if ev.type == QUIT or (ev.type == KEYDOWN and ev.key == K_ESCAPE):
                pygame.quit(); sys.exit()
            if ev.type == KEYDOWN:
                if ev.key == K_SPACE: state = make_state(); t_start = time.time()
                if ev.key == K_d:     lqr_on = not lqr_on
            if ev.type == MOUSEBUTTONDOWN and ev.button == 1:
                mouse_dn = True; mouse_last = pygame.mouse.get_pos()
            if ev.type == MOUSEBUTTONUP and ev.button == 1:
                mouse_dn = False
            if ev.type == MOUSEMOTION and mouse_dn:
                mx, my = pygame.mouse.get_pos()
                cam_yaw   += (mx - mouse_last[0]) * 0.4
                cam_pitch  = max(-89, min(89, cam_pitch + (my-mouse_last[1])*0.4))
                mouse_last  = (mx, my)
            if ev.type == MOUSEBUTTONDOWN:
                if ev.button == 4: cam_dist = max(0.8, cam_dist - 0.25)
                if ev.button == 5: cam_dist = min(25.0, cam_dist + 0.25)

        # ── Physics: integrate the same 4-state linear system ─────────────────
        while acc_dt >= DT:
            acc_dt -= DT
            x = np.array([state['roll'], state['pitch'],
                          state['roll_rate'], state['pitch_rate']])

            if lqr_on:
                u = -K_LQR @ x
                u = np.clip(u, -MAX_SERVO_RAD, MAX_SERVO_RAD)
            else:
                u = np.zeros(2)

            servo = u.copy()

            # dx/dt = A x + B u  — Euler integration
            dxdt        = A @ x + B @ u
            x_next      = x + dxdt * DT

            state['roll']       = x_next[0]
            state['pitch']      = x_next[1]
            state['roll_rate']  = x_next[2]
            state['pitch_rate'] = x_next[3]

        # ── Build quaternion from current roll/pitch for 3D display ───────────
        q = euler_to_quat(state['roll'], state['pitch'])

        # ── Render ────────────────────────────────────────────────────────────
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        cy = math.radians(cam_yaw); cp = math.radians(cam_pitch)
        eye = np.array([
            cam_dist*math.cos(cp)*math.sin(cy),
            cam_dist*math.sin(cp),
            cam_dist*math.cos(cp)*math.cos(cy),
        ])
        gluLookAt(*eye, 0, 0, 0, 0, 1, 0)

        glPushMatrix(); glTranslatef(0, -1.5, 0); glCallList(grid_dl); glPopMatrix()
        draw_world_axes()

        glPushMatrix()
        glMultMatrixf(quat_to_mat4(q).T)
        glCallList(rocket_dl)
        draw_nozzle(servo[0], servo[1])
        glPopMatrix()

        # ── HUD ───────────────────────────────────────────────────────────────
        glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity()
        glOrtho(0, W, H, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity()
        glDisable(GL_DEPTH_TEST)

        # Servo gimbal cross
        draw_servo_bars(math.degrees(servo[0]), math.degrees(servo[1]),
                        W, H, MAX_SERVO_DEG)

        # Pygame panel
        hud.fill((0, 0, 0, 0))
        render_hud(hud, fsm, flg, state, servo, lqr_on,
                   time.time()-t_start, W, H)
        data = pygame.image.tostring(hud, "RGBA", True)
        glRasterPos2f(0, 0)
        glDrawPixels(W, H, GL_RGBA, GL_UNSIGNED_BYTE, data)

        glEnable(GL_DEPTH_TEST)
        glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix()
        glMatrixMode(GL_MODELVIEW)

        pygame.display.flip()

if __name__ == "__main__":
    main()