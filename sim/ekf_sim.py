"""
ekf_sim.py — Validación del EKF descentralizado en la sim 2D
============================================================
Port 1:1 de la matemática que irá a `EKFState` en utils.h (ESP32):

  estado  [x, y, θ]
  predict con odometría (distancia de encoders + Δθ de gyro) por sub-paso
  update  con pose ArUco (x, y, θ) cuando la cámara ve el marker

Escrito con matrices 3x3 explícitas (sin numpy) para que el port a C++ sea
línea a línea. θ interno en radianes; conversión a grados en las fronteras.

Escenarios:
  python ekf_sim.py --scenario oclusion    # GT con ventana de oclusión al medio
  python ekf_sim.py --scenario ciego       # cámara muere a mitad del GT
  python ekf_sim.py --scenario montecarlo  # n corridas, estadística de error

Convenciones de la sim (attabot_sim.py): Y hacia abajo, 0°=+X, CW positivo.
"""

import math
import random
import argparse

try:
    import matplotlib.pyplot as plt   # opcional: solo para los plots
except ImportError:
    plt = None

from attabot_sim import (
    SimRobot, SimWorld, ReactiveNav, normalize_angle,
    CENTER_TO_WHEEL, ARUCO_POS_SIGMA, ARUCO_ANGLE_SIGMA,
    SEGMENT_DISTANCE, ARRIVAL_THRESHOLD, AVOID_FRONT_ANGLE, AVOID_SIDE_ANGLE,
    AVOID_SEGMENT, MOTOR_DRIFT_SIGMA,
)

# ── Ruido de sensores a bordo (a calibrar en lab; valores conservadores) ─────
# Componente ALEATORIA (independiente por medición):
ENC_DIST_SIGMA  = 0.02    # fracción de la distancia medida por encoders
GYRO_TURN_SIGMA = 0.3     # grados de ruido por giro medido
GYRO_DRIFT_SIGMA = 0.05   # grados de deriva por sub-paso de avance (20mm)
# Componente SISTEMÁTICA (sesgo fijo por corrida — calibración residual real):
ENC_SCALE_BIAS_SIGMA  = 0.010  # sesgo de escala de encoders (PPR/diámetro rueda)
GYRO_SCALE_BIAS_SIGMA = 0.005  # sesgo de escala del gyro post yaw_scale

SUBSTEP = 20.0            # mm por sub-paso de MOVE (igual que la sim base)


# ── Álgebra 3x3 explícita (portable a C++) ───────────────────────────────────
def mat3_mult(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def mat3_add(A, B):
    return [[A[i][j] + B[i][j] for j in range(3)] for i in range(3)]


def mat3_transpose(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def mat3_inverse(A):
    a, b, c = A[0]; d, e, f = A[1]; g, h, i = A[2]
    det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)
    return [[(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det],
            [(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det],
            [(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det]]


def wrap_rad(a):
    while a > math.pi:  a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a


# ── EKF [x, y, θ] — espejo del futuro EKFState en utils.h ────────────────────
class EKF:
    def __init__(self, x, y, theta_deg):
        self.x = x
        self.y = y
        self.th = math.radians(theta_deg)
        # P inicial: confianza media en la pose de arranque (viene de ArUco)
        self.P = [[ARUCO_POS_SIGMA**2, 0, 0],
                  [0, ARUCO_POS_SIGMA**2, 0],
                  [0, 0, math.radians(ARUCO_ANGLE_SIGMA)**2]]
        # R ArUco: jitter medido en lab (±30mm, ±2°)
        self.R = [[ARUCO_POS_SIGMA**2, 0, 0],
                  [0, ARUCO_POS_SIGMA**2, 0],
                  [0, 0, math.radians(ARUCO_ANGLE_SIGMA)**2]]

    def predict(self, d, dth_deg):
        """Propaga con odometría: d = mm de encoders, dth = Δθ del gyro."""
        dth = math.radians(dth_deg)
        c, s = math.cos(self.th), math.sin(self.th)   # θ previo — también para F
        self.x += d * c
        self.y += d * s
        self.th = wrap_rad(self.th + dth)

        # Jacobiano del modelo respecto al estado (evaluado en θ previo)
        F = [[1, 0, -d * s],
             [0, 1,  d * c],
             [0, 0, 1]]
        # Ruido de proceso escalado al movimiento del paso
        sd  = ENC_DIST_SIGMA * abs(d) + 0.5
        sth = math.radians(GYRO_DRIFT_SIGMA + GYRO_SCALE_BIAS_SIGMA * abs(dth_deg))
        Q = [[sd**2, 0, 0], [0, sd**2, 0], [0, 0, sth**2]]
        self.P = mat3_add(mat3_mult(mat3_mult(F, self.P), mat3_transpose(F)), Q)

    def update_aruco(self, zx, zy, zth_deg):
        """Corrección con pose ArUco. H = I, así que K = P(P+R)^-1."""
        nu = [zx - self.x,
              zy - self.y,
              wrap_rad(math.radians(zth_deg) - self.th)]
        S = mat3_add(self.P, self.R)
        K = mat3_mult(self.P, mat3_inverse(S))
        self.x += K[0][0] * nu[0] + K[0][1] * nu[1] + K[0][2] * nu[2]
        self.y += K[1][0] * nu[0] + K[1][1] * nu[1] + K[1][2] * nu[2]
        self.th = wrap_rad(self.th + K[2][0] * nu[0] + K[2][1] * nu[1] + K[2][2] * nu[2])
        I_K = [[(1 if i == j else 0) - K[i][j] for j in range(3)] for i in range(3)]
        self.P = mat3_mult(I_K, self.P)

    def pose(self):
        return self.x, self.y, math.degrees(self.th) % 360


# ── Navegación usando la pose del EKF (el firmware futuro) ───────────────────
class EKFNav(ReactiveNav):
    """ReactiveNav.step pero alimentado con una pose externa (la del EKF)."""

    def step_pose(self, pose, ir):
        px, py, pangle = pose
        if self.has_reached(px, py):
            self.is_active = False
            return None
        dx = self.goal_x - px
        dy = self.goal_y - py
        dist = math.sqrt(dx * dx + dy * dy)
        goal_angle = math.degrees(math.atan2(dy, dx)) % 360

        bias, avoiding = 0.0, False
        if ir['front']:
            rel_goal = normalize_angle(goal_angle - pangle)
            bias = -AVOID_FRONT_ANGLE if rel_goal >= 0 else AVOID_FRONT_ANGLE
            avoiding = True
        elif ir['right']:
            bias, avoiding = -AVOID_SIDE_ANGLE, True   # derecha → girar izquierda
        elif ir['left']:
            bias, avoiding = AVOID_SIDE_ANGLE, True    # izquierda → girar derecha

        final_angle = (goal_angle + bias) % 360
        angle_diff = normalize_angle(final_angle - pangle)
        seg = AVOID_SEGMENT if avoiding else min(dist * 0.9, SEGMENT_DISTANCE)
        seg = max(10.0, seg)
        self.steps += 1
        return math.radians(angle_diff) * CENTER_TO_WHEEL, seg


# ── Corrida instrumentada ────────────────────────────────────────────────────
def run_ekf(robot, nav, world, occluded, use_ekf_nav=True, max_steps=60):
    """
    Ejecuta un GT generando la telemetría a bordo:
      TURN → gyro mide el giro real (+ruido/escala) → ekf.predict(0, dθ)
      MOVE → por sub-paso de 20mm: encoders miden d (+ruido) → ekf.predict(d, dθ_drift)
      Fin de segmento → si la cámara NO está ocluida: ekf.update_aruco(pose ruidosa)
    `occluded(step)` decide si la cámara ve al robot en ese segmento.
    `use_ekf_nav=False` = firmware actual: navega con ArUco crudo y ESPERA si
    no hay cámara (baseline).
    Devuelve historial para métricas/plots.
    """
    ekf = EKF(*robot.get_pose_noisy())
    odo = EKF(ekf.x, ekf.y, math.degrees(ekf.th))   # dead-reckoning puro (nunca update)
    # Sesgos sistemáticos de ESTA corrida (calibración residual del robot)
    enc_bias = random.gauss(0, ENC_SCALE_BIAS_SIGMA)
    gyro_bias = random.gauss(0, GYRO_SCALE_BIAS_SIGMA)
    hist = {'truth': [], 'ekf': [], 'odo': [], 'aruco': [], 'occl': [],
            'err_ekf': [], 'err_odo': []}
    arrived = False

    for step in range(max_steps):
        cam = not occluded(step)
        if use_ekf_nav:
            pose = ekf.pose()
        else:
            if not cam:                      # firmware actual sin cámara: espera
                hist['occl'].append(step)
                continue
            pose = robot.get_pose_noisy()

        result = nav.step_pose(pose, world.check_ir(robot))
        if result is None:
            arrived = True
            break
        arc, seg = result

        # TURN — el gyro mide el giro que realmente ocurrió
        if abs(arc) > math.radians(5) * CENTER_TO_WHEEL:
            pre = robot.angle
            robot.turn(arc)
            turned = normalize_angle(robot.angle - pre)
            dth = turned * (1 + gyro_bias) + random.gauss(0, GYRO_TURN_SIGMA)
            ekf.predict(0, dth)
            odo.predict(0, dth)

        # MOVE — encoders por sub-paso
        actual = seg * (1 + random.gauss(0, MOTOR_DRIFT_SIGMA))
        rad = math.radians(robot.angle)
        traveled = 0.0
        while traveled < actual:
            d = min(SUBSTEP, actual - traveled)
            robot.x += math.cos(rad) * d
            robot.y += math.sin(rad) * d
            traveled += d
            robot.trajectory.append((robot.x, robot.y))
            d_meas = d * (1 + enc_bias + random.gauss(0, ENC_DIST_SIGMA))
            dth_meas = random.gauss(0, GYRO_DRIFT_SIGMA)
            ekf.predict(d_meas, dth_meas)
            odo.predict(d_meas, dth_meas)

        # Fin de segmento — corrección ArUco si hay cámara
        if cam:
            zx, zy, zth = robot.get_pose_noisy()
            ekf.update_aruco(zx, zy, zth)
            hist['aruco'].append((zx, zy))
        else:
            hist['occl'].append(step)

        ex, ey, _ = ekf.pose()
        ox, oy, _ = odo.pose()
        hist['truth'].append((robot.x, robot.y))
        hist['ekf'].append((ex, ey))
        hist['odo'].append((ox, oy))
        hist['err_ekf'].append(math.hypot(ex - robot.x, ey - robot.y))
        hist['err_odo'].append(math.hypot(ox - robot.x, oy - robot.y))

    dx, dy = nav.goal_x - robot.x, nav.goal_y - robot.y
    hist['arrived'] = arrived
    hist['final_dist'] = math.hypot(dx, dy)
    return hist


# ── Escenarios ───────────────────────────────────────────────────────────────
START = (200.0, 200.0, 30.0)
GOAL = (1500.0, 1000.0)


def scenario_oclusion(seed=7, plot=True):
    """GT con la cámara ocluida en los segmentos 3-6 (≈1 tramo largo sin ver)."""
    random.seed(seed)
    robot = SimRobot(*START)
    nav = EKFNav(); nav.start(*GOAL)
    world = SimWorld()
    world.robots.append(robot)
    hist = run_ekf(robot, nav, world, occluded=lambda s: 3 <= s <= 6)

    print(f"llegó={hist['arrived']}  dist_final={hist['final_dist']:.0f}mm")
    print(f"error EKF  máx={max(hist['err_ekf']):.0f}mm  "
          f"final={hist['err_ekf'][-1]:.0f}mm")
    print(f"error ODO  máx={max(hist['err_odo']):.0f}mm  "
          f"final={hist['err_odo'][-1]:.0f}mm")
    if plot:
        plot_run(hist, "GT con oclusión de cámara (segmentos 3-6)",
                 "sim_output_ekf_oclusion.png")
    return hist


def scenario_ciego(seed=7, plot=True):
    """La cámara muere a mitad del camino: el EKF debe terminar el GT a ciegas.
    Baseline (firmware actual) con la misma semilla: se queda esperando."""
    random.seed(seed)
    robot = SimRobot(*START)
    nav = EKFNav(); nav.start(*GOAL)
    world = SimWorld(); world.robots.append(robot)
    hist = run_ekf(robot, nav, world, occluded=lambda s: s >= 4)

    random.seed(seed)
    robot_b = SimRobot(*START)
    nav_b = EKFNav(); nav_b.start(*GOAL)
    world_b = SimWorld(); world_b.robots.append(robot_b)
    base = run_ekf(robot_b, nav_b, world_b, occluded=lambda s: s >= 4,
                   use_ekf_nav=False)

    print(f"EKF:      llegó={hist['arrived']}  dist_final={hist['final_dist']:.0f}mm"
          f"  (error EKF final {hist['err_ekf'][-1]:.0f}mm)")
    print(f"baseline: llegó={base['arrived']}  dist_final={base['final_dist']:.0f}mm"
          f"  ← firmware actual: espera cámara para siempre")
    if plot:
        plot_run(hist, "Cámara muere en el segmento 4 — EKF termina a ciegas",
                 "sim_output_ekf_ciego.png")
    return hist, base


def scenario_montecarlo(n=30):
    """Estadística de estimación con oclusión intermitente."""
    err_ekf_max, err_ekf_fin, err_odo_max, finals, arrived = [], [], [], [], 0
    for i in range(n):
        random.seed(100 + i)
        robot = SimRobot(*START)
        nav = EKFNav(); nav.start(*GOAL)
        world = SimWorld(); world.robots.append(robot)
        h = run_ekf(robot, nav, world,
                    occluded=lambda s: s % 4 == 2)   # cámara cae 1 de cada 4 segmentos
        err_ekf_max.append(max(h['err_ekf']))
        err_ekf_fin.append(h['err_ekf'][-1])
        err_odo_max.append(max(h['err_odo']))
        finals.append(h['final_dist'])
        arrived += h['arrived']

    def stats(v):
        m = sum(v) / len(v)
        return f"media={m:.0f}mm  máx={max(v):.0f}mm"

    print(f"n={n}  llegadas={arrived}/{n}  (oclusión 25% de los segmentos)")
    print(f"  error EKF máximo por corrida:  {stats(err_ekf_max)}")
    print(f"  error EKF al llegar:           {stats(err_ekf_fin)}")
    print(f"  error ODO máximo por corrida:  {stats(err_odo_max)}  ← sin ArUco")
    print(f"  distancia final al goal:       {stats(finals)}")


# ── Plot (convención del repo: sim_output_*.png, ignorado por git) ───────────
C_EKF, C_ODO, C_TRUTH, C_MEAS = "#4269d0", "#efb118", "#555555", "#aaaaaa"


def plot_run(hist, title, outfile):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))
    fig.suptitle(title)

    tx, ty = zip(*hist['truth'])
    ex, ey = zip(*hist['ekf'])
    ox, oy = zip(*hist['odo'])
    ax1.plot(tx, ty, '--', color=C_TRUTH, lw=2, label='real (ground truth)')
    ax1.plot(ex, ey, '-', color=C_EKF, lw=2, label='EKF')
    ax1.plot(ox, oy, '-', color=C_ODO, lw=2, label='odometría pura')
    if hist['aruco']:
        mx, my = zip(*hist['aruco'])
        ax1.plot(mx, my, 'o', color=C_MEAS, ms=4, label='mediciones ArUco')
    ax1.plot(*GOAL, 'x', color=C_TRUTH, ms=10, mew=2)
    ax1.annotate('goal', GOAL, textcoords='offset points', xytext=(8, 8))
    ax1.set_xlabel('x (mm)'); ax1.set_ylabel('y (mm)')
    ax1.invert_yaxis()   # Y hacia abajo, como la cámara
    ax1.set_aspect('equal'); ax1.legend(frameon=False)
    ax1.grid(True, alpha=0.25)

    steps = range(len(hist['err_ekf']))
    ax2.plot(steps, hist['err_ekf'], '-', color=C_EKF, lw=2, label='EKF')
    ax2.plot(steps, hist['err_odo'], '-', color=C_ODO, lw=2, label='odometría pura')
    for s in hist['occl']:
        ax2.axvspan(s - 0.5, s + 0.5, color='#dddddd', zorder=0)
    ax2.axvspan(0, 0, color='#dddddd', label='cámara ocluida')  # entrada de leyenda
    ax2.set_xlabel('segmento'); ax2.set_ylabel('error de estimación (mm)')
    ax2.legend(frameon=False); ax2.grid(True, alpha=0.25)

    fig.tight_layout()
    fig.savefig(outfile, dpi=120)
    print(f"plot → {outfile}")


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', default='oclusion',
                    choices=['oclusion', 'ciego', 'montecarlo'])
    a = ap.parse_args()
    {'oclusion': scenario_oclusion,
     'ciego': scenario_ciego,
     'montecarlo': scenario_montecarlo}[a.scenario]()
