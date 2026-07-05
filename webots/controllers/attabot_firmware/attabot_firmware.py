"""
attabot_firmware.py — "firmware" del AttaBot para Webots
=========================================================
Réplica funcional del firmware ESP32 sobre la física de Webots, hablando el
MISMO protocolo UDP del lab (puerto base 6060; cada robot escucha en 6060+id).

Reusa los ports ya validados de la sim 2D:
  - EKFNav (ReactiveNav con pose externa)  ← sim/ekf_sim.py
  - EKF [x,y,θ]                            ← sim/ekf_sim.py (espejo de utils.h)

Sensores desde la física: encoders = PositionSensor de las ruedas,
gyro = Gyro (se integra a yaw como el DMP), IR = DistanceSensor.

Marco de coordenadas: el "de cámara" del lab (x→ancho, y→abajo, ángulo CW).
La conversión con el mundo Webots ocurre solo en dos puntos:
  - yaw: dθ_cam = −ω_z·dt (el gyro de Webots es CCW+, la cámara CW+)
  - la pose absoluta solo la conoce la base virtual (base_camera.py)

Comandos soportados: MOVE|mm, TURN|deg, POSITIONGT|x|y, ABORT_NAV,
CONGREGATION|líder|idx|n, CANCEL_CONGREGATION, LEADER_POSITION|id|x|y|θ,
POSITION_RESPONSE|x|y|θ, GET_STATUS, EKF_NAV|0/1 (conmuta la nav a pose EKF),
NAV_CONFIG|PARKING_DIST|mm.
"""

import json
import math
import os
import socket
import sys

from controller import Robot

sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'sim'))
from ekf_sim import EKF, EKFNav  # noqa: E402

# ── Constantes físicas (mismas del firmware/PROTO) ───────────────────────────
WHEEL_RADIUS_MM = 22.25
CENTER_TO_WHEEL = 41.5
IR_THRESHOLD_M  = 0.20      # obstáculo si el IR reporta menos de esto

MOVE_SPEED   = 8.0          # rad/s de rueda (~178 mm/s)
TURN_SPEED   = 2.5          # rad/s de rueda (~77 °/s de giro)
TURN_LEAD    = 3.0          # ° de brake-lead (como el firmware)
TURN_TOL     = 3.0          # ° tolerancia final
TURN_MAX_COR = 4            # correcciones iterativas máximas
SETTLE_MS    = 300

BASE_ADDR = ('127.0.0.1', 6060)
PROFILES_PATH = os.path.join(os.path.dirname(__file__), '..', '..',
                             'robot_profiles.json')


def load_profile(robot_id):
    """Personalidad del robot real (ver robot_profiles.json). Sin perfil = ideal."""
    try:
        with open(PROFILES_PATH) as f:
            p = json.load(f).get(robot_id, {})
    except (FileNotFoundError, json.JSONDecodeError):
        p = {}
    return {
        'gyro_scale':    p.get('gyro_scale', 1.0),
        'yaw_scale_cal': p.get('yaw_scale_cal', 1.0),
        'enc_scale':     p.get('enc_scale', 1.0),
        'motor_bias':    p.get('motor_bias', 1.0),
    }


class AttabotFirmware:
    def __init__(self):
        self.robot = Robot()
        self.dt = int(self.robot.getBasicTimeStep()) * 2   # 16 ms
        self.robot_id = self.robot.getCustomData() or '1'
        self.name = self.robot.getName()
        self.profile = load_profile(self.robot_id)

        self.left = self.robot.getDevice('left wheel motor')
        self.right = self.robot.getDevice('right wheel motor')
        for m in (self.left, self.right):
            m.setPosition(float('inf'))
            m.setVelocity(0)
        self.enc_l = self.robot.getDevice('left wheel sensor')
        self.enc_r = self.robot.getDevice('right wheel sensor')
        self.gyro = self.robot.getDevice('gyro')
        self.irs = {k: self.robot.getDevice(f'ir {k}') for k in ('front', 'left', 'right')}
        for d in [self.enc_l, self.enc_r, self.gyro, *self.irs.values()]:
            d.enable(self.dt)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.sock.bind(('127.0.0.1', 6060 + int(self.robot_id)))
        except OSError:
            sys.exit(f'[{self.name}] puerto {6060 + int(self.robot_id)} ocupado — '
                     '¿hay OTRO Webots abierto con este mundo? Cerralo '
                     '(flatpak kill com.cyberbotics.webots) y hacé Reset (⏮).')
        self.sock.setblocking(False)

        # Estado de movimiento
        self.state = 'IDLE'          # IDLE | TURN | MOVE | SETTLE
        self.queue = []              # [('TURN', deg) | ('MOVE', mm)]
        self.yaw = 0.0               # ° marco cámara (integrado del gyro)
        self.prev_enc = (0.0, 0.0)
        self.turn_target = 0.0
        self.turn_acc = 0.0
        self.turn_corr = 0
        self.settle_until = 0
        self.move_target = 0.0
        self.move_acc = 0.0
        self.move_sign = 1

        # Navegación / EKF
        self.nav = EKFNav()
        self.ekf = EKF(0, 0, 0)
        self.ekf.initialized = False
        self.ekf_nav = False         # True → la nav usa la pose del EKF
        self.waiting_pos = False
        self.last_request = -1e9
        self.pose = None             # última pose ArUco recibida
        self.last_ekfpose = 0.0      # telemetría EKFPOSE periódica

        # Congregación (port del firmware nuevo: staging + slot del lado propio)
        self.cong_leader = None
        self.cong_idx = 0
        self.cong_n = 1
        self.parking_dist = 300.0
        self.slot = None
        self.slot_angle = None
        self.staging_done = False

    # ── Utilidades ───────────────────────────────────────────────────────────
    def now_ms(self):
        return self.robot.getTime() * 1000

    def send_base(self, text):
        self.sock.sendto(text.encode(), BASE_ADDR)

    def debug(self, text):
        msg = f'DEBUG: -1, ID: {self.robot_id}, {text}'
        print(msg)
        self.send_base(msg)

    def read_ir(self):
        return {k: (d.getValue() < IR_THRESHOLD_M) for k, d in self.irs.items()}

    def set_wheels(self, vl, vr):
        # Desbalance físico de motores del robot real (deriva en MOVE, giros
        # imperfectos). El closed-loop de TURN y las correcciones ArUco/EKF
        # lo compensan — igual que en el lab.
        b = self.profile['motor_bias']
        self.left.setVelocity(vl * b)
        self.right.setVelocity(vr / b)

    def stop_motors(self):
        self.set_wheels(0, 0)

    # ── Sensores → EKF (equivalente de EkfTick del firmware) ────────────────
    def sensor_tick(self):
        wz = self.gyro.getValues()[2]
        d_yaw = -math.degrees(wz) * (self.dt / 1000.0)   # CCW mundo → CW cámara
        d_yaw *= self.profile['gyro_scale']      # error físico del sensor
        d_yaw *= self.profile['yaw_scale_cal']   # corrección CALIBRATE (residuo real)
        self.yaw = (self.yaw + d_yaw) % 360

        enc_scale = self.profile['enc_scale']    # residuo de calibración PPR
        el, er = self.enc_l.getValue(), self.enc_r.getValue()
        d_l = (el - self.prev_enc[0]) * WHEEL_RADIUS_MM * enc_scale
        d_r = (er - self.prev_enc[1]) * WHEEL_RADIUS_MM * enc_scale
        self.prev_enc = (el, er)
        d = (d_l + d_r) / 2.0 if self.state == 'MOVE' else 0.0

        if self.state == 'TURN':
            self.turn_acc += d_yaw
        if self.state == 'MOVE':
            self.move_acc += abs(d)

        self.ekf.predict(d, d_yaw)
        return d_yaw

    # ── Máquina de estados de movimiento ─────────────────────────────────────
    def start_next(self):
        if not self.queue:
            self.state = 'IDLE'
            if self.nav.is_active and not self.waiting_pos:
                self.request_position()
            return
        kind, value = self.queue.pop(0)
        if kind in ('TURN', 'TURNC'):
            self.turn_target = value
            self.turn_acc = 0.0
            # correcciones sin brake-lead (fix validado en hardware 18/06:
            # el lead se comía las correcciones ≤3°)
            self.turn_lead = 0.0 if kind == 'TURNC' else TURN_LEAD
            self.state = 'TURN'
            s = TURN_SPEED if value >= 0 else -TURN_SPEED
            self.set_wheels(s, -s)
        else:  # MOVE
            self.move_target = abs(value)
            self.move_sign = 1 if value >= 0 else -1
            self.move_acc = 0.0
            self.state = 'MOVE'
            self.set_wheels(MOVE_SPEED * self.move_sign,
                            MOVE_SPEED * self.move_sign)

    def motion_tick(self):
        if self.state == 'TURN':
            remaining = abs(self.turn_target) - abs(self.turn_acc)
            if remaining <= self.turn_lead:
                self.stop_motors()
                self.settle_until = self.now_ms() + SETTLE_MS
                self.state = 'SETTLE'
        elif self.state == 'SETTLE':
            if self.now_ms() >= self.settle_until:
                err = self.turn_target - self.turn_acc
                if abs(err) > TURN_TOL and self.turn_corr < TURN_MAX_COR:
                    self.turn_corr += 1
                    self.debug(f'TURN corrección #{self.turn_corr}: {err:.1f}°')
                    self.queue.insert(0, ('TURNC', err))
                else:
                    self.debug(f'TURN IMU: objetivo={self.turn_target:.1f}° '
                               f'real={self.turn_acc:.1f}° err={err:.1f}° '
                               f'corr#{self.turn_corr}')
                    self.turn_corr = 0
                self.start_next()
        elif self.state == 'MOVE':
            ir = self.read_ir()
            if any(ir.values()):
                self.stop_motors()
                self.debug(f'MOVE interrumpido por IR {ir}')
                self.queue.clear()
                self.state = 'IDLE'
                if self.nav.is_active:
                    self.request_position()
                return
            if self.move_acc >= self.move_target:
                self.stop_motors()
                self.debug('Movimiento completado')
                self.start_next()

    # ── Navegación (ciclo GT del firmware) ───────────────────────────────────
    def request_position(self):
        self.waiting_pos = True
        self.last_request = self.now_ms()
        self.send_base('REQUEST_POSITION')

    def nav_step(self, pose):
        result = self.nav.step_pose(pose, self.read_ir())
        if result is None:
            # ¿Etapa de staging de congregación completada?
            if self.cong_leader and not self.staging_done and self.slot:
                self.staging_done = True
                self.nav.start(*self.slot)
                self.debug(f'staging listo, entrando al slot {self.slot}')
                self.nav_step(pose)
                return
            self.debug(f'NAV: llegó a ({pose[0]:.1f},{pose[1]:.1f})')
            return
        arc_mm, seg = result
        deg = math.degrees(arc_mm / CENTER_TO_WHEEL)
        self.queue = []
        if abs(deg) > 5:
            self.queue.append(('TURN', deg))
        self.queue.append(('MOVE', seg))
        self.start_next()

    # ── Congregación ─────────────────────────────────────────────────────────
    def on_leader_position(self, lx, ly):
        if self.slot_angle is None:
            if self.pose and self.cong_n == 1:
                self.slot_angle = math.atan2(self.pose[1] - ly, self.pose[0] - lx)
            else:
                self.slot_angle = 2 * math.pi * self.cong_idx / max(1, self.cong_n)
        a = self.slot_angle
        self.slot = (lx + self.parking_dist * math.cos(a),
                     ly + self.parking_dist * math.sin(a))
        goal_dist = self.parking_dist if self.staging_done else self.parking_dist + 150
        goal = (lx + goal_dist * math.cos(a), ly + goal_dist * math.sin(a))
        if not self.nav.is_active:
            self.nav.start(*goal)
            self.debug(f'CONGREGATION slot {self.cong_idx}/{self.cong_n} → '
                       f'{"parking" if self.staging_done else "staging"} '
                       f'({goal[0]:.0f},{goal[1]:.0f})')
            self.request_position()
        else:
            self.nav.goal_x, self.nav.goal_y = goal

    # ── Protocolo UDP ────────────────────────────────────────────────────────
    def handle(self, msg):
        parts = msg.split('|')
        cmd = parts[0]

        if cmd == 'POSITION_RESPONSE':
            self.waiting_pos = False
            x, y, ang = float(parts[1]), float(parts[2]), float(parts[3])
            self.pose = (x, y, ang)
            was_init = self.ekf.initialized
            if not was_init:
                self.ekf = EKF(x, y, ang)
                self.ekf.initialized = True
            else:
                innov = math.hypot(x - self.ekf.x, y - self.ekf.y)
                self.ekf.update_aruco(x, y, ang)
                self.debug(f'EKF innov={innov:.0f}mm '
                           f'est=({self.ekf.x:.0f},{self.ekf.y:.0f})')
            if self.cong_leader == self.robot_id:
                self.send_base(f'LEADER_POSITION|{self.robot_id}|{x:.1f}|{y:.1f}|{ang:.1f}')
            if self.nav.is_active and self.state == 'IDLE':
                self.nav_step(self.ekf.pose() if self.ekf_nav else self.pose)

        elif cmd in ('POSITIONGT', 'GT'):
            self.nav.start(float(parts[1]), float(parts[2]))
            self.debug(f'GT iniciado: goal=({parts[1]},{parts[2]})')
            self.request_position()

        elif cmd == 'MOVE':
            self.queue.append(('MOVE', float(parts[1])))
            if self.state == 'IDLE':
                self.start_next()

        elif cmd == 'TURN':
            self.queue.append(('TURN', float(parts[1])))
            if self.state == 'IDLE':
                self.start_next()

        elif cmd == 'CONGREGATION':
            self.cong_leader = parts[1]
            self.cong_idx = int(parts[2])
            self.cong_n = int(parts[3]) if len(parts) > 3 else 1
            self.slot = None
            self.slot_angle = None
            self.staging_done = False
            if self.cong_leader == self.robot_id:
                self.debug('Congregación: soy líder')
                self.request_position()
            else:
                self.debug(f'Congregación: slot {self.cong_idx}/{self.cong_n}')
                self.request_position()   # pose propia para el lado del slot

        elif cmd == 'LEADER_POSITION':
            if self.cong_leader and parts[1] == self.cong_leader \
               and parts[1] != self.robot_id:
                self.on_leader_position(float(parts[2]), float(parts[3]))

        elif cmd in ('CANCEL_CONGREGATION', 'ABORT_NAV', 'STOP'):
            self.cong_leader = None
            self.nav.is_active = False
            self.queue.clear()
            self.stop_motors()
            self.state = 'IDLE'
            self.debug(f'{cmd} ejecutado')

        elif cmd == 'EKF_NAV':
            self.ekf_nav = parts[1] == '1'
            self.debug(f'nav con pose {"EKF" if self.ekf_nav else "ArUco"}')

        elif cmd == 'NAV_CONFIG' and parts[1] == 'PARKING_DIST':
            self.parking_dist = float(parts[2])
            self.debug(f'parking={self.parking_dist:.0f}mm')

        elif cmd == 'GET_STATUS':
            e = self.ekf
            self.send_base(f'STATUS|ID:{self.robot_id}|State:{self.state}'
                           f'|NAV:{int(self.nav.is_active)}'
                           f'|Pos:{self.pose}|EKF:({e.x:.0f},{e.y:.0f},{e.pose()[2]:.0f})'
                           f'|Yaw:{self.yaw:.1f}')

    # ── Loop principal ───────────────────────────────────────────────────────
    def run(self):
        self.debug(f'{self.name} listo en puerto {6060 + int(self.robot_id)}')
        while self.robot.step(self.dt) != -1:
            self.sensor_tick()
            self.motion_tick()
            try:
                while True:
                    data, _ = self.sock.recvfrom(512)
                    self.handle(data.decode().strip())
            except BlockingIOError:
                pass
            # Cámara muda + EKF_NAV activo → seguir a ciegas con la pose EKF
            # (el comportamiento objetivo del GT semi-continuo del Bloque A)
            if self.nav.is_active and self.ekf_nav and self.waiting_pos \
               and self.state == 'IDLE' and self.ekf.initialized \
               and self.now_ms() - self.last_request > 1000:
                self.waiting_pos = False
                self.debug('sin cámara — continuando con pose EKF')
                self.nav_step(self.ekf.pose())
            # Reintento de posición (como el firmware, cada 3s)
            elif self.nav.is_active and self.waiting_pos \
                    and self.now_ms() - self.last_request > 3000:
                self.request_position()
            # Telemetría: pose EKF cada 1s mientras navega (para graficar)
            if self.nav.is_active and self.ekf.initialized \
               and self.now_ms() - self.last_ekfpose > 1000:
                self.last_ekfpose = self.now_ms()
                ex, ey, eth = self.ekf.pose()
                self.send_base(f'EKFPOSE|{self.robot_id}|{ex:.1f}|{ey:.1f}|{eth:.1f}')


if __name__ == '__main__':
    AttabotFirmware().run()
