"""
base_camera.py — Base virtual para Webots (supervisor)
=======================================================
Hace el papel de la cámara ArUco + base del lab, hablando el MISMO protocolo:

  - Responde REQUEST_POSITION de cada robot con POSITION_RESPONSE|x|y|θ en el
    marco "de cámara" del lab (origen esquina sup-izq, y hacia abajo, ángulo
    CW), con el jitter medido en el lab (σ=30mm, 2°).
  - Rebroadcast de LEADER_POSITION a todos los robots (como el broadcast UDP).
  - Imprime todo DEBUG/STATUS que envían los robots.
  - Log periódico de poses reales: línea "POS|id|x|y|θ" cada 2 s.

Consola: mandale UDP a 127.0.0.1:6060 con el formato del lab, p. ej.:
    echo -n "1.POSITIONGT|1800|1000" | nc -u -w0 127.0.0.1 6060
    echo -n "CONGREGATION.1"         | nc -u -w0 127.0.0.1 6060   (líder = 1)
    echo -n "OCCLUDE.10"             | nc -u -w0 127.0.0.1 6060   (cámara ciega 10s)
    echo -n "BROADCAST.EKF_NAV|1"    | nc -u -w0 127.0.0.1 6060
"""

import json
import math
import os
import random
import socket

from controller import Supervisor

ARENA_W, ARENA_H = 2.4, 1.55          # m (Webots), arena centrada en el origen
NOISE_POS, NOISE_ANG = 30.0, 2.0      # jitter ArUco genérico (mm, grados)

# Jitter POR ROBOT: la marca de cada robot real es distinta (robot_profiles.json)
try:
    with open(os.path.join(os.path.dirname(__file__), '..', '..',
                           'robot_profiles.json')) as f:
        PROFILES = json.load(f)
except (FileNotFoundError, json.JSONDecodeError):
    PROFILES = {}


def aruco_noise(rid):
    p = PROFILES.get(rid, {})
    return p.get('aruco_pos_sigma', NOISE_POS), p.get('aruco_ang_sigma', NOISE_ANG)

sup = Supervisor()
dt = int(sup.getBasicTimeStep()) * 2

# ── Descubrir los AttaBots del mundo ─────────────────────────────────────────
robots = {}   # id (str) → node
objects = []  # (color, node) — objetos consultables por COLOR_QUERY
COLOR_BY_NAME = {'obstacle box': 'café', 'target rojo': 'rojo'}
children = sup.getRoot().getField('children')
for i in range(children.getCount()):
    node = children.getMFNode(i)
    if node.getTypeName() == 'AttaBot':
        rid = node.getField('customData').getSFString()
        robots[rid] = node
    elif node.getTypeName() == 'Solid':
        name = node.getField('name').getSFString()
        for prefix, color in COLOR_BY_NAME.items():
            if name.startswith(prefix):
                objects.append((color, node))
print(f'[base] robots detectados: {sorted(robots)} · '
      f'objetos: {[c for c, _ in objects]}')


def camera_pose(node):
    """Pose real Webots → marco de cámara del lab (mm, grados CW, y abajo)."""
    px, py, _ = node.getPosition()
    R = node.getOrientation()
    phi = math.atan2(R[3], R[0])            # yaw mundo (CCW+)
    x = (px + ARENA_W / 2) * 1000
    y = (ARENA_H / 2 - py) * 1000
    ang = (-math.degrees(phi)) % 360
    return x, y, ang


sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind(('127.0.0.1', 6060))
except OSError:
    import sys
    sys.exit('[base] puerto 6060 ocupado — ¿hay OTRO Webots abierto con este '
             'mundo? Cerralo (flatpak kill com.cyberbotics.webots) y hacé Reset (⏮).')
sock.setblocking(False)

occluded_until = 0.0
last_pos_log = 0.0


def robot_addr(rid):
    return ('127.0.0.1', 6060 + int(rid))


def rid_from_port(port):
    rid = str(port - 6060)
    return rid if rid in robots else None


def handle_console(msg):
    global occluded_until
    target, _, instruction = msg.partition('.')
    if target == 'OCCLUDE':
        occluded_until = sup.getTime() + float(instruction)
        print(f'[base] cámara OCLUIDA por {instruction}s')
    elif target == 'CONGREGATION':
        leader = instruction.strip()
        followers = sorted(r for r in robots if r != leader)
        sock.sendto(f'CONGREGATION|{leader}|0|{len(followers)}'.encode(),
                    robot_addr(leader))
        for idx, rid in enumerate(followers):
            sock.sendto(f'CONGREGATION|{leader}|{idx}|{len(followers)}'.encode(),
                        robot_addr(rid))
        print(f'[base] congregación: líder {leader}, {len(followers)} seguidor(es)')
    elif target == 'BROADCAST':
        for rid in robots:
            sock.sendto(instruction.encode(), robot_addr(rid))
    elif target in robots:
        sock.sendto(instruction.encode(), robot_addr(target))
    else:
        print(f'[base] comando no reconocido: {msg}')


while sup.step(dt) != -1:
    now = sup.getTime()

    if occluded_until and now >= occluded_until:
        occluded_until = 0.0
        print(f'[base] cámara RESTAURADA (t={now:.1f}s)')

    # Log periódico de poses reales (para métricas/validación externa)
    if now - last_pos_log >= 0.5:
        last_pos_log = now
        for rid, node in sorted(robots.items()):
            x, y, ang = camera_pose(node)
            print(f'POS|{rid}|{x:.1f}|{y:.1f}|{ang:.1f}')

    try:
        while True:
            data, addr = sock.recvfrom(512)
            msg = data.decode().strip()
            sender = rid_from_port(addr[1])

            if sender is None:
                handle_console(msg)
            elif msg == 'REQUEST_POSITION':
                if now < occluded_until:
                    continue                     # la base calla si no "ve"
                x, y, ang = camera_pose(robots[sender])
                sp, sa = aruco_noise(sender)
                x += random.gauss(0, sp)
                y += random.gauss(0, sp)
                ang = (ang + random.gauss(0, sa)) % 360
                sock.sendto(f'POSITION_RESPONSE|{x:.1f}|{y:.1f}|{ang:.1f}'.encode(),
                            addr)
            elif msg == 'COLOR_QUERY':
                # APDS virtual: color del objeto a <250mm y ±45° del frente
                x, y, ang = camera_pose(robots[sender])
                best, best_d = 'nada', 1e9
                for color, node in objects:
                    ox, oy, _ = camera_pose(node)
                    d = math.hypot(ox - x, oy - y)
                    if d > 250 or d >= best_d:
                        continue
                    to_obj = math.degrees(math.atan2(oy - y, ox - x)) % 360
                    diff = abs(((to_obj - ang) + 180) % 360 - 180)
                    if diff < 45:
                        best, best_d = color, d
                if best == 'nada' and min(x, y, 2400 - x, 1550 - y) < 220:
                    best = 'pared'
                sock.sendto(f'COLOR_RESPONSE|{best}'.encode(), addr)
            elif msg.startswith('LEADER_POSITION'):
                for rid in robots:
                    if rid != sender:
                        sock.sendto(msg.encode(), robot_addr(rid))
            else:
                print(f'[{sender}] {msg}')
    except BlockingIOError:
        pass
