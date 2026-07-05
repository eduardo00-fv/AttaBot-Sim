"""Harness: corre attabot_firmware.py contra el mock de Webots + una base
falsa por UDP real. Valida el ciclo GT completo (protocolo, TURN closed-loop,
MOVE, EKF innovación) sin abrir Webots. PASS si llega a <80mm del goal."""
import math
import os
import random
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))   # raíz de AttaBot-Sim
sys.path.insert(0, HERE)                                       # mock controller
sys.path.insert(1, os.path.join(REPO, 'webots', 'controllers', 'attabot_firmware'))
os.environ.setdefault('MOCK_ID', '1')

import controller                    # noqa: E402  (el mock)
import attabot_firmware as fw        # noqa: E402

GOAL = (1800.0, 1000.0)

firmware = fw.AttabotFirmware()
robot = controller.INSTANCES[-1]
threading.Thread(target=firmware.run, daemon=True).start()

base = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
base.bind(('127.0.0.1', 6060))
base.settimeout(0.2)

time.sleep(0.5)
base.sendto(f'POSITIONGT|{GOAL[0]:.0f}|{GOAL[1]:.0f}'.encode(), ('127.0.0.1', 6061))

arrived = False
innovs = []
deadline = time.time() + 40
while time.time() < deadline:
    try:
        data, addr = base.recvfrom(512)
    except socket.timeout:
        continue
    msg = data.decode()
    if msg == 'REQUEST_POSITION':
        x, y, ang = robot.camera_pose()
        x += random.gauss(0, 30)
        y += random.gauss(0, 30)
        ang = (ang + random.gauss(0, 2)) % 360
        base.sendto(f'POSITION_RESPONSE|{x:.1f}|{y:.1f}|{ang:.1f}'.encode(), addr)
    else:
        print('[base]', msg)
        if 'EKF innov=' in msg:
            innovs.append(float(msg.split('innov=')[1].split('mm')[0]))
        if 'llegó' in msg:
            arrived = True
            break

x, y, ang = robot.camera_pose()
dist = math.hypot(x - GOAL[0], y - GOAL[1])
print(f'\nRESULT: arrived={arrived} pose_real=({x:.0f},{y:.0f}) '
      f'dist_al_goal={dist:.0f}mm innovs={["%.0f" % i for i in innovs]}')
ok = arrived and dist < 80
print('PASS' if ok else 'FAIL')
sys.exit(0 if ok else 1)
