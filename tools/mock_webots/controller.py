"""Mock del módulo `controller` de Webots — mini-física cinemática para validar
attabot_firmware.py sin el Webots real. Diferencial ideal + ruido pequeño."""
import math
import os
import random
import time

WHEEL_R = 0.02225   # m
AXLE = 0.0415       # m (centro a rueda)
TIMESTEP_MS = 8
INSTANCES = []      # registro para que el harness alcance la física


class _Motor:
    def __init__(self):
        self.velocity = 0.0
    def setPosition(self, p):
        pass
    def setVelocity(self, v):
        self.velocity = v


class _Encoder:
    def __init__(self):
        self.value = 0.0
    def enable(self, ts):
        pass
    def getValue(self):
        return self.value


class _Gyro:
    def __init__(self):
        self.wz = 0.0
    def enable(self, ts):
        pass
    def getValues(self):
        return [0.0, 0.0, self.wz]


class _IR:
    def __init__(self):
        self.value = 0.25
    def enable(self, ts):
        pass
    def getValue(self):
        return self.value


class Robot:
    """Mundo Webots ENU (x adelante, z arriba). Arranca en la pose equivalente
    a cámara (300,300,0°), como el .wbt."""

    def __init__(self):
        self.wx = float(os.environ.get('MOCK_X', -0.9))
        self.wy = float(os.environ.get('MOCK_Y', 0.475))
        self.phi = float(os.environ.get('MOCK_PHI', 0.0))
        self.t = 0.0
        self.devices = {
            'left wheel motor': _Motor(), 'right wheel motor': _Motor(),
            'left wheel sensor': _Encoder(), 'right wheel sensor': _Encoder(),
            'gyro': _Gyro(),
            'ir front': _IR(), 'ir left': _IR(), 'ir right': _IR(),
        }
        INSTANCES.append(self)

    def getBasicTimeStep(self):
        return TIMESTEP_MS

    def getCustomData(self):
        return os.environ.get('MOCK_ID', '1')

    def getName(self):
        return f"Atta_{self.getCustomData()}"

    def getTime(self):
        return self.t

    def getDevice(self, name):
        return self.devices[name]

    def step(self, ms):
        time.sleep(0.0005)   # ~30x tiempo real: deja respirar al hilo de la base
        dt = ms / 1000.0
        self.t += dt
        vl = self.devices['left wheel motor'].velocity * WHEEL_R
        vr = self.devices['right wheel motor'].velocity * WHEEL_R
        v = (vl + vr) / 2.0
        w = (vr - vl) / (2.0 * AXLE)   # rueda izq en +y: CCW si derecha más rápida
        v *= 1 + random.gauss(0, 0.01)
        w *= 1 + random.gauss(0, 0.01)
        self.wx += v * math.cos(self.phi) * dt
        self.wy += v * math.sin(self.phi) * dt
        self.phi += w * dt
        self.devices['left wheel sensor'].value += \
            self.devices['left wheel motor'].velocity * dt
        self.devices['right wheel sensor'].value += \
            self.devices['right wheel motor'].velocity * dt
        self.devices['gyro'].wz = w
        if self.t > float(os.environ.get('MOCK_TIMEOUT', 180)):
            return -1
        return 0

    def camera_pose(self):
        """Pose real en marco cámara (igual conversión que base_camera.py)."""
        x = (self.wx + 2.4 / 2) * 1000
        y = (1.55 / 2 - self.wy) * 1000
        ang = (-math.degrees(self.phi)) % 360
        return x, y, ang
