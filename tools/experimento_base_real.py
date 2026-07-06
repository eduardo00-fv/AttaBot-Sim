#!/usr/bin/env python3
"""
experimento_base_real.py — E2E: la base REAL del lab controlando Webots
=========================================================================
Arranca AttaBot_Base.py --sim --headless (la base de verdad, la misma que corre
en el lab), después Webots headless en tiempo real, y ejecuta un escenario:

  1. GT del robot 1 a (1800,1000), con una oclusión de cámara de 6s en medio
  2. Congregación con líder 1 (el robot 2 hace staging + parking)
  3. BREAK y verificación: PositionLog/ConsoleLog en formato lab generados
     por la base real desde la simulación

Orden de arranque OBLIGATORIO: base primero (toma 127.0.0.1:6060), Webots
después (base_camera.py encuentra el puerto tomado y entra en modo solo-cámara).

Uso:
    python tools/experimento_base_real.py [--base-dir ~/Documents/Atta-Bot-P_ed/Base]
"""

import argparse
import csv
import glob
import os
import select
import subprocess
import sys
import threading
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(REPO, 'webots', 'worlds', 'attabot.wbt')
DEFAULT_BASE = os.path.expanduser('~/Documents/Atta-Bot-P_ed/Base')
RAW_LOG = '/tmp/experimento_base_real.log'


class Tail:
    """Lee el stdout de un proceso en un hilo; permite esperar por substrings."""

    def __init__(self, proc, tag, raw):
        self.proc = proc
        self.tag = tag
        self.raw = raw
        self.lines = []
        self.lock = threading.Lock()
        t = threading.Thread(target=self._pump, daemon=True)
        t.start()

    def _pump(self):
        for line in self.proc.stdout:
            with self.lock:
                self.lines.append(line.rstrip())
            self.raw.write(f'[{self.tag}] {line}')
            self.raw.flush()

    def wait_for(self, needle, timeout, start=0):
        """Espera a que aparezca `needle` desde el índice `start`.
        Retorna el índice de línea siguiente o None si venció el timeout."""
        deadline = time.time() + timeout
        idx = start
        while time.time() < deadline:
            with self.lock:
                n = len(self.lines)
                for i in range(idx, n):
                    if needle in self.lines[i]:
                        return i + 1
                idx = n
            if self.proc.poll() is not None:
                return None
            time.sleep(0.3)
        return None


def newest(pattern):
    files = glob.glob(pattern)
    return max(files, key=os.path.getmtime) if files else None


def kill_webots():
    subprocess.run(['flatpak', 'kill', 'com.cyberbotics.webots'],
                   stderr=subprocess.DEVNULL)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base-dir', default=DEFAULT_BASE,
                    help='carpeta Base/ del repo del robot (AttaBot_Base.py)')
    a = ap.parse_args()

    env = dict(os.environ)
    env.setdefault('XAUTHORITY', os.path.expanduser('~/.Xauthority'))
    env.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')
    env.setdefault('DISPLAY', ':0')

    kill_webots()
    time.sleep(2)
    raw = open(RAW_LOG, 'w')
    t0 = time.time()

    # ── 1. Base real primero (toma el puerto 6060) ──────────────────────────
    base = subprocess.Popen(
        [sys.executable, '-u', 'AttaBot_Base.py', '--sim', '--headless'],
        cwd=a.base_dir, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, env=env)
    base_tail = Tail(base, 'base', raw)

    def console(cmd):
        base.stdin.write(cmd + '\n')
        base.stdin.flush()
        raw.write(f'>>> {cmd}\n')
        print(f'[exp] consola: {cmd}', flush=True)

    console('2')   # cantidad de robots

    if base_tail.wait_for('bind exitoso', 15) is None:
        print('[exp] FATAL: la base no tomó el puerto 6060 (¿instancia previa?)')
        base.terminate()
        return 1
    print('[exp] base real lista en :6060', flush=True)

    # ── 2. Webots después (base_camera cae en modo solo-cámara) ─────────────
    webots = subprocess.Popen(
        ['flatpak', 'run', '--filesystem=home', 'com.cyberbotics.webots',
         '--no-rendering', '--batch', '--minimize', '--mode=realtime',
         '--stdout', '--stderr', WORLD],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    webots_tail = Tail(webots, 'webots', raw)

    try:
        if webots_tail.wait_for('SOLO-CÁMARA', 60) is None:
            print('[exp] FATAL: base_camera no entró en modo solo-cámara')
            return 1
        print('[exp] supervisor en modo solo-cámara', flush=True)

        # La base debe encontrar los 2 robots por el feed CAM y asignarles puerto
        if base_tail.wait_for('Robots encontrados', 60) is None:
            print('[exp] FATAL: la base no completó el setup de robots')
            return 1
        print('[exp] setup completo — robots asociados', flush=True)
        time.sleep(4)   # TURN|-90 de printRobots

        # ── 3. Escenario: GT + oclusión ──────────────────────────────────────
        mark = len(base_tail.lines)
        console('GOTO.1 1800 1000')
        time.sleep(8)
        console('OCCLUDE.6')

        idx = base_tail.wait_for('NAV: llegó', 180, start=mark)
        if idx is None:
            print('[exp] FATAL: el robot 1 no llegó al goal en 180s')
            return 1
        print(f'[exp] ✓ GT completado (t={time.time()-t0:.0f}s)', flush=True)

        # ── 4. Congregación (robot 2 → staging → slot) ───────────────────────
        console('CONGREGATION.1')
        idx = base_tail.wait_for('staging listo', 120, start=idx)
        if idx is None:
            print('[exp] FATAL: el follower no completó el staging')
            return 1
        idx = base_tail.wait_for('NAV: llegó', 120, start=idx)
        if idx is None:
            print('[exp] FATAL: el follower no llegó al slot')
            return 1
        print(f'[exp] ✓ congregación completada (t={time.time()-t0:.0f}s)', flush=True)

        console('BREAK')
        time.sleep(3)
    finally:
        kill_webots()
        base.terminate()
        try:
            base.wait(timeout=10)
        except subprocess.TimeoutExpired:
            base.kill()
        raw.close()

    # ── 5. Verificación de logs en formato lab ──────────────────────────────
    print('\n=== VERIFICACIÓN DE LOGS ===')
    ok = True
    pos_log = newest(os.path.join(a.base_dir, 'PositionLogs', 'Position_Log_SIM_*.csv'))
    con_log = newest(os.path.join(a.base_dir, 'ConsoleLogs', 'Console_Log_SIM_*.csv'))

    if pos_log:
        with open(pos_log) as f:
            rows = list(csv.DictReader(f))
        ids = sorted({r['idrobot'] for r in rows})
        print(f'PositionLog: {os.path.basename(pos_log)} — {len(rows)} filas, robots {ids}')
        ok &= len(rows) > 50 and {'1', '2'} <= set(ids)
    else:
        print('✗ No se generó Position_Log_SIM_*.csv')
        ok = False

    if con_log:
        with open(con_log) as f:
            msgs = [r['message'] for r in csv.DictReader(f)]
        arrivals = sum('NAV: llegó' in m for m in msgs)
        innovs = sum('EKF innov' in m for m in msgs)
        print(f'ConsoleLog: {os.path.basename(con_log)} — {len(msgs)} mensajes, '
              f'{arrivals} llegadas, {innovs} innovaciones EKF')
        ok &= arrivals >= 2 and innovs > 0
    else:
        print('✗ No se generó Console_Log_SIM_*.csv')
        ok = False

    print(f'\n{"✓ E2E EXITOSO" if ok else "✗ E2E FALLÓ"} — log crudo en {RAW_LOG}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
