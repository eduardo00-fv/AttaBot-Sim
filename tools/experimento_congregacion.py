#!/usr/bin/env python3
"""
experimento_congregacion.py — CONGREGACIÓN CON OBSTÁCULOS (4 robots)
=====================================================================
El experimento del paper: la base REAL --sim controla Webots sobre
attabot_congreg.wbt. El líder (Atta_1) se queda en el centro y los 3 seguidores
se congregan a su anillo de 300mm, EVADIENDO por IR las 3 cajas que hay en el
camino (topología de obstáculos). Usa la CONGREGATION endurecida de la base
(slots wall-safe + anti-cruce). La métrica de aprobación es ground-truth (POS
del supervisor): cada seguidor a 300±120mm del líder.

La corrida genera Position_Log_SIM/Console_Log_SIM en formato lab; analizar con:
    python analyze_logs.py --congregation --session SIM_<...> --layout <...> --plot

Cambiar la topología = mover las 3 cajas en attabot_congreg.wbt (o pasar otro
--world). Reutiliza toda la plomería (Tail, kill_webots, arranque base→Webots)
de experimento_base_real.py.

Uso: python tools/experimento_congregacion.py [--layout 3_cajas] [--world ...]
"""

import argparse
import glob
import math
import os
import re
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from experimento_base_real import Tail, kill_webots, DEFAULT_BASE, WORLD  # noqa: E402

RAW_LOG = '/tmp/experimento_congregacion.log'
RE_POS = re.compile(r'^POS\|(\d+)\|(-?[\d.]+)\|(-?[\d.]+)\|(-?[\d.]+)')
CONGREG_WORLD = os.path.join(os.path.dirname(WORLD), 'attabot_congreg.wbt')
PARKING = 300.0    # radio del anillo de congregación (mm)
TOL = 120.0        # tolerancia de aprobación al radio (mm)


def latest_poses(tail):
    poses = {}
    with tail.lock:
        for line in tail.lines:
            m = RE_POS.match(line.strip())
            if m:
                poses[m.group(1)] = (float(m.group(2)), float(m.group(3)),
                                     float(m.group(4)))
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base-dir', default=DEFAULT_BASE)
    ap.add_argument('--world', default=CONGREG_WORLD)
    ap.add_argument('--layout', default='3_cajas', help='etiqueta de la topología')
    ap.add_argument('--leader', default='1')
    ap.add_argument('--robots', type=int, default=4)
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--gui', action='store_true',
                    help='abrir Webots con ventana y render (para MIRAR la corrida); '
                         'por defecto corre headless, que es más rápido')
    ap.add_argument('--movie', metavar='PATH',
                    help='graba la corrida en video (mp4, grabador nativo de Webots). '
                         'Implica --gui (grabar necesita render) y la ruta debe estar '
                         'bajo $HOME: el flatpak no ve /tmp')
    a = ap.parse_args()

    ids = [str(i) for i in range(1, a.robots + 1)]
    nf = a.robots - 1                       # nº de seguidores
    # Anillo esperado: espeja el auto-escalado de startCongregation (base).
    parking = max(PARKING, nf * 250.0 / (2 * math.pi))
    tol = max(TOL, 150.0)

    env = dict(os.environ)
    env.setdefault('XAUTHORITY', os.path.expanduser('~/.Xauthority'))
    env.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')
    env.setdefault('DISPLAY', ':0')
    env.setdefault('__GLX_VENDOR_LIBRARY_NAME', 'mesa')
    movie = None
    if a.movie:
        a.gui = True                     # el grabador captura la vista 3D
        movie = os.path.abspath(os.path.expanduser(a.movie))
        os.makedirs(os.path.dirname(movie), exist_ok=True)
        env['ATTABOT_MOVIE'] = movie
        print(f'[exp] grabando video → {movie}')
    kill_webots()
    time.sleep(2)
    raw = open(RAW_LOG, 'w')
    t0 = time.time()

    base = subprocess.Popen(
        [sys.executable, '-u', 'AttaBot_Base.py', '--sim', '--headless',
         '--robots', str(a.robots)],
        cwd=a.base_dir, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, env=env)
    base_tail = Tail(base, 'base', raw)

    def console(cmd):
        base.stdin.write(cmd + '\n')
        base.stdin.flush()
        raw.write(f'>>> {cmd}\n')
        print(f'[exp] consola: {cmd}', flush=True)

    if base_tail.wait_for('bind exitoso', 15) is None:
        print('[exp] FATAL: la base no tomó el 6060')
        base.terminate()
        return 1

    display = [] if a.gui else ['--no-rendering', '--minimize']
    webots = subprocess.Popen(
        ['flatpak', 'run', '--filesystem=home', 'com.cyberbotics.webots',
         *display, '--batch', '--mode=realtime',
         '--stdout', '--stderr', a.world],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    webots_tail = Tail(webots, 'webots', raw)

    conv, dists = False, []
    try:
        if base_tail.wait_for('Robots encontrados', 150) is None:
            print(f'[exp] FATAL: setup incompleto (¿{a.robots} robots en el mundo?)')
            return 1
        print(f'[exp] setup completo — {a.robots} robots asociados', flush=True)
        time.sleep(5)

        followers = [r for r in ids if r != a.leader]
        console(f'CONGREGATION.{a.leader}')
        deadline = time.time() + a.timeout
        while time.time() < deadline:
            poses = latest_poses(webots_tail)
            if all(r in poses for r in ids):
                lx, ly, _ = poses[a.leader]
                dists = [math.dist((lx, ly), poses[r][:2]) for r in followers]
                if all(abs(d - parking) <= tol for d in dists):
                    conv = True
                    print(f'[exp] ✓ CONGREGADOS (t={time.time()-t0:.0f}s) — '
                          f'dist. al líder {[f"{d:.0f}" for d in dists]}mm '
                          f'(objetivo {parking:.0f}±{tol:.0f})', flush=True)
                    break
            time.sleep(5)
        if not conv:
            shown = [f'{d:.0f}' for d in dists] if dists else 'n/a'
            print(f'[exp] ✗ no convergieron en {a.timeout:.0f}s — dist. {shown}mm '
                  f'(objetivo {parking:.0f}±{tol:.0f})', flush=True)
        time.sleep(3)
        console('BREAK')
        time.sleep(3)
    finally:
        if movie:
            # Cerrar el video ANTES de matar Webots: si no, el mp4 queda corrupto.
            ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            ctl.sendto(b'MOVIE_STOP', ('127.0.0.1', 6059))
            if webots_tail.wait_for('VIDEO LISTO', 120) is None:
                print('[exp] ⚠ el video no cerró a tiempo — puede quedar incompleto')
            ctl.close()
        kill_webots()
        base.terminate()
        try:
            base.wait(timeout=10)
        except subprocess.TimeoutExpired:
            base.kill()
        raw.close()

    # Localizar el log recién generado y dar el comando de métricas
    pos = max(glob.glob(os.path.join(a.base_dir, 'PositionLogs',
                                     'Position_Log_SIM_*.csv')),
              key=os.path.getmtime, default=None)
    print(f'\n{"✓ CONGREGACIÓN OK" if conv else "✗ CONGREGACIÓN FALLÓ"} '
          f'[{a.layout}] — log crudo en {RAW_LOG}')
    if pos:
        tag = re.sub(r'^Position_Log_', '', os.path.basename(pos)).rsplit('.', 1)[0]
        print(f'PositionLog: {os.path.basename(pos)}')
        print(f'Métricas:\n  python analyze_logs.py --congregation '
              f'--session {tag} --layout {a.layout} --plot')
    return 0 if conv else 1


if __name__ == '__main__':
    sys.exit(main())
