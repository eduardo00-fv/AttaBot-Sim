#!/usr/bin/env python3
"""
experimento_enjambre.py — E2E de comportamientos de enjambre (4 robots)
=========================================================================
Con la base REAL controlando Webots:

  1. DISPERSE|600  — los 4 robots parten en clúster (mín. ~450mm) y se
     repelen hasta que cada uno tiene a su vecino más cercano a ≥600mm.
     Métrica: distancia mínima entre pares (ground truth POS de Webots).
  2. FORMATION.linea 1 — fila perpendicular al heading del líder con slots
     asignados por la base (anti-cruce). Métrica: distancias al líder
     esperadas {300, 300, 600} ±120mm.

Uso: python tools/experimento_enjambre.py [--base-dir ...]
"""

import argparse
import itertools
import math
import os
import re
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from experimento_base_real import Tail, kill_webots, DEFAULT_BASE, WORLD  # noqa: E402

RAW_LOG = '/tmp/experimento_enjambre.log'
RE_POS = re.compile(r'^POS\|(\d+)\|(-?[\d.]+)\|(-?[\d.]+)\|(-?[\d.]+)')
DISPERSE_TARGET = 600.0   # mm de separación mínima objetivo (ground truth)
# El enjambre corre en la arena SIN el campo de obstáculos: dispersión y
# formación son geometría inter-robot y las cajas solo bloquean al líder camino
# al ancla. La evasión/búsqueda se validan aparte en attabot.wbt.
SWARM_WORLD = os.path.join(os.path.dirname(WORLD), 'attabot_swarm.wbt')


def latest_poses(tail):
    """Últimas poses ground-truth {id: (x,y,ang)} del stdout de Webots."""
    poses = {}
    with tail.lock:
        for line in tail.lines:
            m = RE_POS.match(line.strip())
            if m:
                poses[m.group(1)] = (float(m.group(2)), float(m.group(3)),
                                     float(m.group(4)))
    return poses


def min_pairwise(poses):
    return min(math.dist(a[:2], b[:2])
               for a, b in itertools.combinations(poses.values(), 2))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base-dir', default=DEFAULT_BASE)
    ap.add_argument('--world', default=SWARM_WORLD,
                    help='mundo Webots (por defecto la arena despejada de enjambre)')
    a = ap.parse_args()

    env = dict(os.environ)
    env.setdefault('XAUTHORITY', os.path.expanduser('~/.Xauthority'))
    env.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')
    env.setdefault('DISPLAY', ':0')
    kill_webots()
    time.sleep(2)
    raw = open(RAW_LOG, 'w')
    t0 = time.time()

    base = subprocess.Popen(
        [sys.executable, '-u', 'AttaBot_Base.py', '--sim', '--headless',
         '--robots', '4'],
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

    webots = subprocess.Popen(
        ['flatpak', 'run', '--filesystem=home', 'com.cyberbotics.webots',
         '--no-rendering', '--batch', '--minimize', '--mode=realtime',
         '--stdout', '--stderr', a.world],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    webots_tail = Tail(webots, 'webots', raw)

    ok = True
    try:
        if base_tail.wait_for('Robots encontrados', 90) is None:
            print('[exp] FATAL: setup incompleto (¿4 robots en el mundo?)')
            return 1
        print('[exp] setup completo — 4 robots asociados', flush=True)
        time.sleep(5)

        # ── 1. Dispersión ────────────────────────────────────────────────────
        # Métrica de aprobación = separación mínima real medida por el supervisor
        # de Webots (ground truth POS). El observador externo es el instrumento
        # del experimento; las auto-confirmaciones del enjambre ("dispersión
        # lograda") se cuentan solo como diagnóstico, porque dependen de la pose
        # ruidosa de cada robot y de atrapar un tick IDLE y pueden ir rezagadas.
        d0 = min_pairwise(latest_poses(webots_tail))
        print(f'[exp] separación mínima inicial: {d0:.0f}mm', flush=True)
        mark = len(base_tail.lines)
        console(f'BROADCAST.DISPERSE|{DISPERSE_TARGET:.0f}')

        settled_ids = set()
        deadline = time.time() + 420
        d1 = d0
        while time.time() < deadline:
            with base_tail.lock:
                for line in base_tail.lines[mark:]:
                    m = re.search(r'ID: (\d+), dispersión lograda', line)
                    if m and m.group(1) not in settled_ids:
                        settled_ids.add(m.group(1))
                        print(f'[exp] robot {m.group(1)} confirmó '
                              f'({len(settled_ids)}/4, t={time.time()-t0:.0f}s)',
                              flush=True)
                mark = len(base_tail.lines)
            d1 = min_pairwise(latest_poses(webots_tail))
            if d1 >= DISPERSE_TARGET:            # separación física lograda
                break
            time.sleep(5)
        disp_ok = d1 >= DISPERSE_TARGET
        ok &= disp_ok
        print(f'[exp] {"✓" if disp_ok else "✗"} dispersión (ground truth): mínima '
              f'{d0:.0f} → {d1:.0f}mm / objetivo {DISPERSE_TARGET:.0f} '
              f'({len(settled_ids)}/4 auto-confirmaron)', flush=True)

        # ── 2. Formación en línea (líder 1) ──────────────────────────────────
        console('BROADCAST.CANCEL_CONGREGATION')
        time.sleep(2)
        # Posicionar al líder en la única banda despejada para una fila con
        # espaciado 250 (entre las cajas y el cilindro rojo): (1580,875)
        # mirando al este — la perpendicular vertical no cabe y la base debe
        # caer al eje del heading (fila horizontal hacia el oeste)
        mark = len(base_tail.lines)
        console('GOTO.1 1580 875')
        idx = base_tail.wait_for('NAV: llegó', 150, start=mark)
        if idx is None:
            print('[exp] FATAL: el líder no llegó a la banda libre')
            return 1
        time.sleep(2)
        _, _, lang = latest_poses(webots_tail)['1']
        delta = ((0 - lang + 180) % 360) - 180
        console(f'1.TURN|{delta:.0f}')   # heading este (0°)
        time.sleep(8)

        # Igual que la dispersión: la aprobación se mide sobre las posiciones
        # ground truth (distancia de cada seguidor al líder vs. el slot esperado),
        # sondeando hasta que las 3 caigan en tolerancia o venza el deadline. Las
        # "NAV: llegó" quedan como diagnóstico (dependen de la pose del seguidor).
        mark = len(base_tail.lines)
        console('FORMATION.linea 1 250')
        expected = [250, 250, 500]
        arrivals, deadline, dists = 0, time.time() + 300, None
        while time.time() < deadline:
            with base_tail.lock:
                for line in base_tail.lines[mark:]:
                    if 'NAV: llegó' in line and 'ID: 1,' not in line:
                        arrivals += 1
                mark = len(base_tail.lines)
            poses = latest_poses(webots_tail)
            if all(r in poses for r in ('1', '2', '3', '4')):
                lx, ly, _ = poses['1']
                dists = sorted(math.dist((lx, ly), poses[r][:2])
                               for r in ('2', '3', '4'))
                if all(abs(d - e) <= 120 for d, e in zip(dists, expected)):
                    break
            time.sleep(5)
        form_ok = dists is not None and all(
            abs(d - e) <= 120 for d, e in zip(dists, expected))
        ok &= form_ok
        shown = [f'{d:.0f}' for d in dists] if dists else 'n/a'
        print(f'[exp] {"✓" if form_ok else "✗"} formación linea (ground truth): '
              f'distancias al líder {shown} (esperado ~{expected}, '
              f'{arrivals}/3 auto-confirmaron)', flush=True)

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

    print(f'\n{"✓ ENJAMBRE E2E EXITOSO" if ok else "✗ ENJAMBRE E2E FALLÓ"} '
          f'— log crudo en {RAW_LOG}')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
