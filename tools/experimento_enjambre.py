#!/usr/bin/env python3
"""
experimento_enjambre.py — E2E de comportamientos de enjambre (4 robots)
=========================================================================
Con la base REAL controlando Webots:

  1. DISPERSE|600  — los 4 robots parten en clúster (mín. ~450mm) y se
     repelen hasta que cada uno tiene a su vecino más cercano a ≥600mm.
     Métrica: distancia mínima entre pares (ground truth POS de Webots).
  2. FORMATION.{linea,cuna,circulo} 1 — el líder se fija en el centro y cada
     figura re-asigna a los seguidores con slots anti-cruce de la base.
     Métrica (ground truth POS, espaciado s=250): linea [s,s,2s], cuna
     [s√2,s√2,2s√2], circulo [s,s,s], cada distancia al líder ±120mm.

Uso: python tools/experimento_enjambre.py [--base-dir ...] [--world ...]
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

        # ── 2. Formaciones: linea, cuna, circulo (líder 1) ───────────────────
        # El líder va UNA vez al centro despejado (1200,775) mirando al este y se
        # queda fijo (marca "soy líder", no toma slot); cada figura sólo re-asigna
        # a los 3 seguidores. Aprobación por ground truth: distancia de cada
        # seguidor al líder vs. el slot esperado, sondeando hasta tolerancia (±120)
        # o deadline. Distancias esperadas con n=3 seguidores y espaciado s:
        #   linea [s, s, 2s] · cuna [s√2, s√2, 2s√2] · circulo [s, s, s]
        # (offsets: linea ±k·s perpendicular; cuna ±k·s perp − k·s atrás;
        #  circulo radio s a 2π·idx/n). En (1200,775) las tres caben en axis=0.
        console('BROADCAST.CANCEL_CONGREGATION')
        time.sleep(2)
        LX, LY, S = 1200.0, 775.0, 250.0
        mark = len(base_tail.lines)
        console(f'GOTO.1 {LX:.0f} {LY:.0f}')
        if base_tail.wait_for('NAV: llegó', 150, start=mark) is None:
            print('[exp] FATAL: el líder no llegó al centro')
            return 1
        time.sleep(2)
        _, _, lang = latest_poses(webots_tail)['1']
        delta = ((0 - lang + 180) % 360) - 180
        console(f'1.TURN|{delta:.0f}')   # heading este (0°)
        time.sleep(8)

        r2 = math.sqrt(2)
        shapes = [
            ('linea',   sorted([S, S, 2 * S])),
            ('cuna',    sorted([S * r2, S * r2, 2 * S * r2])),
            ('circulo', sorted([S, S, S])),
        ]
        for shape, expected in shapes:
            console('BROADCAST.CANCEL_CONGREGATION')
            time.sleep(2)
            console(f'FORMATION.{shape} 1 {S:.0f}')
            deadline, dists = time.time() + 240, None
            while time.time() < deadline:
                poses = latest_poses(webots_tail)
                if all(r in poses for r in ('1', '2', '3', '4')):
                    lx, ly, _ = poses['1']
                    dists = sorted(math.dist((lx, ly), poses[r][:2])
                                   for r in ('2', '3', '4'))
                    if all(abs(d - e) <= 120 for d, e in zip(dists, expected)):
                        break
                time.sleep(5)
            shape_ok = dists is not None and all(
                abs(d - e) <= 120 for d, e in zip(dists, expected))
            ok &= shape_ok
            shown = [f'{d:.0f}' for d in dists] if dists else 'n/a'
            print(f'[exp] {"✓" if shape_ok else "✗"} formación {shape} '
                  f'(ground truth): distancias al líder {shown} '
                  f'(esperado ~{[round(e) for e in expected]})', flush=True)

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
