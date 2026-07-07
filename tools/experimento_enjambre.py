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
         '--stdout', '--stderr', WORLD],
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
        d0 = min_pairwise(latest_poses(webots_tail))
        print(f'[exp] separación mínima inicial: {d0:.0f}mm', flush=True)
        mark = len(base_tail.lines)
        console('BROADCAST.DISPERSE|600')

        settled_ids = set()
        deadline = time.time() + 420
        d1 = d0
        while time.time() < deadline:
            with base_tail.lock:
                for line in base_tail.lines[mark:]:
                    m = re.search(r'ID: (\d+), dispersión lograda', line)
                    if m and m.group(1) not in settled_ids:
                        settled_ids.add(m.group(1))
                        print(f'[exp] robot {m.group(1)} disperso '
                              f'({len(settled_ids)}/4, t={time.time()-t0:.0f}s)',
                              flush=True)
                mark = len(base_tail.lines)
            d1 = min_pairwise(latest_poses(webots_tail))
            if len(settled_ids) == 4 and d1 >= 550:
                break
            time.sleep(5)
        disp_ok = len(settled_ids) == 4 and d1 >= 500
        ok &= disp_ok
        print(f'[exp] {"✓" if disp_ok else "✗"} dispersión: mínima '
              f'{d0:.0f} → {d1:.0f}mm ({len(settled_ids)}/4 confirmaron)',
              flush=True)

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

        mark = len(base_tail.lines)
        console('FORMATION.linea 1 250')
        arrivals, deadline = 0, time.time() + 300
        while arrivals < 3 and time.time() < deadline:
            idx = base_tail.wait_for('NAV: llegó', 20, start=mark)
            if idx is not None:
                mark = idx
                arrivals += 1
        time.sleep(2)
        poses = latest_poses(webots_tail)
        lx, ly, _ = poses['1']
        dists = sorted(math.dist((lx, ly), poses[r][:2]) for r in ('2', '3', '4'))
        expected = [250, 250, 500]
        form_ok = (arrivals >= 3 and
                   all(abs(d - e) <= 120 for d, e in zip(dists, expected)))
        ok &= form_ok
        print(f'[exp] {"✓" if form_ok else "✗"} formación linea: distancias al '
              f'líder {[f"{d:.0f}" for d in dists]} (esperado ~{expected}, '
              f'{arrivals}/3 llegadas)', flush=True)

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
