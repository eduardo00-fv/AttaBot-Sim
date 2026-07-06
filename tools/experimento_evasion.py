#!/usr/bin/env python3
"""
experimento_evasion.py — Evasión binaria (IR actuales) vs proporcional (ToF)
=============================================================================
Corre en Webots headless 3 configuraciones del robot 1 cruzando un campo de
3 obstáculos, ida y vuelta entre dos esquinas, y compara:

  binary @ 0.20m  — los módulos IR de hoy (umbral, giro fijo)
  prop   @ 0.20m  — solo el cambio de algoritmo, mismo alcance
  prop   @ 0.60m  — el paquete completo VL53L0X (distancia + alcance)

Métricas por pierna (del ground truth POS del supervisor):
  tiempo sim, largo de camino, holgura mínima a obstáculos, interrupciones,
  llegada. Uso:  python3 tools/experimento_evasion.py [--legs 6]
"""
import argparse
import math
import os
import select
import socket
import subprocess
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD = os.path.join(REPO, 'webots', 'worlds', 'attabot.wbt')

# Campo de obstáculos (coords cámara, mm) — debe coincidir con attabot.wbt
OBSTACLES = [(1100, 650), (650, 1000), (1600, 300)]
OBS_HALF = 75.0      # mitad del lado de la caja
ROBOT_R = 75.0
GOAL_A = (2050, 1230)
GOAL_B = (350, 320)

ARMS = [('binary', 0.20), ('prop', 0.20), ('prop', 0.60)]
LEG_TIMEOUT_SIM = 180.0   # s sim por pierna


def clearance(x, y):
    """Holgura superficie-a-superficie robot↔caja más cercana (mm)."""
    best = 1e9
    for ox, oy in OBSTACLES:
        d = max(abs(x - ox), abs(y - oy)) - OBS_HALF - ROBOT_R
        best = min(best, d)
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--legs', type=int, default=6)
    a = ap.parse_args()

    env = dict(os.environ)
    env.setdefault('XAUTHORITY', os.path.expanduser('~/.Xauthority'))
    env.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')
    env.setdefault('DISPLAY', ':0')
    # Matar instancias previas — terminate() no alcanza al webots dentro del
    # sandbox de flatpak y los puertos UDP quedarían tomados
    subprocess.run(['flatpak', 'kill', 'com.cyberbotics.webots'],
                   stderr=subprocess.DEVNULL)
    time.sleep(2)
    proc = subprocess.Popen(
        ['flatpak', 'run', '--filesystem=home', 'com.cyberbotics.webots',
         '--no-rendering', '--batch', '--minimize', '--mode=fast',
         '--stdout', '--stderr', WORLD],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    raw_log = open('/tmp/experimento_evasion_webots.log', 'w')

    def cmd(c):
        sock.sendto(c.encode(), ('127.0.0.1', 6060))
        raw_log.write(f'>>> {c}\n')

    def lines(timeout=1.0):
        """Genera líneas del stdout de Webots con timeout (tee a raw_log)."""
        while True:
            r, _, _ = select.select([proc.stdout], [], [], timeout)
            if not r:
                yield None
                continue
            line = proc.stdout.readline()
            if not line:
                return
            raw_log.write(line)
            raw_log.flush()
            yield line.strip()

    reader = lines()

    # Esperar a que ambos robots y la base estén listos
    ready = 0
    for line in reader:
        if line is None:
            continue
        if 'puerto' in line and 'ocupado' in line:
            print(f'[exp] FATAL: {line} — quedó otra instancia viva', flush=True)
            subprocess.run(['flatpak', 'kill', 'com.cyberbotics.webots'])
            return 1
        if 'listo en puerto' in line and not line.startswith('['):
            ready += 1
        if ready >= 2:
            break
    print('[exp] mundo listo', flush=True)
    time.sleep(1)

    results = {}
    for mode, ir_range in ARMS:
        arm = f'{mode}@{ir_range:.2f}'
        cmd(f'1.NAV_CONFIG|AVOID|{mode}')
        cmd(f'1.NAV_CONFIG|IR_RANGE|{ir_range}')
        time.sleep(0.5)
        legs = []
        for i in range(a.legs):
            goal = GOAL_A if i % 2 == 0 else GOAL_B
            cmd(f'1.POSITIONGT|{goal[0]}|{goal[1]}')
            t_sim, path, min_cl, interrupts = 0.0, 0.0, 1e9, 0
            prev = None
            arrived = False
            wall_start = time.time()
            for line in reader:
                if line is None:
                    if time.time() - wall_start > 300 or proc.poll() is not None:
                        print('[exp] pierna abortada (sin salida de Webots)',
                              flush=True)
                        break
                    continue
                if line.startswith('POS|1|'):
                    _, _, x, y, _ = line.split('|')
                    x, y = float(x), float(y)
                    t_sim += 0.5
                    if prev:
                        path += math.hypot(x - prev[0], y - prev[1])
                    prev = (x, y)
                    min_cl = min(min_cl, clearance(x, y))
                    if t_sim > LEG_TIMEOUT_SIM:
                        cmd('1.ABORT_NAV')
                        break
                elif 'MOVE interrumpido' in line and not line.startswith('['):
                    interrupts += 1
                elif 'NAV: llegó' in line and not line.startswith('['):
                    arrived = True
                    break
            legs.append(dict(t=t_sim, path=path, cl=min_cl,
                             stops=interrupts, ok=arrived))
            print(f'[exp] {arm} pierna {i+1}/{a.legs}: '
                  f'{"llegó" if arrived else "TIMEOUT"} t={t_sim:.0f}s '
                  f'path={path:.0f}mm holgura={min_cl:.0f}mm stops={interrupts}',
                  flush=True)
        results[arm] = legs

    subprocess.run(['flatpak', 'kill', 'com.cyberbotics.webots'],
                   stderr=subprocess.DEVNULL)
    proc.terminate()

    print('\n════════ RESUMEN (medias por pierna) ════════', flush=True)
    print(f'{"config":14} {"llegadas":>9} {"tiempo":>8} {"camino":>9} '
          f'{"holgura mín":>12} {"stops":>6}')
    for arm, legs in results.items():
        ok = sum(l["ok"] for l in legs)
        good = [l for l in legs if l['ok']] or legs
        print(f'{arm:14} {ok:>4}/{len(legs)}'
              f' {sum(l["t"] for l in good)/len(good):>7.0f}s'
              f' {sum(l["path"] for l in good)/len(good):>8.0f}mm'
              f' {sum(l["cl"] for l in good)/len(good):>11.0f}mm'
              f' {sum(l["stops"] for l in legs):>6}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
