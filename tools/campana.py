#!/usr/bin/env python3
"""
campana.py — runner de lote del experimento de topología (paper)
=================================================================
Ejecuta la campaña completa: por cada escenario del plan × N repeticiones,
genera el mundo (obstáculos por solver + poses iniciales aleatorias con semilla
por repetición), corre el protocolo del paper contra la base REAL en modo sim:

    RANDOMW|60 (todos) → CONGREGATION.<líder> → veredicto ground-truth → BREAK

y deja un MANIFIESTO listo para el agregador:
    python analyze_logs.py --campaign <out>/manifiesto.csv --plot

Decisiones deliberadas:
  - --mode=realtime SIEMPRE: la base timestampea con reloj de pared; en fast la
    velocidad de sim varía con la carga y los tiempos de convergencia quedarían
    incomparables entre corridas.
  - Una corrida que NO converge es DATO (así se compara la topología), no error:
    se registra y la campaña sigue.
  - La semilla de cada rep queda en el .scenario.json → cualquier corrida es
    re-ejecutable exactamente.

Plan (JSON):
{
  "arena": "3.8,2.8", "robots": 10, "leader": 1, "base_seed": 20260729,
  "scenarios": [
    {"name": "A_base"},
    {"name": "B_chico_estrecho", "solve": "4.5,150,300"},
    {"name": "D_grande_estrecho", "solve": "4.5,350,300"}
  ]
}

Uso:
    python tools/campana.py --plan plan.json --reps 6 --out ~/campana_29-07
    python tools/campana.py --plan plan.json --reps 1 --scenarios A_base  # humo
"""

import argparse
import glob
import json
import math
import os
import re
import statistics
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from experimento_base_real import Tail, kill_webots, DEFAULT_BASE  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLDS = os.path.join(REPO, 'webots', 'worlds')
RE_POS = re.compile(r'^POS\|(\d+)\|(-?[\d.]+)\|(-?[\d.]+)\|')
RW_SECONDS = 60          # random walk del protocolo
CONV_HOLD_S = 3.0        # sostener el criterio este tiempo


def latest_poses(tail):
    poses = {}
    with tail.lock:
        for line in tail.lines:
            m = RE_POS.match(line.strip())
            if m:
                poses[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    return poses


def frac_converged(poses, radius, frac, centro=None):
    """Criterio del paper sobre ground truth: `frac` de los robots dentro de
    `radius` del punto de referencia. Con destino fijo (topología) se mide
    contra ESE punto, no contra el centroide: si se midiera el centroide, un
    grupo que quedó junto ANTES de la barrera contaría como convergido."""
    pts = list(poses.values())
    if len(pts) < 2:
        return False
    if centro is not None:
        cx, cy = centro
    else:
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
    inside = sum(1 for p in pts if math.dist(p, (cx, cy)) <= radius)
    return inside >= math.ceil(frac * len(pts))


def gen_world(scn, plan, seed, out_wbt):
    cmd = [sys.executable, os.path.join(REPO, 'tools', 'gen_world.py'),
           '--robots', str(plan['robots']), '--arena', plan['arena'],
           '--leader', str(plan.get('leader', 1)), '--out', out_wbt]
    if plan.get('start'):
        # Experimento de topología: los robots arrancan en fila contra la pared
        # de origen, mirando al destino. La aleatorización del protocolo la da
        # el random walk previo, no la posición de partida.
        cmd += ['--start', ';'.join(f'{x},{y}' for x, y in plan['start'])]
    else:
        cmd += ['--scatter', '--seed', str(seed)]
    if scn.get('solve'):
        cmd += ['--solve', scn['solve']]
    elif scn.get('obstacles'):
        cmd += ['--obstacles'] + scn['obstacles']
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f'✗ gen_world falló para {scn["name"]} seed {seed}:\n'
                         f'{r.stdout}{r.stderr}')


def run_once(world, plan, a, raw, log_prefix, gt_path=None):
    """Una corrida completa. Retorna dict con el resultado (o 'error')."""
    n = plan['robots']
    leader = str(plan.get('leader', 1))
    env = dict(os.environ)
    env.setdefault('XAUTHORITY', os.path.expanduser('~/.Xauthority'))
    env.setdefault('XDG_RUNTIME_DIR', '/run/user/1000')
    env.setdefault('DISPLAY', ':0')
    env.setdefault('__GLX_VENDOR_LIBRARY_NAME', 'mesa')
    if gt_path:
        env['SIM_GT_LOG'] = gt_path      # el supervisor vuelca posición real

    kill_webots()
    time.sleep(2)
    t0 = time.time()
    base = subprocess.Popen(
        [sys.executable, '-u', 'AttaBot_Base.py', '--sim', '--headless',
         '--robots', str(n)],
        cwd=a.base_dir, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, text=True, env=env)
    base_tail = Tail(base, f'{log_prefix}|base', raw)

    def console(cmd):
        base.stdin.write(cmd + '\n')
        base.stdin.flush()
        raw.write(f'>>> {cmd}\n')

    result = {'converged': False, 't_conv_s': None, 'error': None}
    webots = None
    try:
        if base_tail.wait_for('bind exitoso', 15) is None:
            result['error'] = 'base no tomó el 6060'
            return result
        webots = subprocess.Popen(
            ['flatpak', 'run', '--filesystem=home', 'com.cyberbotics.webots',
             '--no-rendering', '--batch', '--minimize', '--mode=realtime',
             '--stdout', '--stderr', world],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
        webots_tail = Tail(webots, f'{log_prefix}|webots', raw)
        if base_tail.wait_for('Robots encontrados', 150) is None:
            result['error'] = 'setup incompleto'
            return result
        time.sleep(4)

        # Alcance efectivo del IR. Es CRÍTICO para la fidelidad: el modelo traía
        # 200mm y con eso un hueco de 210mm dispara los laterales a 138mm, o sea
        # evasión perpetua — mientras en el lab esos mismos 210mm se cruzan sin
        # problema. Se declara en el plan y se documenta en el paper.
        ir = plan.get('ir_range_mm')
        if ir:
            console(f'BROADCAST.NAV_CONFIG|IR_RANGE|{ir/1000.0:.3f}')
            print(f'    IR_RANGE = {ir:.0f}mm', flush=True)
            time.sleep(1)

        # ── Fase 1: random walk del protocolo (opcional) ─────────────────────
        rw = plan.get('random_walk_s', RW_SECONDS)
        done = plan['robots']
        if rw <= 0:
            print('    (sin random walk: arranque en fila)', flush=True)
        else:
            console(f'BROADCAST.RANDOMW|{rw}')
            deadline = time.time() + rw + 25
            while time.time() < deadline:
                with webots_tail.lock:
                    done = len({ln.split('ID: ')[1].split(',')[0]
                                for ln in webots_tail.lines
                                if 'Random Walk terminado' in ln})
                if done >= n:
                    break
                time.sleep(2)
            print(f'    walk listo ({done}/{n}, t={time.time() - t0:.0f}s)',
                  flush=True)

        # ── Fase 2: congregación en el punto destino ─────────────────────────
        dest = plan.get('destino')
        if dest:
            # Topología: no hay robot al otro extremo. MEET reparte a los N en
            # un anillo alrededor del punto, así que cruzan la barrera y llegan
            # repartidos en vez de amontonarse como haría un GT.
            console(f'BROADCAST.MEET|{dest[0]:.0f}|{dest[1]:.0f}')
            centro = (float(dest[0]), float(dest[1]))
        else:
            console(f'CONGREGATION.{leader}')
            centro = None
        t_cong = time.time()
        ok_since = None
        deadline = t_cong + a.timeout
        while time.time() < deadline:
            if frac_converged(latest_poses(webots_tail), a.radius, a.frac,
                              centro):
                if ok_since is None:
                    ok_since = time.time()
                elif time.time() - ok_since >= CONV_HOLD_S:
                    result['converged'] = True
                    result['t_conv_s'] = round(ok_since - t_cong, 1)
                    break
            else:
                ok_since = None
            time.sleep(1)
        time.sleep(3)
        console('BREAK')
        time.sleep(3)
    finally:
        kill_webots()
        base.terminate()
        try:
            base.wait(timeout=10)
        except subprocess.TimeoutExpired:
            base.kill()
    return result


ROBOT_RADIUS_MM = 52.5      # AttaBot real: 105mm de diámetro CON ruedas


def min_clearance(log_path, scn):
    """OJO: log_path debe ser la traza VERDADERA del supervisor. Contra el
    PositionLog normal esto no significa nada — ese lleva ruido ArUco de 30mm.
    """
    """Menor distancia entre el borde del robot y el borde de un obstáculo.

    Un escenario se cruza NAVEGANDO o se cruza EMPUJANDO, y el tiempo de
    convergencia no distingue los dos casos. Si esto sale <=0 la corrida no
    mide topología, mide colisiones, y el número no sirve para el paper.
    """
    import csv as _csv
    boxes = []
    for spec in scn['obstacles']:
        parts = [t.strip() for t in spec.split(',')]
        if parts[-1].upper() == 'S':
            parts = parts[:-1]
        v = [float(t) for t in parts]
        x, y, w = v[0], v[1], v[2]
        h = v[3] if len(v) >= 4 else w
        boxes.append((x, y, w, h))
    worst = float('inf')
    try:
        with open(log_path) as f:
            for r in _csv.DictReader(f):
                try:
                    x, y = float(r['x']), float(r['y'])
                except (KeyError, TypeError, ValueError):
                    continue
                for (ox, oy, w, h) in boxes:
                    dx = max(abs(x - ox) - w / 2.0, 0.0)
                    dy = max(abs(y - oy) - h / 2.0, 0.0)
                    d = math.hypot(dx, dy) - ROBOT_RADIUS_MM
                    if d < worst:
                        worst = d
    except OSError:
        return None
    return None if worst == float('inf') else worst



def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plan', required=True, help='JSON con arena/robots/escenarios')
    ap.add_argument('--reps', type=int, default=6)
    ap.add_argument('--out', required=True, help='directorio de la campaña')
    ap.add_argument('--scenarios', nargs='*',
                    help='correr solo estos nombres (default: todos)')
    ap.add_argument('--base-dir', default=DEFAULT_BASE)
    ap.add_argument('--radius', type=float, default=450.0,
                    help='radio del veredicto en vivo (el análisis final usa el suyo)')
    ap.add_argument('--frac', type=float, default=0.9)
    ap.add_argument('--timeout', type=float, default=300.0,
                    help='s máximos de congregación; agotarlo = corrida NO convergida'
                         ' (es dato, no error)')
    a = ap.parse_args()

    with open(a.plan) as f:
        plan = json.load(f)
    scens = [s for s in plan['scenarios']
             if not a.scenarios or s['name'] in a.scenarios]
    if not scens:
        raise SystemExit('✗ ningún escenario del plan matchea --scenarios')
    os.makedirs(a.out, exist_ok=True)
    manifest_path = os.path.join(a.out, 'manifiesto.csv')
    raw = open(os.path.join(a.out, 'campana_raw.log'), 'a')
    base_seed = plan.get('base_seed', 20260729)

    # Reanudable: las (escenario, rep) ya registradas no se repiten — relanzar
    # con los mismos argumentos completa solo lo que falta, sin duplicar filas.
    completed = set()
    if os.path.exists(manifest_path):
        import csv as _csv
        with open(manifest_path) as f:
            for r in _csv.DictReader(f):
                if r.get('rep'):
                    completed.add((r['scenario'], int(r['rep'])))

    total = len(scens) * a.reps
    print(f'=== CAMPAÑA: {len(scens)} escenario(s) × {a.reps} reps = {total} '
          f'corridas (~{total * 3:.0f}-{total * 6:.0f} min) ===')
    new_rows, k = [], 0
    t_start = time.time()
    for scn in scens:
        for rep in range(1, a.reps + 1):
            k += 1
            if (scn['name'], rep) in completed:
                print(f'[{k}/{total}] {scn["name"]}_r{rep} — ya en el manifiesto, '
                      f'salteada', flush=True)
                new_rows.append(None)   # cuenta como hecha para el resumen
                continue
            seed = base_seed + rep
            name = f'{scn["name"]}_r{rep}'
            world = os.path.join(WORLDS, f'camp_{name}.wbt')
            gen_world(scn, plan, seed, world)
            print(f'[{k}/{total}] {name} (seed {seed})', flush=True)
            before = set(glob.glob(os.path.join(
                a.base_dir, 'PositionLogs', 'Position_Log_SIM_*.csv')))
            # Dentro del repo: el sandbox de flatpak que corre Webots no ve
            # rutas arbitrarias (p.ej. /tmp del host). Se copia al --out al final.
            gt_path = os.path.join(REPO, 'webots', 'gt_logs', f'gt_{name}.csv')
            res = run_once(world, plan, a, raw, name, gt_path)
            after = set(glob.glob(os.path.join(
                a.base_dir, 'PositionLogs', 'Position_Log_SIM_*.csv'))) - before
            if res['error']:
                print(f'    ✗ ERROR: {res["error"]} — quedó fuera; relanzá la '
                      f'campaña para completarla', flush=True)
                continue
            if not after:
                print('    ✗ la corrida no dejó PositionLog — descartada', flush=True)
                continue
            tag = re.sub(r'^Position_Log_', '',
                         os.path.basename(max(after, key=os.path.getmtime))
                         ).rsplit('.', 1)[0]
            verdict = (f'✓ convergió en {res["t_conv_s"]:.0f}s' if res['converged']
                       else f'∅ sin converger en {a.timeout:.0f}s (dato válido)')
            print(f'    {verdict} → {tag}', flush=True)
            if os.path.exists(gt_path):
                shutil.copy2(gt_path, os.path.join(a.out, os.path.basename(gt_path)))
            clr = min_clearance(gt_path, scn) if os.path.exists(gt_path) else None
            if clr is not None:
                if clr <= 0:
                    print(f'    ⚠ INVÁLIDA: tocó los obstáculos ({clr:.0f}mm) — '
                          f'cruzó empujando, no navegando', flush=True)
                else:
                    print(f'    holgura mínima {clr:.0f}mm', flush=True)
            row = {'session': tag, 'scenario': scn['name'],
                   'scenario_json': os.path.splitext(world)[0] + '.scenario.json',
                   'rep': rep, 'clearance_mm': ('' if clr is None else f'{clr:.1f}')}
            new_rows.append(row)
            header = not os.path.exists(manifest_path)
            with open(manifest_path, 'a', newline='') as f:
                if header:
                    f.write('session,scenario,scenario_json,rep,clearance_mm\n')
                f.write(f'{row["session"]},{row["scenario"]},'
                        f'{row["scenario_json"]},{rep},{row["clearance_mm"]}\n')
    raw.close()

    done = len(new_rows)
    print(f'\n=== FIN: {done}/{total} corridas registradas en '
          f'{time.time() - t_start:.0f}s ===')
    if done < total:
        print(f'⚠ faltan {total - done} — relanzá con los mismos argumentos: el '
              f'manifiesto es incremental (append)')
    print(f'\nAnálisis:\n  cd {a.base_dir} && python3 analyze_logs.py '
          f'--campaign {manifest_path} '
          f'--campaign-out {os.path.join(a.out, "campana")} --plot')


if __name__ == '__main__':
    main()
