#!/usr/bin/env python3
"""
gen_world.py — generador de mundos Webots para AttaBot (Fase 3a/3b)
====================================================================
Emite un .wbt parametrizado por nº de robots, tamaño de arena y (opcional)
campo de obstáculos, evitando editar a mano decenas de nodos. Ata lab↔sim:
la misma convención de coordenadas que base_camera.py (origen en la esquina
sup-izq del marco cámara, y hacia abajo, mm) — los obstáculos se dan en mm de
lab y se convierten a Webots.

  Webots(m) ← cámara(mm):  x_w = x_mm/1000 − W/2 ·  y_w = H/2 − y_mm/1000

Robots: el líder al centro (0,0), los seguidores repartidos en una elipse
(0.75·semiancho) alrededor — buena separación inicial para congregarse.
base_camera los auto-descubre por customData; los puertos son 6060+id.

Uso:
    python tools/gen_world.py --robots 10 --out webots/worlds/attabot_congreg10.wbt
    python tools/gen_world.py --robots 4  --arena 2.4,1.55 --leader 1 \
        --obstacles 750,500,150 750,1050,150 1800,775,150 --out ...
"""

import argparse
import json
import math
import os

# Paleta de cuerpos (se cicla): líder en gris claro, seguidores en colores.
LEADER_COLOR = '0.85 0.85 0.85'
COLORS = ['0.2 0.35 0.75', '0.7 0.25 0.7', '0.75 0.65 0.15', '0.2 0.6 0.3',
          '0.85 0.4 0.2', '0.3 0.55 0.8', '0.6 0.3 0.55', '0.5 0.5 0.2',
          '0.25 0.7 0.6', '0.8 0.55 0.35', '0.45 0.4 0.7', '0.7 0.5 0.5']

HEADER = '''#VRML_SIM R2025a utf8
# Mundo generado por gen_world.py — {n} robots, arena {w}×{h} m{obs_note}.
# Convención cam↔Webots: x_w=x_mm/1000−{hw:.3f} · y_w={hh:.3f}−y_mm/1000.

EXTERNPROTO "../protos/AttaBot.proto"

WorldInfo {{
  basicTimeStep 8
  contactProperties [
    ContactProperties {{
      material1 "attabot_body"
      coulombFriction [ 0.05 ]
    }}
    ContactProperties {{
      material1 "attabot_wheel"
      coulombFriction [ 3 ]
    }}
  ]
}}
Viewpoint {{
  orientation 0 1 0 {vpang:.4f}
  position {vpx:.3f} -0.077 {vpz:.3f}
}}
Background {{ skyColor [ 0.6 0.7 0.8 ] }}
DirectionalLight {{ direction 0.3 -0.2 -1 intensity 2.5 castShadows TRUE }}
'''

FLOOR = '''Solid {{
  translation 0 0 -0.01
  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.85 0.85 0.82 roughness 1 metalness 0 }}
    geometry Box {{ size {w} {h} 0.02 }} }} ]
  name "floor"
  boundingObject Box {{ size {w} {h} 0.02 }}
}}
'''


def wall(name, tx, ty, sx, sy):
    return (f'Solid {{\n  translation {tx:.3f} {ty:.3f} 0.05\n'
            f'  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.4 0.4 0.45 '
            f'roughness 1 metalness 0 }} geometry Box {{ size {sx:.3f} {sy:.3f} 0.1 }} }} ]\n'
            f'  name "{name}"\n  boundingObject Box {{ size {sx:.3f} {sy:.3f} 0.1 }}\n}}\n')


def robot(rid, tx, ty, color, rot=0.0):
    return (f'AttaBot {{\n  translation {tx:.4f} {ty:.4f} 0\n'
            f'  rotation 0 0 1 {rot:.4f}\n'
            f'  name "Atta_{rid}"\n  customData "{rid}"\n  bodyColor {color}\n}}\n')


OBSTACLE_HEIGHT_M = 0.20        # alto real de los obstáculos del lab (20cm)


def obstacle(idx, tx, ty, sx, sy=None, h=OBSTACLE_HEIGHT_M):
    """Caja del escenario. sy=None → cuadrada (compatibilidad).
    El alto no afecta la navegación (el IR ve cualquier cosa por encima del
    chasis) pero sí el video: con 10cm los obstáculos parecían alfombras."""
    if sy is None:
        sy = sx
    name = 'obstacle box' if idx == 0 else f'obstacle box {idx + 1}'
    return (f'Solid {{\n  name "{name}"\n  translation {tx:.3f} {ty:.3f} {h/2:.3f}\n'
            f'  children [ Shape {{ appearance PBRAppearance {{ baseColor 0.55 0.35 0.2 '
            f'roughness 1 metalness 0 }} geometry Box {{ size {sx:.3f} {sy:.3f} {h:.3f} }} }} ]\n'
            f'  boundingObject Box {{ size {sx:.3f} {sy:.3f} {h:.3f} }}\n}}\n')


def parse_obstacle(spec):
    """'x,y,lado' · 'x,y,ancho,alto' · con sufijo ',S' = ESTRUCTURAL.

    Estructural = pared del escenario (los bloques que acortan la barrera): se
    dibuja y bloquea igual, pero NO cuenta en el área de obstáculos. Sin esta
    distinción la ocupación reportada variaría entre escenarios que por diseño
    tienen la misma área de obstáculos.
    """
    parts = spec.split(',')
    structural = parts[-1].strip().upper() == 'S'
    if structural:
        parts = parts[:-1]
    v = [float(t) for t in parts]
    if len(v) == 3:
        return (v[0], v[1], v[2], v[2], structural)
    if len(v) == 4:
        return (v[0], v[1], v[2], v[3], structural)
    raise SystemExit(f'✗ obstáculo inválido: "{spec}" (x,y,lado | x,y,ancho,alto[,S])')


# Diámetro real del AttaBot medido en el lab (2026-07-28). OJO: la Base dibuja
# el cuerpo con radio 75mm (→150mm), que es el círculo envolvente con margen;
# para las métricas del paper vale la medida física.
ATTA_DIAMETER_MM = 105.0
ATTA_AREA_MM2 = math.pi * (ATTA_DIAMETER_MM / 2.0) ** 2


def draw_scenario(png, arena_w_mm, arena_h_mm, obstacles, metrics, title):
    """Planta a escala del escenario. Vale la pena que la dibuje el generador:
    así el diagrama que va al paper sale del mismo dato que el mundo simulado y
    no puede quedar desactualizado respecto de los obstáculos reales."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle

    fig, ax = plt.subplots(figsize=(arena_w_mm / 400.0, arena_h_mm / 400.0))
    ax.add_patch(Rectangle((0, 0), arena_w_mm, arena_h_mm, fill=False, lw=2))
    for (x, y, sx, sy, st) in obstacles:
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy,
                               facecolor='0.35' if st else '#8a5a2b',
                               edgecolor='k', lw=0.6))
    ax.add_patch(Circle((0, 0), ATTA_DIAMETER_MM / 2, color='none'))
    ax.set_xlim(-50, arena_w_mm + 50)
    ax.set_ylim(arena_h_mm + 50, -50)          # y hacia abajo, marco de cámara
    ax.set_aspect('equal')
    ax.set_title(f'{title} · ocup {metrics["occupancy_pct"]}% · '
                 f'pasaje efectivo {metrics.get("effective_passage_d")} d',
                 fontsize=9)
    ax.set_xlabel('mm')
    fig.tight_layout()
    fig.savefig(png, dpi=110)
    plt.close(fig)


def scenario_metrics(arena_w_mm, arena_h_mm, obstacles):
    """Caracteriza la topología de un escenario. `obstacles` = [(x,y,lado)] en mm
    de lab. Son las tres métricas de escenario del experimento de topología:

      occupancy_pct : área de obstáculos ÷ área de arena. Cuánto del escenario
                      está bloqueado, sin importar cómo se reparte.
      obstacle_area_d: área de CADA obstáculo ÷ área del Atta. Distingue "muchos
                      chicos" de "pocos macizos" a igual ocupación.
      passage_d     : holgura mínima del escenario ÷ diámetro del Atta. Se toma
                      el mínimo entre todos los pares de obstáculos y entre cada
                      obstáculo y cada pared. Para dos cajas alineadas a ejes la
                      holgura real es la distancia rectángulo-rectángulo
                      hypot(dx,dy): si están en diagonal el robot pasa por la
                      diagonal, y medir solo dx o dy subestimaría el pasaje.
    """
    # Acepta 4-tuplas (todo cuenta) o 5-tuplas con marca de estructural
    obstacles = [(o[0], o[1], o[2], o[3], o[4] if len(o) > 4 else False)
                 for o in obstacles]
    arena_area = arena_w_mm * arena_h_mm
    obs_area = sum(sx * sy for _x, _y, sx, sy, st in obstacles if not st)

    # Las cajas que se TOCAN son un solo cuerpo (union-find). Sin esto la
    # métrica se rompe en cuanto el escenario usa bloques estructurales: mediría
    # la distancia de un obstáculo a la pared aunque el bloque tape ese camino,
    # y reportaría un pasaje que no existe.
    n = len(obstacles)
    parent = list(range(n))
    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    def rect_gap(b1, b2):
        x1, y1, w1, h1 = b1[:4]
        x2, y2, w2, h2 = b2[:4]
        dx = max(0.0, abs(x1 - x2) - (w1 + w2) / 2)
        dy = max(0.0, abs(y1 - y2) - (h1 + h2) / 2)
        return math.hypot(dx, dy)
    for i in range(n):
        for j in range(i + 1, n):
            if rect_gap(obstacles[i], obstacles[j]) <= 1.0:
                parent[find(i)] = find(j)

    # ¿Qué paredes toca cada cuerpo? Si el cuerpo llega a una pared, el hueco
    # contra esa pared no es un pasaje para NINGUNA de sus cajas.
    touching = {}
    for i, (x, y, w, h, _st) in enumerate(obstacles):
        g = find(i)
        t = touching.setdefault(g, [False] * 4)
        if x - w / 2 <= 1.0:                   t[0] = True
        if y - h / 2 <= 1.0:                   t[1] = True
        if arena_w_mm - (x + w / 2) <= 1.0:    t[2] = True
        if arena_h_mm - (y + h / 2) <= 1.0:    t[3] = True

    gaps = []
    for i, (x, y, w, h, _st) in enumerate(obstacles):
        t = touching[find(i)]
        for k, d in enumerate((x - w / 2, y - h / 2,
                               arena_w_mm - (x + w / 2),
                               arena_h_mm - (y + h / 2))):
            if not t[k] and d > 1.0:
                gaps.append(d)
        for j in range(i + 1, n):
            if find(i) != find(j):             # mismo cuerpo → no hay pasaje
                gaps.append(rect_gap(obstacles[i], obstacles[j]))

    areas_d = sorted({round(sx * sy / ATTA_AREA_MM2, 2)
                      for _x, _y, sx, sy, st in obstacles if not st})
    eff = effective_passage(arena_w_mm, arena_h_mm, obstacles)
    return {
        'arena_mm': [arena_w_mm, arena_h_mm],
        'n_obstacles': sum(1 for o in obstacles if not o[4]),
        'n_structural': sum(1 for o in obstacles if o[4]),
        'occupancy_pct': round(100.0 * obs_area / arena_area, 2),
        'obstacle_area_d': areas_d,
        'passage_d': round(min(gaps) / ATTA_DIAMETER_MM, 2) if gaps else None,
        'passage_mm': round(min(gaps), 1) if gaps else None,
        # Métrica del paper: el cuello de botella de la ruta más holgada. Es la
        # que decide si el enjambre puede cruzar; `passage_mm` es el rincón más
        # apretado del escenario, esté o no en camino.
        'effective_passage_mm': eff,
        'effective_passage_d': round(eff / ATTA_DIAMETER_MM, 2),
    }


def effective_passage(arena_w_mm, arena_h_mm, obstacles, cell=20.0):
    """Ancho del CUELLO DE BOTELLA de la mejor ruta de un extremo al otro.

    El mínimo global entre pares encuentra el rincón más apretado del escenario
    aunque esté fuera de camino (medido: 335mm en un rincón mientras la ruta
    real tenía 420). Lo que define si el enjambre pasa es la ruta MÁS HOLGADA
    disponible, así que se busca el camino que maximiza su punto más angosto:
    holgura por celda + búsqueda binaria sobre el umbral con inundación.
    Devuelve el ANCHO del corredor (2× la holgura al centro del robot).
    """
    nx = max(2, int(arena_w_mm / cell))
    ny = max(2, int(arena_h_mm / cell))
    clear = [[0.0] * ny for _ in range(nx)]
    for i in range(nx):
        px = (i + 0.5) * arena_w_mm / nx
        for j in range(ny):
            py = (j + 0.5) * arena_h_mm / ny
            # Solo las paredes LATERALES (los flancos de la barrera). Las de
            # entrada y salida no acotan: el robot arranca y termina dentro de
            # la arena, y si contaran, la ruta nunca podría empezar.
            d = min(py, arena_h_mm - py)
            for o in obstacles:
                ox, oy, ow, oh = o[0], o[1], o[2], o[3]
                dx = max(0.0, abs(px - ox) - ow / 2)
                dy = max(0.0, abs(py - oy) - oh / 2)
                d = min(d, math.hypot(dx, dy) if (dx or dy) else 0.0)
            clear[i][j] = d

    def conecta(thr):
        """¿Hay ruta de la columna 0 a la última con holgura ≥ thr?"""
        seen = [[False] * ny for _ in range(nx)]
        stack = [(0, j) for j in range(ny) if clear[0][j] >= thr]
        for i, j in stack:
            seen[i][j] = True
        while stack:
            i, j = stack.pop()
            if i == nx - 1:
                return True
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = i + di, j + dj
                if 0 <= a < nx and 0 <= b < ny and not seen[a][b] \
                        and clear[a][b] >= thr:
                    seen[a][b] = True
                    stack.append((a, b))
        return False

    lo, hi = 0.0, max(arena_w_mm, arena_h_mm) / 2
    for _ in range(24):
        mid = (lo + hi) / 2
        if conecta(mid):
            lo = mid
        else:
            hi = mid
    return round(2 * lo, 1)


def scatter_poses(n, arena_w_mm, arena_h_mm, obstacles, seed, leader,
                  wall_margin=250.0, leader_wall_margin=600.0,
                  robot_sep=400.0, obstacle_clear=280.0):
    """Poses iniciales aleatorias reproducibles (protocolo del paper: cada
    repetición arranca en posición Y orientación distintas; la semilla hace la
    repetición re-ejecutable). Muestreo por rechazo con separaciones mínimas:
    entre robots, a paredes y al borde de cada caja. El LÍDER lleva un margen de
    pared mayor: los slots del anillo necesitan lugar a su alrededor y aunque
    ahora se auto-corrigen (wall_safe_slot), un líder encajonado fuerza rodeos
    que contaminarían la métrica de rutas con algo que no es la topología.
    Retorna [(rid, x_mm, y_mm, theta_rad_webots)].
    """
    import random as _rnd
    rng = _rnd.Random(seed)
    placed, out = [], []
    for rid in range(1, n + 1):
        margin = leader_wall_margin if rid == leader else wall_margin
        for _ in range(4000):
            x = rng.uniform(margin, arena_w_mm - margin)
            y = rng.uniform(margin, arena_h_mm - margin)
            if any(math.hypot(x - px, y - py) < robot_sep for px, py in placed):
                continue
            if any(abs(x - o[0]) < o[2] / 2 + obstacle_clear
                   and abs(y - o[1]) < o[3] / 2 + obstacle_clear
                   for o in obstacles):
                continue
            break
        else:
            raise SystemExit(f'✗ scatter: no hay lugar para el robot {rid} '
                             f'(seed {seed}) — arena muy llena')
        placed.append((x, y))
        out.append((rid, x, y, rng.uniform(0.0, 2.0 * math.pi)))
    return out


def solve_layout(arena_w_mm, arena_h_mm, occupancy_pct, side, gap,
                 clear_r=450.0):
    """Coloca obstáculos para CUMPLIR objetivos de topología, en vez de a ojo.

    Devuelve (obstáculos, métricas). La estructura es una malla centrada de
    cols×rows cajas de lado `side` separadas `gap` borde a borde — describible en
    una línea del paper y con holgura uniforme, que es lo que hace que el ancho
    de pasaje sea una propiedad del escenario y no un accidente de dónde cayó
    cada caja.

    Por qué una malla y no posiciones libres: el ancho de pasaje es un MÍNIMO
    sobre todos los pares y sobre las paredes, así que basta una caja mal puesta
    para que el escenario "amplio" tenga el pasaje más estrecho del set. Con
    malla, el mínimo es `gap` por construcción, siempre que el margen contra la
    pared no sea menor — por eso se centra y se exige margen ≥ gap.

    Malla AL TRESBOLILLO: las columnas impares van corridas media celda, así que
    cada caja queda detrás de un HUECO de la columna anterior y no detrás de otra
    caja. Con las columnas alineadas (como estaba) los huecos se encadenaban en
    pasillos rectos de un extremo al otro: el enjambre cruzaba en línea sin
    maniobrar y la topología dejaba de ser el factor bajo estudio — el robot
    nunca tenía que negociar un paso. Es la misma disposición que ya tenían a
    mano plan_sim.json y plan_lab.json.

    El mínimo entre pares NO cambia con el corrimiento: los vecinos de la misma
    columna siguen a `gap`, y el par en diagonal queda a hypot(gap, step/2−side)
    ≥ gap. O sea que `passage_mm` se sigue cumpliendo por construcción; lo que
    cambia es `effective_passage_mm`, que es justamente lo que se busca medir.

    `clear_r` reserva un disco libre en el centro: si no, el líder puede aparecer
    dentro de una caja. Las celdas de ese disco se descartan, lo que baja la
    ocupación, así que la búsqueda evalúa la ocupación REAL ya descontada.
    """
    step = side + gap
    cols_max = int((arena_w_mm - gap) // step)
    rows_max = int((arena_h_mm - gap) // step)
    if cols_max < 1 or rows_max < 1:
        return [], {'error': f'con pasaje {gap:.0f}mm y cajas de {side:.0f}mm '
                             f'no entra ninguna fila en la arena'}

    cx, cy = arena_w_mm / 2.0, arena_h_mm / 2.0
    target_area = occupancy_pct / 100.0 * arena_w_mm * arena_h_mm

    def build(cols, rows, dx, dy):
        ox = cx - (cols - 1) * step / 2.0 + dx
        oy = cy - (rows - 1) * step / 2.0 + dy
        # margen contra pared: debe quedar ≥ gap o el pasaje real sería ese
        if (ox - side / 2.0) < gap or (oy - side / 2.0) < gap:
            return None
        if (arena_w_mm - (ox + (cols - 1) * step + side / 2.0)) < gap:
            return None
        if (arena_h_mm - (oy + (rows - 1) * step + side / 2.0)) < gap:
            return None
        # Columnas impares corridas media celda y con una fila MENOS: así caen
        # exactamente en los huecos de las pares sin sobresalir del rectángulo
        # que ya validaron los márgenes de arriba.
        cells = []
        for i in range(cols):
            if i % 2 == 0:
                cells += [(ox + i * step, oy + j * step, side, side)
                          for j in range(rows)]
            else:
                cells += [(ox + i * step, oy + j * step + step / 2.0, side, side)
                          for j in range(rows - 1)]
        return [c for c in cells if math.hypot(c[0] - cx, c[1] - cy) > clear_r]

    best, best_key = None, None
    for cols in range(1, cols_max + 1):
        for rows in range(1, rows_max + 1):
            # medio paso de corrimiento: sin esto, con pasajes anchos las pocas
            # cajas que entran caen justo en el centro y el disco libre las borra
            for dx in (0.0, step / 2.0):
                for dy in (0.0, step / 2.0):
                    cells = build(cols, rows, dx, dy)
                    if not cells:
                        continue
                    m = scenario_metrics(arena_w_mm, arena_h_mm, cells)
                    # 1º respetar el pasaje pedido, 2º acercarse a la ocupación
                    passage_err = abs(m['passage_mm'] - gap)
                    area_err = abs(len(cells) * side * side - target_area)
                    key = (round(passage_err), area_err)
                    if best_key is None or key < best_key:
                        best, best_key = cells, key

    if best is None:
        return [], {'error': 'ninguna malla cumple margen de pared ≥ pasaje'}
    m = scenario_metrics(arena_w_mm, arena_h_mm, best)
    m['target_occupancy_pct'] = occupancy_pct
    m['target_passage_mm'] = gap
    # El solver NUNCA miente sobre lo logrado: si la arena no da, se reporta.
    m['meets_passage'] = abs(m['passage_mm'] - gap) <= 1.0
    m['meets_occupancy'] = abs(m['occupancy_pct'] - occupancy_pct) <= 1.0
    return best, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--robots', type=int, default=4)
    ap.add_argument('--arena', default='2.4,1.55', help='W,H en metros')
    ap.add_argument('--leader', type=int, default=1)
    ap.add_argument('--spread', default=None,
                    help='rx,ry (m) del anillo de arranque; por defecto 0.75·semieje')
    ap.add_argument('--obstacles', nargs='*', default=[],
                    help='cajas x,y,lado en mm de lab (ej: 750,500,150)')
    ap.add_argument('--solve', metavar='OCUP,LADO,PASAJE',
                    help='coloca los obstáculos para cumplir objetivos en vez de '
                         'darlos a mano: ocupación %%, lado de caja mm, ancho de '
                         'pasaje mm (ej: 8,150,300). Ignora --obstacles.')
    ap.add_argument('--scatter', action='store_true',
                    help='poses iniciales aleatorias (posición Y orientación) en '
                         'vez del líder-al-centro+elipse; protocolo del paper')
    ap.add_argument('--seed', type=int, default=None,
                    help='semilla del scatter — misma semilla = misma repetición')
    ap.add_argument('--start', metavar='x,y;x,y;...',
                    help='poses iniciales explícitas en mm de lab (una por robot, '
                         'en orden de id). Los robots miran hacia +x, o sea hacia '
                         'el destino del experimento de topología. Ignora --scatter.')
    ap.add_argument('--clear-center', type=float, default=450.0,
                    help='mm de disco libre en el centro (default 450 = anillo de '
                         'congregación + radio del robot), para que ningún robot '
                         'aparezca dentro de una caja')
    ap.add_argument('--out', required=True)
    ap.add_argument('--diagram', metavar='PNG', default=None,
                    help='dibuja el escenario a escala (planta) además del mundo')
    a = ap.parse_args()

    w, h = (float(v) for v in a.arena.split(','))
    hw, hh = w / 2.0, h / 2.0
    if a.spread:
        rx, ry = (float(v) for v in a.spread.split(','))
    else:
        rx, ry = 0.75 * hw, 0.75 * hh

    solved = None
    if a.solve:
        occ, side, gap = (float(v) for v in a.solve.split(','))
        solved, sm = solve_layout(w * 1000.0, h * 1000.0, occ, side, gap,
                                  a.clear_center)
        if 'error' in sm:
            raise SystemExit(f'✗ {sm["error"]}')
        # solve_layout emite (x, y, ancho, alto) — desempaquetar 3 reventaba con
        # "too many values to unpack": --solve estaba caído desde que las cajas
        # pasaron a poder ser rectangulares.
        a.obstacles = [f'{x:.0f},{y:.0f},{sx:.0f},{sy:.0f}'
                       for x, y, sx, sy in solved]
        print(f'Solver: objetivo ocup={occ}% lado={side:.0f}mm pasaje={gap:.0f}mm'
              f'  →  {sm["n_obstacles"]} cajas, ocup={sm["occupancy_pct"]}%, '
              f'pasaje={sm["passage_mm"]:.0f}mm ({sm["passage_d"]} d), '
              f'efectivo={sm["effective_passage_mm"]:.0f}mm '
              f'({sm["effective_passage_d"]} d)')

    obs_note = f', {len(a.obstacles)} obstáculo(s)' if a.obstacles else ''
    # Cámara: misma dirección de siempre, alejada proporcionalmente al escenario
    # y APUNTADA al centro de la arena. Con eje (0,1,0) y ángulo θ la vista mira
    # a (cosθ, 0, −senθ), así que apuntar al origen desde (−2.4k, ·, 3.8k) pide
    # θ = atan2(3.8k, 2.4k) — independiente de k. Antes el ángulo era fijo en
    # 1.2 rad (0.19 de más): la arena quedaba alta y se cortaba al grabar en 16:9.
    k = max(w / 2.4, h / 1.55)
    parts = [HEADER.format(n=a.robots, w=w, h=h, hw=hw, hh=hh, obs_note=obs_note,
                           vpx=-2.4 * k, vpz=3.8 * k,
                           vpang=math.atan2(3.8, 2.4))]
    parts.append(FLOOR.format(w=w, h=h))
    parts.append(wall('wall north', 0, hh + 0.01, w + 0.04, 0.02))
    parts.append(wall('wall south', 0, -(hh + 0.01), w + 0.04, 0.02))
    parts.append(wall('wall east', hw + 0.01, 0, 0.02, h))
    parts.append(wall('wall west', -(hw + 0.01), 0, 0.02, h))

    # Obstáculos (mm de lab → Webots)
    obs = [parse_obstacle(spec) for spec in a.obstacles]
    for i, (xm, ym, sx, sy, _st) in enumerate(obs):
        parts.append(obstacle(i, xm / 1000.0 - hw, hh - ym / 1000.0,
                              sx / 1000.0, sy / 1000.0))

    # Robots: scatter aleatorio con semilla (protocolo del paper) o el layout
    # clásico líder-al-centro + elipse de seguidores.
    if a.start:
        pts = [tuple(float(v) for v in p.split(',')) for p in a.start.split(';')]
        if len(pts) < a.robots:
            raise SystemExit(f'✗ --start trae {len(pts)} poses y hacen falta {a.robots}')
        for rid in range(1, a.robots + 1):
            xm, ym = pts[rid - 1]
            color = LEADER_COLOR if rid == a.leader else COLORS[(rid - 1) % len(COLORS)]
            parts.append(robot(rid, xm / 1000.0 - hw, hh - ym / 1000.0, color, 0.0))
    elif a.scatter:
        seed = a.seed if a.seed is not None else 0
        for rid, xm, ym, th in scatter_poses(a.robots, w * 1000.0, h * 1000.0,
                                             obs, seed, a.leader):
            color = LEADER_COLOR if rid == a.leader else COLORS[(rid - 1) % len(COLORS)]
            parts.append(robot(rid, xm / 1000.0 - hw, hh - ym / 1000.0,
                               color, th))
    else:
        followers = [r for r in range(1, a.robots + 1) if r != a.leader]
        parts.append(robot(a.leader, 0.0, 0.0, LEADER_COLOR))
        for k, rid in enumerate(followers):
            ang = 2.0 * math.pi * k / len(followers)
            parts.append(robot(rid, rx * math.cos(ang), ry * math.sin(ang),
                               COLORS[(rid - 1) % len(COLORS)]))

    parts.append('Robot {\n  name "base_camera"\n  controller "base_camera"\n'
                 '  supervisor TRUE\n}\n')

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, 'w') as f:
        f.write('\n'.join(parts))
    print(f'✓ {a.out} — {a.robots} robots (líder {a.leader}), arena {w}×{h} m'
          f'{obs_note}')

    # Caracterización de la topología, al lado del mundo. Que la escriba el
    # generador garantiza que el escenario reportado en el paper es exactamente
    # el que se simuló: no hay transcripción manual que se pueda desincronizar.
    m = scenario_metrics(w * 1000.0, h * 1000.0, obs)
    m['world'] = os.path.basename(a.out)
    m['robots'] = a.robots
    m['obstacles_mm'] = [list(o[:4]) for o in obs if not o[4]]
    m['structural_mm'] = [list(o[:4]) for o in obs if o[4]]
    if a.scatter:
        m['scatter_seed'] = a.seed if a.seed is not None else 0
    side = os.path.splitext(a.out)[0] + '.scenario.json'
    with open(side, 'w') as f:
        json.dump(m, f, indent=2, ensure_ascii=False)
    print(f'  ocupación={m["occupancy_pct"]}%  '
          f'área_obstáculo={m["obstacle_area_d"]} d²  '
          f'pasaje={m["passage_d"]} d ({m["passage_mm"]} mm)')
    print(f'  → {os.path.basename(side)}')

    if a.diagram:
        draw_scenario(a.diagram, w * 1000.0, h * 1000.0, obs, m,
                      os.path.basename(os.path.splitext(a.out)[0]))
        print(f'  → {a.diagram}')


if __name__ == '__main__':
    main()
