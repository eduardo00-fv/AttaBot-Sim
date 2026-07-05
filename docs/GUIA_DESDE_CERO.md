# Guía desde cero: replicar la simulación AttaBot en Webots

Cómo se construyó esta simulación, paso a paso, asumiendo cero experiencia
con Webots. Cada sección explica QUÉ se hizo, POR QUÉ, y CÓMO reproducirlo.

---

## 0. Conceptos de Webots (lo mínimo indispensable)

**Webots** es un simulador de robots con motor de física (ODE). Sus piezas:

| Concepto | Qué es | En nuestro proyecto |
|---|---|---|
| **Mundo** (`.wbt`) | Archivo de texto que describe la escena: piso, paredes, robots, luces | `webots/worlds/attabot.wbt` |
| **Nodo** | Cada cosa en el mundo (un `Solid`, un `Robot`, una luz). Se anidan en árbol | paredes, obstáculo, robots |
| **PROTO** (`.proto`) | Una "clase" de nodo reutilizable con parámetros — definís el robot una vez, lo instanciás N veces | `webots/protos/AttaBot.proto` |
| **Controller** | Programa (Python/C++) que corre UNO por robot y habla con sus motores/sensores vía la API `controller` | `attabot_firmware.py` |
| **Supervisor** | Un Robot especial cuyo controller puede leer/modificar TODO el mundo (poses reales, resetear, etc.) | `base_camera.py` |
| **basicTimeStep** | El paso de integración física en ms (nosotros: 8ms; los controllers muestrean cada 16ms) | `WorldInfo` |

**El ciclo de vida**: Webots carga el mundo → lanza un proceso por controller →
cada controller hace `robot.step(dt)` en un loop (avanza la física y refresca
sensores) → al presionar ⏮ Reset, los controllers se relanzan.

**Unidades**: metros, radianes, sistema de coordenadas z-arriba (ENU).
Nuestro sistema del lab usa mm, grados, y-hacia-abajo — las conversiones son
parte del diseño (sección 5).

## 1. La decisión de arquitectura (leer antes de tocar nada)

La tentación era "hacer una simulación del AttaBot". Lo que se hizo en cambio:
**replicar el SISTEMA del lab**, con sus tres actores y su protocolo real:

```
   LAB REAL                        WEBOTS
   ────────                        ──────
   Robot ESP32 (firmware C++)  →   attabot_firmware.py (uno por robot)
   Cámara C920 + AttaBot_Base  →   base_camera.py (supervisor)
   UDP puerto 6060             →   UDP 127.0.0.1:6060 (¡el mismo protocolo!)
```

Reglas que hicieron esto barato y confiable:

1. **No reimplementar algoritmos**: la navegación (`ReactiveNav`) y el EKF ya
   estaban portados y validados en `sim/` (la sim 2D). Los controllers los
   IMPORTAN. Webots solo aporta la física.
2. **El mismo protocolo UDP del lab**: `POSITIONGT|x|y`, `REQUEST_POSITION`,
   `POSITION_RESPONSE|x|y|θ`, `CONGREGATION|líder|idx|n`… Un robot simulado y
   uno real son indistinguibles para la base. El día que se quiera, la base
   REAL puede manejar robots simulados.
3. **El mundo de los robots es el marco del lab**: los robots nunca ven
   coordenadas Webots. Solo la base virtual convierte.

## 2. Estructura de proyecto

Webots impone convenciones de carpetas y nombres:

```
webots/
├── worlds/attabot.wbt          # el mundo
├── protos/AttaBot.proto        # el modelo del robot
└── controllers/
    ├── attabot_firmware/
    │   └── attabot_firmware.py # ¡DEBE llamarse igual que su carpeta!
    └── base_camera/
        └── base_camera.py
```

El campo `controller "attabot_firmware"` de un Robot busca
`controllers/attabot_firmware/attabot_firmware.py` relativo al mundo.

## 3. El mundo (attabot.wbt)

Empieza con encabezado + imports de PROTOs:

```vrml
#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/AttaBot.proto"
```

Piezas, en orden:

1. **`WorldInfo`**: `basicTimeStep 8` y las `contactProperties` — pares de
   materiales con su fricción. Clave para el robot diferencial:
   ruedas con fricción ALTA (`attabot_wheel`, μ=3), cuerpo/casters con
   fricción CASI CERO (`attabot_body`, μ=0.05) para que los apoyos deslicen.
2. **`Viewpoint`** + **`DirectionalLight`**: cámara inicial y luz.
3. **Piso y 4 paredes**: nodos `Solid` con `Shape` (lo visible) +
   `boundingObject` (la geometría de colisión) y SIN nodo `physics`
   (= estáticos, masa infinita). Arena de 2.4×1.55m = el FOV de la C920.
4. **Obstáculo**: otro `Solid` tipo caja.
5. **Robots**: instancias del PROTO con su pose inicial y su id:
   ```vrml
   AttaBot { translation -0.9 0.475 0  name "Atta_1"  customData "1" }
   ```
   `customData` es un string libre que el controller puede leer — lo usamos
   como id del robot (define su puerto UDP: 6060+id).
6. **La base virtual**: `Robot { name "base_camera" controller "base_camera" supervisor TRUE }`
   — un robot sin cuerpo, invisible, pero con poderes de supervisor.

## 4. El robot (AttaBot.proto) — modelado físico

Medidas del firmware real: cuerpo cilíndrico r=75mm, ruedas Ø44.5mm sobre el
eje y a ±41.5mm del centro, IR frontal y a ±30°.

Anatomía del PROTO:

- **Cuerpo**: `Pose { children [ Shape { Cylinder } ] }` elevado para que no
  toque el piso.
- **Cada rueda** es el patrón fundamental de Webots para articulaciones:
  ```
  HingeJoint {
    jointParameters HingeJointParameters { axis 0 1 0  anchor 0 0.0415 0.02225 }
    device [
      RotationalMotor  { name "left wheel motor"  maxVelocity 25 }
      PositionSensor   { name "left wheel sensor" }     # ← el encoder
    ]
    endPoint Solid { ... rueda con boundingObject y physics ... }
  }
  ```
  El `PositionSensor` de un motor rotacional ES el encoder: devuelve radianes
  acumulados. mm = Δrad × radio_rueda.
- **Casters**: en vez de modelar una rueda loca (articulación esférica,
  frágil), dos ESFERAS fijas en el `boundingObject` del cuerpo con fricción
  ~0 (vía `contactMaterial "attabot_body"`). Truco estándar (así lo hace el
  e-puck oficial).
- **IR**: `DistanceSensor` con `lookupTable [0 0 0.02, 0.25 0.25 0.02]`
  (devuelve metros con 2% de ruido, alcance 0.25m). El controller marca
  obstáculo si lee < 0.20m.
- **IMU**: nodo `Gyro` — devuelve velocidad angular (rad/s). Se INTEGRA en el
  controller para obtener yaw, igual que el DMP del ICM-20948 real.
- **`physics Physics { density -1 mass 0.35 }`**: masa explícita (350g).

## 5. Los dos marcos de coordenadas (la parte que más confunde)

| | Webots (mundo) | Lab ("cámara") |
|---|---|---|
| Unidades | m, rad | mm, grados |
| Eje y | arriba (norte) | ABAJO (imagen de cámara) |
| Ángulo | CCW positivo | **CW positivo** |
| Origen | centro de la arena | esquina superior izquierda |

**Regla de diseño: los robots viven 100% en el marco del lab.** Solo hay dos
puntos de conversión, y los dos son de 3 líneas:

1. **La base virtual** convierte la pose Webots → lab al responder:
   ```python
   x_lab = (x_w + 1.2) * 1000          # m→mm, origen a la esquina
   y_lab = (0.775 - y_w) * 1000        # y invertida
   θ_lab = (-grados(φ_w)) % 360        # CCW → CW
   ```
2. **El gyro** en el controller: `dθ_lab = −ω_z·dt` (mismo flip de signo).

Con eso, la cinemática en el marco lab queda estándar
(`x += d·cosθ, y += d·sinθ`) y TODO el código de `sim/` funciona sin tocar.

## 6. El controller del robot (attabot_firmware.py)

Es una réplica funcional del firmware ESP32, ~350 líneas:

1. **Setup**: leer `customData` (id) → bind UDP en `6060+id`; obtener devices
   (`getDevice('left wheel motor')`…); `setPosition(inf)` a los motores =
   modo velocidad.
2. **Loop principal** (cada 16ms):
   ```
   robot.step(dt)          # avanza la física
   sensor_tick()           # gyro→yaw, encoders→Δd, EKF.predict()
   motion_tick()           # máquina de estados TURN/MOVE/SETTLE
   procesar UDP entrante   # socket no-bloqueante
   lógica de nav pendiente # reintentos, blind-continue, telemetría
   ```
3. **TURN closed-loop** (port del fix validado en hardware el 18/06):
   girar hasta objetivo − 3° de brake-lead → parar → settle 300ms → medir
   error → si |err|>3°, corrección iterativa SIN lead (hasta 4).
4. **Ciclo GT** (idéntico al firmware): `REQUEST_POSITION` → llega
   `POSITION_RESPONSE` → `nav.step_pose()` decide (giro, avance) → encolar
   TURN+MOVE → al terminar, repetir. La navegación es el `EKFNav` importado
   de `sim/ekf_sim.py`.
5. **EKF a bordo**: `EKF.predict(Δd_encoders, Δθ_gyro)` cada tick;
   `update_aruco()` con cada respuesta de la base; la innovación (distancia
   predicción↔medición) se reporta — es LA métrica de salud del filtro.
6. **Modo `EKF_NAV|1`**: si la cámara calla >1s, la nav continúa con
   `ekf.pose()` en vez de esperar (el "GT semi-continuo" que irá al firmware).

## 7. La base virtual (base_camera.py)

Supervisor que reemplaza cámara + consola del lab (~120 líneas):

- **Descubrir robots**: recorrer los hijos del root del árbol de escena y
  quedarse con los nodos de tipo `AttaBot`; leer su `customData`.
- **Pose real**: `node.getPosition()` y `node.getOrientation()` (matriz 3×3;
  yaw = `atan2(R[3], R[0])`) → convertir al marco lab (sección 5).
- **Responder `REQUEST_POSITION`** con la pose + ruido gaussiano del jitter
  ArUco medido en el lab (σ=30mm, 2°; por-robot desde `robot_profiles.json`).
- **Router de consola**: mensajes UDP con formato `robotId.instrucción`
  (idéntico a la consola del lab) se reenvían al puerto del robot destino.
  Comandos propios: `OCCLUDE.n` (n segundos sin responder posiciones — simula
  taparle el lente a la cámara) y `CONGREGATION.líder`.
- **Ground truth**: imprime `POS|id|x|y|θ` cada 0.5s — la verdad absoluta
  para métricas y gráficas (algo que en el lab NO existe; en sim es gratis).

## 8. Asimetrías por robot (robot_profiles.json)

Los robots reales no son ideales — cada uno tiene su personalidad medida en
lab. Se inyecta cada error DONDE OCURRE en la realidad:

| Error real | Dónde se inyecta |
|---|---|
| Gyro con escala imperfecta (Atta_1 sub-lee 2.3%) | `gyro_scale` multiplica la lectura del Gyro |
| Calibración CALIBRATE con residuo | `yaw_scale_cal` multiplica después (como el firmware) |
| PPR mal calibrado | `enc_scale` multiplica los mm de encoder |
| Motores desparejos → deriva | `motor_bias` desbalancea las velocidades comandadas |
| Marca ArUco ruidosa (la de Atta_1 es peor) | `aruco_pos/ang_sigma` en la base virtual |

El EKF y la navegación NO saben de esto — sufren y compensan, igual que en el
lab. Robot sin perfil = ideal.

## 9. Correr y validar

```bash
# GUI (ver y jugar):
flatpak run com.cyberbotics.webots webots/worlds/attabot.wbt
python3 webots/console.py      # en otra terminal: 1.POSITIONGT|1800|1000

# Headless (validación batch, más rápido que tiempo real):
flatpak run --filesystem=home com.cyberbotics.webots \
  --no-rendering --batch --minimize --mode=fast --stdout --stderr \
  webots/worlds/attabot.wbt

# Sin Webots (validar la LÓGICA de los controllers en segundos):
python3 tools/mock_webots/harness.py
```

El harness merece explicación: `tools/mock_webots/controller.py` es un
**mock del módulo `controller` de Webots** con física cinemática mínima.
Como el firmware simulado solo conoce la API (`getDevice`, `step`…), se le
puede dar el mock en vez del Webots real y validar todo el protocolo UDP,
la máquina de estados y el EKF sin abrir la GUI. Así se detectaron varios
bugs antes de la primera corrida real.

Experimento estrella (EKF bajo oclusión):

```
BROADCAST.EKF_NAV|1
1.POSITIONGT|1800|1000
(esperar ~6 s)
OCCLUDE.12
```

y graficarlo: `python3 tools/plot_ekfnav.py <log> salida.png`.

## 10. Los tropiezos reales (para no repetirlos)

1. **`webots-bin` de AUR segfaultea** al iniciar el rendering en Arch rolling
   (Qt embebido vs Mesa del sistema, híbrido Intel+NVIDIA). Solución: el
   **flatpak** `com.cyberbotics.webots`, que trae su runtime completo.
   Darle acceso al home: `flatpak override --user --filesystem=home com.cyberbotics.webots`.
2. **Una sola instancia del mundo a la vez**: los puertos UDP 6060+N son del
   sistema, no de Webots — dos instancias = "Address already in use".
3. **Correr headless desde scripts**: exportar `XAUTHORITY`, `DISPLAY=:0` y
   `XDG_RUNTIME_DIR=/run/user/1000` (las shells no interactivas no los tienen).
4. **matplotlib no existe dentro del flatpak**: por eso `sim/attabot_sim.py`
   y `sim/ekf_sim.py` lo importan con try/except — los controllers solo
   necesitan las clases, no los plots.
5. **El nombre del controller DEBE coincidir con su carpeta** (sección 2).
6. **Guardar el mundo solo tras un Reset** — si no, los robots quedan
   "guardados" donde estaban parados.

## Referencias

- Webots User Guide: https://cyberbotics.com/doc/guide/
- Webots Reference (nodos/API): https://cyberbotics.com/doc/reference/
- Tutorial oficial (el 1-6 cubre todo lo de aquí): https://cyberbotics.com/doc/guide/tutorials
