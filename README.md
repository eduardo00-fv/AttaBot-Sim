# AttaBot-Sim

Ecosistema de simulación del enjambre [Atta-Bot](https://github.com/jcbrenes/Atta-Bot)
(TEC): simulación 2D rápida, simulación 3D con física en Webots, y herramientas
de validación/análisis. Separado del repo del robot para no mezclar la
infraestructura de simulación (tesis) con el firmware/base del lab.

Extraído de [`Atta-Bot-P_ed`](https://github.com/eduardo00-fv/Atta-Bot-P_ed)
en su estado `e7077d6` (2026-07-04) — la historia previa de estos archivos
vive allá, en la rama `EKF_Descentralizado`.

## Estructura

```
sim/       Simulación 2D (Python+matplotlib, sin dependencias de robot)
           - attabot_sim.py : port 1:1 de ReactiveNavStep del firmware
           - ekf_sim.py     : EKF [x,y,θ] — espejo exacto del EKFState de utils.h
webots/    Simulación 3D con física (Webots R2025a)
           - protos/AttaBot.proto  : el robot con dimensiones reales
           - worlds/attabot.wbt    : arena 2.4×1.55m (FOV del lab)
           - controllers/          : "firmware" por robot + base/cámara virtual
           - robot_profiles.json   : personalidades medidas de los robots reales
           - TUTORIAL.md / README.md : cómo usar
tools/     - plot_ekfnav.py   : gráfica de experimentos EKF+oclusión desde logs
           - mock_webots/     : harness que valida los controllers SIN Webots
                                (mock del módulo controller + física mínima)
```

## Relación con el repo del robot

- Los controllers de Webots hablan **el mismo protocolo UDP** del sistema real
  (puerto 6060, `POSITIONGT`, `CONGREGATION`, `REQUEST_POSITION`…) — la
  referencia de protocolo es `Controller/AttaBot/AttaBot.ino` del repo del robot.
- `sim/ekf_sim.py` y el `EKFState` de `utils.h` (firmware) son espejos: cualquier
  cambio en uno debe replicarse en el otro (validar con `tools/mock_webots/` y
  la cross-validación numérica).
- `webots/robot_profiles.json` guarda las calibraciones/ruidos MEDIDOS en lab
  por robot — actualizarlo cuando se recalibre.

## Quickstart

```bash
# Sim 2D (validación de algoritmos, segundos):
python3 sim/ekf_sim.py --scenario montecarlo

# Webots (física, GUI):
flatpak run com.cyberbotics.webots webots/worlds/attabot.wbt
python3 webots/console.py          # consola de comandos (otra terminal)

# Validar controllers sin abrir Webots:
python3 tools/mock_webots/harness.py
```

En Arch: usar el **flatpak** de Webots (`com.cyberbotics.webots`) — el paquete
AUR `webots-bin` falla al iniciar el rendering. Detalles en `webots/TUTORIAL.md`.
