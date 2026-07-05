# AttaBot en Webots

Simulación 3D con física real (motores, encoders, gyro, colisiones robot-robot)
hablando el **mismo protocolo UDP del lab**. Los algoritmos de navegación y el
EKF son los mismos ports validados de `sim/` — Webots solo aporta la física.

## Arquitectura

```
┌────────────────────────┐  UDP 127.0.0.1:6060   ┌──────────────────────────┐
│ base_camera.py         │◄──────────────────────│ attabot_firmware.py (×N) │
│ (supervisor Webots)    │  REQUEST_POSITION     │ (controller por robot,   │
│ = cámara ArUco virtual │──────────────────────►│  puerto 6060+id)         │
│   + router de consola  │  POSITION_RESPONSE    │ ReactiveNav + EKF de sim/│
└────────────────────────┘  (pose + jitter σ30mm)└──────────────────────────┘
        ▲ comandos "robotId.instrucción" (mismo formato de la consola del lab)
        │ echo -n "1.POSITIONGT|1800|1000" | nc -u -w0 127.0.0.1 6060
```

- Marco de coordenadas = el "de cámara" del lab: origen esquina sup-izq,
  x→ancho, y→abajo, ángulo CW. Arena 2.4×1.55 m (el FOV de la C920).
- La base virtual agrega el jitter ArUco medido (σ=30mm, 2°) y soporta
  **oclusión de cámara** a demanda (`OCCLUDE.10`) para probar el EKF.
- `BROADCAST.EKF_NAV|1` conmuta la navegación de pose ArUco a pose EKF
  (el "GT semi-continuo" que se validará en hardware).

## Cómo correr

```bash
# GUI normal (si el paquete nativo funciona):
webots webots/worlds/attabot.wbt

# ⚠ En Arch, webots-bin (AUR) segfaultea al iniciar el rendering (Qt embebido
# vs Mesa rolling). Workaround: el flatpak, que trae su runtime completo:
flatpak install flathub com.cyberbotics.webots
flatpak run --filesystem=home com.cyberbotics.webots webots/worlds/attabot.wbt

# Headless (validación sin GUI):
flatpak run --filesystem=home com.cyberbotics.webots \
    --no-rendering --batch --minimize --mode=fast --stdout --stderr \
    webots/worlds/attabot.wbt
```

## Comandos de consola (UDP a 127.0.0.1:6060)

```bash
echo -n "1.POSITIONGT|1800|1000" | nc -u -w0 127.0.0.1 6060   # GT robot 1
echo -n "CONGREGATION.1"         | nc -u -w0 127.0.0.1 6060   # líder = robot 1
echo -n "OCCLUDE.10"             | nc -u -w0 127.0.0.1 6060   # cámara ciega 10 s
echo -n "BROADCAST.EKF_NAV|1"    | nc -u -w0 127.0.0.1 6060   # nav con pose EKF
echo -n "1.MOVE|500"             | nc -u -w0 127.0.0.1 6060
echo -n "2.TURN|180"             | nc -u -w0 127.0.0.1 6060
echo -n "BROADCAST.CANCEL_CONGREGATION" | nc -u -w0 127.0.0.1 6060
```

El supervisor imprime `POS|id|x|y|θ` (pose real, marco cámara) cada 2 s — la
fuente de ground truth para métricas.

## Asimetrías por robot (`robot_profiles.json`)

Cada robot simulado hereda la **personalidad medida de su robot real**
(calibraciones del lab 2026-06-18): error de escala del gyro + residuo de
calibración (`yaw_scale_cal`), residuo de PPR (`enc_scale`), desbalance de
motores (`motor_bias`) y el jitter ArUco de SU marca (`aruco_*_sigma` —
la marca de Atta_1 es peor que la de Atta_2, como en el lab). Sin perfil
para un id → robot ideal. Editá el JSON y ⏮ Reset para aplicar.

## Validado (2026-07-04, headless)

- GT con física real: llegada a **6mm** del goal, TURN con corrección iterativa.
- EKF a bordo: innovaciones de 30–37mm con jitter σ=30mm (sano).
- Congregación 2 robots con parking v2 (staging + slot del lado del follower).
- GT a ciegas (EKF_NAV + OCCLUDE 12s): evadió el obstáculo sin cámara y llegó
  a 16mm del goal; re-adquisición con innovación de 26mm.
- Con asimetrías activas: Atta_1 innov media 87mm / Atta_2 37mm (direccional-
  mente igual al lab: la marca mala castiga), y ambos llegan (31mm / 14mm).

## Pendiente / ideas

- Conectar el `AttaBot_Base.py` REAL (reemplazar la fuente de visión por el
  supervisor) para probar la base completa contra robots simulados.
- Marker ArUco texturizado + cámara cenital renderizada para probar la
  detección cv2 de verdad (hoy la pose viene del supervisor con ruido).
