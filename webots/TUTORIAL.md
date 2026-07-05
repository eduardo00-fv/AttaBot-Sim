# Tutorial rápido de Webots para AttaBot

Guía mínima para manejar la simulación sin haber usado Webots antes.
Todo está pensado para el mundo `webots/worlds/attabot.wbt`.

## 1. Abrir

```bash
flatpak run com.cyberbotics.webots webots/worlds/attabot.wbt
```

o desde el menú de aplicaciones ("Webots") y luego `File → Open World`.
(El acceso al home ya quedó configurado con `flatpak override`.)

⚠ NO uses el `webots` de AUR (`/usr/bin/webots`) — segfaultea en esta máquina.

⚠ **Una sola instancia a la vez**: los robots usan los puertos UDP 6060–606N;
si abrís un segundo Webots con el mismo mundo, sus controllers mueren con
"puerto ocupado". Cerrá el sobrante (`flatpak kill com.cyberbotics.webots`)
y dale ⏮ Reset al que quede.

## 2. La ventana

```
┌───────────────┬──────────────────────────────┐
│ Scene Tree    │                              │
│ (árbol de     │        Vista 3D              │
│  la escena)   │                              │
│               │                              │
├───────────────┴──────────────────────────────┤
│ Consola (stdout de los controllers)          │
└──────────────────────────────────────────────┘
```

- **Scene Tree (izquierda)**: cada nodo del mundo. Ahí están `Atta_1`,
  `Atta_2`, las paredes, el obstáculo y `base_camera` (la base virtual).
  Al expandir un nodo ves/editás sus campos (`translation`, `customData`…).
- **Vista 3D (centro)**: el mundo físico.
- **Consola (abajo)**: TODO el stdout — los `DEBUG` de los robots, los `POS`
  del supervisor, los `EKF innov`. Es el equivalente del ConsoleLog del lab.

## 3. Moverse en la vista 3D

| Acción | Cómo |
|---|---|
| Rotar la cámara | arrastrar con **click izquierdo** |
| Desplazar (pan) | arrastrar con **click derecho** |
| Zoom | **rueda** del mouse |
| Seleccionar un robot | click sobre él (se resalta y se marca en el árbol) |
| Seguir al robot con la cámara | seleccionarlo → menú **View → Follow Object** |
| Perdiste la cámara | **View → Reset Viewpoint** |

## 4. Controles de simulación (barra superior)

- ⏸ **Pausa** / ▶ **Tiempo real** / ⏩ **Fast** (tan rápido como dé el CPU —
  útil para experimentos largos; los robots se mueven igual, solo el reloj corre más rápido).
- ⏮ **Reset**: devuelve el mundo al estado inicial **y relanza los controllers**
  (es la forma de "reiniciar los robots" después de editar código Python).
- El contador muestra el tiempo simulado y el factor de velocidad (ej. `1.00x`).

## 5. Comandar los robots (la "base" del lab)

Con Webots corriendo, en OTRA terminal:

```bash
python3 webots/console.py
```

y escribís comandos con el mismo formato de la consola del lab:

```
1.POSITIONGT|1800|1000      # GT del robot 1
CONGREGATION.1              # congregación, líder = robot 1
OCCLUDE.10                  # cámara ciega 10 s (¡mirá el robot seguir!)
BROADCAST.EKF_NAV|1         # navegación con pose EKF (el modo "a ciegas")
1.MOVE|500                  # primitivas sueltas
2.TURN|180
BROADCAST.CANCEL_CONGREGATION
```

La respuesta de los robots aparece en la **consola de Webots** (abajo).

### El experimento estrella (EKF bajo oclusión)

```
BROADCAST.EKF_NAV|1
1.POSITIONGT|1800|1000
(esperar ~6 s)
OCCLUDE.12
```

El robot sigue navegando sin cámara usando encoders+gyro fusionados por el
EKF. Sin `EKF_NAV|1`, con la cámara ocluida se queda esperando (el
comportamiento del firmware actual). Graficarlo: `plot_ekfnav.py` sobre el
log (ver README).

## 6. Visualizaciones útiles (menú View → Optional Rendering)

- **Show DistanceSensor Rays** — dibuja los rayos IR de cada robot (rojos al
  detectar). LA opción más útil para entender las evasiones.
- **Show Contact Points** — puntos de contacto físico (¿se están tocando?).
- **Show Bounding Objects** — la geometría de colisión real (cilindro+casters).

## 7. Editar el mundo

- **Mover un robot/obstáculo**: seleccionalo y editá `translation` en el árbol
  (o arrastrá las flechas de colores que aparecen al seleccionarlo).
  Recordá el mapeo: `x_webots = x_cam/1000 − 1.2`, `y_webots = 0.775 − y_cam/1000`.
- **Agregar otro obstáculo**: click derecho sobre `obstacle box` en el árbol →
  Copy → Paste, y cambiale `translation`.
- **Agregar otro robot**: botón `+` (Add) → `PROTO nodes (Current Project)` →
  `AttaBot`. Cambiale `name` (Atta_3), `customData` ("3") y `translation`.
  El puerto UDP será 6060+3 automáticamente.
- **Guardar**: `File → Save World` — ⚠ SOLO después de un ⏮ Reset, si no
  guardás los robots donde quedaron parados.

## 8. Editar el código de los robots

- `webots/controllers/attabot_firmware/attabot_firmware.py` — el "firmware".
- `webots/controllers/base_camera/base_camera.py` — la cámara/base virtual.
- Después de editar: **⏮ Reset** en Webots (relanza los controllers).
  Si tocaste el PROTO o el .wbt: `File → Reload World` (Ctrl+Shift+R).

## 9. Correr sin GUI (validaciones automáticas)

```bash
flatpak run com.cyberbotics.webots --no-rendering --batch --minimize \
    --mode=fast --stdout --stderr webots/worlds/attabot.wbt
```

Los `POS|id|x|y|θ` que imprime el supervisor son el ground truth para métricas.
