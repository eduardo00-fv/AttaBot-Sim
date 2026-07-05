#!/usr/bin/env python3
"""Consola interactiva para la base virtual de Webots — mismo formato que la
consola del lab (robotId.instrucción). Correr en una terminal aparte mientras
Webots está abierto:

    python3 webots/console.py

Ejemplos:
    1.POSITIONGT|1800|1000     GT del robot 1
    CONGREGATION.1             congregación con líder 1
    OCCLUDE.10                 cámara ciega 10 s
    BROADCAST.EKF_NAV|1        navegación con pose EKF (todos)
    1.MOVE|500                 avance de 500 mm
    2.TURN|180                 giro de 180°
    BROADCAST.CANCEL_CONGREGATION
"""
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
print(__doc__)
while True:
    try:
        line = input('webots> ').strip()
    except (EOFError, KeyboardInterrupt):
        print()
        break
    if not line:
        continue
    if line in ('exit', 'quit'):
        break
    sock.sendto(line.encode(), ('127.0.0.1', 6060))
