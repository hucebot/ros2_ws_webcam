#!/usr/bin/env python3

import cv2

# Lista dei dispositivi da testare
devices = ['/dev/video1', '/dev/video11', '/dev/video13']

# Parametri generali
width = 640
height = 480
fps = 15

caps = []

# Apro tutti i dispositivi
for dev in devices:
    cap = cv2.VideoCapture(dev)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    if cap.isOpened():
        print(f"[OK] Device {dev} aperto correttamente")
        caps.append((dev, cap))
    else:
        print(f"[ERROR] Device {dev} NON aperto")
        cap.release()

# Loop di test per leggere qualche frame
for i in range(100):
    for dev, cap in caps:
        ret, frame = cap.read()
        if ret:
            print(f"[{dev}] Frame ricevuto: {frame.shape}")
        else:
            print(f"[{dev}] NESSUN FRAME")

# Rilascio risorse
for dev, cap in caps:
    cap.release()

print("Test completato.")
