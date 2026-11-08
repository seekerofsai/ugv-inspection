#!/usr/bin/env python3
"""Generate printable ArUco targets; run in a Python environment with cv2.aruco."""
import argparse
from pathlib import Path
import json
import cv2

p = argparse.ArgumentParser()
p.add_argument('--config',type=Path,default=Path(__file__).resolve().parents[1]/'config/panel.json')
p.add_argument('--output',type=Path,required=True)
a = p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
for t in json.loads(a.config.read_text())['targets']:
    marker = cv2.aruco.generateImageMarker(d,t['id'],600) if hasattr(cv2.aruco,'generateImageMarker') else cv2.aruco.drawMarker(d,t['id'],600)
    marker = cv2.copyMakeBorder(marker,100,100,100,100,cv2.BORDER_CONSTANT,value=255)
    cv2.imwrite(str(a.output/f'target-{t["id"]}.png'),marker)
print(f'Print markers from {a.output}; retain the white border and label them with target names.')
