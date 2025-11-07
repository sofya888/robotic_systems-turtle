# C:\ros2_ws\mk_map.py
# Генерим line_map.pgm + line_map.yaml из ASCII
from PIL import Image
import numpy as np
import textwrap
from pathlib import Path
import yaml

ascii_map = textwrap.dedent("""\
▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
▓░░░░░░░░░░░░░░░░░░░░░░░▓
▓░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░▓
▓░▓░░░░░░░░░░░░░░░░░░░▓▓
▓░▓░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░▓▓▓
▓░▓░▓░░░░░░░░░░░░░░░▓▓▓▓
▓░▓░▓░▓▓▓▓▓▓▓▓▓▓▓▓░▓▓▓▓▓
▓░▓░▓░▓░░░░░░░░░░░▓▓▓▓▓▓
▓░▓░▓░▓░▓▓▓▓▓▓▓▓░▓▓▓▓▓▓▓
▓░▓░▓░▓░▓░░░░░░░▓▓▓▓▓▓▓▓
▓░▓░▓░▓░▓░▓▓▓▓░▓▓▓▓▓▓▓▓▓
▓░▓░▓░▓░▓░▓░░░▓▓▓▓▓▓▓▓▓▓
▓░▓░▓░▓░▓░▓▓░▓▓▓▓▓▓▓▓▓▓▓
▓░▓░▓░▓░▓░░░▓▓▓▓▓▓▓▓▓▓▓▓
▓░▓░▓░▓░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
▓░▓░▓░▓░░░░░░░░░░░░░░░▓▓
▓░▓░▓░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░▓▓
▓░▓░▓░░░░░░░░░░░░░░░░░▓▓
▓░▓░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
▓░▓░░░░░░░░░░░░░░░░░░░▓▓
▓░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░▓▓
▓░░░░░░░░░░░░░░░░░░░░░▓▓
▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
""")

# где сохраняем
maps_dir = Path(r"C:\ros2_ws\src\line_follower\maps")
maps_dir.mkdir(parents=True, exist_ok=True)
pgm_path  = maps_dir / "line_map.pgm"
yaml_path = maps_dir / "line_map.yaml"

# переводим символы в картинку: '▓' -> черный (0), остальное -> белый (255)
lines = [list(row) for row in ascii_map.strip().splitlines()]
h = len(lines)
w = max(len(r) for r in lines)
img = np.full((h, w), 255, np.uint8)
for y, row in enumerate(lines):
    for x, ch in enumerate(row):
        if ch == '▓':
            img[y, x] = 0  # линия/стена — черным

# чуть масштабируем, чтобы карта была крупнее
scale = 4  # можно 3–6
img_big = Image.fromarray(img, mode='L').resize((w*scale, h*scale), Image.NEAREST)
img_big.save(pgm_path)  # Pillow сам сохранит как PGM

# параметры карты (0.08 м/пикс → ~ (w*scale*0.08) метров по ширине)
yaml.dump({
    'image': pgm_path.name,
    'resolution': 0.08,
    'origin': [0.0, 0.0, 0.0],
    'occupied_thresh': 0.65,
    'free_thresh': 0.196,
    'negate': 0,
}, open(yaml_path, 'w', encoding='utf-8'))

print("OK:", pgm_path, yaml_path)
