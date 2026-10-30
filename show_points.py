"""
Рисует облако точек стен из points.csv (его сохраняет explore.py) в картинку points.png.

explore.py рисует points.png сам в конце полёта; этот скрипт — чтобы перерисовать вручную.
Запуск:  python3 show_points.py               — points.csv -> points.png
         python3 show_points.py sim_points.csv — любой другой файл -> <имя>.png
Чёрные точки — стены, серая сетка — через каждый метр, крестик — точка взлёта.
"""

import csv
import sys

import numpy as np

from mapping import write_png

RES = 0.01     # м на пиксель: 1 пиксель = 1 см
MARGIN = 0.3   # м пустого места вокруг точек


def render_points(points, dst):
    """Рисует точки стен [(x, y), ...] в картинку dst. Её же вызывает explore.py в конце полёта."""
    xs = [p[0] for p in points] + [0.0]  # + точка взлёта, чтобы она точно попала на картинку
    ys = [p[1] for p in points] + [0.0]
    x0, y0 = min(xs) - MARGIN, min(ys) - MARGIN
    w = int((max(xs) + MARGIN - x0) / RES) + 1
    h = int((max(ys) + MARGIN - y0) / RES) + 1
    img = np.full((h, w), 255, dtype=np.uint8)

    def pixel(x, y):
        # строки картинки идут сверху вниз, поэтому y переворачиваем: вперёд (x) — вправо, влево (y) — вверх
        return h - 1 - int((y - y0) / RES), int((x - x0) / RES)

    # сетка через каждый метр
    for gx in range(int(np.floor(x0)), int(np.ceil(x0 + w * RES)) + 1):
        c = pixel(gx, y0)[1]
        if 0 <= c < w:
            img[:, c] = 200
    for gy in range(int(np.floor(y0)), int(np.ceil(y0 + h * RES)) + 1):
        r = pixel(x0, gy)[0]
        if 0 <= r < h:
            img[r, :] = 200

    # точки стен — квадратики 3×3 пикселя
    for x, y in points:
        r, c = pixel(x, y)
        img[max(0, r - 1):r + 2, max(0, c - 1):c + 2] = 0

    # крестик в точке взлёта
    r, c = pixel(0.0, 0.0)
    img[max(0, r - 12):r + 13, max(0, c - 2):c + 3] = 90
    img[max(0, r - 2):r + 3, max(0, c - 12):c + 13] = 90

    write_png(dst, img)
    print(f'{len(points)} точек -> {dst} ({w * RES:.1f} × {h * RES:.1f} м, сетка — 1 м, '
          'крестик — точка взлёта)')


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else 'points.csv'
    with open(src) as f:
        points = [(float(row['x']), float(row['y'])) for row in csv.DictReader(f)]
    if not points:
        sys.exit(f'В {src} нет точек')
    render_points(points, src.rsplit('.', 1)[0] + '.png')


if __name__ == '__main__':
    main()
