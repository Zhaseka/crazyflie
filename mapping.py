"""
2D-карта комнаты по дальномерам Multi-ranger (occupancy grid). Без cflib.

Каждый замер дальномера — луч из дрона:
  клетки вдоль луча становятся светлее (там пусто),
  клетка в конце луча — темнее (там стена).
Серое — туда ещё не смотрели.

Картинку сохраняем в PNG без matplotlib, точки стен — в CSV (облако точек).
"""

import math
import os
import struct
import zlib

import numpy as np

HIT_RANGE = 3.0   # дальше дальномер видит неуверенно: такие замеры не считаем стеной
FREE_RANGE = 2.0  # если ничего не видно, считаем пустым луч до этой длины
L_FREE = -0.4     # насколько «светлее» клетка, через которую прошёл луч
L_OCC = 0.9       # насколько «темнее» клетка, где луч упёрся
L_MAX = 5.0

# направление каждого дальномера относительно носа дрона, радианы (против часовой)
SENSOR_ANGLE = {'front': 0.0, 'left': math.pi / 2, 'back': math.pi, 'right': -math.pi / 2}


class OccupancyMap:
    def __init__(self, size=16.0, res=0.03):
        self.res = res
        self.n = int(size / res)
        self.grid = np.zeros((self.n, self.n), dtype=np.float32)  # log-odds, 0 = не знаем
        self.points = []  # точки стен (x, y) — облако точек

    def cell(self, x, y):
        """Клетка карты для точки (x, y) в метрах. Начало координат — в центре карты."""
        return int(round(x / self.res)) + self.n // 2, int(round(y / self.res)) + self.n // 2

    def _inside(self, i, j):
        return 0 <= i < self.n and 0 <= j < self.n

    def add_ray(self, x, y, angle, dist):
        """Луч из (x, y) под углом angle (в мире). dist — сколько показал датчик, None — ничего."""
        hit = dist is not None and dist <= HIT_RANGE
        length = dist if hit else min(dist or FREE_RANGE, FREE_RANGE)
        steps = int(length / (self.res / 2))
        dx, dy = math.cos(angle), math.sin(angle)
        seen = set()
        end = self.cell(x + dx * length, y + dy * length)
        for k in range(steps):
            c = self.cell(x + dx * k * self.res / 2, y + dy * k * self.res / 2)
            if c not in seen and c != end and self._inside(*c):
                seen.add(c)
                self.grid[c] = max(-L_MAX, self.grid[c] + L_FREE)
        if hit and self._inside(*end):
            self.grid[end] = min(L_MAX, self.grid[end] + L_OCC)
            self.points.append((x + dx * length, y + dy * length))

    def add_scan(self, x, y, yaw, ranges):
        """yaw — в радианах; ranges — {'front': м или None, ...}."""
        for name, rel in SENSOR_ANGLE.items():
            if name in ranges:
                self.add_ray(x, y, yaw + rel, ranges[name])

    def frontier(self):
        """Граница исследованного: пустые клетки рядом с серыми (не за стеной). Туда и надо лететь."""
        g = self.grid
        free, unknown, occ = g < -0.3, np.abs(g) < 0.1, g > 0.3
        return free & _grow(unknown, 1) & ~_grow(occ, 2)

    def count_near(self, mask, x, y, radius):
        """Сколько клеток маски в квадрате radius вокруг точки (x, y)."""
        i, j = self.cell(x, y)
        r = int(radius / self.res)
        return int(mask[max(0, i - r):i + r + 1, max(0, j - r):j + r + 1].sum())

    def image(self, scale=2, margin=0.5):
        """Серая картинка: белое — пусто, чёрное — стена, серое — неизвестно. Обрезана по исследованному."""
        prob = 1.0 / (1.0 + np.exp(-self.grid))
        img = (255 * (1.0 - prob)).astype(np.uint8)
        img[_grow(self.grid > 0.5, 1)] = 0  # уверенно найденные стены — чётко чёрным
        known = np.argwhere(np.abs(self.grid) > 0.1)
        if len(known):
            m = int(margin / self.res)
            (i0, j0), (i1, j1) = known.min(0) - m, known.max(0) + m
            img = img[max(0, i0):i1 + 1, max(0, j0):j1 + 1]
        # строки картинки — это y (сверху вниз), столбцы — x; переворачиваем, чтобы y смотрел вверх
        img = np.flipud(img.T)
        return np.kron(img, np.ones((scale, scale), dtype=np.uint8))

    def save_png(self, path):
        write_png(path, self.image())

    def save_points(self, path):
        with open(path, 'w') as f:
            f.write('x,y\n')
            for x, y in self.points:
                f.write(f'{x:.3f},{y:.3f}\n')


def _grow(mask, r):
    """Расширяет маску на r клеток во все стороны."""
    out = mask.copy()
    for di in range(-r, r + 1):
        for dj in range(-r, r + 1):
            out |= np.roll(np.roll(mask, di, 0), dj, 1)
    return out


def write_png(path, gray):
    """Записывает серую картинку (numpy uint8, h x w) в PNG без сторонних библиотек."""
    h, w = gray.shape
    raw = b''.join(b'\x00' + gray[r].tobytes() for r in range(h))

    def chunk(kind, data):
        return (struct.pack('>I', len(data)) + kind + data
                + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff))

    png = (b'\x89PNG\r\n\x1a\n'
           + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 6))
           + chunk(b'IEND', b''))
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(png)
    os.replace(tmp, path)  # чтобы VS Code не показал полузаписанный файл
