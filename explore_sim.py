"""
Симулятор режима исследования: проверка логики и карты без дрона.

Запуск:  python3 explore_sim.py
Результат: sim_map.png (карта) и sim_points.csv (облако точек стен).

Комната — набор стен-отрезков, дрон — точка, 4 дальномера — лучи вперёд/назад/влево/вправо.
"""

import math
import random

from explorer import MAX_RANGE, ExploreConfig, Explorer
from mapping import OccupancyMap
from show_points import render_points

DT = 0.1
NOISE = 0.01  # шум дальномера, м


def rect(x0, y0, x1, y1):
    return [((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))]


# комната 7 × 5.5 м: перегородка с проходом, стол, шкаф, пара колонн
WALLS = (
    rect(-1.0, -2.0, 6.0, 3.5)
    + rect(2.0, 1.15, 5.0, 1.25)          # перегородка, справа проход
    + rect(3.0, -1.2, 4.0, -0.6)          # стол
    + rect(-0.9, 2.6, 0.2, 3.4)           # шкаф в углу
    + rect(1.0, -0.9, 1.2, -0.7)          # колонна
    + rect(4.5, 2.4, 4.7, 2.6)            # колонна за перегородкой
)


def cast(x, y, angle):
    """Расстояние от (x, y) до ближайшей стены по лучу angle (радианы)."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = math.inf
    for (x1, y1), (x2, y2) in WALLS:
        ex, ey = x2 - x1, y2 - y1
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((x1 - x) * ey - (y1 - y) * ex) / den
        u = ((x1 - x) * dy - (y1 - y) * dx) / den
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


def sense(x, y, yaw_deg):
    ranges = {}
    for name, rel in (('front', 0), ('left', 90), ('back', 180), ('right', -90)):
        d = cast(x, y, math.radians(yaw_deg + rel)) + random.gauss(0, NOISE)
        ranges[name] = d if d < MAX_RANGE else None
    ranges['up'] = None
    return ranges


def main():
    random.seed(1)
    cfg = ExploreConfig()
    omap = OccupancyMap()
    explorer = Explorer(cfg, omap)
    x = y = yaw = 0.0
    closest = math.inf
    t = 0.0
    while True:
        ranges = sense(x, y, yaw)
        omap.add_scan(x, y, math.radians(yaw), ranges)
        cmd = explorer.step(t, x, y, yaw, ranges)
        if cmd.land:
            break
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        x += (c * cmd.vx - s * cmd.vy) * DT
        y += (s * cmd.vx + c * cmd.vy) * DT
        yaw += cmd.yaw_rate * DT
        near = min(cast(x, y, math.radians(a)) for a in range(0, 360, 10))
        if near < closest:
            closest, closest_at = near, (x, y)
        t += DT

    omap.save_png('sim_map.png')
    omap.save_points('sim_points.csv')
    render_points(omap.points, 'sim_points.png')
    print(f'Летали {t:.0f} с, ближе всего к стене: {closest * 100:.0f} см, '
          f'(в точке x={closest_at[0]:.2f} y={closest_at[1]:.2f}), точек стен: {len(omap.points)}')
    print('Карта: sim_map.png, облако точек: sim_points.csv')


if __name__ == '__main__':
    main()
