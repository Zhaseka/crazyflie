"""
Симулятор, чтобы проверить логику облёта без дрона.

Запуск:  python3 simulate.py            — все сцены
         python3 simulate.py corridor   — одна сцена
Если установлен matplotlib, рядом сохранится картинка траектории sim_<сцена>.png.

Дрон — точка, дальномеры — лучи вдоль осей, препятствия — коробки.
"""

import math
import sys
from dataclasses import dataclass

from avoidance import MAX_RANGE, Avoider, Config, Sensors

DT = 0.1
CEILING = 2.5     # высота потолка в комнате, м
DRONE_R = 0.06    # «радиус» дрона для проверки столкновений


@dataclass
class Box:
    lo: tuple  # (x, y, z) нижний угол
    hi: tuple  # (x, y, z) верхний угол

    def hit(self, p, pad=0.0):
        return all(self.lo[k] - pad <= p[k] <= self.hi[k] + pad for k in range(3))


def box(x0, x1, y0, y1, z0, z1):
    return Box((x0, y0, z0), (x1, y1, z1))


WORLDS = {
    'column': ('Колонна до потолка, слева стена — облёт справа',
               [box(1.3, 1.6, -0.3, 0.3, 0, CEILING), box(0, 6, 0.9, 1.0, 0, CEILING)]),
    'corridor': ('Узкий коридор, поперёк низкий ящик — облёт сверху',
                 [box(0, 6, 0.6, 0.7, 0, CEILING), box(0, 6, -0.7, -0.6, 0, CEILING),
                  box(1.3, 1.7, -0.6, 0.6, 0, 0.6)]),
    'wall': ('Широкая стенка: слева и справа не хватает места — в итоге сверху',
             [box(1.3, 1.5, -3.0, 2.0, 0, 0.7)]),
    'box': ('Коробка на полу посередине — облёт сбоку и дальше до цели',
            [box(1.3, 1.6, -0.2, 0.2, 0, 0.5)]),
    'broken_right': ('Правый датчик сломан (всегда 7 см) — облёт слева',
                     [box(1.3, 1.6, -0.2, 0.2, 0, 0.5)]),
    'slalom': ('Две колонны подряд',
               [box(1.0, 1.2, -0.4, 0.3, 0, CEILING), box(2.0, 2.2, -0.2, 0.6, 0, CEILING)]),
    'dead_end': ('Тупик — облететь некуда, посадка',
                 [box(0, 6, 0.6, 0.7, 0, CEILING), box(0, 6, -0.7, -0.6, 0, CEILING),
                  box(1.3, 1.5, -0.6, 0.6, 0, CEILING)]),
}

# сцены, где какой-то датчик «сломан» и всё время показывает 7 см
BROKEN = {'broken_right': {'right'}}


def ray(boxes, p, axis, sign):
    """Расстояние вдоль оси axis (0=x, 1=y, 2=z) в направлении sign до ближайшего препятствия."""
    dist = float('inf')
    if axis == 2:
        dist = CEILING - p[2] if sign > 0 else p[2]
    others = [k for k in range(3) if k != axis]
    for b in boxes:
        if all(b.lo[k] <= p[k] <= b.hi[k] for k in others):
            d = b.lo[axis] - p[axis] if sign > 0 else p[axis] - b.hi[axis]
            if d >= 0:
                dist = min(dist, d)
    return dist if dist <= MAX_RANGE else None


CONE_TAN = math.tan(math.radians(13.5))  # луч VL53L1x — конус ~27°, половина угла 13.5°


def cone_ray(boxes, p, axis, sign):
    """Как ray(), но учитывает нижний край конуса: он может задеть пол или верх коробки."""
    best = ray(boxes, p, axis, sign)
    best = float('inf') if best is None else best
    step = 0.01
    for i in range(1, int(MAX_RANGE / step)):
        d = i * step
        q = list(p)
        q[axis] += sign * d
        q[2] -= d * CONE_TAN
        if q[2] <= 0 or any(b.hit(q) for b in boxes):
            best = min(best, d * math.sqrt(1 + CONE_TAN ** 2))
            break
    return best if best <= MAX_RANGE else None


def sense(boxes, p, broken=()):
    s = Sensors(
        x=p[0], y=p[1], z=p[2],
        zrange=ray(boxes, p, 2, -1),
        front=cone_ray(boxes, p, 0, +1), back=cone_ray(boxes, p, 0, -1),
        left=cone_ray(boxes, p, 1, +1), right=cone_ray(boxes, p, 1, -1),
        up=ray(boxes, p, 2, +1),
    )
    for side in broken:
        setattr(s, side, 0.07)
    return s


def run(name, cfg):
    desc, boxes = WORLDS[name]
    print(f'\n=== {name}: {desc}')
    p = [0.0, 0.0, cfg.cruise_height]  # считаем, что уже взлетели
    path = [tuple(p)]
    broken = BROKEN.get(name, set())
    avoider = Avoider(cfg, log=lambda msg: print('   ', msg))
    avoider.broken = set(broken)  # как в fly.py: датчик, который сразу показывает < 10 см
    result = 'таймаут'
    for _ in range(int(180 / DT)):
        cmd = avoider.step(sense(boxes, p, broken))
        if cmd.land:
            result = 'посадка'
            break
        p[0] += cmd.vx * DT
        p[1] += cmd.vy * DT
        p[2] += cmd.vz * DT
        path.append(tuple(p))
        if p[2] < 0 or any(b.hit(p, DRONE_R) for b in boxes):
            result = 'СТОЛКНОВЕНИЕ'
            break
    print(f'    Итог: {result} в x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f}, '
          f'время {len(path) * DT:.1f} с')
    plot(name, desc, boxes, path)
    return result != 'СТОЛКНОВЕНИЕ'


def plot(name, desc, boxes, path):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        return
    fig, (top, side) = plt.subplots(2, 1, figsize=(8, 8))
    for ax, (a, b), title in ((top, (0, 1), 'вид сверху (x, y)'), (side, (0, 2), 'вид сбоку (x, z)')):
        for bx in boxes:
            ax.add_patch(Rectangle((bx.lo[a], bx.lo[b]), bx.hi[a] - bx.lo[a], bx.hi[b] - bx.lo[b],
                                   color='grey', alpha=0.5))
        ax.plot([q[a] for q in path], [q[b] for q in path], 'b-')
        ax.set_xlim(-0.2, 6.2)
        ax.set_ylim(-2, 2) if b == 1 else ax.set_ylim(0, CEILING)
        ax.set_aspect('equal')
        ax.set_title(title)
    fig.suptitle(desc)
    fig.savefig(f'sim_{name}.png', dpi=80)
    plt.close(fig)


def main():
    names = sys.argv[1:] or list(WORLDS)
    unknown = [n for n in names if n not in WORLDS]
    if unknown:
        sys.exit(f'Нет сцены {", ".join(unknown)}. Есть: {", ".join(WORLDS)}')
    ok = all([run(n, Config()) for n in names])
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
