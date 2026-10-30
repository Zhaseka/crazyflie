"""
Логика облёта препятствий для Crazyflie с Multi-ranger deck и Flow deck.

Здесь нет зависимостей от cflib, поэтому ту же самую логику можно гонять
и на настоящем дроне (fly.py), и в симуляторе (simulate.py).

Система координат Crazyflie: x — вперёд, y — влево, z — вверх.

Как это работает (машина состояний):
  CRUISE    — летим вперёд по исходной линии y = y_старта на высоте cruise_height.
  DECIDE    — впереди препятствие: зависаем, несколько раз меряем дальномеры
              и выбираем, куда облетать: влево, вправо или вверх.
  SHIFT     — смещаемся вбок, пока передний датчик не перестанет видеть препятствие
              (+ небольшой запас).
  PASS_SIDE — летим вперёд, пока боковой датчик видит препятствие, и ещё чуть-чуть.
  CLIMB     — набираем высоту, пока передний датчик не освободится (+ запас).
  PASS_OVER — летим вперёд над препятствием, пока нижний датчик (Flow deck)
              видит его под собой, и ещё чуть-чуть.
  Потом снова CRUISE, который плавно возвращает дрон на исходную линию и высоту.

Если выбранный вариант не сработал (упёрлись в стену, потолок или слишком
далеко ушли в сторону), этот вариант помечается как неудачный и выбирается
следующий. Если вариантов не осталось — посадка.
"""

import statistics
from dataclasses import dataclass, fields, replace
from typing import NamedTuple, Optional

MAX_RANGE = 4.0  # дальномеры VL53L1x уверенно видят примерно до 4 м

LEFT, RIGHT, UP = 'left', 'right', 'up'
NAMES = {LEFT: 'слева', RIGHT: 'справа', UP: 'сверху'}


@dataclass
class Config:
    cruise_height: float = 0.3   # м, высота обычного полёта
    max_height: float = 0.7     # м, выше не поднимаемся
    forward_speed: float = 0.4  # м/с
    avoid_speed: float = 0.25     # м/с, скорость боковых/вертикальных манёвров
    goal_distance: float = 5.0   # м, сколько пролететь вперёд, потом посадка

    obstacle_dist: float = 0.5   # впереди ближе этого -> начинаем облёт
    clear_dist: float = 0.8      # впереди дальше этого -> путь свободен
    side_margin: float = 0.15    # сдвинуться ещё на столько после того, как путь освободился
    up_margin: float = 0.15      # подняться ещё на столько после того, как путь освободился
    pass_margin: float = 0.3     # пролететь ещё на столько после того, как препятствие кончилось
    max_pass: float = 1.5        # если препятствие сбоку/снизу так и не увидели — считаем, что пролетели
    seen_dist: float = 0.6       # боковой датчик ближе этого -> препятствие рядом
    step_dist: float = 0.15      # под дроном стало ближе на столько -> летим над препятствием

    min_side_room: float = 0.5   # сколько нужно свободного места сбоку, чтобы туда лететь
    min_up_room: float = 0.3     # сколько нужно места сверху
    ceiling_margin: float = 0.3  # не подлетать к потолку ближе
    max_shift: float = 1.5       # м, дальше не уходим от исходной линии
    wall_min: float = 0.25       # отталкиваемся от стен ближе этого
    hand_stop: float = 0.15      # что-то вплотную сверху (например, рука) -> посадка

    up_penalty: float = 0.2      # облёт сверху чуть менее выгоден (дольше и сложнее)
    line_bonus: float = 0.3      # бонус стороне, которая ведёт обратно к исходной линии
    k_return: float = 0.8        # насколько резво возвращаться на линию/высоту
    decide_samples: int = 5      # сколько замеров усреднять перед выбором


@dataclass
class Sensors:
    x: float                 # позиция из оценщика состояния, м
    y: float
    z: float
    zrange: Optional[float]  # расстояние вниз (Flow deck), м
    front: Optional[float]   # дальномеры Multi-ranger, м (None — ничего не видно)
    back: Optional[float]
    left: Optional[float]
    right: Optional[float]
    up: Optional[float]


class Command(NamedTuple):
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    land: bool = False


HOVER = Command()
LAND = Command(land=True)


def _clamp(v, lim):
    return max(-lim, min(lim, v))


def _normalize(s):
    """None (ничего не видно) и всё, что дальше MAX_RANGE, превращаем в MAX_RANGE."""
    ranges = {}
    for f in fields(s):
        if f.name in ('x', 'y', 'z'):
            continue
        v = getattr(s, f.name)
        ranges[f.name] = MAX_RANGE if v is None else min(v, MAX_RANGE)
    return replace(s, **ranges)


class Avoider:
    def __init__(self, cfg=None, log=print):
        self.cfg = cfg or Config()
        self.log = log
        self.state = 'CRUISE'
        self.origin = None  # (x, y) старта; исходная линия полёта — y = origin[1]
        self.tried = set()  # варианты облёта, которые для текущего препятствия не сработали
        self.m = {}         # данные текущего манёвра

    @property
    def line_y(self):
        return self.origin[1]

    def step(self, raw):
        """Вызывать ~10 раз в секунду. Возвращает скорости для MotionCommander."""
        s = _normalize(raw)
        if self.origin is None:
            self.origin = (s.x, s.y)
        if s.up < self.cfg.hand_stop:
            self.log('Что-то вплотную над дроном — сажусь')
            return LAND
        return getattr(self, '_' + self.state.lower())(s)

    def _go(self, state, s, **data):
        self.log(f'[x={s.x:5.2f} y={s.y:5.2f} z={s.z:4.2f}] {self.state} -> {state}')
        self.state = state
        self.m = data

    def _repel(self, s):
        """Небольшой толчок от стен, если подлетели к ним слишком близко."""
        c = self.cfg
        v = 0.0
        if s.left < c.wall_min:
            v -= c.avoid_speed / 2
        if s.right < c.wall_min:
            v += c.avoid_speed / 2
        return v

    # --- обычный полёт ----------------------------------------------------

    def _cruise(self, s):
        c = self.cfg
        if s.front < c.obstacle_dist:
            self.log(f'Препятствие впереди в {s.front:.2f} м — зависаю и осматриваюсь')
            return self._start_decide(s, new_obstacle=True)

        off = self.line_y - s.y
        travelled = s.x - self.origin[0]
        if (travelled >= c.goal_distance and abs(off) < 0.1) or travelled >= c.goal_distance + 1.0:
            self.log('Долетел до цели — сажусь')
            return LAND

        # замедляемся, когда что-то впереди приближается
        vx = c.forward_speed * min(1.0, 0.3 + (s.front - c.obstacle_dist) / 0.5)

        # возвращаемся на исходную линию, если в ту сторону свободно
        vy = 0.0
        if abs(off) > 0.03:
            room = s.left if off > 0 else s.right
            if room > c.min_side_room:
                vy = _clamp(c.k_return * off, c.avoid_speed)
        vy += self._repel(s)

        # возвращаемся на обычную высоту
        vz = _clamp(c.k_return * (c.cruise_height - s.z), c.avoid_speed)
        if vz > 0 and s.up < c.ceiling_margin:
            vz = 0.0
        return Command(vx, vy, vz)

    # --- выбор манёвра ----------------------------------------------------

    def _start_decide(self, s, new_obstacle):
        if new_obstacle:
            self.tried.clear()
        self._go('DECIDE', s, samples=[])
        return HOVER

    def _decide(self, s):
        c = self.cfg
        samples = self.m['samples']
        samples.append(s)
        if len(samples) < c.decide_samples:
            return HOVER

        def med(name):
            return statistics.median(getattr(p, name) for p in samples)

        front, left, right, up = med('front'), med('left'), med('right'), med('up')
        if front > c.clear_dist:
            self.log('Путь освободился сам — лечу дальше')
            self._go('CRUISE', s)
            return HOVER

        off = self.line_y - s.y
        scores = {}
        for side, room in ((LEFT, left), (RIGHT, right)):
            if side in self.tried or room < c.min_side_room:
                continue
            toward_line = abs(off) > 0.05 and (side == LEFT) == (off > 0)
            scores[side] = min(room, 2.0) + (c.line_bonus if toward_line else 0.0)
        up_room = min(up - c.ceiling_margin, c.max_height - s.z)
        if UP not in self.tried and up_room >= c.min_up_room:
            scores[UP] = min(up_room, 2.0) - c.up_penalty

        self.log(f'Осмотр: впереди {front:.2f}, слева {left:.2f}, справа {right:.2f}, '
                 f'места сверху {up_room:.2f} м; оценки '
                 + (', '.join(f'{NAMES[k]}={v:.2f}' for k, v in scores.items()) or 'нет вариантов'))
        if not scores:
            self.log('Облететь некуда — сажусь')
            return LAND

        best = max(scores, key=scores.get)
        self.log(f'Облетаю {NAMES[best]}')
        if best == UP:
            self._go('CLIMB', s, cleared_z=None)
        else:
            self._go('SHIFT', s, side=best, dir=1 if best == LEFT else -1, cleared_y=None)
        return HOVER

    def _give_up(self, s, option, reason):
        self.log(f'{NAMES[option].capitalize()} не получается: {reason} — пробую другой вариант')
        self.tried.add(option)
        return self._start_decide(s, new_obstacle=False)

    # --- облёт сбоку ------------------------------------------------------

    def _shift(self, s):
        c = self.cfg
        d = self.m['dir']
        room = s.left if d > 0 else s.right
        if room < c.wall_min + 0.1:
            return self._give_up(s, self.m['side'], 'сбоку стена')
        if (s.y - self.line_y) * d > c.max_shift:  # уходим от линии в эту сторону слишком далеко
            return self._give_up(s, self.m['side'], 'слишком далеко от линии')

        if s.front > c.clear_dist:
            if self.m['cleared_y'] is None:
                self.m['cleared_y'] = s.y
            elif abs(s.y - self.m['cleared_y']) >= c.side_margin:
                self._go('PASS_SIDE', s, dir=d, start_x=s.x, seen_x=None)
                return HOVER
        else:
            self.m['cleared_y'] = None
        return Command(0.0, d * c.avoid_speed, 0.0)

    def _pass_side(self, s):
        c = self.cfg
        if s.front < c.obstacle_dist:
            self.log(f'Ещё одно препятствие впереди в {s.front:.2f} м')
            return self._start_decide(s, new_obstacle=True)

        # обходили слева -> препятствие справа, и наоборот
        near = s.right if self.m['dir'] > 0 else s.left
        if near < c.seen_dist:
            self.m['seen_x'] = s.x
        if self._passed(s):
            self.log('Препятствие позади — возвращаюсь на линию')
            self._go('CRUISE', s)
        return Command(c.forward_speed, self._repel(s), 0.0)

    def _passed(self, s):
        seen_x = self.m['seen_x']
        if seen_x is None:
            return s.x - self.m['start_x'] >= self.cfg.max_pass
        return s.x - seen_x >= self.cfg.pass_margin

    # --- облёт сверху -----------------------------------------------------

    def _climb(self, s):
        c = self.cfg
        if s.up < c.ceiling_margin:
            return self._give_up(s, UP, 'потолок')
        if s.z > c.max_height:
            return self._give_up(s, UP, 'выше max_height нельзя')

        if s.front > c.clear_dist:
            if self.m['cleared_z'] is None:
                self.m['cleared_z'] = s.z
            elif s.z - self.m['cleared_z'] >= c.up_margin:
                self._go('PASS_OVER', s, start_x=s.x, ref=s.zrange, seen_x=None)
                return HOVER
        else:
            self.m['cleared_z'] = None
        return Command(0.0, 0.0, c.avoid_speed)

    def _pass_over(self, s):
        c = self.cfg
        if s.front < c.obstacle_dist:
            self.log(f'Ещё одно препятствие впереди в {s.front:.2f} м')
            return self._start_decide(s, new_obstacle=True)

        # нижний датчик показывает меньше, чем над полом -> под нами препятствие
        if s.zrange < self.m['ref'] - c.step_dist:
            self.m['seen_x'] = s.x
        if self._passed(s):
            self.log('Препятствие позади — снижаюсь')
            self._go('CRUISE', s)
        vz = -0.1 if s.up < c.ceiling_margin else 0.0
        return Command(c.forward_speed, self._repel(s), vz)
