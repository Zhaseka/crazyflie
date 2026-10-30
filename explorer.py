"""
Режим исследования: дрон сам летает по комнате и строит 2D-карту. Без cflib,
поэтому ту же логику можно гонять и на дроне (explore.py), и в симуляторе (explore_sim.py).

Машина состояний:
  SCAN — зависаем и поворачиваемся на месте (4 дальномера × поворот = обзор вокруг).
  GO   — летим в выбранном направлении, пока впереди не появится препятствие
         или пока не пролетим scan_every метров. Потом снова SCAN.
Куда лететь после осмотра: туда, где свободно, где на карте больше серого
(неисследованного) и где мы ещё не были.

Всё время держимся от стен не ближе keep_dist. Садимся, когда кончилось время,
или исследовать больше нечего.
"""

import math
from dataclasses import dataclass
from typing import NamedTuple

MAX_RANGE = 4.0
N_BINS = 36  # направления для выбора, по 10°


@dataclass
class ExploreConfig:
    height: float = 0.4          # высота полёта, м
    speed: float = 0.3           # скорость вперёд, м/с
    scan_rate: float = 60.0      # скорость поворота при осмотре, °/с
    scan_angle: float = 120.0    # на сколько поворачиваться при осмотре, °
    scan_every: float = 1.5      # пролетели столько — осматриваемся снова, м
    stop_dist: float = 0.6       # впереди ближе — останавливаемся и осматриваемся, м
    keep_dist: float = 0.45      # ближе к стене не подлетаем, м
    min_room: float = 1.0        # направление годится, если свободно хотя бы на столько, м
    max_radius: float = 6.0      # не улетаем от точки взлёта дальше, м
    duration: float = 180.0      # сколько секунд исследуем
    max_yaw_rate: float = 60.0   # °/с


class Cmd(NamedTuple):
    vx: float = 0.0        # м/с вперёд (в системе дрона)
    vy: float = 0.0        # м/с влево
    yaw_rate: float = 0.0  # °/с, плюс — против часовой
    land: bool = False


def _wrap(deg):
    return (deg + 180.0) % 360.0 - 180.0


def _clamp(v, lim):
    return max(-lim, min(lim, v))


class Explorer:
    def __init__(self, cfg, omap, log=print):
        self.cfg = cfg
        self.map = omap
        self.log = log
        self.state = 'SCAN'
        self.t0 = None
        self.visits = {}      # клетка 0.5 м -> сколько шагов мы там провели
        self._start_scan(None)

    # ---------- главный шаг ----------

    def step(self, t, x, y, yaw, ranges):
        """yaw — в градусах; ranges — {'front','back','left','right','up'}: м или None."""
        if self.t0 is None:
            self.t0 = t
        cell = (round(x / 0.5), round(y / 0.5))
        self.visits[cell] = self.visits.get(cell, 0) + 1

        if t - self.t0 > self.cfg.duration:
            self.log(f'Прошло {self.cfg.duration:.0f} с — исследование закончено, сажусь')
            return Cmd(land=True)

        if self.state == 'SCAN':
            cmd = self._scan(x, y, yaw, ranges)
        else:
            cmd = self._go(x, y, yaw, ranges)
        if cmd.land:
            return cmd
        rx, ry = self._repel(ranges)
        return cmd._replace(vx=cmd.vx + rx, vy=cmd.vy + ry)

    # ---------- осмотр ----------

    def _start_scan(self, yaw):
        self.state = 'SCAN'
        self.scan_turned = 0.0
        self.last_yaw = yaw
        self.bins = [0.0] * N_BINS

    def _scan(self, x, y, yaw, ranges):
        if self.last_yaw is not None:
            self.scan_turned += abs(_wrap(yaw - self.last_yaw))
        self.last_yaw = yaw
        # запоминаем, насколько свободно в каждом направлении
        for name, rel in (('front', 0), ('left', 90), ('back', 180), ('right', -90)):
            if name in ranges:
                d = ranges[name]
                d = MAX_RANGE if d is None else min(d, MAX_RANGE)
                b = int(((yaw + rel) % 360) // (360 / N_BINS))
                self.bins[b] = max(self.bins[b], d)

        if self.scan_turned < self.cfg.scan_angle:
            return Cmd(yaw_rate=self.cfg.scan_rate)

        heading = self._choose(x, y)
        if heading is None:
            self.log('Исследовать больше нечего — сажусь')
            return Cmd(land=True)
        self.state = 'GO'
        self.heading = heading
        self.go_from = (x, y)
        self.log(f'[x={x:5.2f} y={y:5.2f}] лечу в направлении {heading:4.0f}°')
        return Cmd()

    def _choose(self, x, y):
        frontier = self.map.frontier()
        best, best_score = None, -1e9
        for b in range(N_BINS):
            # свободно должно быть не по одному лучу, а в «коридоре» ±20°,
            # иначе дрон заденет угол, который датчики не видят (они смотрят только в 4 стороны)
            room = min(self.bins[(b + k) % N_BINS] for k in range(-2, 3))
            if room < self.cfg.min_room:
                continue
            h = (b + 0.5) * 360 / N_BINS
            c, s = math.cos(math.radians(h)), math.sin(math.radians(h))
            step = min(room - 0.5, self.cfg.scan_every)
            tx, ty = x + c * step, y + s * step
            if math.hypot(tx, ty) > self.cfg.max_radius and math.hypot(tx, ty) > math.hypot(x, y):
                continue
            # сколько «границы неизвестного» видно в этом направлении (точки на 1 и 2 м)
            new = sum(self.map.count_near(frontier, x + c * d, y + s * d, 0.6)
                      for d in (1.0, 2.0) if d < room)
            visits = self.visits.get((round(tx / 0.5), round(ty / 0.5)), 0)
            score = 0.3 * min(room, 3.0) + 0.05 * min(new, 60) - 0.05 * min(visits, 40)
            if score > best_score:
                best, best_score = h, score
        return best

    # ---------- полёт ----------

    def _go(self, x, y, yaw, ranges):
        err = _wrap(self.heading - yaw)
        yaw_rate = _clamp(2.0 * err, self.cfg.max_yaw_rate)
        front = ranges.get('front')
        travelled = math.hypot(x - self.go_from[0], y - self.go_from[1])
        if abs(err) < 15 and front is not None and front < self.cfg.stop_dist:
            self._start_scan(yaw)
            return Cmd()
        if travelled > self.cfg.scan_every or math.hypot(x, y) > self.cfg.max_radius + 0.3:
            self._start_scan(yaw)
            return Cmd()
        vx = self.cfg.speed if abs(err) < 20 else 0.0
        return Cmd(vx=vx, yaw_rate=yaw_rate)

    def _repel(self, ranges):
        """Отталкиваемся от стен, которые ближе keep_dist (скорость в системе дрона)."""
        vx = vy = 0.0
        k = 2.0
        for name, ax, sign in (('front', 'x', -1), ('back', 'x', 1), ('left', 'y', -1), ('right', 'y', 1)):
            d = ranges.get(name)
            if d is not None and d < self.cfg.keep_dist:
                push = sign * k * (self.cfg.keep_dist - d)
                if ax == 'x':
                    vx += push
                else:
                    vy += push
        return _clamp(vx, 0.3), _clamp(vy, 0.3)
