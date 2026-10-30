"""
Режим исследования на настоящем дроне: сам летает по комнате, облетает стены и строит 2D-карту.

Нужно: Crazyflie 2.x + Flow deck v2 + Multi-ranger deck + Crazyradio.
Запуск:  python3 explore.py
Карта обновляется каждые 2 секунды в map.png — открой её в VS Code и смотри вживую.
В конце: map.png (карта), points.csv (облако точек стен) и points.png (картинка облака точек).

Защита та же, что в fly.py: подключение не зависает, зависшие датчики и блокировка после
аварийной остановки лечатся перезагрузкой дрона, перевернули вверх ногами — моторы стоп.
Ctrl+C в любой момент — дрон садится, карта сохраняется.
Ставь дрон посередине комнаты, на пол с узором, при хорошем свете.
"""

import math
import sys
import threading
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander

from explorer import ExploreConfig, Explorer
from fly import (FAR, LINK_LOST_TIMEOUT, SIDES, connect_ready, disconnect, emergency_stop,
                 land_safely, reset_estimator, sensors_frozen, stop_setpoints)
from mapping import OccupancyMap
from show_points import render_points

DT = 0.1             # период управления, с
SAVE_EVERY = 2.0     # как часто обновлять map.png, с
RANGES = ('front', 'back', 'left', 'right', 'up')
MAX_JUMP = 0.3       # м за один замер (0.05 с): дальше — Flow deck потерял пол
FROZEN_SAMPLES = 10  # столько замеров подряд (0.5 с) все дальномеры по нулям — датчики зависли


def main():
    cfg = ExploreConfig()
    cflib.crtp.init_drivers()
    scf, broken = connect_ready()
    omap = OccupancyMap()
    try:
        explore(scf, cfg, omap, broken)
    finally:
        disconnect(scf)
        if omap.points:  # если дрон так и не полетел, старую карту не затираем пустой
            omap.save_png('map.png')
            omap.save_points('points.csv')
            print(f'Карта: map.png, облако точек: points.csv ({len(omap.points)} точек)')
            render_points(omap.points, 'points.png')  # картинка облака точек — сразу, без show_points.py


def explore(scf, cfg, omap, broken):
    reset_estimator(scf)  # начало координат карты — точка взлёта; заодно проверка батареи

    lock = threading.Lock()  # карту и данные трогают и поток логов, и главный цикл
    latest = {}
    last_valid = {}          # последнее нормальное показание каждого дальномера, м
    problem = {}             # что пошло не так в полёте (заполняют обработчики данных)
    emergency = threading.Event()
    mc = MotionCommander(scf, default_height=cfg.height)
    sensors = [name for name in RANGES if name not in broken]  # сломанным не верим вовсе
    if broken:
        print('Не использую неработающие датчики: ' + ', '.join(sorted(broken)))

    def on_data(ts, data, lc):
        x, y, z = data['stateEstimate.x'], data['stateEstimate.y'], data['stateEstimate.z']
        raw = {name: data['range.' + name] for name in sensors}
        with lock:
            # боковые дальномеры разом по нулям или все «ничего не вижу» — датчики зависли,
            # лететь вслепую нельзя
            frozen = sensors_frozen([raw[name] for name in SIDES if name in raw])
            latest['frozen'] = latest.get('frozen', 0) + 1 if frozen else 0
            if latest['frozen'] >= FROZEN_SAMPLES and mc._is_flying:
                problem.setdefault('why', 'Датчики расстояния зависли в полёте — сажусь. '
                                          'Запусти explore.py снова: он сам перезагрузит дрон.')
            # одиночный 0 — сбой замера: берём прошлое нормальное значение, на карту не рисуем
            ranges = {}
            for name, mm in raw.items():
                if mm != 0:
                    last_valid[name] = None if mm >= FAR else mm / 1000.0
                if name in last_valid:
                    ranges[name] = last_valid[name]
            if 'x' in latest and max(abs(x - latest['x']), abs(y - latest['y'])) > MAX_JUMP:
                problem.setdefault('why', f'Дрон потерял своё положение (прыжок на '
                                          f'{math.hypot(x - latest["x"], y - latest["y"]):.2f} м) — '
                                          'сажусь. Flow deck плохо видит пол: нужен пол с узором и свет')
            latest.update(x=x, y=y, z=z, yaw=data['stateEstimate.yaw'], ranges=ranges, t=time.time())
            if z > 0.2 and not frozen:  # на земле и на взлёте дальномеры видят пол — не рисуем
                omap.add_scan(x, y, math.radians(latest['yaw']),
                              {name: ranges[name] for name in ranges if raw[name] != 0})

    def on_tilt(ts, data, lc):
        # перевернули вверх ногами — моторы стоп сразу (как в fly.py)
        if abs(data['stabilizer.roll']) > 90 and mc._is_flying and not emergency.is_set():
            emergency.set()
            emergency_stop(scf.cf)

    # 4 числа по 4 байта + 5 по 2 = 26 байт — больше в один пакет логов не влезает,
    # поэтому наклон идёт отдельным пакетом
    logconf = LogConfig(name='Explore', period_in_ms=50)
    for var in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'stateEstimate.yaw'):
        logconf.add_variable(var, 'float')
    for name in RANGES:
        logconf.add_variable('range.' + name, 'uint16_t')
    logconf.data_received_cb.add_callback(on_data)
    tiltconf = LogConfig(name='Tilt', period_in_ms=50)
    tiltconf.add_variable('stabilizer.roll', 'float')
    tiltconf.data_received_cb.add_callback(on_tilt)
    for conf in (logconf, tiltconf):
        scf.cf.log.add_config(conf)
        conf.start()

    # не взлетаем, пока не пришли данные датчиков
    deadline = time.time() + 3.0
    while 't' not in latest and time.time() < deadline:
        time.sleep(0.05)
    if 't' not in latest:
        sys.exit('Дрон не присылает данные датчиков — выключи и включи его, потом запусти снова')

    explorer = Explorer(cfg, omap)
    # взлёт внутри try: если нажать Ctrl+C даже во время взлёта, дрон всё равно сядет
    try:
        print(f'Взлетаю и исследую {cfg.duration:.0f} с. Карта: map.png')
        mc.take_off()
        time.sleep(1.0)
        last_save = time.time()
        while not emergency.is_set():
            with lock:
                s = dict(latest)
                why = problem.get('why')
                if why is None and time.time() - s['t'] > LINK_LOST_TIMEOUT:
                    why = 'Связь с дроном пропадает (данные не приходят) — сажусь. Скорее всего, садится батарея.'
                if why is None and math.hypot(s['x'], s['y']) > cfg.max_radius + 1.5:
                    why = (f'Дрон улетел слишком далеко (x={s["x"]:.2f} y={s["y"]:.2f}) — похоже, '
                           'потерял положение. Сажусь.')
                if why is not None:
                    print(why)
                    break
                cmd = explorer.step(time.time(), s['x'], s['y'], s['yaw'], s['ranges'])
            if cmd.land:
                break
            mc.start_linear_motion(cmd.vx, cmd.vy, 0.0, rate_yaw=cmd.yaw_rate)
            if time.time() - last_save > SAVE_EVERY:
                with lock:
                    omap.save_png('map.png')
                last_save = time.time()
            time.sleep(DT)
    except KeyboardInterrupt:
        if not emergency.is_set():
            print('Ctrl+C — сажусь')
    except Exception:
        # после аварийной остановки cflib может ругаться на прерванный манёвр — это не важно
        if not emergency.is_set():
            raise
    finally:
        if emergency.is_set():
            stop_setpoints(mc)
            print('АВАРИЙНАЯ ОСТАНОВКА: дрон перевернули — моторы выключены.\n'
                  'Чтобы снова лететь, просто запусти explore.py — он сам перезагрузит дрон.')
        else:
            land_safely(mc)
            print('Сел')
        for conf in (logconf, tiltconf):
            conf.stop()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nОстановлено. Дрон не взлетал.')
