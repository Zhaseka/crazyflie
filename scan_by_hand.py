"""
2D-карта комнаты БЕЗ полёта — пока не работает Flow deck. Моторы не включаются.

Ставишь дрон в точку, медленно поворачиваешь его рукой на полный оборот —
дальномеры Multi-ranger рисуют комнату вокруг, поворот считает гироскоп.
Потом переносишь в другую точку, вводишь её координаты (рулеткой) и повторяешь.

Запуск:  python3 scan_by_hand.py
Результат: map.png (карта) и points.csv (облако точек стен).

Советы:
- Положи дрон на книгу/тарелку и крути её, а не дрон: рука перед датчиком — это «стена».
- Подними дрон на стул или коробку (~40 см), чтобы датчики видели стены, а не ножки мебели.
- Ось x — куда смотрел нос дрона при включении, y — влево от неё. Координаты в метрах.
"""

import math
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from fly import FLOW_CAMERA_DEAD, URI, flow_problem
from mapping import OccupancyMap

SCAN_TIME = 20.0   # сколько секунд крутить дрон в одной точке
MIN_RANGE = 0.15   # ближе — это рука или подставка, не стена
SIDES = ('front', 'back', 'left', 'right')


def to_m(mm):
    d = None if mm >= 4000 else mm / 1000.0
    return None if d is not None and d < MIN_RANGE else d


def ask_point():
    """Спрашивает координаты точки. None — закончить."""
    while True:
        text = input('\nКоординаты точки «x y» в метрах (Enter — 0 0, q — закончить): ').strip()
        if text.lower() == 'q':
            return None
        if not text:
            return 0.0, 0.0
        try:
            x, y = (float(v.replace(',', '.')) for v in text.split())
            return x, y
        except ValueError:
            print('Не понял. Пример:  1.5 -0.5')


def main():
    omap = OccupancyMap()
    lock = threading.Lock()
    state = {'point': None}  # где сейчас стоит дрон; None — не рисуем
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        if int(scf.cf.param.get_value('deck.bcMultiranger')) != 1:
            print('Multi-ranger deck не найден — без него карту не построить')
            return
        if flow_problem(scf) == FLOW_CAMERA_DEAD:
            # модуль, не прошедший самопроверку, останавливает весь дрон: датчики показывают нули
            print('Flow deck стоит, но не работает — из-за него дрон не запускает датчики.\n'
                  'Выключи дрон, сними Flow deck снизу, включи и запусти снова.')
            return

        def on_data(ts, data, lc):
            with lock:
                if state['point'] is None:
                    return
                x, y = state['point']
                ranges = {name: to_m(data['range.' + name]) for name in SIDES}
                omap.add_scan(x, y, math.radians(data['stabilizer.yaw']), ranges)

        logconf = LogConfig(name='Scan', period_in_ms=50)
        logconf.add_variable('stabilizer.yaw', 'float')
        for name in SIDES:
            logconf.add_variable('range.' + name, 'uint16_t')
        logconf.data_received_cb.add_callback(on_data)
        scf.cf.log.add_config(logconf)
        logconf.start()

        print('Первая точка — 0 0 (там, где дрон стоит сейчас).')
        try:
            while True:
                point = ask_point()
                if point is None:
                    break
                input(f'Дрон в точке {point}. Нажми Enter и медленно поверни его на полный оборот '
                      f'(~{SCAN_TIME:.0f} с)...')
                with lock:
                    state['point'] = point
                for left in range(int(SCAN_TIME), 0, -1):
                    print(f'\r  кручу... осталось {left:2d} с', end='', flush=True)
                    time.sleep(1)
                with lock:
                    state['point'] = None
                    omap.save_png('map.png')
                print('\r  готово, карта обновлена: map.png        ')
        except KeyboardInterrupt:
            pass
        finally:
            logconf.stop()
            with lock:
                omap.save_png('map.png')
                omap.save_points('points.csv')
            print(f'\nКарта: map.png, облако точек: points.csv ({len(omap.points)} точек)')


if __name__ == '__main__':
    main()
