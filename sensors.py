"""
Показывает, что видят датчики дрона, в реальном времени. Дрон НЕ взлетает.

Запуск:  python3 sensors.py
Остановить: Ctrl+C
"""

import sys
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig

import fly

NAMES = {'front': 'впереди', 'back': 'сзади', 'left': 'слева', 'right': 'справа',
         'up': 'сверху', 'zrange': 'снизу'}


def show(ts, data, logconf):
    parts = []
    for key, name in NAMES.items():
        mm = data['range.' + key]
        parts.append(f'{name} {"  ---  " if mm >= 4000 else f"{mm / 10:4.0f} см"}')
    print(' | '.join(parts))


def connect_with_working_sensors():
    """Подключается; если датчики зависли (все 0) — один раз перезагружает дрон."""
    scf = fly.connect()
    try:
        fly.check_ranges(scf)
        return scf
    except fly.SensorsStuck:
        print('Датчики зависли (0 или «ничего не вижу») — перезагружаю дрон, подожди 5 секунд...')
        fly.disconnect(scf)
        fly.reboot_drone()
        scf = fly.connect()
        try:
            fly.check_ranges(scf)
        except fly.SensorsStuck:
            fly.disconnect(scf)
            sys.exit(fly.DECK_STUCK)
        return scf


def main():
    cflib.crtp.init_drivers()
    fly.find_drone()
    scf = connect_with_working_sensors()
    try:
        logconf = LogConfig(name='Ranges', period_in_ms=300)
        for key in NAMES:
            logconf.add_variable('range.' + key, 'uint16_t')
        logconf.data_received_cb.add_callback(show)
        scf.cf.log.add_config(logconf)
        print('«---» значит дальше 4 м (ничего не видно). Ctrl+C — выход.\n')
        logconf.start()
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        logconf.stop()
    finally:
        fly.disconnect(scf)


if __name__ == '__main__':
    main()
