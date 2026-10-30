"""
Лампочка на дроне по пробелу. Дрон НЕ взлетает.

Светодиод: левый разъём, пин 9 (IO_4) через резистор, пин 10 — GND.
Сначала прошей дрон приложением из папки led_app (см. README).

Запуск:  python3 led_space.py
Пробел — включить/выключить, q — выход.
"""

import logging
import sys
import termios
import tty

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
PARAM = 'extled.on'

logging.basicConfig(level=logging.ERROR)


def read_key():
    """Ждёт одну клавишу без Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        if 'extled' not in cf.param.toc.toc or 'on' not in cf.param.toc.toc['extled']:
            sys.exit('В дроне нет параметра extled.on — сначала прошей приложение из папки led_app.')

        on = False
        cf.param.set_value(PARAM, '0')
        print('Пробел — включить/выключить лампочку, q — выход')
        try:
            while True:
                key = read_key()
                if key == ' ':
                    on = not on
                    cf.param.set_value(PARAM, '1' if on else '0')
                    print('💡 Горит' if on else '⚫ Погасла')
                elif key in ('q', 'Q'):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            cf.param.set_value(PARAM, '0')
            print('Выключил лампочку, выход')


if __name__ == '__main__':
    main()
