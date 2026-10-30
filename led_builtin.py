import logging
import sys
import termios
import tty

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
PARAM = 'led.bitmask'
ENABLE = 1 << 7

LEDS = {
    '1': (0, 'синяя слева'),
    '2': (1, 'зелёная слева'),
    '3': (2, 'красная слева'),
    '4': (3, 'зелёная справа'),
    '5': (4, 'красная справа'),
    '6': (5, 'синяя справа'),
}
ALL = sum(1 << bit for bit, _ in LEDS.values())

logging.basicConfig(level=logging.ERROR)


def read_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def describe(mask):
    on = [name for bit, name in LEDS.values() if mask & (1 << bit)]
    return '💡 ' + ', '.join(on) if on else '⚫ все выключены'


def main():
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        if 'bitmask' not in cf.param.toc.toc.get('led', {}):
            sys.exit('В прошивке дрона нет параметра led.bitmask — она слишком старая. '
                     'Обнови прошивку:  python3 update_firmware.py')

        print('Клавиши:')
        for key, (_, name) in LEDS.items():
            print(f'  {key} — {name}')
        print('  пробел — все сразу, 0 — обычное мигание, q — выход')

        mask = 0
        try:
            while True:
                key = read_key()
                if key in LEDS:
                    mask ^= 1 << LEDS[key][0]
                elif key == ' ':
                    mask = 0 if mask == ALL else ALL
                elif key == '0':
                    cf.param.set_value(PARAM, '0')
                    mask = 0
                    print('Обычное мигание')
                    continue
                elif key in ('q', 'Q'):
                    break
                else:
                    continue
                cf.param.set_value(PARAM, str(ENABLE | mask))
                print(describe(mask))
        except KeyboardInterrupt:
            pass
        finally:
            cf.param.set_value(PARAM, '0')
            print('Лампочки как обычно, выход')


if __name__ == '__main__':
    main()
