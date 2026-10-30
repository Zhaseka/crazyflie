"""
Обновление прошивки Crazyflie на официальную (Bitcraze) по радио.
Ничего компилировать не надо. Дрон НЕ взлетает.

Запуск:  python3 update_firmware.py            — последняя версия
         python3 update_firmware.py 2025.02    — другая версия
Если не получилось: выключи дрон, зажми кнопку включения на 3 секунды (синие огоньки
мигают) и запусти  python3 update_firmware.py --cold
Дрон должен быть включён, Crazyradio — в USB. Не выключай дрон, пока идёт прошивка (1-3 минуты).
"""

import logging
import os
import subprocess
import sys

import cflib.crtp
from cflib.bootloader import Bootloader, Target
from cflib.utils import uri_helper

# версию можно указать:  python3 update_firmware.py 2025.02
VERSION = next((a for a in sys.argv[1:] if not a.startswith('-')), '2026.10')
URL = (f'https://github.com/bitcraze/crazyflie-release/releases/download/'
       f'{VERSION}/firmware-cf2-{VERSION}.zip')
ZIP = os.path.join('firmware', f'firmware-cf2-{VERSION}.zip')
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

logging.basicConfig(level=logging.ERROR)


def progress(msg, percent):
    print(f'\r{percent:3d}%  {msg[:70]:<70}', end='', flush=True)


def main():
    if not os.path.exists(ZIP):
        os.makedirs('firmware', exist_ok=True)
        print(f'Скачиваю официальную прошивку {VERSION}...')
        # curl, а не urllib: Python с python.org на Mac часто не видит сертификаты сайтов
        subprocess.run(['curl', '-fL', '-o', ZIP, URL], check=True)

    cflib.crtp.init_drivers()
    print('Перевожу дрон в режим прошивки и заливаю. Не выключай дрон!')
    cold = '--cold' in sys.argv
    bl = Bootloader(None if cold else URI)
    try:
        # только сам дрон (STM32 и nRF51); обновление модулей пропускаем — на нём прошивка зависала
        targets = [Target('cf2', 'stm32', 'fw', [], []), Target('cf2', 'nrf51', 'fw', [], [])]
        bl.flash_full(filename=ZIP, warm=not cold, targets=targets, progress_cb=progress)
    except Exception as e:
        sys.exit(f'\nНе получилось: {e}\n'
                 'Выключи дрон, зажми кнопку включения на 3 секунды (синие огоньки мигают) '
                 'и запусти:  python3 update_firmware.py --cold')
    finally:
        bl.close()
    print('\nГотово! Дрон перезагрузился с новой прошивкой. Теперь:  python3 led_builtin.py')


if __name__ == '__main__':
    main()
