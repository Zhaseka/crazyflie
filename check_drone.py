"""
Проверка подключения: видно ли Crazyradio и дрон. Дрон НЕ взлетает.

Запуск:  python3 check_drone.py
"""

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.drivers.crazyradio import get_serials

from fly import flow_problem


def main():
    cflib.crtp.init_drivers()

    if not get_serials():
        print('❌ Crazyradio не найден. Вставь USB-антенну в компьютер и запусти снова.')
        return
    print('✅ Crazyradio найден')

    print('Ищу дрон (несколько секунд)...')
    found = []
    for _ in range(3):  # поиск по радио иногда промахивается — пробуем несколько раз
        found = cflib.crtp.scan_interfaces()
        if found:
            break
    if not found:
        print('❌ Дрон не найден. Проверь, что он включён (горят огоньки) и заряжен.')
        print('   Если только что включила — подожди 5 секунд и запусти снова.')
        return
    uri = found[0][0]
    print(f'✅ Дрон найден: {uri}')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        problem = flow_problem(scf)
        print('✅ Flow deck v2 (снизу): работает' if not problem else f'❌ {problem}')
        ok = int(scf.cf.param.get_value('deck.bcMultiranger')) == 1
        print(f'{"✅" if ok else "❌"} Multi-ranger deck (сверху): {"стоит" if ok else "НЕ найден"}')

    if problem or not ok:
        print('\nЛетать пока нельзя — сначала исправь то, что отмечено ❌.')
    elif uri.rstrip('/') not in ('radio://0/80/2M', 'radio://0/80/2M/E7E7E7E7E7'):
        print('\nАдрес дрона не стандартный. Запускай полёт так:')
        print(f'CFLIB_URI={uri} python3 fly.py')
    else:
        print('\nВсё готово. Полёт:  python3 fly.py')


if __name__ == '__main__':
    main()
