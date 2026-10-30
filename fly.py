"""
Полёт настоящего Crazyflie с облётом препятствий.

Нужно: Crazyflie 2.x + Flow deck v2 + Multi-ranger deck + Crazyradio.
Установка:  pip install cflib
Запуск:     python3 fly.py
Другой адрес дрона:  CFLIB_URI=radio://0/80/2M/E7E7E7E7E7 python3 fly.py

Ctrl+C в любой момент (даже во время взлёта) — дрон плавно садится.
"""

import logging
import sys
import threading
import time
import warnings

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger
from cflib.utils.power_switch import PowerSwitch

from avoidance import Avoider, Config, Sensors

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
DT = 0.1  # период управления, с
ESTIMATOR_TIMEOUT = 10.0  # с, сколько ждать, пока дрон поймёт, где он
CONNECT_TIMEOUT = 15.0  # с, сколько ждать подключения к дрону
LINK_LOST_TIMEOUT = 1.0  # с, нет данных от дрона дольше — связь пропадает, садимся

logging.basicConfig(level=logging.ERROR)
# на старой прошивке cflib пишет это предупреждение 10 раз в секунду — оно не мешает полёту
warnings.filterwarnings('ignore', message='Using legacy TYPE_HOVER_LEGACY')


FLOW_CAMERA_DEAD = (
    'Flow deck стоит, но его камера (PMW3901) не отвечает — без неё дрон не знает, где он.\n'
    'Это неисправность самого модуля или его разъёма, программой её не исправить.\n'
    'Попробуй: отсоедини батарею на 10 секунд, переставь Flow deck (стрелкой к носу, до упора).\n'
    'Не помогло — нужен другой Flow deck.')


def flow_problem(scf):
    """None, если Flow deck работает, иначе — что с ним не так."""
    param = scf.cf.param
    if int(param.get_value('deck.bcFlow2')) == 1:
        return None
    # у Flow deck два датчика: высотомер (как у Z-ranger) и камера. Высотомер есть, а
    # Flow deck не запустился — значит, модуль на месте, но камера не ответила
    if int(param.get_value('deck.bcZRanger2')) == 1:
        return FLOW_CAMERA_DEAD
    return 'Flow deck v2 не найден — проверь, что он стоит на дроне снизу'


def check_decks(scf):
    problem = flow_problem(scf)
    if problem:
        sys.exit(problem)
    if int(scf.cf.param.get_value('deck.bcMultiranger')) != 1:
        sys.exit('Multi-ranger deck не найден — проверь, что он стоит на дроне сверху')


RANGE_NAMES = {'front': 'передний', 'back': 'задний', 'left': 'левый', 'right': 'правый',
               'up': 'верхний', 'zrange': 'нижний (Flow deck)'}
DECK_STUCK = ('Датчики расстояния не работают — все показывают 0 или «ничего не вижу», даже '
              'стоя на полу. Так бывает, когда плата Multi-ranger плохо держит контакт.\n'
              'Выключи дрон, сними Multi-ranger и поставь обратно ровно и до упора, включи '
              'дрон и запусти снова. Ещё проверь заряд батареи.')
SIDES = ('front', 'back', 'left', 'right')
FAR = 4000  # мм: столько и больше — датчик «ничего не видит» (зависший датчик шлёт 32766)


class SensorsStuck(Exception):
    """Боковые дальномеры зависли (0 или «ничего не вижу») — помогает перезагрузка дрона."""


def sensors_frozen(values):
    """Дальномеры, смотрящие в стороны, все сразу показывают 0 или все «ничего не вижу».
    У живых так не бывает: и на полу, и в полёте на 30–40 см их лучи задевают пол в 1–2 м."""
    return all(v == 0 for v in values) or all(v >= FAR for v in values)


def check_ranges(scf):
    """До взлёта, дрон стоит на полу: работают ли дальномеры. Живой датчик хоть что-то видит
    (боковые — пол в 10–40 см). Сломанный целую секунду показывает ровно 0, а боковой или
    нижний — ещё и «ничего не вижу». Верхнему «ничего не вижу» простительно: высокий потолок.
    Возвращает боковые датчики, которые не работают, — в ту сторону облетать не будем."""
    samples = []
    logconf = LogConfig(name='Ranges', period_in_ms=100)
    for key in RANGE_NAMES:
        logconf.add_variable('range.' + key, 'uint16_t')
    logconf.data_received_cb.add_callback(lambda ts, data, lc: samples.append(data))
    scf.cf.log.add_config(logconf)
    logconf.start()
    time.sleep(1.0)
    logconf.stop()
    logconf.delete()

    if not samples:
        sys.exit('Дрон не присылает данные датчиков. Выключи и включи дрон и запусти снова.')

    def dead(key):
        values = [d['range.' + key] for d in samples]
        return all(v == 0 for v in values) or (key != 'up' and all(v >= FAR for v in values))

    if all(sensors_frozen([d['range.' + k] for k in SIDES]) for d in samples):
        raise SensorsStuck()
    for key in ('front', 'up', 'zrange'):
        if dead(key):
            sys.exit(f'Не работает {RANGE_NAMES[key]} датчик (всё время 0 или «ничего не вижу») — '
                     'без него лететь нельзя. Проверь, не закрыт ли он, и плотно ли стоит плата.')
    broken = {key for key in ('left', 'right') if dead(key)}
    for key in broken:
        print(f'Не работает {RANGE_NAMES[key]} датчик — облетать в ту сторону не буду.')
    return broken


def reset_estimator(scf):
    """Сбрасывает оценку позиции и ждёт, пока она устоится. Дрон должен спокойно стоять на полу."""
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    history = []
    logconf = LogConfig(name='Kalman', period_in_ms=100)
    for var in ('kalman.varPX', 'kalman.varPY', 'kalman.varPZ', 'pm.vbat'):
        logconf.add_variable(var, 'float')
    logconf.data_received_cb.add_callback(lambda ts, data, lc: history.append(data))
    cf.log.add_config(logconf)
    logconf.start()

    print('Жду, пока дрон определит своё положение (не трогай его)...')
    try:
        deadline = time.time() + ESTIMATOR_TIMEOUT
        while time.time() < deadline:
            last = history[-10:]
            if len(last) == 10 and all(max(d[k] for d in last) - min(d[k] for d in last) < 0.001
                                       for k in last[0] if k.startswith('kalman')):
                print('Положение определено')
                check_battery(last[-1]['pm.vbat'])
                return
            time.sleep(0.1)
    finally:
        logconf.stop()
        logconf.delete()

    if not history:
        sys.exit('Дрон не присылает данные. Выключи и включи дрон кнопкой, подожди 5 секунд '
                 'и запусти снова.')
    check_battery(history[-1]['pm.vbat'])
    # данные шлются 10 раз в секунду; если дошло меньше половины — виновата связь, а не пол
    if len(history) < ESTIMATOR_TIMEOUT * 10 / 2:
        sys.exit(f'Связь с дроном плохая: за {ESTIMATOR_TIMEOUT:.0f} с дошло {len(history)} из '
                 f'{ESTIMATOR_TIMEOUT * 10:.0f} сообщений. Выдерни и вставь Crazyradio, выключи и '
                 'включи дрон, поставь его ближе к компьютеру.')
    sys.exit('Дрон не может определить положение. Поставь его на пол с узором (ковёр), '
             'при хорошем свете, и не трогай во время запуска.')


def check_battery(vbat):
    print(f'Батарея: {vbat:.2f} В')
    if vbat < 3.6:
        sys.exit('Батарея почти села — заряди дрон (минут 30) и запусти снова.')
    if vbat < 3.8:
        print('Батарея садится — полёт будет коротким, лучше скоро зарядить.')


def find_drone():
    """Проверяет, что дрон отвечает по радио, чтобы не зависнуть на подключении."""
    if URI.rstrip('/') not in ('radio://0/80/2M', 'radio://0/80/2M/E7E7E7E7E7'):
        return  # нестандартный адрес — поиск по умолчанию его не найдёт, просто подключаемся
    for _ in range(3):
        if cflib.crtp.scan_interfaces():
            return
    sys.exit('Дрон не отвечает по радио. Проверь, что он включён (горят огоньки) и заряжен. '
             'Если только что включила — подожди 5 секунд и запусти снова.')


def emergency_stop(cf):
    """Аварийная остановка: моторы выключаются сразу, дрон падает (если его держат — остаётся в руке).
    После неё дрон заблокирован до перезагрузки — fly.py сам перезагрузит его при следующем запуске."""
    cf.commander.send_stop_setpoint()
    cf.supervisor.send_emergency_stop()


def stop_setpoints(mc):
    """Перестаёт слать дрону команды движения, не пытаясь садиться (после аварийной остановки)."""
    if mc._thread is not None:
        mc._thread.stop()
        mc._thread = None
    mc._is_flying = False


def is_locked(scf):
    """Заблокирован ли дрон (например, после аварийной остановки). Старая прошивка не скажет — False."""
    for _ in range(3):  # ответ иногда не успевает прийти — пробуем несколько раз
        try:
            return 'Is locked' in scf.cf.supervisor.read_state_list()
        except Exception:
            time.sleep(0.2)
    return False


def land_safely(mc):
    while True:
        try:
            mc.land()
            return
        except KeyboardInterrupt:
            print('Уже сажусь, подожди пару секунд...')


def in_background(func, timeout):
    """Запускает func, но ждёт не дольше timeout секунд. Возвращает (успела ли, ошибка)."""
    error = []

    def run():
        try:
            func()
        except Exception as e:
            error.append(e)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    thread.join(timeout)
    return not thread.is_alive(), (error[0] if error else None)


def connect():
    """Подключается к дрону. Если он не отвечает — не висит, а пишет, что делать."""
    print('Подключаюсь к дрону...')
    scf = SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache'))
    finished, error = in_background(scf.open_link, CONNECT_TIMEOUT)
    if finished and error is None:
        deadline = time.time() + CONNECT_TIMEOUT
        while not scf.is_params_updated() and time.time() < deadline:
            time.sleep(0.1)
        if scf.is_params_updated():
            return scf
    in_background(scf.cf.close_link, 5)
    sys.exit('Не получилось подключиться к дрону: связь обрывается. Чаще всего села батарея — '
             'заряди дрон. Ещё: выключи и включи дрон, выдерни и вставь обратно Crazyradio.')


def disconnect(scf):
    in_background(scf.close_link, 5)


def reboot_drone():
    """Перезагружает дрон по радио (как выключить и включить кнопкой) и ждёт, пока он проснётся."""
    switch = PowerSwitch(URI)
    switch.stm_power_cycle()
    switch.close()
    time.sleep(5.0)  # дрон загружается и калибруется — его нельзя трогать
    find_drone()


def connect_ready():
    """Подключается и готовит дрон к полёту: снимает блокировку после аварийной остановки,
    проверяет платы и датчики (если датчики зависли — сам перезагружает дрон).
    Возвращает (scf, боковые датчики, которые не работают). Используют fly.py и explore.py."""
    find_drone()
    scf = connect()
    try:
        if is_locked(scf):
            print('Дрон заблокирован после аварийной остановки — перезагружаю его, подожди 5 секунд...')
            disconnect(scf)
            reboot_drone()
            scf = connect()
        check_decks(scf)
        try:
            broken = check_ranges(scf)
        except SensorsStuck:
            # датчики иногда зависают и показывают 0; перезагрузка дрона их оживляет
            print('Датчики зависли (все показывают 0 или «ничего не вижу») — перезагружаю дрон, подожди 5 секунд '
                  'и не трогай его...')
            disconnect(scf)
            reboot_drone()
            scf = connect()
            check_decks(scf)
            try:
                broken = check_ranges(scf)
            except SensorsStuck:
                sys.exit(DECK_STUCK)
            print('Датчики снова работают')
        return scf, broken
    except BaseException:
        disconnect(scf)
        raise


def main():
    cfg = Config()
    cflib.crtp.init_drivers()
    scf, broken = connect_ready()
    try:
        fly(scf, cfg, broken)
    finally:
        disconnect(scf)


def fly(scf, cfg, broken):
    reset_estimator(scf)  # начало координат — точка взлёта

    pose = {}
    emergency = threading.Event()
    mc = MotionCommander(scf, default_height=cfg.cruise_height)

    def on_pose(ts, data, lc):
        pose.update(data)
        pose['time'] = time.time()
        # дрон перевернули вверх ногами — выключаем моторы сразу, прямо здесь, не дожидаясь
        # основного цикла: это работает даже во время взлёта. Тангаж (pitch) у дрона бывает
        # только от -90° до 90°, поэтому «вверх ногами» — это крен (roll) больше 90° в любую сторону
        upside_down = abs(data['stabilizer.roll']) > 90
        if upside_down and mc._is_flying and not emergency.is_set():
            emergency.set()
            emergency_stop(scf.cf)

    logconf = LogConfig(name='Pose', period_in_ms=50)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')
    logconf.add_variable('range.zrange', 'uint16_t')
    logconf.add_variable('stabilizer.roll', 'float')
    logconf.add_variable('stabilizer.pitch', 'float')
    logconf.data_received_cb.add_callback(on_pose)
    scf.cf.log.add_config(logconf)
    logconf.start()

    # без позиции взлетать нельзя — ждём первые данные, но не вечно
    deadline = time.time() + 3.0
    while 'time' not in pose and time.time() < deadline:
        time.sleep(0.1)
    if 'time' not in pose:
        logconf.stop()
        sys.exit('Дрон не присылает свою позицию. Выключи и включи дрон и запусти снова.')

    avoider = Avoider(cfg)
    avoider.broken |= broken
    frozen_steps = 0
    with Multiranger(scf) as mr:
        # взлёт внутри try: если нажать Ctrl+C даже во время взлёта, дрон всё равно сядет
        try:
            print('Взлетаю')
            mc.take_off()
            time.sleep(0.5)  # даём датчикам и оценщику устояться после взлёта
            # берём медиану из 5 замеров: один случайный «0» не должен сажать дрон
            readings = []
            if emergency.is_set():
                return
            for _ in range(5):
                readings.append((mr.front, mr.left, mr.right, mr.up))
                time.sleep(0.1)

            def median(i):
                values = sorted(4.0 if r[i] is None else r[i] for r in readings)
                return values[len(values) // 2]

            front, left, right, up = median(0), median(1), median(2), median(3)
            if max(front, left, right, up) < 0.01:
                print(DECK_STUCK.split('\n')[0] + ' Сажусь.')
                return
            if front < 0.10:
                print(f'Передний датчик показывает {front * 100:.0f} см — впереди вплотную что-то '
                      'стоит, или датчик закрыт. Без него лететь нельзя, сажусь.')
                return
            for side, name, value in (('left', 'Левый', left), ('right', 'Правый', right)):
                if value < 0.10 and side not in avoider.broken:
                    avoider.broken.add(side)
                    print(f'{name} датчик показывает {value * 100:.0f} см — похоже, он закрыт '
                          f'или неисправен. Не буду ему верить и облетать в ту сторону.')
            while True:
                if emergency.is_set():
                    break
                # боковые дальномеры зависли прямо в полёте (все 0 или все «ничего не вижу»)
                sides = [mr.front, mr.back, mr.left, mr.right]
                frozen = all(v is None for v in sides) or all(v == 0 for v in sides)
                frozen_steps = frozen_steps + 1 if frozen else 0
                if frozen_steps >= 5:
                    print('Датчики расстояния зависли в полёте — сажусь. Запусти снова: '
                          'программа сама перезагрузит дрон.')
                    break
                if time.time() - pose['time'] > LINK_LOST_TIMEOUT:
                    print('Связь с дроном пропадает (данные не приходят) — сажусь. '
                          'Скорее всего, садится батарея.')
                    break
                s = Sensors(
                    x=pose['stateEstimate.x'],
                    y=pose['stateEstimate.y'],
                    z=pose['stateEstimate.z'],
                    zrange=pose['range.zrange'] / 1000.0,
                    front=mr.front, back=mr.back, left=mr.left, right=mr.right, up=mr.up,
                )
                cmd = avoider.step(s)
                if cmd.land:
                    break
                mc.start_linear_motion(cmd.vx, cmd.vy, cmd.vz)
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
                      'Чтобы снова лететь, просто запусти fly.py — он сам перезагрузит дрон.')
            else:
                land_safely(mc)
                print('Сел')

    logconf.stop()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nОстановлено. Дрон не взлетал.')
