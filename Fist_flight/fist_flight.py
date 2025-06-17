import time

import matplotlib.pyplot as plt
import numpy as np

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

Uri_sensor = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
Uri_drone = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')


acc_x = []
acc_y = []
acc_z = []
z = []
TimePer = 100  # ms
acc_threshold = 2.0  # Gs
GoForward = False
GoBack = False
GoLeft = False
GoRight = False
GoUp = False
GoDown = False
Terminate = False
Executing = False


def position_callback(timestamp, data, logconf):
    z.append(data['stateEstimate.z'])


def start_position_printing(scf):
    log_conf1 = LogConfig(name='Position', period_in_ms=TimePer)
    log_conf1.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(log_conf1)
    log_conf1.data_received_cb.add_callback(position_callback)
    log_conf1.start()


def acceleration_callback(timestamp, data, logconf):
    global GoForward, GoBack
    global GoLeft, GoRight
    global GoUp, GoDown
    global Executing

    acc_x.append(data['acc.x'])
    acc_y.append(data['acc.y'])
    acc_z.append(data['acc.z']-1)

    last_values = {'acc_x': acc_x[-1], 'acc_y': acc_y[-1], 'acc_z': acc_z[-1]}
    max_magnitude = max(last_values, key=lambda k: abs(last_values[k]))
    max_acc = last_values[max_magnitude]

    if Executing is False:
        if abs(max_acc) > acc_threshold:
            Executing = True

            if max_magnitude == 'acc_x':
                if max_acc > 0:
                    GoForward = True
                elif max_acc < 0:
                    GoBack = True
            elif max_magnitude == 'acc_y':
                if max_acc > 0:
                    GoLeft = True
                elif max_acc < 0:
                    GoRight = True
            elif max_magnitude == 'acc_z':
                if max_acc > 0:
                    GoUp = True
                elif max_acc < 0:
                    GoDown = True


def start_acceleration_printing(scf):
    log_conf = LogConfig(name='Acceleration', period_in_ms=TimePer)
    log_conf.add_variable('acc.x', 'float')
    log_conf.add_variable('acc.y', 'float')
    log_conf.add_variable('acc.z', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(acceleration_callback)
    log_conf.start()


def flight_commands(mc, scf):
    global GoForward, GoBack
    global GoLeft, GoRight
    global GoUp, GoDown
    global Executing, Terminate
    timeTest = 0.2
    distance = 0.4
    while Terminate is False:

        if GoUp is True:
            if mc._is_flying is False:
                print('Taking off')
                mc.take_off(2*distance)
            elif mc._is_flying is True:
                print('Going up')
                mc.up(distance, 1)
            GoUp = False
            time.sleep(timeTest)
            print('Go')
            scf.cf.param.set_value('sound.effect', '7')
            Executing = False

        if GoDown is True:
            if z[-1] < distance:
                print('Landing')
                mc.land()
                GoDown = False
                time.sleep(timeTest)
                Executing = False
                Terminate = True
            else:
                print('Going down')
                mc.down(distance, 1)
                GoDown = False
                time.sleep(timeTest)
                Executing = False
                print('Go')
                scf.cf.param.set_value('sound.effect', '7')

        if GoForward is True:
            print('Going forward')
            mc.forward(distance, 1)
            GoForward = False
            time.sleep(timeTest)
            Executing = False
            print('Go')
            scf.cf.param.set_value('sound.effect', '7')

        if GoBack is True:
            print('Going back')
            mc.back(distance, 1)
            GoBack = False
            time.sleep(timeTest)
            Executing = False
            print('Go')
            scf.cf.param.set_value('sound.effect', '7')

        if GoLeft is True:
            print('Going left')
            mc.left(distance, 1)
            GoLeft = False
            time.sleep(timeTest)
            Executing = False
            print('Go')
            scf.cf.param.set_value('sound.effect', '7')

        if GoRight is True:
            print('Going right')
            mc.right(distance, 1)
            GoRight = False
            time.sleep(timeTest)
            Executing = False
            print('Go')
            scf.cf.param.set_value('sound.effect', '7')

        time.sleep(0.001)


def plot_three_acc(list1, list2, list3):
    time = np.arange(len(list1)) * TimePer * 0.001

    plt.figure(figsize=(10, 6))
    plt.plot(time, list1, label='Acc x')
    plt.plot(time, list2, label='Acc y')
    plt.plot(time, list3, label='Acc z')
    plt.axhline(acc_threshold, xmax=len(list1)*TimePer*0.001, color='red')
    plt.axhline(-acc_threshold, xmax=len(list1)*TimePer*0.001, color='red')

    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (Gs)')
    plt.title('Acceleration over Time')
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with SyncCrazyflie(Uri_sensor, cf=Crazyflie(rw_cache='./cache')) as scf_s:
        with SyncCrazyflie(Uri_drone, cf=Crazyflie(rw_cache='./cache')) as scf_d:
            mc = MotionCommander(scf_d)
            scf_d.cf.platform.send_arming_request(True)
            time.sleep(0.5)
            start_acceleration_printing(scf_s)
            start_position_printing(scf_d)
            time.sleep(1)
            scf_s.cf.param.set_value('sound.effect', '7')
            flight_commands(mc, scf_s)
            time.sleep(0.5)
            scf_d.cf.platform.send_arming_request(False)
            time.sleep(0.5)
            scf_s.close_link()
            scf_d.close_link()
            time.sleep(0.5)
            plot_three_acc(acc_x, acc_y, acc_z)
