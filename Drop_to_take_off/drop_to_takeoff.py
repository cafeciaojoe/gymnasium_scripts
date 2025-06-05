import time

import matplotlib.pyplot as plt
import numpy as np

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

Uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

acc_z = []
TimePer = 100  # ms
Takeoff = False
Executing = False


def acceleration_callback(timestamp, data, logconf):
    global Executing, Takeoff

    acc_z.append(data['acc.z'])

    if Executing is False:
        if acc_z[-1] < 0.1:
            Takeoff = True
            Executing = True


def start_acceleration_printing(scf):
    log_conf = LogConfig(name='Acceleration', period_in_ms=TimePer)
    log_conf.add_variable('acc.z', 'float')
    log_conf.add_variable('range.zrange', 'uint16_t')
    log_conf.add_variable('posEstAlt.estimatedZ', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(acceleration_callback)
    log_conf.start()


def plot_acc(list1):
    time = np.arange(len(list1)) * TimePer * 0.001

    plt.figure()
    plt.plot(time, list1, label='Acc z')

    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (Gs)')
    plt.title('Acceleration over Time')
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with SyncCrazyflie(Uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(0.5)
        start_acceleration_printing(scf)
        time.sleep(1)
        scf.cf.platform.send_arming_request(True)
        Terminate = False
        while Terminate is False:
            if Takeoff is True:
                scf.cf.high_level_commander.go_to(0, 0, 0, 0, 2, relative=True)
                time.sleep(3)
                print('Landing...')
                scf.cf.high_level_commander.land(0, 4)
                time.sleep(4.5)
                Terminate = True
            time.sleep(0.001)
        time.sleep(0.5)
        scf.cf.platform.send_arming_request(False)
        time.sleep(0.5)
        scf.close_link()
        time.sleep(0.5)
        plot_acc(acc_z)
