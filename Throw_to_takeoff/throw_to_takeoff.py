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

vel_z = []
acc_z = []
TimePer = 100  # ms
Takeoff = False
Executing = False


def z_axis_callback(timestamp, data, logconf):
    global Executing, Takeoff

    vel_z.append(data['stateEstimate.vz'])
    acc_z.append(data['acc.z'])

    if Executing is False:
        if acc_z[-1] < 0.1 and vel_z[-1] < 0.05:
            Takeoff = True
            Executing = True


def start_callback_printing(scf):
    log_conf = LogConfig(name='Z_axis', period_in_ms=TimePer)
    log_conf.add_variable('stateEstimate.vz', 'float')
    log_conf.add_variable('acc.z', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(z_axis_callback)
    log_conf.start()


def simple_plot(list1, list2):
    time1 = np.arange(len(list1)) * TimePer * 0.001
    time2 = np.arange(len(list2)) * TimePer * 0.001

    fig, axes = plt.subplots(1, 2)

    # Plot for list1 vs time1
    axes[0].plot(time1, list1, label='Acc z')
    axes[0].set_xlabel('Time [s]')
    axes[0].set_ylabel('Acceleration [Gs]')
    axes[0].set_title('Acceleration over Time')
    axes[0].legend()
    axes[0].grid()

    # Plot for list2 vs time2
    axes[1].plot(time2, list2, label='Vel z', color='orange')
    axes[1].set_xlabel('Time [s]')
    axes[1].set_ylabel('Velocity [m/s]')
    axes[1].set_title('Velocity over Time')
    axes[1].legend()
    axes[1].grid()

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with SyncCrazyflie(Uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(0.5)
        start_callback_printing(scf)
        time.sleep(1)
        scf.cf.platform.send_arming_request(True)
        Terminate = False
        print('Ready to takeoff...')
        while Terminate is False:
            if Takeoff is True:
                scf.cf.high_level_commander.go_to(0, 0, 0, 0, 2, relative=True)
                time.sleep(3)
                print('Landing...')
                scf.cf.high_level_commander.land(0, 4)
                time.sleep(4.1)
                Terminate = True
            time.sleep(0.001)
        time.sleep(0.5)
        scf.cf.platform.send_arming_request(False)
        time.sleep(0.5)
        scf.close_link()
        time.sleep(0.5)
        simple_plot(acc_z, vel_z)
