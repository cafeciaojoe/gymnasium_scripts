import time

import matplotlib.pyplot as plt
import numpy as np
import math

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

Uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

acc_x = []
acc_y = []
acc_z = []
acc_3d = []

TimePer = 20  # ms  How fast we log data
SampleTime = 2  # s
samples = int(SampleTime * 1000 / TimePer)  # Number of samples


def acceleration_callback(timestamp, data, logconf):
    global Executing
    acc_x.append(data['stateEstimate.ax'])
    acc_y.append(data['stateEstimate.ay'])
    acc_z.append(data['stateEstimate.az'])

    acc_3d.append(math.sqrt(acc_x[-1]**2 + acc_y[-1]**2 + acc_z[-1]**2))


def start_acceleration_printing(scf):
    log_conf = LogConfig(name='Acceleration', period_in_ms=TimePer)
    log_conf.add_variable('stateEstimate.ax', 'float')
    log_conf.add_variable('stateEstimate.ay', 'float')
    log_conf.add_variable('stateEstimate.az', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(acceleration_callback)
    log_conf.start()


def vibration(scf, power):
    scf.cf.param.set_value('motorPowerSet.m1', str(power))
    scf.cf.param.set_value('motorPowerSet.m2', str(power))
    scf.cf.param.set_value('motorPowerSet.m3', str(power))
    scf.cf.param.set_value('motorPowerSet.m4', str(power))


def power_calculator():
    mean_acc = sum(acc_3d[-samples:]) / len(acc_3d[-samples:])
    power = min(int((mean_acc/.5)*60000), 60000)
    return power


def plot_acc(list1):
    time = np.arange(len(list1)) * TimePer * 0.001

    plt.figure()
    plt.plot(time, list1, label='Acc Magnitude')

    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [Gs]')
    plt.title('Acceleration over Time')
    plt.legend()
    plt.grid()
    plt.show()


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    with SyncCrazyflie(Uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(0.5)
        scf.cf.param.set_value('motorPowerSet.enable', '0')
        start_acceleration_printing(scf)
        time.sleep(1)
        scf.cf.platform.send_arming_request(True)
        time.sleep(0.5)
        scf.cf.param.set_value('motorPowerSet.enable', '1')
        time.sleep(0.5)

        start_time = time.time()
        while time.time()-start_time < 60:  # Duration of the script
            pow = power_calculator()
            vibration(scf, pow)
            time.sleep(0.2)

        time.sleep(0.5)
        scf.cf.param.set_value('motorPowerSet.enable', '0')
        time.sleep(0.5)
        scf.cf.platform.send_arming_request(False)
        time.sleep(0.5)
        scf.close_link()
        time.sleep(0.5)
        plot_acc(acc_3d)
