import math
import random
import time

import matplotlib.pyplot as plt
import numpy as np

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Crazyflie's position
x1 = [0]
y1 = [0]
z1 = [0]

radius = 5  # Vibrations start when d <= radius
min_power = 1000  # Minimum motor power
max_power = 50000  # Maximum motor power
duration = 10000  # Duration of the script
CURVE_TYPE = 1  # 1 for Linear and 2 for Exponential

Stop = False

def random_3d_point():
    # Define the limits of the flying space
    x = [random.uniform(-2.8, 2.6)]
    y = [random.uniform(-3.0, 0.6)]
    z = [random.uniform(0.2, 1.8)]
    return (x, y, z)


def position_callback(timestamp, data, logconf):
    global d
    x1.append(data['stateEstimate.x'])
    y1.append(data['stateEstimate.y'])
    z1.append(data['stateEstimate.z'])

    d = math.sqrt(pow((x1[-1]-x2[-1]), 2)+pow((y1[-1]-y2[-1]), 2)+pow((z1[-1]-z2[-1]), 2))


def start_position_printing(scf):
    log_conf = LogConfig(name='Position', period_in_ms=100)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def power_distribution(pow):
    '''
    Here, we can choose how to power each motor: either
    apply the same power to all motors simultaneously, or
    activate them one by one for more spreaded distribution.
    '''
    scf.cf.param.set_value('motorPowerSet.m1', str(pow))
    scf.cf.param.set_value('motorPowerSet.m2', str(pow))
    scf.cf.param.set_value('motorPowerSet.m3', str(pow))
    scf.cf.param.set_value('motorPowerSet.m4', str(pow))


def power_calculator(dist):
    if CURVE_TYPE == 1:
        power = int((min_power-max_power)/radius * dist + max_power)
    elif CURVE_TYPE == 2:
        b = math.log((min_power+1) / max_power) / (radius)
        a = max_power
        power = int(a * math.exp(b * dist))
    return power


def vibration(scf):
    scf.cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(1)
    global Stop
    while Stop is False:
        if d <= radius:
            power = power_calculator(d)
            power_distribution(power)
            pow_percentage = int((power-min_power)*100/max_power)
            print(f'Distance from target:{d:.3f}, Motor power:{pow_percentage}%')
            time.sleep(0.1)
            if pow_percentage >= 90: #  Could replace this with a distance expression
                scf.cf.param.set_value('sound.effect', '7')
                Stop = True
                time.sleep(0.5)
        else:
            power_distribution(0)
            print('Out of radius. Move closer to the target')
            time.sleep(0.1)

    power_distribution(0)
    time.sleep(1)
    scf.cf.param.set_value('motorPowerSet.enable', '0')
    time.sleep(1)


def simple_plot():
    x_vals = np.linspace(0, radius, 200)
    y_vals = [0] * len(x_vals)
    for i in range(len(x_vals)):
        y_vals[i] = power_calculator(x_vals[i])
    plt.plot(x_vals, y_vals, 'ro-')
    plt.xlabel('Distance')
    plt.ylabel('Power to motors')
    plt.yticks(np.arange(0, max_power+10000, 5000))
    plt.grid()
    print('Close the graph to start...')
    plt.show()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')
    simple_plot()

    x2, y2, z2 = random_3d_point()
    print(x2, y2, z2)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        start_position_printing(scf)
        time.sleep(1)
        try:
            vibration(scf)
        except KeyboardInterrupt:
            print("\n=== STOPPING ALL MOTORS ===")
            Stop = True
            vibration(scf)
