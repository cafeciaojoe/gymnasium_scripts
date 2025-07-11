import time

import matplotlib.pyplot as plt
import numpy as np
import math

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Connection URI for the Crazyflie
uris = [
'radio://0/30/2M/a0a0a0a0aa',
'radio://0/30/2M/a0a0a0a0ae'
]

# Global dictionary to store 3d acceleration data for each Crazyflie
acc_3d_dict = {}

TimePer = 20  # ms  How fast we log data
SampleTime = 2  # s
samples = int(SampleTime * 1000 / TimePer)  # Number of samples

max_power = 50000

global execute
execute = True


def acceleration_callback(timestamp, data, logconf):
    acc_x = data['stateEstimate.ax']
    acc_y = data['stateEstimate.ay']
    acc_z = data['stateEstimate.az']

    # Extract URI from logconf.name
    uri = logconf.name.split(' ')[-1]

    acc_3d = (math.sqrt(acc_x[-1]**2 + acc_y[-1]**2 + acc_z[-1]**2))

    # Ensure each Crazyflie has its own key/list in the dict
    if uri not in acc_3d_dict:
        acc_3d_dict[uri] = []

    #add the data to the list and keep it the length of samples specified at the top of the script. 
    acc_3d_dict[uri].append(acc_3d)
    if len(acc_3d_dict[uri]) > samples:
        acc_3d_dict[uri].pop(0)


def start_logging(scf):
    log_conf = LogConfig(name='Acceleration for '+ scf._link_uri, period_in_ms=TimePer)
    log_conf.add_variable('stateEstimate.ax', 'float')
    log_conf.add_variable('stateEstimate.ay', 'float')
    log_conf.add_variable('stateEstimate.az', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(acceleration_callback)
    log_conf.start()
    print(f"Started logging for {scf._link_uri}")


def vibration(scf):
    scf.cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(1)

    while execute == True:
        power_calculator(scf)
        time.sleep(.05)
    
    time.sleep(1)
    # Turn off all motors
    scf.cf.param.set_value('motorPowerSet.m1', '0')
    scf.cf.param.set_value('motorPowerSet.m2', '0')
    scf.cf.param.set_value('motorPowerSet.m3', '0')
    scf.cf.param.set_value('motorPowerSet.m4', '0')
    time.sleep(1)

def power_calculator(scf):
    # Extract URI from scf
    uri = scf._link_uri

    # Ensure the URI exists in the dictionary and has data
    if uri not in acc_3d_dict or not acc_3d_dict[uri]:
        print(f"No acceleration data available for {uri}. Skipping power calculation.")
        return

    # look up the right acc_3d list from the global dict
    acc_3d = acc_3d_dict[uri]

    mean_acc = sum(acc_3d) / len(acc_3d)
    power = min(int((mean_acc/.5)*max_power), max_power)

    scf.cf.param.set_value('motorPowerSet.m1', str(power))
    scf.cf.param.set_value('motorPowerSet.m2', str(power))
    scf.cf.param.set_value('motorPowerSet.m3', str(power))
    scf.cf.param.set_value('motorPowerSet.m4', str(power))

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
        start_logging(scf)
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
        scf.close_link()
        time.sleep(0.5)
        plot_acc(acc_3d)

if __name__ == '__main__':
    print("=== ACCELERATION VIBRATION ===")
    print("Vibration intensity based on acceleration!")

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:
    # not resetting estimators or arming the crazyflie as it it not flying

        swarm.parallel_safe(start_logging)
        time.sleep(1)

        try: 
            swarm.parallel_safe(vibration)

        except KeyboardInterrupt:
            print("\n=== STOPPING ALL MOTORS ===")
            execute = False 
            swarm.parallel_safe(vibration)
