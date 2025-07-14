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


######################### PLAY WITH THESE NUMBERS ##################################

# Motor power settings 
max_power = 20000  # Maximum motor power 

# if invert is true then more acceleration makes less vibration, being still produces max power.
invert = True

# Handy for tuning values when connected to one crazyflie. 
printing = False

# Smoothing, more samples, smoother response
samples = 4

####################################################################################

# Connection URI for the Crazyflie
uris = [
'radio://0/30/2M/a0a0a0a0aa',
'radio://0/30/2M/a0a0a0a0ae',
'radio://0/30/2M/e7e7e7e7e8'
]

# Global dictionary to store 3d acceleration data for each Crazyflie
acc_3d_dict = {}

# TODO FIND LOG PERIOD THAT SUITS THE BANDWIDTH, 4 DRONES 
log_period = 40 #ms

global execute
execute = True


def acceleration_callback(timestamp, data, logconf):
    acc_x = data['stateEstimate.ax']
    acc_y = data['stateEstimate.ay']
    acc_z = data['stateEstimate.az']

    # Extract URI from logconf.name
    uri = logconf.name.split(' ')[-1]

    acc_3d = (math.sqrt(acc_x**2 + acc_y**2 + acc_z**2))

    # Ensure each Crazyflie has its own key/list in the dict
    if uri not in acc_3d_dict:
        acc_3d_dict[uri] = []

    #add the data to the list and keep it the length of samples specified at the top of the script. 
    acc_3d_dict[uri].append(acc_3d)
    if len(acc_3d_dict[uri]) > samples:
        acc_3d_dict[uri].pop(0)


def start_logging(scf):
    log_conf = LogConfig(name='Acceleration for '+ scf._link_uri, period_in_ms=log_period)
    log_conf.add_variable('stateEstimate.ax', 'float')
    log_conf.add_variable('stateEstimate.ay', 'float')
    log_conf.add_variable('stateEstimate.az', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(acceleration_callback)
    log_conf.start()
    print(f"Started logging for         {scf._link_uri}")


def vibration(scf):
    scf.cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(1)

    if execute == True:
        print(f'Ready to vibrate!           {scf._link_uri}')

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

    if invert == True:
        power = max_power - min(int((mean_acc / 0.5) * max_power), max_power)
    else:
        power = min(int((mean_acc/.5)*max_power), max_power)

    # Monitor output 
    if printing == True:
        print(f'URI: {scf._link_uri}, Angular velocity: {mean_acc:.1f}°/s → Motor power: {power}')
    

    scf.cf.param.set_value('motorPowerSet.m1', str(power))
    scf.cf.param.set_value('motorPowerSet.m2', str(power))
    scf.cf.param.set_value('motorPowerSet.m3', str(power))
    scf.cf.param.set_value('motorPowerSet.m4', str(power))

def filter_uris(uris):
    valid_uris = []
    for uri in uris:
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print(f"Successfully connected to   {uri}")
                valid_uris.append(uri)
        except Exception as e:
            print(f"Failed to connect to {uri}: {e}")
    return valid_uris

if __name__ == '__main__':
    print("=== ACCELERATION VIBRATION ===")
    print("Vibration intensity based on acceleration!")

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    # Filter URIs to only include valid connections
    valid_uris = filter_uris(uris)

    if not valid_uris:
        print("No valid Crazyflie connections found. Exiting.")
        exit()

    with Swarm(valid_uris, factory=factory) as swarm:
        # Not resetting estimators or arming the Crazyflie as it is not flying

        swarm.parallel_safe(start_logging)
        time.sleep(1)

        try:
            swarm.parallel_safe(vibration)

        except KeyboardInterrupt:
            print("\n=== STOPPING ALL MOTORS ===")
            execute = False
            swarm.parallel_safe(vibration)