import time
import threading

import matplotlib.pyplot as plt

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/30/2M/a0a0a0a0aa')
URI = uri_helper.uri_from_env(default='usb://0')

# Crazyflie's attitude
roll = [0]
pitch = [0]

start_roll = 0
start_pitch = 0 

min_power = 7000  # Minimum motor power
max_power = 45000  # Maximum motor power
min_angle = 0   # The Crazyflie hovers while: min_angle < roll,pitch < max_angle
max_angle = 45


def attitude_callback(timestamp, data, logconf):
    roll.append(data['stateEstimate.roll'])
    pitch.append(data['stateEstimate.pitch'])

def start_position_printing(scf):
    log_conf = LogConfig(name='Attitude', period_in_ms=100)
    log_conf.add_variable('stateEstimate.roll', 'float')
    log_conf.add_variable('stateEstimate.pitch', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(attitude_callback)
    log_conf.start()


def power_profile(angle):
    if abs(angle) > max_angle:
        power = int(max_power)
    else:
        #power = int((min_power*max_angle + (max_power-min_power)*abs(angle))/(max_angle-min_angle))
        exponent = 2.5  # Adjust this value: higher = later peaking
        power = int(min_power + (max_power - min_power) * (abs(angle) / max_angle) ** exponent)
    return power


def power_distribution():
    m1_p = 0
    m2_p = 0
    m3_p = 0
    m4_p = 0
    m1_r = 0
    m2_r = 0
    m3_r = 0
    m4_r = 0
    if pitch[-1] < start_pitch:
        m1_p = power_profile(pitch[-1]-start_pitch)
        m4_p = power_profile(pitch[-1]-start_pitch)
    elif pitch[-1] > start_pitch:
        m2_p = power_profile(pitch[-1]-start_pitch)
        m3_p = power_profile(pitch[-1]-start_pitch)
    if roll[-1] < start_roll:
        m3_r = power_profile(roll[-1]-start_roll)
        m4_r = power_profile(roll[-1]-start_roll)
    elif roll[-1] > start_roll:
        m1_r = power_profile(roll[-1]-start_roll)
        m2_r = power_profile(roll[-1]-start_roll)
    m1 = min(m1_p + m1_r, max_power)
    m2 = min(m2_p + m2_r, max_power)
    m3 = min(m3_p + m3_r, max_power)
    m4 = min(m4_p + m4_r, max_power)
    #print(m1, m2, m3, m4)
    print(f'roll = {roll[-1]-start_roll}, {start_roll} pitch = {pitch[-1]-start_pitch}, {start_pitch}')

    scf.cf.param.set_value('motorPowerSet.m1', str(m1))
    scf.cf.param.set_value('motorPowerSet.m2', str(m2))
    scf.cf.param.set_value('motorPowerSet.m3', str(m3))
    scf.cf.param.set_value('motorPowerSet.m4', str(m4))


def vibration(scf):
    scf.cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(1)

    # Flag to control the loop
    running = True
    
    def wait_for_enter():
        nonlocal running
        input("Press Enter to stop vibration...")
        running = False
    
    # Start the input thread
    input_thread = threading.Thread(target=wait_for_enter)
    input_thread.daemon = True
    input_thread.start()
    
    print("Vibration started. Press Enter to stop...")
    
    while running:
        power_distribution()
        time.sleep(0.1)

    scf.cf.param.set_value('motorPowerSet.m1', 0)
    scf.cf.param.set_value('motorPowerSet.m2', 0)
    scf.cf.param.set_value('motorPowerSet.m3', 0)
    scf.cf.param.set_value('motorPowerSet.m4', 0)
    time.sleep(0.5)
    scf.cf.param.set_value('motorPowerSet.enable', '0')
    time.sleep(1)

def simple_plot():
    points = [
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 4 roll
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 1 roll
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 3 roll
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 2 roll
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 4 pitch
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 1 pitch
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 3 pitch
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 2 pitch
    ]
    titles = ['Motor 4', 'Motor 1', 'Motor 3', 'Motor 2']
    y_labels = ['M4 power', 'M1 power', 'M3 power', 'M2 power']

    fig1, axs1 = plt.subplots(2, 2, figsize=(10, 8))

    for i, ax in enumerate(axs1.flat):
        x_vals, y_vals = zip(*points[i])
        ax.plot(x_vals, y_vals, marker='o')
        ax.set_title(titles[i])
        ax.set_xlabel('Roll [deg]')
        ax.set_ylabel(y_labels[i])
        ax.grid(True)

    fig1.tight_layout()

    fig2, axs2 = plt.subplots(2, 2, figsize=(10, 8))

    for i, ax in enumerate(axs2.flat):
        x_vals, y_vals = zip(*points[i+4])
        ax.plot(x_vals, y_vals, marker='o', color='orange')
        ax.set_title(titles[i])
        ax.set_xlabel('Pitch [deg]')
        ax.set_ylabel(y_labels[1])
        ax.grid(True)

    fig2.tight_layout()
    print('Close the graphs to start...')
    plt.show()

def get_start_pose():
    global start_pitch, start_roll
    input("Press Enter to set start pose...")  # This waits for Enter key
    start_pitch = pitch[-1]
    start_roll = roll[-1]
    print(f"start pose set! roll = {start_roll} and pitch = {start_pitch}")


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    # what does this do?
    factory = CachedCfFactory(rw_cache='./cache')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        #simple_plot()
        start_position_printing(scf)
        time.sleep(1)
        get_start_pose()
        vibration(scf)
