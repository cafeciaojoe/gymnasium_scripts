import signal
import threading
import time

import matplotlib.pyplot as plt

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

######################### PLAY WITH THESE NUMBERS ##################################

# NOTE: Press control + C to stop all motors and end the script.
# Turning a Crazyflie upside down stops that one on its own.

min_power = 1000   # Minimum motor power
max_power = 30000  # Maximum motor power. Warning: Avoid setting this above 30000
min_angle = 0      # The Crazyflie hovers while: min_angle < roll,pitch < max_angle
max_angle = 30

# Handy for tuning values. Prints the motor powers of every Crazyflie.
printing = False

# Show the power profile graphs before starting
plotting = True

####################################################################################

# Connection URIs for the Crazyflies. Ones that can't be reached are skipped.
uris = [
    'radio://0/40/2M/BADF00D005',
    'radio://0/40/2M/BADF00D006',
    'radio://0/40/2M/BADF00D007',
    'radio://0/40/2M/BADF00D008',
]

log_period = 100  # ms

# Latest attitude of each Crazyflie, keyed by URI
attitude = {}

# Set by Ctrl+C so every Crazyflie turns its motors off
stop_event = threading.Event()


def handle_ctrl_c(_signum, _frame):
    print('\n=== STOPPING ALL MOTORS ===')
    stop_event.set()


def attitude_callback(timestamp, data, logconf):
    # Extract URI from logconf.name
    uri = logconf.name.split(' ')[-1]
    attitude[uri] = (data['stateEstimate.roll'], data['stateEstimate.pitch'])


def start_logging(scf):
    log_conf = LogConfig(name='Attitude for ' + scf._link_uri, period_in_ms=log_period)
    log_conf.add_variable('stateEstimate.roll', 'float')
    log_conf.add_variable('stateEstimate.pitch', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(attitude_callback)
    log_conf.start()
    print(f'Started logging for         {scf._link_uri}')


def power_profile(angle):
    if abs(angle) > max_angle:
        power = int(max_power)
    else:
        power = int((min_power*max_angle + (max_power-min_power)*abs(angle))/(max_angle-min_angle))
    return power


def power_distribution(scf):
    roll, pitch = attitude.get(scf._link_uri, (0, 0))

    m1_p = 0
    m2_p = 0
    m3_p = 0
    m4_p = 0
    m1_r = 0
    m2_r = 0
    m3_r = 0
    m4_r = 0
    if pitch < 0:
        m1_p = power_profile(pitch)
        m4_p = power_profile(pitch)
    elif pitch > 0:
        m2_p = power_profile(pitch)
        m3_p = power_profile(pitch)
    if roll < 0:
        m3_r = power_profile(roll)
        m4_r = power_profile(roll)
    elif roll > 0:
        m1_r = power_profile(roll)
        m2_r = power_profile(roll)
    m1 = min(m1_p + m1_r, max_power)
    m2 = min(m2_p + m2_r, max_power)
    m3 = min(m3_p + m3_r, max_power)
    m4 = min(m4_p + m4_r, max_power)

    # Monitor output
    if printing:
        print(f'URI: {scf._link_uri}  M1: {m1:^5}  M2: {m2:^5}  M3: {m3:^5}  M4: {m4:^5}')

    scf.cf.param.set_value('motorPowerSet.m1', str(m1))
    scf.cf.param.set_value('motorPowerSet.m2', str(m2))
    scf.cf.param.set_value('motorPowerSet.m3', str(m3))
    scf.cf.param.set_value('motorPowerSet.m4', str(m4))


def vibration(scf):
    scf.cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(1)
    print(f'Ready to hover!             {scf._link_uri}')

    # Keep going until Ctrl+C or this Crazyflie is turned upside down
    while not stop_event.is_set() and abs(attitude.get(scf._link_uri, (0, 0))[0]) < 170:
        power_distribution(scf)
        time.sleep(0.1)

    scf.cf.param.set_value('motorPowerSet.m1', '0')
    scf.cf.param.set_value('motorPowerSet.m2', '0')
    scf.cf.param.set_value('motorPowerSet.m3', '0')
    scf.cf.param.set_value('motorPowerSet.m4', '0')
    time.sleep(0.5)
    scf.cf.param.set_value('motorPowerSet.enable', '0')
    print(f'Motors stopped              {scf._link_uri}')
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
        ax.set_ylabel(y_labels[i])
        ax.grid(True)

    fig2.tight_layout()
    print('Close the graphs to start...')
    plt.show()


def filter_uris(uris):
    valid_uris = []
    for uri in uris:
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print(f'Successfully connected to   {uri}')
                valid_uris.append(uri)
        except Exception as e:
            print(f'Failed to connect to {uri}: {e}')
    return valid_uris


if __name__ == '__main__':
    print('=== SWARM HOVER SIMULATION ===')

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    # Filter URIs to only include valid connections
    valid_uris = filter_uris(uris)

    if not valid_uris:
        print('No valid Crazyflie connections found. Exiting.')
        exit()

    if plotting:
        simple_plot()

    with Swarm(valid_uris, factory=factory) as swarm:
        # Not resetting estimators or arming the Crazyflies as they are not flying

        swarm.parallel_safe(start_logging)
        time.sleep(1)

        signal.signal(signal.SIGINT, handle_ctrl_c)
        swarm.parallel_safe(vibration)
