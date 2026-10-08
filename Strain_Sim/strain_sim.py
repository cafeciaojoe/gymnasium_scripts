import math
import signal
import threading
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander

# Change uris according to your setup
# URIs in a swarm using the same radio must also be on the same channel
Sensor_A = 'radio://0/40/2M/BADF00D009'  # Hand-held sensor drone (does not fly)
Sensor_B = 'radio://0/40/2M/BADF00D004'  # Hand-held sensor drone (does not fly)
Flyer = 'radio://0/40/2M/BADF00D007'     # The drone whose height is controlled

# All three drones the script connects to
uris = {
    Sensor_A,
    Sensor_B,
    Flyer,
}

# ---- Inputs ----
H_MIN = 0.2   # Flyer height [m] when the sensors are at (or closer than) D_MIN
H_MAX = 1.5   # Flyer height [m] when the sensors are at (or further than) D_MAX
D_MIN = 0.1   # Minimum distance between the sensor drones [m]
D_MAX = 1.0   # Maximum distance between the sensor drones [m]

K_P = 1.5           # Gain from height error to vertical velocity
MAX_VELOCITY = 0.5  # Maximum vertical velocity [m/s]
SENSOR_TIMEOUT = 0.5  # Land if a sensor sends no position for this long [s]

# Latest data from each drone, filled in by the log callbacks below
positions = {uri: None for uri in uris}
last_update = {uri: 0.0 for uri in uris}

# Set by Ctrl+C so the flyer lands instead of the script dying mid-air
stop_event = threading.Event()


def handle_ctrl_c(_signum, _frame):
    print('\nCtrl+C pressed, landing...')
    stop_event.set()


# Waits until the drone has sent us its list of settings
def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)


def arm(scf):
    # Only the flyer needs its motors armed
    if scf.cf.link_uri == Flyer:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)


# Runs every time a drone sends its data (every 10 ms).
# Saves the numbers and notes the time, so we can tell if a drone goes quiet.
def position_callback(uri, data):
    positions[uri] = (data['stateEstimate.x'],
                      data['stateEstimate.y'],
                      data['stateEstimate.z'])
    last_update[uri] = time.time()


# Asks a drone to send us its data every 10 ms
def start_position_printing(scf):
    log_conf1 = LogConfig(name='Position', period_in_ms=10)
    log_conf1.add_variable('stateEstimate.x', 'float')
    log_conf1.add_variable('stateEstimate.y', 'float')
    log_conf1.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(log_conf1)
    log_conf1.data_received_cb.add_callback(lambda _timestamp, data, _logconf: position_callback(scf.cf.link_uri, data))
    log_conf1.start()


# Straight-line distance between the two sensor drones (Pythagoras in 3D)
def sensor_distance():
    a = positions[Sensor_A]
    b = positions[Sensor_B]
    return math.sqrt(pow(a[0]-b[0], 2) + pow(a[1]-b[1], 2) + pow(a[2]-b[2], 2))


def distance_to_height(dist):
    '''
    Linearly maps the distance between the sensor drones onto the
    flyer's height: D_MIN -> H_MIN and D_MAX -> H_MAX. Distances
    outside [D_MIN, D_MAX] are clamped to the end heights.
    '''
    dist = min(max(dist, D_MIN), D_MAX)  # Keep the distance inside the range
    # How far along the distance range we are (0 to 1), scaled onto the height range
    return H_MIN + (dist - D_MIN) / (D_MAX - D_MIN) * (H_MAX - H_MIN)


def dropped_sensor():
    '''Returns the URI of a sensor that has gone quiet, or None.'''
    now = time.time()
    for uri in (Sensor_A, Sensor_B):
        if now - last_update[uri] > SENSOR_TIMEOUT:
            return uri
    return None


def strain_sim(scf):
    # This runs on every drone at once, but only the flyer has work to do
    if scf.cf.link_uri != Flyer:
        return

    # Don't start until every drone has sent at least one position
    print('Waiting for position data from all drones...')
    while None in positions.values():
        if stop_event.is_set():
            return
        time.sleep(0.1)

    # Take off straight to the height the sensors are currently asking for.
    # MotionCommander takes off when the block starts and lands if it crashes out.
    start_height = distance_to_height(sensor_distance())
    with MotionCommander(scf, default_height=start_height) as mc:
        # Fly until Ctrl+C or a sensor drops out
        while not stop_event.is_set():
            dropped = dropped_sensor()
            if dropped is not None:
                print(f'\nLost sensor {dropped}, landing...')
                break

            dist = sensor_distance()
            target = distance_to_height(dist)
            # Further from the target height = faster up/down speed (K_P sets how much faster)
            vel_z = K_P * (target - positions[Flyer][2])
            vel_z = min(max(vel_z, -MAX_VELOCITY), MAX_VELOCITY)  # Speed limit

            mc.start_linear_motion(0, 0, vel_z)  # No sideways movement, just up/down
            print(f'distance: {dist:.2f} m  target height: {target:.2f} m', end='\r')
            time.sleep(0.01)

        print()
        mc.land()


if __name__ == '__main__':
    cflib.crtp.init_drivers()  # Start the radio

    # Connect to all drones. The cache saves their settings lists so the next connect is faster.
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:

        swarm.reset_estimators()  # Make every drone re-find its position from scratch

        print('Waiting for parameters to be downloaded...')
        swarm.parallel_safe(wait_for_param_download)
        time.sleep(0.5)

        swarm.parallel_safe(arm)
        time.sleep(0.5)

        swarm.parallel_safe(start_position_printing)
        time.sleep(0.5)

        # From here on Ctrl+C lands the flyer instead of killing the script
        signal.signal(signal.SIGINT, handle_ctrl_c)
        swarm.parallel_safe(strain_sim)  # Runs strain_sim on every drone at once
        time.sleep(0.5)

        swarm.close_links()
