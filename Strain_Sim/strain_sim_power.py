import math
import signal
import threading
import time

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm

# Change uris according to your setup
# URIs in a swarm using the same radio must also be on the same channel
Sensor_A = 'radio://0/40/2M/BADF00D009'  # Hand-held sensor drone (does not fly)
Sensor_B = 'radio://0/40/2M/BADF00D004'  # Hand-held sensor drone (does not fly)
Flyer = 'radio://0/40/2M/BADF00D007'     # The drone whose thrust is controlled

# All three drones the script connects to
uris = {
    Sensor_A,
    Sensor_B,
    Flyer,
}

# ---- Inputs ----
# The flyer takes off to TAKEOFF_HEIGHT and measures the thrust it needs
# to hover. After that the distance between the sensors sets its thrust
# directly, relative to that hover thrust. There is no height hold
# below H_MAX: more thrust than hover climbs, less sinks.
TAKEOFF_HEIGHT = 0.5  # Height the flyer hovers at while measuring [m]
THRUST_BELOW = 8000   # Thrust below hover when the sensors are at (or closer than) D_MIN
THRUST_ABOVE = 6000   # Thrust above hover when the sensors are at (or further than) D_MAX
D_MIN = 0.1   # Minimum distance between the sensor drones [m]
D_MAX = 1.0   # Maximum distance between the sensor drones [m]

# Ceiling: above H_MAX the thrust is capped below hover, so the flyer sinks
H_MAX = 1.5            # Maximum flyer height [m]
K_CEILING = 15000      # Thrust removed per metre above H_MAX

MAX_THRUST_STEP = 200  # Maximum thrust change per loop (every 10 ms)
LAND_DURATION = 2.0    # Time the flyer takes to land [s]
SENSOR_TIMEOUT = 0.5   # Land if a sensor sends no position for this long [s]

# Latest data from each drone, filled in by the log callbacks below
positions = {uri: None for uri in uris}
last_update = {uri: 0.0 for uri in uris}
flyer_thrust = [0]

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


# Saves every thrust reading, so we can average them while hovering
def thrust_callback(_timestamp, data, _logconf):
    flyer_thrust.append(data['stabilizer.thrust'])


# Asks the flyer to send how hard its motors are pushing, every 10 ms
def start_thrust_logging(cf):
    log_conf = LogConfig(name='Thrust', period_in_ms=10)
    log_conf.add_variable('stabilizer.thrust', 'float')
    cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(thrust_callback)
    log_conf.start()


# Straight-line distance between the two sensor drones (Pythagoras in 3D)
def sensor_distance():
    a = positions[Sensor_A]
    b = positions[Sensor_B]
    return math.sqrt(pow(a[0]-b[0], 2) + pow(a[1]-b[1], 2) + pow(a[2]-b[2], 2))


def distance_to_thrust(dist, hover_thrust):
    '''
    Linearly maps the distance between the sensor drones onto the
    flyer's thrust: D_MIN -> hover - THRUST_BELOW and
    D_MAX -> hover + THRUST_ABOVE. Distances outside [D_MIN, D_MAX]
    are clamped to the end thrusts.
    '''
    dist = min(max(dist, D_MIN), D_MAX)  # Keep the distance inside the range
    thrust_min = hover_thrust - THRUST_BELOW
    thrust_max = hover_thrust + THRUST_ABOVE
    # How far along the distance range we are (0 to 1), scaled onto the thrust range
    return thrust_min + (dist - D_MIN) / (D_MAX - D_MIN) * (thrust_max - thrust_min)


def apply_ceiling(thrust, height, hover_thrust):
    '''Above H_MAX, caps the thrust below hover so the flyer sinks back down.'''
    if height > H_MAX:
        # The higher above the ceiling, the less thrust it is allowed
        thrust = min(thrust, hover_thrust - K_CEILING * (height - H_MAX))
    return min(max(thrust, 0), 65535)  # Thrust must be between 0 and 65535


def take_off_and_measure_hover(cf):
    '''
    Takes off with the high level commander, hovers, and returns the
    average thrust the flyer needed to hold its height.
    '''
    cf.high_level_commander.takeoff(TAKEOFF_HEIGHT, 2.0)
    time.sleep(3.0)  # Take off and settle

    # Average all thrust readings that arrive in the next 2 s
    start = len(flyer_thrust)
    time.sleep(2.0)  # Measure
    samples = flyer_thrust[start:]
    hover_thrust = sum(samples) / len(samples)
    print(f'Hover thrust: {hover_thrust:.0f}')
    return hover_thrust


def dropped_sensor():
    '''Returns the URI of a sensor that has gone quiet, or None.'''
    now = time.time()
    for uri in (Sensor_A, Sensor_B):
        if now - last_update[uri] > SENSOR_TIMEOUT:
            return uri
    return None


def land(cf):
    # Hand control back to the high level commander, which lands
    # using the flyer's own position estimate
    cf.commander.send_notify_setpoint_stop()
    cf.high_level_commander.land(0.0, LAND_DURATION)
    time.sleep(LAND_DURATION + 0.5)
    cf.high_level_commander.stop()


def strain_sim(scf):
    # This runs on every drone at once, but only the flyer has work to do
    if scf.cf.link_uri != Flyer:
        return
    cf = scf.cf

    # Don't start until every drone has sent at least one position
    print('Waiting for position data from all drones...')
    while None in positions.values():
        if stop_event.is_set():
            return
        time.sleep(0.1)

    start_thrust_logging(cf)

    # A zero setpoint unlocks the thrust for later. Then hand control to
    # the high level commander for take off.
    cf.commander.send_setpoint(0, 0, 0, 0)
    cf.commander.send_notify_setpoint_stop()

    hover_thrust = take_off_and_measure_hover(cf)
    if stop_event.is_set():
        land(cf)
        return

    # Hand over to the sensors, starting from hover
    print('Handing over to the sensors')
    thrust = hover_thrust

    # Fly until Ctrl+C or a sensor drops out
    while not stop_event.is_set():
        dropped = dropped_sensor()
        if dropped is not None:
            print(f'\nLost sensor {dropped}, landing...')
            break

        # Work out the thrust the sensors are asking for, then the ceiling
        dist = sensor_distance()
        target = distance_to_thrust(dist, hover_thrust)
        target = apply_ceiling(target, positions[Flyer][2], hover_thrust)
        # Move towards that thrust in small steps, so it never jumps suddenly
        thrust += min(max(target - thrust, -MAX_THRUST_STEP), MAX_THRUST_STEP)

        # Stay level (roll 0, pitch 0), don't spin (yaw rate 0), push up this hard
        cf.commander.send_setpoint(0, 0, 0, int(thrust))
        print(f'distance: {dist:.2f} m  thrust: {int(thrust):5d}  height: {positions[Flyer][2]:.2f} m', end='\r')
        time.sleep(0.01)

    print()
    land(cf)


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
