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
Sensor = 'radio://0/40/2M/BADF00D004'  # Hand-held sensor drone (does not fly)
Flyer = 'radio://0/40/2M/BADF00D009'   # The drone that flies

# All drones the script connects to
uris = {
    Sensor,
    Flyer,
}

# The flyer takes off and holds HEIGHT by itself. The sensor's roll, pitch
# and spin steer it. Two ways to steer, set with CONTROL:
#   'speed': sensor tilt sets the flyer's speed. Level the sensor and it
#            stops. Much easier to control.
#   'angle': the flyer copies the sensor's tilt. Tilt means it keeps
#            speeding up that way, so you must tilt back to stop it,
#            like balancing a ball on a tray.
# Directions are from the flyer's point of view: tilt the sensor forward
# and the flyer goes the way its own nose points.
# If the flyer leaves the BOUNDARY around where it took off, it lands.
# Needs the Lighthouse (or another positioning system).

# ---- Inputs ----
CONTROL = 'speed'  # 'speed' or 'angle'
HEIGHT = 0.8       # Flying height [m]

# 'speed' mode
DEAD_ZONE = 0          # Sensor tilts smaller than this are ignored [deg]
SPEED_PER_DEG = 0.02   # Flyer speed per degree of sensor tilt [m/s per deg]
MAX_SPEED = 0.5        # [m/s]

# 'angle' mode
SENSITIVITY = 0.5  # Flyer tilt = sensor tilt x SENSITIVITY
MAX_ANGLE = 10     # [deg]

# Spin, both modes: flyer spin rate = sensor spin rate x YAW_SENSITIVITY
YAW_SENSITIVITY = 1.0
MAX_YAW_RATE = 90  # [deg/s]

BOUNDARY = 1.5          # Land if the flyer gets further than this from its take off spot [m]
LAND_DURATION = 2.0     # Time the flyer takes to land [s]
POSITION_TIMEOUT = 0.5  # Land if a drone sends nothing for this long [s]

# Latest data from each drone, filled in by the log callback below
positions = {uri: None for uri in uris}  # Flyer: (x, y, z) [m]
attitudes = {uri: None for uri in uris}  # Sensor: (roll, pitch, yaw rate) [deg, deg/s]
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
def log_callback(uri, data):
    if uri == Flyer:
        positions[uri] = (data['stateEstimate.x'],
                          data['stateEstimate.y'],
                          data['stateEstimate.z'])
    else:
        attitudes[uri] = (data['stateEstimate.roll'],
                          data['stateEstimate.pitch'],
                          data['gyro.z'])
    last_update[uri] = time.time()


# Asks a drone to send its data every 10 ms: position for the flyer,
# tilt and spin rate for the sensor
def start_logging(scf):
    log_conf = LogConfig(name='State', period_in_ms=10)
    if scf.cf.link_uri == Flyer:
        log_conf.add_variable('stateEstimate.x', 'float')
        log_conf.add_variable('stateEstimate.y', 'float')
        log_conf.add_variable('stateEstimate.z', 'float')
    else:
        log_conf.add_variable('stateEstimate.roll', 'float')
        log_conf.add_variable('stateEstimate.pitch', 'float')
        log_conf.add_variable('gyro.z', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda _timestamp, data, _logconf: log_callback(scf.cf.link_uri, data))
    log_conf.start()


def dropped_drone():
    '''Returns the URI of a drone that has gone quiet, or None.'''
    now = time.time()
    for uri in uris:
        if now - last_update[uri] > POSITION_TIMEOUT:
            return uri
    return None


# Keeps a value between -limit and +limit
def clamp(value, limit):
    return min(max(value, -limit), limit)


# Treats small tilts as zero, so a slightly tilted hand doesn't move the flyer
def dead_zone(angle):
    if abs(angle) < DEAD_ZONE:
        return 0
    return angle - math.copysign(DEAD_ZONE, angle)


def land(cf):
    # Hand control back to the high level commander, which lands
    # using the drone's own position estimate
    cf.commander.send_notify_setpoint_stop()
    cf.high_level_commander.land(0.0, LAND_DURATION)
    time.sleep(LAND_DURATION + 0.5)
    cf.high_level_commander.stop()


def tilt_flight(scf):
    # This runs on every drone at once, but only the flyer has work to do
    if scf.cf.link_uri != Flyer:
        return
    cf = scf.cf

    # Don't start until both drones have sent data
    print('Waiting for data from both drones...')
    while positions[Flyer] is None or attitudes[Sensor] is None:
        if stop_event.is_set():
            return
        time.sleep(0.1)

    home = positions[Flyer]

    # Take off with the built-in auto-pilot and let it settle
    cf.high_level_commander.takeoff(HEIGHT, 2.0)
    end = time.time() + 2.5
    while time.time() < end and not stop_event.is_set():
        time.sleep(0.05)

    print(f'Steering in {CONTROL} mode, tilt the sensor')

    # Fly until Ctrl+C, a drone drops out, or the flyer leaves the boundary
    while not stop_event.is_set():
        dropped = dropped_drone()
        if dropped is not None:
            print(f'\nLost {dropped}, landing...')
            break

        x, y, _ = positions[Flyer]
        if math.hypot(x - home[0], y - home[1]) > BOUNDARY:
            print('\nLeft the boundary, landing...')
            break

        roll, pitch, yaw_rate = attitudes[Sensor]
        spin = clamp(YAW_SENSITIVITY * yaw_rate, MAX_YAW_RATE)

        if CONTROL == 'speed':
            # Nose down (negative pitch) = forward, right side down (positive roll) = right.
            # vx is forward and vy is left, from the flyer's point of view.
            vx = clamp(-SPEED_PER_DEG * dead_zone(pitch), MAX_SPEED)
            vy = clamp(-SPEED_PER_DEG * dead_zone(roll), MAX_SPEED)
            cf.commander.send_hover_setpoint(vx, vy, spin, HEIGHT)
            print(f'forward: {vx:5.2f} m/s  left: {vy:5.2f} m/s  spin: {spin:6.1f}', end='\r')
        else:
            # The firmware holds the height; roll and pitch go straight through
            flyer_roll = clamp(SENSITIVITY * roll, MAX_ANGLE)
            flyer_pitch = clamp(SENSITIVITY * pitch, MAX_ANGLE)
            cf.commander.send_zdistance_setpoint(flyer_roll, flyer_pitch, spin, HEIGHT)
            print(f'roll: {flyer_roll:5.1f}  pitch: {flyer_pitch:5.1f}  spin: {spin:6.1f}', end='\r')

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

        swarm.parallel_safe(start_logging)
        time.sleep(0.5)

        # From here on Ctrl+C lands the flyer instead of killing the script
        signal.signal(signal.SIGINT, handle_ctrl_c)
        swarm.parallel_safe(tilt_flight)  # Runs tilt_flight on every drone at once
        time.sleep(0.5)

        swarm.close_links()
