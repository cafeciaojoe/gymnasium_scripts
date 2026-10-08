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
Flyer = 'radio://0/40/2M/BADF00D005'   # The drone that flies the circle

# All drones the script connects to
uris = {
    Sensor,
    Flyer,
}

# The flyer takes off and flies laps of a circle, like circle_reps.
# Tilting the sensor slides the whole circle in that direction, like a
# joystick: the more you tilt, the faster it slides. Hold the sensor level
# and the circle stays put. "Forward" is the way the sensor is pointing.
# Needs the Lighthouse (or another positioning system).

# ---- Inputs ----
HEIGHT = 0.8       # Flying height [m]
RADIUS = 0.25       # Circle radius [m]
LAP_TIME = 2.5     # Time for one lap [s]. Shorter = faster.
REPS = 300         # Number of laps
CLOCKWISE = False  # Direction, seen from above

DEAD_ZONE = 3       # Sensor tilts smaller than this are ignored [deg]
SHIFT_SPEED = 0.02  # How fast the circle slides per degree of tilt [m/s per deg]
MAX_SHIFT = 1.0     # Furthest the circle centre can slide from where it started [m]

# Vibration: the flyer's motor 1 power is copied to the vibration motor
# on the sensor's m1, so you can feel the flyer working. Settings from
# Vibrate_to_Acceleration/vibe_to_acceleration.py.
VIBRATE = True
# The flyer's m1 only changes a little around hover, so only a small window
# around its hover value is used, stretched over the whole vibration range:
#   hover - M1_SPAN  ->  MIN_VIBE_POWER
#   hover            ->  halfway
#   hover + M1_SPAN  ->  MAX_VIBE_POWER
# Smaller M1_SPAN = small motor changes feel bigger.
MIN_VIBE_POWER = 1000 # Weakest vibration you can still feel. Used as soon as the flyer's m1 is running.
MAX_VIBE_POWER = 40000  # Strongest vibration. Advised no higher than 50000.
M1_SPAN = 5000          # Flyer m1 change (either side of hover) that covers the whole vibration range
M1_MEASURE_TIME = 1.0   # How long the flyer hovers to measure its hover m1, before the laps [s]
VIBE_PERIOD = 0.1      # How often the vibration is updated [s]

LAND_DURATION = 2.0     # Time the flyer takes to land [s]
POSITION_TIMEOUT = 3  # Land if a drone sends nothing for this long [s]

# Latest data from each drone, filled in by the log callback below
positions = {uri: None for uri in uris}  # Flyer: (x, y, z) [m]
flyer_m1 = [0]                           # Flyer: motor 1 power, 0 to 65535
m1_hover = [None]                        # Flyer's m1 at hover, measured before the laps
attitudes = {uri: None for uri in uris}  # Sensor: (roll, pitch, yaw) [deg]
last_update = {uri: 0.0 for uri in uris}

# Set by Ctrl+C so the flyer lands instead of the script dying mid-air
stop_event = threading.Event()

# Set once the flyer has landed, so the sensor stops vibrating
flight_done = threading.Event()


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
        flyer_m1[0] = data['motor.m1']
    else:
        attitudes[uri] = (data['stateEstimate.roll'],
                          data['stateEstimate.pitch'],
                          data['stateEstimate.yaw'])
    last_update[uri] = time.time()


# Asks a drone to send its data every 10 ms: position and motor 1 power
# for the flyer, tilt and heading for the sensor
def start_logging(scf):
    log_conf = LogConfig(name='State', period_in_ms=10)
    if scf.cf.link_uri == Flyer:
        log_conf.add_variable('stateEstimate.x', 'float')
        log_conf.add_variable('stateEstimate.y', 'float')
        log_conf.add_variable('stateEstimate.z', 'float')
        log_conf.add_variable('motor.m1', 'uint16_t')
    else:
        log_conf.add_variable('stateEstimate.roll', 'float')
        log_conf.add_variable('stateEstimate.pitch', 'float')
        log_conf.add_variable('stateEstimate.yaw', 'float')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(lambda _timestamp, data, _logconf: log_callback(scf.cf.link_uri, data))
    log_conf.start()


def wait(seconds):
    '''Sleeps, but returns False early if Ctrl+C is pressed.'''
    end = time.time() + seconds
    while time.time() < end:
        if stop_event.is_set():
            return False
        time.sleep(0.05)
    return True


def dropped_drone():
    '''Returns the URI of a drone that has gone quiet, or None.'''
    now = time.time()
    for uri in uris:
        if now - last_update[uri] > POSITION_TIMEOUT:
            return uri
    return None


# Treats small tilts as zero, so a slightly tilted hand doesn't slide the circle
def dead_zone(angle):
    if abs(angle) < DEAD_ZONE:
        return 0
    return angle - math.copysign(DEAD_ZONE, angle)


def sensor_slide():
    '''
    Speed the circle should slide at, in room directions (vx, vy) [m/s],
    from how far the sensor is tilted.
    '''
    roll, pitch, yaw = attitudes[Sensor]

    # Nose down (negative pitch) = forward, right side down (positive roll) = right
    forward = -SHIFT_SPEED * dead_zone(pitch)
    left = -SHIFT_SPEED * dead_zone(roll)

    # Turn the sensor's forward/left into room directions, using the way it points
    yaw_rad = math.radians(yaw)
    vx = forward * math.cos(yaw_rad) - left * math.sin(yaw_rad)
    vy = forward * math.sin(yaw_rad) + left * math.cos(yaw_rad)
    return vx, vy


def fly_circle(cf, start_centre):
    '''
    Flies REPS laps, sliding the centre with the sensor. Returns the final
    centre, or None if it had to stop early. Sends a new point on the circle
    every 10 ms, so the drone follows it round.
    '''
    direction = -1 if CLOCKWISE else 1
    centre = list(start_centre)
    start = time.time()
    last = start

    while not stop_event.is_set():
        dropped = dropped_drone()
        if dropped is not None:
            print(f'\nLost {dropped}, landing...')
            return None

        now = time.time()
        elapsed = now - start
        if elapsed > REPS * LAP_TIME:
            return centre

        # Slide the centre by (speed x time since last loop), but not too far from the start
        vx, vy = sensor_slide()
        dt = now - last
        last = now
        for i, v in ((0, vx), (1, vy)):
            centre[i] += v * dt
            centre[i] = min(max(centre[i], start_centre[i] - MAX_SHIFT), start_centre[i] + MAX_SHIFT)

        # How far round the circle we should be by now
        angle = direction * 2 * math.pi * elapsed / LAP_TIME
        x = centre[0] + RADIUS * math.cos(angle)
        y = centre[1] + RADIUS * math.sin(angle)

        cf.commander.send_position_setpoint(x, y, HEIGHT, 0)
        print(f'lap {int(elapsed // LAP_TIME) + 1}/{REPS}  centre: ({centre[0]:5.2f}, {centre[1]:5.2f})', end='\r')
        time.sleep(0.01)

    return None


def land(cf):
    # Hand control back to the high level commander, which lands
    # using the drone's own position estimate
    cf.commander.send_notify_setpoint_stop()
    cf.high_level_commander.land(0.0, LAND_DURATION)
    time.sleep(LAND_DURATION + 0.5)
    cf.high_level_commander.stop()


def vibe_power():
    '''Vibration power for the sensor, from the flyer's m1 (see M1_SPAN).'''
    if flyer_m1[0] == 0:
        return 0               # Flyer's motor is off
    if m1_hover[0] is None:
        return MIN_VIBE_POWER  # Taking off, hover not measured yet

    # Where m1 sits in the window around hover: 0 at the bottom, 1 at the top
    low = m1_hover[0] - M1_SPAN
    fraction = (flyer_m1[0] - low) / (2 * M1_SPAN)
    fraction = min(max(fraction, 0), 1)
    return int(MIN_VIBE_POWER + fraction * (MAX_VIBE_POWER - MIN_VIBE_POWER))


def measure_hover_m1():
    '''Averages the flyer's m1 over M1_MEASURE_TIME while it hovers.'''
    samples = []
    end = time.time() + M1_MEASURE_TIME
    while time.time() < end and not stop_event.is_set():
        samples.append(flyer_m1[0])
        time.sleep(0.01)
    if samples:
        m1_hover[0] = sum(samples) / len(samples)
        print(f'Hover m1: {m1_hover[0]:.0f}')


def vibrate(cf):
    '''
    Runs on the sensor: copies the flyer's motor 1 power onto the sensor's
    m1 vibration motor until the flyer has landed.
    '''
    cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(1)

    while not flight_done.is_set():
        cf.param.set_value('motorPowerSet.m1', str(vibe_power()))
        time.sleep(VIBE_PERIOD)

    # Turn off all motors
    cf.param.set_value('motorPowerSet.m1', '0')
    cf.param.set_value('motorPowerSet.m2', '0')
    cf.param.set_value('motorPowerSet.m3', '0')
    cf.param.set_value('motorPowerSet.m4', '0')
    time.sleep(0.5)
    cf.param.set_value('motorPowerSet.enable', '0')
    time.sleep(0.5)


def circle_reps(scf):
    # This runs on every drone at once. The sensor vibrates, the flyer flies.
    if scf.cf.link_uri != Flyer:
        if VIBRATE:
            vibrate(scf.cf)
        return

    try:
        fly(scf.cf)
    finally:
        # Even if something goes wrong, tell the sensor to stop vibrating
        flight_done.set()


def fly(cf):
    # Don't start until both drones have sent data
    print('Waiting for data from both drones...')
    while positions[Flyer] is None or attitudes[Sensor] is None:
        if stop_event.is_set():
            return
        time.sleep(0.1)

    # The circle starts around the spot the flyer takes off from
    centre = positions[Flyer]
    hl = cf.high_level_commander

    # Take off, then fly out to the start of the circle (angle 0 = +x side)
    hl.takeoff(HEIGHT, 2.0)
    if wait(2.5):
        hl.go_to(centre[0] + RADIUS, centre[1], HEIGHT, 0, 2.0)
        if wait(2.5):
            measure_hover_m1()  # Hovering at the circle start, so m1 is at hover
            print(f'Flying {REPS} laps, tilt the sensor to slide the circle')
            end_centre = fly_circle(cf, centre)
            if end_centre is not None:
                # Back to the middle before landing. The high level
                # commander needs to be told it is in charge again first.
                cf.commander.send_notify_setpoint_stop()
                hl.go_to(end_centre[0], end_centre[1], HEIGHT, 0, 2.0)
                wait(2.5)

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
        swarm.parallel_safe(circle_reps)  # Runs circle_reps on every drone at once
        time.sleep(0.5)

        swarm.close_links()
