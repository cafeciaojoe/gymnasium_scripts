import math
import signal
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/40/2M/BADF00D007')  # Change to your Crazyflie's URI

# The drone takes off, measures the thrust it needs to hover, then floats
# on that thrust. Nothing pulls it back to any spot or height. Instead it
# brakes against its own movement (measured by the Lighthouse), so a drift
# slows to a stop. SLIPPERINESS sets how much it brakes.
# When the deck senses a hand nearby, the brakes are released so you can
# move it freely. Take your hand away and it brakes to a stop wherever it is.
# The Lighthouse is needed for all of this, plus take off and landing.

# ---- Inputs ----
TAKEOFF_HEIGHT = 0.5  # [m]
SETTLE_TIME = 3.0     # Wait after take off before measuring hover thrust [s]
MEASURE_TIME = 2.0    # How long hover thrust is averaged over [s]. Longer = steadier value.

# Slipperiness, from 0 to 1:
#   0 = brakes hard, stops almost straight away
#   1 = no brakes at all, slides like on ice
SLIPPERINESS_XY = 0.5  # Sideways
SLIPPERINESS_Z = 0.5   # Up and down

# Strongest brakes, used when slipperiness is 0
MAX_BRAKE_XY = 20     # Tilt against sideways speed [deg per m/s]
MAX_BRAKE_Z = 10000   # Thrust against up/down speed [thrust per m/s]
MAX_TILT = 8          # Most it will ever tilt to brake [deg]
MAX_TRIM = 8000       # Most thrust it will ever add or remove to brake

# Which deck senses your hand: 'multiranger' or 'flow'
#   multiranger: anything closer than HAND_DISTANCE on the front, back,
#                left or right sensor counts as a hand. The up sensor is
#                ignored, because the Lighthouse deck sits on top of it.
#   flow:        the flow deck only looks down, so a hand underneath counts
#                when it reads HAND_MARGIN closer than the floor should be.
HAND_SENSOR = 'multiranger'
HAND_DISTANCE = 0.3      # [m]
HAND_MARGIN = 0.15       # [m]
HAND_RELEASE_TIME = 0.3  # Keep the brakes off this long after the hand leaves [s]

# True: you can twist the drone to a new heading and it stays there.
# False: it twists back to the heading it took off with.
FREE_YAW = False
MAX_YAW_RATE = 200  # Fastest spin it will follow [deg/s]

LAND_DURATION = 2.0     # Time the drone takes to land [s]
POSITION_TIMEOUT = 0.5  # Land if no position arrives for this long [s]

# Latest data from the drone, filled in by the log callbacks below
height = [None]               # [m]
yaw = [0.0]                   # Heading [deg]
velocity = [(0.0, 0.0, 0.0)]  # (vx, vy, vz) [m/s]
yaw_rate = [0.0]              # Spin rate [deg/s]
ranges = {}                   # Distance sensor readings [m], None = nothing in range
last_update = [0.0]
drone_thrust = [0]

# Set by Ctrl+C so the drone lands instead of the script dying mid-air
stop_event = threading.Event()


def handle_ctrl_c(_signum, _frame):
    print('\nCtrl+C pressed, landing...')
    stop_event.set()


# Runs every time the drone sends its data (every 10 ms).
# Notes the time, so we can tell if tracking drops out.
def state_callback(_timestamp, data, _logconf):
    height[0] = data['stateEstimate.z']
    yaw[0] = data['stateEstimate.yaw']
    velocity[0] = (data['stateEstimate.vx'],
                   data['stateEstimate.vy'],
                   data['stateEstimate.vz'])
    yaw_rate[0] = data['gyro.z']
    last_update[0] = time.time()


# Saves every thrust reading, so we can average them while hovering
def thrust_callback(_timestamp, data, _logconf):
    drone_thrust.append(data['stabilizer.thrust'])


# Saves the distance sensor readings. They arrive in mm, and 8000 or
# more means nothing is in range.
def range_callback(_timestamp, data, _logconf):
    for name, mm in data.items():
        ranges[name] = None if mm >= 8000 else mm / 1000.0


# Asks the drone to send its state, thrust and distance sensors every 10 ms
def start_logging(cf):
    log_conf1 = LogConfig(name='State', period_in_ms=10)
    log_conf1.add_variable('stateEstimate.z', 'float')
    log_conf1.add_variable('stateEstimate.yaw', 'float')
    log_conf1.add_variable('stateEstimate.vx', 'float')
    log_conf1.add_variable('stateEstimate.vy', 'float')
    log_conf1.add_variable('stateEstimate.vz', 'float')
    log_conf1.add_variable('gyro.z', 'float')
    cf.log.add_config(log_conf1)
    log_conf1.data_received_cb.add_callback(state_callback)
    log_conf1.start()

    log_conf2 = LogConfig(name='Thrust', period_in_ms=10)
    log_conf2.add_variable('stabilizer.thrust', 'float')
    cf.log.add_config(log_conf2)
    log_conf2.data_received_cb.add_callback(thrust_callback)
    log_conf2.start()

    # Only ask for the sensors the chosen deck has
    log_conf3 = LogConfig(name='Range', period_in_ms=10)
    if HAND_SENSOR == 'multiranger':
        # Not 'range.up': the Lighthouse deck on top would always count as a hand
        for name in ('range.front', 'range.back', 'range.left', 'range.right'):
            log_conf3.add_variable(name, 'uint16_t')
    else:
        log_conf3.add_variable('range.zrange', 'uint16_t')
    cf.log.add_config(log_conf3)
    log_conf3.data_received_cb.add_callback(range_callback)
    log_conf3.start()


def take_off_and_measure_hover(cf):
    '''
    Takes off with the high level commander, hovers, and returns the
    average thrust the drone needed to hold its height.
    '''
    cf.high_level_commander.takeoff(TAKEOFF_HEIGHT, 2.0)
    time.sleep(SETTLE_TIME)  # Take off and settle

    # Average all thrust readings that arrive in the next MEASURE_TIME
    start = len(drone_thrust)
    time.sleep(MEASURE_TIME)
    samples = drone_thrust[start:]
    hover_thrust = sum(samples) / len(samples)
    print(f'Hover thrust: {hover_thrust:.0f}')
    return hover_thrust


def hand_nearby():
    '''True if the chosen deck can see a hand close to the drone.'''
    if HAND_SENSOR == 'multiranger':
        return any(r is not None and r < HAND_DISTANCE for r in ranges.values())

    # Flow deck: normally it sees the floor, about `height` away.
    # Something clearly closer than that must be a hand underneath.
    down = ranges.get('range.zrange')
    return down is not None and down < height[0] - HAND_MARGIN


# Keeps a value between -limit and +limit
def clamp(value, limit):
    return min(max(value, -limit), limit)


def brake_tilt():
    '''
    Roll and pitch that lean against the drone's sideways speed, so it
    slows down. Returns (roll, pitch) in degrees for send_setpoint.
    '''
    brake = MAX_BRAKE_XY * (1 - SLIPPERINESS_XY)
    vx, vy, _ = velocity[0]

    # Which way to lean, in room directions (x, y): against the speed
    lean_x = -brake * vx
    lean_y = -brake * vy

    # Turn room directions into the drone's own forward/left, since it may have yawed
    yaw_rad = math.radians(yaw[0])
    forward = lean_x * math.cos(yaw_rad) + lean_y * math.sin(yaw_rad)
    left = -lean_x * math.sin(yaw_rad) + lean_y * math.cos(yaw_rad)

    # With send_setpoint, +pitch tips the nose down (moves forward)
    # and -roll tips to the left (moves left)
    pitch = clamp(forward, MAX_TILT)
    roll = clamp(-left, MAX_TILT)
    return roll, pitch


def brake_thrust():
    '''Extra thrust against the drone's up/down speed, so it slows down.'''
    # Rising -> less thrust. Sinking -> more thrust.
    brake = MAX_BRAKE_Z * (1 - SLIPPERINESS_Z)
    return clamp(-brake * velocity[0][2], MAX_TRIM)


def land(cf):
    # Hand control back to the high level commander, which lands
    # using the drone's own position estimate
    cf.commander.send_notify_setpoint_stop()
    cf.high_level_commander.land(0.0, LAND_DURATION)
    time.sleep(LAND_DURATION + 0.5)
    cf.high_level_commander.stop()


def pick_and_place(cf):
    # Don't start until the drone has sent at least one position
    print('Waiting for position data...')
    while height[0] is None:
        if stop_event.is_set():
            return
        time.sleep(0.1)

    # A zero setpoint unlocks the thrust for later. Then hand control to
    # the high level commander for take off.
    cf.commander.send_setpoint(0, 0, 0, 0)
    cf.commander.send_notify_setpoint_stop()

    hover_thrust = take_off_and_measure_hover(cf)
    if stop_event.is_set():
        land(cf)
        return

    print('Floating, push me around')
    last_hand_time = 0.0

    # Fly until Ctrl+C or the position tracking drops out
    while not stop_event.is_set():
        if time.time() - last_update[0] > POSITION_TIMEOUT:
            print('\nLost position, landing...')
            break

        # Brakes stay off while a hand is near, and for a moment after
        if hand_nearby():
            last_hand_time = time.time()
        hand = time.time() - last_hand_time < HAND_RELEASE_TIME

        thrust = hover_thrust  # Start from the measured hover thrust
        roll, pitch = 0, 0     # Level
        if not hand:
            thrust += brake_thrust()
            roll, pitch = brake_tilt()

        # Asking for the spin it already has means its target heading
        # turns with it, so it doesn't twist back. Minus because the
        # firmware flips the yaw rate sent with send_setpoint.
        yaw_cmd = 0
        if FREE_YAW:
            yaw_cmd = -clamp(yaw_rate[0], MAX_YAW_RATE)

        cf.commander.send_setpoint(roll, pitch, yaw_cmd, int(min(max(thrust, 0), 65535)))
        status = 'HAND  ' if hand else 'BRAKES'
        print(f'{status}  thrust: {int(thrust):5d}  height: {height[0]:.2f} m', end='\r')
        time.sleep(0.01)

    print()
    land(cf)


if __name__ == '__main__':
    cflib.crtp.init_drivers()  # Start the radio

    # Connect. The cache saves the drone's settings list so the next connect is faster.
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.platform.send_arming_request(True)  # Allow the motors to spin
        time.sleep(1.0)

        start_logging(cf)
        time.sleep(0.5)

        # From here on Ctrl+C lands the drone instead of killing the script
        signal.signal(signal.SIGINT, handle_ctrl_c)
        pick_and_place(cf)
