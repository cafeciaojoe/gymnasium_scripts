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

# The drone switches between two modes:
#   HOLD:   its target is pinned to one spot, so it pulls back there when bumped.
#   FOLLOW: its target is pinned to wherever the drone is right now, so it
#           doesn't pull anywhere and you can carry it to a new spot.
# A push further than PUSH_DISTANCE switches HOLD -> FOLLOW. Holding it
# still for STILL_TIME switches FOLLOW -> HOLD at the new spot.

# ---- Inputs ----
TAKEOFF_HEIGHT = 0.5  # [m]
PUSH_DISTANCE = 0.15  # How far it must be pushed from its spot to let go [m]
STILL_SPEED = 0.05    # Below this speed the drone counts as still [m/s]
STILL_TIME = 0.5      # How long it must be still to lock the new spot [s]
Z_MIN = 0.2           # Never hold or follow lower than this [m]

LAND_DURATION = 2.0     # Time the drone takes to land [s]
POSITION_TIMEOUT = 0.5  # Land if no position arrives for this long [s]

position = [None]  # (x, y, z)
velocity = [None]  # (vx, vy, vz)
last_update = [0.0]

# Set by Ctrl+C so the drone lands instead of the script dying mid-air
stop_event = threading.Event()


def handle_ctrl_c(_signum, _frame):
    print('\nCtrl+C pressed, landing...')
    stop_event.set()


# Runs every time the drone sends its data (every 10 ms).
# Saves the numbers and notes the time, so we can tell if tracking drops out.
def state_callback(_timestamp, data, _logconf):
    position[0] = (data['stateEstimate.x'],
                   data['stateEstimate.y'],
                   data['stateEstimate.z'])
    velocity[0] = (data['stateEstimate.vx'],
                   data['stateEstimate.vy'],
                   data['stateEstimate.vz'])
    last_update[0] = time.time()


# Asks the drone to send its position and speed every 10 ms
def start_state_logging(cf):
    log_conf = LogConfig(name='State', period_in_ms=10)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    log_conf.add_variable('stateEstimate.vx', 'float')
    log_conf.add_variable('stateEstimate.vy', 'float')
    log_conf.add_variable('stateEstimate.vz', 'float')
    cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(state_callback)
    log_conf.start()


# Straight-line distance between two points (Pythagoras in 3D)
def distance(a, b):
    return math.sqrt(pow(a[0]-b[0], 2) + pow(a[1]-b[1], 2) + pow(a[2]-b[2], 2))


# How fast the drone is moving, in any direction
def speed(v):
    return math.sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2))


# Same point, but never lower than Z_MIN
def above_floor(p):
    return (p[0], p[1], max(p[2], Z_MIN))


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
    while position[0] is None:
        if stop_event.is_set():
            return
        time.sleep(0.1)

    # Take off with the built-in auto-pilot and let it settle
    cf.high_level_commander.takeoff(TAKEOFF_HEIGHT, 2.0)
    time.sleep(3.0)

    # Start in HOLD, pinned to where it is now
    target = above_floor(position[0])
    mode = 'HOLD'
    still_since = None

    # Fly until Ctrl+C or the position tracking drops out
    while not stop_event.is_set():
        if time.time() - last_update[0] > POSITION_TIMEOUT:
            print('\nLost position, landing...')
            break

        pos = position[0]

        if mode == 'HOLD':
            # Target stays put. Pushed far enough? Let go.
            if distance(pos, target) > PUSH_DISTANCE:
                mode = 'FOLLOW'
                still_since = None

        else:  # FOLLOW
            # Target moves with the drone, so it doesn't pull anywhere
            target = above_floor(pos)
            # Count how long it has been still. Still long enough? Pin it here.
            if speed(velocity[0]) < STILL_SPEED:
                if still_since is None:
                    still_since = time.time()
                elif time.time() - still_since > STILL_TIME:
                    mode = 'HOLD'
            else:
                still_since = None

        # Fly to the target, facing yaw 0
        cf.commander.send_position_setpoint(target[0], target[1], target[2], 0)
        print(f'{mode:6}  target: ({target[0]:5.2f}, {target[1]:5.2f}, {target[2]:5.2f})', end='\r')
        time.sleep(0.01)

    print()
    land(cf)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Connect. The cache saves the drone's settings list so the next connect is faster.
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.platform.send_arming_request(True)  # Allow the motors to spin
        time.sleep(1.0)

        start_state_logging(cf)
        time.sleep(0.5)

        # From here on Ctrl+C lands the drone instead of killing the script
        signal.signal(signal.SIGINT, handle_ctrl_c)
        pick_and_place(cf)
