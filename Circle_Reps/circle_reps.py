import math
import signal
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/40/2M/BADF00D005')  # Change to your Crazyflie's URI

# The drone takes off, flies out to the edge of a circle around where it
# took off, flies REPS laps, comes back to the middle and lands.
# Needs the Lighthouse (or another positioning system).

# ---- Inputs ----
HEIGHT = 0.8      # Flying height [m]
RADIUS = 0.5      # Circle radius [m]
LAP_TIME = 5.0    # Time for one lap [s]. Shorter = faster.
REPS = 300          # Number of laps
CLOCKWISE = False  # Direction, seen from above

LAND_DURATION = 2.0     # Time the drone takes to land [s]
POSITION_TIMEOUT = 3  # Land if no position arrives for this long [s]

# Latest data from the drone, filled in by the log callback below
position = [None]  # (x, y, z) [m]
last_update = [0.0]

# Set by Ctrl+C so the drone lands instead of the script dying mid-air
stop_event = threading.Event()


def handle_ctrl_c(_signum, _frame):
    print('\nCtrl+C pressed, landing...')
    stop_event.set()


# Runs every time the drone sends its position (every 10 ms).
# Notes the time, so we can tell if tracking drops out.
def position_callback(_timestamp, data, _logconf):
    position[0] = (data['stateEstimate.x'],
                   data['stateEstimate.y'],
                   data['stateEstimate.z'])
    last_update[0] = time.time()


# Asks the drone to send its position every 10 ms
def start_position_logging(cf):
    log_conf = LogConfig(name='Position', period_in_ms=10)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(position_callback)
    log_conf.start()


def wait(seconds):
    '''Sleeps, but returns False early if Ctrl+C is pressed.'''
    end = time.time() + seconds
    while time.time() < end:
        if stop_event.is_set():
            return False
        time.sleep(0.05)
    return True


def fly_circle(cf, centre):
    '''
    Flies REPS laps around `centre`. Returns False if it had to stop early.
    Sends a new point on the circle every 10 ms, so the drone follows it round.
    '''
    direction = -1 if CLOCKWISE else 1
    start = time.time()

    while not stop_event.is_set():
        if time.time() - last_update[0] > POSITION_TIMEOUT:
            print('\nLost position, landing...')
            return False

        elapsed = time.time() - start
        if elapsed > REPS * LAP_TIME:
            return True

        # How far round the circle we should be by now
        angle = direction * 2 * math.pi * elapsed / LAP_TIME
        x = centre[0] + RADIUS * math.cos(angle)
        y = centre[1] + RADIUS * math.sin(angle)

        cf.commander.send_position_setpoint(x, y, HEIGHT, 0)
        print(f'lap {int(elapsed // LAP_TIME) + 1}/{REPS}', end='\r')
        time.sleep(0.01)

    return False


def land(cf):
    # Hand control back to the high level commander, which lands
    # using the drone's own position estimate
    cf.commander.send_notify_setpoint_stop()
    cf.high_level_commander.land(0.0, LAND_DURATION)
    time.sleep(LAND_DURATION + 0.5)
    cf.high_level_commander.stop()


def circle_reps(cf):
    # Don't start until the drone has sent at least one position
    print('Waiting for position data...')
    while position[0] is None:
        if stop_event.is_set():
            return
        time.sleep(0.1)

    # The circle goes around the spot it takes off from
    centre = position[0]
    hl = cf.high_level_commander

    # Take off, then fly out to the start of the circle (angle 0 = +x side)
    hl.takeoff(HEIGHT, 2.0)
    if wait(2.5):
        hl.go_to(centre[0] + RADIUS, centre[1], HEIGHT, 0, 2.0)
        if wait(2.5):
            print(f'Flying {REPS} laps')
            if fly_circle(cf, centre):
                # Back to the middle before landing. The high level
                # commander needs to be told it is in charge again first.
                cf.commander.send_notify_setpoint_stop()
                hl.go_to(centre[0], centre[1], HEIGHT, 0, 2.0)
                wait(2.5)

    print()
    land(cf)


if __name__ == '__main__':
    cflib.crtp.init_drivers()  # Start the radio

    # Connect. The cache saves the drone's settings list so the next connect is faster.
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.platform.send_arming_request(True)  # Allow the motors to spin
        time.sleep(1.0)

        start_position_logging(cf)
        time.sleep(0.5)

        # From here on Ctrl+C lands the drone instead of killing the script
        signal.signal(signal.SIGINT, handle_ctrl_c)
        circle_reps(cf)
