import signal
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/40/2M/BADF00D007')  # Change to your Crazyflie's URI

# The drone takes off, measures the thrust it needs to hover, then just
# stays level at that thrust. Nothing holds it in place sideways, so a
# push moves it and it stays wherever it drifts to.
# The Lighthouse is still needed for take off and landing.

# ---- Inputs ----
TAKEOFF_HEIGHT = 0.5  # [m]
SETTLE_TIME = 3.0     # Wait after take off before measuring hover thrust [s]
MEASURE_TIME = 2.0    # How long hover thrust is averaged over [s]. Longer = steadier value.

# True: the Lighthouse trims the thrust to hold TAKEOFF_HEIGHT, so the
#       drone doesn't climb or sink as the battery drains. Sideways it
#       is still free.
# False: pure hover thrust, nothing holds the height either.
HOLD_HEIGHT = True
K_Z = 10000   # Thrust added per metre below the hold height
K_VZ = 5000   # Thrust removed per m/s of climb (stops it bouncing)
MAX_TRIM = 8000  # Maximum thrust the height hold can add or remove

# True: you can twist the drone to a new heading and it stays there.
# False: it twists back to the heading it took off with.
FREE_YAW = False
MAX_YAW_RATE = 200  # Fastest spin it will follow [deg/s]

LAND_DURATION = 2.0     # Time the drone takes to land [s]
POSITION_TIMEOUT = 0.5  # Land if no position arrives for this long [s]

height = [None]
climb_rate = [0.0]
yaw_rate = [0.0]
last_update = [0.0]
drone_thrust = [0]

# Set by Ctrl+C so the drone lands instead of the script dying mid-air
stop_event = threading.Event()


def handle_ctrl_c(_signum, _frame):
    print('\nCtrl+C pressed, landing...')
    stop_event.set()


# Runs every time the drone sends its data (every 10 ms).
# Saves the numbers and notes the time, so we can tell if tracking drops out.
def state_callback(_timestamp, data, _logconf):
    height[0] = data['stateEstimate.z']
    climb_rate[0] = data['stateEstimate.vz']
    yaw_rate[0] = data['gyro.z']
    last_update[0] = time.time()


# Saves every thrust reading, so we can average them while hovering
def thrust_callback(_timestamp, data, _logconf):
    drone_thrust.append(data['stabilizer.thrust'])


# Asks the drone to send its height, climb speed, spin rate and thrust every 10 ms
def start_logging(cf):
    log_conf1 = LogConfig(name='State', period_in_ms=10)
    log_conf1.add_variable('stateEstimate.z', 'float')
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


def height_trim():
    '''Extra thrust that nudges the drone back to TAKEOFF_HEIGHT.'''
    # Too low -> more thrust. Climbing -> less thrust, so it doesn't overshoot and bounce.
    trim = K_Z * (TAKEOFF_HEIGHT - height[0]) - K_VZ * climb_rate[0]
    return min(max(trim, -MAX_TRIM), MAX_TRIM)


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

    print('Hovering on power, push me around')

    # Fly until Ctrl+C or the position tracking drops out
    while not stop_event.is_set():
        thrust = hover_thrust  # Start from the measured hover thrust
        if HOLD_HEIGHT:
            if time.time() - last_update[0] > POSITION_TIMEOUT:
                print('\nLost position, landing...')
                break
            thrust += height_trim()

        # Asking for the spin it already has means its target heading
        # turns with it, so it doesn't twist back. Minus because the
        # firmware flips the yaw rate sent with send_setpoint.
        yaw = 0
        if FREE_YAW:
            yaw = -min(max(yaw_rate[0], -MAX_YAW_RATE), MAX_YAW_RATE)

        # Stay level (roll 0, pitch 0), nothing about where to be sideways,
        # so a push moves it freely
        cf.commander.send_setpoint(0, 0, yaw, int(min(max(thrust, 0), 65535)))
        print(f'thrust: {int(thrust):5d}  height: {height[0]:.2f} m', end='\r')
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

        start_logging(cf)
        time.sleep(0.5)

        # From here on Ctrl+C lands the drone instead of killing the script
        signal.signal(signal.SIGINT, handle_ctrl_c)
        pick_and_place(cf)
