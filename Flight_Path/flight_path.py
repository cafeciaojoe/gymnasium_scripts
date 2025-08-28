import time

import matplotlib.pyplot as plt
from pynput import mouse
from pynput.mouse import Button

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

Uri_sensor = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
Uri_drone = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')

points = 5
x = []
y = []
z = []
yaw = []
collecting = True
last_time = None
durations = []
ENABLE_YAW = False


def get_estimated_position(scf):
    log_conf = LogConfig(name='Position', period_in_ms=10)
    log_conf.add_variable('stateEstimate.x', 'float')
    log_conf.add_variable('stateEstimate.y', 'float')
    log_conf.add_variable('stateEstimate.z', 'float')
    log_conf.add_variable('stateEstimate.yaw', 'float')

    with SyncLogger(scf, log_conf) as logger:
        for entry in logger:
            x = entry[1]['stateEstimate.x']
            y = entry[1]['stateEstimate.y']
            z = entry[1]['stateEstimate.z']
            yaw = entry[1]['stateEstimate.yaw']
            position = [x, y, z, yaw]
            return position


def simple_plot():
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    ax.scatter(x, y, z, color='blue', s=50)

    # Add numbered labels next to each point
    for i in range(len(x)):
        ax.text(x[i], y[i], z[i], f'{i}', fontsize=10, color='red')

    # Set axis labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(min(0, min(x)), max(0, max(x)))
    ax.set_ylim(min(0, min(y)), max(0, max(y)))
    ax.set_zlim(min(0, min(z)), max(0, max(z)))
    ax.set_box_aspect([1, 1, 1])

    plt.title('3D Setpoints')
    print('Close the graph to fly...')
    plt.show()


def run_sequence(scf, x, y, z, yaw, durations):
    # yaw = [0]*len(x)
    commander = scf.cf.high_level_commander
    # duration = 3  # sec
    print('Drone ready to fly!')

    # Arm the Crazyflie
    scf.cf.platform.send_arming_request(True)
    time.sleep(1.0)

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    for i in range(len(x)):
        commander.go_to(x[i], y[i], z[i], yaw[i] if ENABLE_YAW is True else 0, durations[i])
        time.sleep(durations[i]+0.2)
    time.sleep(2)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()
    scf.cf.platform.send_arming_request(False)


def collect_data(cursor_xpos, cursor_ypos, button, pressed):
    global collecting, last_time
    if pressed and button == Button.left:
        current_time = time.time()
        if last_time is not None:  # The first click is to calibrate the time
            pos = get_estimated_position(scf)
            x.append(pos[0])
            y.append(pos[1])
            z.append(pos[2])
            yaw.append(pos[3])
            duration = current_time - last_time
            durations.append(duration)
            print(f"Time since last click: {duration:.3f} seconds")
        else:
            print('First click recorded.')
        last_time = current_time
        scf.cf.param.set_value('sound.effect', '7')
        time.sleep(1)
        scf.cf.param.set_value('sound.effect', '0')
    elif pressed and button == Button.right:
        print('Right mouse button pressed - stop collecting data.')
        scf.cf.param.set_value('sound.effect', '2')
        time.sleep(1)
        scf.cf.param.set_value('sound.effect', '0')
        collecting = False
        return False  # Stop the listener


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    print('Ready?...')
    time.sleep(1)
    factory = CachedCfFactory(rw_cache='./cache')
    with SyncCrazyflie(Uri_sensor, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('Go!')
        while collecting:
            with mouse.Listener(on_click=collect_data) as listener:
                listener.join()

    simple_plot()
    print('Drone ready to fly!')
    with SyncCrazyflie(Uri_drone, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.set_value('posCtlPid.xVelMax', '5')
        scf.cf.param.set_value('posCtlPid.yVelMax', '5')
        scf.cf.param.set_value('posCtlPid.zVelMax', '5')
        run_sequence(scf, x, y, z, yaw, durations)
