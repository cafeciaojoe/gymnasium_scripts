import time
import math
import matplotlib.pyplot as plt
from pynput import mouse
from pynput.mouse import Button
import csv
from datetime import datetime
import os

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper

from cflib.positioning.motion_commander import MotionCommander

Uri_sensor = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
Uri_drone = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E8')

points = 5
x = []
y = []
z = []
sens_Vz = []
yaw = []
collecting = True
last_time = None
durations = []
ENABLE_YAW = False
USE_TIMESTAMPS = False
time_dur = 3  # s

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


def velocity_callback(timestamp, data, logconf):
    global sens_Vz
    sens_Vz.append(data['stateEstimate.vz'])


def start_velocity_printing(scf):
    log_conf1 = LogConfig(name='Velocity', period_in_ms=20)
    log_conf1.add_variable('stateEstimate.vz', 'float')
    scf.cf.log.add_config(log_conf1)
    log_conf1.data_received_cb.add_callback(velocity_callback)
    log_conf1.start()


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


def vel_from_points(x1, y1, x2, y2, dt):

    d = math.sqrt(pow((x1 - x2), 2)+pow((y1 -y2), 2))
    if USE_TIMESTAMPS is True:
        Vel = 0.8* (d / (dt))
    else:
        Vel = d / time_dur
    Vx = Vel * (x2-x1)/d
    Vy = Vel * (y2-y1)/d
    return Vx, Vy

def run_sequence(scf, x, y, z, yaw, durations):
    # yaw = [0]*len(x)
    # duration = 3  # sec
    print('Drone ready to fly!')

    # Arm the Crazyflie
    scf.cf.platform.send_arming_request(True)
    time.sleep(1.0)

    with MotionCommander(scf, default_height=1.0) as mc:
        time.sleep(2.0)
        for i in range(0, len(durations)):
            print(f'Going from x[i]={x[i]} to x[i+1]={x[i+1]} in {durations[i]}')
            velx, vely = vel_from_points(x[i], y[i], x[i+1], y[i+1], durations[i])
            print(f'With velx = {velx} and vely = {vely}')
            start_time = time.time()
            print(f'start_time = {start_time} and end_time = {start_time+durations[i]}')
            # while time.time() <= start_time + durations[i]:
            while time.time() <= start_time + time_dur:
                mc.start_linear_motion(velx, vely, sens_Vz[-1], 0)
                time.sleep(0.01)
        time.sleep(0.01)
        mc.land()
        time.sleep(2)
        scf.cf.platform.send_arming_request(False)


def collect_data(cursor_xpos, cursor_ypos, button, pressed):
    global collecting, last_time
    if pressed and button == Button.left:
        current_time = time.time()
        if last_time is not None:  # The first click is to calibrate the time
            pos = get_estimated_position(scf_s)
            x.append(pos[0])
            y.append(pos[1])
            z.append(pos[2])
            yaw.append(pos[3])
            duration = current_time - last_time
            durations.append(duration)
            print(f"Time since last click: {duration:.3f} seconds")
        else:
            pos = get_estimated_position(scf_s)
            x.append(pos[0])
            y.append(pos[1])
            z.append(1.0)
            print('First click recorded.')
            pos = get_estimated_position(scf_s)
        last_time = current_time
        scf_s.cf.param.set_value('sound.effect', '7')
        time.sleep(1)
        scf_s.cf.param.set_value('sound.effect', '0')
    elif pressed and button == Button.right:
        print('Right mouse button pressed - stop collecting data.')
        scf_s.cf.param.set_value('sound.effect', '2')
        time.sleep(1)
        scf_s.cf.param.set_value('sound.effect', '0')
        # Generate filename with current date and time
        date_str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')  # e.g. "2025-09-09_14-23-45"
        # Update the path to save CSV files in the `data_files` folder
        data_files_dir = os.path.join(os.path.dirname(__file__), 'data_files')
        os.makedirs(data_files_dir, exist_ok=True)
        filename = os.path.join(data_files_dir, f'data_{date_str}.csv')
        # Save to CSV
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z', 'durations'])  # header
            for t, xi, yi, zi in zip(x, y, z, durations):
                writer.writerow([t, xi, yi, zi])
        print(f'Data saved to {filename}')
        collecting = False
        return False  # Stop the listener


if __name__ == '__main__':
    cflib.crtp.init_drivers()
    print('Ready?...')
    time.sleep(1)
    factory = CachedCfFactory(rw_cache='./cache')
    with SyncCrazyflie(Uri_sensor, cf=Crazyflie(rw_cache='./cache')) as scf_s:
        print('Go!')
        while collecting:
            with mouse.Listener(on_click=collect_data) as listener:
                listener.join()

        simple_plot()
        print('Drone ready to fly!')
        print(f'x: {x},\n y: {y},\n durations: {durations}')

        start_velocity_printing(scf_s)

        with SyncCrazyflie(Uri_drone, cf=Crazyflie(rw_cache='./cache')) as scf_d:
            scf_d.cf.param.set_value('posCtlPid.xVelMax', '5')
            scf_d.cf.param.set_value('posCtlPid.yVelMax', '5')
            scf_d.cf.param.set_value('posCtlPid.zVelMax', '5')
            time.sleep(0.5)
            run_sequence(scf_d, x, y, z, yaw, durations)
