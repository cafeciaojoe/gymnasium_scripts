import time

import matplotlib.pyplot as plt

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

URI = uri_helper.uri_from_env(default='radio://0/40/2M/BADF00D007')  # Change to your Crazyflie's URI

# Yaw is measured in the Lighthouse global frame, so "aligned" means facing
# the global origin's x-axis set by the Lighthouse config (see Getting_Started/Write_Lighthouse_Config).

# Crazyflie's attitude
roll = [0]
pitch = [0]
yaw = [0]

# 2 = Lighthouse is sending positions to the estimator
LIGHTHOUSE_TRACKING = 2
lighthouse_status = [0]

min_power = 1000  # Minimum motor power
max_power = 30000  # Maximum motor power. Warning: Avoid setting this above 30000
min_angle = 0   # The Crazyflie hovers while: min_angle < roll,pitch < max_angle
max_angle = 30
max_yaw_angle = 45  # Yaw away from the global origin [deg] that gives full yaw power
yaw_direction = 1   # Set to -1 if the drone twists further away instead of back


def attitude_callback(timestamp, data, logconf):
    roll.append(data['stateEstimate.roll'])
    pitch.append(data['stateEstimate.pitch'])
    yaw.append(data['stateEstimate.yaw'])
    lighthouse_status.append(data['lighthouse.status'])


def setup_lighthouse(scf):
    if scf.cf.param.get_value('deck.bcLighthouse4') != '1':
        print('No Lighthouse deck found. Attach one and try again.')
        exit()

    # Use the Kalman estimator so yaw follows the Lighthouse global frame
    scf.cf.param.set_value('stabilizer.estimator', '2')
    time.sleep(0.5)
    print('Resetting estimator...')
    reset_estimator(scf.cf)


def start_position_printing(scf):
    log_conf = LogConfig(name='Attitude', period_in_ms=100)
    log_conf.add_variable('stateEstimate.roll', 'float')
    log_conf.add_variable('stateEstimate.pitch', 'float')
    log_conf.add_variable('stateEstimate.yaw', 'float')
    log_conf.add_variable('lighthouse.status', 'uint8_t')
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(attitude_callback)
    log_conf.start()


def power_profile(angle):
    if abs(angle) > max_angle:
        power = int(max_power)
    else:
        power = int((min_power*max_angle + (max_power-min_power)*abs(angle))/(max_angle-min_angle))
    return power


def yaw_power_profile(angle):
    if abs(angle) > max_yaw_angle:
        power = int(max_power)
    else:
        power = int(min_power + (max_power-min_power)*abs(angle)/max_yaw_angle)
    return power


def power_distribution():
    m1_p = 0
    m2_p = 0
    m3_p = 0
    m4_p = 0
    m1_r = 0
    m2_r = 0
    m3_r = 0
    m4_r = 0
    m1_y = 0
    m2_y = 0
    m3_y = 0
    m4_y = 0
    if pitch[-1] < 0:
        m1_p = power_profile(pitch[-1])
        m4_p = power_profile(pitch[-1])
    elif pitch[-1] > 0:
        m2_p = power_profile(pitch[-1])
        m3_p = power_profile(pitch[-1])
    if roll[-1] < 0:
        m3_r = power_profile(roll[-1])
        m4_r = power_profile(roll[-1])
    elif roll[-1] > 0:
        m1_r = power_profile(roll[-1])
        m2_r = power_profile(roll[-1])
    # Yaw: speeding up one diagonal pair of motors twists the drone.
    # M1 & M3 spin counter-clockwise, so they twist the body clockwise (yaw decreases).
    # M2 & M4 spin clockwise, so they twist the body counter-clockwise (yaw increases).
    # No yaw correction while Lighthouse isn't tracking, as yaw can't be trusted then.
    tracking = lighthouse_status[-1] == LIGHTHOUSE_TRACKING
    if not tracking:
        pass
    elif yaw[-1] * yaw_direction > 0:
        m1_y = yaw_power_profile(yaw[-1])
        m3_y = yaw_power_profile(yaw[-1])
    elif yaw[-1] * yaw_direction < 0:
        m2_y = yaw_power_profile(yaw[-1])
        m4_y = yaw_power_profile(yaw[-1])
    m1 = min(m1_p + m1_r + m1_y, max_power)
    m2 = min(m2_p + m2_r + m2_y, max_power)
    m3 = min(m3_p + m3_r + m3_y, max_power)
    m4 = min(m4_p + m4_r + m4_y, max_power)
    print('\n' * 50)  # Clear screen
    print(f'yaw: {yaw[-1]:6.1f} deg' + ('' if tracking else '  (Lighthouse lost, yaw off)'))
    print(f'[{m4:^5}]    [{m1:^5}]')
    print(r'      \   /    ')
    print(r'       \ /     ')
    print(r'       / \     ')
    print(r'      /   \    ')
    print(f'[{m3:^5}]    [{m2:^5}]')
    scf.cf.param.set_value('motorPowerSet.m1', str(m1))
    scf.cf.param.set_value('motorPowerSet.m2', str(m2))
    scf.cf.param.set_value('motorPowerSet.m3', str(m3))
    scf.cf.param.set_value('motorPowerSet.m4', str(m4))


def vibration(scf):
    scf.cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(1)
    while abs(roll[-1]) < 170:
        power_distribution()
        time.sleep(0.1)

    scf.cf.param.set_value('motorPowerSet.m1', 0)
    scf.cf.param.set_value('motorPowerSet.m2', 0)
    scf.cf.param.set_value('motorPowerSet.m3', 0)
    scf.cf.param.set_value('motorPowerSet.m4', 0)
    time.sleep(0.5)
    scf.cf.param.set_value('motorPowerSet.enable', '0')
    time.sleep(1)


def simple_plot():
    points = [
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 4 roll
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 1 roll
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 3 roll
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 2 roll
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 4 pitch
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 1 pitch
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 3 pitch
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 2 pitch
    ]
    titles = ['Motor 4', 'Motor 1', 'Motor 3', 'Motor 2']
    y_labels = ['M4 power', 'M1 power', 'M3 power', 'M2 power']

    fig1, axs1 = plt.subplots(2, 2, figsize=(10, 8))

    for i, ax in enumerate(axs1.flat):
        x_vals, y_vals = zip(*points[i])
        ax.plot(x_vals, y_vals, marker='o')
        ax.set_title(titles[i])
        ax.set_xlabel('Roll [deg]')
        ax.set_ylabel(y_labels[i])
        ax.grid(True)

    fig1.tight_layout()

    fig2, axs2 = plt.subplots(2, 2, figsize=(10, 8))

    for i, ax in enumerate(axs2.flat):
        x_vals, y_vals = zip(*points[i+4])
        ax.plot(x_vals, y_vals, marker='o', color='orange')
        ax.set_title(titles[i])
        ax.set_xlabel('Pitch [deg]')
        ax.set_ylabel(y_labels[i])
        ax.grid(True)

    fig2.tight_layout()

    # Yaw: M1 & M3 push back when yaw is positive, M2 & M4 when it is negative
    yaw_pos = [(-max_yaw_angle, 0), (0, 0), (0, min_power), (max_yaw_angle, max_power)]
    yaw_neg = [(-max_yaw_angle, max_power), (0, min_power), (0, 0), (max_yaw_angle, 0)]
    if yaw_direction < 0:
        yaw_pos, yaw_neg = yaw_neg, yaw_pos
    yaw_points = [yaw_neg, yaw_pos, yaw_pos, yaw_neg]  # Motor 4, 1, 3, 2

    fig3, axs3 = plt.subplots(2, 2, figsize=(10, 8))

    for i, ax in enumerate(axs3.flat):
        x_vals, y_vals = zip(*yaw_points[i])
        ax.plot(x_vals, y_vals, marker='o', color='green')
        ax.set_title(titles[i])
        ax.set_xlabel('Yaw [deg]')
        ax.set_ylabel(y_labels[i])
        ax.grid(True)

    fig3.tight_layout()
    print('Close the graphs to start...')
    plt.show()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        simple_plot()
        setup_lighthouse(scf)
        start_position_printing(scf)

        print('Waiting for Lighthouse tracking...')
        while lighthouse_status[-1] != LIGHTHOUSE_TRACKING:
            time.sleep(0.1)
        print('Lighthouse tracking!')

        time.sleep(1)
        vibration(scf)
