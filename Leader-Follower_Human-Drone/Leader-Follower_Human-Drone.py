import math
import time

import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander

# Change uris according to your setup
# URIs in a swarm using the same radio must also be on the same channel
Follower = 'radio://0/80/2M/E7E7E7E7E7'  # Follower
Leader = 'radio://0/80/2M/E7E7E7E7E8'  # Leader

r_min = 0.75  # The minimum distance between the 2 drones
r_max = 1.25  # The maximum distance between the 2 drones
DEFAULT_HEIGHT = 0.75
MAX_VELOCITY = 2
x1 = [0]
y1 = [0]
z1 = [0]
x2 = [0]
y2 = [0]
z2 = [0]
yaw1 = [0]
yaw2 = [0]

# List of URIs
uris = {
    Follower,
    Leader,
}


def wait_for_param_download(scf):
    while not scf.cf.param.is_updated:
        time.sleep(1.0)
    print('Parameters downloaded for', scf.cf.link_uri)


def arm(scf):
    scf.cf.platform.send_arming_request(True)
    time.sleep(1.0)


def set_max_vel(scf):
    scf.cf.param.set_value('posCtlPid.xVelMax', 1.1*MAX_VELOCITY)
    time.sleep(0.5)


def velocity_profile_plot():

    P0 = (0,  MAX_VELOCITY)
    P1 = (r_min,  0)
    P2 = (r_max,  0)
    P3 = (r_min+r_max,  MAX_VELOCITY)
    P4 = (6, MAX_VELOCITY)

    # Build a list of all points in order
    pts = [P0, P1, P2, P3, P4]

    # Unzip into x and y lists
    xs, ys = zip(*pts)

    plt.figure()
    plt.plot(xs, ys, marker='o', linewidth=2)  # draws P0→P1→P2→P3→P4
    plt.title('Velocity Profile')
    plt.xlabel('Distance [m]')
    plt.ylabel('Velocity [m/s]')
    plt.grid()
    plt.axis('equal')
    print('Close graph to continue...')
    plt.show()


def pos_to_vel(x1, y1, x2, y2, dist):
    '''
    This function takes two points on the x-y plane and outputs
    two components of the velocity vector: one along the x-axis
    and one along the y-axis. The combined vector represents the
    total velocity, pointing from point 1 to point 2, with a
    magnitude relative distance of these points. These 2 velocity
    vectors are meant to be used by the motion commander.
    '''
    if dist == 0:
        magn = 0
    elif dist <= r_min:
        magn = MAX_VELOCITY * (1 - dist / r_min)
    elif r_min < dist <= r_max:
        magn = 0
    elif r_max < dist <= r_min+r_max:
        magn = MAX_VELOCITY * (dist - r_max) / r_min
    elif dist > r_min+r_max:
        magn = MAX_VELOCITY

    Vx = magn * (x2-x1)/dist
    Vy = magn * (y2-y1)/dist
    return Vx, Vy


def position_callback(uri, data):
    global d
    if uri == Follower:
        x1.append(data['stateEstimate.x'])
        y1.append(data['stateEstimate.y'])
        z1.append(data['stateEstimate.z'])
        yaw1.append(data['stateEstimate.yaw'])
    elif uri == Leader:
        x2.append(data['stateEstimate.x'])
        y2.append(data['stateEstimate.y'])
        z2.append(data['stateEstimate.z'])
        yaw2.append(data['stateEstimate.yaw'])

    d = math.sqrt(pow((x1[-1]-x2[-1]), 2)+pow((y1[-1]-y2[-1]), 2))


def start_position_printing(scf):
    log_conf1 = LogConfig(name='Position', period_in_ms=10)
    log_conf1.add_variable('stateEstimate.x', 'float')
    log_conf1.add_variable('stateEstimate.y', 'float')
    log_conf1.add_variable('stateEstimate.z', 'float')
    log_conf1.add_variable('stateEstimate.yaw', 'float')
    scf.cf.log.add_config(log_conf1)
    log_conf1.data_received_cb.add_callback(lambda _timestamp, data, _logconf: position_callback(scf.cf.link_uri, data))
    log_conf1.start()


def leader_follower(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:

        # The follower turns until it is aligned with the global coordinate system
        while abs(yaw1[-1]) > 2:
            if scf.__dict__['_link_uri'] == Follower:
                if yaw1[-1] > 0:
                    mc.start_turn_right(36 if abs(yaw1[-1]) > 15 else 9)
                elif yaw1[-1] < 0:
                    mc.start_turn_left(36 if abs(yaw1[-1]) > 15 else 9)

            elif scf.__dict__['_link_uri'] == Leader:
                mc.stop()
            time.sleep(0.005)

        time.sleep(0.5)

        while z2[-1] > 0.2:  # Fly while this condition is true.

            if scf.__dict__['_link_uri'] == Follower:
                if d > r_max:  # Too far, move closer
                    cmd_vel_x, cmd_vel_y = pos_to_vel(x1[-1], y1[-1], x2[-1], y2[-1], d)
                elif d >= r_min and d <= r_max:  # Optimal distance, stay put
                    cmd_vel_x = 0
                    cmd_vel_y = 0
                elif d < r_min:  # Too close, back away
                    opp_cmd_vel_x, opp_cmd_vel_y = pos_to_vel(x1[-1], y1[-1], x2[-1], y2[-1], d)
                    cmd_vel_x = -opp_cmd_vel_x
                    cmd_vel_y = -opp_cmd_vel_y

                mc.start_linear_motion(cmd_vel_x, cmd_vel_y, 0)

            elif scf.__dict__['_link_uri'] == Leader:
                pass

            time.sleep(0.005)
        mc.land()


def trajectory_plots(uri1x, uri1y, uri1z, uri2x, uri2y, uri2z):
    font1 = {'family': 'serif', 'color': '#2d867e', 'size': 20}
    font2 = {'family': 'serif', 'color': '#2d867e', 'size': 15}

    plt.figure(1)
    plt.plot(uri1x[10:], uri1y[10:], '.', color='#5681e6')
    plt.plot(uri2x[10:], uri2y[10:], '.', color='#d34700')
    plt.xlabel("X-Axis", fontdict=font2)
    plt.ylabel("Y-Axis", fontdict=font2)
    plt.title("2D Drone Trajectories", fontdict=font1)
    plt.legend(["Follower", "Leader"], loc="lower right")
    ax = plt.gca()
    ax.set_aspect('equal')
    plt.grid(color='grey', linestyle='--', linewidth=0.5)

    plt.figure(2)
    bx = plt.axes(projection='3d')
    bx.plot3D(uri1x[10:], uri1y[10:], uri1z[10:], '.', color='#5681e6')
    bx.plot3D(uri2x[10:], uri2y[10:], uri2z[10:], '.', color='#d34700')
    plt.xlabel("X-Axis", fontdict=font2)
    plt.ylabel("Y-Axis", fontdict=font2)
    plt.title("3D Drone Trajectories", fontdict=font1)
    plt.legend(["Follower", "Leader"], loc="lower right")
    bx.set_aspect('equal')
    plt.show()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    velocity_profile_plot()

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:

        swarm.reset_estimators()

        swarm.parallel_safe(set_max_vel)

        print('Waiting for parameters to be downloaded...')
        swarm.parallel_safe(wait_for_param_download)
        time.sleep(0.5)

        swarm.parallel_safe(arm)
        time.sleep(0.5)

        swarm.parallel_safe(start_position_printing)
        time.sleep(0.5)

        swarm.parallel_safe(leader_follower)
        time.sleep(0.5)

        swarm.close_links()
        time.sleep(0.5)

        trajectory_plots(x1, y1, z1, x2, y2, z2)
