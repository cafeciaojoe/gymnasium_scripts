import time
import random
import math

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

SENSOR1 = uri_helper.uri_from_env(default='radio://0/80/2M/BADCAFE002')
SENSOR2 = uri_helper.uri_from_env(default='radio://0/80/2M/BADCAFE003')
URI = uri_helper.uri_from_env(default='radio://0/80/2M/BADCAFE007')

SENSORS = {
    SENSOR1,
    SENSOR2,
}


acc_x1 = [0]
acc_y1 = [0]
acc_z1 = [0]
acc_x2 = [0]
acc_y2 = [0]
acc_z2 = [0]

limit = 4  #gs
score = 0
First_to_win = 3
winner = None

def acceleration_callback(uri, data):
    global winner

    if uri == SENSOR1:
        acc_x1.append(data['acc.x'])
        acc_y1.append(data['acc.y'])
        acc_z1.append(data['acc.z']-1)
        acc_x1.pop(0)
        acc_y1.pop(0)
        acc_z1.pop(0)
    elif uri == SENSOR2:
        acc_x2.append(data['acc.x'])
        acc_y2.append(data['acc.y'])
        acc_z2.append(data['acc.z']-1)
        acc_x2.pop(0)
        acc_y2.pop(0)
        acc_z2.pop(0)
    
    acc1 = math.sqrt(pow(acc_x1[-1], 2)+pow(acc_y1[-1], 2)+pow(acc_z1[-1], 2))
    acc2 = math.sqrt(pow(acc_x2[-1], 2)+pow(acc_y2[-1], 2)+pow(acc_z2[-1], 2))

    if acc1 > acc2 and acc1 > limit:
        winner = 1
    elif acc2 > acc1 and acc2 > limit:
        winner = 2
    else:
        winner = None  # no winner yet


def start_acceleration_printing(scf):
    log_conf1 = LogConfig(name='Acceleration', period_in_ms=10)
    log_conf1.add_variable('acc.x', 'float')
    log_conf1.add_variable('acc.y', 'float')
    log_conf1.add_variable('acc.z', 'float')
    scf.cf.log.add_config(log_conf1)
    log_conf1.data_received_cb.add_callback(lambda _timestamp, data, _logconf: acceleration_callback(scf.cf.link_uri, data))
    log_conf1.start()


def set_color(scf, color, intensity):  #  1:Blue,   2:Red,   3:Green
    scf.cf.param.set_value('ring.effect', '7')  # Solid color

    if color == 1:
        scf.cf.param.set_value('ring.solidBlue', str(intensity))
        scf.cf.param.set_value('ring.solidRed', '0')
        scf.cf.param.set_value('ring.solidGreen', '0')
    elif color == 2:
        scf.cf.param.set_value('ring.solidBlue', '0')
        scf.cf.param.set_value('ring.solidRed', str(intensity))
        scf.cf.param.set_value('ring.solidGreen', '0')
    elif color == 3:
        scf.cf.param.set_value('ring.solidBlue', '0')
        scf.cf.param.set_value('ring.solidRed', '0')
        scf.cf.param.set_value('ring.solidGreen', str(intensity))
    else:
        print(f'Invalid color selection: color = {color}')
    
    time.sleep(0.2)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    factory = CachedCfFactory(rw_cache='./cache')

    with SyncCrazyflie(SENSOR1, cf=Crazyflie(rw_cache='./cache')) as sens1:
        set_color(sens1, 1, 10)

    with SyncCrazyflie(SENSOR2, cf=Crazyflie(rw_cache='./cache')) as sens2:
        set_color(sens2, 2, 10)

    with Swarm(SENSORS, factory=factory) as swarm:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

            swarm.parallel_safe(start_acceleration_printing)
            print('Started collecting accelerometer data')

            with MotionCommander(scf, default_height=0.8) as mc:
                print('Taking off')

                while abs(score) < First_to_win:

                    dist = 0

                    time.sleep(random.uniform(1, 10))  # Wait for a random time period
                    print('Ready for input')

                    set_color(scf, 3, 255)  # Set the color to Green when it's ready for input

                    while dist == 0:

                        if winner == 1:
                            dist = 0.5
                            score += 1
                            col = 1
                            print('Winner Blue')

                        elif winner == 2:
                            dist = -0.5
                            score -= 1
                            col = 2
                            print('Winner Red')

                        time.sleep(0.001)

                    set_color(scf, col, 255)  # Set the color to the winner's color

                    time.sleep(1)

                    mc.move_distance(dist, 0, 0, 0.5)

                    print(f'score: {score}')

                    time.sleep(2)
                    scf.cf.param.set_value('ring.effect', '6')  # Something neutral

                    time.sleep(1)

                mc.land()
                time.sleep(1)
                swarm.close_links()
                time.sleep(1)

                # Winners celebration
                with SyncCrazyflie(SENSOR1, cf=Crazyflie(rw_cache='./cache')) as sens1:
                    with SyncCrazyflie(SENSOR2, cf=Crazyflie(rw_cache='./cache')) as sens2:

                        if score > 0:                                
                            sens1.cf.param.set_value('ring.effect', '4')
                            sens2.cf.param.set_value('ring.effect', '0')
                            time.sleep(0.5)
                        elif score < 0:
                            sens1.cf.param.set_value('ring.effect', '0')
                            sens2.cf.param.set_value('ring.effect', '4')
                            time.sleep(0.5)
