import time
import random
import matplotlib.pyplot as plt
import numpy as np

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
from cflib.utils.multiranger import Multiranger

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

def is_close(range, min_dist):
    if range is None:
        return False
    else:
        return range < min_dist


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        time.sleep(0.5)
        scf.cf.platform.send_arming_request(True)
        with MotionCommander(scf, default_height=1.0) as motion_commander:
            time.sleep(0.5)
            with Multiranger(scf) as multiranger:
                time.sleep(0.5)
                keep_flying = True
                def_vel = 0.1
                
                while keep_flying:
                    velocity_x = 0
                    velocity_y = 0

                    if is_close(multiranger.up, 0.5):
                        vel_z = 5*def_vel
                        sound = 10
                    else:
                        vel_z = -def_vel
                        sound = 13
                    
                    if is_close(multiranger.front, 0.5):
                        velocity_x = -2*def_vel
                    if is_close(multiranger.back, 0.5):
                        velocity_x = 2*def_vel

                    if is_close(multiranger.left, 0.5):
                        velocity_y = -2*def_vel
                    if is_close(multiranger.right, 0.5):
                        velocity_y = 2*def_vel



                    motion_commander.start_linear_motion(velocity_x, velocity_y, vel_z)
                    scf.cf.param.set_value('sound.effect', str(sound))
                    time.sleep(0.05)