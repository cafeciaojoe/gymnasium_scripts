import time

import cflib
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger

URIS = {
    'radio://0/80/2M/E7E7E7E7E7',
    'radio://0/80/2M/E7E7E7E7E8',
}
def is_close(range, min_dist):
    if range is None:
        return False
    else:
        return range < min_dist


def keep_up(scf):
        scf.cf.param.set_value('sound.effect', str(12))
        time.sleep(0.5)
        scf.cf.param.set_value('sound.freq', str(0))
        time.sleep(0.5)
        with MotionCommander(scf, default_height=1.2) as motion_commander:
            print('Taking off...')
            time.sleep(0.5)
            with Multiranger(scf) as multiranger:
                time.sleep(0.5)
                keep_flying = True
                def_vel = 0.1
                
                while keep_flying:
                    velocity_x = 0
                    velocity_y = 0

                    if is_close(multiranger.up, 0.4):
                        sound = 0
                        
                        if is_close(multiranger.up, 0.15):
                            vel_z = 0
                        else:
                            vel_z = 2*def_vel
                    else:
                        vel_z = -2*def_vel
                        sound = 13
                    
                    if is_close(multiranger.front, 0.3):
                        velocity_x = -4*def_vel
                    if is_close(multiranger.back, 0.3):
                        velocity_x = 4*def_vel

                    if is_close(multiranger.left, 0.3):
                        if is_close(multiranger.right, 0.3):
                            keep_flying = False
                        else:
                            velocity_y = -4*def_vel
                    if is_close(multiranger.right, 0.3):
                        if is_close(multiranger.left, 0.3):
                            keep_flying = False
                        else:
                            velocity_y = 4*def_vel



                    motion_commander.start_linear_motion(velocity_x, velocity_y, vel_z)
                    scf.cf.param.set_value('sound.effect', str(sound))
                    time.sleep(0.05)
            time.sleep(0.2)
            scf.cf.param.set_value('sound.effect', 7)
            motion_commander.land()
            time.sleep(3)
            scf.cf.param.set_value('sound.effect', 0)
            time.sleep(0.2)


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')


    with Swarm(URIS, factory=factory) as swarm:
        swarm.parallel_safe(keep_up)