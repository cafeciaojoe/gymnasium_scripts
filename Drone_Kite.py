import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/30/2M/A0A0A0A0AA'

position = [0,0,0]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    # time.sleep(5)
    # logconf.stop()

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='stateEstimate', period_in_ms=1000)
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')

    lg_stab.add_variable('stateEstimate.roll', 'float')
    lg_stab.add_variable('stateEstimate.pitch', 'float')
    lg_stab.add_variable('stateEstimate.yaw', 'float')

    def take_off(cf, height):
        take_off_time = 1.0
        sleep_time = 0.1
        steps = int(take_off_time / sleep_time)
        vz = height / take_off_time

        print(f'take off at {height}')

        for i in range(steps):
            cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
            time.sleep(sleep_time)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        simple_log_async(scf, lg_stab)
        time.sleep(10)
        #take_off(scf)
        commander = scf.cf.high_level_commander()
        scf.commander.send_position_setpoint(position[0],
                                            position[1],
                                            position[2],
                                            position[3])