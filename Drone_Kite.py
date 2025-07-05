import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# URI to the Crazyflie to connect to
uri = 'radio://0/30/2M/BADC0DE012'

position = [0,0,0]
orientation = [0,0,0]  # roll, pitch, yaw

target_position = [0]

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def log_stab_callback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    position[0] = data['stateEstimate.x']
    position[1] = data['stateEstimate.y']
    position[2] = data['stateEstimate.z']
    orientation[0] = data['stateEstimate.roll']
    orientation[1] = data['stateEstimate.pitch']
    orientation[2] = data['stateEstimate.yaw']

    # print(f"Position: {position}")
    # print(f"Orientation: {orientation}")

def log_stab_callback_2(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))

    target_position[0] = data['ctrltarget.x']
    # print(f"Position: {position}")
    # print(f"Orientation: {orientation}")

def simple_log_async(scf, logconf_a, logconf_b):
    cf = scf.cf
    cf.log.add_config(logconf_a)
    logconf_a.data_received_cb.add_callback(log_stab_callback)
    logconf_a.start()

    cf.log.add_config(logconf_b)
    logconf_b.data_received_cb.add_callback(log_stab_callback_2)
    logconf_b.start()
    # time.sleep(5)
    # logconf.stop()

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    lg_stab = LogConfig(name='stateEstimate', period_in_ms=10)
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')

    lg_stab.add_variable('stateEstimate.roll', 'float')
    lg_stab.add_variable('stateEstimate.pitch', 'float')
    lg_stab.add_variable('stateEstimate.yaw', 'float')

    lg_stab_2 = LogConfig(name='ctrltarget', period_in_ms=10)
    lg_stab_2.add_variable('ctrltarget.x', 'float')

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        simple_log_async(scf, lg_stab, lg_stab_2)
        time.sleep(1)
        #exit()
        #take_off(scf)
        scf.cf.platform.send_arming_request(True)
        time.sleep(1)
        
        scf.cf.high_level_commander.takeoff(.5, 1)
        time.sleep(2.5)
        while True:
            #scf.cf.high_level_commander.go_to(position[0], position[1], position[2], 0, 1, relative=False)
            scf.cf.commander.send_position_setpoint(position[0], position[1], position[2],0)
            print(f'a_x = {position[0]}, x_t = {target_position[0]}')
            time.sleep(1.1)
        
        