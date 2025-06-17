import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

# URIs to the Crazyflie to connect to
uri_sensor = 'radio://0/80/2M/E7E7E7E7E7'
uri_drone = 'radio://0/80/2M/A0A0A0A0AE'
latest_cf_sensor_data = False

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def log_stab_callback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global latest_cf_sensor_data
    latest_cf_sensor_data = data
    """adding the timestamp to the latest sensor data dict"""
    latest_cf_sensor_data['timestamp'] = timestamp
    #print(timestamp)

def send_drone(scf, x, y, z, yaw):
        cf = scf.cf
        " note that the commander takes inputs as floats.cf"
        " note that launching the drone and giving it a yaw command is to much and it will often crash"
        cf.commander.send_position_setpoint(x,y,z,0.0)
        print(x,y,z,yaw)
        time.sleep(0.1)

        #cf.commander.send_stop_setpoint()
        # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
        #cf.commander.send_notify_setpoint_stop()

        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        #time.sleep(0.1)


def simple_log_async(scf,scf_2,logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    print('logconf_started')
    last_timestamp = 0
    while latest_cf_sensor_data is False:
        if latest_cf_sensor_data is not False:
            break
    while latest_cf_sensor_data is not False:
        if last_timestamp < latest_cf_sensor_data['timestamp']:
            #print(latest_cf_sensor_data['timestamp'])
            #send_drone(scf_2,0,0,0.5,0)
            """IMPORTANT add or subtract the right buffer to avoid sending the drone directly into the sensor"""
            """send_drone takes floats """
            send_drone(scf_2,latest_cf_sensor_data['stateEstimate.x'],
                        (latest_cf_sensor_data['stateEstimate.y']+2.0),
                        latest_cf_sensor_data['stateEstimate.z'],
                        latest_cf_sensor_data['stateEstimate.yaw'])

            # send_drone(scf_2,latest_cf_sensor_data['stateEstimate.x'],
            #            (latest_cf_sensor_data['stateEstimate.y']+1.5),
            #            latest_cf_sensor_data['stateEstimate.z'],
            #            latest_cf_sensor_data['stateEstimate.yaw'])

            last_timestamp = latest_cf_sensor_data['timestamp']
    logconf.stop()

    
if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    "This is a log config, a list of variables selected from a table of contents (TOC) on the crazyflie,"
    "we will send this list to each crazyflie, it will send back the values every x milliseconds (period_in_ms)"
    log_state_est = LogConfig(name='stateEstimate', period_in_ms=1000)
    log_state_est.add_variable('stateEstimate.x', 'float')
    log_state_est.add_variable('stateEstimate.y', 'float')
    log_state_est.add_variable('stateEstimate.z', 'float')

    log_state_est.add_variable('stateEstimate.roll', 'float')
    log_state_est.add_variable('stateEstimate.pitch', 'float')
    log_state_est.add_variable('stateEstimate.yaw', 'float')

    with SyncCrazyflie(uri_sensor, cf=Crazyflie(rw_cache='./cache')) as scf:
        with SyncCrazyflie(uri_drone, cf=Crazyflie(rw_cache='./cache')) as scf_2:
            reset_estimator(scf)
            reset_estimator(scf_2)
            simple_log_async(scf, scf_2, log_state_est)
    
    print(type(scf))
    print(type(scf_2))
        
