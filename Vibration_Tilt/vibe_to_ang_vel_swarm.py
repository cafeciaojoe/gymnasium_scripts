import time
import threading
import math
import numpy as np
from scipy.spatial.transform import Rotation
from collections import deque

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Connection URI for the Crazyflie
uris = [
'radio://0/30/2M/a0a0a0a0aa',
'radio://0/30/2M/a0a0a0a0ae'
]

# Global dictionary to store quaternion data for each Crazyflie
quat_data_dict = {}

# Motor power settings (swap min and max for inverse experience!)
min_power = 5000   # Minimum motor power 
max_power = 20000  # Maximum motor power 

# Angular velocity response settings
max_angular_velocity_dps = 400  # Maximum expected angular velocity (degrees per second)
velocity_smoothing_factor = 0.7  # Smoothing factor for velocity changes (0-1)
current_angular_velocity = 0.0   # Smoothed angular velocity

# Vibration response curve
vibration_exponent = 1  # Controls vibration intensity curve

log_period = 10 #ms

global execute
execute = True


def attitude_callback(timestamp, data, logconf):
    """
    Callback function that receives quaternion data from Crazyflie.
    Stores quaternion data with timestamps for velocity calculation.
    """
    quat = [data['stateEstimate.qw'], data['stateEstimate.qx'], 
            data['stateEstimate.qy'], data['stateEstimate.qz']]
    
    # Extract URI from logconf.name
    uri = logconf.name.split(' ')[-1]
    
    # Ensure each Crazyflie has its own deque
    if uri not in quat_data_dict:
        quat_data_dict[uri] = deque(maxlen=10)
    
    quat_data_dict[uri].append({
        'timestamp': timestamp / 1000.0,  # Convert milliseconds to seconds
        'quaternion': quat
    })
    
    #print(f"URI: {uri}, Timestamp: {timestamp}, Data: {data}")


def start_logging(scf):
    """
    Set up logging to receive quaternion data from Crazyflie.
    """
    log_conf = LogConfig(name='Quaternion_Attitude for '+ scf._link_uri, period_in_ms=log_period)  # 20 Hz for better velocity estimation
    
    # Add quaternion variables to logging
    log_conf.add_variable('stateEstimate.qw', 'float')
    log_conf.add_variable('stateEstimate.qx', 'float')
    log_conf.add_variable('stateEstimate.qy', 'float')
    log_conf.add_variable('stateEstimate.qz', 'float')
    
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(attitude_callback)
    log_conf.start()
    print(f"Started logging for {scf._link_uri}")


def calculate_angular_velocity(uri):
    """
    Calculate the current angular velocity in degrees per second for a specific Crazyflie.
    Uses recent quaternion data to estimate rotational speed.
    """

    if uri not in quat_data_dict or len(quat_data_dict[uri]) < 2:
        return 0.0
    
    # Get the two most recent data points
    current = quat_data_dict[uri][-1]
    previous = quat_data_dict[uri][-2]
    
    # Calculate time difference
    dt = current['timestamp'] - previous['timestamp']
    if dt <= 0:
        return 0.0
    
    # Convert quaternions to scipy Rotation objects
    curr_quat = current['quaternion']
    prev_quat = previous['quaternion']
    
    # Convert to scipy format [x,y,z,w]
    curr_rot = Rotation.from_quat([curr_quat[1], curr_quat[2], curr_quat[3], curr_quat[0]])
    prev_rot = Rotation.from_quat([prev_quat[1], prev_quat[2], prev_quat[3], prev_quat[0]])
    
    # Calculate relative rotation
    relative_rotation = prev_rot.inv() * curr_rot
    
    # Get rotation vector (axis-angle representation)
    rotation_vector = relative_rotation.as_rotvec()
    
    # Calculate angular velocity magnitude
    angular_displacement_rad = np.linalg.norm(rotation_vector)
    angular_velocity_rad_per_sec = angular_displacement_rad / dt
    angular_velocity_deg_per_sec = np.degrees(angular_velocity_rad_per_sec)
    
    return angular_velocity_deg_per_sec #float

def power_profile(angular_velocity_dps):
    """
    Convert angular velocity to motor power using exponential curve.
    
    Args:
        angular_velocity_dps: Angular velocity in degrees per second
        
    Returns:
        int: Motor power value between min_power and max_power
    """
    # Normalize velocity to 0-1 range
    normalized_velocity = angular_velocity_dps / max_angular_velocity_dps
    
    # Clamp to [0, 1] range
    normalized_velocity = max(0.0, min(1.0, normalized_velocity))
    
    # Apply exponential curve for more responsive feel
    power_ratio = normalized_velocity ** vibration_exponent
    
    # Convert to motor power
    power = int(min_power + (max_power - min_power) * power_ratio)
    
    return power


def power_distribution(scf):
    """
    Calculate motor power based on angular velocity.
    All motors vibrate equally based on how fast the drone is rotating.
    """
    global current_angular_velocity
    
    # Calculate instantaneous angular velocity
    instantaneous_velocity = calculate_angular_velocity(scf._link_uri)
    
    # Apply smoothing to reduce jitter
    current_angular_velocity = (velocity_smoothing_factor * current_angular_velocity + 
                               (1 - velocity_smoothing_factor) * instantaneous_velocity)
    
    # Convert to motor power
    motor_power = power_profile(current_angular_velocity)
    
    # Set all motors to the same power - pure velocity feedback
    m1 = m2 = m3 = m4 = motor_power
    
    # Debug output (reduced frequency to avoid spam)
    if time.time() % 0.2 < 0.05:  # Print every ~200ms
        print(f'URI: {scf._link_uri}, Angular velocity: {current_angular_velocity:.1f}°/s → Motor power: {motor_power}')
    
    # Send commands to all motors
    scf.cf.param.set_value('motorPowerSet.m1', str(m1))
    scf.cf.param.set_value('motorPowerSet.m2', str(m2))
    scf.cf.param.set_value('motorPowerSet.m3', str(m3))
    scf.cf.param.set_value('motorPowerSet.m4', str(m4))


def vibration(scf):
    """
    Main vibration control loop. Angular velocity feedback.
    """
    
    scf.cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(1)

    while execute == True:
        power_distribution(scf)
        time.sleep(.05)  
    
    time.sleep(1)
    # Turn off all motors
    scf.cf.param.set_value('motorPowerSet.m1', '0')
    scf.cf.param.set_value('motorPowerSet.m2', '0')
    scf.cf.param.set_value('motorPowerSet.m3', '0')
    scf.cf.param.set_value('motorPowerSet.m4', '0')

    time.sleep(1)


if __name__ == '__main__':
    """
    Main execution - angular velocity feedback system!
    """
    print("=== ANGULAR VELOCITY VIBRATION ===")
    print("Vibration intensity based on rotational speed!")

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    with Swarm(uris, factory=factory) as swarm:
    # not resetting estimators or arming the crazyflie as it it not flying

        swarm.parallel_safe(start_logging)
        time.sleep(1)

        try: 
            swarm.parallel_safe(vibration)
            time.sleep(10)

        except KeyboardInterrupt:
            print("\n=== STOPPING ALL MOTORS ===")
            execute = False  # Properly stop the vibration lo
            swarm.parallel_safe(vibration)
