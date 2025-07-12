import time
import numpy as np
from scipy.spatial.transform import Rotation

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.swarm import Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

######################### PLAY WITH THESE NUMBERS ##################################

# Max motor power 60000
max_power = 20000  

# if invert is true then more acceleration makes less vibration, being still produces max power.
invert = False

# Handy for tuning values when connected to one crazyflie. 
printing = False

# the required Angular velocity that produces max power
max_angular_velocity_dps = 400  # (degrees per second)

# Smoothing, more samples, smoother and more laggy response
samples = 4

####################################################################################

# Connection URI for the Crazyflie
uris = [
'radio://0/30/2M/a0a0a0a0aa',
'radio://0/30/2M/a0a0a0a0ae',
'radio://0/30/2M/e7e7e7e7e8'
]

# Global dictionary to store quaternion data for each Crazyflie
quat_data_dict = {}

# Response curve, 1 = linear response, 
vibration_exponent = 1 

# TODO FIND LOG PERIOD THAT SUITS THE BANDWIDTH, 4 DRONES 
log_period = 40 #ms

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

    # Ensure each Crazyflie has its own key/list in the dict
    if uri not in quat_data_dict:
        quat_data_dict[uri] = []

    #add the data to the list and keep it the length of samples specified at the top of the script. 
    quat_data_dict[uri].append({
        'timestamp': timestamp / 1000.0,  # Convert milliseconds to seconds
        'quaternion': quat
    })
    if len(quat_data_dict[uri]) > samples:
        quat_data_dict[uri].pop(0)

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

def calculate_average_angular_velocity(uri):
    """
    Calculate the average angular velocity in degrees per second for a specific Crazyflie.
    Uses all quaternion data in the deque to estimate rotational speed.
    """
    if uri not in quat_data_dict or len(quat_data_dict[uri]) < 2:
        return 0.0

    angular_velocities = []
    for i in range(1, len(quat_data_dict[uri])):
        current = quat_data_dict[uri][i]
        previous = quat_data_dict[uri][i - 1]

        # Calculate time difference
        dt = current['timestamp'] - previous['timestamp']
        if dt <= 0:
            continue

        # Convert quaternions to scipy Rotation objects
        curr_quat = current['quaternion']
        prev_quat = previous['quaternion']

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

        angular_velocities.append(angular_velocity_deg_per_sec)

    # Return the average angular velocity
    return np.mean(angular_velocities) if angular_velocities else 0.0

def power_profile(angular_velocity_dps):
    """
    Convert angular velocity to motor power using exponential curve.
    If invert is True, higher angular velocity results in lower motor power.
    
    Args:
        angular_velocity_dps: Angular velocity in degrees per second
        
    Returns:
        int: Motor power value below max_power
    """
    # Normalize velocity to 0-1 range
    normalized_velocity = angular_velocity_dps / max_angular_velocity_dps
    
    # Clamp to [0, 1] range
    normalized_velocity = max(0.0, min(1.0, normalized_velocity))
    
    # Apply exponential curve for more responsive feel
    power_ratio = normalized_velocity ** vibration_exponent
    
    if invert:
        # Invert the power calculation: less movement → higher power
        power_ratio = 1 - power_ratio
    
    # Convert to motor power
    power = int((max_power) * power_ratio)
    
    return power

def power_distribution(scf):
    """
    Calculate motor power based on average angular velocity.
    All motors vibrate equally based on how fast the drone is rotating.
    """
    # Calculate average angular velocity
    average_velocity = calculate_average_angular_velocity(scf._link_uri)

    # Convert to motor power
    motor_power = power_profile(average_velocity)

    # Set all motors to the same power - pure velocity feedback
    m1 = m2 = m3 = m4 = motor_power

    # Monitor output
    if printing == True:
        print(f'URI: {scf._link_uri}, Average angular velocity: {average_velocity:.1f}°/s → Motor power: {motor_power}')

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
    if execute == True:
        print(f'Ready to vibrate! ({scf._link_uri})')

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

def filter_uris(uris):
    valid_uris = []
    for uri in uris:
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print(f"Successfully connected to {uri}")
                valid_uris.append(uri)
        except Exception as e:
            print(f"Failed to connect to {uri}: {e}")
    return valid_uris

if __name__ == '__main__':
    print("=== ANGULAR VELOCITY VIBRATION ===")
    print("Vibration intensity based on rotational speed!")

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    # Filter URIs to only include valid connections
    valid_uris = filter_uris(uris)

    if not valid_uris:
        print("No valid Crazyflie connections found. Exiting.")
        exit()

    #TODO add a plot of the vibration funciton here

    with Swarm(valid_uris, factory=factory) as swarm:
    # not resetting estimators or arming the crazyflie as it it not flying

        swarm.parallel_safe(start_logging)
        time.sleep(1)

        try: 
            swarm.parallel_safe(vibration)
            time.sleep(10)

        except KeyboardInterrupt:
            print("\n=== STOPPING ALL MOTORS ===")
            execute = False 
            swarm.parallel_safe(vibration)

    #TODO add a plot of the movements to show at the end. 