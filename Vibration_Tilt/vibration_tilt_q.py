import time
import threading
import math
import numpy as np
from scipy.spatial.transform import Rotation

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Connection URI for the Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/30/2M/a0a0a0a0aa')

# Quaternion data from Crazyflie
quat_w = [1.0]  # Quaternion w component (scalar part)
quat_x = [0.0]  # Quaternion x component 
quat_y = [0.0]  # Quaternion y component
quat_z = [0.0]  # Quaternion z component

# Reference quaternion (set when user presses Enter)
start_quaternion = None

# Motor power settings
min_power = 7000   # Minimum motor power (drone barely responds)
max_power = 45000  # Maximum motor power (strong vibration)

# Vibration response curve
vibration_exponent = 2.5  # Controls how aggressively vibration increases
                          # Higher = more gentle at small angles, aggressive at large
max_rotation_degrees = 180  # Maximum expected rotation (180° = upside down)


def attitude_callback(timestamp, data, logconf):
    """
    Callback function that receives quaternion data from Crazyflie.
    Stores the clean quaternion data for orientation calculations.
    """
    quat_w.append(data['stateEstimate.qw'])  # Scalar component
    quat_x.append(data['stateEstimate.qx'])  # X component  
    quat_y.append(data['stateEstimate.qy'])  # Y component
    quat_z.append(data['stateEstimate.qz'])  # Z component


def start_position_printing(scf):
    """
    Set up logging to receive quaternion data from Crazyflie.
    """
    log_conf = LogConfig(name='Quaternion_Attitude', period_in_ms=100)  # 10 Hz
    
    # Add quaternion variables to logging
    log_conf.add_variable('stateEstimate.qw', 'float')
    log_conf.add_variable('stateEstimate.qx', 'float')
    log_conf.add_variable('stateEstimate.qy', 'float')
    log_conf.add_variable('stateEstimate.qz', 'float')
    
    scf.cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(attitude_callback)
    log_conf.start()
    print("Started quaternion logging - pure rotational displacement feedback!")


def calculate_total_rotation_angle(current_quat, reference_quat):
    """
    Calculate the total rotational angle between current and reference orientations.
    This gives us the magnitude of rotation regardless of axis - pure displacement.
    
    Args:
        current_quat: [w, x, y, z] current quaternion from drone
        reference_quat: [w, x, y, z] reference quaternion (start position)
        
    Returns:
        float: Total rotation angle in degrees (always positive, 0° to 180°)
               - 0° = exactly at start position
               - 90° = significant rotation from start
               - 180° = maximum possible rotation (upside down relative to start)
    """
    # Convert to scipy Rotation objects (scipy uses [x,y,z,w] format)
    ref_rot = Rotation.from_quat([reference_quat[1], reference_quat[2], 
                                  reference_quat[3], reference_quat[0]])
    curr_rot = Rotation.from_quat([current_quat[1], current_quat[2], 
                                   current_quat[3], current_quat[0]])
    
    # Calculate relative rotation quaternion
    relative_rotation = ref_rot.inv() * curr_rot
    
    # Get rotation vector - magnitude is the total angle
    rotation_vector = relative_rotation.as_rotvec()
    total_angle_radians = np.linalg.norm(rotation_vector)
    
    # Convert to degrees
    total_angle_degrees = np.degrees(total_angle_radians)
    
    return total_angle_degrees


def power_profile(total_rotation_angle):
    """
    Convert total rotational displacement to motor power using exponential curve.
    All motors get the same power - pure magnitude feedback.
    
    Args:
        total_rotation_angle: Total rotation angle in degrees (0° to 180°)
        
    Returns:
        int: Motor power value between min_power and max_power
    """
    # Normalize angle to 0-1 range (0° to 180°)
    normalized_angle = total_rotation_angle / max_rotation_degrees
    
    # Clamp to [0, 1] range
    normalized_angle = max(0.0, min(1.0, normalized_angle))
    
    # Apply exponential curve
    power_ratio = normalized_angle ** vibration_exponent
    
    # Convert to motor power
    power = int(min_power + (max_power - min_power) * power_ratio)
    
    return power


def power_distribution():
    """
    Calculate motor power based on total rotational displacement.
    Simple and elegant - all motors vibrate equally based on how far
    the drone has rotated from its starting orientation.
    """
    global start_quaternion
    
    # Safety check
    if start_quaternion is None:
        return
    
    # Get current quaternion
    current_quaternion = [quat_w[-1], quat_x[-1], quat_y[-1], quat_z[-1]]
    
    # Calculate total rotation angle (0° to 180°)
    total_rotation = calculate_total_rotation_angle(current_quaternion, start_quaternion)
    
    # Convert to motor power
    motor_power = power_profile(total_rotation)
    
    # Set all motors to the same power - pure magnitude feedback
    m1 = m2 = m3 = m4 = motor_power
    
    # Debug output
    print(f'Total rotation: {total_rotation:.1f}° → Motor power: {motor_power}')
    
    # Send commands to all motors
    scf.cf.param.set_value('motorPowerSet.m1', str(m1))
    scf.cf.param.set_value('motorPowerSet.m2', str(m2))
    scf.cf.param.set_value('motorPowerSet.m3', str(m3))
    scf.cf.param.set_value('motorPowerSet.m4', str(m4))


def vibration(scf):
    """
    Main vibration control loop. Pure rotational displacement feedback.
    """
    print("=== TOTAL ROTATION VIBRATION CONTROL ===")
    print("All motors vibrate equally based on total rotation from start pose!")
    print("Press Ctrl+C to stop...")
    
    try:
        while True:
            if start_quaternion is not None:
                power_distribution()
            else:
                print("Waiting for start position...")
            
            time.sleep(0.05)  # 20 Hz control loop
            
    except KeyboardInterrupt:
        print("\n=== STOPPING VIBRATION CONTROL ===")
        
        # Turn off all motors
        scf.cf.param.set_value('motorPowerSet.m1', '0')
        scf.cf.param.set_value('motorPowerSet.m2', '0')
        scf.cf.param.set_value('motorPowerSet.m3', '0')
        scf.cf.param.set_value('motorPowerSet.m4', '0')
        
        print("All motors stopped. Safe to disconnect.")


def get_start_pose():
    """
    Capture the current quaternion as the reference position.
    """
    global start_quaternion
    
    input("Press Enter to set start pose...")
    
    start_quaternion = [quat_w[-1], quat_x[-1], quat_y[-1], quat_z[-1]]
    
    print(f"Start pose set! Ready for total rotation feedback.")
    print("Rotate the drone in ANY direction to feel proportional vibration!")


if __name__ == '__main__':
    """
    Main execution - much simpler than directional version!
    """
    print("=== QUATERNION TOTAL ROTATION VIBRATION ===")
    print("Pure magnitude feedback - no directional complexity!")
    
    cflib.crtp.init_drivers()
    
    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            print(f"Connected to Crazyflie at {URI}")
            
            # Start logging
            start_position_printing(scf)
            time.sleep(2)
            
            # Set reference
            get_start_pose()
            
            # Start vibration control
            vibration(scf)
            
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        print("Program ended.")