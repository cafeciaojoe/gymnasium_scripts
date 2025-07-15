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

"""
work in progress, potentialally plagued by this issue with the crazyflie.
https://github.com/bitcraze/crazyflie-firmware/issues/1292
"""

# Connection URI for the Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/30/2M/a0a0a0a0aa')
#URI = uri_helper.uri_from_env(default='usb://0')

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
vibration_exponent = 1  # Controls how aggressively vibration increases
max_rotation_degrees = 90  # Maximum expected roll/pitch (90° = completely tilted)


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
    print("Started quaternion logging - roll/pitch displacement feedback!")


def calculate_roll_pitch_displacement(current_quat, reference_quat):
    """
    Calculate roll and pitch displacement from reference orientation.
    
    Args:
        current_quat: [w, x, y, z] current quaternion from drone
        reference_quat: [w, x, y, z] reference quaternion (start position)
        
    Returns:
        tuple: (roll_displacement, pitch_displacement) in degrees
               Both values are absolute (always positive)
    """
    # Convert to scipy Rotation objects (scipy uses [x,y,z,w] format)
    ref_rot = Rotation.from_quat([reference_quat[1], reference_quat[2], 
                                  reference_quat[3], reference_quat[0]])
    curr_rot = Rotation.from_quat([current_quat[1], current_quat[2], 
                                   current_quat[3], current_quat[0]])
    
    # Calculate relative rotation
    relative_rotation = ref_rot.inv() * curr_rot
    
    # Convert to Euler angles (roll, pitch, yaw)
    euler_angles = relative_rotation.as_euler('xyz', degrees=True)
    roll_displacement = abs(euler_angles[0])    # X-axis rotation (roll)
    pitch_displacement = abs(euler_angles[1])   # Y-axis rotation (pitch)
    
    return roll_displacement, pitch_displacement


def power_component(displacement_degrees):
    """
    Convert displacement angle to power component using exponential curve.
    
    Args:
        displacement_degrees: Displacement angle in degrees (0° to 90°)
        
    Returns:
        float: Power component between 0.0 and 1.0
    """
    # Normalize angle to 0-1 range (0° to 90°)
    normalized_angle = displacement_degrees / max_rotation_degrees
    
    # Clamp to [0, 1] range
    normalized_angle = max(0.0, min(1.0, normalized_angle))
    
    # Apply exponential curve
    power_component = normalized_angle ** vibration_exponent
    
    return power_component


def power_distribution():
    """
    Calculate motor power based on roll and pitch displacement components.
    Each motor gets power = base_power + (roll_component + pitch_component) * power_range / 2
    This means max power is achieved when both roll AND pitch are at maximum.
    """
    global start_quaternion
    
    # Safety check
    if start_quaternion is None:
        return
    
    # Get current quaternion
    current_quaternion = [quat_w[-1], quat_x[-1], quat_y[-1], quat_z[-1]]
    
    # Calculate roll and pitch displacements
    roll_disp, pitch_disp = calculate_roll_pitch_displacement(current_quaternion, start_quaternion)
    
    # Convert to power components (0.0 to 1.0)
    roll_component = power_component(roll_disp)
    pitch_component = power_component(pitch_disp)
    
    # Combine components - each contributes half of the total power range
    # This means you need BOTH roll AND pitch at max to get maximum power
    total_component = (roll_component + pitch_component) / 2.0
    
    # Convert to motor power
    power_range = max_power - min_power
    motor_power = int(min_power + power_range * total_component)
    
    # Set all motors to the same power
    m1 = m2 = m3 = m4 = motor_power
    
    # Debug output
    print(f'Roll: {roll_disp:.1f}° ({roll_component:.2f}) | Pitch: {pitch_disp:.1f}° ({pitch_component:.2f}) | Total: {total_component:.2f} → Power: {motor_power}')
    
    # Send commands to all motors
    scf.cf.param.set_value('motorPowerSet.m1', str(m1))
    scf.cf.param.set_value('motorPowerSet.m2', str(m2))
    scf.cf.param.set_value('motorPowerSet.m3', str(m3))
    scf.cf.param.set_value('motorPowerSet.m4', str(m4))


def vibration(scf):
    """
    Main vibration control loop. Roll/pitch displacement feedback.
    """
    print("=== ROLL/PITCH DISPLACEMENT VIBRATION CONTROL ===")
    print("Motor power = (roll_component + pitch_component) / 2")
    print("Maximum power requires BOTH roll AND pitch displacement!")
    print("Press Ctrl+C to stop or reset pose...")
    
    scf.cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(1)

    running = True
    
    while running:
        try:
            while True:
                if start_quaternion is not None:
                    power_distribution()
                else:
                    print("Waiting for start position...")
                
                time.sleep(0.05)  # 20 Hz control loop
                
        except KeyboardInterrupt:
            print("\n=== PAUSED ===")
            print("Options:")
            print("Press Enter to reset pose and continue")
            print("Type 'q' + Enter to quit")
            
            try:
                user_input = input("Choice: ").strip().lower()
                if user_input == 'q':
                    running = False
                else:
                    get_start_pose()
                    print("Pose reset! Continuing...")
            except KeyboardInterrupt:
                print("\nForce quit detected...")
                running = False
    
    # Cleanup - always runs when exiting
    print("\n=== STOPPING VIBRATION CONTROL ===")
    
    # Turn off all motors
    scf.cf.param.set_value('motorPowerSet.m1', '0')
    scf.cf.param.set_value('motorPowerSet.m2', '0')
    scf.cf.param.set_value('motorPowerSet.m3', '0')
    scf.cf.param.set_value('motorPowerSet.m4', '0')

    time.sleep(1)

    scf.cf.param.set_value('motorPowerSet.enable', '0')
    time.sleep(1)
    
    print("All motors stopped. Safe to disconnect.")


def get_start_pose():
    """
    Capture the current quaternion as the reference position.
    """
    global start_quaternion
    
    start_quaternion = [quat_w[-1], quat_x[-1], quat_y[-1], quat_z[-1]]
    
    print(f"Start pose set! Ready for roll/pitch displacement feedback.")


if __name__ == '__main__':
    """
    Main execution - roll/pitch component-based vibration.
    """
    print("=== QUATERNION ROLL/PITCH COMPONENT VIBRATION ===")
    print("Power = (roll_displacement + pitch_displacement) / 2")
    print("Maximum vibration requires BOTH axes displaced!")
    

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    
    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            print(f"Connected to Crazyflie at {URI}")
            
            # Start logging
            start_position_printing(scf)
            time.sleep(2)
            
            input("Press Enter to set start pose...")
            # Set reference
            get_start_pose()
            
            # Start vibration control
            vibration(scf)
            
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        print("Program ended.")
