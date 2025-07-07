import time
import threading
import math  # For basic math operations

import matplotlib.pyplot as plt
import numpy as np  # For array operations
from scipy.spatial.transform import Rotation  # For quaternion math

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Connection URI for the Crazyflie
URI = uri_helper.uri_from_env(default='radio://0/30/2M/a0a0a0a0aa')
#URI = uri_helper.uri_from_env(default='usb://0')

# Quaternion data from Crazyflie (the clean solution!)
quat_w = [1.0]  # Quaternion w component (scalar part)
quat_x = [0.0]  # Quaternion x component 
quat_y = [0.0]  # Quaternion y component
quat_z = [0.0]  # Quaternion z component

# Reference quaternion (set when user presses Enter)
start_quaternion = None

# Motor power settings
min_power = 5000   # Minimum motor power (drone barely responds)
max_power = 45000  # Maximum motor power (strong vibration)
min_angle = 0      # Minimum angle before motors activate
max_angle = 90     # Maximum angle for full power response

def attitude_callback(timestamp, data, logconf):
    """
    Callback function that receives quaternion data from Crazyflie.
    This replaces the old roll/pitch logging with clean quaternion data.
    
    Args:
        timestamp: Time when data was received
        data: Dictionary containing quaternion components
        logconf: Log configuration object (unused)
    """
    # Store quaternion components from Crazyflie
    quat_w.append(data['stateEstimate.qw'])  # Scalar component
    quat_x.append(data['stateEstimate.qx'])  # X component  
    quat_y.append(data['stateEstimate.qy'])  # Y component
    quat_z.append(data['stateEstimate.qz'])  # Z component
    
    # Optional: Print for debugging (comment out for production)
    # print(f"Quat: w={data['stateEstimate.qw']:.3f}, x={data['stateEstimate.qx']:.3f}, "
    #       f"y={data['stateEstimate.qy']:.3f}, z={data['stateEstimate.qz']:.3f}")


def start_position_printing(scf):
    """
    Set up logging to receive quaternion data from Crazyflie.
    This configures the drone to send us orientation data 10 times per second.
    
    Args:
        scf: SyncCrazyflie object for communicating with drone
    """
    # Create logging configuration for quaternion data
    log_conf = LogConfig(name='Quaternion_Attitude', period_in_ms=10)  # 10 Hz updates
    
    # Add quaternion variables to logging (these are the key change!)
    log_conf.add_variable('stateEstimate.qw', 'float')  # Quaternion w (scalar)
    log_conf.add_variable('stateEstimate.qx', 'float')  # Quaternion x 
    log_conf.add_variable('stateEstimate.qy', 'float')  # Quaternion y
    log_conf.add_variable('stateEstimate.qz', 'float')  # Quaternion z
    
    # Register the logging configuration with the Crazyflie
    scf.cf.log.add_config(log_conf)
    
    # Set callback function to receive data
    log_conf.data_received_cb.add_callback(attitude_callback)
    
    # Start logging quaternion data
    log_conf.start()

def quaternion_to_orientation_angles(current_quat, reference_quat):
    """
    Convert quaternion difference to pitch/roll angles relative to reference.
    
    Args:
        current_quat: [w, x, y, z] current quaternion from drone
        reference_quat: [w, x, y, z] reference quaternion (start position)
        
    Returns:
        tuple: (pitch_diff, roll_diff) in degrees
               - pitch_diff: + = nose up, - = nose down
               - roll_diff: + = right tilt, - = left tilt  
    """
    # Convert to scipy Rotation objects (note: scipy uses [x,y,z,w] format)
    ref_rot = Rotation.from_quat([reference_quat[1], reference_quat[2], 
                                  reference_quat[3], reference_quat[0]])
    curr_rot = Rotation.from_quat([current_quat[1], current_quat[2], 
                                   current_quat[3], current_quat[0]])
    
    # Calculate relative rotation: how much has drone rotated from start?
    relative_rotation = ref_rot.inv() * curr_rot
    
    # Convert to Euler angles (roll, pitch) in degrees
    # Using 'xyz' order which matches aircraft convention
    euler_angles = relative_rotation.as_euler('xyz', degrees=True)
    
    # Extract individual angles
    roll_diff = euler_angles[0]   # Rotation around X-axis (left/right tilt)
    pitch_diff = euler_angles[1]  # Rotation around Y-axis (nose up/down)
    
    return pitch_diff, roll_diff


def get_start_pose():
    """
    Capture the current quaternion as the reference position.
    User presses Enter when drone is in desired starting orientation.
    """
    global start_quaternion
    
    input("Press Enter to set start pose...")
    
    # Capture current quaternion as reference
    start_quaternion = [quat_w[-1], quat_x[-1], quat_y[-1], quat_z[-1]]
    
    print(f"Start pose set! quaternion = [{start_quaternion[0]:.3f}, {start_quaternion[1]:.3f}, "
          f"{start_quaternion[2]:.3f}, {start_quaternion[3]:.3f}]")
    print("Ready for vibration control - tilt the drone!")

def power_profile(angle):
    """
    Convert tilt angle to motor power using exponential curve.
    This creates stronger vibration for larger tilts.
    
    Args:
        angle: Absolute angle difference in degrees (always positive)
        
    Returns:
        int: Motor power value between min_power and max_power
    """
    # Clamp angle to maximum to prevent excessive power
    if abs(angle) > max_angle:
        power = int(max_power)
    else:
        # Exponential power curve: starts gentle, ramps up quickly
        # Higher exponent = more aggressive response to larger tilts
        exponent = 2.5  # Adjust this value: higher = later peaking
        power = int(min_power + (max_power - min_power) * (abs(angle) / max_angle) ** exponent)
    
    return power
    
def power_distribution():
    """
    Calculate motor powers based on drone orientation using quaternions.
    This is the heart of the vibration system - converts tilt to motor response.
    
    Motor Layout (looking down at drone):
        M4 ← → M1
        ↑      ↑
        M3 ← → M2
    
    Control Logic:
    - Pitch forward (nose down): Activate M1, M4 (front motors)
    - Pitch backward (nose up): Activate M2, M3 (back motors) 
    - Roll right: Activate M1, M2 (right motors)
    - Roll left: Activate M3, M4 (left motors)
    """
    global start_quaternion
    
    # Safety check: make sure we have a reference position
    if start_quaternion is None:
        print("No start position set! Press Enter to set reference.")
        return
    
    # Get current quaternion from latest data
    current_quaternion = [quat_w[-1], quat_x[-1], quat_y[-1], quat_z[-1]]
    
    # Convert quaternions to clean pitch/roll differences (no gimbal lock!)
    pitch_diff, roll_diff = quaternion_to_orientation_angles(
        current_quaternion, start_quaternion)
    
    # Initialize motor power contributions for each axis
    # Each motor gets contributions from pitch (p), roll (r)
    m1_p = 0
    m2_p = 0
    m3_p = 0
    m4_p = 0
    m1_r = 0
    m2_r = 0
    m3_r = 0
    m4_r = 0
    
    # PITCH CONTROL: Forward/backward tilt
    if pitch_diff < 0:  # Pitched forward (nose down)
        m1_p = power_profile(abs(pitch_diff))  # Front-right motor
        m4_p = power_profile(abs(pitch_diff))  # Front-left motor
    elif pitch_diff > 0:  # Pitched backward (nose up)
        m2_p = power_profile(abs(pitch_diff))  # Back-right motor
        m3_p = power_profile(abs(pitch_diff))  # Back-left motor
    
    # ROLL CONTROL: Left/right tilt
    if roll_diff > 0:  # Rolled right
        m1_r = power_profile(abs(roll_diff))  # Right-front motor
        m2_r = power_profile(abs(roll_diff))  # Right-back motor
    elif roll_diff < 0:  # Rolled left
        m3_r = power_profile(abs(roll_diff))  # Left-back motor
        m4_r = power_profile(abs(roll_diff))  # Left-front motor
    
    # Combine all contributions and clamp to max_power
    m1 = min(m1_p + m1_r, max_power)
    m2 = min(m2_p + m2_r, max_power)
    m3 = min(m3_p + m3_r, max_power)
    m4 = min(m4_p + m4_r, max_power)
    
    # Debug output: show current differences and motor powers
    print(f'Orientation: pitch={pitch_diff:.1f}°, roll={roll_diff:.1f}°')
    print(f'Motors: M1={m1}, M2={m2}, M3={m3}, M4={m4}')
    
    # Send motor commands to Crazyflie
    scf.cf.param.set_value('motorPowerSet.m1', str(m1))
    scf.cf.param.set_value('motorPowerSet.m2', str(m2))
    scf.cf.param.set_value('motorPowerSet.m3', str(m3))
    scf.cf.param.set_value('motorPowerSet.m4', str(m4))

def vibration(scf):
    """
    Main vibration control loop. Continuously monitors drone orientation
    and adjusts motor power to create vibration feedback.
    
    Args:
        scf: SyncCrazyflie object for drone communication
    """
    scf.cf.param.set_value('motorPowerSet.enable', '1')
    time.sleep(1)

    print("Vibration started. Press Ctrl+C to stop...")

    try:
        while True:
            power_distribution()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping vibration...")

    scf.cf.param.set_value('motorPowerSet.m1', 0)
    scf.cf.param.set_value('motorPowerSet.m2', 0)
    scf.cf.param.set_value('motorPowerSet.m3', 0)
    scf.cf.param.set_value('motorPowerSet.m4', 0)
    time.sleep(0.5)
    scf.cf.param.set_value('motorPowerSet.enable', '0')
    time.sleep(1)

def plot_power_profile():
    points = [
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 4 roll
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 1 roll
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 3 roll
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 2 roll
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 4 pitch
        [(-max_angle, max_power), (min_angle, min_power), (max_angle, min_power)],  # Motor 1 pitch
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 3 pitch
        [(-max_angle, min_power), (min_angle, min_power), (max_angle, max_power)],  # Motor 2 pitch
    ]
    titles = ['Motor 4', 'Motor 1', 'Motor 3', 'Motor 2']
    y_labels = ['M4 power', 'M1 power', 'M3 power', 'M2 power']

    fig1, axs1 = plt.subplots(2, 2, figsize=(10, 8))

    for i, ax in enumerate(axs1.flat):
        x_vals, y_vals = zip(*points[i])
        ax.plot(x_vals, y_vals, marker='o')
        ax.set_title(titles[i])
        ax.set_xlabel('Roll [deg]')
        ax.set_ylabel(y_labels[i])
        ax.grid(True)

    fig1.tight_layout()

    fig2, axs2 = plt.subplots(2, 2, figsize=(10, 8))

    for i, ax in enumerate(axs2.flat):
        x_vals, y_vals = zip(*points[i+4])
        ax.plot(x_vals, y_vals, marker='o', color='orange')
        ax.set_title(titles[i])
        ax.set_xlabel('Pitch [deg]')
        ax.set_ylabel(y_labels[1])
        ax.grid(True)

    fig2.tight_layout()
    print('Close the graphs to start...')
    plt.show()

def get_start_pose():
    """
    Capture the current quaternion as the reference position.
    User presses Enter when drone is in desired starting orientation.
    """
    global start_quaternion
    
    input("Press Enter to set start pose...")
    
    # Capture current quaternion as reference
    start_quaternion = [quat_w[-1], quat_x[-1], quat_y[-1], quat_z[-1]]
    
    print(f"Start pose set! quaternion = [{start_quaternion[0]:.3f}, {start_quaternion[1]:.3f}, "
          f"{start_quaternion[2]:.3f}, {start_quaternion[3]:.3f}]")
    print("Ready for vibration control - tilt the drone!")

if __name__ == '__main__':
    """
    Main execution: Connect to drone, start logging, set reference, begin vibration control.
    """
    
    # Initialize Crazyflie drivers
    cflib.crtp.init_drivers()
    
    # Create cached factory for drone connection (improves connection reliability)
    factory = CachedCfFactory(rw_cache='./cache')

    # Connect to Crazyflie drone
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print(f"Connected to Crazyflie at {URI}")
        #plot_power_profile()
        # Start quaternion data logging (10 Hz)
        start_position_printing(scf)
        time.sleep(1)
        get_start_pose()
        # Start the main vibration control loop
        vibration(scf)
        
