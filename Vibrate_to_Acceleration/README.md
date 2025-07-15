# Acceleration-Based Vibration Control

The purpose of `vibe_to_acceleration.py` is to control the vibration intensity of multiple Crazyflie drones based on their acceleration. This script uses data from the drones to calculate acceleration and adjust motor power accordingly.

## Hardware Requirements
- At least 1 Crazyflie drone, no decks required. We recommend replacing the motors with vibration motors or at least replacing the propellers with a small counterweight for each shaft.

![](resources/vibration_motors_1.jpg)

## How It Works
The script connects to multiple Crazyflie drones and logs their acceleration data in real-time. Using this data, it calculates the mean acceleration of each drone and adjusts the motor power to simulate vibration intensity. The vibration intensity is proportional to the acceleration, creating a feedback loop based on movement.

### Adjustable Parameters
- `max_power`: works between 1000 and 60000
- `samples`: Number of acceleration samples used for smoothing (default: 4).
- `invert`: If `True`, higher acceleration results in lower motor power.

### Termination
The script can be terminated by pressing `Ctrl+C`. All motors are automatically turned off when the script exits.

### Bandwidth
The log_period might need to be lengthened if you add so many crazyflies that you exceed the bandwidth of the radio. The log_period will also affect the responsiveness of the vibration. 

## Visualization
Future updates will include plots to visualize the vibration function and acceleration data for better understanding