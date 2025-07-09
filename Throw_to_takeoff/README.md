# Throw to take off

The `throw_to_takeoff.py` script aims to enable human-drone interaction,even for simple tasks.
It's pretty similar to `drop_to_takeoff.py`.
Specifically, if the Crazyflie is released with 0 vertical speed, both scripts have the exact same behavior.


## Hardware requirements
- 1 Crazyflie drone preferably with the thrust upgrade kit
- 1 Lighthouse positioning deck or a Flow deck v2

![](resources/ThrowToTakeoffHardware.JPG)

## How it works

The user holds the Crazyflie and throws it with a vertical velocity.
The Crazyflie has to be as leveled as possible.
It will then perform a vertical free throw, taking off just before it begins to fall back down.
It will stabilize at that position and land shortly afterward.

The way this script works, is by constantly logging the acceleration and the velocity of the Crazyflie on the z-axis.
When the Crazyflie is on the ground or being carried around, the acceleration is approximately 1g upwards.
During free fall, acceleration approaches 0.
To activate the motors, the acceleration and the velocity have to be close to 0.

ToDo: Add figure
