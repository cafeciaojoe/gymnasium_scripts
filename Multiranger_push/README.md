# Multiranger Push

The `multiranger_push.py` script enables direct human-drone interaction, allowing the user to push the Crazyflie drone around without physically touching it.
It is taken directly from Bitcraze's [cflib examples](https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/multiranger/multiranger_push.py).

## Hardware requirements
- 1 Crazyflie drone
- 1 Multiranger deck
- 1 Flow deck v2

![](resources/MultirangerPushHardware.JPG)

## How it works
The Crazyflie takes off at a height defined by the user through the `DEFAULT_HEIGHT` parameter.
The Multiranger deck continuously measures the distances in all directions and tries to keep away from anything that comes closer than 0.2m by setting a velocity in the opposite direction.

The script is terminated by placing your hand above the Crazyflie.