# Keep Up

This folder contains two scripts designed to interact with Crazyflie drones, allowing users to "keep up" the drones by placing their hand on top of the flying drone to prevent it from falling and to raise it up. These scripts utilize the Multiranger deck for proximity sensing.

## Scripts

### `keep_up.py`
This script controls a single Crazyflie drone. The drone hovers at a default height and adjusts its position based on proximity sensor readings. When a hand is placed above the drone, it detects the obstacle and increases its thrust to rise. If no obstacle is detected, the drone gradually lowers its altitude. The script also includes basic collision avoidance for the front, back, left, and right directions.

### `keep_up_swarm.py`
This script extends the functionality to a swarm of Crazyflie drones, operating simultaneously.

## Requirements
- Crazyflie drone(s) 
- Flow deck(s) V2
- Multiranger deck(s)
- Buzzer deck(s)

## Usage
1. Run `keep_up.py` for a single drone or `keep_up_swarm.py` for multiple drones.
2. Place your hand above the drone(s) to interact with their altitude control.
