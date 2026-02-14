# Flight Path Interactive

This script allows users to define a flight path for a Crazyflie drone interactively. The user can control the z-velocity dynamically using a sensor while the drone follows a predefined path in the x and y directions. The script also provides a graphical interface for plotting the flight path before execution.

## Features
- **Interactive Path Definition:**
  - Use mouse clicks to define waypoints for the drone.
  - Left-click to add waypoints and right-click to finish the path.
- **3D Visualization:**
  - Visualize the defined flight path in a 3D plot before execution.
- **Dynamic Altitude Control:**
  - The z-velocity is dynamically adjusted based on sensor input.
- **Data Logging:**
  - Logs the x, y, z coordinates and durations between waypoints to a CSV file.

## Requirements
- 1 Crazyflie drone (for flying)
- 1 Crazyflie drone (for sensing)
- 2 Lighthouse positioning decks
- 1 Buzzer deck

## Usage
1. Use the mouse to define the flight path:
   - Left-click to add waypoints.
   - Right-click to finish and save the path.
2. Review the 3D plot of the flight path.
3. The drone will execute the flight path, dynamically adjusting its altitude based on sensor input.
