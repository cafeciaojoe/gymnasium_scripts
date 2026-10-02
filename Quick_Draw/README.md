# Quick Draw

The `quick_draw.py` script is a two-player reaction game. Each player holds a Crazyflie as a "sensor" and waits for the flying drone to give the signal. Whoever draws (shakes their Crazyflie) first wins the round and pulls the flying drone towards their side.

## Features
- **Reaction Game:**
  - After a random delay (1-10 s), the flying drone's LED ring turns green to signal "draw!".
  - The first player whose Crazyflie exceeds the acceleration threshold wins the round.
- **Scoring:**
  - The drone moves 0.5 m towards the round's winner (along the x-axis).
  - The first player to get 3 points ahead of the other wins the game.
- **LED Feedback:**
  - Player 1's sensor is lit blue and Player 2's sensor is lit red.
  - The flying drone shows the round winner's color before moving.
  - When the game is over, the drone lands and the winner's sensor shows a light effect while the loser's ring turns off.

## Requirements
- 1 Crazyflie drone (for flying)
- 2 Crazyflie drones (as handheld sensors)
- 3 LED ring decks
- 1 Positioning deck for the flying drone (e.g. Flow deck v2 or Lighthouse deck)

## Usage
1. Set the radio addresses of the two sensors (`SENSOR1`, `SENSOR2`) and the flying drone (`URI`) in the script.
2. Place the flying drone between the two players, with Player 1 (blue) in the positive x-direction and Player 2 (red) in the negative x-direction.
3. Run `quick_draw.py`. The drone takes off and the game starts.
4. Wait for the drone to turn green, then shake your Crazyflie as fast as you can.
