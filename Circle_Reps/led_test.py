import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/40/2M/BADF00D004')  # Change to the drone with the LED ring

# Tests the LED ring deck on its own, no motors: checks the deck is found,
# switches it to the solid colour effect, fades pink up and down a few
# times, then turns it off. Press Ctrl+C to stop early.

# ---- Inputs ----
PINK = (255, 105, 180)  # Red, green, blue at full brightness, 0 to 255 each
FADES = 3               # How many times to fade up and down
FADE_TIME = 2.0         # Time for one fade up (and one fade down) [s]
STEPS = 20              # Brightness steps per fade


# Sets all LEDs on the ring to one (red, green, blue) colour
def set_led(cf, colour):
    cf.param.set_value('ring.solidRed', str(colour[0]))
    cf.param.set_value('ring.solidGreen', str(colour[1]))
    cf.param.set_value('ring.solidBlue', str(colour[2]))


# PINK at a brightness from 0 to 1
def pink(brightness):
    return tuple(int(c * brightness) for c in PINK)


if __name__ == '__main__':
    cflib.crtp.init_drivers()  # Start the radio

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # 1 = the LED ring deck was found when the drone started up
        print('LED ring deck found:', cf.param.get_value('deck.bcLedRing'))

        # Switch to solid colour (effect 7) and read it back
        set_led(cf, (0, 0, 0))
        cf.param.set_value('ring.effect', '7')
        time.sleep(0.5)
        print('ring.effect is now:', cf.param.get_value('ring.effect'), '(should be 7)')

        try:
            for i in range(FADES):
                print(f'Fade {i + 1}/{FADES}')
                # Up from dark to full, then back down
                levels = [s / STEPS for s in range(STEPS + 1)]
                for brightness in levels + levels[::-1]:
                    set_led(cf, pink(brightness))
                    time.sleep(FADE_TIME / STEPS)
        except KeyboardInterrupt:
            print('Stopped')

        set_led(cf, (0, 0, 0))
        time.sleep(0.5)
        print('Done, LEDs off')
