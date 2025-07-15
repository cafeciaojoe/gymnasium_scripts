import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.localization import LighthouseConfigWriter
from cflib.utils import uri_helper
from cflib.utils.power_switch import PowerSwitch


uris = [
    'radio://0/80/2M/E7E7E7E7E7',
    'radio://0/80/2M/E7E7E7E7E8',
    'radio://0/80/2M/E7E7E7E7E9',
    'radio://0/80/2M/E7E7E7E7EA',
    'radio://0/80/2M/E7E7E7E7EB',
    'radio://0/80/2M/E7E7E7E7EC',
]


def write_one(file_name, scf):
    print(f'Writing to \033[92m{uri}\033[97m...', end='', flush=True)
    writer = LighthouseConfigWriter(scf.cf)
    writer.write_and_store_config_from_file(None, file_name)
    print('Success!')
    time.sleep(1)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        raise ValueError('File name missing')
    file_name = sys.argv[1]
    print(f"Using file {file_name}")
    cflib.crtp.init_drivers()

    for uri in uris:
        try:
            Drone = uri_helper.uri_from_env(default=uri)
            with SyncCrazyflie(Drone, cf=Crazyflie(rw_cache='./cache')) as scf:
                print(f'\033[92m{uri} \033[97mFully connected: {scf.is_params_updated()}. ', end='', flush=True)
                while scf.is_params_updated() is False:
                    print('.', end='', flush=True)
                    time.sleep(0.1)
                print(f'Fully connected: {scf.is_params_updated()}')
                time.sleep(0.5)
                write_one(file_name, scf)
                ps = PowerSwitch(Drone)
                ps.platform_power_down()
                time.sleep(2)

        except (Exception):
            print(f'Couldnt connect to \033[91m{uri}\033[97m')
            continue
