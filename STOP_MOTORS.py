import time
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.swarm import CachedCfFactory, Swarm
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# List your Crazyflie URIs here
uris = [
    'radio://0/30/2M/a0a0a0a0aa',
    'radio://0/30/2M/a0a0a0a0ae',
    'radio://0/30/2M/e7e7e7e7e7',
    'radio://0/30/2M/e7e7e7e7e8'
]

def filter_uris(uris):
    valid_uris = []
    for uri in uris:
        try:
            with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
                print(f"Successfully connected to   {uri}")
                valid_uris.append(uri)
        except Exception as e:
            print(f"Failed to connect to {uri}: {e}")
    return valid_uris

def stop_motors(scf):
    scf.cf.param.set_value('motorPowerSet.enable', '0')
    time.sleep(1)
    scf.cf.param.set_value('motorPowerSet.m1', '0')
    #scf.cf.param.set_value('motorPowerSet.m2', '0')
    #scf.cf.param.set_value('motorPowerSet.m3', '0')
    #scf.cf.param.set_value('motorPowerSet.m4', '0')
    print(f"Motors stopped for {scf._link_uri}")

if __name__ == '__main__':
    print("=== STOPPING ALL MOTORS ===")
    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')
    valid_uris = filter_uris(uris)
    if not valid_uris:
        print("No valid Crazyflie connections found. Exiting.")
        exit()
    with Swarm(valid_uris, factory=factory) as swarm:
        swarm.parallel_safe(stop_motors)
        time.sleep(1)
    