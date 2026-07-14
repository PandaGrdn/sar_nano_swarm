from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp

cflib.crtp.init_drivers()
URI = 'udp://127.0.0.1:19850'

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf
    print("=== PARAM groups containing 'arm' or 'safe' or 'lock' ===")
    for name in cf.param.toc.toc.keys():
        if any(k in name.lower() for k in ['arm', 'safe', 'lock', 'motor']):
            print(" ", name, list(cf.param.toc.toc[name].keys()))

    print("\n=== LOG groups containing 'thrust' or 'motor' or 'stabilizer' ===")
    for name in cf.log.toc.toc.keys():
        if any(k in name.lower() for k in ['thrust', 'motor', 'stabilizer', 'sys']):
            print(" ", name, list(cf.log.toc.toc[name].keys()))
