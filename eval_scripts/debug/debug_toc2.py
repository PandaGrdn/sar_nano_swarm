from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import cflib.crtp

cflib.crtp.init_drivers()
URI = 'udp://127.0.0.1:19850'

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf
    print("=== PARAM groups: supervisor / commander / flight / kill / cf2 ===")
    for name in cf.param.toc.toc.keys():
        if any(k in name.lower() for k in ['supervisor', 'commander', 'flight', 'kill', 'cf2']):
            print(" ", name, list(cf.param.toc.toc[name].keys()))

    print("\n=== LOG groups: supervisor ===")
    for name in cf.log.toc.toc.keys():
        if 'supervisor' in name.lower():
            print(" ", name, list(cf.log.toc.toc[name].keys()))
