import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
import cflib.crtp

cflib.crtp.init_drivers()
URI = 'udp://127.0.0.1:19850'

def cb(ts, data, lc):
    print(f"  info={data['supervisor.info']:016b} canfly={data['sys.canfly']} "
          f"armed?={'armed' in dir()} thrust={data['stabilizer.thrust']:.0f}", flush=True)

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf

    # Check what arming-related params/methods exist
    print("platform methods:", [m for m in dir(cf.platform) if not m.startswith('_')], flush=True)

    lc = LogConfig(name='A', period_in_ms=300)
    lc.add_variable('supervisor.info', 'uint16_t')
    lc.add_variable('sys.canfly', 'uint8_t')
    lc.add_variable('stabilizer.thrust', 'float')
    cf.log.add_config(lc)
    lc.data_received_cb.add_callback(cb)
    lc.start()

    print("--- before arming ---", flush=True)
    time.sleep(2)
    print("--- sending arming request ---", flush=True)
    cf.platform.send_arming_request(True)
    time.sleep(3)
    lc.stop()
