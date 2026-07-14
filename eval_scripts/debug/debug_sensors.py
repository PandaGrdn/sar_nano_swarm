import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
import cflib.crtp

cflib.crtp.init_drivers()
URI = 'udp://127.0.0.1:19850'

def cb(ts, data, lc):
    print(f"  accX={data['acc.x']:.3f} accZ={data['acc.z']:.3f} "
          f"gyroZ={data['gyro.z']:.2f} baro={data['baro.asl']:.2f} "
          f"canfly={data['sys.canfly']} tumbled={data['sys.isTumbled']}", flush=True)

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf
    lc = LogConfig(name='S', period_in_ms=300)
    lc.add_variable('acc.x','float'); lc.add_variable('acc.z','float')
    lc.add_variable('gyro.z','float'); lc.add_variable('baro.asl','float')
    lc.add_variable('sys.canfly','uint8_t'); lc.add_variable('sys.isTumbled','uint8_t')
    cf.log.add_config(lc); lc.data_received_cb.add_callback(cb); lc.start()
    print("Reading sensors 5s (drone should be sitting on ground)...", flush=True)
    time.sleep(5)
    lc.stop()
