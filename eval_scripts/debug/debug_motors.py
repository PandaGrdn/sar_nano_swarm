import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
import cflib.crtp

cflib.crtp.init_drivers()
URI = 'udp://127.0.0.1:19850'

def cb(timestamp, data, logconf):
    print(f"  canfly={data['sys.canfly']} isFlying={data['sys.isFlying']} isTumbled={data['sys.isTumbled']} "
          f"m1={data['motor.m1']} m2={data['motor.m2']} thrust={data['stabilizer.thrust']}", flush=True)

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf

    log_config = LogConfig(name='Debug', period_in_ms=250)
    log_config.add_variable('sys.canfly', 'uint8_t')
    log_config.add_variable('sys.isFlying', 'uint8_t')
    log_config.add_variable('sys.isTumbled', 'uint8_t')
    log_config.add_variable('motor.m1', 'uint16_t')
    log_config.add_variable('motor.m2', 'uint16_t')
    log_config.add_variable('stabilizer.thrust', 'float')
    cf.log.add_config(log_config)
    log_config.data_received_cb.add_callback(cb)
    log_config.start()

    print("Baseline (5s, no commands)...", flush=True)
    time.sleep(5)

    print("Sending arming request...", flush=True)
    cf.platform.send_arming_request(True)
    time.sleep(2)

    print("Sending high thrust setpoint directly (bypass MotionCommander)...", flush=True)
    for _ in range(40):
        cf.commander.send_setpoint(0, 0, 0, 30000)  # roll, pitch, yawrate, thrust(0-65535)
        time.sleep(0.1)

    cf.commander.send_stop_setpoint()
    log_config.stop()
