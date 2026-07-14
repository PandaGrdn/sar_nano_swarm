import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
import cflib.crtp

cflib.crtp.init_drivers()
URI = 'udp://127.0.0.1:19850'

def wait_for_estimator(scf):
    print("Waiting for state estimator to converge...", flush=True)
    log_config = LogConfig(name='Kalman', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')
    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            if data['kalman.varPX'] < 0.001 and data['kalman.varPY'] < 0.001 and data['kalman.varPZ'] < 0.001:
                print("Estimator converged.", flush=True)
                break

def position_callback(timestamp, data, logconf):
    print(f"  [POS] x={data['stateEstimate.x']:.3f} y={data['stateEstimate.y']:.3f} z={data['stateEstimate.z']:.3f} yaw={data['stateEstimate.yaw']:.1f}", flush=True)

def start_position_logging(cf):
    log_config = LogConfig(name='Position', period_in_ms=250)
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')
    log_config.add_variable('stateEstimate.yaw', 'float')
    cf.log.add_config(log_config)
    log_config.data_received_cb.add_callback(position_callback)
    log_config.start()
    return log_config

print(f"Connecting to {URI} ...", flush=True)
with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    print("Enabling high-level commander...", flush=True)
    scf.cf.param.set_value('commander.enHighLevel', '1')
    time.sleep(1)

    wait_for_estimator(scf)
    pos_log = start_position_logging(scf.cf)

    print("Arming...", flush=True)
    scf.cf.platform.send_arming_request(True)
    time.sleep(1)

    with MotionCommander(scf, default_height=0.5) as mc:
        print("Taking off, hovering 3s...", flush=True)
        time.sleep(3)
        print("Moving forward 0.5m...", flush=True)
        mc.forward(0.5)
        time.sleep(1)
        print("Spinning 360 in place...", flush=True)
        mc.turn_left(360, rate=45)
        time.sleep(1)
        print("Landing...", flush=True)

    pos_log.stop()
print("Done.", flush=True)
