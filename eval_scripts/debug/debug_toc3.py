import struct
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
import cflib.crtp

cflib.crtp.init_drivers()
URI = 'udp://127.0.0.1:19850'

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf

    val = cf.param.get_value('commander.enHighLevel')
    print(f"commander.enHighLevel = {val}")

    log_config = LogConfig(name='Sup', period_in_ms=200)
    log_config.add_variable('supervisor.info', 'uint16_t')
    with SyncLogger(scf, log_config) as logger:
        for i, log_entry in enumerate(logger):
            data = log_entry[1]
            info = data['supervisor.info']
            print(f"supervisor.info = {info} (binary: {info:016b})")
            if i >= 3:
                break
