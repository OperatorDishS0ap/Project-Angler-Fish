import ms5837


def init_bar30():
    sensor_dev = ms5837.MS5837_30BA()
    if not sensor_dev.init():
        print("[bar30] init failed")
        return None
    if not sensor_dev.read():
        print("[bar30] first read failed")
        return None
    return sensor_dev


def read_bar30(sensor_dev):
    if sensor_dev is None:
        return 0.0, 0.0, 0.0
    if not sensor_dev.read():
        return 0.0, 0.0, 0.0
    pressure = sensor_dev.pressure(ms5837.UNITS_psi)
    temperature = sensor_dev.temperature(ms5837.UNITS_Centigrade)
    depth = sensor_dev.depth()
    return pressure, temperature, depth
