class LabraxTCPConfig(object):
    def __init__(
        self,
        ip='127.0.0.1',
        port_thrust=5011,
        port_fins=5012,
        port_imu=5006,
        port_dvl=5007,
        port_gnss=5009,
        port_battery=5010,
        socket_timeout_s=2.0,
        connect_timeout_s=2.0,
        reconnect_delay_s=0.2,
        thrust_max=1000000,
        fins_min=5,
        fins_max=250,
        fins_neutral=128,
        fins_amplitude=122,
    ):
        self.ip = ip
        self.port_thrust = int(port_thrust)
        self.port_fins = int(port_fins)
        self.port_imu = int(port_imu)
        self.port_dvl = int(port_dvl)
        self.port_gnss = int(port_gnss)
        self.port_battery = int(port_battery)
        self.socket_timeout_s = float(socket_timeout_s)
        self.connect_timeout_s = float(connect_timeout_s)
        self.reconnect_delay_s = float(reconnect_delay_s)
        self.thrust_max = int(thrust_max)
        self.fins_min = int(fins_min)
        self.fins_max = int(fins_max)
        self.fins_neutral = int(fins_neutral)
        self.fins_amplitude = int(fins_amplitude)
