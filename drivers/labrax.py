from .actuators import FinsCommand, FinsDriver, ThrustDriver
from .config import LabraxTCPConfig
from .parsers import clamp
from .sensors import BatteryDriver, DVLDriver, GPSDriver, IMUDriver


class MotionCommand(object):
    def __init__(self, thrust, fins):
        self.thrust = int(thrust)
        self.fins = fins


class Labrax(object):
    def __init__(self, config=None):
        self.config = config or LabraxTCPConfig()
        self.thruster = ThrustDriver(self.config)
        self.fins = FinsDriver(self.config)
        self.imu = IMUDriver(self.config)
        self.dvl = DVLDriver(self.config)
        self.gps = GPSDriver(self.config)
        self.battery = BatteryDriver(self.config)
        self._armed = False

    @property
    def armed(self):
        return self._armed

    def arm(self):
        self._armed = True

    def disarm(self):
        self._armed = False
        self.neutralize()

    def start(self):
        self.imu.start()
        self.dvl.start()
        self.gps.start()
        self.battery.start()

    def stop(self):
        self._armed = False
        for action in (
            self.thruster.stop,
            self.fins.neutral,
            self.imu.stop,
            self.dvl.stop,
            self.gps.stop,
            self.battery.stop,
            self.thruster.close,
            self.fins.close,
        ):
            try:
                action()
            except Exception:
                pass

    def neutralize(self):
        self.thruster.stop()
        self.fins.neutral()

    def set_raw(self, thrust, fin_top, fin_bottom, fin_left, fin_right):
        if not self._armed:
            self.neutralize()
            neutral = self.config.fins_neutral
            return MotionCommand(0, FinsCommand(neutral, neutral, neutral, neutral))

        thrust = int(clamp(thrust, -self.config.thrust_max, self.config.thrust_max))
        self.thruster.set(thrust)
        self.fins.set_raw(fin_top, fin_bottom, fin_left, fin_right)
        return MotionCommand(thrust, FinsCommand(fin_top, fin_bottom, fin_left, fin_right))

    def set_normalized(self, thrust, yaw, pitch):
        if not self._armed:
            self.neutralize()
            neutral = self.config.fins_neutral
            return MotionCommand(0, FinsCommand(neutral, neutral, neutral, neutral))

        thrust_i = int(clamp(thrust, -1.0, 1.0) * self.config.thrust_max)
        self.thruster.set(thrust_i)
        fins_cmd = self.fins.set_normalized(yaw=yaw, pitch=pitch)
        return MotionCommand(thrust_i, fins_cmd)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
