from .core import Endpoint, TCPClient
from .parsers import clamp


class FinsCommand(object):
    def __init__(self, top, bottom, left, right):
        self.top = int(top)
        self.bottom = int(bottom)
        self.left = int(left)
        self.right = int(right)


class ThrustDriver(object):
    def __init__(self, config):
        self._config = config
        self._client = TCPClient(
            Endpoint(config.ip, config.port_thrust),
            timeout_s=config.socket_timeout_s,
            connect_timeout_s=config.connect_timeout_s,
        )

    def set(self, thrust):
        thrust = int(clamp(thrust, -self._config.thrust_max, self._config.thrust_max))
        payload = ('V=%d\r\nG\r\n' % (-thrust)).encode('ascii')
        self._client.sendall(payload)

    def stop(self):
        self.set(0)

    def close(self):
        self._client.close()


class FinsDriver(object):
    def __init__(self, config):
        self._config = config
        self._client = TCPClient(
            Endpoint(config.ip, config.port_fins),
            timeout_s=config.socket_timeout_s,
            connect_timeout_s=config.connect_timeout_s,
        )

    def _clip_fin(self, value):
        return int(clamp(value, self._config.fins_min, self._config.fins_max))

    def set_raw(self, top, bottom, left, right):
        v1 = self._clip_fin(top)
        v2 = self._clip_fin(bottom)
        v3 = self._clip_fin(left)
        v4 = self._clip_fin(right)

        payload = bytearray([
            0xFF, 0x01, v1,
            0xFF, 0x02, v2,
            0xFF, 0x03, v3,
            0xFF, 0x04, v4,
        ])
        self._client.sendall(payload)

    def set_normalized(self, yaw, pitch):
        yaw = clamp(yaw, -1.0, 1.0)
        pitch = clamp(pitch, -1.0, 1.0)
        pitch = -pitch

        center = self._config.fins_neutral
        amp = self._config.fins_amplitude

        v1 = int(round(center + amp * yaw))
        v2 = int(round(center + amp * yaw))
        v3 = int(round(center + amp * pitch))
        v4 = int(round(center + amp * pitch))

        cmd = FinsCommand(
            top=self._clip_fin(v1),
            bottom=self._clip_fin(v2),
            left=self._clip_fin(v3),
            right=self._clip_fin(v4),
        )
        self.set_raw(cmd.top, cmd.bottom, cmd.left, cmd.right)
        return cmd

    def neutral(self):
        n = self._config.fins_neutral
        self.set_raw(n, n, n, n)

    def close(self):
        self._client.close()
