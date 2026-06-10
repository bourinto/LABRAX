import socket
import threading
import time

from .core import Endpoint, TCPClient
from .models import BatteryData, DVLData, GPSData, IMUData
from .parsers import parse_compass, parse_gga, parse_son31


class _BaseSensor(object):
    def __init__(self, client, reconnect_delay_s):
        self._client = client
        self._reconnect_delay_s = float(reconnect_delay_s)
        self._thread = None
        self._running = False
        self._connected = False
        self._lock = threading.Lock()

    @property
    def connected(self):
        with self._lock:
            return self._connected

    def start(self):
        with self._lock:
            if self._running:
                return
            self._running = True

        self._thread = threading.Thread(target=self._loop)
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        with self._lock:
            self._running = False

        self._client.close()
        if self._thread is not None:
            self._thread.join()

    def _is_running(self):
        with self._lock:
            return self._running

    def _set_connected(self, connected):
        with self._lock:
            self._connected = bool(connected)

    def _disconnect(self):
        self._set_connected(False)
        self._client.close()
        self._buf = ''

    def _loop(self):
        raise NotImplementedError


class IMUDriver(_BaseSensor):
    def __init__(self, config):
        _BaseSensor.__init__(
            self,
            client=TCPClient(
                Endpoint(config.ip, config.port_imu),
                timeout_s=config.socket_timeout_s,
                connect_timeout_s=config.connect_timeout_s,
            ),
            reconnect_delay_s=config.reconnect_delay_s,
        )
        self._buf = ''
        self._data = IMUData()

    @property
    def data(self):
        return self._data

    def _loop(self):
        while self._is_running():
            try:
                chunk = self._client.recv(1024)
                if not chunk:
                    raise socket.error('IMU socket closed')

                self._set_connected(True)
                self._buf += chunk.decode('ascii', 'ignore')
                if len(self._buf) > 4096:
                    self._buf = self._buf[-4096:]

                while self._buf:
                    start = self._buf.find('$')
                    if start == -1:
                        self._buf = ''
                        break
                    if start > 0:
                        self._buf = self._buf[start:]

                    star = self._buf.find('*', 1)
                    if star == -1 or len(self._buf) < star + 3:
                        break

                    frame = self._buf[:star + 3]
                    self._buf = self._buf[star + 3:]

                    parsed = parse_compass(frame)
                    if parsed is None:
                        continue

                    self._data = IMUData(
                        heading=parsed.get('heading'),
                        pitch=parsed.get('pitch'),
                        roll=parsed.get('roll'),
                        temperature=parsed.get('temperature'),
                        depth=parsed.get('depth'),
                    )

            except socket.timeout:
                continue
            except socket.error:
                self._disconnect()
                time.sleep(self._reconnect_delay_s)
            except Exception:
                continue


class GPSDriver(_BaseSensor):
    def __init__(self, config):
        _BaseSensor.__init__(
            self,
            client=TCPClient(
                Endpoint(config.ip, config.port_gnss),
                timeout_s=config.socket_timeout_s,
                connect_timeout_s=config.connect_timeout_s,
            ),
            reconnect_delay_s=config.reconnect_delay_s,
        )
        self._buf = ''
        self._data = GPSData()

    @property
    def data(self):
        return self._data

    def _loop(self):
        while self._is_running():
            try:
                chunk = self._client.recv(2048)
                if not chunk:
                    raise socket.error('GNSS socket closed')

                self._set_connected(True)
                self._buf += chunk.decode('ascii', 'ignore')
                if len(self._buf) > 8192:
                    self._buf = self._buf[-8192:]

                while '\n' in self._buf:
                    line, self._buf = self._buf.split('\n', 1)
                    line = line.strip('\r\t ')
                    if not line.startswith('$GPGGA,'):
                        continue

                    sentence = line.split('*', 1)[0]
                    fields = sentence.split(',')

                    parsed = parse_gga(fields)
                    if parsed is None:
                        continue
                    latitude, longitude, satellites = parsed
                    self._data = GPSData(
                        latitude=latitude,
                        longitude=longitude,
                        satellites=satellites,
                    )

            except socket.timeout:
                continue
            except socket.error:
                self._disconnect()
                time.sleep(self._reconnect_delay_s)
            except Exception:
                continue


class DVLDriver(_BaseSensor):
    def __init__(self, config):
        _BaseSensor.__init__(
            self,
            client=TCPClient(
                Endpoint(config.ip, config.port_dvl),
                timeout_s=config.socket_timeout_s,
                connect_timeout_s=config.connect_timeout_s,
            ),
            reconnect_delay_s=config.reconnect_delay_s,
        )
        self._buf = ''
        self._data = DVLData()

    @property
    def data(self):
        return self._data

    def _loop(self):
        while self._is_running():
            try:
                chunk = self._client.recv(2048)
                if not chunk:
                    raise socket.error('DVL socket closed')

                self._set_connected(True)

                self._buf += chunk.decode('ascii', 'ignore')
                if len(self._buf) > 8192:
                    self._buf = self._buf[-8192:]

                while '\n' in self._buf:
                    line, self._buf = self._buf.split('\n', 1)
                    line = line.strip('\r\t ')
                    if not line.startswith('$SON31,'):
                        continue

                    sentence = line.split('*', 1)[0]
                    fields = sentence.split(',')
                    parsed = parse_son31(fields)
                    if parsed is None:
                        continue

                    self._data = DVLData(
                        timestamp=parsed.get('timestamp'),
                        vx=parsed.get('vx'),
                        vy=parsed.get('vy'),
                        vz=parsed.get('vz'),
                        DTB=parsed.get('DTB'),
                        DTS=parsed.get('DTS'),
                    )

            except socket.timeout:
                continue
            except socket.error:
                self._disconnect()
                time.sleep(self._reconnect_delay_s)
            except Exception:
                continue


class BatteryDriver(_BaseSensor):
    def __init__(self, config):
        _BaseSensor.__init__(
            self,
            client=TCPClient(
                Endpoint(config.ip, config.port_battery),
                timeout_s=config.socket_timeout_s,
                connect_timeout_s=config.connect_timeout_s,
            ),
            reconnect_delay_s=config.reconnect_delay_s,
        )
        self._buf = ''
        self._data = BatteryData()

    @property
    def data(self):
        return self._data

    def _loop(self):
        while self._is_running():
            try:
                chunk = self._client.recv(1024)
                if not chunk:
                    raise socket.error('Battery socket closed')

                self._set_connected(True)
                self._buf += chunk.decode('ascii', 'ignore')
                if len(self._buf) > 4096:
                    self._buf = self._buf[-4096:]

                while '\n' in self._buf:
                    line, self._buf = self._buf.split('\n', 1)
                    line = line.strip('\r\t ')
                    if not line.startswith('$OCEANA'):
                        continue

                    fields = line.split('*', 1)[0].split(',')
                    if len(fields) < 2:
                        continue

                    try:
                        percent = int(fields[1])
                    except ValueError:
                        continue

                    self._data = BatteryData(percent=percent)

            except socket.timeout:
                continue
            except socket.error:
                self._disconnect()
                time.sleep(self._reconnect_delay_s)
            except Exception:
                continue
