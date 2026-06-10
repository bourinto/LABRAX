import socket
import threading
import time

from .core import Endpoint, TCPClient
from .models import BatteryData, DVLData, GPSData, IMUData
from .parsers import parse_compass, parse_gga, parse_son31, parse_son51


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
            self._thread.join(1.0)

    def _is_running(self):
        with self._lock:
            return self._running

    def _set_connected(self, connected):
        with self._lock:
            self._connected = bool(connected)

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
            except Exception:
                self._set_connected(False)
                self._client.close()
                time.sleep(self._reconnect_delay_s)


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
                    if not fields:
                        continue

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
            except Exception:
                self._set_connected(False)
                self._client.close()
                time.sleep(self._reconnect_delay_s)


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
        self._config = config
        self._buf = ''
        self._data = DVLData()
        self._last_any_sample_t = 0.0
        self._last_valid_sample_t = 0.0

    @property
    def data(self):
        return self._data

    def _set_stale_state(self, now_t):
        if self._last_valid_sample_t <= 0.0:
            self._data = DVLData(
                timestamp_sensor=self._data.timestamp_sensor,
                fix_type=self._data.fix_type,
                fix_quality=self._data.fix_quality,
                vx=self._data.vx,
                vy=self._data.vy,
                vz=self._data.vz,
                vel_err=self._data.vel_err,
                dx=self._data.dx,
                dy=self._data.dy,
                DTB=self._data.DTB,
                DTS=self._data.DTS,
                beam_corr_a1=self._data.beam_corr_a1,
                beam_corr_a2=self._data.beam_corr_a2,
                beam_corr_a3=self._data.beam_corr_a3,
                beam_corr_a4=self._data.beam_corr_a4,
                velocity_source=self._data.velocity_source,
                valid=False,
                stale=True,
                timestamp=now_t,
            )
            return

        if now_t - self._last_valid_sample_t > self._config.dvl_stale_after_s:
            self._data = DVLData(
                timestamp_sensor=self._data.timestamp_sensor,
                fix_type=self._data.fix_type,
                fix_quality=self._data.fix_quality,
                vx=self._data.vx,
                vy=self._data.vy,
                vz=self._data.vz,
                vel_err=self._data.vel_err,
                dx=self._data.dx,
                dy=self._data.dy,
                DTB=self._data.DTB,
                DTS=self._data.DTS,
                beam_corr_a1=self._data.beam_corr_a1,
                beam_corr_a2=self._data.beam_corr_a2,
                beam_corr_a3=self._data.beam_corr_a3,
                beam_corr_a4=self._data.beam_corr_a4,
                velocity_source=self._data.velocity_source,
                valid=False,
                stale=True,
                timestamp=now_t,
            )

    def _loop(self):
        while self._is_running():
            try:
                chunk = self._client.recv(2048)
                if not chunk:
                    raise socket.error('DVL socket closed')

                now_t = time.time()
                self._last_any_sample_t = now_t
                self._set_connected(True)

                self._buf += chunk.decode('ascii', 'ignore')
                if len(self._buf) > 8192:
                    self._buf = self._buf[-8192:]

                while '\n' in self._buf:
                    line, self._buf = self._buf.split('\n', 1)
                    line = line.strip('\r\t ')
                    if not line.startswith('$SON'):
                        continue

                    sentence = line.split('*', 1)[0]
                    fields = sentence.split(',')
                    if not fields:
                        continue

                    prev = self._data
                    msg = fields[0]
                    if msg == '$SON31':
                        parsed = parse_son31(fields)
                        if parsed is None:
                            continue

                        fix_type = parsed.get('fix_type')
                        fix_quality = parsed.get('fix_quality')
                        if fix_type == 2:
                            velocity_source = 'bottom-track'
                        elif fix_type == 1:
                            velocity_source = 'water-track'
                        else:
                            velocity_source = None

                        vel_err = self._data.vel_err
                        quality_ok = fix_quality is not None and fix_quality >= self._config.dvl_min_fix_quality
                        vel_err_ok = vel_err is None or vel_err <= self._config.dvl_max_vel_err
                        valid = velocity_source is not None and quality_ok and vel_err_ok
                        stale = not valid
                        if valid:
                            self._last_valid_sample_t = now_t

                        self._data = DVLData(
                            timestamp_sensor=parsed.get('timestamp_sensor'),
                            fix_type=fix_type,
                            fix_quality=fix_quality,
                            vx=parsed.get('vx'),
                            vy=parsed.get('vy'),
                            vz=parsed.get('vz'),
                            vel_err=vel_err,
                            dx=parsed.get('dx'),
                            dy=parsed.get('dy'),
                            DTB=parsed.get('DTB'),
                            DTS=parsed.get('DTS'),
                            beam_corr_a1=prev.beam_corr_a1,
                            beam_corr_a2=prev.beam_corr_a2,
                            beam_corr_a3=prev.beam_corr_a3,
                            beam_corr_a4=prev.beam_corr_a4,
                            velocity_source=velocity_source,
                            valid=valid,
                            stale=stale,
                            timestamp=now_t,
                        )
                    elif msg == '$SON51':
                        parsed = parse_son51(fields)
                        if parsed is None:
                            continue

                        vel_err = parsed.get('vel_err')
                        fix_quality = prev.fix_quality
                        quality_ok = fix_quality is not None and fix_quality >= self._config.dvl_min_fix_quality
                        vel_err_ok = vel_err is None or vel_err <= self._config.dvl_max_vel_err
                        valid = prev.fix_type in (1, 2) and quality_ok and vel_err_ok
                        stale = not valid
                        if valid:
                            self._last_valid_sample_t = now_t

                        self._data = DVLData(
                            timestamp_sensor=prev.timestamp_sensor,
                            fix_type=prev.fix_type,
                            fix_quality=prev.fix_quality,
                            vx=prev.vx,
                            vy=prev.vy,
                            vz=prev.vz,
                            vel_err=vel_err,
                            dx=prev.dx,
                            dy=prev.dy,
                            DTB=prev.DTB,
                            DTS=prev.DTS,
                            beam_corr_a1=parsed.get('a1'),
                            beam_corr_a2=parsed.get('a2'),
                            beam_corr_a3=parsed.get('a3'),
                            beam_corr_a4=parsed.get('a4'),
                            velocity_source=prev.velocity_source,
                            valid=valid,
                            stale=stale,
                            timestamp=now_t,
                        )

                self._set_stale_state(now_t)

            except socket.timeout:
                now_t = time.time()
                if self._last_any_sample_t > 0.0 and now_t - self._last_any_sample_t > self._config.dvl_disconnect_after_s:
                    self._set_connected(False)
                self._set_stale_state(now_t)
                continue
            except Exception:
                self._set_connected(False)
                self._client.close()
                self._set_stale_state(time.time())
                time.sleep(self._reconnect_delay_s)


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
            except Exception:
                self._set_connected(False)
                self._client.close()
                time.sleep(self._reconnect_delay_s)
