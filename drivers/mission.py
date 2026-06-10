import csv
import math
import os
import sys
import time

from .config import LabraxTCPConfig
from .labrax import Labrax

DVL_OFFSET_X_FROM_ROBOT = 0.68#1.08


def wrap_angle_deg(angle_deg):
    if angle_deg is None:
        return None
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def _finite_float(value):
    if value is None:
        return None
    value = float(value)
    if math.isnan(value) or math.isinf(value):
        return None
    return value


def _world_velocity_from_dvl(imu, dvl, vz_ignored=True):
    """
    Convert DVL body-frame velocities into world-frame velocities using the
    current IMU orientation.

    The function extracts the DVL velocity components (vx, vy, vz).

    Orientation is obtained from the IMU roll, pitch, and heading fields.
    A ZYX rotation matrix is then constructed and applied to the
    body-frame velocity vector to produce world-frame velocity components.

    Args:
        imu: IMU object providing ``roll``, ``pitch``, and ``heading`` fields
            in degrees.
        dvl: DVL object providing ``vx``, ``vy``, ``vz``
            velocity components in the vehicle body frame.
        vz_ignored: If True, the DVL vz component is ignored and treated as zero.

    Returns:
        tuple[float, float, float] | None: The velocity vector
        ``(wx, wy, wz)`` expressed in the world frame
    """
    vx = _finite_float(dvl.vx)
    vy = _finite_float(dvl.vy)
    vz = 0.0 if vz_ignored else _finite_float(getattr(dvl, 'vz', None))
    roll = _finite_float(imu.roll)
    pitch = _finite_float(imu.pitch)
    heading = _finite_float(imu.heading)
    if vx is None or vy is None or vz is None or roll is None or pitch is None or heading is None:
        return None

    yaw = math.radians(heading)
    pitch = math.radians(pitch)
    roll = math.radians(roll)

    cy = math.cos(yaw)
    sy = math.sin(yaw)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)

    wx = cy * cp * vx + (cy * sp * sr - sy * cr) * vy + (cy * sp * cr + sy * sr) * vz
    wy = sy * cp * vx + (sy * sp * sr + cy * cr) * vy + (sy * sp * cr - cy * sr) * vz
    wz = -sp * vx + cp * sr * vy + cp * cr * vz
    return (wx, wy, wz)


def _robot_offset_from_dvl(imu):
    roll = _finite_float(imu.roll)
    pitch = _finite_float(imu.pitch)
    heading = _finite_float(imu.heading)
    if roll is None or pitch is None or heading is None:
        return None

    yaw = math.radians(heading)
    pitch = math.radians(pitch)

    ox = DVL_OFFSET_X_FROM_ROBOT
    return (
        math.cos(yaw) * math.cos(pitch) * ox,
        math.sin(yaw) * math.cos(pitch) * ox,
        -math.sin(pitch) * ox,
    )


class DeadReckoning(object):
    """
    Simple dead-reckoning position estimator based on DVL velocities IMU
    orientation and depth measurements.

    The estimator maintains a 3D position estimate ``(x, y, z)`` in the world
    frame. Each new DVL sample is integrated once using its local reception
    timestamp and trapezoidal integration.
    
    Then computed DVL position is shifted in space with transform matrix and attitude that gives true robot positon.    
    
    A new velocity estimate is computed from the latest IMU and DVL
    measurements and stored for use during the next DVL sample.

    Args:
        vz_ignored (float, optional, True): value provided to _world_velocity_from_dvl() to ignore the DVL vz component and treat it as zero.

    Attributes:
        x (float): Estimated world-frame X position.
        y (float): Estimated world-frame Y position.
        z (float): Estimated world-frame Z position.

    Notes:
        - ``reset()`` clears the position estimate and reinitializes the
          integration state.
        - ``update()`` should be called periodically with the latest IMU and
          DVL measurements.
        - Time integration uses the DVL ``received_at`` timestamps.
    """
    def __init__(self, vz_ignored=True):
        self.vz_ignored = bool(vz_ignored)
        self.reset()

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self._dvl_x = 0.0
        self._dvl_y = 0.0
        self._dvl_z = 0.0
        self._last_sample_t = None
        self._last_velocity = None

    def update(self, imu, dvl):
        depth = _finite_float(getattr(imu, 'depth', None))
        if depth is not None:
            self._dvl_z = depth

        sample_t = getattr(dvl, 'received_at', None)
        if sample_t is not None and sample_t != self._last_sample_t:
            velocity = _world_velocity_from_dvl(imu, dvl, self.vz_ignored)
            if self._last_sample_t is not None and self._last_velocity is not None and velocity is not None:
                dt = sample_t - self._last_sample_t
                if dt > 0.0:
                    self._dvl_x += 0.5 * (self._last_velocity[0] + velocity[0]) * dt
                    self._dvl_y += 0.5 * (self._last_velocity[1] + velocity[1]) * dt
            self._last_sample_t = sample_t
            self._last_velocity = velocity

        offset = _robot_offset_from_dvl(imu)
        if offset is None:
            self.x = self._dvl_x
            self.y = self._dvl_y
            self.z = self._dvl_z
        else:
            self.x = self._dvl_x + offset[0]
            self.y = self._dvl_y + offset[1]
            self.z = self._dvl_z + offset[2]

        return (self.x, self.y, self.z)


class Security(object):
    def __init__(self, duration_s=None):
        self.duration_s = None if duration_s is None else float(duration_s)
        self._start = None

    def init(self, duration_s=None, now_t=None):
        if duration_s is not None:
            self.duration_s = float(duration_s)
        self._start = time.time() if now_t is None else float(now_t)

    def check(self, now_t=None, exit=False):
        if self.duration_s is None:
            raise ValueError('Security duration not set. Call init(duration_s).')
        if self._start is None:
            self.init(now_t=now_t)
        now_t = time.time() if now_t is None else float(now_t)
        ok = (now_t - self._start) < self.duration_s
        if (not ok) and exit:
            sys.exit(1)
        return ok


class Mission(object):
    def __init__(self, script_path, ip='127.0.0.1', hz=20.0, include_dr=False, output_path=None):
        self.config = LabraxTCPConfig(ip=ip)
        self.robot = Labrax(self.config)
        self.hz = float(hz)
        self.dt = 1.0 / self.hz if self.hz else 0.0
        self.include_dr = bool(include_dr)
        self.output_path = output_path or self._default_output_path(script_path)
        self._csv_file = None
        self._writer = None
        self.start_time = None

    def _default_output_path(self, script_path):
        script_name = os.path.splitext(os.path.basename(script_path))[0]
        filename = '%s_%s.csv' % (script_name, time.strftime('%H%M%S', time.localtime()))
        return os.path.join(os.path.dirname(script_path), filename)

    def __enter__(self):
        self.robot.start()
        self.robot.arm()
        self._csv_file = open(self.output_path, 'wb')
        self._writer = csv.writer(self._csv_file)
        header = [
            'timestamp',
            'roll',
            'pitch',
            'yaw',
            'depth',
            'vsurge',
            'vsway',
            'vheave',
            'DTS',
            'DTB',
            'ut',
            'uy',
            'up',
        ]
        if self.include_dr:
            header.append('x_dr')
            header.append('y_dr')
        self._writer.writerow(header)
        self.start_time = time.time()
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.robot.disarm()
        except Exception:
            pass
        try:
            self.robot.stop()
        except Exception:
            pass
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            except Exception:
                pass

    @property
    def imu(self):
        return self.robot.imu.data

    @property
    def dvl(self):
        return self.robot.dvl.data

    def yaw(self, imu=None):
        imu = self.imu if imu is None else imu
        heading = imu.heading
        if heading is None:
            return None
        return float(heading)

    def pitch_reg(self, target_deg, scale_deg, pitch=None, invert=True):
        pitch = self.imu.pitch if pitch is None else pitch
        if pitch is None:
            return 0.0
        value = math.tanh((float(target_deg) - float(pitch)) / float(scale_deg))
        return -value if invert else value

    def _fmt(self, value, precision=6):
        if value is None:
            return ''
        if isinstance(value, float):
            return ('%%.%df' % precision) % value
        return str(value)

    def _write_row(self, now_t, ut, uy, up, imu=None, dvl=None, x_dr=None, y_dr=None, yaw=None):
        imu = self.imu if imu is None else imu
        dvl = self.dvl if dvl is None else dvl
        yaw_value = self.yaw(imu) if yaw is None else yaw
        elapsed = now_t - self.start_time

        row = [
            self._fmt(elapsed, 3),
            self._fmt(imu.roll, 3),
            self._fmt(imu.pitch, 3),
            self._fmt(yaw_value, 3),
            self._fmt(imu.depth, 4),
            self._fmt(dvl.vx, 4),
            self._fmt(dvl.vy, 4),
            self._fmt(getattr(dvl, 'vz', None), 4),
            self._fmt(dvl.DTS, 3),
            self._fmt(dvl.DTB, 3),
            self._fmt(ut, 3),
            self._fmt(uy, 3),
            self._fmt(up, 3),
        ]
        if self.include_dr:
            row.append(self._fmt(x_dr, 3))
            row.append(self._fmt(y_dr, 3))
        self._writer.writerow(row)

    def sleep(self, loop_start):
        if self.dt <= 0.0:
            return
        sleep_s = self.dt - (time.time() - loop_start)
        if sleep_s > 0:
            time.sleep(sleep_s)

    def send(self, ut, uy, up, imu=None, dvl=None, x_dr=None, y_dr=None, yaw=None, now=None, sleep=True):
        now_t = time.time() if now is None else now
        self.robot.set_normalized(ut, uy, up)
        self._write_row(now_t, ut, uy, up, imu=imu, dvl=dvl, x_dr=x_dr, y_dr=y_dr, yaw=yaw)
        self._csv_file.flush()
        if sleep:
            self.sleep(now_t)
        return now_t

    def run_for(self, duration_s, ut, uy, up, dr=None):
        security = Security()
        security.init(duration_s)
        while security.check():
            loop_start = time.time()
            imu = self.imu
            dvl = self.dvl
            x_dr = None
            y_dr = None
            if dr is not None:
                dr.update(imu, dvl)
                x_dr = dr.x
                y_dr = dr.y
            self.send(ut, uy, up, imu=imu, dvl=dvl, x_dr=x_dr, y_dr=y_dr, now=loop_start)

    def wait_pitch(self, pitch_limit_deg=8.0, duration_s=10.0):
        security = Security()
        security.init(duration_s)
        while security.check():
            loop_start = time.time()
            imu = self.imu
            self.send(0.0, 0.0, 0.0, imu=imu, now=loop_start)
            pitch = imu.pitch
            if pitch is not None and math.fabs(pitch) < pitch_limit_deg:
                break
