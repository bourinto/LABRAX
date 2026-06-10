import time


class IMUData(object):
    def __init__(self, heading=None, pitch=None, roll=None, temperature=None, depth=None, timestamp=None):
        self.heading = heading
        self.pitch = pitch
        self.roll = roll
        self.temperature = temperature
        self.depth = depth
        self.timestamp = time.time() if timestamp is None else timestamp


class GPSData(object):
    def __init__(self, latitude=None, longitude=None, satellites=None, valid_fix=False, timestamp=None):
        self.latitude = latitude
        self.longitude = longitude
        self.satellites = satellites
        self.valid_fix = bool(valid_fix)
        self.timestamp = time.time() if timestamp is None else timestamp


class BatteryData(object):
    def __init__(self, percent=None, timestamp=None):
        self.percent = percent
        self.timestamp = time.time() if timestamp is None else timestamp


class DVLData(object):
    def __init__(
        self,
        timestamp_sensor=None,
        fix_type=None,
        fix_quality=None,
        vx=None,
        vy=None,
        vz=None,
        vel_err=None,
        dx=None,
        dy=None,
        DTB=None,
        DTS=None,
        beam_corr_a1=None,
        beam_corr_a2=None,
        beam_corr_a3=None,
        beam_corr_a4=None,
        velocity_source=None,
        valid=False,
        stale=True,
        timestamp=None,
    ):
        self.timestamp_sensor = timestamp_sensor
        self.fix_type = fix_type
        self.fix_quality = fix_quality
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.vel_err = vel_err
        self.dx = dx
        self.dy = dy
        self.DTB = DTB
        self.DTS = DTS
        self.beam_corr_a1 = beam_corr_a1
        self.beam_corr_a2 = beam_corr_a2
        self.beam_corr_a3 = beam_corr_a3
        self.beam_corr_a4 = beam_corr_a4
        self.velocity_source = velocity_source
        self.valid = bool(valid)
        self.stale = bool(stale)
        self.timestamp = time.time() if timestamp is None else timestamp
