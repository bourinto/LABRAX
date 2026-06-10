class IMUData(object):
    def __init__(self, heading=None, pitch=None, roll=None, temperature=None, depth=None):
        self.heading = heading
        self.pitch = pitch
        self.roll = roll
        self.temperature = temperature
        self.depth = depth


class GPSData(object):
    def __init__(self, latitude=None, longitude=None, satellites=None):
        self.latitude = latitude
        self.longitude = longitude
        self.satellites = satellites


class BatteryData(object):
    def __init__(self, percent=None):
        self.percent = percent


class DVLData(object):
    def __init__(self, timestamp=None, vx=None, vy=None, vz=None, DTB=None, DTS=None):
        self.timestamp = timestamp
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.DTB = DTB
        self.DTS = DTS
