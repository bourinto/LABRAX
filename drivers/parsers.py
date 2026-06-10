import re


def clamp(x, lo, hi):
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def _to_float(value):
    if value == '':
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _to_negative_float(value):
    value = _to_float(value)
    return None if value is None else -value


def parse_compass(frame):
    def field(letter):
        if letter == 'C':
            match = re.search(r'\$C([+-]?\d+(?:\.\d+)?)', frame)
        else:
            match = re.search(r'%s([+-]?\d+(?:\.\d+)?)' % re.escape(letter), frame)
        if not match:
            return None
        return float(match.group(1))

    heading = field('C')
    pitch = field('P')
    roll = field('R')
    temperature = field('T')
    depth = field('D')

    if heading is None or pitch is None or roll is None or temperature is None or depth is None:
        return None

    return {
        'heading': heading,
        'pitch': pitch,
        'roll': roll,
        'temperature': temperature,
        'depth': depth,
    }


def nmea_to_decimal(coord, hemi):
    if not coord or not hemi:
        return None

    try:
        value = float(coord)
    except ValueError:
        return None

    deg = int(value // 100)
    minutes = value - deg * 100
    dec = deg + minutes / 60.0

    hemi = hemi.upper()
    if hemi in ('S', 'W'):
        dec = -dec

    return dec


def parse_gga(fields):
    if len(fields) < 8:
        return None

    latitude = nmea_to_decimal(fields[2], fields[3])
    longitude = nmea_to_decimal(fields[4], fields[5])

    try:
        satellites = int(fields[7]) if fields[7] else None
    except ValueError:
        satellites = None

    return latitude, longitude, satellites


def parse_son31(fields):
    if len(fields) < 12:
        return None

    vx = _to_negative_float(fields[4])
    vy = _to_float(fields[5])
    vz = _to_float(fields[6])
    DTB = _to_float(fields[10])
    DTS = _to_float(fields[11])
    if vx is None or vy is None or vz is None or DTB is None or DTS is None:
        return None

    return {
        'vx': vx,
        'vy': vy,
        'vz': vz,
        'DTB': DTB,
        'DTS': DTS,
    }
