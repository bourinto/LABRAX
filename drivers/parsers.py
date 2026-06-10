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


def _to_int(value):
    if value == '':
        return None
    try:
        return int(value)
    except ValueError:
        return None


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

    return {
        'timestamp_sensor': _to_float(fields[1]),
        'fix_type': _to_int(fields[2]),
        'fix_quality': _to_float(fields[3]),
        'vx': -_to_float(fields[4]),
        'vy': _to_float(fields[5]),
        'vz': _to_float(fields[6]),
        'dx': _to_float(fields[8]),
        'dy': _to_float(fields[9]),
        'DTB': _to_float(fields[10]),
        'DTS': _to_float(fields[11]),
    }


def parse_son51(fields):
    if len(fields) < 11:
        return None

    return {
        'timestamp_sensor': _to_float(fields[1]),
        'cell_id': _to_int(fields[2]),
        'vx': -_to_float(fields[3]),
        'vy': _to_float(fields[4]),
        'vz': _to_float(fields[5]),
        'vel_err': _to_float(fields[6]),
        'a1': _to_float(fields[7]),
        'a2': _to_float(fields[8]),
        'a3': _to_float(fields[9]),
        'a4': _to_float(fields[10]),
    }
