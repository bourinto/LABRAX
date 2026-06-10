# Python 3
import fcntl
import math
import os
import select
import struct
import sys
import time

import numpy as np
import cv2

from drivers import Labrax, LabraxTCPConfig


WHITE = (238, 238, 238)
MUTED = (160, 160, 160)
GREEN = (0, 180, 0)
ORANGE = (0, 165, 255)
RED = (0, 0, 255)
PANEL = (28, 28, 28)
PANEL_ALT = (34, 34, 34)
BORDER = (70, 70, 70)
BORDER_SOFT = (55, 55, 55)

FONT = cv2.FONT_HERSHEY_SIMPLEX
LINE_AA = cv2.LINE_AA

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

LEFT_Y_AXIS = 1
RIGHT_X_AXIS = 3
RIGHT_Y_AXIS = 4
START_BUTTONS = (7, 9)
SELECT_BUTTONS = (6, 8)
JOYSTICK_DEADZONE = 1.0
LOOP_DT = 1.0 / 20.0
DISARMED_SEND_DT = 1.0


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def text(img, value, pos, scale, color, thickness=1):
    cv2.putText(img, str(value), pos, FONT, scale, color, thickness, LINE_AA)


def box(img, x0, y0, x1, y1, fill, border=BORDER, thickness=1):
    cv2.rectangle(img, (x0, y0), (x1, y1), fill, -1)
    cv2.rectangle(img, (x0, y0), (x1, y1), border, thickness)


def status_tile(img, x0, y0, x1, y1, title, value, color):
    box(img, x0, y0, x1, y1, PANEL_ALT, color, 2)
    text(img, title, (x0 + 10, y0 + 20), 0.48, MUTED, 1)
    text(img, value, (x0 + 10, y0 + 52), 0.82, color, 2)


def value_bar(img, label, value, x0, x1, y):
    bar_h = 16
    bar_y0 = y + 2
    bar_y1 = bar_y0 + bar_h
    mid_x = (x0 + x1) // 2
    value = clamp(float(value), -100.0, 100.0)
    value_x = int(round(mid_x + (value / 100.0) * ((x1 - x0) / 2.0)))

    text(img, label, (x0, y - 8), 0.48, MUTED, 1)
    text(img, str(int(round(value))), (x1 - 92, y - 8), 0.48, WHITE, 1)
    box(img, x0, bar_y0, x1, bar_y1, (42, 42, 42), BORDER_SOFT, 1)
    if value_x != mid_x:
        cv2.rectangle(img, (min(mid_x, value_x), bar_y0 + 1),
                      (max(mid_x, value_x), bar_y1 - 1), GREEN, -1)
    cv2.line(img, (mid_x, bar_y0), (mid_x, bar_y1), WHITE, 1, LINE_AA)


def actuator_tile(img, x0, y0, x1, y1, title, value, value_color=WHITE):
    box(img, x0, y0, x1, y1, PANEL_ALT, BORDER_SOFT, 1)
    text(img, title, (x0 + 10, y0 + 20), 0.45, MUTED, 1)
    text(img, str(int(round(value))), (x0 + 10, y0 + 52), 0.62, value_color, 2)


def _fmt_or_na(value, fmt):
    if value is None:
        return "N/A"
    return fmt % value


def _battery_color(connected, percent):
    if not connected or percent is None:
        return RED
    if percent >= 60:
        return GREEN
    if percent >= 30:
        return ORANGE
    return RED


def _battery_value(connected, percent):
    if not connected or percent is None:
        return "N/A"
    return "%d%%" % int(round(percent))


class Joystick(object):
    def __init__(self, path='/dev/input/js0'):
        self.path = path
        self.fd = None
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0
        self.start_pressed = False
        self.select_pressed = False
        self._open()

    @property
    def connected(self):
        return self.fd is not None

    def _open(self):
        if self.fd is not None:
            return
        try:
            self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
            flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
            fcntl.fcntl(self.fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        except OSError:
            self.fd = None

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = None

    def _drop(self):
        self.close()
        self.left_y = 0.0
        self.right_x = 0.0
        self.right_y = 0.0

    def update(self):
        self.start_pressed = False
        self.select_pressed = False
        self._open()
        if self.fd is None:
            return

        try:
            while True:
                ready, _, _ = select.select([self.fd], [], [], 0)
                if not ready:
                    break
                event = os.read(self.fd, 8)
                if len(event) != 8:
                    break
                _, value, event_type, number = struct.unpack('IhBB', event)
                event_type = event_type & ~JS_EVENT_INIT
                if event_type == JS_EVENT_AXIS:
                    percent = clamp(value / 32767.0 * 100.0, -100.0, 100.0)
                    if number == LEFT_Y_AXIS:
                        self.left_y = -percent
                    elif number == RIGHT_X_AXIS:
                        self.right_x = percent
                    elif number == RIGHT_Y_AXIS:
                        self.right_y = -percent
                elif event_type == JS_EVENT_BUTTON:
                    if number in START_BUTTONS and value:
                        self.start_pressed = True
                    elif number in SELECT_BUTTONS and value:
                        self.select_pressed = True
        except (OSError, IOError):
            self._drop()


def _local_xy(lat, lon, lat0, lon0):
    if lat is None or lon is None or lat0 is None or lon0 is None:
        return None, None
    meters_per_deg_lat = 110540.0
    meters_per_deg_lon = 111320.0 * math.cos(math.radians(lat0))
    return (lon - lon0) * meters_per_deg_lon, (lat - lat0) * meters_per_deg_lat


def _fins_list(command, neutral):
    if command is None or command.fins is None:
        return [neutral, neutral, neutral, neutral]
    return [command.fins.top, command.fins.bottom, command.fins.left, command.fins.right]


def _command_outputs(config, thrust, yaw, pitch):
    thrust_i = int(clamp(thrust, -1.0, 1.0) * config.thrust_max)
    yaw = clamp(yaw, -1.0, 1.0)
    pitch = -clamp(pitch, -1.0, 1.0)

    center = config.fins_neutral
    amp = config.fins_amplitude
    fins = [
        int(round(center + amp * yaw)),
        int(round(center + amp * yaw)),
        int(round(center + amp * pitch)),
        int(round(center + amp * pitch)),
    ]
    fins = [int(clamp(value, config.fins_min, config.fins_max)) for value in fins]
    return thrust_i, fins


def _stick_to_unit(value):
    if abs(value) < JOYSTICK_DEADZONE:
        return 0.0
    return clamp(value / 100.0, -1.0, 1.0)


def _safe_disarm(robot):
    try:
        robot.disarm()
    except Exception:
        pass



def draw_ui(left_y, right_x, right_y, thrust, fins, connected, armed,
            heading, heading_connected, compass_pitch, compass_roll,
            dvl_connected, dvl_valid, dvl_stale, dvl_source,
            dvl_fix_quality, dvl_vx, dvl_vy, dvl_vz, dvl_vel_err,
            gnss_connected, lat, lon, lat0, lon0, local_x, local_y, sats,
            battery_connected, battery_percent):
    img = np.zeros((620, 980, 3), dtype=np.uint8)

    text(img, "LABRAX", (18, 34), 0.95, WHITE, 2)
    text(img, "START toggles ARM   Q or Esc exits", (18, 54), 0.44, MUTED, 1)

    tile_y0 = 68
    tile_y1 = 140
    left_margin = 16
    gap = 10
    tile_w = (980 - 2 * left_margin - 3 * gap) // 4
    tile_x = [left_margin + i * (tile_w + gap) for i in range(4)]

    state_value = "ARMED" if armed else "DISARMED"
    state_color = GREEN if armed else RED
    battery_value = _battery_value(battery_connected, battery_percent)
    battery_color = _battery_color(battery_connected, battery_percent)
    gamepad_status = "OK" if connected else "DISCONNECTED"
    gamepad_color = GREEN if connected else RED
    gnss_value = "OK" if gnss_connected else "DISCONNECTED"
    gnss_color = GREEN if gnss_connected else RED

    status_tile(img, tile_x[0], tile_y0, tile_x[0] + tile_w, tile_y1,
                "STATE", state_value, state_color)
    status_tile(img, tile_x[1], tile_y0, tile_x[1] + tile_w, tile_y1,
                "BATTERY", battery_value, battery_color)
    status_tile(img, tile_x[2], tile_y0, tile_x[2] + tile_w, tile_y1,
                "GAMEPAD", gamepad_status, gamepad_color)
    status_tile(img, tile_x[3], tile_y0, tile_x[3] + tile_w, tile_y1,
                "GNSS", gnss_value, gnss_color)

    main_y0 = 156
    main_y1 = 500
    sensors_x0 = 16
    sensors_x1 = 478
    control_x0 = 496
    control_x1 = 964

    box(img, sensors_x0, main_y0, sensors_x1, main_y1, PANEL, BORDER, 1)
    text(img, "Sensors", (sensors_x0 + 10, main_y0 + 22), 0.58, WHITE, 1)
    box(img, control_x0, main_y0, control_x1, main_y1, PANEL, BORDER, 1)
    text(img, "Control", (control_x0 + 10, main_y0 + 22), 0.58, WHITE, 1)

    inner_x0 = sensors_x0 + 10
    inner_x1 = sensors_x1 - 10
    sensor_y = main_y0 + 28

    dvl_y0 = sensor_y
    dvl_y1 = dvl_y0 + 96
    if not dvl_connected:
        dvl_fill = (38, 28, 28)
        dvl_border = RED
        dvl_status = "DISCONNECTED"
    elif dvl_valid:
        dvl_fill = (24, 38, 24)
        dvl_border = GREEN
        dvl_status = "OK"
    else:
        dvl_fill = (42, 36, 18)
        dvl_border = ORANGE
        dvl_status = "STALE"

    box(img, inner_x0, dvl_y0, inner_x1, dvl_y1, dvl_fill, dvl_border, 1)
    text(img, "DVL", (inner_x0 + 10, dvl_y0 + 22), 0.58, WHITE, 1)
    dvl_source_text = dvl_source if dvl_source else "N/A"
    text(img, "Status: %s   Src: %s" % (dvl_status, dvl_source_text),
         (inner_x0 + 10, dvl_y0 + 48), 0.43, dvl_border, 1)
    text(img, "Err: %s   Q: %s" % (_fmt_or_na(dvl_vel_err, "%.3f"),
                                  _fmt_or_na(dvl_fix_quality, "%.1f")),
         (inner_x0 + 10, dvl_y0 + 70), 0.43, dvl_border, 1)
    dvl_vel_color = WHITE if dvl_valid else dvl_border
    text(img, "Vx %s  Vy %s  Vz %s" % (_fmt_or_na(dvl_vx, "%+.2f"),
                                      _fmt_or_na(dvl_vy, "%+.2f"),
                                      _fmt_or_na(dvl_vz, "%+.2f")),
         (inner_x0 + 10, dvl_y0 + 92), 0.43, dvl_vel_color, 1)

    gnss_y0 = dvl_y1 + 8
    gnss_y1 = gnss_y0 + 112
    gnss_fill = (24, 34, 24) if gnss_connected else (38, 24, 24)
    gnss_border = GREEN if gnss_connected else RED
    gnss_status = "OK" if gnss_connected else "DISCONNECTED"
    sats_text = str(sats) if sats is not None else "N/A"

    box(img, inner_x0, gnss_y0, inner_x1, gnss_y1, gnss_fill, gnss_border, 1)
    text(img, "GNSS", (inner_x0 + 10, gnss_y0 + 22), 0.58, WHITE, 1)
    text(img, "Status: %s  Sats %s" % (gnss_status, sats_text),
         (inner_x0 + 10, gnss_y0 + 44), 0.40, gnss_border, 1)
    text(img, "Lat %s  Lon %s" % (_fmt_or_na(lat, "%.6f"),
                                  _fmt_or_na(lon, "%.6f")),
         (inner_x0 + 10, gnss_y0 + 62), 0.40, WHITE, 1)
    text(img, "Lat0 %s  Lon0 %s" % (_fmt_or_na(lat0, "%.6f"),
                                    _fmt_or_na(lon0, "%.6f")),
         (inner_x0 + 10, gnss_y0 + 80), 0.40, WHITE, 1)
    text(img, "X East %s m  Y North %s m" % (_fmt_or_na(local_x, "%+.2f"),
                                            _fmt_or_na(local_y, "%+.2f")),
         (inner_x0 + 10, gnss_y0 + 98), 0.40, WHITE, 1)

    orient_y0 = gnss_y1 + 8
    orient_y1 = orient_y0 + 72
    orient_fill = (24, 34, 24) if heading_connected else (38, 24, 24)
    orient_border = GREEN if heading_connected else RED

    box(img, inner_x0, orient_y0, inner_x1, orient_y1, orient_fill,
        orient_border, 1)
    text(img, "Orientation", (inner_x0 + 10, orient_y0 + 22), 0.58, WHITE, 1)
    text(img, "Heading %s" % _fmt_or_na(heading, "%.1f"),
         (inner_x0 + 10, orient_y0 + 40), 0.43, orient_border, 1)
    text(img, "Pitch %s  Roll %s" % (_fmt_or_na(compass_pitch, "%.1f"),
                                    _fmt_or_na(compass_roll, "%.1f")),
         (inner_x0 + 10, orient_y0 + 60), 0.43, WHITE, 1)

    control_inner_x0 = control_x0 + 10
    control_bar_x1 = control_x1 - 36
    text(img, "Command", (control_inner_x0, main_y0 + 44),
         0.62, WHITE, 1)
    value_bar(img, "Yaw", right_x, control_inner_x0, control_bar_x1,
              main_y0 + 82)
    value_bar(img, "Thrust", left_y, control_inner_x0, control_bar_x1,
              main_y0 + 152)
    text(img, "Pitch cmd: %+.2f" % (right_y / 100.0),
         (control_inner_x0, main_y0 + 222), 0.58, WHITE, 1)

    bottom_x0 = 16
    bottom_x1 = 964
    bottom_y0 = 512
    bottom_y1 = 606
    box(img, bottom_x0, bottom_y0, bottom_x1, bottom_y1, PANEL, BORDER, 1)
    text(img, "Actuators", (bottom_x0 + 10, bottom_y0 + 22), 0.58, WHITE, 1)

    inner_bottom_x0 = 26
    inner_bottom_y0 = bottom_y0 + 26
    tile_gap = 8
    tile_h = 58
    thrust_w = 178
    inner_bottom_x1 = bottom_x1 - 10
    fin_w = (inner_bottom_x1 - inner_bottom_x0 - thrust_w - 4 * tile_gap) // 4
    tile_y1 = inner_bottom_y0 + tile_h

    thrust_color = GREEN if thrust else WHITE
    actuator_tile(img, inner_bottom_x0, inner_bottom_y0,
                  inner_bottom_x0 + thrust_w, tile_y1, "Thrust", thrust,
                  thrust_color)

    fin_values = list(fins)[:4]
    while len(fin_values) < 4:
        fin_values.append(0)

    x = inner_bottom_x0 + thrust_w + tile_gap
    for i in range(4):
        actuator_tile(img, x, inner_bottom_y0, x + fin_w, tile_y1,
                      "Fin%d" % (i + 1), fin_values[i], WHITE)
        x += fin_w + tile_gap

    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


if __name__ == '__main__':
    ip = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
    config = LabraxTCPConfig(ip=ip)
    robot = Labrax(config)
    joystick = Joystick()
    lat0 = None
    lon0 = None
    last_command = None
    last_disarmed_send = 0.0
    cv2.namedWindow("LABRAX", cv2.WINDOW_AUTOSIZE)

    robot.start()
    try:
        while True:
            loop_start = time.time()
            joystick.update()

            if joystick.start_pressed:
                robot.arm()
            if joystick.select_pressed:
                _safe_disarm(robot)
                last_disarmed_send = 0.0

            thrust_input = _stick_to_unit(joystick.left_y) if joystick.connected else 0.0
            yaw_input = _stick_to_unit(joystick.right_x) if joystick.connected else 0.0

            if robot.armed:
                pitch_input = 1.0 if thrust_input < 0.0 else thrust_input
                thrust, fins = _command_outputs(config, thrust_input, yaw_input, pitch_input)
                try:
                    last_command = robot.set_normalized(thrust_input, yaw_input, pitch_input)
                except Exception:
                    last_command = None
            else:
                pitch_input = 0.0
                thrust = 0
                fins = [config.fins_neutral] * 4
                last_command = None
                if loop_start - last_disarmed_send >= DISARMED_SEND_DT:
                    try:
                        robot.set_normalized(0.0, 0.0, 0.0)
                    except Exception:
                        pass
                    last_disarmed_send = loop_start

            imu = robot.imu.data
            dvl = robot.dvl.data
            gps = robot.gps.data
            battery = robot.battery.data

            lat = gps.latitude
            lon = gps.longitude
            if lat0 is None and lat is not None and lon is not None:
                lat0 = lat
                lon0 = lon
            local_x, local_y = _local_xy(lat, lon, lat0, lon0)

            neutral = config.fins_neutral
            if last_command is not None:
                fins = _fins_list(last_command, neutral)
                thrust = last_command.thrust
            command_thrust_display = thrust_input * 100.0 if robot.armed else 0.0
            command_yaw_display = yaw_input * 100.0 if robot.armed else 0.0
            command_pitch_display = pitch_input * 100.0 if robot.armed else 0.0

            ui_rgb = draw_ui(
                command_thrust_display,
                command_yaw_display,
                command_pitch_display,
                thrust,
                fins,
                joystick.connected,
                robot.armed,
                imu.heading,
                robot.imu.connected,
                imu.pitch,
                imu.roll,
                robot.dvl.connected,
                dvl.valid,
                dvl.stale,
                dvl.velocity_source,
                dvl.fix_quality,
                dvl.vx,
                dvl.vy,
                dvl.vz,
                dvl.vel_err,
                robot.gps.connected,
                lat,
                lon,
                lat0,
                lon0,
                local_x,
                local_y,
                gps.satellites,
                robot.battery.connected,
                battery.percent,
            )

            cv2.imshow("LABRAX", cv2.cvtColor(ui_rgb, cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break

            sleep_s = LOOP_DT - (time.time() - loop_start)
            if sleep_s > 0:
                time.sleep(sleep_s)
    finally:
        _safe_disarm(robot)
        try:
            robot.stop()
        except Exception:
            pass
        joystick.close()
        cv2.destroyAllWindows()
