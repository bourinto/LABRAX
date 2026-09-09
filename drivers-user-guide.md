# LABRAX driver user guide

This guide explains how the LABRAX Python drivers are organised, how to use
them safely, and what each existing executable program is intended to do. All
examples assume they are run from the repository root.

## 1. Architecture at a glance

The software has three layers:

```text
Mission scripts / UI.py
          |
          v
   Mission or Labrax       -----> CSV mission log
          |
          +---- actuators: thruster and fins ----> TCP services
          |
          +---- sensors: IMU, DVL, GNSS, battery <---- TCP streams
```

- `Labrax` is the main vehicle facade. It owns all sensor and actuator drivers
  and applies a software arm/disarm gate to commands.
- `Mission` adds a fixed-rate loop, CSV logging, timed actions, and automatic
  cleanup around `Labrax`.
- The individual drivers expose the underlying TCP devices when lower-level
  access is necessary.

The TCP services may be hardware gateways or simulator endpoints; the Python
code sees the same interface in either case.

## 2. Safety and important behaviour

Read these points before sending a command:

1. `Labrax.arm()` only changes an in-process Boolean. It does not verify sensor
   health, establish a physical safety interlock, or wait for a device reply.
2. Entering `with Mission(...)` starts the sensor threads **and immediately
   arms the vehicle**. Existing missions then wait one second, but this delay is
   not a connection or data-validity test.
3. Sensor fields initially contain `None`. Test both `driver.connected` and the
   required data values before using feedback in a new control law.
4. Thruster and fin commands travel over separate TCP connections. A partial
   failure is possible: one device may receive its command while the other
   raises an exception.
5. Calling `ThrustDriver` or `FinsDriver` directly bypasses the `Labrax` arming
   gate. Prefer `Labrax.set_normalized()` or `Labrax.set_raw()`.
6. `Labrax.stop()` makes best-effort calls to stop the thruster, centre the
   fins, stop sensors, and close sockets. It deliberately suppresses errors,
   so an independent hardware emergency stop remains necessary.

Use conservative limits during bench testing, keep propulsion physically safe,
and check the sign conventions on the actual vehicle before deployment.

## 3. Python versions and dependencies

### Vehicle missions

The root mission files explicitly target Python 2.7. The `Mission` logger also
opens CSV output in the Python 2 binary mode, so it should currently be treated
as a Python 2.7 API. The drivers use only standard-library modules.

### Operator UI

`UI.py` targets Python 3 on Linux and requires NumPy and OpenCV:

```bash
python3 -m pip install numpy opencv-python
```

It reads Linux joystick events from `/dev/input/js0` and needs a graphical
session for the OpenCV window.

### Offline analysis

`analysis/trajectory_dvl.py` targets Python 3 and requires:

```bash
python3 -m pip install numpy pandas matplotlib scipy
```

Dependencies are not pinned in this repository, so record tested versions when
reproducing an experiment.

## 4. Connection configuration

Create a configuration and pass it to `Labrax`:

```python
from drivers import Labrax, LabraxTCPConfig

config = LabraxTCPConfig(ip='192.0.2.10')
robot = Labrax(config)
```

`192.0.2.10` is only an example address. The defaults are:

| Setting | Default | Direction | Role |
| --- | ---: | --- | --- |
| `ip` | `127.0.0.1` | — | Host running all TCP services |
| `port_thrust` | `5011` | output | Thruster command service |
| `port_fins` | `5012` | output | Fin command service |
| `port_imu` | `5006` | input | Attitude, temperature, and depth stream |
| `port_dvl` | `5007` | input | DVL velocity and distance stream |
| `port_gnss` | `5009` | input | NMEA GGA stream |
| `port_battery` | `5010` | input | Battery percentage stream |
| `socket_timeout_s` | `2.0` | — | Connected socket read timeout |
| `connect_timeout_s` | `2.0` | — | TCP connection timeout |
| `reconnect_delay_s` | `0.2` | — | Sensor retry delay after disconnection |
| `thrust_max` | `1000000` | — | Absolute raw thrust limit |
| `fins_min` | `5` | — | Minimum raw fin value |
| `fins_max` | `250` | — | Maximum raw fin value |
| `fins_neutral` | `128` | — | Centred fin value |
| `fins_amplitude` | `122` | — | Normalized-to-raw fin scale |

All values can be overridden in the constructor:

```python
config = LabraxTCPConfig(
    ip='192.0.2.10',
    port_dvl=6007,
    socket_timeout_s=1.0,
    thrust_max=250000,
)
```

`TCPClient` opens connections lazily. Sensor calls to `start()` cause their
background threads to connect through `recv()`; actuator sockets connect when
the first command is sent. Actuator sends are tried at most twice, reopening
the socket after the first failure.

## 5. Using the `Labrax` vehicle facade

### Read sensors without arming

The context manager starts all sensor threads and stops everything on exit. It
does not arm automatically:

```python
import time

from drivers import Labrax, LabraxTCPConfig

config = LabraxTCPConfig(ip='192.0.2.10')
with Labrax(config) as robot:
    time.sleep(1.0)
    if robot.imu.connected:
        imu = robot.imu.data
        print(imu.heading, imu.pitch, imu.roll, imu.depth)
```

`robot.imu.data` is the most recent complete data object. The other current
snapshots are `robot.dvl.data`, `robot.gps.data`, and `robot.battery.data`.

### Send a normalized command

This example performs explicit cleanup even when an exception occurs. It will
move real hardware and must only be used in a prepared test environment:

```python
from drivers import Labrax, LabraxTCPConfig

robot = Labrax(LabraxTCPConfig(ip='192.0.2.10'))
robot.start()
try:
    robot.arm()
    command = robot.set_normalized(thrust=-0.2, yaw=0.0, pitch=0.0)
    print(command.thrust, command.fins.top, command.fins.left)
finally:
    try:
        robot.disarm()
    finally:
        robot.stop()
```

The normalized inputs are clipped to `[-1, 1]`:

| Input | Meaning in project scripts | Conversion |
| --- | --- | --- |
| `thrust` | positive forward, negative reverse | `int(input * thrust_max)` |
| `yaw` | steering command | top and bottom fins: `neutral + amplitude * yaw` |
| `pitch` | pitch command | left and right fins: `neutral - amplitude * pitch` |

The signs above describe how the existing mission code uses the inputs. Verify
the resulting physical motion after any wiring, firmware, or calibration
change. `FinsDriver` rounds calculated positions and clips every raw fin value
to `[fins_min, fins_max]`.

`set_normalized()` and `set_raw()` return a `MotionCommand` containing the raw
thruster value and a `FinsCommand`. The normalized method reports the clipped
fin values actually sent. The raw method's returned fin values are the caller's
integer requests even though the wire values are clipped inside `FinsDriver`.
When disarmed, either method instead sends zero thrust and neutral fins.

### Lifecycle methods

| Method/property | Effect |
| --- | --- |
| `start()` | Starts the four sensor background threads |
| `arm()` | Enables commands through the software gate |
| `armed` | Reports that gate's current state |
| `disarm()` | Clears the gate and sends zero thrust plus neutral fins |
| `neutralize()` | Sends zero thrust plus neutral fins without changing the arm flag |
| `stop()` | Disarms, neutralizes, stops sensor threads, and closes actuator sockets |
| `set_normalized(thrust, yaw, pitch)` | Sends clipped normalized commands |
| `set_raw(thrust, top, bottom, left, right)` | Sends device-level values |

## 6. Actuator drivers and wire formats

### Thruster

`ThrustDriver.set(value)` clips the requested raw value to
`[-thrust_max, thrust_max]`, negates it for the device protocol, and sends:

```text
V=<negated raw value>\r\nG\r\n
```

For example, a driver request of `250000` sends `V=-250000`. This protocol
negation is already included in the driver; callers must not apply it again.
`stop()` is equivalent to `set(0)`.

### Fins

`FinsDriver.set_raw(top, bottom, left, right)` sends four three-byte records:

```text
FF 01 <top>  FF 02 <bottom>  FF 03 <left>  FF 04 <right>
```

Values are integer-clipped before they are placed in the packet.
`set_normalized(yaw, pitch)` calculates the four values described in the
previous section and returns the values actually sent. `neutral()` sends
`fins_neutral` to all four channels.

## 7. Sensor drivers and data

Every sensor driver has the same basic interface:

```python
sensor.start()
connected = sensor.connected
latest = sensor.data
sensor.stop()
```

`start()` is idempotent and launches a daemon thread. After a socket error the
thread closes the connection, waits `reconnect_delay_s`, and tries again.
`stop()` closes the socket and joins the thread.

The `connected` property means that at least one non-empty chunk has been read
since the most recent socket error. It does not guarantee that a valid frame
has been parsed, and it does not provide an age or freshness check. Malformed
frames are ignored.

### IMU and depth

`IMUDriver` finds frames beginning with `$` and ending at `*` plus two trailing
characters. `parse_compass()` requires all five labelled fields:

```text
$C<heading>P<pitch>R<roll>T<temperature>D<depth>*xx
```

It publishes `IMUData` with `heading`, `pitch`, `roll`, `temperature`, and
`depth`. The parser does not validate the two characters after `*` as a
checksum.

### DVL

`DVLDriver` accepts newline-delimited `$SON31` records. It publishes:

| `DVLData` field | Source field | Treatment |
| --- | ---: | --- |
| `timestamp` | 1 | Converted to `float` |
| `vx` | 4 | Converted to `float` and sign-negated |
| `vy` | 5 | Converted to `float` |
| `vz` | 6 | Converted to `float` |
| `DTB` | 10 | Converted to `float` |
| `DTS` | 11 | Converted to `float` |

The control and analysis code treats velocity as metres per second and distance
as metres. Confirm that the producing service uses those units.

### GNSS

`GPSDriver` accepts only `$GPGGA` sentences. It converts NMEA
degrees-and-decimal-minutes coordinates to signed decimal degrees and publishes
`GPSData(latitude, longitude, satellites)`. Southern and western coordinates
are negative. No fix-quality field is exposed, so consumers should not infer a
valid fix solely from `connected`.

### Battery

`BatteryDriver` accepts newline-delimited records beginning with `$OCEANA` and
parses the second comma-separated field as an integer percentage. It publishes
`BatteryData(percent)`. The value is not range-clipped.

### Initial and invalid data

Before the first valid record, all model fields are `None`. A valid frame
replaces the entire latest-data object; a malformed frame leaves the previous
object in place. New control code should therefore check required values and,
where possible, track freshness separately.

## 8. Running and logging a mission

`Mission` is a Python 2.7 context manager around `Labrax`:

```python
# Python 2.7
import time

from drivers import Mission

with Mission(__file__, ip='192.0.2.10', hz=20.0) as mission:
    time.sleep(1.0)
    mission.run_for(2.0, ut=-0.1, uy=0.0, up=0.0)

print('Log: %s' % mission.output_path)
```

Entering the context performs these actions in order:

1. starts all sensor threads;
2. sets the `Labrax` software arm flag;
3. opens the output CSV file and writes its header; and
4. records the mission start time.

Exiting attempts to disarm and stop the vehicle, then closes the log, including
when the body raises an exception.

### Constructor arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `script_path` | required | Used to derive the log directory and filename |
| `ip` | `127.0.0.1` | TCP service host |
| `hz` | `20.0` | Target period used by `send()` and helper loops |
| `include_dr` | `False` | Adds online dead-reckoning `x_dr`, `y_dr` columns |
| `output_path` | generated | Explicit CSV destination if supplied |

The default name is `<script-name>_<HHMMSS>.csv` beside the mission script.

### Command helpers

| Helper | Purpose |
| --- | --- |
| `send(ut, uy, up, ...)` | Sends one normalized command, logs one row, flushes the file, and normally waits for the remainder of the loop period |
| `run_for(duration_s, ut, uy, up, dr=None)` | Repeats a constant command until the duration expires |
| `wait_pitch(pitch_limit_deg=8, duration_s=10)` | Sends neutral commands until `abs(pitch)` is below the limit or the timeout expires |
| `yaw(imu=None)` | Returns heading as a float, or `None` |
| `pitch_reg(target_deg, scale_deg, pitch=None, invert=True)` | Returns a bounded hyperbolic-tangent pitch correction |

The three mission command names are `ut` (thrust), `uy` (yaw), and `up`
(pitch). Pass already-read `imu` and `dvl` snapshots to `send()` when a loop
must log exactly the measurements used to calculate its command.

`pitch_reg()` evaluates
`-tanh((target_deg - pitch) / scale_deg)` with its default `invert=True`. It
returns zero if pitch is unavailable. A zero scale is invalid.

### CSV columns

| Column | Content |
| --- | --- |
| `timestamp` | Seconds since the mission context was entered |
| `roll`, `pitch`, `yaw` | IMU attitude in degrees |
| `depth` | IMU/depth-sensor value |
| `vsurge`, `vsway`, `vheave` | Parsed DVL `vx`, `vy`, and `vz` |
| `DTS`, `DTB` | Parsed DVL distance fields |
| `ut`, `uy`, `up` | Normalized thrust, yaw, and pitch requests |
| `x_dr`, `y_dr` | Optional online dead-reckoned position |

Unavailable values are written as empty cells. The logger flushes after each
command, favouring recoverable experiment data over maximum write performance.

### Timed safety helper

`Security` bounds a loop by elapsed wall-clock time:

```python
from drivers import Security

security = Security(5.0)
security.init()
while security.check():
    pass
```

`check(exit=False)` returns `False` at the deadline. With `exit=True`, it calls
`sys.exit(1)` instead; a surrounding `Mission` context still performs its exit
cleanup. This helper is a software timeout, not a hardware watchdog.

`wrap_angle_deg(angle)` maps an angle to `[-180, 180)` and is useful for a
shortest-path heading error.

## 9. Online dead reckoning

`DeadReckoning` combines DVL velocity, IMU orientation, and measured depth:

```python
from drivers import DeadReckoning

dr = DeadReckoning()
dr.update(mission.imu, mission.dvl)
print(dr.x, dr.y, dr.z)
```

For each new DVL timestamp it:

1. transforms body velocity into the world frame with a ZYX
   yaw-pitch-roll rotation;
2. integrates horizontal velocity with the trapezoidal rule;
3. takes vertical position from measured depth; and
4. shifts the DVL position to the vehicle position using a longitudinal sensor
   offset.

By default `vz_ignored=True`, so DVL vertical velocity is set to zero during
the rotation. Set `DeadReckoning(vz_ignored=False)` to include it. Repeated
calls with the same DVL timestamp are not integrated twice. `reset()` clears
the position and integration history.

Two implementation differences matter when comparing online and offline
results:

- online `DeadReckoning` uses a DVL-to-vehicle offset of **1.08 m**, whereas
  `analysis/trajectory_dvl.py` currently uses **0.98 m**; and
- online estimation ignores DVL `vz` by default, whereas the offline analysis
  uses all three velocity components before replacing vertical position with
  measured depth.

These parameters must be reconciled or justified before treating the two
trajectories as equivalent experimental estimates.

## 10. Existing executable programs

The values in these files are experiment-specific. Read and adapt their
constants, termination conditions, and IP configuration before execution.

### Manual operation

| Program | Runtime | Purpose |
| --- | --- | --- |
| `UI.py [ip]` | Python 3 | Displays IMU, DVL, GNSS, battery, command, and actuator state; reads `/dev/input/js0`; START arms, SELECT disarms, and Q/Esc exits |

The UI maps left-stick Y to thrust and right-stick X to yaw. Its current pitch
command is coupled to thrust: reverse thrust requests `pitch=1.0`, while
non-reverse thrust uses the normalized thrust value as pitch. The right-stick Y
axis is read by `Joystick` but is not used by the main control loop.

### Mission scripts

| Program | Purpose |
| --- | --- |
| `short_mission.py` | Compact reverse-dive, pitch recovery, forward-cruise, and reverse-braking sequence; the clearest minimal `Mission.run_for()` example |
| `turn_backward.py` | Reverse dive followed by a sustained reverse turn and a short forward braking command |
| `circle_backward.py` | Reverse dive with pitch regulation, then a reverse circular turn terminated by a heading crossing or timeout |
| `evaluate_curvature.py` | Prompts for forward or backward motion, performs a regulated turn, and logs data for comparing curvature in both directions |
| `heave_test.py` | Performs a reverse dive, then sends and logs neutral commands to observe subsequent heave behaviour |
| `leaping.py` | Exercises an aggressive reverse-dive to forward-motion transition with pitch regulation |
| `regulation_depth.py` | Tests a PID-like depth controller after a reverse dive; logs depth, attitude, DVL data, and commands |
| `station_keeping.py` | Repeats dive, attitude recovery, heading-controlled cruise, braking, and passive-ascent phases while updating and logging online dead reckoning |

Run a reviewed mission from the repository root, for example:

```bash
python2.7 short_mission.py
python2.7 evaluate_curvature.py
```

These scripts do not accept a command-line IP address. Their `Mission(...)`
calls use `127.0.0.1`; pass `ip='actual-address'` in the source if the TCP
services are remote.

### Trajectory analysis

`analysis/trajectory_dvl.py` reads a mission CSV, rejects samples whose DVL
speed exceeds 3 m/s, rotates body-frame velocity into a world frame, integrates
it with the trapezoidal rule, replaces vertical position with measured depth,
applies the DVL offset, and displays a 3D trajectory. It also prints the final
position, approximate travelled distance, and retained sample count.

Its documented convention is NED-like: world X is north, world Y is east, and
world Z is depth-positive-down. Body X, Y, and Z are surge, sway, and heave;
attitude is applied as a ZYX yaw-pitch-roll rotation.

```bash
python3 analysis/trajectory_dvl.py path/to/log.csv
python3 analysis/trajectory_dvl.py path/to/log.csv --cylinder
```

`--cylinder` draws vehicle bodies at sampled poses in addition to the trajectory
and body-axis arrows. The input must contain `timestamp`, `depth`, `vsurge`,
`vsway`, `vheave`, `yaw`, `pitch`, and `roll` columns.

## 11. Driver module reference

| Module | Responsibility |
| --- | --- |
| `drivers/config.py` | TCP endpoints, timeouts, and actuator limits |
| `drivers/core.py` | `Endpoint` and reconnecting/locked `TCPClient` primitives |
| `drivers/models.py` | Small sensor data containers |
| `drivers/parsers.py` | Clamping and IMU, NMEA GGA, and SON31 parsing |
| `drivers/sensors.py` | Background sensor connections and latest-value publication |
| `drivers/actuators.py` | Thruster and fin protocol encoding |
| `drivers/labrax.py` | Whole-vehicle lifecycle, arming gate, and command interface |
| `drivers/mission.py` | Mission timing, CSV logging, regulation helpers, and dead reckoning |
| `drivers/__init__.py` | Public imports exposed by `from drivers import ...` |

The public package exports `Labrax`, `LabraxTCPConfig`, `MotionCommand`,
`FinsCommand`, `Mission`, `DeadReckoning`, `Security`, `wrap_angle_deg`, all four
data models, and all six device drivers.

## 12. Troubleshooting

### A sensor stays disconnected

- Confirm that the configured host and sensor port are reachable.
- Confirm that the service is a TCP server and is producing the expected frame
  type.
- Remember that each sensor uses a different port.
- A sensor thread retries automatically after socket errors; no manual restart
  should be required.

### A sensor is connected but fields are `None`

The TCP connection may be carrying incomplete or unsupported records. Check the
formats in section 7. GNSS accepts `$GPGGA`, not arbitrary NMEA sentences, and
the IMU parser requires every labelled field.

### Data stops changing while `connected` remains true

The current API has no sample-age indicator for IMU, GNSS, or battery data.
Add freshness tracking in safety-critical control code and inspect the source
TCP stream for repeated or malformed records.

### Commands have the wrong physical sign

Verify the actuator gateway and vehicle calibration with a low-energy secured
test. Remember that the thrust driver negates raw commands on the wire and the
normalized fin driver negates pitch internally.

### The UI cannot find the joystick

Check that the controller appears as `/dev/input/js0`, that the user has read
permission, and that its Linux axis/button numbering matches the constants at
the top of `UI.py`.

### No log is produced

`Mission` opens its output only when entering the `with` block. Check write
permission in the script directory and use `output_path=` when another location
is required. Generated `*.csv` files are intentionally ignored by Git.

## 13. License and attribution

The project is distributed under the BSD 3-Clause License. Source and binary
redistributions, including modified versions and forks, must preserve the
copyright notice, license conditions, and disclaimer found in `LICENSE`.

Content based on LABRAX should credit the project using `CITATION.cff` or the
short citation given in the root `README.md`. Attribution does not grant
permission to imply that bourinto endorses a derived project or its results.
