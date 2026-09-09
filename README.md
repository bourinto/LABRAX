# LABRAX embedded control

This repository contains the Python software used to control and study the
LABRAX autonomous underwater vehicle (AUV). It provides:

- TCP drivers for the thruster, fins, IMU/depth sensor, DVL, GNSS, and battery;
- helpers for running timed missions and recording CSV logs;
- experimental missions for forward and reverse manoeuvres;
- a joystick interface for manual operation; and
- a tool for reconstructing a 3D trajectory from a mission log.

> **Safety:** several scripts command a real thruster and control surfaces.
> Mission scripts arm the software as soon as their `Mission` context starts.
> Secure the vehicle, keep an independent emergency stop available, check the
> configured IP address and ports, and review all command values before running
> a script on hardware.

## Repository map

| Path | Purpose |
| --- | --- |
| `drivers/` | Reusable TCP, sensor, actuator, vehicle, mission, and logging code |
| `UI.py` | Python 3 joystick control and sensor dashboard |
| `short_mission.py` and other root scripts | Python 2.7 experimental missions |
| `analysis/trajectory_dvl.py` | Python 3 reconstruction and display of a logged 3D trajectory |
| `docs/drivers-user-guide.md` | Detailed driver API, examples, protocols, and program catalog |

## Runtime requirements

The driver package itself uses only the Python standard library. The current
mission runner and mission scripts target **Python 2.7**. The two Python 3 tools
need additional packages:

```bash
# Manual-control UI
python3 -m pip install numpy opencv-python

# Offline trajectory analysis
python3 -m pip install numpy pandas matplotlib scipy
```

Run commands from the repository root so that Python can import `drivers`.
There is currently no package installer or pinned dependency file.

## Basic use

The drivers connect to TCP services on `127.0.0.1` by default. Use the robot or
gateway IP when those services run on another host.

Start the manual UI in a Linux graphical session:

```bash
python3 UI.py 192.168.2.1
```

The UI starts disarmed. Press **START** to arm, **SELECT** to disarm, and **Q**
or **Esc** to exit. It expects a Linux joystick at `/dev/input/js0`.

After reviewing its constants and safety conditions, a mission can be launched
with Python 2.7:

```bash
python2.7 short_mission.py
```

Mission scripts use localhost unless an `ip` is passed to `Mission(...)`. They
write timestamped CSV logs beside the script; CSV files are ignored by Git.

Plot a recorded trajectory with Python 3:

```bash
python3 analysis/trajectory_dvl.py path/to/mission_log.csv --cylinder
```

See the [driver user guide](drivers-user-guide.md) before creating a
mission or connecting the software to the vehicle.

## License and attribution

Copyright © 2026 [bourinto](https://github.com/bourinto).

This project is free and open-source software under the
[BSD 3-Clause License](LICENSE). It may be used, modified, forked, and
redistributed, including commercially, provided that the copyright notice,
license conditions, and disclaimer are preserved as required by the license.

Publications, videos, figures, datasets, or other content based on this project
should also cite it. Citation metadata is provided in [`CITATION.cff`](CITATION.cff),
and the suggested short form is:

> bourinto, *LABRAX Embedded Control*, 2026,
> <https://github.com/bourinto/LABRAX>.

This attribution must not imply endorsement by the author.
