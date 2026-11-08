# UGV inspection demonstration

A scripted baseline for a rover that approaches an equipment panel, points its camera,
captures three required views, and recovers from temporary loss of inspection images.
No policy training is required. Intended hardware: Waveshare UGV Rover PT PI5 ROS2 Kit.

Run the complete **synthetic demonstration now** on a Mac or DGX Spark. Connect the
ROS 2 adapter after the rover arrives and has been commissioned. The simulator is a
small kinematic model with projected target boxes, not Isaac Lab/Gazebo, a trained
policy, or a validated model of the rover. Its images are synthetic. Hardware support
is implemented against public vendor interfaces but has not been tested on a robot.

## Run now

Python 3.10+ is the only dependency for simulation, tests and HTML reports.
No GPU, ROS, model downloads, Python packages, or service accounts are needed.

```bash
cd ~/repos/ugv-inspection
./scripts/check.sh
./scripts/demo.sh --scenario dropout --output runs/dropout-demo
```

Each output directory must be new, so existing results are never overwritten.
The default demo runs faster than real time; add `--realtime` to watch its timing.

On the Mac, open the interactive replay:

```bash
open runs/dropout-demo/replay.html
```

The replay runs locally without a server or external assets. It shows the route,
camera observations, timeline, captures and mission events. On the Spark, serve runs
on loopback and forward the port from the Mac:

```bash
# Spark (leave running)
cd ~/repos/ugv-inspection
python3 -m http.server 8083 --bind 127.0.0.1 --directory runs
```

```bash
# Mac (leave running)
ssh -N -o ExitOnForwardFailure=yes -L 8083:127.0.0.1:8083 saiseeker@spark-c64c
```

Open http://localhost:8083/dropout-demo/replay.html on the Mac.

## Capture a simulation video

Install system FFmpeg if it is missing (`sudo apt install ffmpeg` on Spark,
`brew install ffmpeg` on Mac). Then:

```bash
./scripts/record.sh runs/dropout-demo
```

This creates `runs/dropout-demo/simulation.mp4`, a synthetic inspection-camera replay.
The HTML replay also includes the base route, state transitions, and timestamps.
This is not a 3D photorealistic rover video. Video rendering is separate from the mission.

## Scenarios and evaluation

```bash
./scripts/demo.sh --scenario nominal --output runs/nominal
./scripts/demo.sh --scenario frozen-camera --output runs/frozen
./scripts/demo.sh --scenario permanent-dropout --output runs/permanent
./scripts/demo.sh --scenario navigation-failure --output runs/nav-failure
./scripts/demo.sh --scenario blurred-camera --output runs/blurred
./scripts/benchmark.sh --seeds 5 --output runs/benchmark
```

The last three scenarios deliberately fail (exit status 1), preserve partial evidence,
and stop. Nominal, temporary dropout and frozen-camera scenarios should complete.
The benchmark returns success when all outcomes match their expectations; it does not
call an intentionally failed mission successful. Seeds perturb starting position and
observation noise only; these trials are software regression checks, not evidence of
real-world robustness.

The dropout begins while aligning the second target, lasts two seconds by default,
and interrupts only inspection frames. Navigation sensing is independent. The frozen
scenario repeats the last camera frame with its original timestamp. Repeated or stale
frames cannot satisfy the capture dwell.

Every run saves:

- `config.json`: exact mission parameters.
- `report.json`: success/failure, captures, capture quality, events and injected faults.
- `telemetry.jsonl`: timestamps, pose, commands, state and observed target boxes.
- `captures/target-N.png`: the exact accepted frame for each target.
- `replay.html`: self-contained interactive report, with local capture image links.

## Mission and extension points

`NAVIGATE → ACQUIRE → ALIGN → STABILIZE → capture → next target → COMPLETE`

Missing/stale targets enter `REACQUIRE` and a bounded pan sweep. Motion or stale
odometry enters `SETTLE_BASE`. Timeouts and navigation failures enter `FAILED`.
A capture requires a fresh, fully visible, sufficiently large, centered, sharp target,
low image motion, a stationary base and a continuous stable dwell. There is one
accepted image per required target; no false success from partial coverage.

Edit `config/panel.json` for the map waypoint, target IDs, simulation target positions,
limits and thresholds. Coordinates are meters; angles are radians; time is seconds.
Camera pan is positive toward image right; tilt is positive up in the abstract interface.
The hardware configuration maps these conventions to actual servos. Simulation target
world coordinates are not used by the controller or hardware detector.

- `src/ugv_inspection/core.py`: mission logic and backend interface.
- `src/ugv_inspection/sim.py`: kinematics, synthetic observations and fault injection.
- `src/ugv_inspection/ros2.py`: Nav2, image detection, odometry and vendor pan–tilt adapter.
- `src/ugv_inspection/report.py`: HTML replay and structured results.
- `tests/test_mission.py`: end-to-end scenarios and capture/stop behavior.

A future learned policy can replace `Mission.command` alignment/search decisions while
preserving freshness, capture checks, bounds and the same evaluation reports. Policies
from Reachy or Microduck do not directly control this rover.

## When the rover arrives

Read [hardware integration](docs/HARDWARE.md) before enabling the hardware backend.
The repo defaults to simulation. It neither installs vendor firmware nor starts robot
services. No hardware commands have been sent during development.

## Copy to the Spark

From the Mac:

```bash
ssh saiseeker@spark-c64c 'mkdir -p ~/repos'
rsync -av --exclude runs --exclude __pycache__ --exclude .venv \
  ~/repos/ugv-inspection/ saiseeker@spark-c64c:~/repos/ugv-inspection/
```

Then use the same demo/test commands on Spark. The repository is initially local;
no GitHub remote or publication is created automatically.

## References

- [Waveshare kit](https://www.waveshare.com/ugv-rover-ros2-kit.htm?sku=28847)
- [Waveshare ROS 2 wiki](https://www.waveshare.com/wiki/UGV_Rover_PI_ROS2)
- [Vendor ROS workspace](https://github.com/waveshareteam/ugv_ws)

Vendor source reviewed: `f0b3ad9c16e7acb6d6145f261116d71bf6338f4a`, branch
`ros2-humble-develop`. No vendor source or mesh assets are copied into this repository.
