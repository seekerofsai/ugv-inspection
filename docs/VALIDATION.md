# Validation

Validated locally on macOS with Python 3.10.7. Core simulation has no third-party Python
dependencies. No physical robot was connected and no ROS runtime was started.

- 14 unittest checks passed, including nominal coverage, temporary dropout, stale/frozen
  frames, permanent dropout, navigation failure, blur, moving base, stale odometry,
  command/rate limits, interrupted cleanup, reproducibility and output preservation.
- Six scenarios × three seeds: all 18 runs matched expected success/failure outcomes.
  These are software regression tests with a simple simulator, not hardware statistics.
- Verified example: three captures at 9.1, 13.3 and 15.1 simulated seconds; temporary
  visual loss triggered reacquisition, with observed recovery latency about 2.1 seconds.
- Interactive HTML tested in the browser: route/camera diagrams, Play/Pause, timeline
  progress and final COMPLETE state with three captures.
- FFmpeg created an H.264 MP4, 480×320, 151 frames, 15.1 seconds; verified using ffprobe.
  Frames carry a SIMULATION label. Captured PNGs and HTML are synthetic evidence.
- All Python modules compile, including the ROS adapter. ROS dependencies, actual
  image detection, real pan–tilt commands and navigation have not been exercised.

The simulator uses a simple differential-drive approach controller, bounded servo
motion, geometric target projection and a motion-based sharpness proxy. It has no
collision geometry, realistic lens model, motor dynamics, slip or photometric physics.
The hardware adapter uses real image marker detection and ROI sharpness, so thresholds
must be calibrated again on hardware. It does not implement equipment defect detection.
