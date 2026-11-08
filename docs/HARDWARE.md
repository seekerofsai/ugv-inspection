# Connect the Waveshare rover

Hardware integration is **not yet validated**. The working demonstration and tests use
a synthetic simulator. The ROS adapter provides a concrete integration starting point;
commissioning requires the actual rover, its installed ROS image and calibrated camera.

## Verified vendor interface

At vendor commit `f0b3ad9c16e7acb6d6145f261116d71bf6338f4a`,
`src/ugv_main/ugv_bringup/ugv_bringup/ugv_driver.py` subscribes to:

| Interface | Message | Purpose |
| --- | --- | --- |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Base velocity command |
| `ugv/joint_states` | `sensor_msgs/msg/JointState` | Pan–tilt COMMAND, not measured joint feedback |

The driver uses joint names `pt_base_link_to_pt_link1` and `pt_link1_to_pt_link2`.
It reads positions in radians, converts them to degrees, and sends firmware command
T=134. This repo uses that ROS interface and never opens the serial port directly.

The adapter expects a Nav2 `NavigateToPose` action server, `/odom`, and the pan–tilt
camera's uncompressed `sensor_msgs/Image` stream. The vendor camera launcher and
vision examples use `/image_raw`; confirm on the shipped system. OAK-D Lite and the
pan–tilt RGB camera are distinct streams; select the one that actually moves with the
servos. All topic/action names are configurable.

## Prepare the system

1. Bring up the vendor base driver, camera, LiDAR, map/localization and Nav2 using the
   instructions for the shipped image. Establish working teleoperation and navigation
   before starting this mission. Keep the LiDAR fixed relative to the base.
2. Work inside the same ROS 2 Humble environment/container as those nodes. Source
   `/opt/ros/humble/setup.bash` and your actual vendor workspace `install/setup.bash`.
   Use its system Python, not a separate uv Python that cannot import ROS packages.
3. Ensure `rclpy`, `nav2_msgs`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `action_msgs`,
   `cv_bridge` and OpenCV with `cv2.aruco` are available. On an Ubuntu ROS installation
   the relevant packages include `ros-humble-cv-bridge`, `ros-humble-nav2-msgs`,
   `python3-opencv` and `python3-numpy`. Verify `cv2.aruco` in the actual container;
   do not replace its OpenCV/NumPy stack blindly with pip wheels.
4. Confirm the topics with `ros2 topic list -t` and `ros2 action list -t`.
5. Provide a local stop mechanism and verify the base firmware/command watchdog stops
   on lost commands. This Python adapter publishes zero velocity and cancels Nav2 on
   exit, but cannot provide a hardware stop after power loss or a killed process.
   Ensure the mission is the sole navigation/pan–tilt command source while running.

## Panel and calibration

Generate marker images inside that Python environment:

```bash
cd ~/repos/ugv-inspection
python3 scripts/make-panel.py --output runs/printable-panel
```

The initial hardware detector uses **ArUco DICT_4X4_50 IDs 0, 1, 2**, not AprilTags.
Retain the white borders and mount each marker adjacent to the corresponding inspection
feature. This prototype checks the marker's framing and sharpness; it does not classify
faults, read gauges, or prove the surrounding equipment is fully visible.

Create separate hardware configs:

```bash
cp config/panel.json config/panel.hardware.json
cp config/ros2.example.json config/ros2.hardware.json
```

Edit the waypoint `[x,y,yaw]` to a measured pose in your navigation map. Place the panel
at a reachable camera height. Set realistic `min_target_width`, centered tolerance and
hold duration. Confirm servo neutral positions, positive directions, usable travel and
cable clearance; set `pan_sign`, `tilt_sign`, `pan_zero`, `tilt_zero`, and mission limits.
Check the physical angles after applying signs and offsets stay inside hardware limits.

The image quality score is target-ROI Laplacian variance divided by
`laplacian_reference`, capped at 1. Calibrate the reference using sharp and blurred
samples from the actual camera and lighting; the default is only an initial value.
Use camera calibration/rectification upstream for the wide-angle lens. The adapter
uses normalized pixel error and does not perform lens rectification itself.

Image and odometry header timestamps must be nonzero and use the same synchronized ROS
clock as the adapter. Old, repeated and future-dated frames are rejected. The mission
requires fresh odometry even after Nav2 reports successful arrival. Odometry in the HTML
route view is illustrative; the goal is in the map frame and the replay does not apply TF.

Set `commissioned` to true only after completing these checks. Then run:

```bash
./scripts/hardware.sh \
  --mission config/panel.hardware.json \
  --hardware-config config/ros2.hardware.json \
  --output runs/hardware-first \
  --execute
```

This sends real navigation and pan–tilt commands. There is no simulated dropout injection
in hardware mode. For a later dropout test, mask only the inspection image stream with
a separate relay while keeping LiDAR, odometry, navigation and stop controls running.
The same mission logic will reacquire when fresh images return. A physical occluder can
also test target loss. Never mislabel replayed image timestamps as fresh.

## Remaining hardware work

- Validate shipped topic names, Nav2 version, ROS clocks and network behavior.
- Measure wheel/servo behavior and camera geometry; calibrate thresholds and signs.
- Test physical stopping, cancellation, and lost-camera behavior at low speed.
- Replace the simple simulator with a calibrated Gazebo/Isaac Lab model if needed.
- Extend marker inspection to equipment regions and validated defect/quality detectors.

The supplied vendor Gazebo model is not automatically imported or validated by this repo.
