# RPi5 Acceptance Testing

All motion tests start with the chassis secured and every wheel clear of the
ground. Complete the ECU calibration procedure before setting
`vehicle.calibrated: true`.

## Build and interface checks

```bash
sudo apt install python3-colcon-meson meson ninja-build \
  python3-jinja2 python3-ply python3-yaml \
  libdw-dev libevent-dev libgnutls28-dev libunwind-dev libyaml-dev
source /opt/ros/jazzy/setup.bash
vcs import src --skip-existing < dependencies.repos
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
  --skip-keys=libcamera
colcon build --symlink-install --meson-args \
  -Dpipelines=rpi/pisp -Dipas=rpi/pisp \
  -Dtest=false -Ddocumentation=disabled -Dpycamera=disabled \
  -Dgstreamer=disabled -Dv4l2=disabled
source install/setup.bash
colcon test --packages-select vc_description vc_safety vc_bringup
colcon test-result --verbose
```

After launch, verify the ECU contract and rates:

```bash
ros2 node list
ros2 topic list -t
ros2 topic info --verbose /imu/data_raw
ros2 topic info --verbose /odom
ros2 topic hz /imu/data_raw
ros2 topic hz /odom
ros2 topic hz /joint_states
ros2 topic hz /diagnostics
ros2 topic echo /vehicle/safety/diagnostics
ros2 topic echo /vehicle/diagnostics
ros2 topic echo /vehicle/safety/statistics
ros2 lifecycle get /safety_gate
```

Expected rates are IMU 50 Hz, odometry and JointState 30 Hz, and diagnostics
5 Hz. Raw sensor topics must be best-effort, volatile, depth 1. Commands and
diagnostics must be reliable, volatile, depth 1.

## IMX219 camera checks

Verify libcamera before starting ROS, then check the standard camera contract:

```bash
rpicam-hello --list-cameras
ros2 launch vc_bringup camera.launch.py \
  camera_config:=$(ros2 pkg prefix vc_bringup)/share/vc_bringup/config/camera.yaml
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/camera_info
ros2 topic echo --once /camera/image_raw/compressed --field format
```

The default profile is 1280x720 at 30 Hz and uses
`camera_optical_frame`; the compressed transport must report `jpg`. A missing
calibration file produces zero intrinsics;
calibrate with `camera_calibration` before using rectification, projection, or
metric vision:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/camera/image_raw camera:=/camera
```

Replace the checkerboard dimensions with the printed target's actual inner
corner count and square size. Save the resulting YAML at the location configured
by `camera_info_url`.

## TF checks

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link imu_link
ros2 run tf2_tools view_frames
```

There must be exactly one `odom -> base_link` publisher. Static transforms must
match the measured configuration.

## Safe command test

Publish requests first, then explicitly enable motion:

```bash
ros2 topic pub -r 10 /cmd_vel_request geometry_msgs/msg/Twist \
  '{linear: {x: 0.05}, angular: {z: 0.0}}'

ros2 service call /vehicle/motion_enable std_srvs/srv/SetBool '{data: true}'
```

Stop the request publisher. The gate must publish zero after 250 ms and the ECU
must be stopped within its 500 ms watchdog deadline. Restarting the Agent,
unplugging USB, rebooting the ECU, or regressing a timestamp must leave
`/vehicle/motion_enabled` false until a new enable request succeeds.

## rosbag2 evidence

Launch with `record:=true`. The recorder uses MCAP, the ECU-compatible QoS
override file, and stores the git revision and vehicle configuration SHA-256 in
bag metadata. For a bounded flight recorder, also set `snapshot_mode:=true` and
call `/rosbag2_recorder/snapshot` after the fault.

Record Agent absent at boot, Agent termination, USB disconnect/reconnect, ECU
reboot, epoch synchronization failure, IMU failure, command timeout, and encoder
stall as separate runs. Confirm monotonic source stamps, no stale command replay,
and a disabled gate after every recovery.

For replay verification, run the replay profile in an isolated ROS domain and
confirm that `/cmd_vel` has no publisher. Recorded gated commands must appear
only on `/replay/recorded/cmd_vel` (or the equivalent namespaced path).
