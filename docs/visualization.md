# Visualization and simulation guide

## Tool roles

- `RViz2`: URDF, TF, filtered odometry, wheel state, and camera display.
- `rqt_graph`: node and topic connection inspection.
- `ros2 bag`: existing MCAP recording and isolated replay.
- Gazebo Sim: optional `/sim` vehicle physics and sensor simulation.

Tools are installed on development PC. RPi5 production deployment keeps the
GUI and Gazebo packages out of `vehicle_computer`.

```bash
sudo apt install ros-jazzy-rviz2 ros-jazzy-rqt-graph ros-jazzy-ros-gz
```

## Live hardware

Start hardware bringup on RPi5:

```bash
ros2 launch vc_bringup vehicle.launch.py \
  serial_device:=/dev/serial/by-id/<device>
```

Set the same dedicated domain on approved PC and RPi5 network:

```bash
export ROS_DOMAIN_ID=42
ros2 launch vc_visualization rviz.launch.py
```

RViz2 subscribes to:

```text
/tf
/tf_static
/joint_states
/odometry/filtered
/camera/image_raw/compressed
```

The default display fixed frame is `odom`. RViz2 and rqt_graph must not be used
as command publishers during live hardware tests.

## Fake stack

```bash
ros2 launch vc_visualization fake_visualization.launch.py
```

This starts fake ECU, state estimation, safety, diagnostics, and RViz2. It does
not start `micro_ros_agent` or open a serial device.

## MCAP replay

```bash
ros2 launch vc_visualization replay_visualization.launch.py \
  bag_path:=<bag-directory>
```

Replay uses `use_sim_time=true`. Recorded filtered odometry and TF are displayed
under `/replay/recorded`. The existing replay launch remaps the safety output to
`replay/cmd_vel_sink`; it never creates a hardware `/cmd_vel` publisher.

## Gazebo Sim

```bash
ros2 launch vc_visualization simulation.launch.py
```

Simulation uses namespace `/sim` and the fake vehicle dimensions by default:

```text
/sim/cmd_vel_request
/sim/cmd_vel
/sim/odom
/sim/imu/data_raw
/sim/joint_states
/sim/diagnostics
/sim/odometry/filtered
/sim/tf
/sim/tf_static
```

Gazebo receives `/sim/cmd_vel` through `ros_gz_bridge`. Gazebo odometry, IMU,
joint state, and clock are bridged back to ROS 2. The safety gate and EKF remain
the ROS-side authorities; Gazebo does not publish `odom -> base_link` TF.

Motion requires the same explicit enable sequence as hardware:

```bash
ros2 service call /sim/vehicle/motion_enable std_srvs/srv/SetBool '{data: true}'
ros2 topic pub -r 10 /sim/cmd_vel_request geometry_msgs/msg/Twist \
  '{linear: {x: 0.05}, angular: {z: 0.0}}'
```

Stop the command publisher or set the simulation health node parameter
`healthy:=false`; the safety gate must clear enable and publish zero.

`rqt_graph` is optional and can be opened in any running profile:

```bash
ros2 launch vc_visualization graph.launch.py
```
