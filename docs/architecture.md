# Architecture

## Runtime boundary

```text
teleop / navigation
        |
  /cmd_vel_request
        |
 lifecycle vc_safety <---- /diagnostics, /odom, /imu/data_raw
        |
     /cmd_vel
        |
micro_ros_agent ==== 921600 baud XRCE-DDS ==== ESP32 vehicle_ecu

/odom + /imu/data_raw ---> isolated robot_localization -> /odometry/filtered
/joint_states ----------> composed robot_state_publisher -> /tf
vehicle URDF ------------------------------------> /tf_static

IMX219 -> libcamera -> camera_ros -> /camera/image_raw + /camera/camera_info

ECU + safety + EKF diagnostics -> diagnostics mux -> diagnostic_aggregator
ROS subscription statistics -------------------> /vehicle/safety/statistics
```

The ESP32 remains responsible for the 50 Hz motor loop, wheel odometry and the
500 ms command watchdog. The RPi5 owns the URDF, state estimation, dynamic TF,
and the supervisory command gate. Only `vc_safety` publishes `/cmd_vel`.

The safety process is a managed lifecycle node. `active` means supervision is
running; motion still starts disabled and requires the separate enable service.
The state publisher runs as a Jazzy component, while the EKF and safety gate
remain isolated processes.

## TF ownership

```text
odom                         robot_localization EKF
└── base_link
    ├── imu_link             fixed URDF joint
    ├── front_left_wheel_link
    ├── rear_left_wheel_link
    ├── front_right_wheel_link
    └── rear_right_wheel_link
```

The rear wheel joints mimic the corresponding ECU-owned representative joint.
The ECU never publishes TF. No component other than the EKF may publish
`odom -> base_link`.

## Motion gate

The gate starts disabled. A successful `/vehicle/motion_enable` request requires
fresh ECU diagnostics, synchronized epoch time, drive and IMU status `OK`, and
fresh odometry and IMU samples. A command must arrive after enable; a command
received while disabled is discarded.

Any invalid command, unsupported Twist axis, speed-limit violation, invalid
sensor frame or payload, status loss, sensor timeout, timestamp regression, or
clock range violation publishes zero and clears the enable latch. Recovery never
re-enables motion or restores the old command. The current block reason and the
latched last trip reason are reported separately. This RPi policy complements,
but does not replace, the independent ECU watchdog and motor fault handling.
The first valid command publisher after each enable owns that motion session.
A command from a different DDS writer clears the latch instead of racing the
active controller.

DDS QoS incompatibility, publisher disappearance, and changed odometry, IMU, or
diagnostics publisher GIDs also clear the latch. Topic statistics and aggregated
diagnostics are observability inputs only and never override the steady-clock
safety decision.

## Runtime profiles

- `vehicle.launch.py`: schema-validated calibrated hardware, bounded Agent
  restart, an enabled-by-default IMX219 camera, and optional MCAP recording and
  tracing. The camera runs as an isolated `camera_ros` process and can be
  disabled with `camera:=false`. Exhausting the Agent restart budget stops the
  launch so an external service supervisor can create a clean session.
- `fake_ecu.launch.py`: namespaced deterministic ECU for CI and fault injection.
- `replay.launch.py`: simulated time, a non-hardware command sink, and remapping
  of recorded command, EKF, TF, and safety outputs under `replay/recorded`.
