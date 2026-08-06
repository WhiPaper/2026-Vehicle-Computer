# ROS 인터페이스 레퍼런스

이 문서는 package version 0.2.2의 현재 구현을 기준으로 한 외부 계약이다.
토픽·서비스·파라미터·frame 이름을 바꾸려면 이 문서, 관련 launch/config,
테스트를 같은 변경으로 갱신한다.

## 계약 원칙

- RPi5의 vc_safety만 hardware cmd_vel을 발행한다.
- 상위 제어기는 cmd_vel이 아니라 cmd_vel_request를 발행한다.
- lifecycle active는 supervision 가능 상태일 뿐 motion enabled를 의미하지 않는다.
- enable 이후 첫 command publisher가 해당 motion session을 소유한다.
- 센서·진단·publisher identity·QoS·timestamp 이상은 fail-closed zero command로
  이어진다.
- recovery는 자동 enable하지 않는다. 운용자가 다시 enable하고 새 command를
  발행해야 한다.

## 주요 토픽

| 토픽 | 타입 | 방향·소유자 | QoS | 주기·계약 |
|---|---|---|---|---|
| /cmd_vel_request | geometry_msgs/msg/Twist | 외부 제어기 → vc_safety | reliable, volatile, keep-last 1 | 10 Hz 이상 권장. 유효 축은 linear.x와 angular.z |
| /cmd_vel | geometry_msgs/msg/Twist | vc_safety → ECU | reliable, volatile, keep-last 1 | safety gate가 20 Hz로 출력. 비활성·오류 시 zero |
| /odom | nav_msgs/msg/Odometry | ECU → safety/EKF | best-effort, volatile, keep-last 1 | 30 Hz. parent odom, child base_link |
| /imu/data_raw | sensor_msgs/msg/Imu | ECU → safety/EKF | best-effort, volatile, keep-last 1 | 50 Hz. frame imu_link |
| /joint_states | sensor_msgs/msg/JointState | ECU → robot_state_publisher | best-effort, volatile, keep-last 1 | 30 Hz |
| /diagnostics | diagnostic_msgs/msg/DiagnosticArray | ECU → safety/aggregator | reliable, volatile, keep-last 1 | 5 Hz heartbeat. transport/drive/IMU/time 상태 필수 |
| /odometry/filtered | nav_msgs/msg/Odometry | robot_localization → consumers | robot_localization 기본 계약 | EKF frequency 15 Hz. odom → base_link authority |
| /vehicle/motion_enabled | std_msgs/msg/Bool | vc_safety → consumers | reliable, transient-local, keep-last 1 | enable latch 상태. 새 subscriber가 마지막 상태를 수신 |
| /vehicle/safety/diagnostics | diagnostic_msgs/msg/DiagnosticArray | vc_safety → diagnostics | reliable, volatile, keep-last 1 | safety 상태 5 Hz |
| /vehicle/safety/statistics | statistics_msgs/msg/MetricsMessage | ROS 2 topic statistics → operations | ROS 2 statistics 기본 계약 | 1초 window. 안전 결정에는 사용하지 않음 |
| /vehicle/diagnostics | diagnostic_msgs/msg/DiagnosticArray | diagnostics aggregator → consumers | reliable, volatile, keep-last 1 | aggregate diagnostics 5 Hz |
| /tf | tf2_msgs/msg/TFMessage | EKF/state publisher → consumers | ROS TF 기본 계약 | EKF가 odom → base_link, state publisher가 joint TF |
| /tf_static | tf2_msgs/msg/TFMessage | robot_state_publisher → consumers | reliable, transient-local | URDF fixed transform |

raw sensor는 depth 1 best-effort로 ECU와 호환되어야 한다. command와
diagnostics는 reliable이어야 하며, QoS incompatibility 또는 publisher
disappearance는 safety latch를 해제한다.

## 서비스

| 서비스 | 타입 | 성공 조건 | 실패·복구 |
|---|---|---|---|
| /vehicle/motion_enable | std_srvs/srv/SetBool | data=true일 때 ECU transport/drive/IMU, time sync, odom, IMU가 모두 최신이고 lifecycle이 active | data=false 또는 readiness 실패는 disabled. 오류 복구 후 새 enable 필요 |
| /safety_gate/get_state | lifecycle_msgs/srv/GetState | lifecycle state 조회 | active여도 motion은 기본 disabled |
| /safety_gate/change_state | lifecycle_msgs/srv/ChangeState | 표준 lifecycle 전이 | configure/activate와 motion enable을 혼동하지 않음 |
| /rosbag2_recorder/snapshot | rosbag2_interfaces/srv/Snapshot | snapshot_mode recorder가 실행 중 | recorder가 없거나 일반 recording이면 사용할 수 없음 |

lifecycle 서비스의 실제 이름은 namespace에 따라 달라진다. 기본 hardware
profile에서는 safety gate가 /safety_gate이다.

## Safety gate 파라미터

기본 파일은 src/vc_safety/config/safety.yaml이다. lifecycle configure
시 검증하고, safety envelope에 영향을 주는 값은 read-only로 취급한다.

| 파라미터 | 기본값 | 단위·범위 | 의미 |
|---|---:|---|---|
| publish_rate_hz | 20.0 | Hz, 0.001–1000 | cmd_vel timer 주기 |
| status_rate_hz | 5.0 | Hz, 0.001–1000 | safety diagnostics 주기 |
| data_timeout_ms | 200 | ms, 1–60000 | odom/IMU 수신 허용 age |
| diagnostics_timeout_ms | 500 | ms, 1–60000 | ECU diagnostics 허용 gap |
| command_timeout_ms | 250 | ms, 1–60000 | command request 허용 gap |
| future_tolerance_ms | 100 | ms, 1–60000 | sensor timestamp 미래 허용량 |
| max_linear_speed_mps | 1.0 | m/s, 0.001–1000 | 허용 absolute linear.x |
| max_angular_speed_rps | 2.0 | rad/s, 0.001–1000 | 허용 absolute angular.z |
| odom_frame | odom | 비어 있지 않은 frame | raw odometry parent |
| base_frame | base_link | 비어 있지 않은 frame | raw odometry child |
| imu_frame | imu_link | 비어 있지 않은 frame | raw IMU frame |
| enable_topic_statistics | true | boolean | ROS topic statistics 활성화 |
| statistics_topic | vehicle/safety/statistics | relative topic | statistics 출력 이름 |
| use_sim_time | false | boolean | replay/simulation clock 사용 |

설정 파일의 unknown key, missing key, 잘못된 type, NaN/Inf, 범위 밖 값은
bringup을 시작하지 못하게 한다. 실제 차량의 max speed는 코드 기본값을
그대로 사용하지 말고 검증된 안전 한계로 낮춘다.

## 차량 설정 schema

기본 운영 파일은 src/vc_bringup/config/vehicle.yaml이며, 안전한 개발
예제는 src/vc_bringup/config/vehicle.example.yaml이다.

| 경로 | 타입·조건 | 단위 | 설명 |
|---|---|---|---|
| schema_version | integer = 1 | - | 현재 schema |
| vehicle.calibrated | boolean, hardware는 true | - | 실측 완료 여부 |
| vehicle.base_length | finite positive number | m | 차체 길이 |
| vehicle.base_width | finite positive number | m | 차체 폭 |
| vehicle.base_height | finite positive number | m | base 높이 |
| vehicle.wheel_radius | finite positive number | m | ECU와 동일한 wheel radius |
| vehicle.wheel_width | finite positive number | m | wheel 폭 |
| vehicle.wheel_base | finite positive number | m | 전후 wheel 중심 거리 |
| vehicle.track_width | finite positive number | m | 좌우 wheel 중심 거리 |
| imu.x/y/z | finite number | m | base_link 기준 IMU 위치 |
| imu.roll/pitch/yaw | finite number | rad | base_link 기준 IMU 자세 |

vehicle.example.yaml은 calibrated=false이므로 hardware launch에 전달하면
실패해야 한다. 운영 파일을 바꾼 뒤에는 configuration SHA-256과 인수시험
결과를 함께 보관한다.

## Frame ownership

| frame 관계 | authority | 종류 |
|---|---|---|
| odom → base_link | robot_localization EKF | dynamic |
| base_link → imu_link | URDF/robot_state_publisher | fixed |
| base_link → wheel links | URDF/robot_state_publisher | joint TF |
| URDF fixed joints | robot_state_publisher | /tf_static |

ECU는 TF를 발행하지 않는다. odom → base_link publisher는 EKF 하나만
허용한다. frame prefix는 현재 profile에서 비워 두며, namespace와 frame
이름을 혼동하지 않는다.

## Safety state와 failure semantics

| 조건 | 즉시 동작 | 재활성화 |
|---|---|---|
| 시작·lifecycle activate | zero, enabled=false | ECU/sensor readiness 뒤 enable |
| command timeout 250 ms | zero, latch 해제 | 새 enable + 새 command |
| odom/IMU timeout 200 ms | zero, latch 해제 | 입력 복구 후 새 enable |
| diagnostics timeout 500 ms | zero, latch 해제 | ECU heartbeat 복구 후 새 enable |
| invalid Twist, NaN/Inf, 축·속도 위반 | zero, trip 기록 | 원인 제거 후 새 enable |
| timestamp regression/future range 위반 | zero, trip 기록 | 정상 timestamp + 새 enable |
| QoS incompatible/publisher 변경 | zero, latch 해제 | 정상 writer 확인 후 새 enable |
| Agent/ECU 재시작 | diagnostics 오류 또는 timeout | 새 session에서 disabled 시작 |

현재 차단 원인은 safety diagnostics의 block_reason으로, 마지막 latch 해제
원인은 last_trip_reason으로, 누적 횟수는 trip_count로 확인한다.

## Launch profiles

### vehicle.launch.py

| 인자 | 기본값 | 필수 | 효과 |
|---|---|---|---|
| serial_device | 없음 | 예 | 단일 /dev/serial/by-id/<device> |
| baudrate | 921600 | 아니오 | micro-ROS Agent baud |
| agent_verbosity | 4 | 아니오 | Agent verbosity |
| agent_respawn_delay | 2.0 | 아니오 | Agent 재시작 간격, 초 |
| agent_respawn_limit | 300 | 아니오 | 최대 재시작 횟수 |
| use_sim_time | false | 아니오 | ROS clock 사용 |
| namespace | 빈 문자열 | 아니오 | 전체 relative 이름 namespace |
| record | false | 아니오 | MCAP recorder 시작 |
| camera | true | 아니오 | camera_ros 시작 |
| snapshot_mode | false | 아니오 | bounded snapshot recorder |
| trace | false | 아니오 | ros2_tracing 시작 |
| bag_output | bags/vehicle | 아니오 | bag 출력 경로 |
| record_storage_id | mcap | 아니오 | rosbag storage plugin |
| trace_session | vehicle | 아니오 | tracing session 이름 |
| trace_path | traces | 아니오 | trace 출력 경로 |
| vehicle_config | vc_bringup/config/vehicle.yaml | 아니오 | 차량 schema 파일 |
| safety_config | vc_safety/config/safety.yaml | 아니오 | safety 파라미터 파일 |
| camera_config | vc_bringup/config/camera.yaml | 아니오 | camera_ros 파라미터 파일 |

### 기타 profile

| profile | 주요 인자 | 하드웨어 격리 |
|---|---|---|
| fake_ecu.launch.py | namespace, vehicle_config | Agent·serial 없음 |
| replay.launch.py | bag_path 필수, namespace, vehicle_config | cmd_vel을 replay/cmd_vel_sink로 remap |
| simulation.launch.py | namespace 기본 sim, vehicle_config, safety_config, headless | /sim namespace, Agent·serial 없음 |
| fake_visualization.launch.py | namespace, vehicle_config | fake ECU + RViz2 |
| replay_visualization.launch.py | bag_path 필수, namespace | replay output은 replay/recorded 아래 |
| rviz.launch.py | namespace, use_sim_time, replay, simulation, rviz_config | 제어 publisher 없음 |

profile을 선택할 때 [시각화 가이드](visualization.md)와
[개발 가이드](development.md)의 격리 조건을 함께 확인한다.

## Diagnostics tree

aggregate diagnostics는 다음 경로를 유지한다.

    /Vehicle
      /ECU/Transport
      /ECU/Drive
      /ECU/IMU
      /Computer/SafetyGate
      /Computer/StateEstimation
      /Computer/SerialDevice       hardware only
      /Computer/ECUConnection      hardware only

diagnostics는 관찰과 운영 판단을 돕지만, 안전 결정은 gate 내부의 steady
clock timeout과 입력 검증이 authoritative하다.
