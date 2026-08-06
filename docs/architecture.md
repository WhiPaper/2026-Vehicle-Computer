# 아키텍처

이 문서는 RPi5 vehicle computer의 현재 runtime boundary와 ownership을
설명한다. 외부 연동에 필요한 이름·타입·QoS·주기는
[ROS 인터페이스 레퍼런스](interfaces.md)를 기준으로 한다.

## 시스템 경계

    teleop / navigation
            |
      /cmd_vel_request
            |
      lifecycle vc_safety <---- /diagnostics, /odom, /imu/data_raw
            |
         /cmd_vel
            |
    micro_ros_agent ==== 921600 baud XRCE-DDS ==== ESP32 vehicle_ecu

    /odom + /imu/data_raw ---> isolated robot_localization
                                      -> /odometry/filtered
    /joint_states ----------> composed robot_state_publisher -> /tf
    vehicle URDF -----------------------------------------> /tf_static

    IMX219 -> libcamera -> camera_ros
                       -> /camera/image_raw + /camera/camera_info

    ECU + safety + EKF diagnostics -> mux -> diagnostic_aggregator
    ROS subscription statistics -----------------> /vehicle/safety/statistics

ESP32는 50 Hz motor loop, wheel odometry, 독립 500 ms command watchdog을
소유한다. RPi5는 URDF, state estimation, dynamic TF, supervisory command gate를
소유한다. RPi5에서 physical /cmd_vel을 발행하는 구성요소는 vc_safety 하나다.

## 프로세스 경계

| 프로세스 | 책임 | 격리 이유 |
|---|---|---|
| micro_ros_agent | serial XRCE-DDS transport | Agent 재시작이 gate state를 복구시키지 않도록 분리 |
| vehicle_state_container | robot_state_publisher component | non-safety state publication composition |
| ekf_filter_node | odom + IMU → filtered odometry, odom TF | robot_localization이 component를 제공하지 않음 |
| safety_gate | lifecycle safety decision | visualization/state estimation 오류로부터 보호 |
| diagnostic aggregator | 운영용 health tree | 관찰용이며 safety decision authority가 아님 |
| camera_node | IMX219 image stream | camera failure가 command supervision을 중단하지 않음 |
| rosbag2 recorder | MCAP observability evidence | optional, bounded snapshot 가능 |

Safety gate는 single-threaded executor와 mutually-exclusive callback group에서
센서·진단·command·service·timer state mutation을 순서대로 처리한다. topic
statistics와 DDS event callback은 진단 입력이며 steady-clock timeout을
대체하지 않는다.

## TF ownership

    odom                         robot_localization EKF
    └── base_link
        ├── imu_link             fixed URDF joint
        ├── front_left_wheel_link
        ├── rear_left_wheel_link
        ├── front_right_wheel_link
        └── rear_right_wheel_link

- EKF만 odom → base_link를 발행한다.
- robot_state_publisher는 URDF fixed/joint transform을 발행한다.
- ECU는 TF를 발행하지 않는다.
- rear wheel joint는 ECU representative joint를 mimic한다.
- 현재 launch는 frame prefix를 비워 둔다. namespace와 frame prefix는 서로
  다른 격리 수단이다.

## Motion gate

gate는 configure/activate 과정과 motion enable을 분리한다.

1. configure는 parameter와 endpoint를 검증하고 disabled state를 만든다.
2. activate는 publisher와 supervision을 시작하지만 motion은 disabled다.
3. motion_enable=true는 ECU transport/drive/IMU, epoch time sync, 최신 odom/IMU가
   모두 준비된 경우에만 성공한다.
4. enable 이후 첫 valid command publisher가 session owner가 된다.
5. command가 없거나 invalid하면 zero를 발행하고 latch를 해제한다.

다음 조건은 모두 fail-closed trip 또는 block으로 처리한다.

- unsupported Twist axis, NaN/Inf, speed limit 위반
- odometry/IMU frame 또는 payload 오류
- sensor, diagnostics, command timeout
- message timestamp regression 또는 future range 위반
- DDS QoS incompatibility, publisher disappearance, publisher GID 변경
- ECU status 또는 time synchronization 손실

복구는 latch를 자동으로 되살리지 않는다. 기존 command를 재사용하지 않고,
새 enable과 그 뒤의 새 command를 요구한다. 이 정책은 ECU watchdog과
motor fault handling에 추가되는 RPi supervisory policy다.

## Diagnostics ownership

aggregate tree는 다음처럼 유지한다.

    /Vehicle
      /ECU/Transport
      /ECU/Drive
      /ECU/IMU
      /Computer/SafetyGate
      /Computer/StateEstimation
      /Computer/SerialDevice
      /Computer/ECUConnection

ECU diagnostics와 raw sensor는 safety gate의 직접 입력이다. aggregated
diagnostics는 운영 화면과 기록을 위한 결과이며 steady-clock safety decision을
override하지 않는다.

## Runtime profiles

| profile | 입력 | 출력·격리 |
|---|---|---|
| vehicle | real Agent, calibrated vehicle.yaml, optional camera/record/trace | physical /cmd_vel, systemd에서 실행 |
| fake_ecu | deterministic ROS ECU | Agent·serial 없음, namespace 시험 가능 |
| replay | MCAP, simulated time, fake vehicle config | safety output은 replay/cmd_vel_sink, recorded output은 replay/recorded |
| simulation | Gazebo world, fake config, /sim namespace | /sim/cmd_vel만 Gazebo로 전달, Agent·serial 없음 |

profile 사이에 safety state machine과 ROS contract를 복제하지 않는다. 실제
차량용 launch와 replay/simulation launch의 가장 중요한 차이는 hardware
output boundary다.

## Recording boundary

MCAP는 raw/filtered state, command request와 gated command, diagnostics,
parameter events, statistics, TF를 기록한다. recording launch는 git revision과
vehicle config SHA-256을 custom metadata에 넣는다.

replay는 기록된 gated command, EKF/TF, safety output을 replay/recorded 아래로
remap하고 gate output을 replay/cmd_vel_sink로 보낸다. 그러므로 replay가
physical /cmd_vel publisher를 만들지 않는 것이 불변 조건이다.

## 의도적으로 제외한 것

- ros2_control을 RPi와 ECU 사이에 추가하지 않는다. ECU가 motor loop,
  odometry, watchdog, wire contract를 이미 소유한다.
- small Twist/IMU/Odometry에 loaned message나 zero-copy를 safety requirement로
  사용하지 않는다.
- ECU가 matching QoS를 제공하기 전에는 DDS deadline/liveliness 정책을
  임의로 강화하지 않는다.
- lifecycle active를 motion enable의 대체 의미로 사용하지 않는다.
- SROS 2 enclave policy는 target vehicle의 network boundary와 identity가
  결정될 때 별도 설계한다.
