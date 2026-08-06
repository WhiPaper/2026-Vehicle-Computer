# 시각화·replay·simulation 가이드

이 패키지는 PC에서 graph를 관찰하고, 기록을 안전하게 replay하고, Gazebo
simulation을 실행하기 위한 도구를 제공한다. production
vehicle_computer에는 GUI와 Gazebo 의존성을 넣지 않는다.

실차 제어 publisher가 아니므로 RViz2와 rqt_graph에서 cmd_vel_request를
발행하지 않는다. 실제 설치와 장애 대응은 [운영 가이드](operations.md)를,
ROS contract는 [인터페이스 레퍼런스](interfaces.md)를 따른다.

## 도구 역할

| 도구 | 역할 |
|---|---|
| RViz2 | URDF, TF, filtered odometry, wheel state, camera display |
| rqt_graph | node와 topic connection 확인 |
| ros2 bag | MCAP 기록과 격리된 replay |
| Gazebo Sim | /sim namespace의 vehicle physics와 sensor simulation |

개발 PC에 설치한다.

    sudo apt install ros-jazzy-rviz2 ros-jazzy-rqt-graph ros-jazzy-ros-gz

## Live hardware

RPi5에서 hardware bringup을 실행한다.

    ros2 launch vc_bringup vehicle.launch.py \
      serial_device:=/dev/serial/by-id/<device>

승인된 PC와 RPi5가 같은 전용 ROS_DOMAIN_ID를 사용하도록 설정한다.

    export ROS_DOMAIN_ID=42
    ros2 launch vc_visualization rviz.launch.py

기본 RViz fixed frame은 odom이다. 다음 데이터를 표시한다.

    /tf
    /tf_static
    /joint_states
    /odometry/filtered
    /camera/image_raw/compressed
    /camera/camera_info

hardware 테스트 중 RViz2와 rqt_graph는 command publisher가 아니다.
RPi5와 PC의 domain을 다른 차량과 공유하지 않는다.

## Fake stack

    ros2 launch vc_visualization fake_visualization.launch.py

다음 요소를 함께 시작한다.

- deterministic fake ECU
- state estimation
- lifecycle safety gate
- diagnostics
- RViz2

serial device와 micro-ROS Agent는 시작하지 않는다. motion enable을 연습할
때도 차체가 실제로 연결되지 않은 개발 환경인지 확인한다.

## MCAP replay

    ros2 launch vc_visualization replay_visualization.launch.py \
      bag_path:=<bag-directory>

replay는 use_sim_time=true를 사용한다. recorded filtered odometry, TF, safety
output은 replay/recorded 아래에서 표시한다. safety gate output은
replay/cmd_vel_sink로 보내므로 physical /cmd_vel publisher를 만들지 않는다.

확인 명령:

    ros2 topic info /cmd_vel
    ros2 topic list -t
    ros2 topic echo /replay/recorded/vehicle/motion_enabled

replay는 별도 ROS_DOMAIN_ID에서 실행하고, bag의 source timestamp와
replay/recorded remap을 섞지 않는다.

## Gazebo simulation

    ros2 launch vc_visualization simulation.launch.py

기본 namespace는 sim이다.

    /sim/cmd_vel_request
    /sim/cmd_vel
    /sim/odom
    /sim/imu/data_raw
    /sim/joint_states
    /sim/diagnostics
    /sim/odometry/filtered
    /sim/tf
    /sim/tf_static

Gazebo command는 ros_gz_bridge를 통해 sim/cmd_vel로 전달한다. Gazebo의
odom, IMU, joint state, clock은 ROS 2로 bridge한다. safety gate와 EKF가
ROS-side authority이며 Gazebo는 odom → base_link TF를 직접 발행하지 않는다.

motion은 hardware와 같은 explicit enable sequence를 사용한다.

    ros2 service call /sim/vehicle/motion_enable std_srvs/srv/SetBool \
      '{data: true}'
    ros2 topic pub -r 10 /sim/cmd_vel_request geometry_msgs/msg/Twist \
      '{linear: {x: 0.05}, angular: {z: 0.0}}'

command publisher를 중단하거나 simulation health parameter를 healthy=false로
바꾸면 safety gate는 enable을 해제하고 zero를 발행해야 한다.

Headless 실행:

    ros2 launch vc_visualization simulation.launch.py headless:=true

## RViz profile 인자

| 인자 | 기본값 | 용도 |
|---|---|---|
| namespace | 빈 문자열 | node와 topic namespace |
| use_sim_time | false | replay/simulation clock |
| replay | false | replay/recorded TF와 odometry remap |
| simulation | false | simulation namespace TF remap |
| rviz_config | vehicle.rviz | RViz display 설정 |

## rqt_graph

실행 중인 profile에서 graph만 확인한다.

    ros2 launch vc_visualization graph.launch.py

graph를 통해 safety gate의 입력과 output publisher를 확인하되, graph 화면에서
command를 직접 발행하지 않는다.
