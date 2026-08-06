# 개발 가이드

이 문서는 ROS 2 Jazzy workspace를 빌드하고 fake/replay/simulation profile로
변경을 검증하는 개발자용 안내서다. 실제 ECU와 차량을 움직이는 절차는
[운영 가이드](operations.md)와 [인수시험 체크리스트](acceptance-testing.md)를
사용한다.

## 개발 환경

### devcontainer

저장소의 .devcontainer는 ROS 2 Jazzy, colcon, Meson, Python 도구, camera
의존성을 포함한다. 기본 container는 serial device를 매핑하지 않으므로
fake/replay/simulation을 안전하게 사용할 수 있다.

실제 ECU를 연결하는 경우에만 container 생성 설정에 stable serial path와
필요한 host device를 명시적으로 전달한다. video/render group만으로는
camera device 접근이 충분하지 않을 수 있다.

container 생성 후 workspace에서 다음을 실행한다.

    source /opt/ros/jazzy/setup.bash
    vcs import src --skip-existing < dependencies.repos
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
      --skip-keys="libcamera vehicle_computer_runtime"

### 호스트

Ubuntu 24.04와 ROS 2 Jazzy를 사용한다. 프로젝트 build prerequisite는
CI와 .devcontainer의 목록을 기준으로 유지한다.

    sudo apt install python3-colcon-meson meson ninja-build \
      python3-jinja2 python3-ply python3-yaml \
      libdw-dev libgnutls28-dev libudev-dev libunwind-dev libyaml-dev
    source /opt/ros/jazzy/setup.bash
    vcs import src --skip-existing < dependencies.repos
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
      --skip-keys="libcamera vehicle_computer_runtime"

PC 시각화·simulation 의존성:

    sudo apt install ros-jazzy-rviz2 ros-jazzy-rqt-graph ros-jazzy-ros-gz

## 빌드와 테스트

개발 빌드:

    colcon build --symlink-install --meson-args \
      -Dpipelines=rpi/pisp -Dipas=rpi/pisp \
      -Dtest=false -Ddocumentation=disabled -Dpycamera=disabled \
      -Dgstreamer=disabled -Dv4l2=disabled
    source install/setup.bash

문서·repository 검사:

    pre-commit run --all-files
    python3 scripts/check_docs.py

ROS 패키지 테스트:

    colcon test --packages-select \
      vc_description vc_safety vc_bringup vc_visualization
    colcon test-result --verbose

안전 gate 변경은 pure SafetyState, input validation, ECU diagnostics,
lifecycle launch test를 모두 통과해야 한다. launch profile 변경은 fake
stack과 replay isolation test를 함께 실행한다.

## 패키지별 개발 범위

| 패키지 | 책임 | 변경 시 확인 |
|---|---|---|
| vc_description | URDF/Xacro, static frame, simulation plugin | URDF link/joint 및 Gazebo plugin test |
| vc_bringup | launch orchestration, EKF, diagnostics, recording | profile arg, config schema, graph test |
| vc_safety | fail-closed gate와 lifecycle component | state, validation, QoS, GID, timeout test |
| vc_visualization | RViz2, replay, Gazebo 격리 | /sim namespace, bridge, no Agent/serial test |
| vehicle_computer | 설치 의존성 variant | package set와 release build |

ECU wire contract, 50 Hz motor loop, ECU odometry, 500 ms ECU watchdog은 이
workspace의 변경 대상이 아니다.

## 실행 profile

### Fake stack

    ros2 launch vc_bringup fake_ecu.launch.py

fake ECU는 deterministic odom, IMU, joint state, diagnostics를 발행한다.
namespace를 사용하는 테스트:

    ros2 launch vc_bringup fake_ecu.launch.py namespace:=ci_vehicle

정상 상태에서 lifecycle active를 확인하고, motion enable 뒤 request를
발행한다. 다음 fault를 parameter 또는 publisher 조작으로 확인한다.

- healthy=false 또는 diagnostics 손실
- time_synchronized=false
- command publisher 변경
- incompatible QoS
- command·sensor timeout
- timestamp regression/future range

### Replay

    ros2 launch vc_bringup replay.launch.py bag_path:=<bag-directory>

replay는 use_sim_time=true이며 fake vehicle config와 non-hardware command
sink를 사용한다. /cmd_vel, EKF/TF, safety state의 기록 output은
replay/recorded 아래로 remap된다.

    ros2 topic info /cmd_vel
    ros2 topic list -t

replay 환경에서 physical /cmd_vel publisher가 나오면 profile 변경을
승인하지 않는다.

### Visualization replay

    ros2 launch vc_visualization replay_visualization.launch.py \
      bag_path:=<bag-directory>

RViz2는 기록된 filtered odometry와 TF를 표시하지만 control publisher가
아니다.

### Gazebo simulation

    ros2 launch vc_visualization simulation.launch.py

simulation 기본 namespace는 sim이다. Agent와 serial device를 시작하지
않으며 Gazebo command는 sim/cmd_vel을 사용한다. motion은 hardware와 같은
명시적 enable 순서를 요구한다.

    ros2 service call /sim/vehicle/motion_enable std_srvs/srv/SetBool \
      '{data: true}'
    ros2 topic pub -r 10 /sim/cmd_vel_request geometry_msgs/msg/Twist \
      '{linear: {x: 0.05}, angular: {z: 0.0}}'

headless CI:

    ros2 launch vc_visualization simulation.launch.py headless:=true

simulation health를 healthy=false로 바꾸거나 센서를 끊으면 safety gate가
zero를 발행하고 enabled state를 false로 되돌려야 한다.

## 설정 변경

vehicle 설정:

- vehicle.example.yaml을 개발용 미보정 fixture로 사용한다.
- vehicle.yaml은 현재 프로젝트 차량의 운영 profile이므로 임의로 generic
  예제값으로 되돌리지 않는다.
- 치수는 m, IMU 자세는 rad를 사용한다.
- schema_version과 unknown-key 검증을 유지한다.

safety 설정:

- timeout과 speed limit은 안전 envelope이므로 변경 시 인수시험을 다시 한다.
- safety gate active 중 parameter를 바꾸지 않는다.
- 변경 순서는 disable → deactivate → parameter 변경 → configure → activate다.

camera 설정:

- 기본은 IMX219, 1280x720, 30 Hz, camera_optical_frame이다.
- calibration 전 CameraInfo를 metric vision에 사용하지 않는다.
- camera 없는 CI는 vehicle.launch.py camera:=false를 사용한다.

## launch argument 확인

구현과 문서의 argument 표가 맞는지 다음 명령과 scripts/check_docs.py로
확인한다.

    ros2 launch vc_bringup vehicle.launch.py --show-args
    ros2 launch vc_bringup fake_ecu.launch.py --show-args
    ros2 launch vc_bringup replay.launch.py --show-args
    ros2 launch vc_visualization simulation.launch.py --show-args

지원되지 않는 logging-level 인자를 문서나 실행 스크립트에 추가하지 않는다.
전체 인자와 기본값은 [인터페이스 레퍼런스](interfaces.md)에 있다.

## 릴리스

다섯 ROS package.xml과 Python setup.py version을 같은 값으로 맞춘다.
release tag는 vMAJOR.MINOR.PATCH 형식이어야 한다. CI는 AMD64와 ARM64에서
build/test하고 ROS Debian package와 vehicle-computer-runtime을 만든다.

릴리스 전 체크:

1. 문서의 package version과 실제 version을 확인한다.
2. python3 scripts/check_docs.py를 실행한다.
3. pre-commit과 ROS test를 통과시킨다.
4. vehicle/safety/camera 설정 변경과 hash를 검토한다.
5. 두 architecture package의 SHA-256과 provenance를 보관한다.
6. 정식 hardware는 [인수시험 체크리스트](acceptance-testing.md)를 실행한다.

## 로그와 증적

개발 profile은 output=screen을 사용한다. 실패한 launch의 stdout/stderr와
colcon log를 보관한다. 기록 profile은 git revision과 vehicle config
SHA-256을 MCAP metadata에 남긴다.

문서나 public ROS contract를 변경하면 변경한 토픽·서비스·파라미터와
관련 테스트를 commit 설명에 기록한다. 세부 규칙은 [기여 가이드](../CONTRIBUTING.md)를
따른다.
