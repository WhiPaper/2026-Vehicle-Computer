# 2026 Vehicle Computer

Raspberry Pi 5에서 ROS 2 Jazzy와 ESP32 vehicle_ecu를 연결하는 차량용
컴퓨터 workspace입니다. micro-ROS serial transport, URDF/TF, wheel
odometry와 IMU의 EKF 융합, 명시적 motion enable 기반의 안전 게이트,
카메라·기록·시각화 도구를 제공합니다.

실차를 움직이는 시스템입니다. 아래 문서와 인수시험을 읽기 전에는
cmd_vel을 발행하거나 motion enable을 요청하지 마십시오. RPi 안전 게이트는
ECU의 독립 watchdog과 motor fault handling을 대체하지 않습니다.

## 문서 길찾기

| 목적 | 문서 |
|---|---|
| 전체 구조와 빠른 시작 | 이 README |
| ROS 토픽·서비스·파라미터·TF 계약 | [인터페이스 레퍼런스](docs/interfaces.md) |
| APT/systemd 설치와 장애 대응 | [운영 가이드](docs/operations.md) |
| devcontainer·빌드·개발 profile | [개발 가이드](docs/development.md) |
| 런타임 경계와 소유권 | [아키텍처](docs/architecture.md) |
| ROS 2 설계 원칙과 구현 상태 | [ROS 2 설계](docs/ros2-native-design.md) |
| RViz2·replay·Gazebo | [시각화 가이드](docs/visualization.md) |
| 실제 차량 인수시험 | [인수시험 체크리스트](docs/acceptance-testing.md) |
| 문서와 인터페이스 변경 규칙 | [기여 가이드](CONTRIBUTING.md) |

문서의 기준 환경은 Ubuntu 24.04, ROS 2 Jazzy, package version 0.2.2입니다.
vendor 디렉터리의 upstream 문서는 이 프로젝트의 운영 기준 문서가 아닙니다.

## 패키지

- vc_description: 4륜 Xacro와 base_link, imu_link, wheel frame
- vc_bringup: micro-ROS Agent, 상태 발행기, EKF, 진단, IMX219,
  rosbag/tracing 및 hardware/fake/replay launch
- vc_safety: cmd_vel_request를 검증해 cmd_vel로 전달하는 C++ lifecycle
  supervisory gate
- vc_visualization: PC 전용 RViz2, rqt_graph, MCAP replay, Gazebo Sim
- vehicle_computer: 전체 ROS 패키지와 Raspberry Pi runtime을 설치하는 variant

현재 ROS graph의 authoritative ownership은
[인터페이스 레퍼런스](docs/interfaces.md)와
[아키텍처](docs/architecture.md)에 기록합니다.

## 빠른 시작

### 개발 환경

권장 개발 환경은 저장소의 .devcontainer입니다. 장치를 연결하지 않은
환경에서도 fake ECU, replay, simulation을 사용할 수 있습니다.

호스트에서 직접 준비할 때는 다음 순서를 사용합니다.

    sudo apt install python3-colcon-meson meson ninja-build \
      python3-jinja2 python3-ply python3-yaml \
      libdw-dev libgnutls28-dev libudev-dev libunwind-dev libyaml-dev
    source /opt/ros/jazzy/setup.bash
    vcs import src --skip-existing < dependencies.repos
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
      --skip-keys="libcamera vehicle_computer_runtime"
    colcon build --symlink-install --meson-args \
      -Dpipelines=rpi/pisp -Dipas=rpi/pisp \
      -Dtest=false -Ddocumentation=disabled -Dpycamera=disabled \
      -Dgstreamer=disabled -Dv4l2=disabled
    source install/setup.bash
    python3 scripts/check_docs.py

위 의존성 목록은 CI와 devcontainer를 기준으로 유지합니다. 상세한 개발
profile과 테스트 명령은 [개발 가이드](docs/development.md)를 따릅니다.

### 설정 파일

설정 파일은 역할을 구분합니다.

- src/vc_bringup/config/vehicle.example.yaml: 미보정 개발 예제입니다.
  calibrated가 false이므로 hardware launch가 거부합니다.
- src/vc_bringup/config/vehicle.yaml: 현재 프로젝트 차량의 보정된 운영
  프로파일입니다. 다른 차량에는 복사 후 모든 치수를 실측값으로 교체합니다.
- src/vc_safety/config/safety.yaml: 선속도·각속도 한계, timeout, 기대 frame을
  포함하는 fail-closed 안전 설정입니다.

차량의 wheel radius와 track width는 ECU와 동일한 실측 결과를 사용해야
합니다. 길이는 m, 장착 각도는 rad입니다. 보정 설정을 변경한 뒤에는
[인수시험 체크리스트](docs/acceptance-testing.md)의 설정·TF 시험을 다시
수행하고 설정 파일의 hash를 기록합니다.

### Fake ECU

실제 ECU 없이 전체 ROS graph와 안전 게이트를 확인합니다.

    ros2 launch vc_bringup fake_ecu.launch.py
    ros2 lifecycle get /safety_gate
    ros2 topic echo /vehicle/diagnostics

fake ECU는 serial device와 micro-ROS Agent를 시작하지 않습니다.
명령·서비스·timeout 동작은 [인터페이스 레퍼런스](docs/interfaces.md)를
확인합니다.

### Hardware bringup

운영 전에는 안정적인 serial-by-id 경로와 service user 권한을 확인합니다.

    ls -l /dev/serial/by-id/
    ros2 launch vc_bringup vehicle.launch.py \
      serial_device:=/dev/serial/by-id/<device>

장치의 실제 그룹을 확인하고 해당 사용자에게 읽기·쓰기 권한을 부여합니다.

    ls -l "$(readlink -f /dev/serial/by-id/<device>)"
    sudo usermod -aG "$(stat -c %G \
      "$(readlink -f /dev/serial/by-id/<device>)")" "$USER"

새 로그인 세션이 필요할 수 있습니다. Agent는 기본적으로 2초 간격으로
최대 300회 재시작하며, 한도에 도달하면 launch를 종료합니다. 모든 재기동과
복구 경로에서 motion은 disabled로 시작합니다.

motion enable은 lifecycle active와 별개입니다. ECU diagnostics, time
synchronization, odometry, IMU가 정상인 것을 확인한 뒤에만 요청합니다.

    ros2 service call /vehicle/motion_enable std_srvs/srv/SetBool '{data: true}'
    ros2 topic echo /vehicle/safety/diagnostics

상위 제어기는 cmd_vel이 아니라 cmd_vel_request를 10 Hz 이상으로 발행합니다.
유효하지 않은 명령, publisher 변경, sensor/diagnostics timeout, frame 오류는
zero command를 만들고 enable latch를 해제합니다. 복구 후에도 새 enable과
새 명령이 필요합니다.

## 시각화·replay·simulation

PC에는 다음 GUI 의존성을 설치합니다.

    sudo apt install ros-jazzy-rviz2 ros-jazzy-rqt-graph ros-jazzy-ros-gz

대표 profile은 다음과 같습니다.

    ros2 launch vc_visualization fake_visualization.launch.py
    ros2 launch vc_visualization replay_visualization.launch.py \
      bag_path:=<bag-directory>
    ros2 launch vc_visualization simulation.launch.py

실차 시각화는 RPi5에서 hardware launch를 실행하고, 승인된 PC에서 같은
전용 ROS_DOMAIN_ID를 사용해 RViz2만 실행합니다.

    export ROS_DOMAIN_ID=42
    ros2 launch vc_visualization rviz.launch.py

replay는 안전 게이트 출력을 replay/cmd_vel_sink로 보내며 실제 cmd_vel
publisher를 만들지 않습니다. Gazebo는 /sim namespace와 /sim/cmd_vel만
사용합니다. 상세 토픽과 격리 조건은 [시각화 가이드](docs/visualization.md)를
따릅니다.

## 운영 배포

운영 환경은 서명된 APT 저장소에서 ros-jazzy-vehicle-computer를 설치하는
방법을 우선합니다. 직접 workspace를 /opt/vehicle-computer에 배치하는
경로는 커스텀 빌드와 개발용입니다.

설치, 설정 백업, systemd 상태 확인, journalctl, 장애 복구, 업데이트와
롤백은 [운영 가이드](docs/operations.md)에만 기록하며 README에는 절차를
중복하지 않습니다.

## 검증

문서와 코드의 기본 정합성은 다음으로 확인합니다.

    pre-commit run --all-files
    python3 scripts/check_docs.py
    colcon test --packages-select \
      vc_description vc_safety vc_bringup vc_visualization
    colcon test-result --verbose

실제 차량의 pass/fail과 bag/log 증적은
[인수시험 체크리스트](docs/acceptance-testing.md)에 기록합니다.

## 지원 정보

문제가 발생하면 먼저 다음 순서로 확인합니다.

1. /dev/serial/by-id 경로와 service user 권한
2. /vehicle/diagnostics와 /vehicle/safety/diagnostics
3. safety gate lifecycle 상태와 motion_enabled
4. /odom, /imu/data_raw, /diagnostics의 발행 주기와 QoS
5. systemd 로그와 Agent 재시작 한도

문서에 없는 동작을 추가하거나 토픽·서비스·파라미터 계약을 바꾸는 경우
[기여 가이드](CONTRIBUTING.md)의 문서 갱신 절차를 함께 적용합니다.
