# 2026 Vehicle Computer

Raspberry Pi 5용 ROS 2 Jazzy workspace다. ESP32 `vehicle_ecu`와 921600 baud
micro-ROS serial transport로 연결하고, URDF/TF, wheel odometry와 IMU의 EKF
융합, 명시적 enable 방식의 주행 안전 게이트를 제공한다.

## Packages

- `vc_description`: 4륜 Xacro와 `base_link`, `imu_link`, wheel TF
- `vc_bringup`: Agent, component 기반 상태 발행기, EKF, 진단 집계,
  `camera_ros` 기반 IMX219, rosbag/tracing 및 hardware/fake/replay launch
- `vc_safety`: `/cmd_vel_request`를 검증해 ECU `/cmd_vel`로 전달하는
  C++ lifecycle supervisory gate/component
- `vehicle_computer`: 위 ROS 패키지와 플랫폼 runtime을 실행 의존성으로
  묶는 설치용 variant 패키지

현재 경계와 데이터 흐름은 [architecture](docs/architecture.md), ROS 2
기능을 확장 적용하는 목표 구조는
[ROS 2-native design](docs/ros2-native-design.md), 실차 검증 절차는
[acceptance testing](docs/acceptance-testing.md)을 참고한다.

## First setup

Ubuntu 24.04와 ROS 2 Jazzy를 사용한다. devcontainer를 열거나 호스트에
의존성을 설치한 뒤 workspace를 빌드한다.

```bash
sudo apt install python3-colcon-meson meson ninja-build \
  python3-jinja2 python3-ply python3-yaml \
  libdw-dev libgnutls28-dev libudev-dev libunwind-dev libyaml-dev
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

`src/vc_bringup/config/vehicle.yaml`의 `schema_version: 1`을 유지하면서
실측 SI 단위 치수와 IMU 장착 자세를 기록하고 마지막에
`calibrated: true`로 변경한다. 미보정 템플릿, 알 수 없는 키, 누락값,
0/음수 치수로는 최상위 bringup이 시작되지 않는다. ECU의 wheel radius,
track width와 이 파일의 값은 동일한 실측 결과를 사용해야 한다.
`vc_safety/config/safety.yaml`의 최대 선속도·각속도도 실제 차량의
검증된 안전 한계로 낮추고, 운영 시에는 `safety_config` 인자로 설치된
읽기 전용 설정 경로를 전달한다.

## Run

```bash
ls -l /dev/serial/by-id/
ros2 launch vc_bringup vehicle.launch.py \
  serial_device:=/dev/serial/by-id/<CP2102-device>
```

Agent는 2초 간격으로 최대 300회 재시작한다. 한도에 도달하면 전체 launch를
종료하며, systemd 배포에서는 서비스를 새 세션으로 재시작한다. 모든 재기동
경로에서 안전 게이트는 disabled 상태로 시작한다. 개발 및 CI에서는 실제 ECU
없이 전체 그래프를 실행할 수 있다.

```bash
ros2 launch vc_bringup fake_ecu.launch.py
ros2 lifecycle get /safety_gate
ros2 topic echo /vehicle/diagnostics
```

bag replay는 안전 게이트 출력을 `replay/cmd_vel_sink`로 보내고, bag에
기록된 `/cmd_vel`, EKF/TF 및 안전 상태 출력도 `/replay/recorded/...`로
강제 remap한다. 따라서 물리 `/cmd_vel` publisher를 만들지 않는다.

```bash
ros2 launch vc_bringup replay.launch.py bag_path:=<bag-directory>
```

개발 컨테이너에서 실제 ECU를 연결할 때는 호스트의 해당 직렬 장치를
컨테이너 생성 설정에 명시적으로 전달해야 한다. 장치가 없는 환경에서도
컨테이너가 시작되도록 기본 설정에는 장치 매핑을 넣지 않았다.

## APT deployment

`vMAJOR.MINOR.PATCH` 태그가 네 ROS 패키지의 `package.xml` 버전과 일치하면
GitHub Actions가 Ubuntu 24.04의 AMD64와 ARM64에서 각각 빌드·테스트한다.
Bloom으로 ROS `.deb`를 만들고 Raspberry Pi libcamera, systemd unit 및
보존 설정을 `vehicle-computer-runtime`으로 패키징한 뒤 GitHub Release와
서명된 GitHub Pages APT 저장소에 배포한다.

ROS 2에서는 특별한 metapackage 형식을 쓰지 않는다. `vehicle_computer`는
코드가 없는 일반 `ament_cmake` 패키지이며 `exec_depend`로 전체 설치 구성을
표현한다. 따라서 사용자는 아키텍처와 관계없이 이 패키지 하나를 설치한다.

```bash
curl -fsSL \
  https://whipaper.github.io/2026-Vehicle-Computer/vehicle-computer-archive-keyring.gpg |
  sudo tee /usr/share/keyrings/vehicle-computer.gpg >/dev/null
echo "deb [signed-by=/usr/share/keyrings/vehicle-computer.gpg] https://whipaper.github.io/2026-Vehicle-Computer noble main" |
  sudo tee /etc/apt/sources.list.d/vehicle-computer.list
sudo apt update
sudo apt install ros-jazzy-vehicle-computer

sudoedit /etc/vehicle-computer/vehicle-computer.env
sudoedit /etc/vehicle-computer/vehicle.yaml
sudoedit /etc/vehicle-computer/safety.yaml
sudo systemctl enable --now vehicle-computer.service
```

차량이 예기치 않게 기동하지 않도록 패키지는 서비스를 자동 활성화하지
않는다. GitHub Release에는 두 아키텍처의 `.deb`, 아키텍처별 SHA-256
목록과 빌드 provenance가 남는다. Pages 저장소는 현재 태그의 패키지
스냅샷이며 이전 버전 파일은 Release에서 보존한다.

저장소 관리자는 최초 배포 전에 GitHub의 **Settings → Pages → Source**를
**GitHub Actions**로 설정하고, `APT_GPG_PRIVATE_KEY`와
`APT_GPG_PASSPHRASE` Actions secrets를 등록해야 한다. 공개키는 별도 secret
없이 배포 시 저장소 루트에 export된다. 릴리스 순서는 다음과 같다.

```bash
# 모든 자체 package.xml과 src/vc_bringup/setup.py의 version을
# 먼저 같은 값으로 변경한다.
git tag -s v0.2.2 -m "v0.2.2"
git push origin v0.2.2
```

직접 workspace를 배치하는 운영 예시는 `deploy/systemd`에 있다.
workspace를 `/opt/vehicle-computer`에 배치하고 환경 파일과 보정 설정을
설치한 뒤 서비스를 활성화한다.

```bash
sudo addgroup --system vehicle-computer
sudo adduser --system --ingroup vehicle-computer \
  --home /var/lib/vehicle-computer --no-create-home vehicle-computer
sudo adduser vehicle-computer dialout
sudo adduser vehicle-computer video
sudo adduser vehicle-computer render
sudo install -d /etc/vehicle-computer
sudo install -m 0644 deploy/systemd/vehicle-computer.env.example \
  /etc/vehicle-computer/vehicle-computer.env
sudo install -m 0644 src/vc_bringup/config/vehicle.yaml \
  /etc/vehicle-computer/vehicle.yaml
sudo install -m 0644 src/vc_safety/config/safety.yaml \
  /etc/vehicle-computer/safety.yaml
sudo install -m 0644 src/vc_bringup/config/camera.yaml \
  /etc/vehicle-computer/camera.yaml
sudo install -m 0644 deploy/systemd/vehicle-computer.service \
  /etc/systemd/system/vehicle-computer.service
sudo systemctl daemon-reload
sudo systemctl enable --now vehicle-computer.service
```

환경 파일의 stable serial path, 차량 설정과 전용 `ROS_DOMAIN_ID`를 실제
운영 값으로 바꾼 뒤 활성화해야 한다. 안전 설정 파일은 누락, 알 수 없는 키,
잘못된 타입이나 범위가 있으면 bringup 전체가 시작되지 않는다. systemd는
SIGINT로 종료해 안전 게이트의 명시적 zero 전송 경로를 실행한다.

IMX219는 hardware launch에서 기본으로 시작하며 `camera_ros`의 표준
`/camera/image_raw`, `/camera/image_raw/compressed`, `/camera/camera_info`
인터페이스를 제공한다. 먼저 `rpicam-hello --list-cameras`로 libcamera
인식을 확인하고 다음처럼 카메라만 검증할 수 있다.

```bash
ros2 launch vc_bringup camera.launch.py \
  camera_config:=$(ros2 pkg prefix vc_bringup)/share/vc_bringup/config/camera.yaml
ros2 topic hz /camera/image_raw
ros2 topic echo --once /camera/image_raw/compressed --field format
```

기본값은 1280x720, 30 Hz, `camera_optical_frame`이다. 렌즈 보정 전
`CameraInfo`의 내부 파라미터는 0일 수 있으므로 거리·투영·왜곡 보정에
사용하기 전에 `camera_calibration`으로 보정하고 `camera_info_url`을
설정한다. 카메라가 없는 개발 환경에서는 `vehicle.launch.py`에
`camera:=false`를 전달한다.

컨테이너에서 실행할 때는 `video`/`render` 그룹만으로 충분하지 않으며
호스트의 `/dev/media*`, `/dev/video*`, `/dev/v4l-subdev*`,
`/dev/dma_heap/*`를 컨테이너에 명시적으로 전달해야 한다. Raspberry Pi
OS의 libcamera 포크와 컨테이너의 libcamera ABI를 섞지 않는다.
`dependencies.repos`는 현재 RPi5 CFE/PiSP에서 검증한 Raspberry Pi
`libcamera` 포크를 고정하며, colcon overlay가 ROS 저장소의 일반
libcamera보다 먼저 로드되어야 한다.

상위 제어기는 `/cmd_vel`이 아니라 `/cmd_vel_request`를 10 Hz 이상으로
발행한다. ECU diagnostics와 raw sensor가 정상인 상태에서 명시적으로
enable한다.

```bash
ros2 service call /vehicle/motion_enable std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /vehicle/safety/diagnostics
```

disable, 입력 timeout, 통신 또는 센서 이상은 즉시 zero command를 만들고
enable latch를 해제한다. 상태가 복구돼도 새 enable과 그 이후의 새 명령
없이는 다시 움직이지 않는다. 게이트는 ECU IMU 진단, sensor frame,
NaN/Inf, 지원하지 않는 Twist 축과 설정된 선속도·각속도 한계도 검사한다.
enable 이후 첫 명령 publisher가 해당 세션의 소유자가 되며, 다른 publisher의
명령이 들어오면 latch를 해제한다.
현재 차단 원인과 마지막 trip 원인은 각각 `block_reason`,
`last_trip_reason`으로 구분된다.

MCAP 기록과 callback tracing은 hardware launch 옵션으로 활성화한다.

```bash
ros2 launch vc_bringup vehicle.launch.py \
  serial_device:=/dev/serial/by-id/<CP2102-device> \
  record:=true snapshot_mode:=true trace:=true

ros2 service call /rosbag2_recorder/snapshot rosbag2_interfaces/srv/Snapshot
```
