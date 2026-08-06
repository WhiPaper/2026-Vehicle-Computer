# 운영 가이드

이 문서는 Raspberry Pi 5에 차량용 컴퓨터를 설치하고, 안전하게 시작하고,
장애를 분류하고, 이전 버전으로 되돌리는 절차다. 기준 환경은 Ubuntu 24.04,
ROS 2 Jazzy, package version 0.2.2이다.

실차 작업은 차체를 고정하고 모든 wheel을 지면에서 띄운 상태에서 시작한다.
motion enable은 이 문서의 preflight와 [인수시험 체크리스트](acceptance-testing.md)를
통과한 뒤에만 요청한다.

## 운영 경로 선택

| 경로 | 용도 | 기준 |
|---|---|---|
| signed APT | 정식 운영 배포 | 권장. release provenance와 architecture별 package 사용 |
| 직접 workspace + systemd | 커스텀 빌드, 개발, 긴급 검증 | 운영 전에 동일 인수시험을 다시 실행 |
| fake/replay/simulation | CI와 교육 | 실제 serial과 hardware cmd_vel에 접근하지 않음 |

## APT 설치

### 저장소 등록

저장소 관리자가 GitHub Pages를 Actions source로 설정하고
APT_GPG_PRIVATE_KEY와 APT_GPG_PASSPHRASE secret을 등록해야 한다. 차량
운영자는 공개 keyring을 등록한 뒤 package를 설치한다.

    curl -fsSL \
      https://whipaper.github.io/2026-Vehicle-Computer/vehicle-computer-archive-keyring.gpg \
      | sudo tee /usr/share/keyrings/vehicle-computer.gpg >/dev/null
    echo "deb [signed-by=/usr/share/keyrings/vehicle-computer.gpg] \
      https://whipaper.github.io/2026-Vehicle-Computer noble main" \
      | sudo tee /etc/apt/sources.list.d/vehicle-computer.list
    sudo apt update
    sudo apt install ros-jazzy-vehicle-computer

서비스는 package 설치만으로 자동 활성화되지 않는다. 차량별 설정을 먼저
검토한 뒤 enable한다.

### 설정

설치 후 다음 파일을 검토한다.

    sudoedit /etc/vehicle-computer/vehicle-computer.env
    sudoedit /etc/vehicle-computer/vehicle.yaml
    sudoedit /etc/vehicle-computer/safety.yaml
    sudoedit /etc/vehicle-computer/camera.yaml

필수 확인 항목:

- VEHICLE_SERIAL_DEVICE가 단일 stable /dev/serial/by-id/<device>인지 확인
- ROS_DOMAIN_ID를 승인된 차량 controller만 사용하는 값으로 지정
- vehicle.yaml의 schema_version이 1인지 확인
- 차량 치수와 IMU pose가 실측값인지 확인
- vehicle.calibrated가 true인지 확인
- ECU의 wheel radius와 track width가 일치하는지 확인
- max_linear_speed_mps와 max_angular_speed_rps가 검증된 한계인지 확인
- camera_info_url이 보정 파일을 가리키는지 확인

설정 파일은 service user가 읽을 수 있어야 하지만 일반 사용자가 임의로
안전 한계를 바꾸지 못하도록 운영 정책에 맞는 소유권과 권한을 적용한다.
설정 변경 후에는 파일 SHA-256과 인수시험 결과를 기록한다.

## 직접 workspace + systemd

이 경로는 source build 또는 커스텀 libcamera가 필요한 경우에 사용한다.
workspace를 /opt/vehicle-computer에 배치하고 다음 파일을 설치한다.

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

서비스 파일은 /opt/vehicle-computer/install/setup.bash를 선택적으로
source하고 SIGINT로 종료한다. 시작 전에 env 파일의 serial path,
namespace, config path, camera 설정, ROS_DOMAIN_ID를 변경한다.

    sudo systemctl enable vehicle-computer.service
    sudo systemctl start vehicle-computer.service

## 시작 전 preflight

실차에서 다음 순서로 확인한다.

1. 차체를 고정하고 wheel 주변의 사람·장애물을 제거한다.
2. stable serial device가 존재하고 service user가 읽기·쓰기 가능한지 확인한다.
3. camera를 사용할 경우 rpicam-hello --list-cameras로 IMX219를 확인한다.
4. vehicle.yaml과 safety.yaml의 hash와 변경 승인 기록을 확인한다.
5. ROS_DOMAIN_ID가 다른 차량과 격리되어 있는지 확인한다.
6. service를 시작하고 safety gate가 active인지 확인한다.
7. motion_enabled가 false인지 확인한다.
8. diagnostics, odom, IMU 주기와 frame을 확인한다.
9. wheel을 띄운 상태에서만 safe command 시험을 수행한다.
10. 모든 시험을 통과한 뒤에만 실제 motion enable을 요청한다.

대표 확인 명령:

    systemctl status vehicle-computer.service --no-pager
    ros2 lifecycle get /safety_gate
    ros2 topic echo --once /vehicle/motion_enabled
    ros2 topic echo /vehicle/safety/diagnostics
    ros2 topic echo /vehicle/diagnostics
    ros2 topic hz /odom
    ros2 topic hz /imu/data_raw
    ros2 topic hz /diagnostics
    ros2 topic info --verbose /cmd_vel

예상 주기는 odom/joint_states 30 Hz, IMU 50 Hz, diagnostics 5 Hz다. raw
sensor는 best-effort, command와 diagnostics는 reliable이어야 한다.

## 정상 동작

hardware launch:

    ros2 launch vc_bringup vehicle.launch.py \
      serial_device:=/dev/serial/by-id/<device>

상위 제어기는 다음처럼 request를 발행한다.

    ros2 topic pub -r 10 /cmd_vel_request geometry_msgs/msg/Twist \
      '{linear: {x: 0.05}, angular: {z: 0.0}}'

request가 정상적으로 들어오고 ECU/sensor readiness가 확인된 뒤에만 enable한다.

    ros2 service call /vehicle/motion_enable std_srvs/srv/SetBool '{data: true}'

정지할 때는 먼저 command publisher를 중단하거나 SetBool false를 요청한다.
command timeout은 250 ms이며, gate는 zero를 발행하고 latch를 해제해야 한다.

    ros2 service call /vehicle/motion_enable std_srvs/srv/SetBool '{data: false}'

## 상태·로그 확인

systemd:

    systemctl is-active vehicle-computer.service
    systemctl status vehicle-computer.service --no-pager
    journalctl -u vehicle-computer.service -b --no-pager
    journalctl -u vehicle-computer.service -f

ROS:

    ros2 node list
    ros2 topic list -t
    ros2 lifecycle get /safety_gate
    ros2 topic echo /vehicle/safety/diagnostics
    ros2 topic echo /vehicle/diagnostics
    ros2 topic echo /vehicle/safety/statistics

diagnostics의 Vehicle/Computer/SerialDevice는 장치의 존재·권한을,
Vehicle/Computer/ECUConnection은 ECU heartbeat와 graph 상태를 나타낸다.
연결 상태가 OK로 복구되어도 motion enable은 자동으로 복구되지 않는다.

## 장애 대응

### Serial device missing 또는 permission denied

1. ls -l /dev/serial/by-id/로 stable link를 확인한다.
2. readlink -f로 실제 device와 group을 확인한다.
3. service user가 dialout, video, render에 속하는지 확인한다.
4. env 파일의 VEHICLE_SERIAL_DEVICE가 단일 by-id 경로인지 확인한다.
5. 권한을 수정했다면 새 login session 또는 service restart를 수행한다.

서비스를 반복 재시작하기 전에 실제 장치 권한과 USB cable을 확인한다.

### Agent 종료 또는 USB 분리

Agent는 2초 간격으로 최대 300회 재시작한다. 재시작 중 safety gate는
disabled이며 zero command를 유지한다. Agent restart limit에 도달하면
현재 launch가 종료되고 systemd가 새 disabled session을 시작한다.

    journalctl -u vehicle-computer.service -b --no-pager
    ros2 topic echo /vehicle/diagnostics
    ros2 topic echo /vehicle/safety/diagnostics

장치와 heartbeat가 복구된 뒤 diagnostics가 정상인지 확인하고, 원인을
기록한 다음 새 enable을 명시적으로 요청한다.

### ECU heartbeat 또는 sensor timeout

ECU diagnostics가 500 ms 이상 없거나 odom/IMU가 200 ms 이상 오래되면 gate는
zero를 발행하고 latch를 해제한다. 다음을 확인한다.

- diagnostics의 transport, drive, IMU, time_synchronized 값
- /odom의 frame_id=odom, child_frame_id=base_link
- /imu/data_raw의 frame_id=imu_link
- topic QoS compatibility와 publisher GID 변경
- ECU firmware 상태와 독립 500 ms watchdog

입력이 정상으로 돌아와도 새 enable과 그 뒤의 새 request가 필요하다.

### Command가 움직이지 않음

다음 순서로 확인한다.

1. lifecycle state가 active인지 확인한다.
2. /vehicle/motion_enabled가 true인지 확인한다.
3. safety diagnostics의 block_reason과 last_trip_reason을 확인한다.
4. request가 10 Hz 이상인지 확인한다.
5. 다른 command publisher가 session을 차지하거나 publisher가 바뀌지 않았는지 확인한다.
6. max speed와 unsupported Twist axis를 확인한다.
7. ECU drive status와 command watchdog을 확인한다.

오류를 우회하기 위해 직접 /cmd_vel을 발행하지 않는다.

## 카메라 운영

카메라 인식:

    rpicam-hello --list-cameras
    ros2 launch vc_bringup camera.launch.py \
      camera_config:=$(ros2 pkg prefix vc_bringup)/share/vc_bringup/config/camera.yaml
    ros2 topic hz /camera/image_raw
    ros2 topic echo --once /camera/camera_info

기본 profile은 1280x720, 30 Hz, camera_optical_frame이다. 보정 전
CameraInfo의 intrinsic 값은 0일 수 있다. 거리·투영·왜곡 보정에는
camera_calibration으로 보정한 YAML을 camera_info_url에 지정한다.

컨테이너에서는 host의 /dev/media*, /dev/video*, /dev/v4l-subdev*,
/dev/dma_heap/*를 명시적으로 전달한다. host Raspberry Pi OS의 libcamera
ABI와 container의 libcamera ABI를 섞지 않는다.

## 기록·replay

장애 증적용 MCAP:

    ros2 launch vc_bringup vehicle.launch.py \
      serial_device:=/dev/serial/by-id/<device> \
      record:=true snapshot_mode:=true

fault가 발생하면:

    ros2 service call /rosbag2_recorder/snapshot rosbag2_interfaces/srv/Snapshot

bag metadata에는 git revision과 vehicle configuration SHA-256이 기록된다.
replay는 별도 ROS domain에서 실행하고 기록된 output은 replay/recorded 아래로
보낸다. 실제 /cmd_vel publisher가 존재하지 않는지 확인한다.

## 업데이트·롤백

업데이트 전에 다음을 보관한다.

- 현재 package version과 git/release 정보
- /etc/vehicle-computer의 env 및 YAML 백업
- 각 설정 파일 SHA-256
- 마지막 인수시험 결과와 bag/log 위치

APT 후보 확인:

    apt-cache policy ros-jazzy-vehicle-computer
    apt-cache madison ros-jazzy-vehicle-computer

업데이트 후에는 service를 자동으로 enable하기 전에 설정과 package version을
확인하고, [인수시험 체크리스트](acceptance-testing.md)의 build/interface,
safe command, replay 시험을 다시 수행한다.

    sudo apt update
    sudo apt install ros-jazzy-vehicle-computer
    sudo systemctl daemon-reload
    sudo systemctl restart vehicle-computer.service

문제가 생기면 저장해 둔 설정을 먼저 복원하고, repository에 남아 있는 이전
release version으로 package를 지정해 재설치한다. 이전 version이 현재
APT Pages snapshot에 없으면 GitHub Release의 해당 architecture package와
SHA-256을 사용한다. rollback 뒤에는 새 session이 disabled인지 확인한다.

## 금지 사항

- wheel이 지면에 닿은 상태에서 처음 enable하지 않는다.
- 직접 /cmd_vel을 발행해 gate를 우회하지 않는다.
- replay 또는 PC 시각화 profile을 hardware controller로 사용하지 않는다.
- 검증되지 않은 속도 한계를 높이지 않는다.
- recovery 직후 자동으로 enable한다고 가정하지 않는다.
- ECU watchdog과 RPi safety gate 중 하나만 안전장치라고 간주하지 않는다.
