# RPi5 인수시험 체크리스트

이 문서는 package version 0.2.2, Ubuntu 24.04, ROS 2 Jazzy 기준의 운영
인수시험 기록 양식이다. 각 시험은 실제 실행 결과와 증적 위치를 채워야
완료된 것으로 본다.

motion 시험은 반드시 차체를 고정하고 모든 wheel을 지면에서 띄운 상태에서
수행한다. ECU calibration이 끝나고 vehicle.calibrated=true인 운영 설정을
사용하기 전에는 실제 hardware launch를 시작하지 않는다.

## 실행 기록

| 항목 | 값 |
|---|---|
| 시험 세트 ID | |
| 차량/ECU 식별자 | |
| 실행자 | |
| 실행 일시 | |
| ROS distribution | Jazzy |
| package version | |
| git revision | |
| vehicle config SHA-256 | |
| safety config SHA-256 | |
| bag/log 위치 | |
| 최종 판정 | PASS / FAIL / BLOCKED |

각 시험의 결과는 PASS, FAIL, BLOCKED 중 하나로 표시한다. FAIL이면 원인과
재시험 조건을 기록하고, BLOCKED이면 필요한 장비·권한·설정을 기록한다.

## 공통 결과 표

| 시험 ID | 결과 | 실행자·일시 | 증적 | 비고 |
|---|---|---|---|---|
| PRE-001 | | | | |
| BUILD-001 | | | | |
| CFG-001 | | | | |
| IF-001 | | | | |
| CAM-001 | | | | |
| TF-001 | | | | |
| SAFE-001 | | | | |
| SAFE-002 | | | | |
| SAFE-003 | | | | |
| CONN-001 | | | | |
| REC-001 | | | | |
| REP-001 | | | | |
| SIM-001 | | | | |

## PRE-001 — 안전 사전조건

목적: 사람·장비·설정의 사전조건을 확인한다.

대상: hardware.

사전조건:

- 차체가 고정되고 wheel이 지면에서 분리됨
- 비상 정지와 ECU watchdog 절차를 운용자가 숙지함
- service user와 승인된 ROS_DOMAIN_ID가 준비됨
- vehicle.yaml과 safety.yaml의 hash가 기록됨

실행:

    id
    groups
    echo "$ROS_DOMAIN_ID"
    ls -l /dev/serial/by-id/
    ls -l "$(readlink -f /dev/serial/by-id/<device>)"
    sha256sum /etc/vehicle-computer/vehicle.yaml
    sha256sum /etc/vehicle-computer/safety.yaml

기대 결과:

- stable by-id path가 단일 장치로 해석됨
- service user가 device를 읽고 쓸 수 있음
- 설정이 승인된 차량과 일치함
- ROS domain이 다른 차량과 공유되지 않음

증적: command output, 설정 승인 기록, 사진 또는 작업 로그.

## BUILD-001 — 빌드·정적·ROS 테스트

목적: 현재 source와 설치 의존성이 재현 가능함을 확인한다.

대상: development/CI.

실행:

    source /opt/ros/jazzy/setup.bash
    vcs import src --skip-existing < dependencies.repos
    rosdep install --from-paths src --ignore-src --rosdistro jazzy -y \
      --skip-keys="libcamera vehicle_computer_runtime"
    colcon build --symlink-install --meson-args \
      -Dpipelines=rpi/pisp -Dipas=rpi/pisp \
      -Dtest=false -Ddocumentation=disabled -Dpycamera=disabled \
      -Dgstreamer=disabled -Dv4l2=disabled
    source install/setup.bash
    pre-commit run --all-files
    python3 scripts/check_docs.py
    colcon test --packages-select \
      vc_description vc_safety vc_bringup vc_visualization
    colcon test-result --verbose

기대 결과:

- build 성공
- 문서 검사와 pre-commit 성공
- 선택된 ROS package test가 모두 PASS
- test result에 failure가 없음

증적: colcon log, test-result, CI run URL 또는 artifact.

## CFG-001 — 보정 설정 fail-closed

목적: 미보정 설정이 hardware launch를 시작시키지 않는지 확인한다.

대상: development/CI.

실행:

    ros2 launch vc_bringup state_estimation.launch.py \
      vehicle_config:=$(ros2 pkg prefix vc_bringup)/share/vc_bringup/config/vehicle.example.yaml

기대 결과:

- launch가 실패함
- calibrated 관련 오류가 출력됨
- vehicle.example.yaml은 calibrated=false를 유지함

운영 설정 확인:

    ros2 launch vc_bringup state_estimation.launch.py \
      vehicle_config:=$(ros2 pkg prefix vc_bringup)/share/vc_bringup/config/vehicle.yaml

기대 결과:

- 승인된 보정값을 가진 경우에만 state estimation이 시작됨
- unknown key, missing value, non-finite, zero/negative dimension은 거부됨

증적: launch output, 두 설정 파일의 hash와 승인 기록.

## IF-001 — ROS graph·주기·QoS

목적: external contract와 publisher ownership을 확인한다.

대상: fake 또는 hardware.

실행:

    ros2 node list
    ros2 topic list -t
    ros2 topic info --verbose /cmd_vel_request
    ros2 topic info --verbose /cmd_vel
    ros2 topic info --verbose /imu/data_raw
    ros2 topic info --verbose /odom
    ros2 topic info --verbose /joint_states
    ros2 topic info --verbose /diagnostics
    ros2 topic hz /imu/data_raw
    ros2 topic hz /odom
    ros2 topic hz /joint_states
    ros2 topic hz /diagnostics
    ros2 lifecycle get /safety_gate

기대 결과:

- IMU 약 50 Hz
- odom과 joint_states 약 30 Hz
- diagnostics 약 5 Hz
- raw sensor는 best-effort, volatile, depth 1
- command와 diagnostics는 reliable, volatile, depth 1
- physical /cmd_vel publisher는 vc_safety 하나
- lifecycle state는 자동 start profile에서 active이지만 motion_enabled는 false

증적: topic info, topic hz, node list output.

## CAM-001 — IMX219 camera contract

목적: camera discovery, image rate, frame, calibration 상태를 확인한다.

대상: hardware 또는 camera host.

사전조건:

- libcamera가 host에서 인식됨
- camera config path가 존재함

실행:

    rpicam-hello --list-cameras
    ros2 launch vc_bringup camera.launch.py \
      camera_config:=$(ros2 pkg prefix vc_bringup)/share/vc_bringup/config/camera.yaml
    ros2 topic hz /camera/image_raw
    ros2 topic echo --once /camera/camera_info
    ros2 topic echo --once /camera/image_raw/compressed --field format

기대 결과:

- IMX219 camera가 발견됨
- 기본 profile은 1280x720, 약 30 Hz
- frame은 camera_optical_frame
- compressed format은 jpg
- calibration 전 intrinsic이 0일 수 있음을 기록함
- metric vision 사용 전 camera_info_url 보정 파일을 지정함

증적: rpicam output, topic hz/info, calibration YAML hash.

## TF-001 — TF ownership과 frame

목적: frame tree가 중복 publisher 없이 계약을 따르는지 확인한다.

대상: fake 또는 hardware.

실행:

    ros2 run tf2_ros tf2_echo odom base_link
    ros2 run tf2_ros tf2_echo base_link imu_link
    ros2 run tf2_tools view_frames

기대 결과:

- odom → base_link publisher는 EKF 하나
- base_link → imu_link는 고정 transform
- wheel frame이 URDF에 존재함
- raw odometry는 odom/base_link frame
- raw IMU는 imu_link frame
- ECU가 TF를 발행하지 않음

증적: view_frames PDF/YAML, tf2_echo output.

## SAFE-001 — 정상 enable과 gated command

목적: explicit enable 이후에만 command가 ECU output으로 전달되는지 확인한다.

대상: fake, simulation 또는 wheel을 띄운 hardware.

실행:

    ros2 topic echo /vehicle/motion_enabled
    ros2 topic pub -r 10 /cmd_vel_request geometry_msgs/msg/Twist \
      '{linear: {x: 0.05}, angular: {z: 0.0}}'
    ros2 service call /vehicle/motion_enable std_srvs/srv/SetBool '{data: true}'
    ros2 topic echo /cmd_vel
    ros2 topic echo /vehicle/safety/diagnostics

기대 결과:

- enable 이전에는 non-zero command가 출력되지 않음
- readiness가 충족되면 SetBool response가 성공
- enable 뒤 새 request가 gated cmd_vel에 나타남
- motion_enabled가 true가 됨
- diagnostics에 block_reason이 healthy 상태로 표시됨

마무리:

    ros2 service call /vehicle/motion_enable std_srvs/srv/SetBool '{data: false}'

증적: service response, motion_enabled, safety diagnostics, cmd_vel sample.

## SAFE-002 — command timeout

목적: request publisher가 멈추면 250 ms 안에 zero와 latch 해제가 발생하는지
확인한다.

대상: fake, simulation 또는 wheel을 띄운 hardware.

실행:

    ros2 topic pub -r 10 /cmd_vel_request geometry_msgs/msg/Twist \
      '{linear: {x: 0.05}, angular: {z: 0.0}}'
    ros2 service call /vehicle/motion_enable std_srvs/srv/SetBool '{data: true}'
    # 위 command publisher를 Ctrl-C로 중단
    ros2 topic echo /cmd_vel
    ros2 topic echo /vehicle/motion_enabled
    ros2 topic echo /vehicle/safety/diagnostics

기대 결과:

- command_timeout이 250 ms 이내에 발생
- /cmd_vel이 zero를 발행
- motion_enabled가 false
- last_trip_reason에 command timeout이 기록됨
- 새 enable과 새 command 없이 재활성화되지 않음

증적: timestamp가 포함된 cmd_vel와 diagnostics log.

## SAFE-003 — 잘못된 입력·publisher 변경·복구

목적: invalid input과 DDS writer 변경이 fail-closed 동작을 유지하는지 확인한다.

대상: fake 또는 simulation.

시나리오:

- unsupported Twist axis를 non-zero로 발행
- NaN/Inf 또는 speed limit 초과 command를 발행
- odom/IMU frame을 잘못된 값으로 발행
- diagnostics 또는 sensor timestamp를 회귀시킴
- enable 후 다른 DDS writer로 command를 발행
- diagnostics QoS incompatibility를 유도
- ECU/fake publisher를 재시작

기대 결과:

- 각 시나리오에서 zero가 발행됨
- motion_enabled가 false가 됨
- block_reason과 last_trip_reason이 구분됨
- publisher identity 변경 뒤 old command가 복구되지 않음
- 정상 입력 복구만으로 자동 enable되지 않음

증적: fault injection 명령, diagnostics, trip_count, bag.

## CONN-001 — Agent·USB·ECU recovery

목적: transport failure가 safe session을 만든 뒤 명시적 enable을 요구하는지
확인한다.

대상: hardware.

시나리오:

1. Agent를 종료한다.
2. USB serial device를 분리한다.
3. device를 재연결한다.
4. ECU를 재부팅하거나 micro-ROS session을 재시작한다.
5. diagnostics heartbeat가 복구되는지 관찰한다.

기대 결과:

- SerialDevice 또는 ECUConnection diagnostics가 ERROR가 됨
- safety gate는 disabled와 zero를 유지함
- Agent는 2초 간격으로 재시작함
- 300회 한도에 도달하면 launch가 종료됨
- heartbeat 복구 후 diagnostics는 OK로 돌아올 수 있음
- motion enable은 자동으로 복구되지 않음
- 새 enable과 새 command 뒤에만 motion session이 시작됨

증적: journalctl, diagnostics, Agent restart count, 재연결 시간.

## REC-001 — MCAP flight recorder

목적: 장애 재현에 필요한 command·state·diagnostics와 metadata를 보존하는지
확인한다.

대상: hardware 또는 fake.

실행:

    ros2 launch vc_bringup vehicle.launch.py \
      serial_device:=/dev/serial/by-id/<device> \
      record:=true snapshot_mode:=true

fault를 유도한 뒤:

    ros2 service call /rosbag2_recorder/snapshot rosbag2_interfaces/srv/Snapshot
    ros2 bag info <bag-directory>

기대 결과:

- MCAP storage가 사용됨
- raw/filtered state, request/gated command, diagnostics, statistics, TF가 존재함
- metadata에 git_revision이 존재함
- metadata에 vehicle_config_sha256이 존재함
- bounded snapshot이 fault 시점의 데이터를 보존함

증적: ros2 bag info, metadata, bag path와 SHA-256.

## REP-001 — 격리 replay

목적: 기록을 재생해도 hardware cmd_vel publisher가 만들어지지 않는지 확인한다.

대상: development PC, 별도 ROS domain 권장.

실행:

    ros2 launch vc_visualization replay_visualization.launch.py \
      bag_path:=<bag-directory>
    ros2 topic list -t
    ros2 topic info /cmd_vel
    ros2 topic echo /replay/recorded/vehicle/motion_enabled

기대 결과:

- use_sim_time=true
- filtered odometry/TF/safety output은 replay/recorded 아래
- safety output은 replay/cmd_vel_sink
- physical /cmd_vel publisher가 없음
- bag source와 replay output이 namespace collision을 만들지 않음

증적: topic info/list, RViz screenshot, replay log.

## SIM-001 — Gazebo namespace 격리

목적: simulation이 Agent·serial 없이 /sim namespace에서 실행되는지 확인한다.

대상: development PC.

실행:

    ros2 launch vc_visualization simulation.launch.py
    ros2 topic list -t
    ros2 service call /sim/vehicle/motion_enable std_srvs/srv/SetBool \
      '{data: true}'
    ros2 topic pub -r 10 /sim/cmd_vel_request geometry_msgs/msg/Twist \
      '{linear: {x: 0.05}, angular: {z: 0.0}}'

기대 결과:

- Agent launch 또는 serial device 접근이 없음
- ROS simulation topics가 /sim 아래에 있음
- Gazebo command는 /sim/cmd_vel
- EKF가 /sim/odometry/filtered와 /sim/tf를 소유함
- health, IMU, odom fault 시 safety gate가 zero와 disabled를 유지함

headless 확인:

    ros2 launch vc_visualization simulation.launch.py headless:=true

증적: node/topic list, Gazebo log, safety diagnostics, screenshot.

## 판정 기준

시험 세트는 다음 조건을 모두 만족해야 PASS다.

- PRE-001, CFG-001, IF-001, SAFE-001, SAFE-002 통과
- 대상 장비가 있으면 CAM-001, CONN-001 통과
- hardware motion을 승인하려면 TF-001과 SAFE-003도 통과
- 기록을 운영 증적으로 사용할 경우 REC-001과 REP-001 통과
- simulation profile을 배포 artifact로 사용할 경우 SIM-001 통과
- FAIL 시험에 대한 재시험 결과와 설정·소프트웨어 버전이 기록됨
