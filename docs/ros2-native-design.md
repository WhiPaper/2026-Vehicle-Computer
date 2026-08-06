# ROS 2-native 차량 컴퓨터 설계

이 문서는 현재 workspace가 ROS 2 기능을 어떤 경계에 적용했는지와, 아직
도입하지 않은 기능을 분리해서 설명한다. 현재 지원되는 public contract는
[인터페이스 레퍼런스](interfaces.md), 실제 운영은 [운영 가이드](operations.md)를
기준으로 한다.

## 구현 상태

### 현재 구현

- lifecycle safety gate component와 standalone executable
- typed parameter descriptor, read-only safety parameter, relative topic name
- QoS event callback, topic statistics, ECU·command publisher GID tracking
- hardware/fake/replay profile과 bounded Agent restart
- composed robot_state_publisher, isolated EKF와 safety process
- diagnostic_updater, diagnostics mux와 aggregator
- MCAP recording, QoS override, snapshot mode, build metadata
- 별도 ros2_tracing launch
- lifecycle, namespace, QoS, ECU restart, EKF/TF, replay isolation test

### 향후 또는 배포 선택 사항

SROS 2 enclave 정책은 target vehicle의 enclave identity와 host network
boundary가 정해진 뒤 별도 배포 항목으로 결정한다. 다중 차량의 frame rewrite,
wheel calibration action, stationary self-test action도 현재 public contract에
포함하지 않는다.

## 1. 설계 원칙

1. safety decision은 DDS event 전달, topic statistics, lifecycle manager 없이도
   steady-clock과 입력 검증으로 결정된다.
2. lifecycle state와 motion enable은 서로 다른 상태다. active gate도 기본은
   disabled다.
3. production topic 이름은 launch remapping으로 선택하고 application code는
   relative name을 사용한다.
4. composition은 process ownership을 공유해도 되는 non-safety component에만
   사용한다. safety와 restart-prone process는 분리한다.
5. ROS time은 message timestamp와 TF에 사용하고, watchdog·publish cadence·
   receive timeout은 steady clock을 사용한다.
6. 관찰용 diagnostics와 statistics는 safety decision을 override하지 않는다.
7. RPi policy는 ECU의 독립 motor loop와 watchdog에 추가되며, 이를 대체하지 않는다.

## 2. Runtime architecture

    Process: micro_ros_agent
      serial XRCE-DDS <------------------------------> vehicle_ecu

    Process: vehicle_state_container
      robot_state_publisher component
        joint_states ---------------------------------> tf / tf_static

    Process: ekf_filter_node
      odom + imu/data_raw ----------------------------> odometry/filtered
                                                        odom -> base_link

    Process: safety_gate (LifecycleNode)
      cmd_vel_request + ECU health -------------------> cmd_vel
      lifecycle services
      motion_enable service
      motion_enabled state
      safety diagnostics/statistics

    Process: diagnostics_aggregator
      diagnostics ------------------------------------> vehicle health tree

    Optional: rosbag2 recorder
      selected topics, tf, diagnostics, statistics --> MCAP

Jazzy의 robot_state_publisher는 component로 실행한다. 설치된
robot_localization EKF는 component로 export되지 않으므로 별도 process로
유지한다. 안전 gate도 별도 process로 유지하여 RViz2나 state publication의
오류가 command supervision을 종료시키지 않게 한다.

## 3. Managed safety gate

| lifecycle state | 리소스 | motion 동작 |
|---|---|---|
| unconfigured | parameter 중심 | non-zero command 없음 |
| inactive | subscription/service/timer 준비 | zero command, enable 거부 |
| active | lifecycle publisher 활성 | readiness가 있으면 enable 검토 |
| error/finalized | best-effort final zero | latch와 cached command 제거 |

전이는 다음 책임을 가진다.

- on_configure: parameter를 검증하고 endpoint를 만들며 state를 reset한다.
- on_activate: publisher를 activate하고 disabled supervision을 시작한다.
- on_deactivate/on_cleanup/on_shutdown/on_error: zero를 발행하고 enable latch와
  command를 지운 뒤 리소스를 해제한다.
- launch는 process 시작 뒤 configure → activate를 요청한다.
- Agent availability는 lifecycle prerequisite가 아니다. gate가 diagnostics
  부재를 관찰하고 motion enable을 거부한다.

lifecycle active는 vehicle motion permission이 아니다. motion은 별도
vehicle/motion_enable service의 성공을 요구한다.

## 4. Names와 namespace

Node code는 다음 relative name을 사용한다.

| node-local name | 기본 production 이름 |
|---|---|
| cmd_vel_request | /cmd_vel_request |
| cmd_vel | /cmd_vel |
| imu/data_raw | /imu/data_raw |
| odom | /odom |
| joint_states | /joint_states |
| diagnostics | /diagnostics |
| vehicle/motion_enable | /vehicle/motion_enable |
| vehicle/motion_enabled | /vehicle/motion_enabled |
| vehicle/safety/diagnostics | /vehicle/safety/diagnostics |
| vehicle/safety/statistics | /vehicle/safety/statistics |

namespace launch argument와 explicit remap으로 fake/replay/simulation을 격리한다.
TF frame은 현재 ECU 계약의 unprefixed 이름을 유지한다. 다중 차량 frame
rewrite는 별도 경계가 없으므로 현재 launch에서 노출하지 않는다.

## 5. QoS와 DDS event

현재 authoritative QoS는 다음과 같다.

- sensor input: best-effort, volatile, keep-last depth 1
- command와 diagnostics: reliable, volatile, keep-last depth 1
- motion-enabled state: reliable, transient-local, keep-last depth 1
- tf_static: reliable, transient-local

safety gate는 다음 event를 관찰한다.

- incompatible QoS
- publisher matched/unmatched
- RMW가 제공하는 message lost
- ECU QoS가 노출하는 liveliness 변화
- odom, IMU, diagnostics publisher GID 변경

event는 조기 진단과 latch 해제에 사용하지만 receive timeout을 없애지 않는다.
ECU가 제공하지 않는 deadline이나 custom liveliness lease를 요청하여 endpoint
compatibility를 깨지 않는다.

command publisher도 GID를 추적한다. enable 이후 다른 writer가 들어오면 두
controller가 watchdog을 동시에 갱신하지 못하도록 latch를 해제한다.

## 6. Parameters

safety gate는 C++ ParameterDescriptor로 type, description, range, read-only
속성을 선언한다. 현재 range는 positive integer 1–60000, positive double
0.001–1000이며, 실제 설정 파일의 단위와 의미는
[인터페이스 레퍼런스](interfaces.md)에 있다.

다음 값은 active 상태에서 바꾸지 않는다.

- timeout과 publish/status rate
- speed limit
- expected odom/base/IMU frame
- topic statistics enable/name

필요한 변경 순서:

    disable motion
    deactivate lifecycle
    set validated parameters
    configure
    activate

vehicle dimensions는 versioned calibrated YAML 하나에서 Xacro argument로
전달한다. unknown schema/key, missing value, non-finite, zero, negative
dimension은 launch에서 거부한다.

## 7. Executor와 callback group

safety gate는 single-threaded executor와 mutually-exclusive callback group으로
sensor, diagnostics, command, service, timer callback의 state mutation을
total ordering한다. 이 결정은 recursive mutex에 의존하지 않는다.

향후 diagnostic formatting이 20 Hz command timer를 지연시킨다는 측정 결과가
나오면 formatting만 별도 callback group으로 옮긴다. safety snapshot은 짧은
lock 아래 복사하고 command decision은 mutually-exclusive group에 남긴다.

non-safety component container는 multi-threaded executor와 intra-process
communication을 사용할 수 있다.

## 8. Diagnostics와 statistics

safety gate는 fail-closed decision snapshot을 직접 발행한다. state estimation
monitor는 diagnostic_updater를 사용하고, aggregator는 다음 tree를 만든다.

    /Vehicle
      /ECU/Transport
      /ECU/Drive
      /ECU/IMU
      /Computer/SafetyGate
      /Computer/StateEstimation
      /Computer/SerialDevice
      /Computer/ECUConnection

safety diagnostics에는 lifecycle state, ECU publisher identity generation,
QoS incompatibility count, current block reason, last trip reason, trip count를
포함한다.

odom, imu/data_raw, cmd_vel_request의 topic statistics는 operations와 bag
analysis를 위한 windowed metric이다. safety는 message timestamp와 steady
clock age를 계속 authoritative하게 사용한다.

## 9. Launch 구조

주요 reusable launch는 다음과 같다.

    vc_bringup/launch/
      agent.launch.py
      state_estimation.launch.py
      safety.launch.py
      vehicle.launch.py
      recording.launch.py
      replay.launch.py

vehicle.launch.py의 실제 public argument는 serial_device, baudrate,
agent_verbosity, agent_respawn_delay, agent_respawn_limit, use_sim_time,
namespace, record, camera, snapshot_mode, trace, bag_output,
record_storage_id, trace_session, trace_path, vehicle_config, safety_config,
camera_config이다. 지원되지 않는 logging-level 인자는 문서나 운영 스크립트에서
사용하지 않는다.

launch는 다음 원칙을 따른다.

- robot_state_publisher는 composable-node action으로 실행
- safety lifecycle은 transition event로 configure/activate
- Agent는 bounded restart 후 launch shutdown
- shutdown에서 disable/zero 경로를 먼저 요청
- global hard-coded name 대신 relative name과 explicit remap 사용

## 10. Interface 선택

- sensor, command, TF, diagnostics, state stream은 topic을 사용한다.
- motion enable은 짧고 확인 가능한 SetBool service를 사용한다.
- lifecycle service는 process readiness를 관리하고 vehicle motion을 관리하지
  않는다.
- wheel calibration이나 stationary self-test 같은 cancellable 장기 작업이
  필요해질 때만 action을 검토한다.
- velocity streaming 자체를 action으로 모델링하지 않는다.

## 11. Recording, tracing, security

recording launch는 raw/filtered state, command before/after gate,
diagnostics, parameter events, statistics, tf, tf_static을 MCAP에 기록한다.
metadata에는 git revision과 vehicle calibration file hash를 넣는다.

ros2_tracing은 정상 운영과 분리된 profiling launch에서 callback/executor
latency를 측정한다. profiling을 위해 safety decision semantics를 바꾸지 않는다.

운영은 dedicated ROS_DOMAIN_ID와 승인된 network interface를 사용한다.
SROS 2 enclave는 target network와 identity가 결정된 뒤 추가한다. navigation과
teleop은 cmd_vel_request만, safety enclave는 cmd_vel을 발행하는 정책을
고려할 수 있다.

## 12. 검증과 변경 순서

설계 변경은 다음 순서를 따른다.

1. 인터페이스와 ownership을 문서에서 먼저 갱신한다.
2. fake/replay/simulation에서 namespace와 isolation을 검증한다.
3. fail-closed safety test를 통과시킨다.
4. hardware acceptance에서 실제 device, timing, QoS, recovery를 검증한다.
5. 기록된 bag와 설정 hash를 release evidence로 보관한다.

문서 정합성은 python3 scripts/check_docs.py, pre-commit, ROS package test로
검증한다. 기능 변경 없이 문서·argument description·API comment만 바꾼
경우에도 이 검사를 실행한다.
