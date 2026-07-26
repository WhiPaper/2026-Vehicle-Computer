# ROS 2-native vehicle computer design

## Implementation status

Implemented in the current workspace:

- lifecycle safety component and standalone single-threaded executable;
- typed/read-only parameter descriptors, relative names, QoS callbacks,
  topic statistics, ECU and command publisher GID tracking;
- split hardware/fake/replay launch profiles and bounded Agent restart;
- composed `robot_state_publisher`, isolated EKF and safety processes;
- `diagnostic_updater` EKF monitor, diagnostics mux and aggregator;
- MCAP recording with QoS overrides, snapshot mode and build metadata;
- non-interactive `ros2_tracing` launch profile;
- lifecycle, namespace, QoS, ECU restart, EKF/TF and safe-output tests.

SROS 2 deployment policy remains optional deployment work because enclave
identities and host network boundaries must be selected for the target vehicle.

## 1. Design principles

This design uses ROS 2 features where they improve lifecycle control,
observability, testability, or integration. The ESP32 wire contract and the
independent ECU watchdog remain unchanged.

1. The safety decision is deterministic and remains valid without DDS event
   delivery, topic statistics, or a lifecycle manager.
2. Lifecycle state and motion enable state are separate. An `active` safety
   node is allowed to evaluate requests; it does not mean that motion is
   enabled.
3. Production topic names are selected by launch remapping. Application code
   uses relative names so the stack can run under a namespace in tests and
   multi-vehicle systems.
4. Composition is used only where shared process ownership is acceptable.
   Safety-critical and restart-prone processes are isolated.
5. ROS time is used for message timestamps and TF. A steady clock is used for
   watchdogs, publish cadence, and receive timeouts.

## 2. Target runtime architecture

```text
Process: micro_ros_agent (launch respawn)
  serial XRCE-DDS <----------------------------> vehicle_ecu

Process: vehicle_state_container
  robot_state_publisher component
    joint_states ------------------------------> tf / tf_static
  future non-safety sensor adapters

Process: ekf_node
  odom + imu/data_raw --------------------------> odometry/filtered
                                                  odom -> base_link

Process: vc_safety_gate (LifecycleNode)
  cmd_vel_request + ECU health ----------------> cmd_vel
  lifecycle services
  motion_enable service
  motion_enabled transient state
  diagnostics and topic statistics

Process: diagnostics_aggregator
  diagnostics ---------------------------------> vehicle health tree

Optional process: rosbag2 recorder
  selected topics, tf, diagnostics, statistics -> MCAP flight record
```

`robot_state_publisher` is available as a Jazzy component. The installed Jazzy
`robot_localization` package does not export its EKF as a component, so the EKF
remains a separate process. The safety gate also remains isolated: an error in
visualization or state publication must not terminate command supervision.

## 3. Managed safety gate

`vc_safety_gate` becomes an `rclcpp_lifecycle::LifecycleNode` while the
pure `SafetyState` class remains ROS-independent.

| Lifecycle state | Resources | Motion behavior |
|---|---|---|
| `unconfigured` | parameters only | no non-zero publisher |
| `inactive` | subscriptions, services, timers allocated | zero command, enable rejected |
| `active` | lifecycle publishers active | enable may be accepted if ready |
| `error/finalized` | best-effort final zero before transition | latch and stored command cleared |

Lifecycle transitions have the following responsibilities:

- `on_configure`: validate all parameters atomically, create endpoints, reset
  the state machine, and publish the initial disabled state.
- `on_activate`: activate publishers and begin supervision in the disabled
  state. It never restores a previous enable.
- `on_deactivate`, `on_cleanup`, `on_shutdown`, `on_error`: publish zero while
  the publisher is active, clear the enable latch and cached command, then
  release resources as appropriate.

Launch drives `configure -> activate` after the process starts. Agent
availability is not a lifecycle prerequisite: the active gate observes the
missing diagnostics and rejects motion. The standard lifecycle services and
`/vehicle/motion_enable` serve different purposes and must not be coupled.

## 4. ROS graph and names

Node code uses relative names:

| Node-local name | Default production remap |
|---|---|
| `cmd_vel_request` | `/cmd_vel_request` |
| `cmd_vel` | `/cmd_vel` |
| `imu/data_raw` | `/imu/data_raw` |
| `odom` | `/odom` |
| `joint_states` | `/joint_states` |
| `diagnostics` | `/diagnostics` |
| `motion_enable` | `/vehicle/motion_enable` |
| `motion_enabled` | `/vehicle/motion_enabled` |
| `safety/diagnostics` | `/vehicle/safety/diagnostics` |
| `safety/statistics` | `/vehicle/safety/statistics` |

The default launch preserves the existing public contract. A `namespace`
launch argument and explicit remappings support fake-ECU tests. TF frames
retain their ECU-defined unprefixed names. Multi-vehicle deployment requires a
separate frame-rewriting boundary and is intentionally not exposed as a
partially working launch option.

## 5. QoS and DDS graph events

The ECU-compatible QoS remains authoritative:

- sensor inputs: best-effort, volatile, keep-last depth 1;
- diagnostics and commands: reliable, volatile, keep-last depth 1;
- current motion-enabled state: reliable, transient-local, keep-last depth 1.

The safety gate adds subscription and publisher event callbacks for:

- incompatible QoS;
- publisher matched/unmatched;
- message lost where supported by the RMW implementation;
- liveliness changes where exposed by the existing ECU QoS.

These events improve diagnosis but do not replace steady-clock receive
timeouts. A finite requested DDS deadline or custom liveliness lease is not
added because the ECU currently offers the default policies; requesting a
stronger policy could make the endpoints incompatible.

Publisher GIDs from odometry, IMU, and diagnostics message metadata are tracked.
A changed ECU publisher identity, Agent rediscovery, or disappearance
immediately clears the motion latch. This closes the case where a restarted ECU
returns with otherwise valid, monotonic timestamps.

The first valid command publisher after enable owns that motion session. A
different command publisher GID clears the latch, preventing two controllers
from racing while continuously refreshing the command watchdog.

## 6. Parameters

Safety parameters are generated and validated with
`generate_parameter_library` or equivalent typed descriptors:

- positive integer ranges for timeout values;
- positive floating-point ranges for publish rates;
- read-only maximum linear and angular command speeds;
- expected odometry, base, and IMU frames;
- descriptions and units visible through `ros2 param describe`;
- safety timeouts and topic bindings marked read-only after configuration.

Vehicle dimensions remain in one versioned calibrated YAML file. Launch converts
them into Xacro arguments and rejects an unknown schema, unknown keys, missing,
non-finite, zero, or negative values.
Parameters that affect the safety envelope are never changed while the gate is
active. A required change follows:

```text
disable motion -> deactivate -> set parameters -> configure -> activate
```

ROS parameter events are recorded for traceability. Runtime display and
diagnostic-rate parameters may be dynamic only when they do not affect the
safety decision.

## 7. Executors and callback groups

The safety gate uses a single-threaded executor and one mutually-exclusive
callback group for sensor, diagnostics, command, service, and timer callbacks.
This provides a total ordering of state-machine mutations without depending on
a recursive mutex.

If later profiling proves that diagnostic formatting delays the 20 Hz command
timer, formatting moves to a second callback group with a multi-threaded
executor. The safety state snapshot is copied under a short lock; the command
decision remains in the mutually-exclusive group.

The component container uses a multi-threaded executor only for non-safety
components. Intra-process communication is enabled when both endpoints are in
that container.

## 8. Diagnostics and statistics

The safety gate publishes its fail-closed decision snapshot directly, while the
state-estimation monitor uses `diagnostic_updater`. `diagnostic_aggregator`
groups:

```text
/Vehicle
  /ECU/Transport
  /ECU/Drive
  /ECU/IMU
  /Computer/SafetyGate
  /Computer/StateEstimation
```

The existing safety diagnostic keys remain stable. Additional keys include
the lifecycle state, ECU publisher identity generation, QoS incompatibility
count, current block reason, last trip reason, and trip count.

ROS 2 topic statistics are enabled on `odom`, `imu/data_raw`, and
`cmd_vel_request`, publishing to `safety/statistics`. They provide message-age
and period distributions for operations and bag analysis. Safety decisions
continue to use the existing per-message timestamp and steady-clock checks
because statistics are windowed and intentionally delayed.

## 9. Launch structure

Launch is divided into reusable files:

```text
vc_bringup/launch/
  agent.launch.py
  state_estimation.launch.py
  safety.launch.py
  vehicle.launch.py
  recording.launch.py
```

`vehicle.launch.py` exposes:

- `serial_device` (required stable by-id path);
- `baudrate`, `agent_verbosity`, `use_sim_time`;
- `vehicle_config`, `namespace`;
- `record`, `record_storage_id`;
- `log_level`.

Launch uses:

- composable-node actions for `robot_state_publisher`;
- lifecycle transition events for the safety gate;
- process-exit handlers and bounded Agent respawn followed by launch shutdown;
- shutdown handlers that request disable before process teardown;
- explicit remappings rather than hard-coded global names.

Three launch profiles share the same nodes and contracts:

- `hardware`: real Agent and calibrated vehicle file;
- `fake_ecu`: deterministic publishers and fault injection for CI;
- `replay`: rosbag2 input with `use_sim_time=true`, with `/cmd_vel` remapped to
  a non-hardware sink so replay can never move the vehicle.

The production deployment runs the hardware profile under systemd. SIGINT is
used for controlled shutdown, and an exhausted Agent restart budget causes the
service supervisor to create a fresh disabled session.

## 10. Interface selection

- Topics remain the correct interface for continuous sensor, command, TF,
  diagnostics, and state streams.
- `SetBool` remains the short, acknowledged motion-enable service.
- Lifecycle services manage process readiness, not vehicle motion.
- Actions are introduced only for cancellable long-running operations such as
  wheel calibration or a stationary self-test. Velocity streaming is never
  modeled as an action.

No ROS interface is added merely to demonstrate a feature.

## 11. Recording, tracing, and security

The recording launch uses rosbag2 with MCAP and records:

- raw and filtered state;
- commands before and after the gate;
- diagnostics, parameter events, statistics;
- `/tf` and `/tf_static`.

The hardware acceptance profile supports a bounded flight-recorder mode and
records metadata containing the git revision and vehicle calibration file
hash. `ros2_tracing` is enabled in a separate profiling launch profile to
measure callback and executor latency without affecting normal deployment.
Replay remaps recorded command, EKF, TF, and safety output topics below
`replay/recorded`; only raw inputs and command requests retain their recorded
names.

Production deployment assigns a dedicated `ROS_DOMAIN_ID`, restricts DDS to
the intended interface or localhost where applicable, and can add SROS 2
enclaves. The safety enclave is allowed to publish `/cmd_vel`; navigation and
teleop enclaves are allowed to publish only `cmd_vel_request`.

## 12. Deliberately excluded

- `ros2_control` is not placed between the RPi and ECU in this phase. The ECU
  already owns the motor loop, odometry, watchdog, and fixed wire contract;
  adding a parallel controller would create duplicate ownership.
- Loaned messages and zero-copy transport are not safety requirements for
  small Twist/IMU/Odometry messages crossing the XRCE-DDS boundary.
- DDS deadline and lease policies are not strengthened until the ECU publishes
  matching offered QoS.
- Lifecycle activation never implies motion enable.

## 13. Implementation sequence

1. Refactor the safety node to a lifecycle component/standalone executable,
   add relative names, typed parameter descriptors, and GID/QoS event tracking.
2. Split launch files and add lifecycle transition handling, namespace/remap
   support, shutdown disable, and hardware/fake/replay profiles.
3. Compose `robot_state_publisher`, then add diagnostic updater/aggregator and
   topic statistics.
4. Add launch tests for lifecycle transitions, Agent/ECU identity changes,
   namespace isolation, QoS incompatibility, EKF TF ownership, and safe replay.
5. Add MCAP recording and tracing profiles, followed by optional SROS 2 policy.

Each step retains the existing ECU contract and must pass the current
fail-closed safety tests before the next step begins.
