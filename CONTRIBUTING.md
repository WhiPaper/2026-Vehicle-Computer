# 기여 가이드

이 저장소의 문서와 ROS interface는 차량 운영 계약의 일부다. 기능이나 설정을
바꾸는 pull request는 코드만 통과하면 완료된 것으로 보지 않는다.

## 변경별 갱신 대상

| 변경 | 함께 갱신할 문서·검사 |
|---|---|
| 토픽·서비스·QoS·frame | docs/interfaces.md, architecture.md, 관련 test |
| launch argument | 실제 launch description, interfaces.md, development.md, docs checker |
| safety parameter·timeout·speed | safety.yaml, interfaces.md, operations.md, acceptance-testing.md |
| vehicle schema·calibration | vehicle config, interfaces.md, operations.md, CFG 시험 |
| hardware/fake/replay/simulation profile | development.md, visualization.md, acceptance-testing.md |
| deployment·systemd·APT | operations.md, README.md, release workflow |
| safety state/failure reason | architecture.md, interfaces.md, SAFE/CONN 시험 |
| public C++/Python API | source comment/docstring, interfaces.md 또는 package 설명 |
| vendor dependency | upstream 문서와 license/commit 정보를 별도로 확인 |

문서의 기준은 현재 구현이다. 구현에 없는 기능은 사용 예제로 쓰지 말고
미지원 또는 향후 계획으로 표시한다.

## 문서 작성 규칙

- 프로젝트 고유 Markdown은 한국어로 작성한다.
- ROS message/service name, topic, parameter, frame, shell command는 원문을
  유지한다.
- 안전 동작은 조건, 시간, output, 재활성화 조건을 함께 쓴다.
- 운영 명령에는 대상 profile과 필요한 사전조건을 적는다.
- 실제 차량을 움직일 수 있는 명령에는 wheel 고정·검증 조건을 명시한다.
- 현재 구현과 향후 설계를 같은 목록에 섞지 않는다.
- 설정 예제와 운영 설정을 구분한다.
- vendor 디렉터리의 upstream 문서는 번역하거나 프로젝트 계약으로 복사하지
  않는다.

## 변경 전 확인

다음 정보를 먼저 source에서 확인한다.

- launch의 DeclareLaunchArgument와 실제 기본값
- package.xml/setup.py version
- YAML schema와 validator
- publisher/subscriber/service owner
- QoS, 주기, frame, timeout
- 기존 test와 acceptance evidence

추측으로 인터페이스 표를 갱신하지 않는다. 코드가 문서와 다르면 먼저
현재 동작을 기준으로 문서를 고치고, 동작을 바꾸려는 경우 별도 설계와
안전시험을 포함한다.

## 로컬 검증

    pre-commit run --all-files
    python3 scripts/check_docs.py
    colcon test --packages-select \
      vc_description vc_safety vc_bringup vc_visualization
    colcon test-result --verbose

ROS graph를 바꾸는 변경은 fake stack과 replay isolation을 확인한다.
hardware safety 동작을 바꾸는 변경은 [인수시험 체크리스트](docs/acceptance-testing.md)의
관련 ID를 실행하고 bag/log/hash 증적 위치를 기록한다.

## Pull request 설명

다음 항목을 PR 설명에 포함한다.

- 변경한 public interface와 ownership
- 갱신한 문서 목록
- 실행한 정적 검사와 ROS test
- 변경한 설정의 이전/이후 값과 안전 영향
- 인수시험 ID, 결과, 증적 위치
- 동작을 바꾸지 않은 경우 그 사실

문서 검증 스크립트가 실패한 상태로 merge하지 않는다.
