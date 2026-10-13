# HPC blocking 차로 변경의 첫 차이와 입력 경계

2026-10-11 기준. 주차는 범위에서 제외한다. 보유 bag의 차로 변경 RTC 인터페이스 전체 `is_auto_mode` 출력은 585/585개가 true지만, 개별 협력 상태 575개는 모두 `auto_mode=false`다. 원본 `RTCInterface::updateCooperateCommandStatus()`는 개별 명령을 받으면 해당 UUID만 수동 상태로 바꾸므로 두 출력은 양립한다. 개별 승인 **명령 입력과 그 수신 시각**은 없다. 아래 승인 재생은 임의로 170~350번에 승인 상태를 준 조건부 실험이며, 기록 당시 원본 실행을 복원한 결과가 아니다.

## 1. 행동 경로의 첫 차이

원본 `LaneChangeInterface::isExecutionRequested()`는 모듈이 `RUNNING`이면 새 요청 계산 결과와 무관하게 실행을 계속한다. `updateData()`는 승인 대기 또는 중단 때만 `updateLaneChangeStatus()`를 호출한다. 따라서 실행 중 경로는 승인된 `status_.lane_change_path.path`를 유지한다. 근거는 원본 `autoware_behavior_path_lane_change_module/src/interface.cpp`의 `isExecutionRequested`, `updateData`와 `src/scene.cpp`의 `NormalLaneChange::generateOutput`이다.

planning-aa도 route와 목표 차로가 같은 동안 새 후보 요청이 비어 있어도 승인된 실행 경로를 유지하도록 고쳤다. 합성 지도 테스트에서 승인, 요청 누락, 명시적 비활성화 순서를 검사했다. 위 가상 승인 재생에서는 실행 경로 프레임이 81→105개로 늘었다. 승인 입력이 없는 기본 재생은 573프레임의 최종 점 수·차량 위치 계획 속도·첫 정지 거리가 기존 CSV와 모두 같다.

원본 `NormalLaneChange::generateOutput()`는 유효한 실행 경로를 낸 뒤 목표 차로에 `insert_stop_point()`를 적용한다. 목표 차로가 선호 차로가 아니면 경로 끝에서 `next_dist_buffer.min`을 뺀 위치에 정지점을 넣는다. planning-aa의 조건부 승인 경로에는 이 종단 정지점이 없다. 가상 승인과 root 표식 수신 시각, 후방 7.1m를 함께 사용한 진단에서 279번 행동 경로는 독립 32점/원본 59점, 첫 정지 거리 없음/52.690m다. 320번은 32점/54점, 첫 정지 거리 없음/41.904m다. 320번 독립 경로 시작도 원본보다 약 15m 뒤에 있다. 해당 후방 길이는 기록 당시 설정으로 확인되지 않았고, 가상 승인 시작 프레임에 따라 승인 경로 자체가 달라진다. 원본 stop 수치만 강제로 넣는 방식은 경로·후속 차로 조건을 검증하지 못하므로 기본 계산에 적용하지 않았다.

## 2. 하위 모듈의 동일 입력 분리

[원본 순항 노드 대조](obstacle_cruise_slowdown_source_probe_20261011.md)에서 300~344번 원본 노드는 45/45프레임 순항하고 감속은 없었다. 독립 route 재생은 314~326번의 13프레임에서 추가 감속을 선택했다. 기록된 원본 동작 속도 **입력 경로를 조건부로** planning-aa 순항 코어에 제공하면 45/45프레임 순항 선택이 같고 원시 순항 거리의 최대 차이가 0.017m였다. 이 경로는 기본 route→최종 궤적 재생에 사용하지 않았다. 추가 감속의 우선 원인은 상류 경로·후보 선택 차이로 좁혀진다.

가상 승인만 적용한 320번 최종 계획 속도는 독립 2.361m/s, 원본 1.858m/s이며 기본 독립은 5.138m/s다. 279번은 가상 승인에서도 독립 6.115m/s, 원본 3.041m/s다. 가상 승인 전체의 속도 오차 95백분위는 2.958m/s로 기본 3.707m/s보다 작지만 중앙값은 0.337m/s로 기본 0.145m/s보다 크다. 해당 가정이 원본 동등성을 높였다고 판정할 수 없다.

추가로 모든 573프레임에 `approved=1`을 준 **순수 자동 활성화 근사**를 재생했다. 안전한 차로 변경 후보는 코어에서 계속 선별한다. 최종 속도 오차 중앙값/95백분위는 0.164/2.958m/s, 0.5m/s 이내는 370/573프레임(64.6%)이며 기본 재생은 0.145/3.707m/s, 343/573프레임(59.9%)이다. 279·320번 독립 속도는 6.115/2.361m/s로 원본 3.041/1.858m/s와 여전히 다르다. 원본 기록에는 전체 자동 모드와 개별 UUID의 수동 명령 상태가 함께 존재하므로 이 실험은 당시 상태 재현이 아니다. `approved=1`은 원본 RTC의 `safe && !requested`와 실행 중 상태 전이를 완전히 대체하지도 않는다.

## 3. 입력 수신 시각과 RTC 조건부 재생

route 재생기에 `--lane-change-approval-timeline-csv`를 추가했다. CSV 열은 오름차순 고유 `receipt_ns,approved`이며 가장 최근 이벤트의 상태를 프레임에 적용한다. 프레임별 승인 CSV와 동시에 사용할 수 없다. 현재 재생의 적용 경계는 기록된 **행동 경로 출력 수신 시각**이다. 실제 행동 경로 콜백 시작·RTC 입력 시각을 뜻하지 않는다. 170번 출력 수신 시각에 승인, 351번에 해제하는 합성 이벤트를 넣었을 때 기존 프레임별 가상 승인 재생과 573프레임의 모든 CSV 열이 같았다.

순항 콜백을 기록된 동작 속도 입력 수신 시각에 대응시킨 별도 진단은 573/573프레임 시계를 대응했고, 행동 경로 뒤 중앙 지연은 31.8ms였다. 이 시계에서 320번 최종 계획 속도는 2.351m/s로 프레임 시계의 2.361m/s와 다르고, 가상 승인 전체 95백분위 오차는 3.085m/s였다. 시계 선택은 결과에 영향을 준다. 원본의 RTC 명령 이벤트, 런타임 설정, 각 polling 입력 ID를 확보하기 전에는 정확한 원본 승인 시각이나 종단 동등성을 확정할 수 없다.

재현: `tests/planning_pipeline_rosbag_replay.py`의 `--from-route --callback-clock recorded-behavior-receipt --lane-change-approval-timeline-csv <csv> --export-series-csv <csv>`를 사용한다. 순항 시계 분리는 `--cruise-callback-clock recorded-motion-velocity-receipt`를 추가한다. root 표식 수신 시각과 7.1m 후방 길이는 `--experimental-route-sequence-clip --input-snapshot-clock root-marker-receipt --route-backward-path-length 7.1 --diagnose-frame-index 279 --diagnose-frame-index 320`으로만 진단한다.
