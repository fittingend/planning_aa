# 장애물 감속의 원본 노드 대조와 종단 재생

2026-10-11 기준 `ObstacleCruisePlannerNode::createSlowDownObstacleForPredictedObject`와 `PlannerInterface::insertSlowDownVelocity`를 현재 decision 이미지에서 실행해 planning-aa의 독립 계산과 비교했다. `reference/obstacle_cruise_reference_probe.py`는 보유 IPC 보행자 bag의 시계·메시지 틀에 합성 객체와 경로를 넣어 원본 노드를 연속 구동한다. 합성 결과는 해당 분기의 코드 대조 근거이며, 기록 당시 실행 설정의 복원 증거는 아니다.

| 합성 입력 | 원본 첫 활성 | planning-aa 첫 활성 | 원본 첫 활성 감속 시작/종료 | planning-aa | 원본 목표 속도 | planning-aa |
| --- | --- | --- | --- | --- | ---: | ---: |
| 직선·정지 보행자 | 244 | 244 | 0.770/11.510m | 0.770/11.510m | 6.760m/s | 6.760m/s |
| 직선·0.6m/s 보행자 | 244 | 244 | 2.588/12.398m | 2.588/12.398m | 6.140m/s | 6.140m/s |
| 곡선·0.6m/s 보행자 | 244 | 244 | 4.005/12.831m | 4.005/12.831m | 5.416m/s | 5.416m/s |
| 직선·5프레임 뒤 횡거리 1.232m | 244–248 | 244–248 | 시작 244/245/246번: 0.770/0.769/0.211m | 동일 | 6.760→6.760m/s | 동일 |

첫 세 입력은 각각 `obstacle_cruise_synthetic_slowdown_source.json`, `obstacle_cruise_synthetic_slowdown_moving_source.json`, `obstacle_cruise_synthetic_slowdown_curve_moving_source.json`에 원본 디버그를 저장했다. 이탈 입력은 `obstacle_cruise_synthetic_slowdown_exit_source.json`이다. planning-aa 값은 `planning_pipeline_cruise_bridge_test`의 원본 수치 회귀 검사로 확인했다. 원본 디버그의 음수 감속 시작 거리와 경로 적용 뒤 0m로 잘린 출력은 서로 다른 값이다.

코드 변경은 감속 대상의 정밀 횡거리·5프레임 진입/이탈 상태를 원본 조건에 맞추고, 충돌 영역에 횡 1.3m를 적용한 것이다. 이동 여부에는 객체의 지도 `y` 속도 대신 경로 투영 횡속도를 사용한다. 곡선에서는 듬성듬성 표본화한 충돌 경로에서 교점을 고른 다음 **원래 궤적에 교점 좌표를 다시 투영**해 감속 호길이를 계산한다. 차량 앞 오프셋은 설정된 차량 외형에서 가져온다.

## 기록된 출력과 현재 원본의 차이

원본 bag의 감속 디버그는 HPC 끼어들기 10프레임, HPC 돌발진입 1프레임, HPC S자 42프레임에서 활성이다. 반면 같은 입력을 현재 decision 이미지의 원본 노드에 재주입하면 끼어들기 495–524번과 S자 520–579번 구간에서 감속은 0프레임이고, 각 구간에서 순항 객체가 13프레임·43프레임 선택된다. 돌발진입 225–264번 구간의 현재 원본 감속은 235–257번 23프레임이며, 기록의 235번 한 프레임과 다르다. 현재 이미지의 파라미터·실행 상태·polling 콜백 입력을 기록 당시와 동일하게 복원했다는 근거가 없어, 이 차이를 planning-aa 코드 오류 하나로 귀속할 수 없다.

기본 route 독립 종단 재생에 감속 후보를 선택적으로 켠 10개 bag, 총 6,085프레임에서 최종 궤적 생성은 모두 유지됐다. 감속 후보의 활성은 돌발진입 236–258번 23프레임, 끼어들기 0프레임, S자 0프레임이었다. 그 외 HPC 차단에서 13프레임이 활성이고 나머지 여섯 bag은 0프레임이다. 기록 디버그와 동시 활성은 0프레임이다. 돌발진입의 차량 위치 계획 속도 절대 오차 95백분위는 감속 후보 제외 1.220m/s에서 포함 0.693m/s로 줄었다. 이 값은 전체 경로의 종단 차이를 나타내며, 감속 분기 자체의 bag 동등성으로 읽으면 안 된다.

종단 재생의 감속 진단 프레임별 CSV는 [압축 보관 파일](../../251231_KCity_ori/planning_aa_result_diagnostics_20261011.tar.gz)에 있고, 전체 표는 `251231_KCity_ori/planning_aa_scenario_comparison_summary_slowdown_diagnostic.csv`다. CSV의 `source_slowdown_object_count`는 기록된 원본 디버그를 순항 입력 수신 시각에 대응한 값이고, `independent_slowdown_candidate_count`는 계획 경로의 독립 후보 수다. 입력 궤적과 이전 상태가 달라 둘의 차이를 곧바로 계산 오류로 단정하지 않는다.

현재 원본 돌발진입 활성 235–257번과 planning-aa의 기본 프레임 시계 236–258번은 한 프레임 차이다. 원본 순항 입력 궤적은 route 독립 재생의 프레임 시계보다 약 40ms 뒤에 수신된다. 기록된 `motion_velocity_planner/trajectory`의 **수신 시각만** 순항 콜백 시계로 쓰고 odometry·객체·가속도를 다시 짝지으면 planning-aa 활성도 235–257번으로 맞는다. 원본 중간 궤적의 점·속도를 계산 입력으로 재사용하지 않았다. 이 시계 진단에서 돌발진입의 최종 계획 속도 절대 오차 95백분위는 0.654m/s다. 40프레임 돌발진입, 30프레임 끼어들기, 60프레임 S자 범위에서 현재 원본 노드와 planning-aa의 감속 활성 여부는 각각 40/40, 30/30, 60/60프레임 같다. [활성 비교 그림](obstacle_cruise_slowdown_source_activation_20261011.png)과 [프레임 표](obstacle_cruise_slowdown_source_activation_20261011.csv)에 기록했다.

돌발진입 활성 23프레임의 첫 감속 구간을 추가 대조했다. 현재 원본 디버그 대비 planning-aa의 시작 거리 절대 차이 중앙값/최대값은 **0.023/0.099m**, 종료 거리는 **0.011/0.059m**, 안정화 목표 속도는 **0.0065/0.0077m/s**다. 독립 route→동작 속도 경로를 사용한 값이라 경로 형상과 차량 상태의 작은 차이도 포함된다. [구간·속도 그래프](obstacle_cruise_runout_intervals_20261011.png)와 [23프레임 수치](obstacle_cruise_runout_intervals_20261011.csv)는 `compare_obstacle_cruise_runout_intervals.py`로 다시 만들 수 있다. 이 비교는 현재 원본 노드의 재실행 결과에 대한 것이며, 보유 bag의 기록 당시 감속 1프레임과 같은 결과라고 뜻하지 않는다.

구간 수치의 planning-aa 입력 파일은 [HPC runout 감속 구간 CSV](data/planning_aa_vs_autoware_runout_hpc_cruise_input_clock_diagnostic_slowdown_interval.csv)다. `tests/planning_pipeline_rosbag_replay.py`의 `--from-route --with-cruise-candidates --with-cruise-slowdown --cruise-callback-clock recorded-motion-velocity-receipt --export-series-csv`로 생성했다. 원본 파일은 `obstacle_cruise_runout_slowdown_225_265_source.json`이다.

HPC 차단 300–344번에서는 현재 원본 노드가 **45/45프레임 순항**하고 감속은 없지만, 독립 route 재생은 314–326번 **13프레임 감속**을 추가로 선택한다. 저장된 원본 동작 속도 입력 궤적을 **조건부**로 후보 코어에 넣으면 순항은 현재 원본과 45/45프레임 같고 순항 원시 거리의 최대 차이는 0.017m다. 이 궤적은 독립 종단 재생에는 사용하지 않았다. 따라서 이 구간의 추가 감속은 감속 상태 조건을 bag에 맞춰 바꿀 근거가 아니라, 앞선 독립 경로·객체 선택 경계에서 순항 후보가 빠지는 문제의 결과다. 320번 프레임의 첫 차이는 행동 경로다. 독립 경로는 차로 36 끝에서 13점으로 끝나고 첫 정지점이 차량 13.31m 앞이지만, 원본은 차로 15850·15910까지 54점으로 이어져 첫 정지점이 41.90m 앞이다. [활성 그림](obstacle_cruise_slowdown_source_activation_20261011.png)은 추가 감속 13프레임과 기록/현재 원본의 차이를 함께 표시한다. 다음 수정 대상은 이 차로 변경의 외부 승인 시각과 행동 경로 길이·동작 속도 입력 형상이다.

기록된 `/planning/auto_mode_status/lane_change_left`는 585/585개가 `is_auto_mode=true`다. 따라서 차로 변경 RTC 인터페이스는 기록 내내 자동 모드로 설정돼 있었다. 동시에 `/planning/cooperate_status/lane_change_left`의 개별 상태 575개는 모두 `auto_mode=false`이고, 314·320·326번에 대응하는 상태에는 `safe=true`, `command_activated=true`인 항목이 있다. 원본 `RTCInterface::updateCooperateCommandStatus()`는 개별 협력 명령을 받으면 해당 UUID의 `auto_mode`만 false로 바꾸며, 인터페이스 전체의 `is_auto_mode_enabled_`는 유지한다. 따라서 자동 모드 설정과 개별 차로 변경의 수동 명령 상태는 양립한다. 출력 수신 시각을 행동 경로 프레임에 대응하면 개별 활성 상태가 245–408번에도 이어진다. 이는 **원본 모듈의 출력 상태**이며 독립 재생의 승인 입력으로 되먹이지 않았다. 현재 `sample_adsw_decision_run.launch.xml`의 `enable_all_modules_auto_mode=true` 설정과도 모순되지 않는다. 실제 협력 명령의 입력·도착 시각은 bag에 없어 재생 시작 시점은 확정할 수 없다.

`tests/behavior_lane_change_rtc_rosbag_replay.py`를 해당 HPC 차단 bag에 직접 실행해 RTC 상태 배열 1,167개와 그중 상태 575개의 독립 신호 조립이 원본 규칙과 불일치 0개임을 확인했다. 이 검사는 저장된 **상태 출력의 해석**을 검증하며, 저장되지 않은 승인 요청 입력 시각을 복원하지 않는다.

최종 수정 뒤 전체 C++ 빌드와 등록된 CTest 238개가 모두 통과했다. rosbag 종단 재생·현재 원본 노드 재실행·합성 원본 대조의 입력 범위와 결론은 각각 위에 구분해 기록했다.
