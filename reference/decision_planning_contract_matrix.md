# Decision Planning 노드 대응표

기준은 `../../katech-autoware/katech-autoware/docs/decision-migration/README.md`의 기본 decision launch다. 전체 ROS 이름과 remap은 같은 디렉터리의 `nodes.md`, 숨은 서비스·시간·QoS 계약은 `interfaces.md`를 따른다. 아래의 **계산 연결**은 노드의 ROS topic/service·lifecycle 구현 완료를 의미하지 않는다. Adaptive AUTOSAR 배포에서는 같은 ROS endpoint를 발행할 의무가 없으며 요청·상태·오류·시각 의미를 대상 인터페이스로 전달한다. `nodes.md`의 `/planning/` 항목 27개 중 component container 5개와 glog component 5개를 제외한 **Planning 기능 노드 17개 전부**를 이 표에 배치했다. System/진단은 별도 AA와의 계약으로 남긴다. **주차 기능은 현재 구현·검증 범위에서 제외한다.** 아래 주차 노드 행은 원본 목록의 대응을 잃지 않기 위한 표시다.

| 기준 Planning 노드 | 현재 `planning-aa` 대응 | 남은 계약·검증 |
| --- | --- | --- |
| mission_planner | `LaneletBinAdapter::routeThroughCheckpoints`, `MissionPlannerCore`, `MissionRouteMapAdapter`: 목적지 경로와 설정·해제·수정 목표·상태. 기본 도착 조건 1m·45도·최근 1초 odometry의 3차원 속도 검사를 입력 샘플 버퍼로 계산하며 지도 공통 차로의 자율주행 중 재경로 안전 길이도 판정. 기본 성공·실패의 ADAPI 코드 숫자도 맞춤. [원본 설정·해제·실패 재경로](mission_planner_lifecycle_reference_20261010.md)에서 UNSET→ROUTING→route→SET→UNSET, 실패 시 SET→REROUTING→SET·기존 route 유지 및 코드 2/3 일치 | 재경로의 원본 Lanelet2 길이·호길이 수치, 원본 stop checker의 별도 odometry callback·frame ID 검사, route marker, TF 오류 등 transport 예외 코드·발행 시각, TF 좌표 변환 |
| route_selector | `RouteSelectorCore`, `MissionRouteMapAdapter`: main/MRM 요청 전달·중단·복구, 경로와 상태 보존. [원본 노드 설정→MRM→복구](route_selector_live_reference_20261010.md)와 MRM 중 main 요청 교체와 실패 MRM·실패 main 재설정 사례에서 서비스 응답·상태 경계·복구 목표/UUID를 대조. 실패 MRM은 코드 3을 반환하고 main 경로를 유지하며, 실패 main은 코드 3 뒤 내부 보관 경로만 비운다. [planner 서비스 부재 원본 실행](route_selector_unready_reference.json)의 main/MRM 설정·MRM 해제 50001 응답과 실패 main 목표 재시도도 코어에서 확인. 세 노드의 실제 graph에서 route/state/echo 7 topic의 transient local과 경로 서비스 9개 확인. 빈 UUID는 원본과 같은 정적 MT19937 8비트 엔진으로 채운다 | 독립 ROS 서비스 서버/클라이언트의 준비 확인·오류 코드·QoS, 동시 요청 경합. 원본 client는 준비되지 않은 서비스에 `SERVICE_UNREADY=50001`을 반환하고 준비 뒤 `future.get()`으로 제한 시간 없이 응답을 기다리므로 이 정책의 실제 연동 검증이 필요 |
| goal_pose_visualizer | `MissionRouteMapAdapter::takeGoalPoseEchoes`는 발행된 route의 목표·시각·map frame을 보존한다. 별도 [`ros/goal_pose_visualizer.py`](../ros/goal_pose_visualizer.py)는 기준 route→`PoseStamped` echo와 depth 1·transient local·reliable QoS를 실제 ROS 토픽에 연결했고 늦은 구독자 수신과 노드 재시작 뒤 route 재수신을 검사했다. [원본 mission planner·selector+독립 echo 종단 실행](route_selector_independent_echo_scurve_reference.json)의 main→MRM→main route 세 건 모두 목표·header가 echo와 일치하며 실패 MRM·main 요청에는 추가 echo가 없음. 원본 goal visualizer 재실행과 세 목표 순서·graph 타입·QoS depth/durability/reliability 일치 | 독립 mission planner·selector ROS 서버 자체의 연동, 동시 요청과 재시작/지연 입력 |
| scenario_selector | `ScenarioSelector`, 지도 `scenarioPosition` | 실제 입력 freshness·TF·상태 발행 어댑터. 주차 활성은 이번 범위 제외 |
| external_velocity_limit_selector | `VelocityLimitSelector` | API 기본값·후보·해제 서비스/토픽 어댑터, 상태 재시작 |
| velocity_smoother | `JerkFiltered` 계산·상태 관리, 선택적 `Analytical` 계산 | 이전 내부 상태·동적 제한과 원본 콜백 짝, `current_max_velocity` 발행, 배포 풀이기 |
| behavior_path_planner | `BehaviorRouteAction`과 지도 경로 조립, 출발·정적 회피·차로 변경·목표·주차 일부. 동적 회피의 앞 차량·뒤 객체·원거리 이동 객체·cut-in/out 후보 선별과 제외 다각형의 좌우 최적화 경계 일부 반영. 후보 선별에는 인식 초기 자세·설정 차량 외형을 사용하고 일시 누락 시 마지막 전체 객체를 보존. 뒤쪽 제외·cut-in 거리·충돌 시작 시각은 회전된 객체 외곽의 경로 방향 최소·최대 거리로 계산하며 합성 지도 사례를 검사. 규제 객체의 측방 이동 가능 시작 경계와 진입 시점 자차 상태를 보존하고, 연속 관측 유효 대상·최종 회피 판정 대상·실제 다각형 생성 대상 목록을 구분. 예측 경로가 없는 비규제 객체의 최종 판정과 다각형 누락도 합성 검사하며 최종 판정의 횡방향 이전 상태는 다각형 생성 실패와 무관하게 보존. 굽은 경로의 다각형 변은 세계 좌표에서 단면과 교차 | 동적 회피의 최종 객체 선별·충돌 시간·측방 가능 경계의 원본 점별 대조, 원본 주행 경계 다각형 연결/겹침 처리, 다중 모듈 상태·RTC, 방향지시등·비상등·modified goal·reroute availability 출력, 활성 원본 연속 비교 |
| behavior_velocity_planner | `BehaviorVelocityMapPlanner`와 지도 플러그인 계산. `loadModule`/`unloadModule` 및 pipeline 어댑터 전달로 순서 있는 모듈 적재·해제와 상태 제거 코어 구현 | 기본 활성 10개 모듈의 남은 분기·상태, RTC·plugin **요청/응답 어댑터**, stop reason/velocity factor 출력, 원본 활성 결과 대조 |
| elastic_band_smoother | `ElasticBandPlanner`; IPC 정지선 470개 연속 재계획에서 원본과 내부·최종 경로 점 수 차이 0, 최종 위치 최대 차이 0.001708m | 종단 영속도 경계 76프레임(각 마지막 1점), 다른 기록에서 직전 내부 경로로 초기화한 뒤의 원본 연속 상태·재계획 경계 비교 [연속 감사](elastic_band_sequence_audit_20261010.md) |
| path_optimizer | `PathOptimizerCore`. IPC 정지선 전체 571프레임 중 비교 541프레임을 두 단계별 수신 시각의 odometry로 재생하면 ProxSuite 최적화 점 수·속도 차이 0, 위치 최대 차이 0.039122m | 이전 경계·QP 고정점 상태와 긴 연속 경로의 시작점·점 수·속도 차이, 다른 기록의 콜백 상태 정렬 |
| motion_velocity_planner | `MotionVelocityCore` 및 일부 플러그인 계산 | 기본 활성 3개 플러그인의 실제 지도·객체·점군 입력과 원본 활성 수치 비교. `~/service/{load_plugin,unload_plugin}` 및 `~/metrics` 출력도 미구현 |
| obstacle_cruise_planner | 정지·순항·감속 계산과 pipeline bridge. 기준 launch 정지 시간 1초·유지 속도 1.5m/s를 적용. 감속의 정밀 횡거리·연속 상태·원 궤적 교점 투영도 연결 | 기록 당시 설정·polling 입력과 현재 이미지의 분기 차이, 실제 bag의 차로 밖 정지 활성·감속 전체 경로. 이전 조건부 HPC S자 650프레임 감사에서는 기록 정지 434/독립 376, 기록 감속 42/독립 91이었다. 이번 현재 이미지 재실행의 HPC 돌발진입 감속 23프레임은 순항 수신 시계의 route 독립 재생과 활성 시각이 일치하고 시작 거리·종료 거리·목표 속도의 최대 차이는 0.099m·0.059m·0.008m/s다. [콜백 경계](obstacle_cruise_callback_boundary_20261010.md)·[감속 재실행](obstacle_cruise_slowdown_source_probe_20261011.md) |
| costmap_generator | 과거에 구현·검사한 `CostmapGeneratorCore`/지도 어댑터가 있음 | **이번 범위 제외:** 주차용 격자 생성·발행 계약은 완료 조건에 포함하지 않음 |
| freespace_planner | 과거에 구현·검사한 `FreespacePlannerCore`가 있음 | **이번 범위 제외:** 주차 경로·완료 상태·발행 계약은 완료 조건에 포함하지 않음 |
| planning_validator | `PlanningValidatorCore`, 진단 계산 | 최종 trajectory·validation status·`/diagnostics` 발행 어댑터와 실제 입력 시각 정렬 |
| planning_evaluator | `PlanningEvaluatorCore::tick`: trajectory/reference/ego/objects/수정 목표와 이전 trajectory 상태를 사용한 기본 선택 지표·kinematic scalar 계산. `PlanningEvaluatorMapAdapter`가 route/map에서 자차 lane ID·호길이·측방 거리 공급. 실제 K-City 지도·route·odometry를 현재 이미지에 단독 재발행한 차로 ID 29·s 5.662072m·t 0.146032m가 독립 어댑터와 일치. IPC 실측 12종 중 10종은 571/573, 편차 2종은 570/573프레임 1e-6 이내 | 원본 10Hz polling·stamped debug 발행, 안정성 지표별 재표본화/투영과 갓길 포함 lane 선택 검증. [현재 이미지 실제 graph·합성 활성](planning_evaluator_rosbag_20261010.md)도 `MetricArray`·약 10Hz·VOLATILE/RELIABLE이며 직선 3점의 기본 길이·속도와 정차 연속 27→39개 지표가 코어와 일치하고, 보유 실행 기록은 `DiagnosticArray`이므로 버전별 ROS 어댑터가 필요. launch의 `output_metrics=false`는 종료 시 JSON 출력을 끄며 runtime 지표 발행 자체를 끄지 않음 |
| remaining_distance_time_calculator | `RemainingDistanceTimeCore`, `RemainingDistanceMapAdapter`: 시나리오·속도 제한 상태와 지도 경로 호길이. IPC 정지선 571프레임·HPC S자 755프레임의 원본 활성 출력과 거리 중앙 차이 0m, 각각 569·754프레임이 1cm 이내. [원본 parking 발행 시험](remaining_distance_rosbag_20261010.md)에서 입력 게이트·영값·약 10Hz 및 출력 타입·VOLATILE/RELIABLE 확인 | 일부 odometry callback 짝과 Lanelet2 최단 경로·투영 경계의 추가 수치 비교, 10Hz 발행·map/route/ego/scenario/current limit 입력 어댑터 |

## 우선순위 2 서비스·출력 계약

아래 경로는 기준 `nodes.md`와 `interfaces.md`의 해석된 이름이다. “계산”은 ROS endpoint를 생성했다는 뜻이 아니다.

| 기준 endpoint 또는 출력 | 코어 대응 | 남은 구현 |
| --- | --- | --- |
| `/planning/mission_planning/mission_planner/{clear_route,set_waypoint_route,set_lanelet_route}` | `MissionPlannerCore::handle`의 clear/waypoint/lanelet 요청·응답 | service server, 오류 코드·입력 좌표 변환·발행 시각 |
| `/planning/mission_planning/{route,state}` | `MissionPlannerCore::takeEvents`의 route/state 발행 순서 | transient local topic, header·map marker·도착 기본 검사. 내부 `state`는 원본 소스와 기록에서 `tier4_planning_msgs/msg/RouteState`; `interfaces.md`의 ADAPI 형식 표기는 이 내부 토픽에 맞지 않음 |
| `/planning/mission_planning/route_selector/main/{clear_route,set_waypoint_route,set_lanelet_route}` | `RouteSelectorCore::handle(Main, ...)` | 세 service server, planner client의 비동기 순서 |
| `/planning/mission_planning/route_selector/mrm/{clear_route,set_waypoint_route,set_lanelet_route}` | `RouteSelectorCore::handle(Mrm, ...)`와 main 복구. planner 부재의 원본 응답은 50001이며 main 실패 요청 뒤 clear는 저장된 목표를 다시 waypoint로 보낸다 | 세 service server, MRM 운영 중 요청 경합과 준비 완료 뒤 제한 시간 없는 planner 응답 대기 |
| `/planning/mission_planning/route_selector/{main,mrm}/{route,state}` | `RouteSelectorCore::takeEvents` | 채널별 transient local topic, 원본 stamp·UUID 정책 검증 |
| `/planning/mission_planning/echo_back_goal_pose` | `MissionRouteMapAdapter::takeGoalPoseEchoes`와 [`goal_pose_visualizer.py`](../ros/goal_pose_visualizer.py)의 `PoseStamped` transient local publisher. ROS smoke에서 route header·goal pose 값과 늦은 구독자 및 노드 재시작 뒤 수신 확인. 원본 planner/selector를 입력으로 쓴 main→MRM→main 세 route의 목표·header도 일치 | 독립 route selector/mission planner의 ROS publisher와 연결한 종단 발행 검사 |
| `parking/costmap_generator/{grid_map,occupancy_grid}` | 과거 계산 코어만 보관 | **이번 범위 제외** |
| `parking/freespace_planner/{trajectory,is_completed}` | 과거 계산 코어만 보관 | **이번 범위 제외** |
| `planning_evaluator/metrics`, `~/debug/processing_time_ms` | `PlanningEvaluatorCore::tick`의 지표 통계와 route/map 기반 차로 지표. 실측 12종 중 10종은 571/573, 편차 2종은 570/573프레임 일치 | 현재 소스 `MetricArray`와 실측 기록 `DiagnosticArray`의 버전별 10Hz publisher·처리 시간, 나머지 지표 수치 비교 |
| `/planning/mission_remaining_distance_time` | `RemainingDistanceMapAdapter`의 거리·시간 출력 | 10Hz 메시지 publisher, 원본 경로/속도·시각 대조 |
| `behavior_velocity_planner/service/{load_plugin,unload_plugin}` | `BehaviorVelocityMapPlanner::{loadModule,unloadModule}` 및 pipeline 전달. 원본 `BehaviorVelocityPlannerManager`의 중복 적재 거부·적재 순서·제거 시 상태 삭제를 코어에서 반영. 단일 모듈 plugin 13개의 이름 변환과 `IntersectionModulePlugin`의 stuck/collision 모듈 묶음 적재·해제를 pipeline 어댑터에 추가 | 실행 앱 요청·응답 어댑터, 동시 호출과 오류 정책, 교차로 복합 plugin의 실제 계산·상태 동등성 검증. 원본 `node.cpp`의 load 요청은 plugin XML의 **전체 class 이름**, unload 요청은 `getModuleName()`의 **짧은 이름**을 받는다. 원본 `String` 서비스 응답은 이 호출부에서 성공·실패 결과를 채우지 않으므로 Adaptive 응답 오류 정책은 별도로 정의해야 한다. |
| `motion_velocity_planner/service/{load_plugin,unload_plugin}` | 기본 launch의 `OutOfLaneModule`, `ObstacleVelocityLimiterModule`, `DynamicObstacleStopModule`을 원본 순서로 결과 적용하고 전체 class 이름으로 적재·해제할 수 있다. 재적재한 모듈은 목록 끝에서 실행한다. `PlanningOutOfLaneInput`이 별도 콜백의 객체·적색 신호와 최소 제동 호길이 또는 콜백 속도·가속도·평활화 제한을 제공하면 지도 후보·정지/감속 상태를 종단에 연결한다. 도착시각 배열을 생략하면 원본의 0.5m 재표본화와 1m/s 속도 하한을 적용한다 | 대상 실행 앱의 요청·응답 연결, 실제 콜백별 입력·런타임 평활화 제한 공급, 원본 연속 상태 대조. 원본 `autoware_motion_velocity_planner_node_universe/srv/{LoadPlugin,UnloadPlugin}`은 `success` 필드를 가지지만 현재 `node.cpp`의 콜백은 이 필드에 값을 대입하지 않는다. Adaptive 요청·응답은 명시적인 결과 정책이 필요하다. |
| `motion_velocity_planner/metrics` | 동작 속도 플러그인의 계산 결과 일부만 보유 | `MetricArray` publisher, 플러그인별 지표 집계·header·발행 시각 미구현 |

주요 Planning 데이터 출력은 다음과 같이 추적한다. route/state, goal echo, parking 격자, 주차 trajectory/completed, evaluator metrics, remaining distance/time 외에 `scenario`, `max_velocity`, `current_max_velocity`, 행동 경로·속도·평활화·최적화·동작 속도·순항·validator의 각 중간 경로와 최종 trajectory가 있다. 계산 코어가 있는 항목도 발행 순서·지속성·헤더가 빠진 경우는 미완료로 표시한다. `turn_indicators_cmd`, `hazard_lights_cmd`, `modified_goal`, `is_reroute_available`, stop reasons/velocity factors, validation status, RTC와 plugin 서비스도 노드 표의 미완료 계약을 따른다.

### 기본 활성 Planning 출력 전수 대조

약어는 `P=/planning/scenario_planning`, `B=P/lane_driving/behavior_planning`, `M=P/lane_driving/motion_planning`이다. `nodes.md`의 remap과 `interfaces.md`의 숨은 endpoint를 함께 확인했다. `~/` 이름은 해당 노드의 namespace 안에 있다.

| 기준 생산 노드 | 기본 연결·외부 출력 | 추가 발행 또는 서비스 | 현재 판정 |
| --- | --- | --- | --- |
| mission_planner | `/planning/mission_planning/{route,state}` | `/planning/mission_planning/route_marker`, `~/debug/goal_footprint`, `~/debug/processing_time_ms`; 위 3개 서비스 | 경로·상태 코어와 `PlanningRouteCoordinator`의 이벤트 전달, marker·실행 앱 출력 미완료 |
| route_selector | `/planning/mission_planning/route_selector/{main,mrm}/{route,state}` | main/MRM 각각 3개 서비스, mission planner 서비스 client, `~/debug/processing_time_ms` | 선택·복구 코어와 coordinator의 요청 전달, 프로세스 간 요청·출력 어댑터 미완료 |
| goal_pose_visualizer | `/planning/mission_planning/echo_back_goal_pose` | 없음 | `PoseStamped` 발행·QoS·늦은 구독자·노드 재시작 뒤 수신 및 원본 route 생산자와 종단 연결 확인; 독립 route 생산자 연동 미완료 |
| scenario_selector | `ScenarioSelector`, 지도 `scenarioPosition` | 실제 입력 freshness·TF·상태 발행 어댑터. 주차 활성은 이번 범위 제외 |
| external_velocity_limit_selector | `P/max_velocity` | `P/external_velocity_limit_selector/debug` | 선택 코어, debug·API 발행 미완료 |
| velocity_smoother | `P/velocity_smoother/trajectory`, `P/current_max_velocity` | `P/velocity_smoother/distance_to_stopline`, virtual wall, closest velocity/acceleration/jerk/max/merged velocity, 조건부 debug 경로·processing time/detail | 평활화 코어, current limit와 거리·debug 발행 미완료 |
| behavior_path_planner | `B/path_with_lane_id`, `/planning/{turn_indicators_cmd,hazard_lights_cmd}`, `P/modified_goal` | `P/status/stop_reasons`, `B/behavior_path_planner/output/is_reroute_available`, avoidance·turn signal·bound debug, 모듈별 동적 candidate/reference 경로, RTC 승인/상태 | 경로 일부 계산, 부가 출력·RTC 미완료 |
| behavior_velocity_planner | `B/path`, `P/status/{stop_reasons,infrastructure_commands}`, `B/debug/traffic_signal` | velocity factor, RTC, `~/service/{load_plugin,unload_plugin}` | 경로 일부와 plugin 적재·해제 코어 계산, 부가 출력·요청/응답 어댑터 미완료 |
| elastic_band_smoother | `M/path_smoother/path` | `~/output/traj`, debug·processing time | 계산 코어, 연속 상태 차이·발행 미완료 |
| path_optimizer | `M/path_optimizer/trajectory` | debug·processing time | 계산 코어, 연속 상태 차이·발행 미완료 |
| motion_velocity_planner | `M/motion_velocity_planner/trajectory`, `P/status/stop_reasons`, `/planning/velocity_factors/motion_velocity_planner` | `M/motion_velocity_planner/service/{load_plugin,unload_plugin}`, `M/motion_velocity_planner/metrics`, plugin/RTC debug·processing time | 궤적 일부 계산, plugin 서비스·metrics·상태 출력 미완료 |
| obstacle_cruise_planner | `P/lane_driving/trajectory`, `P/status/stop_reasons`, `P/{max_velocity_candidates,clear_velocity_limit}` | `~/output/stop_speed_exceeded`, `~/metrics`, debug 정보·표식·가상벽·processing time. `~/optimized_sv_trajectory`, `~/optimized_st_graph`, `~/boundary`는 기본 `pid_base`에서 만들지 않는 `optimization_base` 전용 | 정지·순항·감속 코어, 활성 상태·발행 미완료 |
| costmap_generator | 과거에 구현·검사한 `CostmapGeneratorCore`/지도 어댑터가 있음 | **이번 범위 제외:** 주차용 격자 생성·발행 계약은 완료 조건에 포함하지 않음 |
| freespace_planner | 과거에 구현·검사한 `FreespacePlannerCore`가 있음 | **이번 범위 제외:** 주차 경로·완료 상태·발행 계약은 완료 조건에 포함하지 않음 |
| planning_validator | `P/trajectory`, `/planning/planning_validator/validation_status` | `/diagnostics`, debug·processing time | 검증 코어, status·diagnostics 발행 미완료 |
| planning_evaluator | `/planning/planning_evaluator/metrics` | `~/debug/processing_time_ms` | 12종 실측 수치 비교 및 현재 이미지 출력 타입·QoS 확인; 버전별 독립 발행·다른 지표 비교 미완료 |
| remaining_distance_time_calculator | `/planning/mission_remaining_distance_time` | 없음 | 거리·시간 코어와 실측 원본 수치 비교; 독립 ROS 10Hz 발행·QoS 연결 미완료 |

출력 17개 노드와 route 서비스 생산자 2개 노드(9개 서비스), 행동 속도 plugin 서비스 2개 및 동작 속도 plugin 서비스 2개를 이 표에 배치했다. [출력·서비스 생성 코드 목록](planning_endpoint_source_inventory_20261010.md)은 원본 `source-api-evidence.json`의 이 17개 노드에서 기본 launch에 해당하는 publisher 생성 코드 99개와 service 생성 코드 13개를 파일·줄별로 열거한다. 처음의 104개에는 별도 `invalid_trajectory_publisher` 노드 1개와 기본 `pid_base`에서 사용하지 않는 `optimization_base` 출력 4개가 섞여 있어 활성 건수에서 제외했다. [입력 생성 코드 목록](planning_input_source_inventory_20261010.md)은 같은 기준의 구독 생성 코드 84개와 planner service client 3개를 polling/callback별로 기록한다. plotter/visualizer·다른 evaluator 노드·선언만 있는 항목도 제외했다. 이 대조에서 mission goal footprint, obstacle cruise의 `stop_speed_exceeded`·`metrics`, costmap `processing_time` 등 요약표에서 빠졌던 계약을 보완했다. 동작 속도 서비스와 metrics는 `nodes.md`의 remap 표에 없지만 원본 `autoware_motion_velocity_planner_node_universe/src/node.cpp`의 생성 코드에서 확인했다. 별도로 [K-City 출력 관측표](planning_output_bag_inventory_20261010.md)에 10개 원본 기록의 주요 기본 출력 38개별 실제 메시지 수·ROS 형식·publisher QoS를 집계했다. 보유 기록에는 MRM route, 주차 격자·trajectory, modified goal, 동작 속도 metrics 메시지가 없지만 MRM state와 주차 완료 상태는 발행됐다. 이 부재를 해당 노드나 기능이 비활성이라는 결론으로 사용하지 않는다. evaluator 지표는 기록 `DiagnosticArray`/현재 소스 `MetricArray`의 형식 차이가 실제로 확인됐다. route state의 내부 형식은 소스·기록 모두 `tier4_planning_msgs/msg/RouteState`이며 `interfaces.md`의 ADAPI 형식 표기와 다르다. debug/RTC/plugin endpoint의 동적 이름과 실제 생성 여부는 `interfaces.md` 자체도 runtime 확인 대상으로 남긴다. 서비스 호출은 rosbag topic만으로 확인할 수 없고 실제 DDS graph 전체도 아직 확인하지 못했다.

## 공통 외부 계약

- 입력 최소 묶음: 지도, 경로와 상태, 자차 자세·속도·가속도, 예측 객체, 점유 격자·점군, 신호, operation mode, 속도 제한, 차량 제원, 명시적 시계. 각 묶음에 좌표계·단위·stamp·유효기간·지도/경로 버전을 포함한다.
- 출력은 최종 trajectory 외에 route/state, 방향지시등·비상등, 수정 목표, 재계획 가능 상태, 속도 제한·해제, RTC·서비스 응답, 진단·유효성 상태와 remaining distance/time이 있다. 실제 배포 브리지의 스키마는 아직 정하지 않았다.
- route/map과 일부 상태는 transient local이며 polling subscriber는 최신값 또는 전체 수신 정책을 사용한다. 단순 함수 호출 순서만으로 원본 콜백·발행 순서를 대체하지 않는다.
- 기준 이미지에 설치됐거나 코드가 있는 비활성 플러그인도 있다. 기본 대체의 완료 판정은 기본 launch의 활성 구성부터 한다.
- 주차용 **점유 격자 입력**(`freespace_planner` 및 행동 경로)과 costmap generator의 **점유 격자 생성**은 별도 계약이다. 생성 코어의 수치 동등성과 격자/상태 발행은 아직 완료되지 않았다.

## 검증 단계

1. 노드별 원본 입력·출력 topic/service와 QoS를 기준 실행 graph에서 확인한다. 정적 remap·source 검색만으로 실제 endpoint를 확정하지 않는다.
2. 계산 단위는 원본 출력과 상태·점 수·위치·속도·시각을 비교한다. 합성 활성 사례와 보유 rosbag의 비활성 사례를 분리한다.
3. 동일 입력 시계로 route → 행동 경로 → 행동 속도 → 평활화 → 최적화 → 동작 속도 → 장애물 순항 → 시나리오 선택 → 속도 평활화 → validator를 독립 연속 실행한다. 원본 중간 출력으로 재시작한 검사와 구분한다.
4. Planning과 Safety/Status AA·MRM·control partition 사이의 상태, timeout, 서비스 응답과 비상/가용성 출력을 통합 시험한다.
