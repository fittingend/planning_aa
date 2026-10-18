# PlanningAA–perception/control 인터페이스 구현 목록

## 대조 범위

- 계약: `OSS_KATECH_PlanningInterface_260713_updated.xlsx`의 `Decision <-> 외부 IF`(행 5–30), `AAP_IF`, `ServiceInterfaces모음`, `Port&ID`.
- 실행 코드: 이 저장소의 `adaptive/app/main.cpp`, `adaptive/app/trajectory_output.cpp`, 매니페스트와 형제 저장소 `../oss-ap-planning-bridge/src/Ecu_Machine/Exe_ServiceCreator/src/{servicecreator_node.cpp,ap_channel_para.cpp}` 및 서비스 매니페스트. **구현됨**은 정적 코드 배선을 뜻하며 실제 ROS 2↔SOME/IP 종단 성공을 뜻하지 않는다.
- 대체 대상: `../katech-autoware/katech-autoware`의 `sample_adsw_decision_run.launch.xml`이 띄우던 **decision 컨테이너 전체**. `adaptive/run-katech-stack.sh`는 `sample_adsw_perception_run.launch.xml`과 `sample_adsw_control_run.launch.xml`을 사용한다. 로컬 이미지 `adsw-perception`(sha256 `c16e36ef…`)·`adsw-control`(sha256 `aa60e82a…`) 안의 `adsw_*.launch.xml`도 확인했다. 2026-10-16 확인 시 이 두 컨테이너는 실행 중이지 않아 live ROS graph는 검사하지 못했다.
- 경계: perception/control과 교환하는 항목을 launch·소비 노드까지 추적한다. `AAP_IF`의 planning 내부 노드 간 토픽, 디버그/parameter 서비스 전체를 브릿지 외부 계약으로 간주하지 않는다. 다만 원래 decision의 system/MRM 기능은 컨테이너 대체 범위에 포함한다.

## 3분할 컨테이너 기준 결론

**현재 구현은 완전한 decision 컨테이너 대체가 아니다.** perception의 주요 지도·위치·인식 입력과 control의 최종 궤적·main route·route 요청은 코드상 연결돼 있다. 하지만 control의 vehicle command gate가 구독하는 방향지시등·비상등, control AD API가 구독하는 운행 모드 가용성, 원래 decision의 MRM/system 출력은 없다. Event 12401은 정의서에는 있지만 control 기본 launch의 소비 경로에는 없다.

근거가 되는 실제 launch 경로는 `obigo_launch/launch/{sample_adsw_perception_run,sample_adsw_control_run,adsw_decision}.launch.xml`, `tier4_control_launch/launch/control.launch.xml`, `autoware_default_adapi/src/{routing,operation_mode,planning}.cpp`이다. `adsw_decision.launch.xml`은 planning과 system을 모두 include하지만 `adsw_control.launch.xml`은 control·vehicle·API만 include한다. `launch_system=true` 인자가 control에 선언되어 있는 것만으로 system 노드가 실행되지는 않는다.

| 실제 상대 노드/기능 | decision 대체 경계 | 현재 판정 |
| --- | --- | --- |
| perception map/localization/object/pointcloud/grid/traffic signals, simulation `/clock` | 브릿지 ROS 구독 → 4308 → PlanningAA | 11개 계약 중 해당 입력의 정적 배선 있음. 점군·격자는 계획 계산 미연결; 실제 발행·수신은 미검증 |
| control `trajectory_follower`, `operation_mode_transition_manager`, `control_validator`, 기본 활성 `lane_departure_checker` | `/planning/scenario_planning/trajectory` | 12506 → 브릿지 발행 구현. `lane_departure_checker`는 route도 요구 |
| control `lane_departure_checker`, AD API `RoutingNode` | `/planning/mission_planning/route`, `/planning/mission_planning/route_selector/main/{route,state}` 및 main route 서비스 3개 | 10602/10604/10801/10809/10812 배선 구현. 실제 수신·route 의미 검증 필요 |
| control `vehicle_cmd_gate` | `/planning/turn_indicators_cmd`, `/planning/hazard_lights_cmd` | **미구현**. 원래 behavior path planner 출력이며 260713 XLSX 최소 외부 계약에 없음 |
| control AD API `OperationModeNode` | `/system/operation_mode/availability` (`tier4_system_msgs/msg/OperationModeAvailability`) | **미구현**. 원래 decision의 diagnostic graph 쪽 출력. AD API는 autonomous availability를 기본 `false`로 시작함 |
| control `vehicle_cmd_gate`의 비상 입력 | `/system/emergency/{control_cmd,hazard_lights_cmd,gear_cmd}`, `/system/fail_safe/mrm_state` | **미구현**. 원래 decision의 MRM handler/operator 소유; 정상 궤적 배선과 별도 기능 |
| control AD API `PlanningNode` | `/planning/scenario_planning/trajectory`, `/planning/planning_factors/*` | 궤적은 구현. planning factors는 미구현이므로 API의 속도·조향 요인 결과가 비어 있을 수 있음 |

## 현재 배선

| 방향 | SOME/IP 계약 | ROS 2 대응 | 현재 상태 |
| --- | --- | --- | --- |
| perception/clock/control → PlanningAA | Service 4308, Event 1 `clock`, 4001 `localization_acceleration`, 4002 `localization_kinematicstate`, 6001 `map_vectormap` | `/clock`, `/localization/acceleration`, `/localization/kinematic_state`, `/map/vector_map` | 브릿지 ROS 구독·ARA 송신, AA 구독 구현 |
| perception → PlanningAA | 4308, Event 30101 `perception_objectrecognition_objects`, 30201 `perception_obstaclesegmentation_pointcloud`, 30301 `perception_occupancygridmap_map`, 30401 `perception_trafficlightrecognition_trafficsignals` | `/perception/object_recognition/objects`, `/perception/obstacle_segmentation/pointcloud`, `/perception/occupancy_grid_map/map`, `/perception/traffic_light_recognition/traffic_signals` | 통신 배선 구현. 점군·점유 격자는 AA가 샘플만 보관 |
| 외부 planning/control → PlanningAA | 4308, Event 11629 `...input_lateraloffset`, 12503 `...maxvelocitydefault`, 6101 `system_operationmode_state` | `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/lateral_offset`, `/planning/scenario_planning/max_velocity_default`, `/system/operation_mode/state` | 통신 배선 구현. lateral offset은 AA가 샘플만 보관. 첫 두 토픽의 발행자는 실행 구성에서 별도 확인 필요 |
| 외부 route 요청 → PlanningAA | Service 1817, Method 10801/10809/10812 (`clearroute`/`setlaneletroute`/`setwaypointroute`) | `/planning/mission_planning/route_selector/main/{clear_route,set_lanelet_route,set_waypoint_route}` 및 `mission_planner/*` 별칭 | 브릿지 ROS 서비스·ARA 호출, AA 응답 구현 |
| PlanningAA → control | Service 2317, Event 12506 `planning_scenarioplanning_trajectory` | `/planning/scenario_planning/trajectory` (`autoware_planning_msgs/msg/Trajectory`) | 검증된 최종 궤적의 ARA 송신·브릿지 ROS 발행 구현 |
| PlanningAA → route 소비자 | Service 1814, Event 10604 `planning_missionplanning_state` | `/planning/mission_planning/route_selector/main/state` (`tier4_planning_msgs/msg/RouteState`) | ARA 송신·브릿지 ROS 발행 구현 |
| PlanningAA → route 소비자 | Service 1814, Event 10602 `planning_missionplanning_route` | `/planning/mission_planning/route`, `/planning/mission_planning/route_selector/main/route` (`autoware_planning_msgs/msg/LaneletRoute`) | ARA 송신·브릿지 ROS 발행 코드 구현. 종단 검증 필요 |

근거: 브릿지 `servicecreator_node.cpp`의 `setupInputEvents`, `setupInputMethods`, `setupOutputEvents`; `ap_channel_para.cpp`의 `SERVICECREATOR_FOR_EACH_INPUT_EVENT` 및 출력 proxy; PlanningAA `adaptive/app/main.cpp`의 `InputService`, `RouteService`, `RouteStatePublisher`, 최종 궤적 `Send()`.

## 추가 구현 필요

### 1. Control이 직접 소비하는 출력

1. **방향지시등·비상등:** `tier4_control_launch/launch/control.launch.xml`의 `vehicle_cmd_gate` remap 69–70행에 맞춰 `autoware_vehicle_msgs/msg/TurnIndicatorsCommand`를 `/planning/turn_indicators_cmd`, `autoware_vehicle_msgs/msg/HazardLightsCommand`를 `/planning/hazard_lights_cmd`로 출력한다. 현재 PlanningAA/브릿지에 두 ROS publisher와 대응 SOME/IP 포트·변환이 없다. 경로/교차로 상태에서 명령을 산출하는 원래 behavior path planner 정책과 발행 주기를 확인한 뒤 구현한다. 260713 XLSX에 ID가 없으므로 service/event ID를 임의로 재사용하지 말고 계약을 새로 확정한다.
2. **운행 모드 가용성:** `/system/operation_mode/availability`의 생산 주체를 대체 구성에서 마련한다. 원래 decision의 `tier4_system_component` → diagnostic graph aggregator가 생산했고 control AD API `OperationModeNode`가 구독한다. 진단 평가 없이 항상 `true`를 발행하는 임시 구현으로 정상 운행을 판정해서는 안 된다. 누락 시 AD API의 autonomous 전환 요청은 기본 가용성 `false`에 막힐 수 있다.
3. **MRM/system:** 원래 decision의 `obigo_system.launch.xml`은 MRM handler, comfortable/emergency stop operator, system diagnostic monitor를 실행한다. 이들이 control gate에 제공하던 `/system/emergency/*`와 `/system/fail_safe/mrm_state`, MRM operate/status 서비스·토픽의 대체 주체와 동작을 정해야 한다. PlanningAA 소스에 route의 MRM 상태 자료구조가 있다는 것과 시스템 MRM을 실제 실행한다는 것은 다르다.

### 2. 차로 주행 중간 궤적 Event 12401 — 정의서에 있으나 실행 경계 미구현

| 항목 | 내용 |
| --- | --- |
| 계약 | `planning_scenarioplanning_lanedriving_trajectory`, `autoware_planning_msgs/msg/Trajectory`, Event **12401**, ROS `/planning/scenario_planning/lane_driving/trajectory`. `Decision <-> 외부 IF` 출력 행 29, `ServiceInterfaces모음` 행 355. |
| 현재 누락 | PlanningAA는 내부 `frame.lane_driving`을 만들지만 외부로는 최종 12506만 송신한다. 브릿지에도 12401 proxy, ROS publisher, 변환이 없다. |
| 구현 위치 | PlanningAA에 중간 궤적용 P-port/skeleton·양쪽 SOME/IP 매니페스트를 추가하고 `lane_driving` 변환·송신; 브릿지에 대응 R-port/proxy, 수신·ROS 발행을 추가한다. 생성된 예전 OCP→SS 포트의 12401을 그대로 브릿지 포트로 간주하지 말고 실행 중인 단일 PlanningAA 서비스 계약을 확정해야 한다. |
| 완료 조건 | 12401과 12506을 각각 구독해 목적에 맞는 궤적이 발행되는지 확인. 12401의 구체적 service ID/instance/버전·전송방식은 새 포트 생성 시 계약 확정이 필요하다. 기존 `Port&ID`의 OCP→ServiceCreator 후보는 **2017**이며, 현재 활성 브릿지/PlanningAA 매니페스트에 해당 포트가 없다. |

12401은 최종 control 입력인 12506과 다르다. 실제 control 기본 launch의 controller·validator는 12506을 구독한다. 따라서 12401은 **정의서 충족 항목**이지만 위의 control 직접 소비 누락보다 우선할 이유는 확인되지 않았다.

### 3. 수신된 입력의 PlanningAA 계산 연결 — 서비스 추가가 아닌 기능 누락

- Event **30201** 점군, **30301** 점유 격자, **11629** 횡방향 오프셋은 브릿지에서 전달되고 AA가 구독하지만 현재 주행 프레임 계산에는 반영되지 않는다. 해당 planner 입력으로 변환·주입할 경로와 유효시간, 좌표계 정책을 정해야 한다.
- 이미 사용 중인 인식 객체 **30101**, 교통 신호 **30401**, 가속도 **4001**, 기본 속도 제한 **12503**, 운영 모드 **6101**을 이 미연결 항목과 혼동하지 않는다.
- `control.launch.xml`의 제어 노드는 지도·route·odometry·인식 객체를 **ROS에서 직접** 구독한다. 점군을 받는 `obstacle_collision_checker`는 기본 preset에서 비활성이다. 이 토픽들을 브릿지를 다시 통과해야 하는 PlanningAA 입력으로 중복 분류하지 않는다.

## 검증·계약 확인 사항

1. perception/control/브릿지/PlanningAA를 같은 ROS 도메인에서 실행하고 각 입력의 실제 발행자·타입·QoS를 확인한다. 브릿지의 정적 구독 코드만으로 발행자 존재를 보장할 수 없다.
2. 지도·점군·점유 격자 등 큰 UDP Event의 실전송 가능 크기와 분할 설정을 양쪽 매니페스트에서 확인한다. 현재 `max-segment-len: 0`만으로 큰 메시지 전달 성공이 입증되지 않는다.
3. route 요청 → 10602 route/10604 state → 12506 최종 궤적 → control 수신을 종단 확인한다. 12401을 추가할 때는 별도 토픽에서 점·시각·frame을 비교한다.
4. 10602 route는 현재 코드에 송수신이 추가되어 있다. set/clear/reroute 뒤 UUID, header/frame, 시작·목표 pose, segment, 상태가 일관되는지 확인하고, clear 후 기존 route를 구독자에게 어떻게 해제할지 결정한다. `Decision <-> 외부 IF` 최소 출력 표에는 10602가 없지만 `AAP_IF`와 1814 매니페스트에는 정의되어 있다.
5. control image의 `vehicle_cmd_gate`·AD API에서 두 조명 명령과 운행 모드 가용성의 발행자를 확인한다. autonomous 전환, 방향지시등/비상등, 정상·비상 control gate 입력을 각각 검증한다. 시스템 진단·MRM은 오류를 유발하는 시험 조건에서도 따로 검증해야 한다.
