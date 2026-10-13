# Planning AA

Autoware Decision Planning의 계산과 상태 처리를 ROS 메시지에 의존하지 않는 C++ 코어로 옮기는 저장소다. 목표 배포 형태는 Adaptive AUTOSAR 애플리케이션이다. **2026-10-11 현재 10개 rosbag의 6,085프레임에서 최종 궤적을 생성하지만, 원본과 점별 동등성·남은 코드 이관·Adaptive 실행 앱은 미완료다.**

## 이번 작업 범위

- 대상: route·시나리오 선택, 차로 주행의 행동 경로와 속도, 경로 평활화·최적화, 장애물 정지·순항·감속, 최종 속도 평활화·검증, 평가·남은 거리 등 필요한 Planning 기능.
- **주차 기능은 이번 구현·검증 범위에서 제외한다.** 저장소에 남아 있는 costmap·자유공간 경로·주차 상태 코드는 과거 작업 결과이며 완료 조건이 아니다. 출발·회피가 사용하는 점유 격자 입력은 별개다.
- ROS 서비스·토픽을 그대로 발행하는 것은 배포 완료 조건이 아니다. 원본의 요청·상태·출력 의미를 내부 호출 또는 목표 플랫폼의 Adaptive 인터페이스로 전달해야 한다. 원본 endpoint 목록은 [기능 계약표](reference/decision_planning_contract_matrix.md)에 보관한다.

## Adaptive AUTOSAR 앱 In/Out 인터페이스 초안

아래는 **차로 주행용 `PlanningAdaptiveApp`의 외부 계약 v0.1**이다. 구현 완료를 뜻하지 않으며 서비스 인스턴스, 전송 바인딩, ARXML, 실제 타입 이름과 timeout 수치는 대상 Adaptive Platform·ECU를 정한 뒤 확정한다. 주차 계획은 제외하되 출발·회피에 필요한 점유 격자 입력은 포함한다. 원본 ROS endpoint별 의미와 현재 연결 상태는 [기능 계약표](reference/decision_planning_contract_matrix.md)를 따른다.

### 공통 데이터 규칙

모든 전송 데이터는 `schema_version`, `source_id`, `sequence_id`, 원천 생성 시각 `source_timestamp_ns`, 송신 시각 `publish_timestamp_ns`, `frame_id`를 가진다. 적용 가능한 경우 `map_version`과 `route_uuid`도 포함한다. 거리·속도·각도 단위는 각각 m·m/s·rad이다. 앱은 계획에 사용한 입력들의 `sequence_id`를 결과에 기록하고, 계산 전에 시각·좌표계·지도/route 버전의 일관성과 유효기간을 검사한다. 입력별 허용 지연과 궤적 유효기간은 목표 계획 주기와 제어기 계약을 기준으로 정한다. 재시작 또는 시각 역행 시 `sequence_id`만으로 신구 데이터를 판정하지 않도록 source 식별자와 시각을 함께 사용한다.

### In: 구독 데이터

| 인터페이스 | 최소 데이터 | 사용처·조건 |
| --- | --- | --- |
| `MapSnapshot` | Lanelet 지도, 지도 버전, 좌표계 | route·차로·규칙 판단. 지도 변경 시 기존 route와 연속 상태의 유효성을 재검사 |
| `EgoState` | pose, 선속도, 가속도, yaw rate, 관측 시각 | 계획 주기의 기준 자세와 동역학 상태 |
| `PredictedObjects` | 객체 ID·형상·분류·pose·속도·예측 경로 | 회피·정지·순항·차로 변경 안전 판단 |
| `TrafficSignals` | 신호 ID·상태·유효 시각 | 교차로·정지선·차로 밖 주행 판단 |
| `OccupancyGrid` / `PointCloud` | 격자·점군, 센서 좌표계·시각 | 이를 쓰는 활성 모듈에 공급. 필요 여부와 유효기간은 모듈별로 정의 |
| `OperationMode` | 자율주행·제어 활성, MRM 상태 | 계획 가능 여부와 main/MRM route 선택 |
| `VehicleConfig` | 차량 외형, 동역학 한계, 계획 파라미터 버전 | 충돌·기동·제동 계산 |
| `ExternalVelocityLimit` | 제한 속도, 적용 위치·범위, 해제 여부, 가감속·저크 제약 | 외부 속도 제한 선택과 최종 평활화 |

route를 이 앱이 소유하면 아래 route 요청의 결과를 내부 현재 상태로 사용한다. 다른 AA가 route를 소유하는 배치라면 `ActiveRoute`·`RouteState`를 같은 의미의 구독 데이터로 받아야 한다. 콜백마다 시각이 다른 입력을 한 프레임으로 묶는 규칙도 정의해야 하며, 단순한 최신값 취합이 원본 콜백 순서를 재현한다고 가정하지 않는다.

### In: 요청/응답

| Method | 요청 | 응답 |
| --- | --- | --- |
| `SetRoute` | `request_id`, main/MRM 채널, 목적지·경유지 또는 Lanelet 경로, 수정 허용 여부 | 수락 여부, 결과 코드, route UUID |
| `ClearRoute` | `request_id`, main/MRM 채널 | 결과 코드 |
| `UpdateGoal` | `request_id`, route UUID, 새 목표 | 결과 코드, 적용된 route UUID |
| `SetRtcApproval` | `request_id`, 대상 모듈·후보 ID, 승인/철회, 명령 시각 | 결과 코드, 적용 상태 |
| `LoadPlanningModule` / `UnloadPlanningModule` | `request_id`, 행동/동작 속도 모듈 종류와 이름 | 결과 코드, 적용된 모듈 목록 |

응답은 **요청 처리의 수락·실패**를 나타낸다. 그 뒤 변경된 route·상태·궤적은 별도 출력으로 발행한다. `request_id`로 중복 요청과 응답을 대응시키고, 동시 요청의 적용 순서·timeout·오류 코드를 정한다. 현재 `PlanningRouteCoordinator`와 일부 모듈 적재·해제 코어는 있으나 Adaptive 요청/응답 어댑터는 없다.

### Out: 발행 데이터

| 인터페이스 | 최소 데이터 | 발행 조건 |
| --- | --- | --- |
| **`PlannedTrajectory`** | 공통 메타데이터, 유효기간, 경로점 배열, 계획 상태, 사용한 입력 ID | 유효한 최종 계획이 나온 때 |
| `PlanningStatus` | `READY`/`DEGRADED`/`UNAVAILABLE`, 원인 코드, 현재 route UUID, 마지막 정상 계획 시각 | 상태 변경 시 및 주기적 상태 보고 |
| `RouteState` / `ActiveRoute` | main/MRM 상태, 선택된 route·UUID | route 요청·전환·도착·복구 시 |
| `ValidationStatus` | 궤적 유효성, 실패 항목, 이전 결과 사용 여부 | 매 계획 결과 |
| `PlanningReason` | 정지·감속 사유, 관련 객체·신호 ID, 적용 위치 | 해당 판단 발생·변경 시 |
| `VelocityLimitState` | 현재 적용 제한, 발생 주체, 해제 상태 | 제한 변경 시 |
| `VehicleSignalRequest` | 방향지시등·비상등 요청과 유효기간 | 기동 판단 변경 시 |
| `RerouteAvailability` / `ModifiedGoal` | 재경로 가능 여부, 수정 목표 | 상태 변경 시 |
| `RemainingDistanceTime` | 남은 거리·예상 시간, 기준 route UUID | 유효한 route에서 갱신 시 |

`PlannedTrajectory`의 경로점은 최소 `x/y/z`, 자세 quaternion, 종방향 속도·가속도, yaw rate를 담는다. 이는 내부 [`TrajectoryPoint`](include/planning/trajectory.hpp)의 필드를 바탕으로 한 외부 전송 형식 제안이며 내부 C++ 구조체를 그대로 직렬화한다는 뜻은 아니다. 중간 단계 경로, evaluator 지표, 상세 debug 출력은 기본 제어 계약과 분리해 필요성을 결정한다.

### 오류·수명주기 규칙

- 필수 입력이 없거나 만료되면 `PlanningStatus=UNAVAILABLE`과 원인 코드를 발행한다. 유효기간이 지난 궤적을 새 결과처럼 재발행하지 않는다.
- 계산 실패 시 이전 궤적 사용 여부는 validator 정책과 남은 유효기간으로 판단하고, 사용했다면 `ValidationStatus`에 표시한다. 제어기·Safety/Status AA와 timeout 및 안전 반응을 함께 정한다.
- MRM 전환, route UUID 변경, 지도 버전 변경, 앱 재시작에서 연속 계산 상태별 유지·초기화 규칙을 정한다. 현재 최종 스무더·validator는 route UUID 변경만으로 모두 초기화하지 않는다.
- 연속 데이터·결과는 `ara::com` Event, 명령은 Method로 모델링하는 구성을 우선 검토한다. 늦게 연결한 소비자가 현재 값을 읽어야 하는 상태·route는 Field 또는 초기값 제공 방식을 플랫폼 지원과 소비자 요구에 맞춰 정한다. 프로세스 시작·종료·상태 보고와 매니페스트는 Execution Management 통합 작업에 포함한다. [AUTOSAR Communication Management R25-11](https://www.autosar.org/fileadmin/standards/R25-11/AP/AUTOSAR_AP_SWS_CommunicationManagement.pdf), [Platform Design R25-11](https://www.autosar.org/fileadmin/standards/R25-11/AP/AUTOSAR_AP_EXP_PlatformDesign.pdf)

첫 종단 구현의 필수 계약은 `MapSnapshot`, `EgoState`, `PredictedObjects`, `TrafficSignals`, `OperationMode`, `SetRoute`/`ClearRoute`, `PlannedTrajectory`, `PlanningStatus`, `ValidationStatus`로 시작한다. 활성 모듈에 필요한 나머지 입력은 해당 기능을 연결하기 전에 추가하고, RTC·동적 모듈 제어와 부가 출력은 다음 통합 단계에서 연결한다.

## 소스 구조

`src`는 원본 Autoware Planning의 기능 경계를 따라 정리했다. `behavior_path_planner`, `behavior_velocity_planner`, `motion_velocity_planner`, `mission_planner`가 주요 계산 모듈이다. 행동 속도 안의 횡단보도·교차로·돌발진입 등은 각 하위 폴더에 모았다. 그 밖에 `path_optimizer`, `path_smoother`, `velocity_smoother`, `obstacle_cruise_planner`, `planning_validator`, `pipeline` 등이 있다. 지도·포인트클라우드 변환은 `adapters`에 두었으며, 공개 헤더는 기존 `include/planning`에 있다. 이 정리는 파일 위치만 바꾼 것으로 계산 동작과 이관 완료 범위에는 영향을 주지 않는다.

## 구현과 검증 상태

| 영역 | 현재 구현 | 확인한 범위와 남은 차이 |
| --- | --- | --- |
| mission planner·route selector | 지도 경로, 설정·해제, main/MRM 전환과 상태 코어, 두 코어의 요청·route/state 이벤트를 잇는 `PlanningRouteCoordinator` | 원본의 성공·실패·복구 사례 일부와 코어 간 이벤트 순서를 대조했다. 독립 실행 프로세스의 요청 전달·동시성·재시작·TF·발행 시각 계약이 남았다. [원본 생명주기](reference/mission_planner_lifecycle_reference_20261010.md), [route 전환](reference/route_selector_live_reference_20261010.md) |
| route 기준선 | 원본의 원시 중심선 절단과 최근접 선분 선택을 조건부 진단 경로에 연결하고, 끝점·복수 차로 ID 점의 0.1m 근접 격자점 대체 규칙을 이관했다 | 종점·차로 경계를 2m 격자에서 0.05m 옮긴 합성 지도 검사를 통과했다. 후방 7.1m와 root 표식 시각을 가정한 진단의 점별 일치는 blocking 374/552, S자 540/728프레임이다. 기록 당시 후방 설정·콜백 입력 ID가 없어 이 가정을 기본값이나 원본 동등성으로 판정하지 않는다. [기준선 감사](reference/behavior_route_callback_boundary_20261011.md) |
| 행동 경로·행동 속도 | 차로 변경·출발·목표·정적 회피의 일부 경로와 지도 속도 모듈. 승인된 차로 변경 경로의 연속 유지·종료, 행동 속도 단일 모듈 plugin 13개와 교차로 복합 plugin의 이름별 적재·해제를 추가했다 | IPC 정지선 초기 25프레임에서는 첫 프레임을 제외한 행동 속도 경로 24/24가 점 수·점별 속도에서 일치했다. 차로 변경의 실제 승인 입력 시각과 나머지 활성 분기·출력 계약은 미완료다. |
| Elastic Band·경로 최적화 | 재표본화, QP, 이전 경로 상태의 일부 | IPC 정지선 Elastic Band 연속 470개 입력의 점 수 일치, 최종 위치 최대 차이 0.001708m. 최적화의 541개 비교 프레임은 점 수·속도 일치, 위치 최대 차이 0.039122m. S자 첫 유효 재생 전에 원본 Elastic Band 3개·최적화 7개 출력이 있어 현재 기록만으로 초기 상태를 동일하게 복원할 수 없으며, 재계획 경계도 남았다. [연속 상태 감사](reference/elastic_band_sequence_audit_20261010.md) |
| 동작 속도 | 동적 장애물 정지·장애물 속도 제한의 결과 적용. 별도 입력을 받은 차로 밖 주행의 지도 후보·연속 정지/감속 결정과 콜백 속도·가속도·평활화 제한에서 최소 정지 호길이를 계산하는 분기를 연결했다. 원본의 전체 class 이름으로 3개 plugin을 적재·해제해 적재 순서대로 실행 | 합성 지도에서 차로 밖 주행 정지 계획과 제동 호길이 분기를 검사했다. 보유 bag의 실제 동작 속도 콜백 입력 짝·당시 평활화 제한은 확정할 수 없어 기본 종단 재생에서는 차로 밖 주행 입력을 공급하지 않는다. 대상 실행 앱의 요청·응답 연결은 남았다. |
| 장애물 정지·순항·감속 | 객체 후보, PID 순항, 정지·감속 선택과 궤적 적용의 일부. 원본 `LongitudinalInfo::safe_distance_margin`을 정지 후보에도 전달해 기존 4m 고정값을 제거했다 | 현재 원본 노드와 같은 입력 시계로 재실행한 HPC 돌발진입의 감속 활성은 23/23프레임 일치하고 감속 구간 시작·종료 거리 최대 차이는 0.099/0.059m다. 기록 당시 원본과 현재 이미지의 객체 선택이 다른 구간 및 실제 차로 밖 정지 활성 사례는 추가 대조가 필요하다. [감속 대조](reference/obstacle_cruise_slowdown_source_probe_20261011.md) |
| 최종 속도·검증·평가 | 외부 제한의 감속 거리·저크 제약, 최종 스무더, validator 및 기본 평가·남은 거리 계산. route UUID 변경만으로 독립 스무더·validator 상태를 지우지 않고, 원본의 경로 절단·저속 출발 외부 제한 순서와 제한점 삽입 기준을 반영했다. 이전 궤적의 뒤쪽 속도 보존에서는 원본처럼 거리·방향 제약으로 이전 **점**을 고른 후 선분 보간한다 | 10개 bag 전체에서 최종 궤적을 생성했다. S자 613·630번의 큰 차이는 제한점 삽입 수정으로 줄었지만, 시나리오 전체의 상태·입력 형상·콜백 시각은 더 대조해야 한다. |

CTest의 기준 데이터에는 실제 bag, 원본 노드 실행 결과, 합성 입력이 섞여 있다. 합성 사례의 수치 일치를 실제 주행의 종단 동등성으로 해석하지 않는다. 주차 관련 등록 검사는 과거 코드의 회귀 검사일 뿐 이번 완료 범위에 포함되지 않는다.

## K-City 독립 종단 재생

10개 rosbag의 route부터 최종 validator까지 **6,085/6,085프레임**에서 planning-aa가 궤적을 만들었다. 원본 최종 궤적과 같은 차량 위치에서 속도를 비교할 수 있는 프레임은 6,047개다. 아래 값은 원본·독립 궤적 모두 차량 위치에서 2m 이내인 프레임의 계획 속도 절대 오차다. 기본 재생은 감속 후보를 켠 별도 진단과 구분한다.

| 시나리오 | 비교 프레임 | 속도 오차 중앙값 / 95백분위 (m/s) | 0.5m/s 이내 |
| --- | ---: | ---: | ---: |
| HPC blocking | 573 | 0.145 / **3.707** | 59.9% |
| HPC cutinout | 633 | 0.026 / 1.757 | 75.8% |
| HPC pedestrian2 | 511 | 0.013 / 1.059 | 86.7% |
| HPC rotary | 525 | 0.005 / 0.068 | 99.8% |
| HPC runout | 539 | 0.012 / 0.418 | 95.7% |
| HPC scurve | 749 | 0.036 / **1.741** | 71.7% |
| HPC stopline2 | 742 | 0.002 / 0.058 | 100% |
| IPC cutinout | 747 | 0.015 / 1.135 | 85.9% |
| IPC pedestrian | 454 | 0.020 / 1.124 | 76.7% |
| IPC stopline | 574 | 0.002 / 0.026 | 100% |

전체 비교 프레임의 85.1%가 0.5m/s 이내다. 이는 **차량 위치의 속도** 통계이며 궤적 점별 동등성을 뜻하지 않는다. 각 시나리오의 그래프·CSV와 재생 조건은 [시나리오 비교](reference/planning_pipeline_multi_scenario_20261011.md)에 있다. `251231_KCity_ori`의 각 `result`에 planning-aa 산출물로는 기본 CSV·PNG와 재생 조건 JSON만 남겼다. 추가 진단 CSV·로그는 `251231_KCity_ori/planning_aa_result_diagnostics_20261011.tar.gz`에 보관하고 핵심 대조 입력은 [reference/data](reference/data/README.md)에 두었다.

지도 경로는 goal 높이를 모든 점에 복사하지 않고 Lanelet2 중심선 높이를 유지한다. 원본 장애물 순항 노드의 안전거리 설정을 정지 후보에도 전달했고, 최종 평활화의 외부 속도 제한점은 원본처럼 최근접 선분 시작점에서 거리를 재도록 맞췄다. 이번 기준선 재표본화 수정 후 10개 bag을 다시 재생해 **6,085/6,085 생성, 6,047개 속도 비교**를 확인했고 CSV·비교 PNG를 갱신했다. blocking의 속도 오차 95백분위 3.707m/s와 S자의 1.741m/s는 사실상 그대로다.

### 가장 큰 차이의 시작점

- **HPC blocking:** 차로 변경 구간에서 행동 경로부터 다르다. 320번 프레임의 독립 경로는 차로 36에서 13점으로 끝나고 원본은 차로 15850·15910까지 54점으로 이어진다. 원본 RTC 인터페이스의 자동 모드 출력은 585/585개가 true지만, 기록된 개별 차로 변경 상태는 수동 명령 상태다. 승인 *입력 시각*은 없다. 원본 동작 속도 경로를 조건부로 후보 코어에 넣으면 순항 활성은 현재 원본 노드와 45/45프레임 일치하므로, 우선 경로·승인 상태를 맞춰야 한다. [대조 기록](reference/obstacle_cruise_slowdown_source_probe_20261011.md)
- **HPC scurve:** 기본 재생은 390번 CAR를 순항으로 선택하지만, 원본 패키지의 3.5/3.0m/s 전환 임계값과 동작 속도 수신 시계의 조건부 재생에서는 차로 주행 정지점이 원본 10.266m/독립 약 10.3m로 가까워진다. 478~535번은 미래 위치가 주행 영역을 벗어난다는 판정으로 독립 정지가 빠진다. 보관된 현재 원본 노드 재실행도 같은 58프레임에서 정지를 제외하므로 기록 당시 설정·버전·입력 짝은 여전히 미확인이다. 외부 제한점 위치를 원본에 맞춘 뒤 기본 재생의 613번 최종 정지점은 원본 64.3m/독립 63.9m, 630번 최종 차량 위치 속도는 원본 7.08m/s/독립 7.09m/s가 됐다. 종단 전체 점별 동등성은 아직 아니다.

갱신한 기본 CSV에서 0.5m/s를 넘는 연속 구간은 blocking 204~433번 230프레임, S자 362~573번 212프레임에 집중된다. 조건부 전체 단계 진단에서는 blocking의 첫 차이가 행동 경로인 프레임이 420/573이고, S자는 행동 경로 238/752·Elastic Band 180/752·경로 최적화 334/752프레임이다. 장애물 객체 선택, 승인 상태, 이전 최적화 상태와 콜백 입력 시각을 단계별로 분리해야 한다.

[행동 경로 기준선·콜백 시각 감사](reference/behavior_route_callback_boundary_20261011.md)에서는 원본 root 표식과 독립 기준선을 분리해 비교했다. S자 130번 이후의 첫 차이는 원시 중심선 절단 격자와 입력 선택 시각에 있다. 원본 절단 순서와 표식 수신 시각을 사용한 **진단**에서 후방 길이 7.1m를 가정하면 S자·blocking의 여러 초기 구간 root 좌표가 일치한다. 기록 당시 설정값은 확인되지 않았다. 원본식 원시 경로 절단은 선택적 진단으로 유지하며, 외부 속도 제한점 삽입 계산은 원본 코드에 맞춰 기본 재생에 반영했다. 원본 동작 모드에 따른 현재 route 차로 재선택도 선택적 진단 입력으로 연결했으며, 승인 모듈의 정확한 상태가 없으면 동등성을 판정하지 않는다.

진단 경로의 최근접 선분 선택도 원본의 거리·방향 제약과 대체 탐색으로 맞췄다. 이어 원본 `motion_utils::resamplePath`의 끝점·복수 차로 ID 점에 대한 0.1m 근접 격자점 대체 규칙을 기준선 재표본화에 적용했다. 첫 160프레임에서 후방 7.1m·root 표식 시각을 함께 준 조건부 root 좌표 일치는 blocking 107→127/140, S자 130→134/135프레임이다. 전체 bag 진단에서는 각각 374/552, 540/728프레임이다. 원본 기본 YAML은 5m이고 기록 당시 설정과 콜백 입력 ID가 없으므로 7.1m를 기본 설정이나 원본 동등성으로 확정하지 않는다. 기본 10개 시나리오 종단 그래프도 다시 생성했으며 속도 지표는 거의 변하지 않았다.

### 조건부 검증과 미확인 사항

원본 `LaneChangeInterface`는 실행 중 승인 경로를 유지하고 `NormalLaneChange::hasFinishedLaneChange`에서 목표 차로·종료 거리 또는 측방/방향 오차로 종료한다. 이에 따라 승인 경로를 콜백 사이에 유지하도록 연결했다. 원본 Lanelet2의 `offsetNoThrow`를 사용해 변경 방향의 목표 차로 바깥쪽만 1m 확장하고, 원본 객체 선별에 없는 엄격한 다각형 유효성 조기 거부를 제거했다. polygon 객체는 원본처럼 외곽 점으로 판정하며 길이·폭 필드가 0이어도 제외하지 않는다.

blocking bag에서 **가상의** 170~350번 승인 입력을 준 조건부 재생은 기존에는 후보 350개 모두 확장 다각형 검사에서 거부됐으나 수정 뒤 안전 후보와 차로 변경 경로가 나왔다. 실행 중 새 후보 요청이 비어도 승인 경로를 유지하는 원본 `LaneChangeInterface` 동작을 이식한 뒤 차로 변경 실행은 81→105프레임이다. 320번 최종 계획 속도는 원본 1.858m/s, 기본 독립 5.138m/s, 가상 승인 독립 2.361m/s다. 가상 승인 재생의 95백분위 오차는 기본 3.707→2.958m/s로 줄었지만 중앙 오차는 0.145→0.337m/s로 커졌다. 279·320번의 가상 승인 행동 경로에는 원본의 목표 차로 종단 정지점이 없고 경로 형상도 다르다. 원본 승인 **명령 시각**은 bag에 없으므로 이 결과는 원본 동등성이나 기본 성능 개선의 증거가 아니다. 승인 입력이 없는 blocking 기본 재생 573프레임의 최종 점 수·차량 위치 속도·첫 정지 거리는 기존 CSV와 동일했다. 첫 차이, 하위 모듈의 동일 입력 분리, 시계 진단은 [blocking 경계 보고서](reference/blocking_lane_change_boundary_20261011.md)에 정리했다.

순항 객체의 거친 횡거리 검사와 충돌 탐색 반경은 원본 `calcObstacleMaxLength`처럼 박스의 반대각선 절반, 원기둥의 반지름, 다각형 외곽 점의 최대 거리로 계산한다. 다각형의 길이·폭 필드가 0이어도 유효한 외곽 점을 가진 객체는 검사한다. 합성 다각형 분기를 확인하고 10개 bag을 다시 재생했으며, 기본 결과는 6,085/6,085프레임 출력, blocking 95백분위 3.707m/s, S자 1.741m/s로 이전과 같았다. 따라서 이 수정이 두 시나리오의 남은 큰 차이를 해소했다는 근거는 없다.

원본 순항 노드의 궤적 진행 방향 판정도 후보 코어에 연결했다. 첫 두 경로점과 첫 점의 방향으로 후진을 판정하면 원본처럼 순항 후보를 제외하고, 정지 교점·도착 시간에는 뒤 범퍼와 후진 방향을 적용한다. 후진 합성 입력 검사를 통과했고 blocking·S자 기본 재생 CSV는 수정 전과 바이트 단위로 같았다. 보유 bag에서 이 후진 분기가 활성이라는 증거는 없으므로 후진 전체 동등성은 합성 분기 검증 수준이다.

route 독립 재생기에 프레임별 또는 수신 시각별 외부 승인 CSV 입력과 첫 차이 단계·후보 안전 진단 열을 추가했다. 수신 시각 입력의 현재 적용 경계는 행동 경로 **출력** 수신 시각이며 실제 승인 콜백 시각은 아니다. 기록된 RTC 상태는 계속 비교 출력으로만 사용한다.

HPC S자의 기록 중간 경로와 이전 스무더 출력을 넣은 **조건부** 계산에서는 390~419번의 공통 경로 속도 최대 차이가 0.012m/s, 620~649번은 0.187m/s였다. 당시 630번 종단 차이 4.713m/s는 외부 제한점 삽입 기준을 원본에 맞춘 뒤 기본 재생에서 거의 없어졌다. 다만 다른 정지 후보 구간과 콜백 입력 짝·연속 상태가 여전히 달라 전체 동등성을 뜻하지 않는다.

## 남은 코드 작업

1. 출발·정적 회피의 남은 상태/안전 분기와 기본 활성 속도 모듈의 남은 계산을 종단 경로에 연결한다. 차로 변경의 실제 RTC 승인 명령 시각은 보유 기록에서 알 수 없으므로 승인 분기의 원본 동등성은 미확인으로 유지한다. 현재 승인 경로 유지와 곡선 차로 확장 경계는 코어에 반영했다.
2. 연결된 mission/route 코어와 행동·동작 속도 모듈 적재·해제를 대상 실행 앱의 요청·응답 및 출력 계약으로 옮긴다. 행동 속도 단일 모듈 plugin 13개와 교차로 복합 plugin의 원본 load class 이름·unload 모듈 이름을 pipeline 어댑터에 연결했다. 동작 속도 plugin 3개는 원본의 전체 class 이름으로 적재·해제하며, 기본 launch 순서인 OutOfLane → 장애물 제한 → 동적 정지로 결과를 적용한다. OutOfLane은 별도 콜백의 객체·신호와 최소 제동 호길이 또는 그 계산에 필요한 콜백 상태·평활화 제한을 명시적으로 받으면 종단 계산에 연결된다. 기본 재생의 정확한 콜백 입력 공급, RTC, 좌표계·시각·유효기간·버전, 부가 출력·오류 의미는 미완료다. 주차 계약은 제외한다.
3. 평활화·최적화·장애물 후보·최종 스무더의 연속 상태를 같은 원본 콜백 입력으로 비교하고, blocking·S자에서 계산의 첫 차이를 줄인다. S자 초기 조건부 재생은 행동 경로·행동 속도 158/160프레임이 점별로 같아도 첫 프레임 Elastic Band가 482/484점이고 시작점이 약 1m 다르다. 원본 Elastic Band에는 독립 재생 시작 전 출력 3개가 있고 그 첫 출력보다 첫 odometry 기록이 늦다. 이전 최적화 상태를 현재 bag 입력만으로 재현할 수 있는지와 `cropPoints` 후방 절단·실행 당시 입력 자세·후방 길이를 분리해 확인해야 한다. S자 390번의 정지 누락은 원본 패키지 전환 임계값을 쓰는 조건부 재생에서 해소되지만, 478~535번 미래 충돌·일시 통과 판정은 기록 원본과 현재 원본 노드 재실행부터 다르다. 당시 실행 설정·버전이나 실제 polling 입력을 확보해 차이를 분리해야 한다. 630번은 기본 재생 속도가 가까워졌으나 전체 시나리오와 연속 상태까지 동등하지 않다. 최종 스무더의 원본 순서인 앞 200m·뒤 5m 절단 후 외부 제한·정지 접근 적용, 저속 출발 중 외부 제한 직접 적용, 정지 중 제한 거리 초기화와 제한점 삽입 기준은 연결했다. 10개 bag 기본 재생의 S자 749개 비교 프레임 중앙값/95백분위는 0.036/1.741m/s다. 원본 당시 콜백 ID·설정이 없는 분기는 동등성 미확인으로 유지한다.
4. 대상 Adaptive AUTOSAR 플랫폼과 프로세스 경계를 정한 뒤 실행 앱·매니페스트·브리지·재시작/상태 복원을 구현하고 목표 장치에서 시간·풀이기 정책을 검증한다. 이 단계는 아직 구현 전이다.

[동작 속도 콜백 감사](reference/motion_velocity_callback_audit_20261011.md)는 blocking·S자 입력 경로와 원본 출력의 header를 대응시키고, 그 사이에 새 odometry·acceleration·객체가 들어온 프레임 수를 기록한다. 당시 첫 큰 차이였던 S자 630번 최종 스무더 속도는 이번 제한점 삽입 수정으로 줄었다. blocking 320번의 행동 경로와 S자 390번의 차로 주행 정지 삽입은 남는다. 이 감사는 실제 polling 반환 메시지 ID를 복원하지 않는다.

## 빌드

### Adaptive AUTOSAR 컨테이너 부트스트랩

`adaptive/`에는 PopcornSAR PARA **R20-11 x86_64** SDK를 사용한 별도 빌드 진입점, Execution Management 매니페스트, `PlanningAA` 프로세스와 Docker 이미지 정의가 있다. SDK는 저장소에 복사하지 않는다. 기본 SDK 경로는 인접한 `../oss-ap/sdk/x86_64`이며, 다른 설치본은 `PARA_SDK`로 지정한다. 이 SDK의 버전은 `R20-11_v26.04.03_native-linux-gcc`이다. SDK 라이선스가 복제·배포를 제한하므로 SDK 런타임을 포함한 이미지는 권한을 확인하기 전까지 로컬 개발용으로만 사용한다.

```sh
./adaptive/build-image.sh
./adaptive/run-image.sh
```

`IMAGE_TAG`로 이미지 이름을 바꿀 수 있다. `build-image.sh`는 Docker BuildKit의 별도 `para_sdk` 빌드 컨텍스트를 사용해 SDK를 빌드 중에 공급하며, 설치 트리에 EM·PARA 런타임·`PlanningAA`를 모아 실행 이미지에 넣는다. Docker 없이 CMake 빌드만 확인하려면 다음을 실행한다.

```sh
cmake -S adaptive -B build-adaptive -G Ninja \
  -DPARA_SDK="$(realpath ../oss-ap/sdk/x86_64)" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$(pwd)/output/adaptive"
cmake --build build-adaptive -j 4
cmake --install build-adaptive --component Standalone
```

현재 `PlanningAA`는 `planning_core`에 링크하고 `ara::core` 초기화, `ara::exec`의 Running 보고, 종료 신호 처리를 수행하는 **실행 골격**이다. [위 In/Out 계약](#adaptive-autosar-앱-inout-인터페이스-초안)의 `ara::com` 서비스·이벤트, 지도 입력, 실제 궤적 발행은 아직 연결되지 않았다. 초기 빌드는 지도 어댑터와 ProxQP를 끄므로 원본 계산 동등성이나 실차 배포를 검증한 이미지가 아니다.

### 계산 코어

C++17, CMake, Eigen 3, Boost.Geometry 헤더가 필요하다. 지도·ProxSuite 등 선택 기능은 설치 의존성에 따라 활성화된다. 계산 코어에는 ROS/ament/ROS 메시지 생성물이 필요하지 않다.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j 4
ctest --test-dir build --output-on-failure
```

선택적 지도·격자 어댑터를 제외한 코어 빌드:

```sh
cmake -S . -B build-core-only -DCMAKE_BUILD_TYPE=Release \
  -DPLANNING_BUILD_LANELET_BIN_ADAPTER=OFF \
  -DPLANNING_BUILD_OSM_ADAPTER=OFF \
  -DPLANNING_BUILD_GRID_ADAPTER=OFF \
  -DPLANNING_USE_PROXQP=OFF
cmake --build build-core-only -j 4
ctest --test-dir build-core-only --output-on-failure
```

마지막 전체 구성의 **240개 CTest**와 지도·선택 라이브러리를 뺀 코어 구성의 **103개 CTest**가 모두 통과했다. CTest 통과는 위 원본 종단 차이의 해소를 의미하지 않는다. `cmake --install build --prefix /설치/경로`로 `planning_core`를 설치할 수 있으며 [독립 소비 예제](examples/standalone_consumer)가 공개 헤더·링크 사용을 보여 준다. 설치본의 route coordinator 공개 헤더도 별도 소비자에서 컴파일·링크·실행했다.
