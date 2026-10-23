# PlanningAA Adaptive 실행 구성

이 디렉터리는 `planning_core`를 PopcornSAR PARA R20-11 기반 Adaptive AUTOSAR 앱으로 빌드하고, `oss-ap-planning-bridge` 복사본의 Service Creator와 별도 컨테이너에서 실행하기 위한 파일을 담는다. PlanningAA 빌드에 필요한 SDK와 생성 파일은 `vendor/`에 로컬 복사되어 있다. 원본 `oss-ap` 저장소는 수정하지 않는다.

## 현재 인터페이스

| PlanningAA 포트 | 방향 | SOME/IP 서비스 | 내용 |
| --- | --- | --- | --- |
| `RPort_SCr2MP` | `oss-ap` → PlanningAA | 4308 | clock, localization, 지도, perception, 속도 제한, 운영 모드 등 입력 Event 11개 |
| `PPort_MP2SCr` | PlanningAA ↔ `oss-ap` | 1817 | clear route 10801, set lanelet route 10809, set waypoint route 10812 |
| `PPort_MP2RS` | PlanningAA → `oss-ap` | 1814 | 경로 상태 Event 10604 (`route_selector/main/state`) |
| `PPort_PV2SCr` | PlanningAA → `oss-ap` | 2317 | **최종 궤적 Event 12506 하나** |

`PlanningAA`는 `map_vectormap`의 `LaneletMapBin.data`를 Lanelet 어댑터로 해독한다. 지도, route, odometry와 운영 모드가 준비되면 새 odometry마다 route 기반 주행 경로를 만들고, 인식 객체·교통 신호·가속도·속도 제한을 반영해 `PlanningFinalFrame`을 조립한다. 최종 파이프라인의 검증을 통과한 궤적만 Event 12506으로 변환해 `Send()`한다. AA의 `PlanningValidatorCore` 결과 14개는 `run-katech-stack.sh`가 시작한 ROS 진단 어댑터로 전달되어 `/diagnostics`에 발행된다. 검증에 실패하면 ERROR 진단을 보내고 궤적을 발행하지 않는다. 진단 어댑터는 검증 계산을 하지 않으며 Control 이미지에도 Planning 검증 노드를 추가하지 않는다. `Planning services active`는 통신 준비 상태이며, 실제 송신은 `Published final trajectory Event 12506` 로그로 확인한다. 지도와 odometry의 `frame_id`가 일치해야 한다. Lanelet route 요청의 segment를 사용하고, waypoint route 요청은 해독한 지도에서 경로를 탐색한다. 자세한 상태는 [저장소 README](../README.md#adaptive-autosar-컨테이너와-oss-ap-브리지)를 참고한다.

실행 매니페스트는 `CM=Running`을 `PlanningAA`의 시작 조건으로 둔다. CM이 초기화되기 전에 `OfferService()`를 호출하면 서비스 제공이 실패할 수 있다.

## 필요한 것

- Linux x86_64, Docker Buildx. 로컬 CMake 빌드에는 CMake, Ninja, C++17 컴파일러, Eigen 3, Boost 헤더·serialization 라이브러리가 필요하다. 이미지 빌드는 호스트의 `/usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.74.0`을 사용하며 `BOOST_SERIALIZATION_LIBRARY`로 경로를 바꿀 수 있다.
- PARA SDK `R20-11_v26.04.03_native-linux-gcc`. 기본 위치는 `vendor/para-sdk/x86_64`이며 `PARA_SDK`로 바꿀 수 있다.
- 생성 ARA 헤더·매니페스트. 형제 디렉터리의 `oss-ap-planning-bridge`가 있으면 이를 사용하며, `OSS_AP_SOURCE`로 바꿀 수 있다. 기존 `vendor/oss-ap`에는 Service 1814 생성물이 없으므로 이 버전의 이미지 빌드에는 최신 브리지 저장소가 필요하다.
- `../oss-ap-planning-bridge`에서 빌드한 `oss-ap-planning-bridge:local` 이미지. 다른 태그를 쓸 때는 `BRIDGE_IMAGE`를 지정한다.

`vendor/`에는 SDK 약 150MB와 기존 생성 헤더·매니페스트를 복사했다. 새 Service 1814 생성물은 형제 디렉터리의 브리지 저장소에서 가져온다. `.gitignore`와 `.dockerignore`가 `vendor/`를 제외한다. Docker BuildKit은 빌드 스크립트가 지정하는 별도 로컬 컨텍스트에서 이를 읽는다. Lanelet 지도 해독에는 저장소의 `lib/Rosless-Lanelet2`와 Boost serialization이 필요하다. 빌드 스크립트는 호스트의 Boost serialization 라이브러리를 임시 로컬 컨텍스트로 전달한다. SDK 및 생성 파일과 이를 포함한 실행 이미지는 라이선스 조건 때문에 외부에 배포하지 않는다. 다른 환경으로 소스만 옮길 때는 `vendor/`를 별도로 준비해야 한다.

## 이미지 빌드와 실행

두 저장소가 같은 상위 디렉터리에 있을 때 다음 순서로 실행한다.

```sh
cd ../oss-ap-planning-bridge
./oss-ap setup
./oss-ap build
./oss-ap image
cd ../planning-aa
./adaptive/build-image.sh
./adaptive/run-stack.sh
```

KATECH Autoware의 perception·control 이미지와 함께 시험할 때는 Autoware 저장소를 수정하지 않고 다음 스크립트를 사용한다. 이 스크립트는 기존 `run_partitions.sh`의 호스트 네트워크/ROS 도메인 42와 브리지 전용 네트워크/도메인 0의 불일치를 없애기 위해 네 컨테이너를 `planning-aa-net`, ROS 도메인 42, Cyclone DDS로 실행한다. 종료 시 시작한 컨테이너를 정지한다.

```sh
KATECH_MAP_PATH="$HOME/autoware_map/sample-map-planning" \
  ./adaptive/run-katech-stack.sh
```

`simple_planning_simulator`는 창이 없는 ROS 노드다. 지도와 차량을 보여주는 창은 RViz이며, 스크립트는 데스크톱 세션의 `DISPLAY`와 X11 소켓이 있으면 perception 컨테이너에서 RViz를 함께 실행한다. 화면이 없는 환경에서는 자동으로 끄며, `KATECH_RVIZ=true` 또는 `KATECH_RVIZ=false`로 명시할 수 있다. RViz에서 유효한 초기 위치를 지정해야 시뮬레이터의 `waiting initialization...` 상태가 끝난다. 창이 열리지 않으면 `docker logs planning-aa-perception`에서 `rviz2` 오류를 확인한다.

`run-katech-stack.sh`는 `PLANNING_AA_SIMULATION=1`을 기본으로 전달한다. 샘플 시뮬레이터에 신호 인식 메시지가 없으면 신호등 앞에서 계속 정지하지 않도록 하는 원본 planning simulator 설정과 같다. 실제 신호를 처리해야 하는 실행에서는 `PLANNING_AA_SIMULATION=0 ./adaptive/run-katech-stack.sh`로 시작한다. 이때 신호 메시지가 없거나 오래되면 정지한다.

`run-katech-stack.sh`는 perception, control, 브리지, PlanningAA의 로그를 각각 별도 터미널에 표시한다. 로그 파일은 실행마다 `output/katech-logs/` 아래 새 디렉터리에 저장되고, 시작 터미널에 정확한 경로가 출력된다. `KATECH_LOG_DIR`로 저장 위치를 지정할 수 있다. 화면이나 터미널 프로그램을 사용할 수 없는 환경에서도 파일 수집은 계속된다. 시작 터미널에서 Ctrl+C를 누르면 스택이 종료되며 로그 파일은 남는다.

KATECH 실행에서는 브리지의 `SERVICECREATOR_TRACE_VALUES=1`을 켜서 `planning-aa-bridge.log`에 궤적의 점 개수·첫 점·시각을 기록한다. Control 컨테이너에서는 읽기 전용 진단 구독자를 실행해 `/planning/scenario_planning/trajectory`의 발행자, 수신 개수, 첫 점·끝 점, 유한하지 않은 필드를 `trajectory-diagnostic.log`에 저장한다. AA가 계산한 검증 결과는 Docker 내부망의 UDP 12507을 통해 브릿지 컨테이너의 `validator-diagnostic-bridge.py`로 전달하고, 이 어댑터가 `/diagnostics`에 발행한다. 진단 어댑터의 기동 오류는 `validator-diagnostic-bridge.log`, 각 진단 상태 변화는 `trajectory-diagnostic.log`에 남는다. Goal pose를 지정한 뒤 두 궤적 로그의 시각과 첫 점을 대조하면 브리지와 ROS 수신 사이의 차이를 확인할 수 있다. 값을 끄려면 `SERVICECREATOR_TRACE_VALUES=0`으로 실행한다.

`planning-aa.log`의 `PLAA_TIMING`에는 경로 계산, 최종 처리, 스무더, 첫 QP 풀이와 재시도 시간·반복 횟수가 프레임마다 기록된다. KATECH 실행에서 기본 활성화되며 `PLANNING_AA_TIMING=0`으로 끌 수 있다. RViz의 `ScenarioTrajectory`는 제어용 국소 궤적으로, 원본과 같은 후처리 최소 길이 30 m 때문에 목표 전에 끝나 보일 수 있다. 전체 경로는 Perception 컨테이너의 UDP 12508 진단 어댑터가 `/planning/mission_planning/route_marker`의 `RouteArea`에 별도로 표시한다. `route-visualization-bridge.log`에 발행된 경로 점 수와 끝 좌표를 남긴다. 이 마커는 제어용 궤적에 영향을 주지 않는다.

### 정적 보행자 회피 확인

RViz에서 검증된 배치를 좌표 입력 없이 재현하려면 실행 중인 AA 스택에 다음 명령을 보낸다. 기본값은 차량 `(3646.5, 73520.0)`, 목표 `(3632.5, 73628.0)`, 정지 보행자 `(3645.6, 73545.0)`이다. 스크립트는 기존 더미 객체와 경로를 지우고, `/initialpose3d`와 waypoint route를 설정한 뒤 RViz의 `2D Dummy Pedestrian`과 같은 `DummyObject`를 발행한다. 보행자가 `/perception/object_recognition/objects`에 들어온 것을 확인하고 종료하며, 차량 주행은 RViz에서 Autonomous를 켜야 시작한다.

이 배치의 회피는 **동일 차로 안에서 약 0.2 m 비켜가는 정적 회피**다. 옆 차로로 이동했다가 보행자 뒤에서 원래 차로로 돌아오는 차로 변경 궤적이 아니다. 지도 경로의 lanelet `102 → 87` 옆에는 반대 방향 lanelet `103 → 86`이 있으며, 원본 설정의 `avoidance_by_lane_change.target_filtering.target_type.pedestrian`은 `false`다. 따라서 이 배치의 보행자만 옮겨서는 옆 차로 왕복 궤적을 검증할 수 없다.

```sh
./adaptive/set-pedestrian-scenario.sh
```

좌표를 바꾸려면 예를 들어 `./adaptive/set-pedestrian-scenario.sh --start-x 3648.46 --start-y 73505 --ped-x 3645.6 --ped-y 73545`처럼 전달한다. 시작 위치 `(3648.46, 73505.0)`는 보행자 통과 주행 시험에, 기본 위치 `(3646.5, 73520.0)`는 정차 상태에서 회피 곡선을 바로 보는 데 사용했다.

직선 구간의 차선 가장자리에 정지 보행자를 두면, 별도의 행동 명령 없이 인식 객체를 정적 회피에 전달한다. 원본 설정과 같이 RTC 승인을 요구하지 않으며, 인식이 잠시 끊겨도 등록된 회피 이동을 이어간다. `RouteArea`에는 이동 전 전역 경로를, `ScenarioTrajectory`에는 회피를 적용한 제어 궤적을 표시한다. 원인을 추적할 때 `PLANNING_AA_AVOIDANCE_DEBUG=1 ./adaptive/run-katech-stack.sh`로 시작하면 `planning-aa.log`에 객체 선택과 회피 활성화 또는 거부 단계를 출력한다.

샘플 지도에서 아래 명령은 이미 초기 위치와 목표를 설정한 스택에 정지 보행자를 5초 동안 발행하고, route UUID 유지 여부·궤적 끝점 이동량·차량 이동량을 JSON으로 출력한다. 앞서 `compare_runtime_probe.py`로 초기 위치 `(3648.0, 73505.0)`와 목표 `(3632.5, 73628.0)`을 설정할 수 있다.

```sh
docker exec -i planning-aa-control bash -c \
  'source /opt/ros/humble/setup.bash && source /opt/autoware/setup.bash && python3 -u - --x 3645.6 --y 73545.0 --duration 5' \
  < adaptive/static_avoidance_runtime_probe.py
```

2026-10-19 첫 실행에서는 보행자 정지 판정 후 `StaticAvoidance`가 활성화됐고 route UUID가 유지됐으며, 201개 제어 궤적을 수신했다. 정지 차량 앞 약 30 m까지 발행되는 국소 궤적 끝점은 기본 경로보다 최대 0.182 m 옆으로 이동했다. 보행자는 초기 차량 위치에서 약 40 m 앞에 있어 이 시점에는 회피 경로 전체가 국소 궤적 창에 들어오지 않았다. 초기 위치 `(3648.0, 73505.0)`에서는 Control 종방향 명령이 0 m/s, 기어 명령이 PARK로 남아 출발하지 않았다.

초기 위치를 같은 차로 중심에 가까운 `(3648.46, 73505.0)`으로 바꾼 다음 같은 보행자를 계속 발행하고 Autonomous 모드를 켠 실행에서는 차량이 출발해 보행자 위치를 통과했다. 재실행한 검사 스크립트는 route UUID 유지, 제어 궤적 895개 수신, 차량 이동 `64.935 m`, 보행자 중심까지 최소 거리 `2.449 m`, 보행자 위치 통과를 기록했다. `planning-aa.log`에서 회피 활성화와 지속적인 최종 궤적 송신을 확인했다. 보행자 중심 `(3645.6, 73545.0)`을 지나는 시각의 차량 중심은 `(3643.123, 73545.065)`였고 속도는 `4.164 m/s`였다. 이 결과는 샘플 지도와 이 초기 위치·보행자 배치에서의 종단 주행 확인이다.

2026-10-21에는 원본 스택과 AA 스택을 각각 새로 시작해 위 좌표에서 35초간 보행자를 발행하고 Autonomous 모드로 실제 주행시켰다. 두 스택 모두 route UUID를 유지하며 보행자 위치를 통과했다. 원본의 이동 거리·보행자 중심 최소 거리는 `116.483 m`·`2.408 m`, AA는 `116.605 m`·`2.365 m`였다. 보행자의 Y 좌표 `73545.0`을 지나는 순간의 차량 중심 X 좌표·속도는 원본 `3643.174 m`·`4.166 m/s`, AA `3643.207 m`·`4.163 m/s`였다. 보행자 전후 Y 좌표 `73540`–`73550` 구간의 최저 속도는 각각 `4.152`·`4.151 m/s`로 정지하지 않았다. 두 주행 기록을 Y 좌표 기준으로 보간하면 보행자 전후 20 m 구간의 최대 가로 위치 차이는 `0.072 m`, 최대 속도 차이는 `0.006 m/s`였다. 기록은 `output/katech-logs/20261021_153948_426724/drive-cross-validation/`에 있다. `static_avoidance_runtime_probe.py`의 `--engage-autonomous` 옵션으로 이 시험을 재현할 수 있다.

2026-10-22에는 README 7.1.3의 RViz `2D Dummy Pedestrian` 입력 경로를 재현했다. 차량 `(3648.46, 73505.0)`, 목표 `(3632.5, 73628.0)`, 보행자 `(3645.6, 73545.0)`, 차량 초기 yaw `1.697865 rad`에서 `set-pedestrian-scenario.sh`로 더미 보행자를 생성했다. 보행자가 인식되고 정적 회피가 활성화됐으며, Autonomous 주행 중 route UUID가 유지됐다. 차량은 보행자 Y 좌표를 `4.167 m/s`로 지나갔고 보행자 중심 최소 거리는 `2.407 m`, 인근 최저 속도는 `4.154 m/s`였다. 주행 기록은 `output/katech-logs/20261022_165922_1767613/docs_scenario_drive.json`에 있다. 원본 README의 스크린샷에는 지도 좌표 메타데이터가 없어 촬영 당시 클릭 좌표 자체를 복원한 결과는 아니다. 이 좌표는 같은 정상 판정 조건을 실측으로 재현한다.

2026-10-19 13:40 원본 `katech-autoware-new`의 Decision·Perception·Control 스택에서도 **같은 초기 위치 `(3648.46, 73505.0)`, 목표 `(3632.5, 73628.0)`, 정지 보행자 `(3645.6, 73545.0)`**를 시험했다. 6초 동안 route UUID가 유지됐고, 보행자 부근 행동 경로는 기본 경로보다 최대 `0.203 m`, 최종 궤적 끝점은 `0.188 m` 옆으로 이동했다. 이어 동일한 입력을 AA에 주입한 실행에서 `selected=1 candidates=1`, `PLAA_AVOIDANCE active=1`이 기록됐고 최종 궤적 끝점 이동량은 `0.181 m`였다. 이 수치는 정지 차량에서 국소 궤적 창에 들어온 회피 시작 부분만 비교한다.

13:35의 별도 RViz 배치에서는 인식 객체가 AA에 들어와 `selected=1`까지 갔지만 `candidates=0`이므로 회피가 활성화되지 않았다. 당시 로그에는 객체의 x좌표 약 `3761.5`만 남았고 y좌표와 후보 거부 단계가 없어, 이 배치의 정확한 실패 조건은 판정할 수 없다. 후속 실행에서는 `PLANNING_AA_AVOIDANCE_DEBUG=1`을 켜면 거부 단계를 볼 수 있고, `PLAA_AVOIDANCE_INPUT`에는 첫 객체의 x·y·분류·속도도 남긴다.

14:07 RViz 수동 배치에서는 차량 초기 위치 `(3730.405, 73677.742)`, 목표 `(3795.017, 73711.516)`, 인식된 보행자 `(3759.44, 73694.09)`였다. AA는 보행자를 정적 회피 대상으로 선택했지만 `candidates=0`이었다. 같은 좌표에 정지 보행자를 주입해 재현한 결과 `geometry_margin shoulder=1.508 required=2.696`으로, 보행자에서 회피 반대쪽 주행 가능 경계까지의 폭이 차량 폭·보행자 안전 여유·경계 여유의 합보다 작았다. AA의 최종 궤적은 보행자 앞에서 멈췄고 route UUID는 유지됐다. 원본 KATECH Decision·Perception·Control에도 같은 초기 위치·목표·보행자를 주입했으며, 원본도 행동 경로와 최종 궤적의 측방 이동 없이 보행자 앞에서 멈췄다. 이 배치는 원본에서도 회피 가능한 위치가 아니다. 이 조건을 통과시키려고 안전 여유를 줄이지 않는다. 성공한 재현 위치는 위의 `(3648.46, 73505.0)` 출발·`(3632.5, 73628.0)` 목표·`(3645.6, 73545.0)` 보행자 조합을 사용한다.

이미지 기본 태그는 `katech-autoware-system-control:adsw-perception` 및 `katech-autoware-system-control:adsw-control`이다. 두 이미지는 `../katech-autoware/katech-autosare-new`에서 `./partition/partition_build.sh --repo katech-autoware-system-control --no-cuda --skip-vcs-update`로 빌드한다. Control 이미지에는 이동된 System이 포함된다. `KATECH_IMAGE_REPO` 또는 개별 `KATECH_PERCEPTION_IMAGE`, `KATECH_CONTROL_IMAGE`로 태그를 바꿀 수 있다. 브리지는 동일한 ROS 도메인과 `rmw_cyclonedds_cpp`를 사용한다. 지도 디렉터리는 perception·control 컨테이너의 `/autoware_map`에 읽기 전용으로 탑재한다. 스크립트는 이미지의 존재를 먼저 확인하며 이미지를 내려받지는 않는다.

| 브리지 경계 | ROS 2 이름 | 제공/소비 |
| --- | --- | --- |
| 4308 map_vectormap | `/map/vector_map` | perception의 map loader → PlanningAA |
| 4308 localization_kinematicstate, localization_acceleration | `/localization/kinematic_state`, `/localization/acceleration` | perception localization → PlanningAA |
| 4308 objects, pointcloud, occupancy grid, traffic signals | `/perception/object_recognition/objects`, `/perception/obstacle_segmentation/pointcloud`, `/perception/occupancy_grid_map/map`, `/perception/traffic_light_recognition/traffic_signals` | perception → PlanningAA; 점군·격자는 현재 프레임에 미반영 |
| 4308 operation mode | `/system/operation_mode/state` | control의 operation mode transition manager → PlanningAA |
| 4308 clock | `/clock` | simulation clock → PlanningAA |
| 4308 velocity limit, lateral offset | `/planning/scenario_planning/max_velocity_default`, `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/input/lateral_offset` | 별도 발행자가 있어야 유입; 필수 입력 아님 |
| 1817 route methods | `/planning/mission_planning/route_selector/main/{clear_route,set_lanelet_route,set_waypoint_route}` | 외부 경로 요청 → PlanningAA; 기존 `mission_planner/*` 별칭도 유지 |
| 2317 trajectory event | `/planning/scenario_planning/trajectory` | PlanningAA → control |
| 2516 selected trajectory event | `/planning/scenario_planning/scenario_selector/trajectory` | PlanningAA → bridge ROS |
| 2340 evaluator event | `/planning/planning_evaluator/metrics` | PlanningAA → bridge ROS |
| 2341 remaining distance event | `/planning/mission_remaining_distance_time` | PlanningAA → bridge ROS |
| 2342 turn command event | `/planning/turn_indicators_cmd` | PlanningAA → bridge ROS |
| 2343 hazard command event | `/planning/hazard_lights_cmd` | PlanningAA → bridge ROS |

Control의 operation mode 노드는 실행되지만 실제 모드와 제어 허용 여부는 서비스 명령·시뮬레이터 상태에 따른다. Perception·control만 실행하면 route 요청과 `/planning/mission_planning/route` 발행자는 없다. 경로 요청을 별도로 보내야 PlanningAA가 궤적을 생성한다.

2026-10-19에 원본 `partition/run_partitions.sh --headless`와 AA `run-katech-stack.sh`를 각각 실행하고 같은 차량 초기 위치 `(3648.0, 73505.0, 19.0)`와 목표 `(3632.5, 73628.0, 19.0)`를 `/initialpose3d` 및 main waypoint route 서비스에 보냈다. 두 실행 모두 route 요청이 성공했고, 출력 route의 lanelet 구간 `102 → 87`, 시작·목표 자세, 최종 trajectory의 점 수(171점)와 `time_from_start`가 일치했다. 수집한 trajectory 3건의 대응 점에서 최대 위치 차이는 `9.5e-6 m`, quaternion 성분 차이는 `7.6e-7`, 종방향 속도 차이는 `2.4e-7 m/s`였다. 비교에는 `compare_runtime_probe.py`와 `compare_runtime_results.py`를 사용했다. 매 실행에서 생성되는 route UUID와 header 시각은 비교 대상에서 제외했다.

2026-10-19에 같은 초기 위치와 목표로 추가 출력을 재검증했다. 원본과 AA 모두 선택 궤적 259점, 방향지시등 0, 비상등 1을 발행했다. 남은 거리는 원본 `123.97266581346074 m`, AA `123.97266581345065 m`였고 남은 시간은 각각 `29.72965552644317 s`, `29.72965607037186 s`였다. 최종 궤적 171점의 대응 위치 최대 차이는 `1.63e-9 m`, 종방향 속도 차이는 0이었다. 평가 지표의 차로 ID, 차로 내 거리·횡방향 거리, 차량 속도·가속도·저크도 일치했고 궤적 통계는 수치상 근접했다. 원본의 `modified_goal`은 이 경로에서 발행되지 않았다.

이 한 경로만으로 decision 전체 대체를 증명할 수는 없다. 초기 평가 지표 표본에서는 원본이 42개, AA가 54개 항목을 냈지만 충분히 기다린 뒤에는 양쪽 모두 54개 항목을 발행했다. AA의 방향지시등 판단은 원본의 교차로 차로 속성, 경로 변경 및 후진 출발 규칙을 기반으로 하지만 원본의 모든 behavior 모듈 간 우선순위와 시점 조건을 아직 재현하지 않는다. 차로 변경, 회피, 주차, 실제 주행 신호에서 별도 검증이 필요하다.

`oss-ap-planning-bridge`는 Service Creator만 빌드하고 이전 planning 앱의 실행·서비스 매니페스트도 배포에서 제외한다. `build-bridge-image.sh`는 이 전용 이미지를 입력으로 받아 매니페스트 수를 확인하는 선택 단계다. `build-image.sh`는 `planning-aa-adaptive:local`을 만든다. `run-stack.sh`는 `planning-aa-net`(`192.168.100.0/24`)을 만들고 브리지에 `.92`, PlanningAA에 `.93`을 할당한다. 두 컨테이너의 `/dev/shm`은 기본 2 GiB다. PlanningAA 쪽은 `PLANNING_AA_SHM_SIZE`로 조절할 수 있다. PlanningAA가 종료되면 스크립트가 브리지 컨테이너도 정지한다.

이미지 태그는 `OSS_AP_IMAGE`, `BRIDGE_IMAGE`, `IMAGE_TAG`로 지정할 수 있다. ROS 2 도메인은 `ROS_DOMAIN_ID`(기본 `0`)로 전달한다. `RMW_IMPLEMENTATION`은 상대 Autoware와 맞춰야 한다. 네트워크 이름은 `AP_NET`, PlanningAA 주소는 `PLANNING_AA_IP`로 지정할 수 있지만, 현재 CM 매니페스트의 PlanningAA 주소는 `.93`으로 생성되므로 주소를 바꿀 때 매니페스트도 함께 바꿔야 한다.

Docker 없이 컴파일·설치만 확인하려면 저장소 루트에서 실행한다.

```sh
cmake -S adaptive -B build-adaptive -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$(pwd)/output/adaptive"
cmake --build build-adaptive -j 4
cmake --install build-adaptive --component Standalone
```

## 파일 구성

| 파일 | 역할 |
| --- | --- |
| `CMakeLists.txt` | `vendor/`에서 3개 포트와 CM 설정을 빌드 디렉터리에 준비하고 PlanningAA를 빌드 |
| `vendor/` | 로컬 빌드용 PARA SDK와 필요한 생성 ARA 파일. Git 제외 |
| `app/main.cpp`, `app/trajectory_output.*` | `ara::com` 입력 구독, route Method 제공, 검증된 최종 궤적의 Event 변환·송신 함수, EM 상태 보고 |
| `app/manifest/`, `platform/` | 앱 실행·EM·머신·기능 그룹 매니페스트 |
| `Dockerfile`, `build-image.sh`, `entrypoint.sh` | PlanningAA 실행 이미지 빌드·시작 |
| `Dockerfile.bridge`, `build-bridge-image.sh` | 브릿지 전용 이미지의 매니페스트 검증과 파생 태그 생성 (선택) |
| `run-image.sh`, `run-stack.sh` | PlanningAA 단독 또는 브리지와 함께 실행 |

## 현재 제한

점군·점유 격자·lateral offset은 여전히 최신 샘플만 보관하며 이 주행 프레임에 반영하지 않는다. Parking 경로는 연결되지 않았다. 지도·점군·점유 격자처럼 큰 UDP Event는 기존 `oss-ap` CM의 기본 `max-data-bytes=0` 설정에서 전송 실패할 수 있다. 대형 입력의 전송 방식과 실제 ROS2↔SOME/IP↔PlanningAA 종단 동작은 아직 검증되지 않았다.

Event 변환은 validator가 이번 프레임을 유효하다고 판정한 궤적에만 적용한다. 내부 `TrajectoryPoint`에는 `time_from_start`가 없으며, 원본 planning_validator의 출력과 동일하게 이 필드를 0으로 발행한다.
