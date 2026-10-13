# Route selector main/MRM 원본 노드 전환 (2026-10-10)

기준 이미지의 `MissionPlanner`와 `RouteSelector`를 같은 component container에서 실행하고, K-City HPC S자 지도·odometry와 비자율 STOP mode를 입력했다. main waypoint 목표는 행동 경로 50번, MRM 목표는 45번이다. 원본 서비스의 세 호출(main 설정 → MRM 설정 → MRM 해제)이 모두 성공했고 응답 코드는 0이었다. [원본 결과](route_selector_scurve_reference.json)에서 main과 MRM은 초기 `INITIALIZING(1) → UNSET(2)`이며, main 설정으로 main route/`SET(4)`이 발행됐다. MRM 설정으로 main은 `INTERRUPTED(8)`, MRM은 `SET(4)`이 됐다. MRM 해제 시 저장된 main route의 **같은 UUID·목표·구간**이 다시 발행되고 MRM은 `UNSET(2)`으로 돌아갔다.

[MRM 중 main 요청을 교체한 사례](route_selector_scurve_replace_main_reference.json)도 수집했다. 새 main 요청 서비스는 즉시 성공하고 main `INTERRUPTED(8)`이 발행되지만, 그 시점에 planner의 새 route 발행은 없다. MRM 해제 시 새 main 목표(경로 55번)에 대한 planner route가 발행되고 main은 `SET(4)`, MRM은 `UNSET(2)`이 됐다. `RouteSelectorCore`는 저장된 요청과 이전 route 복구·새 목표 재계산을 구현하며 `route_selector_core` CTest가 이 두 분기를 검사한다.

[실패 MRM 선행 사례](route_selector_invalid_mrm_scurve_reference.json)에서는 main 경로를 설정한 뒤 지도 밖 목표로 MRM 경로를 요청했다. 서비스는 `success=false, code=3`을 반환했고 main route는 그대로였으며 main `INTERRUPTED`와 MRM route는 발행되지 않았다. 그 뒤 정상 MRM 설정·해제는 각각 성공해 기존 main 경로를 복구했다. 원본 planner는 실패 계산 중 `REROUTING(5) → SET(4)`을 발행했으나 이 시험에서 selector main state 구독에는 최종 `SET(4)`만 관측됐다. 경유 topic의 빠른 상태 발행을 모두 수신한다는 보장은 이 시험에 없다. `RouteSelectorCore`는 실패 응답일 때 MRM 운영 상태를 시작하지 않는 결정을 검사한다.

[실패 main 재설정 사례](route_selector_invalid_main_scurve_reference.json)에서는 main 성공 뒤 지도 밖 목표를 다시 main에 요청했다. 두 번째 응답은 `success=false, code=3`이며 원본 planner는 `REROUTING(5) → SET(4)`을 발행했다. selector의 main state는 최종 `SET(4)`을 유지했고 route topic에 새 메시지는 없었다. 원본 `RouteSelector::on_set_waypoint_route_main`은 planner 호출 전에 저장한 main route를 비우므로, 이때 내부 복구용 route는 사라지지만 이미 발행한 transient local topic의 마지막 route는 남는다. 독립 코어는 이 내부 상태와 실패 응답을 검사한다. 이 사례는 MRM을 시작하지 않아 이후 복구 경로는 검증하지 않는다.

같은 container의 원본 `GoalPoseVisualizer`는 planner route가 처음 발행될 때, MRM으로 바뀔 때, main으로 복구될 때 각각 `PoseStamped`를 한 번 발행했다. 두 시나리오 모두 매 echo의 목표 자세와 `map` frame, header stamp가 대응 route와 정확히 같다. 원본 시험기는 XYZ·quaternion 전 값과 헤더를 검사한다. 독립 `MissionRouteMapAdapter::takeGoalPoseEchoes`는 같은 헤더·목표를 보존하며 별도 `ros/goal_pose_visualizer.py`에서 원본 route 입력과 transient local echo publisher를 구현했다. 독립 mission planner·selector publisher와의 연결은 남아 있다.

[독립 echo 원본 종단 재실행](route_selector_independent_echo_scurve_reference.json)에서는 K-City HPC S자 지도·odometry를 사용해 원본 mission planner와 route selector를 실행하고 원본 `GoalPoseVisualizer` 대신 `planning-aa` ROS 노드를 구독자로 연결했다. main 설정→MRM 설정→main 복구 서비스 세 번이 모두 성공했고 planner route 세 건에 대해 echo 세 건의 목표 자세 7개 값(XYZ·quaternion), `map` frame, header stamp가 각각 정확히 같았다. 세 목표는 main `(2516.184776, 24347.656471)`, MRM `(2524.376948, 24341.940472)`, 복구 main 순서다. echo publisher의 실제 graph 형식은 `geometry_msgs/msg/PoseStamped`, QoS는 depth 1·`TRANSIENT_LOCAL`·`RELIABLE`이었다. [같은 입력의 원본 goal visualizer 재실행](route_selector_original_echo_runtime_qos_reference.json)과 직접 비교해 세 목표 자세 순서가 같고 7개 topic graph와 9개 route service graph도 정확히 일치했다. 실행마다 route header 시각은 새로 찍히며 각 실행 내부에서는 세 route의 header가 각각 echo와 정확히 같다. [대체 launch](route_selector_independent_echo_probe.launch.xml)와 시험기의 `--independent-goal-echo` 옵션으로 재현한다. 독립 mission planner·selector ROS 서버 자체의 종단 시험은 남아 있다.

[실패 MRM 선행 독립 echo 실행](route_selector_independent_echo_invalid_mrm_reference.json)에서는 main 설정 성공, 지도 밖 MRM 설정 실패 코드 3, 정상 MRM 설정, MRM 해제 순으로 호출했다. 실패 요청에는 새 planner route나 echo가 없었고 전체 planner route 세 건에 대해 독립 echo가 정확히 세 건 나왔다. 세 echo의 목표 순서는 main→MRM→복구 main이며 header도 각각 대응 route와 일치했다. 재현 시 시험기에 `--independent-goal-echo --invalid-mrm-first`를 함께 준다.

[실패 main 재설정 독립 echo 실행](route_selector_independent_echo_invalid_main_reference.json)에서도 첫 main 설정 성공 뒤 지도 밖 목표 요청은 코드 3으로 실패했고, 새 planner route나 echo가 나오지 않아 두 요청에 대한 route·echo는 각각 한 건이었다. 재현 옵션은 `--independent-goal-echo --invalid-main-after-set`이다.

정상·실패 MRM·실패 main 세 실행에서 원본 echo와 독립 echo를 같은 입력으로 각각 재실행했다. 각 topic 안에서 관측한 planner/main/MRM의 route·state 종류, 목표 자세와 상태값, echo의 목표 자세 순서는 세 사례 모두 같다. 서비스 결과와 일곱 topic·아홉 서비스 graph도 각각 일치한다. 서로 다른 topic의 콜백이 합쳐진 전체 수신 순서는 실행마다 달라질 수 있으므로 동등성 검사에 쓰지 않는다.

실행 중 ROS graph에서도 mission planner의 route/state, route selector의 main/MRM route/state, echo `PoseStamped`까지 **7개 topic**을 확인했다. 일곱 publisher 모두 durability가 `TRANSIENT_LOCAL`이다. mission planner 3개와 selector main/MRM 각 3개인 **9개 경로 서비스**의 실제 타입은 `tier4_planning_msgs/srv/{ClearRoute,SetWaypointRoute,SetLaneletRoute}`였다. 이는 `/probe` 이름으로 재배치한 원본 세 노드의 graph이며, decision 전체 graph 검증으로 확대하지 않는다.

관측 topic의 콜백 수신 순서는 service 내부 발행 순서와 다를 수 있다. 첫 사례의 MRM 해제에서는 MRM UNSET을 main SET보다 먼저 받았지만 둘째 사례에서는 나중에 받았다. 원본의 `RouteSelector::on_state/on_route`가 planner 발행을 MRM 운영 플래그에 따라 전달한다는 점과 서비스 성공 후 상태 경계만 이 검증의 동등성 근거로 삼는다. 원본 `service_utils::sync_call`은 서비스가 준비되지 않았으면 `SERVICE_UNREADY=50001`을 반환하고, 준비된 뒤에는 `future.get()`으로 제한 시간 없이 응답을 기다린다. 독립 ROS 어댑터의 이 준비 확인·응답 대기·QoS 및 여러 요청 경합은 아직 검증하지 않았다. 빈 UUID는 원본의 정적 MT19937 8비트 엔진과 같은 방식으로 생성하도록 코어를 맞췄다.

[planner 서비스 부재 실행](route_selector_unready_reference.json)에서는 원본 route selector만 기동하고 client 목적지를 존재하지 않는 `/probe/missing_planner`로 연결했다. main waypoint 설정, MRM waypoint 설정, MRM 해제 세 서비스 모두 `success=false, code=50001`을 반환했다. 응답 message는 세 번 모두 `/probe/missing_planner/set_waypoint_route`였다. 첫 main 설정 요청은 planner 전달에 실패해도 selector에 저장되므로 MRM 해제는 clear 서비스 대신 저장된 main 목표를 waypoint 서비스로 다시 보내려 한다. 독립 코어의 같은 실패 후 재시도 정책을 `route_selector_core` 단위 검사로 확인했다. [단독 launch](route_selector_unready_probe.launch.xml)와 [실행기](route_selector_unready_probe.py)가 원본 재현 절차다. 위 Docker 명령에서 마지막 실행기만 `python3 /probe/reference/route_selector_unready_probe.py`로 바꾸면 된다.

재현: [시험기](route_selector_reference_probe.py)에 같은 파일의 `--bag`, `--output`을 넣어 실행한다. 교체 사례는 `--replace-main-during-mrm 55`, 실패 사례는 `--invalid-mrm-first` 또는 `--invalid-main-after-set`을 추가한다. 시험기의 [launch](route_selector_probe.launch.xml)는 원본 두 노드의 서비스·route·state를 연결한다.

독립 echo를 연결한 재현 명령은 `planning-aa` 디렉터리에서 다음과 같다. ROS 도메인 번호는 다른 실행과 겹치지 않는 232 이하 값을 사용한다.

```sh
docker run --rm --net=host --pid=host --ipc=host \
  -e ROS_DOMAIN_ID=142 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e ROS_LOCALHOST_ONLY=0 \
  -v /home/sujin/Documents/01_GIT/01_OSS/katech-autoware/katech-autoware:/source:ro \
  -v "$(pwd):/probe" \
  -v /home/sujin/Documents/01_GIT/01_OSS/251231_KCity/hpc/rosbag_scurve:/bag:ro \
  autoware-partition:sample-adsw-decision \
  python3 /probe/reference/route_selector_reference_probe.py \
  --bag /bag/rosbag_20251231_022311_0.db3 \
  --output /probe/reference/route_selector_independent_echo_scurve_reference.json \
  --independent-goal-echo
```
