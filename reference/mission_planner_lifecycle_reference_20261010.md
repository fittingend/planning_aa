# Mission planner 설정·해제 상태 계약 원본 대조 (2026-10-10)

기준 이미지의 `MissionPlanner`를 K-City HPC S자 지도·차량 자세와 함께 실행했다. `SetWaypointRoute` 성공 뒤 `ClearRoute`를 호출하고 durable route/state topic을 관측했다. [수집 결과](mission_planner_lifecycle_scurve_reference.json)는 처음 준비 후 `UNSET(2) → ROUTING(3) → SET(4) → UNSET(2)`을 보이며, route 메시지는 설정 시 한 번만 발행됐다. 설정과 해제 서비스의 `status.success=true`, `code=0`이었다. 해제 시 route 빈 메시지를 재발행하지 않고 내부 경로를 지우는 원본 코드의 동작과 일치한다.

성공한 경로 설정 직후 지도 밖 목표로 다시 설정하는 실패 사례도 확인했다. 운행 모드 메시지가 없으면 [서비스 코드 2](mission_planner_rejected_after_set_scurve_reference.json)로 거절하고 route/state 발행은 늘지 않았다. 합성 STOP 모드를 먼저 발행하면 [서비스 코드 3](mission_planner_rejected_after_set_stop_mode_scurve_reference.json)으로 실패하며 `SET(4) → REROUTING(5) → SET(4)` 상태가 추가되고 기존 route 메시지 1개와 목표 좌표는 유지된다. 이어 경로 해제는 성공하여 `UNSET(2)`으로 돌아갔다. 실패 응답 코드와 상태 전이는 `mission_planner_core` CTest에서도 검사한다. 성공 설정·해제 사건은 `UNSET → ROUTING → route → SET → UNSET` 순서로 검사한다. 이 사례는 지도 밖 목표와 정지 운행 모드를 **합성한 노드 시험**이다. ROS 서비스 서버·topic 인코딩·QoS, 자율주행 중 재경로의 비동기 시각, TF 오류는 아직 독립 실행기로 검증하지 않았다.

```sh
docker run --rm --network host --ipc host \
  -v /home/sujin/Documents/01_GIT/01_OSS/planning-aa:/probe \
  -v /home/sujin/Documents/01_GIT/01_OSS/katech-autoware/katech-autoware:/source \
  -v /home/sujin/Documents/01_GIT/01_OSS/251231_KCity:/bags \
  autoware-partition:sample-adsw-decision \
  python3 /probe/reference/mission_planner_reference_probe.py \
  --bag /bags/hpc/rosbag_scurve/rosbag_20251231_022311_0.db3 \
  --output /probe/reference/mission_planner_lifecycle_scurve_reference.json \
  --clear-after-set
```
