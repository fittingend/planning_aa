# 남은 거리·시간 활성 출력 대조 (2026-10-10)

기록된 `/map/vector_map`의 LaneletMapBin 내부 자료, `/planning/mission_planning/route`의 목표, 타이머 출력 직전의 odometry와 `/planning/scenario_planning/current_max_velocity`를 `RemainingDistanceMapAdapter`에 전달했다. 비교 대상은 원본 `/planning/mission_remaining_distance_time`이 발행된 LaneDriving 프레임이다. 원본 출력을 독립 계산의 입력으로 사용하지 않았다.

| 기록 | 비교 프레임 | 거리 차이 중앙값/최대 | 1mm 이내 | 1cm 이내 | 시간 차이 중앙값/최대 |
| --- | ---: | ---: | ---: | ---: | ---: |
| IPC 정지선 | 571 | 0.000000/0.076217m | 455 | 569 | 0.000000/0.006860s |
| HPC S자 | 755 | 0.000000/0.025232m | 754 | 754 | 0.000000/0.002271s |

IPC 정지선의 최대 차이 434번에서 원본 거리는 29.234999612741106m, 출력 시각 직전 odometry로 계산한 거리는 29.1587825740333m다. **한 단계 이전 odometry**(출력 수신 시각보다 22.12ms 이전)로 다시 계산하면 29.235m가 된다. HPC S자의 최대 차이 207번도 같은 방식으로 원본 198.8462689471986m를 재현한다(한 단계 이전 odometry의 수신 시각은 출력보다 36.212ms 이전). 이 두 사례는 계산식 차이보다 원본 노드가 사용한 odometry callback 시각과 rosbag 수신 순서의 차이를 시사한다. 전체 프레임에서 원본과 동일한 callback 입력을 복원했다고 주장하지 않는다.

재현 절차:

```bash
cmake --build /tmp/planning-aa-map-build --target remaining_distance_rosbag_runner
python3 tests/remaining_distance_rosbag_replay.py \
  /tmp/planning-aa-map-build/remaining_distance_rosbag_runner \
  - /home/sujin/Documents/01_GIT/01_OSS/251231_KCity/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3
```

`-`는 해당 rosbag에서 지도를 자동 추출한다. HPC S자도 같은 명령에 기록 경로만 바꿔 전달한다. 두 기록의 CTest 기준은 `remaining_distance_*_rosbag_replay`로 등록했다. 이 검사는 실제 원본 `MissionRemainingDistanceTime` 수치의 활성 대조다. ROS 10Hz publisher와 QoS·frame 전달의 구현 검사는 별도다.

[현재 기준 이미지 노드의 parking 발행 시험](remaining_distance_parking_graph_reference.json)은 지도·route 없이 scenario만 합성했다. scenario 입력 전 0.6초와 `LANEDRIVING` 입력 뒤 0.6초에는 출력이 0개였고, `PARKING`으로 전환한 1.2초 동안 **12개**의 거리 0m·시간 0s 출력이 발행됐다. 수신 간격 중앙값은 100.02ms다. 실제 graph의 출력 형식은 `autoware_internal_msgs/msg/MissionRemainingDistanceTime`, durability는 `VOLATILE`, reliability는 `RELIABLE`이다. graph 도구가 보고한 depth는 0인 반면 소스 publisher 생성 인자와 [기록 metadata](planning_output_bag_inventory_20261010.md)는 모두 depth 10이므로, depth 0을 큐 깊이 계약으로 해석하지 않는다. 독립 `RemainingDistanceTimeCore`의 scenario 게이트와 parking 영값 출력은 이 정책과 같다. 실제 노드 publisher 연결은 아직 없다.
