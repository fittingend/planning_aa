# 장애물 순항 원본 이미지 재실행 대조 (2026-10-10)

`autoware-partition:sample-adsw-decision`의 `ObstacleCruisePlannerNode`를 실제 HPC S자 기록의 궤적·odometry·가속도·예측 객체로 실행했다. 합성 `/clock`은 각 궤적 수신 시각보다 5ms 뒤로 맞췄다. 시험기 `obstacle_cruise_reference_probe.py`는 입력 상태를 먼저 발행하고, 노드의 정지 디버그와 객체 표식을 수집한다. 프레임 번호는 `obstacle_cruise_projection_rosbag_replay.py`처럼 **초기 객체·odometry가 준비되기 전 궤적 19개를 제외한 번호**다.

기준 launch 설정의 순항→정지/정지→순항 객체 속도 임계값은 1.0/1.5m/s이고, 패키지 자체 YAML은 3.0/3.5m/s다. 478번 프레임에서 launch 설정의 노드는 같은 객체를 `obstacles_to_cruise`로 표기하고 정지를 출력하지 않았다. 속도 임계값을 3.0/3.5m/s로 바꾸자 이 객체는 순항에서 제외됐지만, 미래 위치가 통로를 벗어난다고 판정해 `intentionally_ignored_obstacles`로 표기하고 여전히 정지하지 않았다. 패키지의 충돌 시간 여유 4.0초와 안전 거리 5.0m까지 적용해도 같은 결과였다.

패키지 쪽 네 수치를 적용해 450–482번을 **한 노드에서 연속 실행**하면 현재 이미지의 정지는 28/33프레임, 기록의 정지는 33/33프레임이다. 현재 이미지는 475–477번에 정지(20.79, 20.59, 20.88m)를 출력하다 478–481번에 `intentionally_ignored_obstacles`로 전환한다. 기록은 478번에도 정지 디버그 20.497m와 객체 정지 표식을 남겼다. 482번 현재 이미지의 표식에는 감속 후보와 미래 통과 객체가 함께 보인다. 독립 재생의 정지 누락과 감속 추가가 시작되는 경계와 같다.

같은 설정으로 450–649번 **200프레임**까지 연속 재실행한 [결과](obstacle_cruise_scurve_valid450_650_package_stop.json)를 기록의 다음 50ms 이내 디버그 출력과 [비교](obstacle_cruise_live_reference_compare.py)했다. 정지는 공동 활성 28, 기록만 활성 **58(478–535번)**, 현재 이미지 재실행만 활성 0, 공동 비활성 114프레임이다. 기록만 활성인 58프레임의 번호가 독립 계산의 58개 정지 누락과 같다. 현재 이미지의 실제 원본 노드도 같은 경계에서 정지하지 않는다는 강한 근거다. 그러나 이 재실행이 기록 당시의 실행 파일·파라미터·polling 콜백을 재현했다는 근거는 아니다.

감속은 이 200프레임에서 기록만 활성 42(531–572번), 현재 이미지 재실행만 활성 36(482–517번), 공동 활성 0이었다. 재실행 감속 활성은 marker의 `obstacles_to_slow_down`, 기록 활성은 `slow_down_planning_info` 배열 길이 7 이상을 기준으로 했다. 독립 계산을 재실행과 같은 sample 차량 외형(앞 3.79m·뒤 1.10m·폭 1.896m)으로 앞 650프레임부터 연속 실행하면 기록 활성 42개를 모두 포함하고 482–530번에 49개를 더 선택한다. **감속 분기의 현재 이미지·독립 계산·기록 사이에는 별도 차이**가 남는다. 재실행의 선택 객체와 속도 수치를 동일하다고 주장하지 않는다.

독립 재생도 `--start-frame 450 --frames 200 --stop-vehicle-shape sample`로 450번에서 상태를 새로 시작해 보았다. 정지 활성 28, 원본만 활성 58은 현재 이미지와 같지만 감속은 여전히 독립 91/기록 42다. 따라서 **450번의 상태 초기화만으로** 현재 이미지의 감속 36프레임과 독립 91프레임 차이를 설명하지 못한다. 독립 시험기는 순항·정지·감속의 전체 ROS callback 실행과 동일하지 않다.

현재 이미지 marker를 구간별로 보면 478–481번은 미래 통과 객체, 482–517번은 감속 객체와 미래 통과 객체가 함께 표시되고, 518–562번의 45프레임은 `obstacles_to_cruise`로 바뀐다. 기록의 감속 531–572번과 겹치는 531–562번에서 현재 이미지가 객체를 순항으로 분류하는 것이 감속 부재에 관여한다. 563–572번의 감속 부재까지 이 표식만으로 설명할 수는 없다.

기준 decision launch의 값(순항→정지 1.0m/s, 정지→순항 1.5m/s, 충돌 여유 1초, 안전 거리 4m)만 사용해 같은 450–649번을 [별도로 연속 실행](obstacle_cruise_scurve_valid450_650_launch.json)하면 정지·감속 활성은 **둘 다 0/200**이고 순항 객체 표식은 112프레임이다. 패키지 쪽 설정을 적용한 28개 정지도 사라진다. 따라서 이 기록의 정지 86프레임을 현재 이미지의 기준 launch 설정만으로 설명할 수 없다. 기록 당시의 실행 조건과 코드 버전을 확인하지 않고 독립 코어를 기록에 맞추는 것은 근거가 부족하다.

현재 이미지의 정지 디버그 타입은 `autoware_internal_debug_msgs/msg/Float32MultiArrayStamped`, 보유 rosbag의 타입은 `tier4_debug_msgs/msg/Float32MultiArrayStamped`다. 또한 기록의 정지 여유는 6.0m이고 기준 launch의 안전 거리 설정은 4.0m다. 실행 버전 또는 파라미터·콜백 입력이 기록 시점과 같지 않다는 근거지만, 정확히 어느 차이가 위 정지 분기를 바꿨는지는 아직 확정되지 않았다. 현재 이미지의 단일/연속 재실행을 기록 당시 노드의 동일 실행으로 취급하지 않는다.

재현 예시:

```sh
docker run --rm --network host --ipc host \
  -v /home/sujin/Documents/01_GIT/01_OSS/planning-aa:/probe \
  -v /home/sujin/Documents/01_GIT/01_OSS/katech-autoware/katech-autoware:/source \
  -v /home/sujin/Documents/01_GIT/01_OSS/251231_KCity:/bags \
  autoware-partition:sample-adsw-decision \
  python3 /probe/reference/obstacle_cruise_reference_probe.py \
  --bag /bags/hpc/rosbag_scurve/rosbag_20251231_022311_0.db3 \
  --output /probe/reference/obstacle_cruise_scurve_valid450_483_package_stop.json \
  --start 450 --end 483 --cruise-to-stop-velocity 3.0 \
  --stop-to-cruise-velocity 3.5 --collision-time-margin 4.0 \
  --safe-distance-margin 5.0
```

이 시험기는 각 프레임의 직전 객체·차량 상태를 사용하므로 기록 당시 내부 polling callback의 정확한 입력을 증명하지 않는다. 기록된 노드 실행 파일과 파라미터 덤프가 없으면 해당 분기의 점별 동등성 완료 판정은 보류한다.

200프레임 수치 비교:

```sh
python3 reference/obstacle_cruise_live_reference_compare.py \
  /home/sujin/Documents/01_GIT/01_OSS/251231_KCity/hpc/rosbag_scurve/rosbag_20251231_022311_0.db3 \
  reference/obstacle_cruise_scurve_valid450_650_package_stop.json
```
