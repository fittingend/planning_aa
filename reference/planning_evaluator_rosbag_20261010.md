# Planning evaluator 원본 출력 대조 (2026-10-10)

IPC 정지선 기록의 `/planning/planning_evaluator/metrics`는 584개 메시지이며, 기록 당시 형식은 `diagnostic_msgs/msg/DiagnosticArray`다. 현재 체크인된 `autoware_planning_evaluator` 소스는 100ms 타이머에서 `tier4_metric_msgs/msg/MetricArray`를 발행한다. 따라서 메시지 종류와 값 인코딩은 버전별 계약으로 다뤄야 한다.

[현재 기준 이미지의 실제 노드 graph](planning_evaluator_current_image_graph_reference.json)에서도 `~/metrics`는 `tier4_metric_msgs/msg/MetricArray`, `~/debug/processing_time_ms`는 `autoware_internal_debug_msgs/msg/Float64Stamped`였다. 두 publisher 모두 `VOLATILE`·`RELIABLE`이다. 입력 없이 1.2초 관측하면 빈 MetricArray가 12개 발행됐고 간격 중앙값은 99.82ms였다. 따라서 기록의 `DiagnosticArray`와 현재 이미지의 `MetricArray` 차이는 소스 표기만의 추측이 아니라 실제 두 실행 환경에서 확인됐다. 첫 graph 관측은 depth를 0으로 보고했지만 합성 입력 재실행의 graph, 현재 소스 생성 인자와 [기록 metadata](planning_output_bag_inventory_20261010.md)는 1이므로 depth 0을 큐 깊이 계약으로 해석하지 않는다. 독립 발행 어댑터의 버전별 직렬화는 아직 없다.

[현재 이미지 합성 trajectory 실행](planning_evaluator_current_image_synthetic_reference.json)에는 `map` frame의 0m·1m·2m 직선 세 점과 속도 1·1·0m/s, 정지 odometry를 입력했다. 첫 유효 MetricArray는 27개 항목이며 `length=2m`, `point_interval=1m`, `duration=2s`, `velocity/{min,max,mean}=0/1/0.6666667m/s`, 객체가 없는 `obstacle_distance=DBL_MAX`를 담았다. 이어지는 타이머 출력은 39개 항목으로, 이전 trajectory와의 국소·선행 거리, 안정성, Fréchet 거리 지표 네 종류가 모두 0이다. 독립 `PlanningEvaluatorCore`는 정차 시 평가 거리 0도 원본처럼 이전 경로의 선행 점들을 사용하도록 고쳤다. 같은 입력 두 번의 지표 종류는 원본과 27개→39개로 일치하며 기본 수치·추가 지표 0을 `planning_evaluator_core` CTest에 넣었다. 이 사례는 합성 입력의 선택 지표만 확인하며 현재 이미지의 모든 지표·메시지 인코딩 전체 동등성을 뜻하지 않는다.

[정차 상태에서 이전 경로를 유지하다 측방 1m 옮긴 사례](planning_evaluator_current_image_shifted_reference.json)도 원본 타이머에서 네 이전 경로 지표의 최소·최대·평균이 모두 1m였다. 독립 코어에 동일한 세 점 이동을 넣은 `planning_evaluator_core` CTest도 네 지표가 모두 1m로 일치한다. 정차 시 평가 거리 0의 분기를 단순한 영값 사례뿐 아니라 경로 변경 사례에서도 확인했다.

[속도 2m/s·가속도 0.5m/s² 합성 입력](planning_evaluator_current_image_kinematic_reference.json)에서는 원본 첫 출력이 30개 항목이고 `kinematic_state/{vel,acc,jerk}=2,0.5,0`이었다. 같은 2m 길이 경로를 다시 평가한 출력은 39개 항목이며, 5초 선행 평가에 필요한 10m 길이가 부족해 `lateral_trajectory_displacement_lookahead`는 빠졌다. 독립 코어도 두 출력의 항목 수와 이 세 수치, 짧은 경로의 선행 지표 생략이 일치한다. 이 시험은 합성 입력이며 실제 기록의 acceleration callback 시각을 복원한 것은 아니다.

[수정 목표를 자차에서 전방 3m·좌측 4m에 둔 합성 입력](planning_evaluator_current_image_modified_goal_reference.json)에서 원본의 목표 종·횡·방향 편차는 각각 3m·4m·0rad였다. 첫 출력은 기본 27개에 목표 지표 9개를 더한 36개, 두 번째는 이전 경로 지표까지 48개다. 독립 `PlanningEvaluatorCore`도 같은 세 수치와 36→48개 항목을 만든다. 보유 K-City 기록에는 modified goal 메시지가 없으므로 이 검증은 합성 활성 사례로 분리한다.

[정적 객체 중심을 경로 X=1.1m에 둔 합성 입력](planning_evaluator_current_image_obstacle_reference.json)에서는 원본의 세 경로점 객체 거리 평균이 0.7m이고 첫 충돌 예상 시간은 1s였다. 첫·둘째 출력의 항목 수는 30→42개다. 독립 코어도 이 거리·시간과 항목 수가 일치한다. 이 사례는 한 객체 중심의 거리 지표를 확인하며 객체 외형이나 움직임을 고려하는 검증은 아니다. 현재 원본 지표 계산도 객체 중심을 사용한다.

[K-City HPC S자 기록의 실제 지도·route·odometry를 현재 이미지 노드에 입력한 사례](planning_evaluator_current_image_lane_graph_reference.json)는 trajectory 없이 `ego_lane_info/{lane_id,s,t}` 세 항목을 약 10Hz로 발행했다. 첫 값은 차로 ID 29, 차로 내 거리 5.662072m, 측방 거리 0.146032m이다. [독립 지도 어댑터 재생](../tests/planning_evaluator_lane_reference_replay.py)도 세 값이 모두 원본과 0.001m 이내로 일치했고 `planning_evaluator_scurve_lane_reference_replay` CTest로 등록했다. 입력은 실제 기록이지만 현재 이미지 원본 노드에 단독 재발행한 사례이므로 당시 전체 실행의 callback 짝과는 구분한다.

같은 지도·route에서 자차 X 좌표만 합성으로 [−3m](planning_evaluator_current_image_lane_xminus3_reference.json)·[+3m](planning_evaluator_current_image_lane_xplus3_reference.json) 옮기면 원본의 측방 거리는 각각 +3.136058m·−2.843995m, 호길이는 5.906491m·5.417654m였다. 독립 지도 어댑터도 두 부호와 수치가 0.001m 이내로 일치했다. 두 오프셋은 차로 ID 29로 계속 계산되며 갓길이나 다른 차로 선택의 일반화는 여전히 미검증이다.

기록의 출력 시각 직전 수신된 최종 trajectory·기준 path optimizer trajectory·객체 중심·odometry를 독립 `PlanningEvaluatorCore`에 시간순으로 넣었다. 2점 이상의 trajectory가 있는 573프레임에서 `length`, `point_interval`, `velocity`, `acceleration`, `jerk`, `duration`, `curvature`, `relative_angle`, `velocity_deviation`, `obstacle_distance`는 각각 571프레임, `lateral_deviation`과 `yaw_deviation`은 각각 570프레임의 최소·최대·평균이 1e-6 이내다. 기본 궤적 지표의 두 불일치 프레임(비교 인덱스 494, 498)의 원본 길이는 각각 출력 0.847ms·0.510ms 전에 기록된 최신 trajectory의 **바로 이전** trajectory 길이와 정확히 같다. 이 프레임들의 최대 길이 차이는 0.50008135m, 최대 duration 차이는 0.264814907s다. 추가 편차 불일치 435번은 기준 trajectory가 출력 0.492ms 전에 기록됐다. 일률적으로 1ms 전의 입력을 택하면 다른 프레임이 틀리므로 고정 지연 규칙으로 원본 callback 짝을 완전히 복원할 수 없다.

추가로 기록의 `DiagnosticArray`에는 음수만 누적한 614개 지표의 `max`가 `DBL_MIN`(2.2250738585072014e-308)으로 남아 있다. 현재 소스의 Accumulator와 독립 코어는 실제 음수 최대값을 사용한다. 재생 검사는 이 614개 값만 구버전 초기값 차이로 따로 계수하고 수치 불일치 판정에서 제외한다. 계산 코어를 기록의 잘못된 최대값에 맞추지 않았다.

재현:

```bash
python3 tests/planning_evaluator_rosbag_replay.py \
  /tmp/planning-aa-map-build/planning_evaluator_rosbag_runner \
  /home/sujin/Documents/01_GIT/01_OSS/251231_KCity/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3 \
  --maximum-basic-mismatches 2 --maximum-extended-mismatches 3
```

이 검사는 trajectory·기준 궤적·객체 거리 통계 12종만 확인한다. 안정성, 차로 지표, scalar 값과 처리 시간, 출력 메시지 직렬화의 원본 동등성은 별도 검증이 필요하다.
