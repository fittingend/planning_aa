# 장애물 순항 콜백 입력 시각과 현재 원본 노드 대조 (2026-10-11)

## 실제 입력 경계

기준 `motion_planning.launch.xml`은 순항 노드의 `~/input/trajectory`를 `motion_velocity_planner/trajectory`에 연결한다. `path_optimizer/trajectory`는 한 단계 앞 출력이다. 원본 `ObstacleCruisePlannerNode::onTrajectory`는 콜백 시작에서 odometry·예측 객체·가속도를 `takeData()`로 가져온 뒤 정지, 순항, 감속을 순서대로 계산한다. 순항 디버그 발행은 계산이 끝난 뒤다. HPC cutinout의 행동 경로 수신→동작 속도 궤적 수신 중앙 지연은 약 59ms이고, 동작 속도 궤적 수신→순항 디버그 발행 중앙 지연은 약 5ms다. HPC blocking은 각각 약 32ms·3ms, IPC cutinout은 약 49ms·4ms다.

`PlanningPipelineFrame::CruiseCallback`과 선택적 `PlanningCruisePerceptionFrame`을 추가해 순항 후보·감속·정지·PID가 행동 경로 단계와 다른 차량 자세/속도·가속도·객체·시각을 사용하도록 연결했다. 원본 호출 순서에 맞춰 먼저 정지 궤적을 만들고 순항 제한을 계산한 뒤 감속을 적용한다. 감속 후보의 기하 계산에는 원래 동작 속도 궤적을, 적용에는 정지 반영 궤적을 사용한다. 진단 옵션 `--cruise-callback-clock recorded-motion-velocity-receipt`는 기록된 **동작 속도 궤적의 수신 시각만** 사용하고, 그 시각 이전에 도착한 최신 odometry·객체·가속도를 고른다. 입력 궤적의 점은 여전히 route부터 독립 계산한다. 별도 `recorded-optimizer-receipt` 옵션은 한 단계 이른 시각의 민감도 진단으로 남겨 두었다. 두 옵션 모두 기록된 중간 출력의 **값**은 독립 종단 계산에 넣지 않는다. 정확한 ROS 콜백 시작 시각과 polling 구독자 내부 큐 순서는 rosbag만으로 알 수 없고, 동작 속도 출력 수신은 콜백 시작의 하한에 가깝다.

## 현재 원본 노드 이미지와 기록 출력의 차이

기록된 실제 입력 궤적의 값까지 후보 코어에 넣는 **조건부 진단**은 route 독립 재생과 별개다. [재생기](obstacle_cruise_motion_input_distance_audit.py)는 해당 궤적 수신 시각 이전의 차량·객체를 선택하고 [후보 실행기](../tests/obstacle_cruise_motion_input_audit_runner.cpp)를 연속 호출한다. 같은 입력을 `autoware-partition:sample-adsw-decision` 원본 노드에 발행한 표식도 비교했다. 원본 노드 이미지는 기준 decision launch 설정과 이 bag에서 역산된 순항 RSS 안전거리 6m를 사용했다. 보유 기록 당시 실행 파일·파라미터와 같다고 확인된 것은 아니다. HPC S자 기록의 정지 디버그 타입은 `tier4_debug_msgs`, 현재 이미지 출력 타입은 `autoware_internal_debug_msgs`여서 실행 버전 차이의 단서도 있다.

| bag·조건부 진단 구간 | planning-aa 후보/현재 원본 이미지 순항 표식 | 보유 기록 순항 활성 | 첫 활성 (현재 이미지·후보 / 기록) |
| --- | ---: | ---: | ---: |
| HPC cutinout, 동작 속도 유효 입력 180–213 | 30/30프레임, 활성 집합 완전 일치 | 13/34프레임 | 184 / 201 |
| HPC blocking, 동작 속도 유효 입력 150–344 | 75/75프레임, 활성 집합 완전 일치 | 47/195프레임 | 163 / 276 |

HPC cutinout의 원본 기록이 활성인 201–212번 12프레임에서 현재 후보 코어가 계산한 원시 객체 거리는 11프레임이 기록값과 0.00014m 이내다. 211번의 차이는 약 0.218m이며 동작 속도 궤적 수신 뒤 odometry가 한 번 더 도착한 프레임이다. 전체 201–213번 공동 활성 13프레임의 원시 거리 절대 오차 중앙값/최대는 0.000020/0.218159m다. HPC blocking의 공동 활성 47프레임은 0.005654/0.205082m다. 수치 오차와 남은 활성 차이는 콜백 중 도착한 입력·기록 당시 설정/버전을 구분해 해석해야 한다. 첫 충돌 거리 계산이나 후보 진입을 rosbag 출력에 맞춰 임의로 늦출 근거는 없다.

[HPC cutinout 후보 감사](obstacle_cruise_cutinout_motion180_214_distance_audit.json), [현재 원본 노드 실행](obstacle_cruise_cutinout_motion180_230_launch.json), [HPC blocking 후보 감사](obstacle_cruise_blocking_motion150_345_distance_audit.json), [현재 원본 노드 실행](obstacle_cruise_blocking_motion150_345_launch.json)에 프레임별 근거를 보관했다. 조건부 후보 감사는 원본 `motion_velocity_planner/trajectory` 점을 사용하므로 독립 종단 동등성을 증명하지 않는다.

HPC S자 유효 동작 속도 입력 300–549번에서도 같은 조건부 비교를 수행했다. [후보 감사](obstacle_cruise_scurve_motion300_550_distance_audit.json)의 planning-aa 정지/순항 활성은 25/224프레임이고, [현재 원본 이미지 실행](obstacle_cruise_scurve_motion300_550_launch.json)은 25/224프레임이다. 정지는 활성 프레임 집합이 정확히 같고, 순항은 223프레임이 겹치며 517/518번에서 한 프레임 차이가 있다. 현재 이미지와 planning-aa가 모두 순항을 계산하고 단일 후보 거리도 있는 215프레임의 원시 거리 절대 오차 중앙값/95백분위/최대는 0.003796/0.065135/0.276832m다. 반면 **보유 기록**의 정지/순항 표식은 233/8프레임이다. 특히 325번은 기록의 정지 표식과 현재 원본 이미지·planning-aa의 순항 후보가 갈린다. 이는 현재 이미지와의 이관 대조 및 기록 당시 실행 대조를 분리해야 함을 보여 준다. package 전환 파라미터를 준 기존 원본 이미지 재실행도 450–549번의 기록 정지 85프레임 중 28프레임만 정지 표식을 만들었다. 따라서 package 값 하나를 기록 당시 설정으로 확정할 수 없다.

원본 `createStopObstacleForPredictedObject`의 내부 정지 객체 미래 충돌 확인에는 `behavior_determination.stop.crossing_obstacle.collision_time_margin`이 적용된다. planning-aa에서 고정 1초를 설정값으로 바꾸고 기본 launch 1초와 package YAML 4초를 선택할 수 있게 했다. package 전환 속도 3.5/3.0m/s와 충돌 시간 4초를 함께 사용한 [planning-aa 조건부 감사](obstacle_cruise_scurve_motion450_550_package_stop_distance_audit.json)는 같은 값으로 실행한 [현재 원본 이미지](obstacle_cruise_scurve_valid450_650_package_stop.json)의 450–549번 정지 활성 28프레임과 정확히 일치한다. 순항은 현재 이미지 32, planning-aa 31프레임이며 518번 하나가 다르다. 보유 기록 정지 85프레임과는 여전히 차이가 크므로 package 설정을 기록 당시 값으로 간주하지 않는다.

같은 100프레임에서 package 전환 속도를 유지하고 충돌 시간만 4초에서 1초로 바꾼 [민감도 감사](obstacle_cruise_scurve_motion450_550_package_speed_margin1_audit.json)의 정지·순항 활성은 같았다. 이 기록 구간은 충돌 시간 설정의 효과를 직접 확인하지 못한다. 대신 미래에 횡방향으로 이탈하는 합성 객체에서 1초는 정지 후보를 제외하고 4초는 유지하는 분기를 `planning_pipeline_cruise_bridge` 검사로 확인했다. 이 설정은 원본 코드에 있는 값이므로 bag의 최종 오차를 맞추기 위한 보정은 아니다.

IPC cutinout 유효 동작 속도 입력 300–499번의 [후보 감사](obstacle_cruise_ipc_cutinout_motion300_500_distance_audit.json)는 보유 기록과 planning-aa의 순항 활성 집합이 135/135프레임으로 같다. [현재 원본 이미지 실행](obstacle_cruise_ipc_cutinout_motion300_500_launch.json)도 순항 135프레임과 정지 6프레임의 활성 집합이 planning-aa와 정확히 같다. 현재 이미지와 planning-aa의 순항 원시 거리 135개는 절대 오차 중앙값/최대 0.000051/0.000996m다. 보유 기록의 정지 표식은 1프레임이므로 이 분기는 기록 당시 상태와 구분한다.

HPC pedestrian2 유효 동작 속도 입력 100–399번은 보유 기록의 정지 표식이 69프레임, 기본 launch 설정의 [planning-aa 조건부 감사](obstacle_cruise_pedestrian2_motion100_400_distance_audit.json)는 60프레임이었다. 빠진 249–257번의 보행자 관측 속도는 약 2.04–3.08m/s이며 이전 정지점 좌표가 유지된다. 원본 `checkConsistency`는 `behavior_determination.obstacle_velocity_threshold_from_stop_to_cruise`를 이전 정지 객체 유지에도 쓴다. planning-aa의 `CruiseStopConsistency` 호출에는 1.5m/s 기본값이 고정돼 있었으므로 이를 순항 후보 설정의 동일 임계값으로 연결했다. [package 전환 속도 3.5/3.0m/s 감사](obstacle_cruise_pedestrian2_motion100_400_package_speed_margin1_audit.json)는 기록과 정지 활성 69/69프레임이 정확히 같다. 충돌 시간 여유만 1초에서 4초로 바꾼 [대조](obstacle_cruise_pedestrian2_motion100_400_package_stop_distance_audit.json)는 이 구간의 활성 집합을 바꾸지 않았다. 기록 당시 설정 덤프가 없으므로 이 일치는 package 설정이 실제 적용됐다는 증명은 아니다.

현재 원본 이미지의 [기본 launch 재실행](obstacle_cruise_pedestrian2_motion100_400_launch.json)은 planning-aa 기본 감사의 정지 활성 60/60프레임과 정확히 같다. 차이가 발생한 240–269번을 [package 설정으로 재실행](obstacle_cruise_pedestrian2_motion240_270_package_stop.json)하면 현재 이미지와 planning-aa의 정지 활성 18/18프레임도 정확히 같다. 따라서 이전 상태 유지 임계값 전달을 실제 원본 노드 양쪽 설정에서 확인했지만, 기록 당시 어느 설정이 사용됐는지는 여전히 미확인이다.

동일한 package 전환 속도·충돌 시간 조건부 감사를 [HPC runout 100–399번](obstacle_cruise_runout_motion100_400_package_audit.json)과 [IPC pedestrian 100–399번](obstacle_cruise_ipc_pedestrian_motion100_400_package_audit.json)에도 적용했다. 두 기록의 정지 활성 집합과 planning-aa는 각각 132/132, 83/83프레임으로 정확히 같다. 기본 launch 값에서는 각각 103/132, 73/83프레임만 활성이다. HPC pedestrian2까지 세 보행자 관련 구간 900프레임의 기록 정지 표식과 package 조건부 후보 집합이 맞는 것은 기록 당시 전환 속도 설정에 대한 강한 단서다. 다만 원본의 런타임 파라미터 파일·실행 버전·polling 입력 ID를 확보하지 못해 설정을 확정하거나 K-City 기본 재생을 바꾸지는 않았다.

HPC pedestrian2의 route 독립 종단 [package 설정 진단 CSV](data/planning_aa_vs_autoware_pedestrian2_hpc_package_settings_diagnostic.csv)도 기본 결과와 분리했다. 517개 독립 최종 출력은 유지되며 원본과의 차량 위치 계획 속도 절대 오차 중앙값은 기본 0.013989에서 진단 0.011901m/s로 변하고, 95백분위는 둘 다 1.063713m/s다. 63프레임의 독립 차량 위치 속도가 바뀌지만 최대 변화는 0.585m/s다. 정지 후보의 조건부 활성 일치가 최종 경로 전체 동등성을 뜻하지 않으며, package 설정을 기본값으로 바꿀 근거는 아직 없다.

HPC S자의 route 독립 종단 [package 설정 진단 CSV](data/planning_aa_vs_autoware_scurve_hpc_package_settings_diagnostic.csv)도 분리했다. 753개 최종 출력은 유지되고 속도 절대 오차 중앙값은 기본 0.250079에서 진단 0.185279m/s로 줄지만, 95백분위는 1.902755에서 1.972623m/s로 커진다. 329프레임의 독립 차량 위치 속도가 바뀌며 최대 변화는 1.405m/s다. 조건부 정지 후보와 현재 원본 이미지의 일치만으로 종단 그래프를 전 구간 개선하지 못한다.

## route 독립 재생의 결과

10개 K-City bag은 별도 순항 시계 진단에서도 route부터 validator까지 전체 프레임의 최종 출력을 생성했다. 결과 [요약 CSV](../../251231_KCity_ori/planning_aa_scenario_comparison_summary_cruise_input_clock_diagnostic.csv)를 기본 재생과 분리했다. 프레임별 진단 CSV는 `251231_KCity_ori/planning_aa_result_diagnostics_20261011.tar.gz`에 보관했다. 필요할 때 다시 만드는 그래프의 네 번째 패널은 순항이 활성인 bag에서 원본·독립 필터 객체 거리를 실제 순항 수신/디버그 시각에 표시한다.

HPC cutinout에서 원본 후보는 **route 재생 CSV 기준** 200번부터, planning-aa는 183번부터 활성이다. 위 표의 원본 201번은 동작 속도/디버그 스트림의 유효 입력 번호이며, 다른 번호 축이다. 원본 경로 값을 사용하지 않은 route 독립 재생에서도 17프레임의 조기 활성은 남는다. 프레임 240의 원본/독립 필터 거리는 23.210009/23.210302m로 가깝지만, 그 전 17프레임의 PID 누적 상태와 최종 평활화 상태는 다르다. HPC blocking·S자에도 기록과 독립 후보의 활성 차이가 남는다.

`motion_velocity_planner/trajectory` 수신 시계는 입력 선택을 원본 구조에 더 가깝게 하지만, 모든 시나리오의 최종 속도 오차를 줄이지는 않는다. 특히 HPC blocking에서는 행동 경로의 차로 변경·정지점 차이가 앞선 계산 단계에 남아 있다. 정확한 외부 승인 입력 시각, 기록 당시 실행 설정과 파일, polling 입력의 메시지 ID를 확보하면 그 경계부터 다시 비교해야 한다.

순항 정지→감속 순서 변경 전후의 기본 route 독립 재생은 10개 bag 모두에서 최종 계획 프레임 수와 속도 오차 중앙값·95백분위가 같았다. 이 bag들에서 순서 차이가 최종 속도 수치로 드러나지 않았다는 뜻이다. 별도 동작 속도 수신 시계·가속도 진단도 10개 bag 6,085프레임의 최종 출력을 모두 생성했다. HPC cutinout의 속도 절대 오차 중앙값/95백분위는 기본 0.027/1.735m/s, 진단 0.023/1.703m/s였지만 HPC blocking은 0.146/3.707에서 0.214/3.721m/s로 커졌다. 이 시계를 기본 입력 짝으로 확정하지 않는다.

최종 진단 CSV 6,085프레임 중 60프레임은 행동 경로 수신 뒤 90ms 안에 대응하는 기록 동작 속도 궤적 수신이 없어 행동 경로 수신 시각으로 대체했다. 30프레임은 선택한 시각 이전의 가속도 기록이 없어 0m/s²를 사용했고, 이 중 29프레임에는 대응하는 원본 최종 궤적도 없다. 별도 순항 시각의 odometry 수신은 6,002프레임에서 행동 경로 단계의 수신과 달랐다. 따라서 별도 스냅샷 경로가 실제로 사용됐지만, 모든 프레임의 순항 콜백 입력을 복원한 것은 아니다. 원본 노드는 가속도 입력 자체가 없으면 궤적을 발행하지 않는 분기가 있으므로 초기 대체 프레임의 동등성을 주장하지 않는다.

재현 예시:

```sh
cmake --build /tmp/planning-aa-map-prox-build --target obstacle_cruise_motion_input_audit_runner planning_pipeline_rosbag_runner
python3 reference/obstacle_cruise_motion_input_distance_audit.py \
  /tmp/planning-aa-map-prox-build/obstacle_cruise_motion_input_audit_runner \
  /home/sujin/Documents/01_GIT/01_OSS/251231_KCity_ori/hpc/rosbag_cutinout/rosbag_20251231_024435_0.db3 \
  --start 180 --frames 34
python3 reference/run_planning_pipeline_scenario_plots.py \
  --cruise-callback-clock recorded-motion-velocity-receipt \
  --include-existing-stopline
python3 reference/compare_obstacle_cruise_motion_audit.py \
  reference/obstacle_cruise_scurve_motion300_550_distance_audit.json \
  reference/obstacle_cruise_scurve_motion300_550_launch.json
python3 reference/obstacle_cruise_motion_input_distance_audit.py \
  /tmp/planning-aa-map-prox-build/obstacle_cruise_motion_input_audit_runner \
  /home/sujin/Documents/01_GIT/01_OSS/251231_KCity_ori/hpc/rosbag_scurve/rosbag_20251231_022311_0.db3 \
  --start 450 --frames 100 --package-cruise-hysteresis \
  --stop-collision-time-margin 4
```
