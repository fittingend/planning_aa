# 동작 속도 콜백 입력·최소 제동 거리 감사 (2026-10-11)

## 원본 계산과 이관 범위

원본 `MotionVelocityPlannerNode::on_trajectory`는 경로 수신 뒤 `update_planner_data`에서 odometry, acceleration, predicted objects를 각각 `PollingSubscriber::takeData()`로 읽는다. 출력 궤적은 입력 경로의 header를 복사한다. 따라서 같은 header의 입력·출력을 bag에서 연결할 수 있지만, 그 사이에서 polling이 정확히 어떤 메시지를 반환했는지는 bag의 수신 시각만으로 알 수 없다. 신호는 선택 입력이며 이전 관측을 보유할 수 있다.

원본 `OutOfLaneModule::plan`의 최소 정지 거리는 `PlannerData::calculate_min_deceleration_distance(0.0).value_or(0.0)`으로 계산한다. 인자는 콜백의 자차 속도·가속도와 velocity smoother의 `getMinDecel()`, `abs(getMinJerk())`, `getMinJerk()`다. 이전 감속점이 있으면 그 점의 경로 호길이와 감속 거리를 비교한 후, 자차가 첫 유지 경로 선분에서 얼마나 앞섰는지를 더한다. 이번 이관은 [종단 어댑터](../src/pipeline/planning_pipeline_map_adapter.cpp)의 선택적 `PlanningOutOfLaneBrakingInput`에 이 값을 명시하면 기존 원본 저크 제약 감속 공식으로 최소 정지 호길이를 구하고 이전 감속점도 반영한다. 완성된 최소 호길이를 직접 제공하는 입력도 유지한다. 제한값의 기본 추정은 두지 않는다.

현재 원본 launch의 동작 속도 노드는 `Analytical.param.yaml`도 받지만, `SmootherBase::getMinDecel()`과 `getMinJerk()`가 실제로 읽는 키는 **`normal.min_acc`와 `normal.min_jerk`**다. launcher의 `common.param.yaml`은 둘 다 `-1.0`으로 설정한다. `Analytical.param.yaml`의 `backward.min_acc=-2.5`, `backward.min_jerk=-1.5`를 이 계산에 대입하면 안 된다. **기록 당시의 런타임 파라미터 덤프는 없다.** 현재 파일값을 과거 실행값으로 단정하지 않는다. `calcCrosswalkDecelDistance`는 원본 `calcDecelDistWithJerkAndAccConstraints`의 세 감속 유형을 이관한 기존 코어로, 원본 테스트의 정상·급제동 거리 표본으로 이미 검증돼 있다.

## bag 수신 시각으로 확인한 범위

[감사 스크립트](audit_motion_velocity_callbacks.py)는 optimizer 경로와 같은 header stamp의 동작 속도 출력을 묶고, 경로 수신 직전 최신 입력의 나이와 출력 전에 새 입력이 들어온 프레임 수를 센다. 출력 전 수신은 **원본 콜백이 그 메시지를 사용했다는 뜻이 아니다.** 아래 수치는 입력 짝의 불확실성을 나타낸다.

| HPC bag | optimizer 입력 / 같은 header의 후속 출력 | 직전 odometry 나이 중앙값/95백분위 | 출력 전 새 odometry / acceleration / objects 수신 프레임 | 차로 밖 주행 겹침 영역 표식 / virtual wall |
| --- | ---: | ---: | ---: | ---: |
| blocking | 580 / 576 | 7.910 / 18.790ms | 179 / 202 / 169 | 43 / 0 |
| S자 | 760 / 758 | 14.218 / 20.407ms | 645 / 640 / 597 | 4 / 0 |

원본 차로 밖 주행 debug marker의 겹침 영역이 비어 있지 않은 프레임도 있었지만 virtual wall 메시지는 없었다. 기존 조건부 경로·객체 진단에서는 두 bag 모두 독립 감속 후보가 0개였고 원본 동작 속도 궤적의 변경도 0프레임이었다. 겹침 영역 표식만으로 감속 결정이 활성화됐다고 볼 수 없다. 기본 종단 재생에 임의의 polling 짝이나 제동 제한을 넣지 않은 이유다.

## 첫 계산 차이

10개 시나리오 기본 CSV의 `first_difference_stage`는 **최초의 1mm 또는 0.001m/s 점별 차이**를 뜻한다. 큰 최종 속도 차이가 시작된 단계를 별도로 확인했다.

| 위치 | 첫 수치 차이 | 큰 차이가 나타나는 단계 |
| --- | --- | --- |
| blocking 전체 573프레임 | 모두 행동 경로 (점 수 449, 수치 124) | 320번 행동 경로 13점/원본 54점, 차량 위치 경로 속도 11.111/2.780m/s. 최종 스무더 5.138/1.858m/s. 원본 RTC 상태 출력은 승인 명령 시각이 아니므로 차로 변경 분기의 동등성은 미확인. |
| S자 0~105번 | Elastic Band | 초반 최종 차량 위치 속도 차이는 작다. 130번 이후에는 행동 경로가 대다수 프레임의 첫 차이 단계다. |
| S자 390번 | 행동 경로 점 수 102/101 | 동작 속도 단계의 첫 정지 거리는 189.725/189.728m로 가깝다. 차로 주행 단계에서 독립 189.725m, 원본 10.266m로 갈라져 최종 스무더 2.346/1.380m/s가 된다. |
| S자 630번 | 행동 경로 수치 차이 (점 수 64/64) | 동작 속도·차로 주행 점 수 242/242, 첫 정지 거리 약 114.269m가 가깝다. 최종 스무더 차량 위치 속도는 2.363/7.076m/s로 갈라진다. 이전 상태·경로 형상·원본 콜백 입력 짝이 남은 확인 대상이다. |

재현은 `audit_motion_velocity_callbacks.py <bag.db3>` 및 `tests/planning_pipeline_rosbag_replay.py --from-route --callback-clock recorded-behavior-receipt --diagnose-frame-index <번호>`로 한다. blocking·S자의 활성 첫 차이는 차로 밖 주행 결과보다 상류 행동 경로, 원본 차로 주행의 정지 삽입, 최종 스무더 상태에 먼저 있다. 원본 RTC 승인 **명령 시각**과 정확한 polling 반환 메시지 ID를 보유 기록에서 복원할 수 없으므로 동등성을 주장하지 않는다.

후속 [행동 경로 기준선 감사](behavior_route_callback_boundary_20261011.md)에서 S자 630번의 큰 최종 차이는 원본식 원시 중심선 선행 절단을 적용하면 차량 위치 속도 7.088/원본 7.076m/s로 줄어듦을 확인했다. 위 표의 2.363m/s는 **기본 재생** 값이다. 390번의 기록 정지 삽입은 차로 주행 단계에서 계속 빠지며, 전체 bag 속도 오차가 개선되지는 않아 선행 절단은 진단 옵션으로 남겼다.
