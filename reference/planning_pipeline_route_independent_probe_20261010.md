# Route 입력 독립 연속 재생 및 첫 차이 추적 (2026-10-10)

## 계산 경계

`tests/planning_pipeline_rosbag_replay.py --from-route`는 K-City IPC 정지선 rosbag의 route·Lanelet2 지도·odometry·acceleration·객체·신호를 외부 입력으로 사용한다. 기록된 행동 경로와 그 뒤의 중간 출력은 **독립 계산에 공급하지 않는다**. 하나의 `PlanningPipelineMapAdapter`와 `PlanningFinalPipelineCore` 인스턴스를 여러 프레임에 유지한다. 계산 순서는 route → 행동 경로 → 행동 속도 → Elastic Band → 경로 최적화 → 동작 속도 → 장애물 순항·정지 → 차로 주행 → scenario selector → 최종 JerkFiltered velocity smoother → 최종 trajectory validator다. 기존 차로 주행 단계의 validator도 별도 상태로 남아 있다.

`--callback-clock odometry`는 odometry 수신 다섯 건마다 실행한다. `--callback-clock recorded-behavior-receipt`는 기록된 행동 경로 **메시지 수신 시각만** 계획 tick으로 사용한다. 그 메시지의 경로 값은 비교에만 쓴다. 각 tick은 그 수신 시각 이전의 최신 외부 입력을 택한다. 행동 속도 원본 출력은 tick 이후 80ms 안의 첫 수신으로 대응하고, 후속 원본 단계는 행동 속도 출력의 **헤더 시각**으로 연결한다. 이 방법은 입력 선택 시각과 원본 출력 체인을 좁히지만 원본 callback ID, 실제 내부 콜백 시각, 모듈 상태의 동일성을 증명하지 않는다.

## 관측

- 수정 전 odometry 시계 초기 25프레임: 독립 코어 완료 25/25, 차로 주행 출력 25/25. 수신 시각 이후 100ms 이내의 첫 원본 차로 주행 출력 24개 중 점 수가 같은 것은 23개였고, 점 수가 같은 대응의 최대 위치 차이는 0.147849m였다. 이 수치는 수정 전 진단이며 정확한 콜백 짝도 아니다.
- 행동 경로 수신 시계 초기 25프레임: 독립 코어·차로 주행·scenario 선택·최종 스무더·최종 validator가 각각 25/25프레임 출력했다. 행동 속도 수신 및 헤더 시각으로 원본 출력 체인을 25개 대응했다. 수정 전 행동 경로는 첫 프레임에서 독립 34점, 원본 76점이고 시작점이 약 6m 달랐다. 원본의 2m 중심선 표본화, 자차 기준 뒤쪽 절단, 발행 경로 재표본화를 이관한 후 **25/25프레임에서 행동 경로의 점 수·시작점·끝점이 일치**했다. 차로별 Lanelet2 제한속도를 사용한 뒤 행동 경로 속도 차이도 0이 됐다.
- 목표점 절단·출력 입력점 보존을 적용하기 전 초기 25프레임의 단계별 `비교 가능/점 수 일치/점별 위치 1mm·속도 0.001m/s 이내`는 행동 경로 `25/25/0`, 행동 속도 `25/0/0`, Elastic Band `22/22/0`, 경로 최적화·동작 속도·차로 주행·scenario 선택 각각 `25/24/0`, 최종 스무더·validator 각각 `25/25/0`이었다. 행동 경로의 최대 인덱스별 위치 차이는 **0.083892m**였다. 아래의 재검증 결과가 최신 값이다.
- 이전의 잘못된 4.17m/s 노드 제한 설정을 쓴 재생에서 최종 validator 경로는 25/25에서 점 수가 같고, 같은 인덱스 점의 최대 위치 차이는 **0.004484m**, 최대 속도 차이는 **6.669986m/s**였다. 프레임마다 계산한 최대 차이의 중앙값은 위치 **0.004475m**, 속도 **2.499984m/s**였다. 차로 주행 경로의 점 수가 같은 24프레임은 최대 위치 **0.135862m**, 속도 차이 **0m/s**였다. 이 값은 최신 제한 설정 결과가 아니며, 수신·헤더 시각으로 추정 대응한 경로의 인덱스별 값이다.
- **가장 큰 속도 차이는 첫 프레임의 궤적 종단부**다. 169점 중 인덱스 167(끝에서 두 번째 점), 시작점부터 약 33.742m, 좌표 `(2582.6984, 23938.0222)`에서 독립 출력은 0m/s, 원본은 6.669986m/s였다. 가장 큰 위치 차이는 16번째 프레임의 인덱스 164(169점 중 끝에서 다섯 번째), 시작점부터 약 30.743m, 좌표 `(2581.3527, 23935.3418)`에서 약 4.524mm였다. 이 점들이 실제 정지선의 정확한 위치라는 판정은 하지 않았다.

### 목표점 절단·입력점 보존 후 재검증

원본 `debug/root_reference_path` 표식의 76개 점을 같은 수신 시각의 독립 경로와 비교했다. 25/25프레임에서 위치가 수치 정밀도 안에서 일치했다. 원본 행동 경로 발행 결과도 25/25프레임에서 점 수 76개와 점별 위치·속도가 1mm·0.001m/s 이내로 일치했다. 두 경로 모두 동일한 기록 경로를 계산 입력으로 사용하지 않았다.

blind spot 제어점 삽입 전 단계별 `비교 가능/점 수 일치/점별 위치 1mm·속도 0.001m/s 이내`는 행동 경로 `25/25/25`, 행동 속도 `25/0/0`, Elastic Band `22/22/0`, 경로 최적화·동작 속도·차로 주행·scenario 선택 각각 `25/24/0`, 최종 스무더·validator 각각 `25/25/0`이었다. 아래의 행동 속도 재검증이 최신 값이다. 원본 내부 callback ID와 계획 모듈의 이전 상태는 기록만으로 확인되지 않았다.

원본 `behavior_path_planner`는 `getReferencePath`에서 자차 주변 lanelet sequence를 선택하고 중심선을 2m 간격으로 구성한 뒤, `backward_path_length + input_path_interval`과 `forward_path_length`로 잘라낸다. `getPath`에서 출력 경로를 다시 2m 간격으로 표본화하고 발행 직전에도 자차 위치를 기준으로 자른다. 기준 설정은 뒤 5m, 앞 300m, 입력·출력 간격 각각 2m다. 원본 `RouteHandler::getCenterLinePath`는 Lanelet2 교통규칙 속도를 각 차로의 경로점에 기록한다. 수정 전 `PlanningPipelineMapAdapter::onRouteFrame`은 선택한 route 차로의 지도 중심선 점을 그대로 이어 붙이고 속도를 고정 10m/s로 채웠다. 현재 일반 route 경로에는 목표점 호길이 절단과 원본 `resamplePathWithSpline`의 입력점 우선 보존 순서까지 적용했다. 명시적 행동 모듈 경로는 별도 검증이 필요하다.

## 재현

```sh
python3 tests/planning_pipeline_rosbag_replay.py \
  /tmp/planning-aa-map-build/planning_pipeline_rosbag_runner \
  /home/sujin/Documents/01_GIT/01_OSS/251231_KCity_ori/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3 \
  --from-route --callback-clock recorded-behavior-receipt --frames 25 \
  --assert-route-behavior-reference \
  --assert-behavior-velocity-after-warmup
```

`--callback-clock`을 생략하면 odometry 시계를 사용한다. `--from-route`를 생략하면 기존 검사처럼 기록된 행동 경로를 계산 입력으로 사용한다. `planning_pipeline_route_inputs_rosbag_replay`와 `planning_pipeline_route_callback_clock_rosbag_replay` CTest는 두 시계에서 초기 5프레임의 최종 단계 출력을 검사한다.

## 남은 경계

### 행동 속도 단계 추가 진단 (2026-10-10)

원본 `behavior_velocity_planner` 노드는 플러그인 실행 뒤 `filterLitterPathPoint` → `interpolatePath`(1m 간격과 원래 경로점 모두 유지) → `filterStopPathPoint` 순서로 경로를 발행한다. 경로 수정 전 독립 route 재생의 행동 속도 출력은 153/154/154점, 원본은 175/175/175점이었다. 경로 수정 뒤 blind spot 제어점 연결 전에는 독립 173/174/174점, 원본 175/175/175점이었다. 현재 첫 3개 비교 프레임은 독립 174/175/175점, 원본 175/175/175점이다. 2~3번째 프레임의 첫 정지점과 점별 속도는 원본과 일치하지만, 첫 프레임의 독립 계산은 종점에서야 0m/s가 된다. 첫 비교 프레임 이전에 원본 행동 경로 메시지가 2개 발행됐지만 독립 재생은 새 인스턴스로 시작한다. 원본 모듈의 이전 상태와 정확한 콜백 입력을 복원할 수 없으므로 첫 프레임의 원인을 신호 판단 코드만으로 확정하지 않는다.

현재 독립 route 경로 자체가 원본과 점별 일치하므로 앞 단계의 형상 증폭은 해결됐다. 이전에 빠졌던 `(2596.4658633, 23989.0617245)` 부근 제어점은 원본 `BlindSpotModule::generateStopLine`의 경로 삽입이었다. 원본은 안전 판단 전에 0.2m 보간 경로에서 차로 시작점에 가장 가까운 선분의 시작점을 넣으며, 다중 차로 ID가 있는 입력점을 보간 경로에 보존한다. 이관 코드도 기본 route에서 blind spot을 활성화하고 안전 판단 전 기본·긴급 제어점을 삽입하도록 수정했다. 교차로 후보를 임의 추가한 앞선 실험은 회귀 때문에 되돌렸다. 독립 종단 재생에는 원본 행동 경로를 입력하지 않았다.

원본 `resamplePathWithSpline`에 맞춰 route 경로 발행의 0.2m 근접점 판정, 첫 정지점 보존, 재표본화 후 접선 방향 재계산을 반영했다. 추가로 `getReferencePath`의 목표점 호길이 절단과 `getPath`에서 실행 중인 목표 모듈이 입력점을 보존하는 순서를 반영했다. 신호 요소의 `SOLID_OFF`를 필터링하던 rosbag 입력 변환도 원본 `processTrafficSignals`와 `traffic_light_utils`가 상태 필터 없이 요소를 전달하는 방식에 맞췄다(이 bag의 신호 요소는 모두 `SOLID_ON`이라 수치 영향 없음). 종단 runner의 행동 속도 플러그인은 기준 `default_preset.yaml`과 `behavior_planning.launch.xml`의 활성 목록 및 순서로 교정했다. 최신 25프레임 종단 재생의 행동 속도는 24/25프레임에서 원본과 점 수가 같고 해당 24프레임의 점별 속도도 같다. 같은 인덱스 위치의 최대 잔차는 0.002362m이며 원본보다 1mm 큰 교차로 제어점 보간 차이다. 최종 최대 속도 차이는 여전히 6.669986m/s다.

진단 재현: `python3 tests/planning_pipeline_rosbag_replay.py /tmp/planning-aa-map-build/planning_pipeline_rosbag_runner /home/sujin/Documents/01_GIT/01_OSS/251231_KCity_ori/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3 --from-route --callback-clock recorded-behavior-receipt --frames 3 --diagnose-behavior-velocity`. 기록된 행동 경로를 넣어 모듈 자체의 잔차를 분리하려면 `--from-route --callback-clock recorded-behavior-receipt`를 빼고 실행한다.

1. 명시적 행동 모듈 경로에도 발행 단계의 표본화·절단 규칙을 연결하고 원본 활성 출력과 비교한다. 일반 route의 목표점 직전 8.4cm 잔차는 이번 수정에서 해소했다.
2. 첫 프레임의 신호·정지 상태와 각 단계가 소비했을 가능성이 높은 odometry·객체·신호의 헤더/수신 시각을 기존 기록 안에서 더 좁힌다. 원본 callback ID와 누락된 이전 상태는 보유 기록만으로 확정할 수 없으며, 이번 수신 시각 대응은 추정이다.
3. 외부 속도 제한·운영 모드·주차 경로가 실제 들어오는 연속 기록에서 상태 전환, 최종 스무더의 이전 출력, validator 정책까지 점별 비교한다. 이 IPC 구간에서 비어 있는 입력은 현재 기본값으로 처리된다.

최종 속도 잔차의 다음 추적 지점은 원본 scenario 선택 출력과 속도 평활화 입력의 대응이다. 이 rosbag에는 `velocity_smoother/debug/trajectory_raw`, `trajectory_lateral_acc_filtered`, `trajectory_time_resampled`, `forward_filtered_trajectory`, `backward_filtered_trajectory`, `merged_filtered_trajectory` 토픽 정의가 있으나 **메시지는 0건**이다. 따라서 내부 경로를 이 기록만으로 직접 비교할 수 없다. 보유 자료에서는 기록된 scenario 출력과 평활화의 `closest_velocity`·`closest_acceleration` 등 스칼라 진단을 대조하고, 내부 경로는 원본 코드와 합성 입력으로 계산 순서를 검증한다. 현재 차로 주행의 같은 인덱스 위치 잔차가 최대 0.135456m이므로 최종 속도 차이를 평활화 풀이기만의 문제로 단정할 수 없다.

### 다른 보유 rosbag 및 풀이기 대조

보유한 K-City rosbag 10개 모두 지도·route·scenario 선택·최종 평활화 출력이 있다. 그러나 위 평활화 내부 경로 디버그 토픽 여섯 종류의 메시지는 **10개 전부 0건**이다. 다른 기록으로 새 원본 내부 경로를 얻을 수는 없지만, 장면을 바꾼 출력 대조는 가능하다.

- HPC `rosbag_stopline2` 초기 25프레임의 route 독립 재생은 최종 출력까지 25회 계산했다. 원본 행동 경로는 독립 83점 대 원본 84점으로 시작했고 25회 모두 점 수가 달랐다. 행동 속도는 23/25회 점 수가 같았지만 최대 위치 1.004910m·속도 11.111111m/s 차이여서 이 기록을 IPC 정지선과 같은 동등성 사례로 간주할 수 없다. 원본 종단 헤더 시각 대응도 validator 20/25회라 비교가 일부만 가능했다. 이 탐색은 `--allow-incomplete-source-chain`으로 누락 대응을 허용했다.
- 저장된 **원본 scenario 출력**을 스무더 코어에 직접 넣는 `smoother_rosbag_long_replay.py`는 IPC 정지선과 HPC 정지선2에서 각각 25프레임을 원본 최종 출력과 대조했다. 양쪽에서 공통 경로 위치와 종단 5m를 제외한 속도 차이는 출력 정밀도 0.001m·0.001m/s 이하였다. 이 검사는 ProxQP 빌드를 사용하고 첫 비교 프레임 전에 기록된 원본 평활화 출력을 이전 상태로 넣으므로, route 독립 종단 동등성을 뜻하지 않는다.
- IPC 정지선 route 독립 재생을 같은 25프레임에 ProxQP 빌드로 다시 실행하자 차로 주행 경로의 최대 위치 잔차가 ADMM의 0.135456m에서 0.023118m로 줄었다. 행동 속도 잔차는 0.002362m로 같고, 최종 최대 속도 차이도 6.669986m/s로 그대로였다. 최종 위치 차이의 프레임별 중앙값은 ADMM 0.004474m에서 ProxQP 0.000033m로 줄었다. 따라서 풀이기 차이는 경로 형상에 영향을 주지만 현재 종단 속도 잔차 전체의 원인은 아니다.

재현할 때 ProxQP 지도 빌드는 `-DPLANNING_USE_PROXQP=ON -DPLANNING_BUILD_LANELET_BIN_ADAPTER=ON`으로 구성하고, 위 25프레임 명령의 runner를 그 빌드의 `planning_pipeline_rosbag_runner`로 바꾼다. 스무더 단독 대조는 `python3 tests/smoother_rosbag_long_replay.py <smoother_preprocess_runner> <smoother_reference_runner> <bag.db3> --frames 25`로 실행한다. 남은 속도 차이는 원본 직전 상태를 공급하지 않은 독립 초기화와 콜백 입력 짝, scenario 입력 경로의 형상 차이를 각각 분리해서 확인해야 한다.

### 최종 속도 차이 원인 확인

이전 6.669986m/s 최대 잔차에는 **재생기 설정 오류와 첫 프레임 초기 상태 차이**가 겹쳐 있었다. 재생기는 외부 입력 토픽 `/planning/scenario_planning/max_velocity`가 비어 있을 때 4.17m/s를 노드 최대 속도로 넣었다. 원본 launch에서 이 토픽은 스무더 **입력**이고 `/planning/scenario_planning/current_max_velocity`는 스무더 **출력**이다. 이 bag에는 입력 메시지가 0건, 현재 제한 출력은 11.11111068725586m/s 한 건이다. 기록된 출력은 실행 당시 노드 설정값의 근거로만 읽고 프레임별 독립 계산 입력으로 재공급하지 않았다. `--smoother-node-max-velocity` 옵션으로 노드 설정값을 명시하고 외부 입력 토픽은 그대로 유지했다.

ProxQP 지도 빌드에 이 설정값을 주고 IPC 정지선 25프레임을 다시 독립 재생하자 최종 스무더·validator는 25/25회 점 수가 같고, **첫 프레임 이후 24/24회는 점별 위치 1mm·속도 0.001m/s 이내**였다. 그 24프레임의 점별 속도 최대 차이는 0.000000954m/s, 최종 위치 잔차의 프레임별 최대값 중앙값은 0.000033m였다. ADMM 빌드도 기본 노드 최대 속도를 11.1m/s로 고치자 초기 5프레임 중 2~5번째의 최종 속도 차이는 수십 µm/s 수준으로 줄었다. 반면 ADMM의 경로 형상 위치 잔차는 약 4.5mm로 남았다.

첫 프레임은 독립 scenario 경로가 278점·첫 정지점 인덱스 277인데 원본은 279점·인덱스 168이었다. 독립 스무더는 자차 속도 0m/s와 이전 출력 없음으로 시작해 169개 최종점의 속도를 모두 0m/s로 냈다. 원본은 첫 비교 tick 전에도 최종 trajectory를 발행했고, 근처 `closest_velocity`·`closest_acceleration` 진단값은 각각 약 0.250035m/s·0.500004m/s²였다. 원본 최종 궤적 인덱스 167의 속도는 6.669986m/s였다. 같은 두 번째 tick을 새 독립 인스턴스의 첫 프레임으로 단독 실행하면 다시 전체 0m/s와 같은 최대 차이가 나타나지만, 첫 tick부터 연속 실행하면 두 번째 tick의 점별 속도가 원본과 맞는다. 이는 **냉시작과 이전 출력·engage 초기 운동 상태가 첫 프레임 차이의 직접 원인**임을 뒷받침한다. 첫 tick의 행동 속도 정지점 누락과 원본 callback 입력 짝 차이도 scenario 경로를 바꾸므로 그 기여량을 이 기록만으로 완전히 분리할 수는 없다.

최신 재현: `python3 tests/planning_pipeline_rosbag_replay.py /tmp/planning-aa-map-prox-build/planning_pipeline_rosbag_runner /home/sujin/Documents/01_GIT/01_OSS/251231_KCity_ori/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3 --from-route --callback-clock recorded-behavior-receipt --frames 25 --smoother-node-max-velocity 11.11111068725586 --assert-route-behavior-reference --assert-behavior-velocity-after-warmup --assert-final-speed-after-warmup`. 기존 관측의 프레임별 최대 속도 차이 중앙값 2.499984m/s는 잘못된 4.17m/s 제한 설정에 따른 값이며 최신 결과로 사용하지 않는다. 전체 25프레임 최대값 6.669986m/s는 첫 프레임 때문에 여전히 남는다.

### IPC 정지선 전체 기록 검증 (574프레임)

같은 bag의 초기 25프레임 밖까지 `--frames 10000`으로 재생해 입력이 모두 있는 **574프레임**을 선택했다. ProxQP 빌드와 노드 제한값 11.11111068725586m/s를 사용했고, 원본 행동 경로·행동 속도·각 종단 출력은 계산 입력으로 넣지 않았다. 독립 route→validator 계산은 574/574회 끝났으며, 원본 행동 속도 수신/헤더를 통한 최종 출력 체인도 574/574회 대응했다. `--allow-incomplete-source-chain` 없이도 이 대응 검사가 통과했다. 아래 표는 원시 중심선 절단 반영 전 결과다. 콜백 ID와 내부 상태가 같은지는 확인되지 않는다.

| 단계 | 원본과 비교 가능 | 점 수 일치 | 점별 위치 1mm·속도 0.001m/s 이내 |
| --- | ---: | ---: | ---: |
| 행동 경로 | 574 | 247 | 76 |
| 행동 속도 | 574 | 130 | 0 |
| 경로 최적화·동작 속도·차로 주행·scenario 선택 | 각각 574 | 각각 322 | 각각 0 |
| 최종 스무더·validator | 각각 574 | 각각 63 | 각각 47 |

프레임 인덱스는 0부터 센다. 최종 궤적은 첫 프레임의 냉시작 차이를 제외하면 **1~47프레임 연속** 점별 기준에 맞았다. 48~53프레임은 점 수는 같지만 최대 위치 잔차가 1mm를 넘었고, 54프레임부터 점 수 차이가 나타났다. 63~502프레임은 최종 스무더 점 수가 계속 달랐다. 행동 경로는 0~59프레임 일치, 60~61프레임 점 수 차이, 62~77프레임 다시 일치한 뒤 78프레임부터 위치/점 수 차이가 지속됐다. 원본 `debug/root_reference_path`도 같은 76프레임에서 1mm 이내로 맞고, 60~61프레임의 일시적인 점 수 차이와 78프레임 이후 지속적인 차이를 보였다. 따라서 최종 이탈은 행동 경로의 지속 이탈보다 **30프레임 먼저** 시작하며, 전부 행동 경로 원인으로 돌릴 수 없다.

점 수가 다른 최종 경로는 같은 인덱스 속도 비교가 무의미하므로, 독립 경로점에서 원본 경로의 최근접 선분까지 5cm 이내인 점에 한해 선분 보간 속도를 진단용으로 비교했다. 63~573프레임에서 70,144/77,246점이 이 조건에 들었고, 511프레임 중 230프레임은 대응점의 최대 속도 잔차가 1m/s를 넘었다. 최대는 93번째 프레임의 6.844647m/s다. 이 수치는 서로 다른 시점·형상에서 가장 가까운 선분을 택한 **공간 대응 진단**이며 원본 콜백 동등성이나 정확한 속도 오차를 증명하지 않는다.

원본 `utils::getReferencePath`는 현재 차로의 lanelet sequence를 고르고, 자차 중심 호길이에서 뒤 15m(`backward_path_length` 5m + `extra_margin` 10m) 지점부터 원시 중심선을 잘라 **그다음** 2m 재표본화한다. 이전 이관 코드는 전체 route 중심선을 먼저 재표본화하고 자차 기준으로 잘라냈다. 48프레임부터 시작되는 최종 경로 위치 잔차는 별도로 Elastic Band·최적화·평활화의 첫 위치 이탈을 추적해야 한다.

### 현재 차로열 절단 실험과 기본 경로 복구

기존 `Rosless-Lanelet2` 중심선과 route 차로를 사용해, 다음 route 구간에서 실제 지도 연결이 있는 차로를 고른다. 현재 차로에서 전방 300m를 채우고 후방 15m가 부족하면 이전 차로를 포함한 다음 원시 중심선을 `ego_s - 15m`부터 `ego_s + 300m`까지 잘라 재표본화하는 실험 경로를 만들었다. 새 Lanelet adapter는 만들지 않았다. 합성 연속 차로 지도에서 다음 차로 초입의 이전 차로 포함, 다음 차로 안쪽의 원시 중심선 절단, 선호 차로가 직접 연결되지 않았을 때 연결된 route 차로 선택을 검사했다. 전체 bag 일치율이 낮아져 현재는 `--experimental-route-sequence-clip`을 지정할 때만 켜고, 기본 경로는 기존 동작으로 복구했다.

동일한 ProxQP·노드 속도·수신 시각 조건의 574프레임 재생에서 행동 경로 점 수 일치는 247→283프레임, 점별 1mm·0.001m/s 일치는 76→60프레임이었다. 최종 궤적 점별 일치는 47→47프레임으로 유지됐다. 초기 25프레임의 행동 경로 25/25 및 첫 프레임 뒤 최종 궤적 24/24 일치도 유지됐다. 따라서 절단 순서 구현은 확인했지만, 이 rosbag의 원본 활성 출력에 대한 동등성이 개선됐다고 볼 수 없다.

새 절단은 선택한 odometry가 첫 차로의 15m 지점을 지난 62프레임부터 격자 원점을 움직인다. 원본 `debug/root_reference_path`는 62~77프레임에 고정된 2m 격자를 유지하고 78프레임부터 연속적으로 움직인다. 원본 행동 경로 콜백이 사용한 odometry의 정확한 헤더·수신 시각은 이 기록에서 확인되지 않는다. 현재 실험 구현은 route 시작 선호 차로에서 지도 연결을 따라가지만, 원본 RouteHandler의 상태 있는 현재 차로 선택과 전체 route primitive 탐색까지 이관한 것은 아니다. 원본 계산의 입력 짝과 차로 선택을 먼저 좁혀야 하며, 현재의 60/574 수치를 알고리즘 단독 오차로 해석해서는 안 된다.

2026-10-11 추가 감사에서 55~85프레임의 행동 경로 메시지 수신 직전 최신 odometry 나이는 약 12~18ms, `root_reference_path` 표식 수신 차이는 약 8~13ms, 원본 `debug/total_time/processing_time_ms`는 대체로 8~11ms였다. 이 수치만으로 원본 폴링 구독자가 실제 소비한 odometry 메시지를 식별할 수는 없다. 재생 입력을 수신 시각보다 0.5·1·1.5·2초 앞당겨도 30~109프레임 구간의 행동 경로 일치는 각각 32·32·32·32/80프레임으로, 지연 0초의 30/80프레임과 큰 차이가 없고 후반 일치를 회복하지 못했다. 원본 첫 route 차로 24744에는 `waypoints` 속성이 없었다.

`backward_path_length=7m`라면 원시 절단이 17m에서 시작해 78프레임 변화를 설명할 수 있다는 가설도 검사했다. 당시 선행 절단 실험의 초기 110프레임 일치는 기본 5m에서 60/110, 7m에서 2/110으로 악화됐고, 기본 5m 경로는 76/110이었다. 아래 재검증에서 이 실험에는 후방 절단 인덱스 오류가 함께 있었다는 것을 확인했다.

### 2026-10-11 원본 후방 절단 인덱스와 목표점 경계 재검증

위 7m 실험의 2/110 수치는 원본 설정 가설 자체를 반증하지 않는다. 해당 시점의 `cropRouteReference`는 원본 `motion_utils::cropBackwardPoints`의 `begin + i` 대신 한 표본 이전인 `begin + i - 1`을 택했다. 이 인덱스를 실험 경로에서 원본대로 수정하고, `RouteHandler::getCenterLinePath`의 목표점 호길이 절단을 재표본화 전에 수행하도록 고쳤다. 기본 경로는 기존 5m·인덱스를 유지했다.

새 `reference/route_reference_stage_compare.py`와 `route_reference_geometry_probe`는 동일 bag의 지도·route 전체 primitive·선택 odometry를 읽어 차로, 중심선 호길이, 원시 절단 시작점, 원본 `root_reference_path` 시작점을 프레임별로 비교한다. 원본 ROS `RouteHandler` 바이너리를 실행한 것은 아니며, checkout 코드의 경계 계산을 별도 재현한 진단이다. 원본 표식은 첫 프레임에 중심선 호길이 약 6m에서 시작하고, 62프레임에 약 8m, 78프레임부터 약 10m로 이동한다. 7m 후방 길이·2m 재표본화·10m 원시 여유의 조합이 이 전환과 부합한다. 로컬 checkout YAML 기본값은 5m이므로 **bag 활성 바이너리 설정이 7m였다는 것은 추정**이다.

`--experimental-route-sequence-clip --route-backward-path-length 7`의 전체 574프레임 재생 결과는 행동 경로 점 수 일치 **294/574**, 점별 1mm·0.001m/s 이내 **76/574**, 최종 궤적 점별 일치 **47/574**다. 이전 실험의 60/574보다는 행동 경로가 좋아졌지만 기본 경로의 점별 76/574보다 낫지 않다. 78프레임부터 계산 시작점과 원본 표식은 약 0.1m 차이로 이어지고, 이후 2m 격자 선택 차이도 보인다. 표식 수신 시각을 odometry 선택 시계로 바꿔도 이 차이는 거의 줄지 않았다. 몇 프레임에서 0.1m는 더 이전 odometry 1~3건의 이동량과 비슷하지만, 원본 콜백 입력 ID가 없으므로 시각 오차라고 단정할 수 없다. 초기 110프레임에서 현재 차로는 모두 24744로 식별되어, 이 구간의 첫 차이를 route primitive 분기 선택 탓으로 돌릴 근거는 없다. 원본과 독립 재생의 첫 출력 차이는 여전히 행동 속도 단계다.

실험 경로에는 현재 route 차로 ID를 프레임 간 유지하고, 새 route일 때 초기화하며, 선택 차로를 포함하는 구간을 기준으로 앞뒤 primitive 연결을 구성하는 정책도 추가했다. 이 변경 후 IPC 574프레임의 위 일치 수치는 그대로다. 원본의 거리·yaw 제약 및 승인 모듈 상태에 따른 현재 차로 갱신까지 동등하다는 뜻은 아니다.

같은 수정 코드에서 실험 옵션만 켜고 기본 후방 5m를 유지하면 행동 경로 점별 일치는 0/574다. 따라서 실험 결과를 재현할 때는 7m 인자를 함께 지정해야 한다. 이 차이도 7m 설정의 원본 사용을 증명하지는 않는다.

교차 확인으로 HPC `rosbag_stopline2` 첫 25프레임도 재생했다. 기본·실험 7m 모두 독립 종단 계산 25/25, 행동 경로 점별 일치 0/25로 같았다. 이 기록은 IPC 경로의 7m 추정을 일반화하는 근거가 되지 않는다. IPC `rosbag_pedestrian` 첫 25프레임은 두 설정 모두 경로 계산 또는 최종 검증 단계가 빠져 재생기 완료 조건을 만족하지 못했으므로, 동등성 수치에 포함하지 않았다.

재현: `cmake --build /tmp/planning-aa-map-prox-build --target route_reference_geometry_probe planning_pipeline_rosbag_runner planning_route_action_map_test -j 4`; `python3 reference/route_reference_stage_compare.py /tmp/planning-aa-map-prox-build/route_reference_geometry_probe <bag.db3> --frames 574 --backward-length 7`; `python3 tests/planning_pipeline_rosbag_replay.py /tmp/planning-aa-map-prox-build/planning_pipeline_rosbag_runner <bag.db3> --from-route --callback-clock recorded-behavior-receipt --frames 10000 --smoother-node-max-velocity 11.11111068725586 --experimental-route-sequence-clip --route-backward-path-length 7 --diagnose-segments`. 다음 확인 대상은 원본 callback이 소비한 odometry 헤더 ID와 활성 바이너리 설정, 상태 있는 현재 차로 선택 및 전체 primitive 분기다.

재현에는 위 전체 재생 명령에 `--experimental-route-sequence-clip`을 추가한다. 후방 길이 가설을 별도로 검사할 때만 `--route-backward-path-length 7`, odometry 수신 시각 가설을 검사할 때만 `--odometry-receipt-lag-ms 500`과 `--offset 30`을 추가한다. 이 옵션은 비교용이며 기본 경로의 입력 선택을 바꾸지 않는다.

전체 재현: `python3 tests/planning_pipeline_rosbag_replay.py /tmp/planning-aa-map-prox-build/planning_pipeline_rosbag_runner /home/sujin/Documents/01_GIT/01_OSS/251231_KCity_ori/ipc/rosbag_stopline/rosbag_20251231_132036_0.db3 --from-route --callback-clock recorded-behavior-receipt --frames 10000 --smoother-node-max-velocity 11.11111068725586 --diagnose-segments --diagnose-spatial-final`. 이 전체 범위의 불일치 때문에 초기 25프레임 결과를 rosbag 전체의 동등성으로 해석하면 안 된다.
