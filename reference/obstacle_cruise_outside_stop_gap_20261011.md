# 장애물 순항 노드 차로 밖 정지 분기 이관 (2026-10-11)

## 원본 근거와 설정

원본 `autoware_obstacle_cruise_planner/src/node.cpp`의 `createStopObstacleForPredictedObject`와 `createCollisionPointForOutsideStopObstacle`를 기준으로 이관했다. 원본은 객체가 현재 궤적 외형에서 정지 횡여유 0.3m 밖에 있을 때 다음 순서로 판정한다.

1. 차로 밖 정지 허용 객체 종류와 `precise_lat_dist / approach_velocity <= max_lat_time_margin_for_stop`을 확인한다.
2. 보행자·자전거는 `객체 속도 / (2 × 종류별 감속률)`까지 예측하고, 다른 종류는 기본 10초를 사용한다.
3. 신뢰도 순으로 지정 개수의 예측 경로를 0.1초마다 표본화한다. 객체 외형과 기존 주행 다각형의 미래 충돌점들을 찾는다. 정지 횡여유 0.3m는 빠른 거리 필터에만 더한다.
4. 첫·마지막 충돌 시각과 자차의 도착·통과 시각의 차이가 `collision_time_margin`을 넘으면 그 경로를 제외한다. 통과한 첫 경로의 충돌점 위치를 정지 후보에 연결한다.

기준 **Obigo decision launch**의 `common.stop_obstacle_type.outside.*`는 모두 `false`다. 패키지 기본 YAML에서는 unknown을 제외한 객체 종류가 `true`다. 두 설정을 혼동하면 보유 K-City 기록의 기본 재생이 달라지므로, planning-aa의 기본 종류 마스크는 Obigo launch처럼 모두 꺼져 있다. 합성 원본 비교에는 원본 노드와 planning-aa에서 같은 종류를 명시적으로 켰다. launch의 경로 수는 1개, 횡진입 시간은 4.5초, 보행자·자전거 감속률은 각 0.5m/s², 충돌 시간 여유는 1초다.

## 이관 내용

`PlanningCruiseStopCandidateCore::plan`은 기존 차로 밖 객체 건너뛰기를 원본 분기로 교체했다. `CruiseCandidateParameters`에 허용 종류, 예측 경로 수, 횡진입 시간, 종류별 감속률을 추가하고, 기존 `stop_collision_time_margin_s`를 충돌 시각 판정에 연결했다. 미래 외형 충돌에는 원본처럼 여유를 넓히지 않은 기존 주행 다각형을 사용한다. 첫 충돌 경로의 위치는 원본 `polygon_utils::getCollisionPoint`처럼 첫 충돌 경로점의 호길이에서 차량 앞부분 기준 최대 충돌 길이를 뺀다. 이전 정지 객체 유지와 종단 정지점 삽입은 내부 객체 분기와 공유한다.

rosbag replay와 조건부 원본 입력 감사 실행기에 차로 밖 정지 종류와 예측 경로 수 설정을 추가했다. 재생 실행기에서는 횡진입 시간·보행자 및 자전거 감속률도 선택할 수 있다.

## 원본 노드 대조

현재 `autoware-partition:sample-adsw-decision` 이미지의 원본 노드에 IPC 보행자 bag의 시계·메시지 틀을 사용하고, 경로·차량·객체만 동일한 합성 횡단 객체 입력으로 바꿔 열 조건을 실행했다. planning-aa의 ROS 없는 후보 실행기는 같은 수치 입력과 콜백·객체 시각을 받았다. [CSV](obstacle_cruise_outside_stop_synthetic.csv), [PNG](obstacle_cruise_outside_stop_synthetic.png), 재현 [스크립트](plot_obstacle_cruise_outside_stop_synthetic.py)에 결과가 있다.

| 조건 | 원본 정지 수 | planning-aa 정지 수 | 선택 충돌 거리 |
| --- | ---: | ---: | ---: |
| Obigo launch처럼 종류 끔 | 0 | 0 | 없음 |
| 차로 밖 보행자 횡단 | 1 | 1 | 양쪽 10.91m |
| 충돌 시각이 늦은 35m 위치 | 0 | 0 | 없음 |
| 최고 신뢰도 경로가 비충돌, 1개 검사 | 0 | 0 | 없음 |
| 같은 입력에서 2개 검사 | 1 | 1 | 양쪽 10.91m |
| 차로 밖 자전거 횡단 | 1 | 1 | 양쪽 10.91m |
| 차로 밖 차량 횡단 | 1 | 1 | 양쪽 9.21m |
| 곡선 경로의 보행자 횡단 | 1 | 1 | 원본 10.83567047m, planning-aa 10.83567011m |
| 삼각형 외형의 보행자 | 1 | 1 | 양쪽 11.21m |
| 곡선·삼각형 외형의 보행자 | 1 | 1 | 원본 11.13885880m, planning-aa 11.13885888m |

[연속 유지 CSV](obstacle_cruise_outside_stop_hold.csv)·[그래프](obstacle_cruise_outside_stop_hold.png)는 첫 프레임에 충돌한 보행자가 다음 프레임부터 멀어지는 합성 입력 18프레임을 비교한다. 현재 원본 노드와 planning-aa의 정지 활성은 18/18프레임 일치하고, 둘 다 객체 헤더 시각이 1초에 도달하는 250번 프레임에서 이전 정지를 해제한다. 유지 중 선택 충돌 거리는 각각 10.90999985m·10.91m다. 재현 스크립트는 [연속 비교기](compare_obstacle_cruise_outside_hold.py)다.

별도로 IPC 보행자 bag 240–269번의 실제 입력을 차로 밖 종류를 켠 원본 노드에 재생한 결과, 정지 표식은 원본과 planning-aa 모두 21/30프레임이었다. 기본 설정에서도 원본이 21/30프레임이므로 이 구간은 차로 밖 분기의 활성 증거가 아니다. 당시 기록의 원본 표식은 30/30으로 현재 이미지와 다르며, 기록 당시 바이너리·설정 및 polling 입력 선택은 확인되지 않았다.

## 남은 확인

열 합성 조건과 18프레임의 유지 조건은 만든 입력에 한정된다. 실제 K-City 차로 밖 활성 연속 입력에 대한 원본 동등성은 확인되지 않았다. 기본 launch에서 차로 밖 정지가 꺼져 있으므로, 이 검증 결과를 기본 K-City 기록의 종단 속도 차이 해소로 해석하지 않는다.
