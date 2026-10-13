# Reeds–Shepp 및 RRT* 계산

이 디렉터리는 `autoware_freespace_planning_algorithms`의 Reeds–Shepp 거리 계산과 RRT* 탐색 코드를 포함한다. 원본 저작권·라이선스 고지를 각 파일에 유지했다. 독립 C++ 빌드를 위해 사용하지 않는 ROS 헤더와 JSON 상태 덤프를 제외하고, 재현 가능한 검사를 위해 난수 시드를 입력받도록 조정했다. `planning_core`는 선택적 자유공간 주차 탐색과 휴리스틱에 이 코드를 사용한다.
