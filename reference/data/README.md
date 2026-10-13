# 원본 대조에 쓰는 진단 CSV

각 bag의 `result` 폴더는 기본 종단 비교 CSV·PNG만 보관한다. 이 폴더에는 문서와 원본 대조 그림의 재생성에 직접 필요한 진단 CSV 10개를 보관한다.

- `*_cruise_input_clock_diagnostic_slowdown_diagnostic.csv` 4개: HPC blocking·cutinout·runout·scurve의 감속 활성 비교 그림 입력
- `*_cruise_input_clock_diagnostic_slowdown_interval.csv` 1개: HPC runout의 감속 시작·종료 거리와 목표 속도 비교 그림 입력
- `*_package_settings_diagnostic.csv` 2개, `*_package_hysteresis.csv` 1개: 원본 패키지 설정 민감도 비교
- `*_recorded_limit.csv` 1개, `*_lateral_time_1p5_diagnostic.csv` 1개: 조건부 속도 제한과 순항 후보 경계 비교

나머지 진단 CSV·로그·재생 설정 기록은 `../../../251231_KCity_ori/planning_aa_result_diagnostics_20261011.tar.gz`에 원래 경로를 유지한 채 보관했다. 이후 `run_planning_pipeline_scenario_plots.py`로 재생한 진단 결과는 각 bag의 `result`가 아닌 `251231_KCity_ori/planning_aa_diagnostics/`에 생성된다.
