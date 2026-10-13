"""Replay every K-City scenario and plot recorded vs planning-aa final plans."""

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-root", type=Path,
                        default=Path(__file__).resolve().parents[2] / "251231_KCity_ori")
    parser.add_argument("--runner", type=Path,
                        default=Path("/tmp/planning-aa-map-prox-build/planning_pipeline_rosbag_runner"))
    parser.add_argument("--include-existing-stopline", action="store_true")
    parser.add_argument("--reuse-csv", action="store_true",
                        help="기존 재생 CSV가 있으면 필요한 그림과 요약만 다시 만듭니다")
    parser.add_argument("--plot-diagnostics", action="store_true",
                        help="진단 재생의 PNG도 생성합니다 (기본은 CSV와 요약만 보관)")
    parser.add_argument("--with-cruise-slowdown", action="store_true",
                        help="원본 장애물 감속 후보를 연결한 별도 진단 결과를 만듭니다")
    parser.add_argument("--cruise-safe-distance-margin", type=float, default=6.0,
                        help="K-City 순항 디버그 RSS 값에서 확인한 실행 마진 (기본: 6m)")
    parser.add_argument("--cruise-outside-predicted-path-count", type=int, default=3,
                        help="원본 launch 설정의 차로 밖 순항 예측 경로 검사 개수")
    parser.add_argument("--cruise-max-lateral-time-margin", type=float, default=5.0,
                        help="원본 launch 설정의 차로 밖 순항 횡방향 진입 시간 상한")
    parser.add_argument("--cruise-callback-clock",
                        choices=("frame", "recorded-optimizer-receipt",
                                 "recorded-motion-velocity-receipt"),
                        default="frame",
                        help="원본 중간 출력의 수신 시각만 순항 입력 스냅샷 시계로 사용합니다")
    args = parser.parse_args()
    repo = Path(__file__).resolve().parents[1]
    root = args.bag_root.resolve()
    summary = []
    for bag in sorted(root.glob("*/rosbag*/*.db3")):
        platform = bag.parent.parent.name
        scenario = bag.parent.name.removeprefix("rosbag_")
        if platform == "ipc" and scenario == "stopline" and not args.include_existing_stopline:
            continue
        name = f"{scenario}_{platform}"
        result = bag.parent / "result"
        result.mkdir(exist_ok=True)
        suffix = ("" if args.cruise_callback_clock == "frame" else
                  "_cruise_clock_diagnostic" if args.cruise_callback_clock ==
                  "recorded-optimizer-receipt" else
                  "_cruise_input_clock_diagnostic")
        if args.with_cruise_slowdown:
            suffix += "_slowdown_diagnostic"
        output = result if not suffix else root / "planning_aa_diagnostics" / name
        output.mkdir(parents=True, exist_ok=True)
        csv_path = output / f"planning_aa_vs_autoware_{name}{suffix}.csv"
        png_path = output / f"planning_aa_vs_autoware_{name}{suffix}.png"
        log_path = output / f"planning_aa_vs_autoware_{name}{suffix}.log"
        manifest_path = output / f"planning_aa_vs_autoware_{name}{suffix}.replay.json"
        replay = [sys.executable, str(repo / "tests/planning_pipeline_rosbag_replay.py"),
                  str(args.runner), str(bag), "--from-route", "--callback-clock",
                  "recorded-behavior-receipt", "--frames", "100000",
                  "--simulation", "--crosswalk-auto-activation",
                  "--with-cruise-candidates",
                  "--cruise-safe-distance-margin",
                  str(args.cruise_safe_distance_margin),
                  "--cruise-outside-predicted-path-count",
                  str(args.cruise_outside_predicted_path_count),
                  "--cruise-max-lateral-time-margin",
                  str(args.cruise_max_lateral_time_margin),
                  "--smoother-node-max-velocity", "11.11111068725586",
                  "--allow-incomplete-source-chain", "--allow-incomplete-calculation",
                  "--export-series-csv", str(csv_path)]
        if args.cruise_callback_clock != "frame":
            replay.extend(("--cruise-callback-clock", args.cruise_callback_clock))
        if args.with_cruise_slowdown:
            replay.append("--with-cruise-slowdown")
        replay_signature = {
            "command": replay,
            "runner_mtime_ns": args.runner.stat().st_mtime_ns,
            "bag_mtime_ns": bag.stat().st_mtime_ns,
        }
        replay_exit = "reused"
        previous_signature = (json.loads(manifest_path.read_text())
                              if manifest_path.exists() else None)
        if (not args.reuse_csv or not csv_path.exists() or
                previous_signature != replay_signature):
            print(f"replay {name}", flush=True)
            completed = subprocess.run(
                replay, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True)
            replay_exit = completed.returncode
            if replay_exit:
                log_path.write_text(completed.stdout)
            else:
                log_path.unlink(missing_ok=True)
            if replay_exit == 0:
                manifest_path.write_text(
                    json.dumps(replay_signature, ensure_ascii=False, indent=2) + "\n")
        valid_csv = replay_exit in (0, "reused") and csv_path.exists()
        workbook = next(iter(sorted(result.glob("signals_merged_*.xlsx"))), None)
        processed = bag.parent / "processed_output"
        plot_result = None
        if valid_csv and (not suffix or args.plot_diagnostics):
            plot = [sys.executable, str(repo / "reference/plot_planning_pipeline_stopline.py"),
                    str(csv_path), str(workbook) if workbook else "-",
                    str(processed) if workbook else "-", str(png_path),
                    "--scenario-label", f"{platform.upper()} {scenario}"]
            plot_result = subprocess.run(plot, capture_output=True, text=True)
            if plot_result.returncode:
                print(plot_result.stderr, flush=True)
        row = {"scenario": name, "bag": str(bag),
               "cruise_callback_clock": args.cruise_callback_clock,
               "cruise_safe_distance_margin_m": args.cruise_safe_distance_margin,
               "cruise_outside_predicted_path_count": args.cruise_outside_predicted_path_count,
               "cruise_max_lateral_time_margin_s": args.cruise_max_lateral_time_margin,
               "replay_exit": replay_exit,
               "plot_exit": plot_result.returncode if plot_result else "",
               "frames": 0, "source_plans": 0, "planning_aa_plans": 0,
               "paired_speed_samples": 0,
               "median_speed_abs_error_mps": "", "p95_speed_abs_error_mps": "",
               "png": str(png_path) if plot_result and png_path.exists() else "",
               "csv": str(csv_path) if valid_csv else "",
               "log": str(log_path) if log_path.exists() else ""}
        if valid_csv:
            data = pd.read_csv(csv_path)
            row["frames"] = len(data)
            row["source_plans"] = int(data.source_planned_speed_mps.notna().sum())
            row["planning_aa_plans"] = int(data.independent_planned_speed_mps.notna().sum())
            valid = (data.source_ego_path_offset_m.le(2) &
                     data.independent_ego_path_offset_m.le(2))
            errors = (data.loc[valid, "source_planned_speed_mps"] -
                      data.loc[valid, "independent_planned_speed_mps"]).abs().dropna()
            row["paired_speed_samples"] = len(errors)
            if not errors.empty:
                row["median_speed_abs_error_mps"] = float(errors.median())
                row["p95_speed_abs_error_mps"] = float(np.percentile(errors, 95))
        summary.append(row)
        print(f"{name}: {row['planning_aa_plans']}/{row['frames']} planning-aa plans; "
              f"{row['source_plans']} source plans; plot exit {row['plot_exit']}", flush=True)
    summary_path = root / ("planning_aa_scenario_comparison_summary" +
                           ("" if args.cruise_callback_clock == "frame" else
                            "_cruise_clock_diagnostic" if args.cruise_callback_clock ==
                            "recorded-optimizer-receipt" else
                            "_cruise_input_clock_diagnostic") +
                           ("_slowdown_diagnostic" if args.with_cruise_slowdown
                            else "") + ".csv")
    with summary_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    print(f"summary {summary_path}")
    if any(row["replay_exit"] not in (0, "reused") or row["plot_exit"] != 0
           for row in summary):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
