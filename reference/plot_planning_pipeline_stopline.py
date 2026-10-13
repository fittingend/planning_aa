"""Plot recorded Autoware and ROS-less final plans on a shared scenario clock."""

import argparse
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def original_time_origin_ns(directory):
    scripts = sorted(directory.parent.glob("merge_and_plot_signals*.py"))
    if not scripts:
        raise ValueError("원본 그림 생성 스크립트를 찾을 수 없습니다")
    filenames = set(re.findall(r"PROCESSED_[\w]+\.jsonl",
                               scripts[0].read_text()))
    first = None
    for filename in filenames:
        path = directory / filename
        if not path.exists():
            continue
        with path.open() as stream:
            for line in stream:
                stamp = json.loads(line).get("timestamp")
                if isinstance(stamp, (int, float)):
                    stamp = int(stamp)
                    if stamp < 1_000_000_000_000:
                        stamp *= 1_000_000_000
                    first = stamp if first is None else min(first, stamp)
    if first is None:
        raise ValueError("원본 그림의 시간 원점을 계산할 processed_output이 없습니다")
    return first


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("series_csv", type=Path)
    parser.add_argument("original_xlsx", help="원본 그림의 xlsx; 없으면 -")
    parser.add_argument("processed_dir", help="원본 processed_output; 없으면 -")
    parser.add_argument("output_png", type=Path)
    parser.add_argument("--scenario-label", default="K-City scenario")
    args = parser.parse_args()
    plans = pd.read_csv(args.series_csv)
    has_original_figure = args.original_xlsx != "-"
    if not has_original_figure:
        original = pd.DataFrame({
            "time_sec": (plans["receipt_ns"] - plans["receipt_ns"].min()) * 1e-9,
            "speed_kmh": plans["measured_speed_mps"] * 3.6,
        })
        origin = int(plans["receipt_ns"].min())
    else:
        original = pd.read_excel(args.original_xlsx, sheet_name="signals")
        origin = original_time_origin_ns(Path(args.processed_dir))
    plans["time_sec"] = (plans["receipt_ns"] - origin) * 1e-9
    # A distant projection is not a meaningful ego plan value.
    for prefix in ("source", "independent"):
        far = plans[f"{prefix}_ego_path_offset_m"] > 2.0
        for field in ("planned_speed_mps", "planned_acceleration_mps2",
                      "first_stop_distance_m"):
            plans.loc[far, f"{prefix}_{field}"] = np.nan

    has_cruise = (
        "source_cruise_obstacle_distance_filtered_m" in plans and
        (plans["source_cruise_obstacle_distance_raw_m"].gt(0).any() or
         plans["independent_cruise_obstacle_distance_filtered_m"].notna().any()))
    fig, axes = plt.subplots(4 if has_cruise else 3, 1, sharex=True,
                             figsize=(15, 15 if has_cruise else 12))
    t = plans["time_sec"].to_numpy()
    original_t = original["time_sec"].to_numpy()
    source_color = "tab:blue"
    independent_color = "tab:orange"

    axes[0].plot(original_t, original["speed_kmh"].to_numpy(),
                 color="0.7", linewidth=1, label="Measured vehicle speed")
    for prefix, label, color in (("source", "Autoware final plan", source_color),
                                 ("independent", "planning-aa final plan",
                                  independent_color)):
        axes[0].plot(t, plans[f"{prefix}_planned_speed_mps"].to_numpy() * 3.6,
                     color=color, linewidth=1.6, label=label)
    axes[0].set(title="Speed at ego position", ylabel="Speed [km/h]")

    if "target_acceleration" in original:
        axes[1].plot(original_t, original["target_acceleration"].to_numpy(),
                     color="0.55", linewidth=0.8, alpha=0.7,
                     label="Recorded control target")
    if "measured_acceleration" in original:
        axes[1].plot(original_t, original["measured_acceleration"].to_numpy(),
                     color="0.8", linewidth=0.7, alpha=0.65,
                     label="Measured vehicle acceleration")
    for prefix, label, color in (("source", "Autoware final plan", source_color),
                                 ("independent", "planning-aa final plan",
                                  independent_color)):
        axes[1].plot(t, plans[f"{prefix}_planned_acceleration_mps2"].to_numpy(),
                     color=color, linewidth=1.5, label=label)
    axes[1].set(title="Along-path acceleration near ego",
                ylabel="Acceleration [m/s²]")

    context = next(((field, label) for field, label in (
        ("start_distance", "Recorded traffic-light stopline"),
        ("crosswalk_start_distance", "Recorded crosswalk start"),
        ("lead_ego_distance_m", "Recorded lead-vehicle distance"))
        if field in original), None)
    if context:
        axes[2].plot(original_t, original[context[0]].to_numpy(),
                     color="0.65", linewidth=1, label=context[1])
    for prefix, label, color in (("source", "Autoware first zero-speed sample", source_color),
                                 ("independent", "planning-aa first zero-speed sample",
                                  independent_color)):
        axes[2].plot(t, plans[f"{prefix}_first_stop_distance_m"].to_numpy(),
                     color=color, linewidth=1.5, label=label)
    axes[2].set(title="Distance to first zero-speed sample (may be path terminal)",
                ylabel="Distance [m]")
    status = next(((field, label) for field, label in (
        ("command_type", "Traffic control"),
        ("stop_pose_flag", "Pedestrian stop pose"),
        ("route_obstacle", "Route obstacle")) if field in original), None)
    signal_axis = None
    if status:
        signal_axis = axes[2].twinx()
        commands = original[status[0]]
        for value, color in ((0, "red"), (1, "green")):
            selected = commands == value
            signal_axis.scatter(original.loc[selected, "time_sec"].to_numpy(),
                                np.full(selected.sum(), value), color=color,
                                s=3, alpha=0.6, label=f"{status[1]}={value}")
        signal_axis.set(ylabel=status[1], ylim=(-0.15, 1.15), yticks=(0, 1))
    if has_cruise:
        source_cruise_t = (
            plans["source_cruise_debug_receipt_ns"] - origin) * 1e-9
        independent_cruise_t = (
            plans["cruise_clock_receipt_ns"] - origin) * 1e-9
        source_filtered = plans[
            "source_cruise_obstacle_distance_filtered_m"].where(
                plans["source_cruise_obstacle_distance_raw_m"] > 0)
        independent_filtered = plans[
            "independent_cruise_obstacle_distance_filtered_m"]
        axes[3].plot(source_cruise_t.to_numpy(), source_filtered.to_numpy(),
                     color=source_color, linewidth=1.5,
                     label="Autoware cruise filtered obstacle distance")
        axes[3].plot(independent_cruise_t.to_numpy(),
                     independent_filtered.to_numpy(),
                     color=independent_color, linewidth=1.5,
                     label="planning-aa cruise filtered obstacle distance")
        axes[3].set(title="Cruise candidate activity and filtered obstacle distance",
                    ylabel="Distance [m]")
    axes[-1].set_xlabel("Time [s] on " +
                        ("original figure axis" if has_original_figure else
                         "scenario receipt axis"))
    for axis in axes:
        axis.grid(True, alpha=0.4)
        axis.legend(loc="upper right", fontsize=8)
    valid_plans = int(plans["independent_planned_speed_mps"].notna().sum())
    if valid_plans == 0:
        axes[0].text(0.5, 0.12, "planning-aa: no final trajectory in this replay",
                     transform=axes[0].transAxes, ha="center", va="center",
                     fontsize=12, color=independent_color,
                     bbox={"facecolor": "white", "alpha": 0.85,
                           "edgecolor": independent_color})
    if signal_axis:
        signal_axis.legend(loc="lower right", fontsize=8)
    axes[-1].set_xlim(0, max(original["time_sec"].max(), t.max()))
    fig.suptitle(f"{args.scenario_label}: recorded Autoware vs planning-aa\n"
                 "Final trajectory projected at the same recorded ego pose")
    fig.tight_layout()
    args.output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output_png, dpi=150)
    plt.close(fig)
    print(f"saved {args.output_png}; frames={len(plans)}, "
          f"source plans={plans['source_planned_speed_mps'].notna().sum()}, "
          f"planning-aa plans={valid_plans}")


if __name__ == "__main__":
    main()
