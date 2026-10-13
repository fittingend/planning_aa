"""Compare current source slowdown debug against independent route replay."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "reference/obstacle_cruise_runout_slowdown_225_265_source.json"
REPLAY = (ROOT / "reference/data/"
          "planning_aa_vs_autoware_runout_hpc_"
          "cruise_input_clock_diagnostic_slowdown_interval.csv")
OUTPUT = ROOT / "reference/obstacle_cruise_runout_intervals_20261011"


def main():
    source = json.loads(SOURCE.read_text())
    independent = pd.read_csv(REPLAY).set_index("frame")
    rows = []
    for item in source:
        values = item["slowdown_debug"]
        if not values:
            continue
        frame = item["frame"]
        replay = independent.loc[frame]
        if not replay.independent_slowdown_candidate_count:
            continue
        rows.append({
            "frame": frame,
            "source_start_arc_m": values[1],
            "planning_aa_start_arc_m": replay.independent_slowdown_start_arc_m,
            "source_end_arc_m": values[2],
            "planning_aa_end_arc_m": replay.independent_slowdown_end_arc_m,
            "source_target_velocity_mps": values[4],
            "planning_aa_target_velocity_mps":
                replay.independent_slowdown_target_velocity_mps,
        })
    if not rows:
        raise RuntimeError("감속 구간이 동시에 활성인 프레임이 없습니다")
    with OUTPUT.with_suffix(".csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    data = pd.DataFrame(rows)
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True,
                             layout="constrained")
    for axis, name, unit in zip(axes, ("start_arc", "end_arc", "target_velocity"),
                                ("m", "m", "m/s")):
        source_values = data[f"source_{name}_{'mps' if unit == 'm/s' else 'm'}"]
        independent_values = data[
            f"planning_aa_{name}_{'mps' if unit == 'm/s' else 'm'}"]
        axis.plot(data.frame.to_numpy(), source_values.to_numpy(),
                  label="Current source", color="#3572b0")
        axis.plot(data.frame.to_numpy(), independent_values.to_numpy(),
                  label="planning-aa",
                  color="#35a070", linestyle="--")
        axis.set_ylabel(f"{name.replace('_', ' ')} ({unit})")
        axis.grid(alpha=0.25)
        error = (source_values - independent_values).abs()
        print(name, "frames", len(error), "median", float(np.median(error)),
              "max", float(error.max()))
    axes[0].legend()
    axes[-1].set_xlabel("Valid trajectory frame")
    fig.suptitle("HPC runout: active obstacle slowdown intervals")
    fig.savefig(OUTPUT.with_suffix(".png"), dpi=160)
    plt.close(fig)
    print(OUTPUT)


if __name__ == "__main__":
    main()
