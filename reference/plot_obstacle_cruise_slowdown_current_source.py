"""Plot recorded, current-source, and independent slowdown activation."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd


ROOT = Path(__file__).resolve().parents[1]
CASES = (
    ("blocking", "obstacle_cruise_blocking_slowdown_300_345_source.json"),
    ("cutinout", "obstacle_cruise_cutinout_slowdown_495_525_source.json"),
    ("runout", "obstacle_cruise_runout_slowdown_225_265_source.json"),
    ("scurve", "obstacle_cruise_scurve_slowdown_520_580_source.json"),
)


def main():
    rows = []
    fig, axes = plt.subplots(len(CASES), 1, figsize=(12, 8.5),
                             layout="constrained")
    for axis, (scenario, source_file) in zip(axes, CASES):
        source = json.loads((ROOT / "reference" / source_file).read_text())
        result = (ROOT / "reference/data" /
                  f"planning_aa_vs_autoware_{scenario}_hpc_"
                  "cruise_input_clock_diagnostic_slowdown_diagnostic.csv")
        independent = pd.read_csv(result).set_index("frame")
        for item in source:
            frame = item["frame"]
            record = independent.loc[frame]
            rows.append({
                "scenario": scenario,
                "frame": frame,
                "recorded_source": int(record.source_slowdown_object_count > 0),
                "current_source": int(bool(item["slowdown_debug"])),
                "planning_aa": int(record.independent_slowdown_candidate_count > 0),
            })
        subset = [row for row in rows if row["scenario"] == scenario]
        for name, offset, color, label in (
            ("recorded_source", 2, "#b3594b", "Recorded source"),
            ("current_source", 1, "#3572b0", "Current source probe"),
            ("planning_aa", 0, "#35a070", "planning-aa"),
        ):
            active = [row["frame"] for row in subset if row[name]]
            axis.scatter(active, [offset] * len(active), s=22,
                         color=color, label=label)
        axis.set_yticks((0, 1, 2), ("planning-aa", "Current source", "Recorded source"))
        axis.set_xlim(subset[0]["frame"] - 1, subset[-1]["frame"] + 1)
        axis.set_title(f"HPC {scenario}")
        axis.grid(axis="x", alpha=0.25)
    axes[-1].set_xlabel("Valid trajectory frame")
    output = ROOT / "reference" / "obstacle_cruise_slowdown_source_activation_20261011"
    fig.savefig(output.with_suffix(".png"), dpi=160)
    plt.close(fig)
    with output.with_suffix(".csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    for scenario, _ in CASES:
        subset = [row for row in rows if row["scenario"] == scenario]
        print(scenario, len(subset),
              *(sum(row[name] for row in subset)
                for name in ("recorded_source", "current_source", "planning_aa")),
              "current/AA agreement",
              sum(row["current_source"] == row["planning_aa"]
                  for row in subset))
    print(output)


if __name__ == "__main__":
    main()
