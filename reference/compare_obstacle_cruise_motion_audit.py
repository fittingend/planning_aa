"""Compare conditional planning-aa cruise candidates with a live original-node probe."""

import argparse
import json
import statistics
from pathlib import Path


def active_frames(rows, field):
    return {row["frame"] for row in rows if row[field] > 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("candidate_audit", type=Path)
    parser.add_argument("original_probe", type=Path)
    args = parser.parse_args()
    candidate_rows = json.loads(args.candidate_audit.read_text())
    original_rows = json.loads(args.original_probe.read_text())
    candidates = {row["frame"]: row for row in candidate_rows}
    original = {row["frame"]: row for row in original_rows}
    common = sorted(candidates.keys() & original.keys())
    if not common:
        raise ValueError("같은 프레임 번호가 없습니다")
    for label, marker, field in (
        ("정지", "obstacles_to_stop", "independent_stop_candidate_count"),
        ("순항", "obstacles_to_cruise", "independent_candidate_count"),
    ):
        source_set = {frame for frame in common if
                      original[frame].get("markers", {}).get("counts", {}).get(
                          marker, 0) > 0}
        candidate_set = {frame for frame in common if candidates[frame][field] > 0}
        print(f"{label}: 현재 원본 {len(source_set)}, planning-aa "
              f"{len(candidate_set)}, 공통 {len(source_set & candidate_set)}, "
              f"원본만 {sorted(source_set - candidate_set)}, "
              f"planning-aa만 {sorted(candidate_set - source_set)}")
    errors = []
    for frame in common:
        raw = original[frame].get("cruise_debug")
        predicted = candidates[frame].get("independent_raw_distance_m")
        if raw and len(raw) > 3 and raw[3] > 0 and predicted is not None:
            errors.append(abs(raw[3] - predicted))
    if errors:
        print(f"현재 원본 순항 원시 거리 공동 {len(errors)}프레임, "
              f"절대 오차 중앙/최대 "
              f"{statistics.median(errors):.6f}/{max(errors):.6f}m")


if __name__ == "__main__":
    main()
