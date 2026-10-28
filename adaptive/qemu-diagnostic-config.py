#!/usr/bin/env python3
"""Adjust only PlanningAA trajectory liveness for ARM64 QEMU simulation."""

import argparse
from pathlib import Path
import re


def adjust_topics(source):
    pattern = re.compile(
        r"(?P<head>- module: planning\n  mode: \[[^\n]+\]\n  type: autonomous\n  args:\n"
        r"    node_name_suffix: scenario_planning_trajectory\n)"
        r"(?P<body>.*?)(?=\n- module:|\Z)",
        re.DOTALL,
    )
    matches = list(pattern.finditer(source))
    if len(matches) != 1:
        raise ValueError("Expected one scenario planning trajectory monitor")
    match = matches[0]
    body = match.group("body")
    for key, value in (("warn_rate", "0.5"), ("error_rate", "0.25"), ("timeout", "4.0")):
        body, count = re.subn(rf"(?m)^(    {key}: )[^\n]+$", rf"\g<1>{value}", body)
        if count != 1:
            raise ValueError(f"Expected one {key} in trajectory monitor")
    return source[:match.start("body")] + body + source[match.end("body"):]


def adjust_planning_graph(source):
    pattern = re.compile(r"(?m)^(    node: planning_validator\n    name: trajectory_validation_[^\n]+)$")
    result, count = pattern.subn(r"\1\n    timeout: 4.0", source)
    if count != 10:
        raise ValueError(f"Expected ten validator graph leaves, found {count}")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for filename, adjust in (
        ("topics.yaml", adjust_topics),
        ("planning.yaml", adjust_planning_graph),
    ):
        source = (args.source_dir / filename).read_text()
        (args.output_dir / filename).write_text(adjust(source))


if __name__ == "__main__":
    main()
