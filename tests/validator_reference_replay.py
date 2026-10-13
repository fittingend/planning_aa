"""원본 validator의 전체 상태값과 독립 C++ 계산 결과를 비교합니다."""

import csv
import gzip
import json
import subprocess
import sys
import tempfile
from pathlib import Path


def main():
    fixture = Path(sys.argv[2])
    if fixture.suffix == ".gz":
        with gzip.open(fixture, "rt") as stream:
            records = json.load(stream)
    else:
        records = json.loads(fixture.read_text())
    with tempfile.TemporaryDirectory() as directory:
        input_path = Path(directory) / "trajectory.csv"
        with input_path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            for index, record in enumerate(records):
                ego = record["ego"]
                for point in record["trajectory"]:
                    writer.writerow((
                        index, point["x"], point["y"], point["z"],
                        point["orientation_x"], point["orientation_y"],
                        point["orientation_z"], point["orientation_w"],
                        point["velocity"], point["heading_rate"],
                        point["acceleration"], ego["x"], ego["y"], ego["velocity"],
                    ))
        process = subprocess.run(
            (sys.argv[1], str(input_path)), capture_output=True, text=True, check=True
        )
    actual = [{} for _ in records]
    for index, name, value in csv.reader(process.stdout.splitlines()):
        actual[int(index)][name] = float(value)
    for record, observed in zip(records, actual):
        expected = record["status"]
        assert observed.keys() == expected.keys(), (record["case"], observed.keys())
        largest = (0.0, "")
        for name, want in expected.items():
            difference = abs(observed[name] - want)
            if difference > largest[0]:
                largest = (difference, name)
            assert difference < 1.0e-4, (
                record["case"], name, observed[name], want, difference
            )
        print(f"{record['case']}: {len(expected)}개 상태값 일치, 최대 차이 {largest}")


if __name__ == "__main__":
    main()
