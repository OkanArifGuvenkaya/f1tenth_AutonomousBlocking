#!/usr/bin/env python3
"""Read raceline_{1,2,3}.csv, add 1.0 to last column, save to target directory."""
import csv
from pathlib import Path
import argparse

increase_speed = 0.2

def process_file(in_path: Path, out_path: Path):
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with in_path.open('r', newline='') as f_in, out_path.open('w', newline='') as f_out:
        reader = csv.reader(f_in)
        writer = csv.writer(f_out)
        for row in reader:
            if not row:
                continue
            # try to parse third column
            if len(row) >= 3:
                try:
                    val = float(row[2])
                    val += increase_speed
                    row[2] = f"{val:.4f}"
                except ValueError:
                    pass
            writer.writerow(row)

def main():
    parser = argparse.ArgumentParser(description="Increment third column of raceline CSVs and save copies.")
    parser.add_argument('--in-dir', type=Path, default=Path(__file__).resolve().parents[1] / 'src' / 'pure_pursuit' / 'racelines')
    parser.add_argument('--out-dir', type=Path, default=Path('/home/okanarif/repositories/f1tenth_AutonomousBlocking/lab_ws25/src/pure_pursuit/reacelines_2'))
    args = parser.parse_args()

    files = [args.in_dir / f"raceline_{i}.csv" for i in (1,2,3)]
    for f in files:
        if not f.exists():
            print(f"Skipping missing file: {f}")
            continue
        out = args.out_dir / f.name
        process_file(f, out)
        print(f"Wrote: {out}")

if __name__ == '__main__':
    main()
