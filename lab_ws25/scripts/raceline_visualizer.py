#!/usr/bin/env python3
"""Simple raceline visualizer.

Usage:
  python3 raceline_visualizer.py --dir ../src/pure_pursuit/racelines

Dependencies: numpy, matplotlib
"""
import argparse
import glob
import os
import sys

import numpy as np
import matplotlib.pyplot as plt


def load_raceline(path):
    try:
        data = np.loadtxt(path, delimiter=",")
    except Exception:
        data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[np.newaxis, :]
    if data.shape[1] < 2:
        raise ValueError(f"File {path} has too few columns")
    return data


def plot_racelines(files, show_indices=False, save_path=None):
    fig, ax = plt.subplots(figsize=(10, 8))
    cmap = plt.get_cmap("viridis")
    for i, f in enumerate(files):
        data = load_raceline(f)
        x = data[:, 0]
        y = data[:, 1]
        c = data[:, 2] if data.shape[1] >= 3 else None
        if c is None:
            ax.plot(x, y, label=os.path.basename(f))
        else:
            pts = np.vstack((x, y)).T
            sc = ax.scatter(x, y, c=c, cmap=cmap, s=8, label=os.path.basename(f))
            ax.plot(x, y, alpha=0.35)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Racelines")
    ax.legend(fontsize="small")
    if any((load_raceline(f).shape[1] >= 3) for f in files):
        cax = fig.colorbar(plt.cm.ScalarMappable(cmap=cmap), ax=ax)
        cax.set_label("3rd column (color)")
    if save_path:
        fig.savefig(save_path, dpi=200)
        print(f"Saved figure to {save_path}")
    plt.show()


def find_csv_files(directory):
    p = os.path.expanduser(directory)
    if os.path.isfile(p):
        return [p]
    files = sorted(glob.glob(os.path.join(p, "*.csv")))
    return files


def main():
    parser = argparse.ArgumentParser(description="Visualize raceline CSV files")
    # Default to the racelines folder next to the package in this repo
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_racelines = os.path.normpath(
        os.path.join(script_dir, "..", "src", "pure_pursuit", "racelines")
    )
    parser.add_argument(
        "--dir",
        "-d",
        default=default_racelines,
        help=f"Directory or single file containing raceline CSVs (default: {default_racelines})",
    )
    parser.add_argument("--save", "-s", help="Save plotted figure to this path (png/pdf)")
    args = parser.parse_args()
    files = find_csv_files(args.dir)
    # fallback: search for raceline CSVs elsewhere in workspace
    if not files:
        print(f"No CSV files found in {args.dir}, searching workspace for raceline CSVs...")
        root = os.path.abspath(os.path.join(script_dir, ".."))
        candidates = glob.glob(os.path.join(root, "**", "racelines", "*.csv"), recursive=True)
        if not candidates:
            candidates = glob.glob(os.path.join(root, "**", "*raceline*.csv"), recursive=True)
        files = sorted(candidates)
    if not files:
        print("No raceline CSV files found.")
        sys.exit(1)
    print(f"Found {len(files)} file(s):")
    for f in files:
        print(" -", f)
    plot_racelines(files, save_path=args.save)


if __name__ == "__main__":
    main()
