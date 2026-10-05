#!/usr/bin/env python3
import argparse
import csv
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# look at bench
NUMERIC_FIELDS = {
    "eps", "ok", "len_m", "first_ms", "median_ms", "p95_ms", "check_ms",
    "build_ms", "search_ms", "convert_ms", "corner_ms", "line_ms", "expand",
    "succ", "jump_steps", "cell_queries", "heap_push",
}

def load_rows(paths):
    rows = []
    for path in paths:
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                for k in NUMERIC_FIELDS & row.keys():
                    try:
                        row[k] = float(row[k])
                    except ValueError:
                        pass
                rows.append(row)
    return rows

def sorted_unique(values):
    """Numeric sort for case names like '5m'/'15m'/'30m'; falls back to a
    plain string sort for non-numeric names like 'weave'/'nopath'."""
    def key(v):
        digits = "".join(ch for ch in v if ch.isdigit() or ch == ".")
        return (0, float(digits)) if digits else (1, v)
    return sorted(set(values), key=key)

def plot_scene(rows, scene, metric, err_metric, out_dir):
    scene_rows = [r for r in rows if r["scene"] == scene]
    if not scene_rows:
        return None

    backends = sorted_unique(r["backend"] for r in scene_rows)
    cases = sorted_unique(r["case"] for r in scene_rows)
    modes = sorted_unique(r["mode"] for r in scene_rows)
    algo_eps = sorted(set((r["algo"], r["eps"]) for r in scene_rows),
                       key=lambda ae: (ae[0], ae[1]))
    series_keys = [(a, e, m) for (a, e) in algo_eps for m in modes]

    fig, axes = plt.subplots(1, len(backends), figsize=(6.5 * len(backends), 4.5),
                              squeeze=False, sharey=True)
    axes = axes[0]

    cmap = plt.get_cmap("tab10")
    color = {key: cmap(i % 10) for i, key in enumerate(series_keys)}
    bar_w = 0.8 / max(len(series_keys), 1)

    def label(algo, eps, mode):
        n_eps = len({e for (a, e) in algo_eps if a == algo})
        algo_label = f"{algo}(e{eps:g})" if n_eps > 1 else algo
        return f"{algo_label}/{mode}"

    for ax, backend in zip(axes, backends):
        by_key = {(r["case"], r["algo"], r["eps"], r["mode"]): r
                  for r in scene_rows if r["backend"] == backend}
        for si, (algo, eps, mode) in enumerate(series_keys):
            ys, yerr = [], []
            for case in cases:
                r = by_key.get((case, algo, eps, mode))
                ys.append(r[metric] if r else float("nan"))
                yerr.append(max(r[err_metric] - r[metric], 0) if r and err_metric else 0)
            xs = [i + si * bar_w for i in range(len(cases))]
            ax.bar(xs, ys, width=bar_w, label=label(algo, eps, mode),
                   color=color[(algo, eps, mode)],
                   yerr=yerr if err_metric else None, capsize=2)
        ax.set_xticks([i + bar_w * (len(series_keys) - 1) / 2
                       for i in range(len(cases))])
        ax.set_xticklabels(cases, rotation=30, ha="right")
        ax.set_title(f"{scene} / {backend}")
        ax.set_ylabel(metric + (f" (whisker: {err_metric})" if err_metric else ""))
        ax.grid(axis="y", alpha=0.3)

    axes[-1].legend(fontsize=8, loc="upper left", bbox_to_anchor=(1.02, 1.0))
    fig.tight_layout()
    out_path = os.path.join(out_dir, f"{scene}_{metric}.png")
    fig.savefig(out_path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return out_path


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="bench_jps_rog_map CSV file(s)")
    ap.add_argument("-o", "--out-dir", default=".", help="where to write PNGs")
    ap.add_argument("-m", "--metric", action="append",
                     help="numeric column to plot (repeatable); default: median_ms "
                          "(with p95_ms as a whisker)")
    args = ap.parse_args()

    rows = load_rows(args.csv)
    if not rows:
        print("no rows found", file=sys.stderr)
        return 1

    metrics = args.metric or ["median_ms"]
    scenes = sorted_unique(r["scene"] for r in rows)
    os.makedirs(args.out_dir, exist_ok=True)

    written = []
    for scene in scenes:
        for metric in metrics:
            err = "p95_ms" if metric == "median_ms" else None
            path = plot_scene(rows, scene, metric, err, args.out_dir)
            if path:
                written.append(path)

    for p in written:
        print(f"wrote {p}")
    if not written:
        print("nothing plotted -- check --metric / csv contents", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
