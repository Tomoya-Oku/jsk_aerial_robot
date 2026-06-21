#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Derive Dracomancer safety thresholds [hard_min, min] from collected fc data.

Reads the shape-sweep CSV produced by collect_fc_data.py and proposes:
  hard_min : danger floor  -> a low percentile of the fc distribution (just above
             the feasibility "cliff"), optionally scaled by a safety factor.
  min      : full-safe point -> a fraction of the typical (median) fc.

This is a pure offline analysis (no ROS), so it can be re-run on any CSV.
Usage: analyze_fc_data.py <csv> [--hard-pct 5] [--min-frac 0.8]
"""

import sys
import csv
import argparse
import statistics as st


def load(path):
    f, t = [], []
    with open(path) as fp:
        for row in csv.DictReader(fp):
            try:
                f.append(float(row["fc_f_min_mean"]))
                t.append(float(row["fc_t_min_mean"]))
            except (KeyError, ValueError):
                pass
    return f, t


def pct(xs, p):
    xs = sorted(xs)
    if not xs:
        return float("nan")
    k = (len(xs) - 1) * p / 100.0
    lo = int(k)
    hi = min(lo + 1, len(xs) - 1)
    return xs[lo] + (xs[hi] - xs[lo]) * (k - lo)


def summarize(name, xs):
    print("\n[%s]  n=%d" % (name, len(xs)))
    if not xs:
        return
    print("  min=%.4f  p5=%.4f  p25=%.4f  median=%.4f  p75=%.4f  max=%.4f  mean=%.4f std=%.4f"
          % (min(xs), pct(xs, 5), pct(xs, 25), st.median(xs), pct(xs, 75),
             max(xs), st.mean(xs), st.pstdev(xs)))


def propose(name, xs, hard_pct, min_frac, safety):
    if not xs:
        return None
    cliff = pct(xs, hard_pct)
    hard_min = max(0.0, cliff * safety)
    soft_min = st.median(xs) * min_frac
    if soft_min < hard_min:
        soft_min = hard_min
    print("  -> %s: hard_min=%.4f (p%g x%.2f), min=%.4f (median x%.2f)"
          % (name, hard_min, hard_pct, safety, soft_min, min_frac))
    return hard_min, soft_min


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--hard-pct", type=float, default=5.0)
    ap.add_argument("--min-frac", type=float, default=0.8)
    ap.add_argument("--safety", type=float, default=1.0,
                    help="multiply the cliff percentile to get hard_min")
    a = ap.parse_args()

    f, t = load(a.csv)
    summarize("fc_f_min", f)
    summarize("fc_t_min", t)
    print("\nProposed thresholds [hard_min, min]:")
    propose("force ", f, a.hard_pct, a.min_frac, a.safety)
    propose("torque", t, a.hard_pct, a.min_frac, a.safety)


if __name__ == "__main__":
    main()
