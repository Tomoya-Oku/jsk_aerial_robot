#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Plots for the shape-target reaching task results.

Reads a results directory produced by scripts/shape_task/ (trial_XXX_raw.csv,
summary_metrics.csv, session.yaml) and writes PNGs into <results>/plots/:

  joint_error_trial_XXX.png   E_q with the success threshold
  shape_error_trial_XXX.png   E_s with the success threshold
  safety_margin_trial_XXX.png mu with the mu=0 danger line
  gate_state_trial_XXX.png    r_f / r_tau vs thresholds + gate_rejected band
  summary_completion_time.png / summary_joint_error.png

Usage: ./plot_task.py <results_dir>
Thresholds are taken from session.yaml when present.
"""

import argparse
import csv
import glob
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

try:
    import yaml
except ImportError:
    yaml = None

DEFAULTS = {
    "E_q_threshold": 0.15, "E_s_threshold": 0.05,
    "r_f_hard": 0.108990, "r_f_safe": 0.249220,
    "r_tau_hard": 0.015400, "r_tau_safe": 0.278159,
}


def to_float(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def load_csv(path):
    with open(path, "r", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    return {k: [r.get(k, "") for r in rows] for k in rows[0]}


def load_params(results_dir):
    params = dict(DEFAULTS)
    path = os.path.join(results_dir, "session.yaml")
    if yaml is not None and os.path.exists(path):
        try:
            with open(path) as f:
                session = yaml.safe_load(f) or {}
            params.update({k: v for k, v in (session.get("params") or {}).items()
                           if k in DEFAULTS})
        except yaml.YAMLError as e:
            print("session.yaml unreadable (%s); using defaults" % e)
    return params


def save(fig, plots_dir, name):
    path = os.path.join(plots_dir, name)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print("wrote", path)


def plot_trial(raw_path, plots_dir, params):
    tag = re.search(r"trial_(\d+)_raw", os.path.basename(raw_path)).group(1)
    data = load_csv(raw_path)
    if not data:
        print("empty raw csv:", raw_path)
        return
    t = np.array([to_float(v) for v in data["time"]])
    t = t - t[0] if len(t) else t
    target = data.get("target_name", [""])[0]

    def series(key):
        return np.array([to_float(v) for v in data.get(key, [])])

    # E_q
    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.plot(t, series("E_q"), label="E_q")
    ax.axhline(params["E_q_threshold"], color="r", ls="--", lw=1,
               label="threshold %.2f" % params["E_q_threshold"])
    ax.set_xlabel("time [s]"), ax.set_ylabel("E_q [rad]")
    ax.set_title("Joint error, trial %s (%s)" % (tag, target))
    ax.legend(), ax.grid(alpha=0.3)
    save(fig, plots_dir, "joint_error_trial_%s.png" % tag)

    # E_s
    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.plot(t, series("E_s"), color="tab:orange", label="E_s")
    ax.axhline(params["E_s_threshold"], color="r", ls="--", lw=1,
               label="threshold %.2f" % params["E_s_threshold"])
    ax.set_xlabel("time [s]"), ax.set_ylabel("E_s [m]")
    ax.set_title("Shape error, trial %s (%s)" % (tag, target))
    ax.legend(), ax.grid(alpha=0.3)
    save(fig, plots_dir, "shape_error_trial_%s.png" % tag)

    # mu
    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.plot(t, series("mu"), color="tab:green", label="mu")
    ax.axhline(0.0, color="r", ls="--", lw=1, label="danger (mu=0)")
    ax.axhline(1.0, color="gray", ls=":", lw=1, label="safe (mu=1)")
    ax.set_xlabel("time [s]"), ax.set_ylabel("safety margin mu [-]")
    ax.set_title("Safety margin, trial %s (%s)" % (tag, target))
    ax.legend(), ax.grid(alpha=0.3)
    save(fig, plots_dir, "safety_margin_trial_%s.png" % tag)

    # gate: measured radii vs thresholds, with rejected band
    fig, axes = plt.subplots(2, 1, figsize=(7, 5), sharex=True)
    for ax, key, hard, safe, unit in (
            (axes[0], "r_f", params["r_f_hard"], params["r_f_safe"], "N"),
            (axes[1], "r_tau", params["r_tau_hard"], params["r_tau_safe"], "N·m")):
        ax.plot(t, series(key), label=key)
        ax.axhline(hard, color="r", ls="--", lw=1, label="hard %.3f" % hard)
        ax.axhline(safe, color="g", ls=":", lw=1, label="safe %.3f" % safe)
        ax.set_ylabel("%s [%s]" % (key, unit))
        ax.legend(fontsize=8), ax.grid(alpha=0.3)
    gate = np.array([to_float(v) for v in data.get("gate_rejected", [])])
    if np.any(gate == 1):
        for ax in axes:
            ax.fill_between(t, *ax.get_ylim(), where=gate == 1,
                            color="red", alpha=0.15, label="gate rejected")
    axes[1].set_xlabel("time [s]")
    axes[0].set_title("Gate state, trial %s (%s)" % (tag, target))
    save(fig, plots_dir, "gate_state_trial_%s.png" % tag)


def plot_summary(results_dir, plots_dir):
    path = os.path.join(results_dir, "summary_metrics.csv")
    if not os.path.exists(path):
        print("no summary_metrics.csv; skip summary plots")
        return
    data = load_csv(path)
    trials = ["%s\n%s" % (t, n) for t, n in
              zip(data.get("trial", []), data.get("target_name", []))]
    success = [str(s).lower() in ("true", "1") for s in data.get("success", [])]
    colors = ["tab:green" if s else "tab:red" for s in success]

    comp = np.array([to_float(v) for v in data.get("completion_time", [])])
    fig, ax = plt.subplots(figsize=(max(5, 1.2 * len(trials)), 3.5))
    ax.bar(range(len(trials)), np.where(np.isnan(comp), 0.0, comp), color=colors)
    ax.set_xticks(range(len(trials))), ax.set_xticklabels(trials, fontsize=8)
    ax.set_ylabel("completion time [s]")
    ax.set_title("Completion time (green=success, red=fail)")
    ax.grid(axis="y", alpha=0.3)
    save(fig, plots_dir, "summary_completion_time.png")

    eq = np.array([to_float(v) for v in data.get("final_E_q", [])])
    fig, ax = plt.subplots(figsize=(max(5, 1.2 * len(trials)), 3.5))
    ax.bar(range(len(trials)), np.where(np.isnan(eq), 0.0, eq), color=colors)
    ax.set_xticks(range(len(trials))), ax.set_xticklabels(trials, fontsize=8)
    ax.set_ylabel("final E_q [rad]")
    ax.set_title("Final joint error")
    ax.grid(axis="y", alpha=0.3)
    save(fig, plots_dir, "summary_joint_error.png")


def main():
    parser = argparse.ArgumentParser(description="shape task result plots")
    parser.add_argument("results_dir")
    args = parser.parse_args()
    results_dir = os.path.expanduser(args.results_dir)
    if not os.path.isdir(results_dir):
        sys.exit("not a directory: %s" % results_dir)
    plots_dir = os.path.join(results_dir, "plots")
    os.makedirs(plots_dir, exist_ok=True)
    params = load_params(results_dir)
    for raw in sorted(glob.glob(os.path.join(results_dir, "trial_*_raw.csv"))):
        try:
            plot_trial(raw, plots_dir, params)
        except Exception as e:
            print("plot failed for %s: %s" % (raw, e))
    plot_summary(results_dir, plots_dir)


if __name__ == "__main__":
    main()
