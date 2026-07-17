#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Render shape-task target snapshots from the DRAGON FK model.

The images show the DRAGON link chain at each q_star in
config/shape_task/targets.yaml, expressed in the link4-root frame used by the
shape-task shadow and E_s metric.
"""

import ast
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
DRACOMANCER = os.path.join(ROOT, "robots", "dracomancer")
SHAPE_TASK = os.path.join(DRACOMANCER, "scripts", "shape_task")
sys.path.insert(0, SHAPE_TASK)

import dragon_fk  # noqa: E402


TARGETS_YAML = os.path.join(DRACOMANCER, "config", "shape_task", "targets.yaml")
OUT_DIR = os.path.join(DRACOMANCER, "figures", "shape_task_targets")


def load_targets(path):
    targets = []
    current = None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            name_match = re.match(r"\s*-\s+name:\s*(.+?)\s*$", line)
            if name_match:
                if current is not None:
                    targets.append(current)
                current = {"name": name_match.group(1)}
                continue
            q_match = re.match(r"\s*q_star:\s*(\[.*\])\s*$", line)
            if q_match and current is not None:
                current["q_star"] = [float(v) for v in ast.literal_eval(q_match.group(1))]
        if current is not None:
            targets.append(current)
    return [t for t in targets if "name" in t and "q_star" in t]


def set_equal_axes(ax, all_points):
    mins = all_points.min(axis=0)
    maxs = all_points.max(axis=0)
    center = 0.5 * (mins + maxs)
    radius = 0.55 * max(maxs - mins)
    radius = max(radius, 0.25)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def draw_shape(ax, pts, name, q_star, color):
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-", color=color, linewidth=5)
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=70, color=color, edgecolor="black", linewidth=0.8)
    labels = ["link4", "joint3", "joint2", "joint1", "link1 tip"]
    for label, p in zip(labels, pts):
        ax.text(p[0], p[1], p[2] + 0.035, label, fontsize=8)
    ax.set_title(name, fontsize=13, fontweight="bold", pad=12)
    ax.text2D(
        0.02,
        0.94,
        "q* = [" + ", ".join("%.1f" % v for v in q_star) + "] rad",
        transform=ax.transAxes,
        fontsize=8,
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.view_init(elev=24, azim=-58)
    ax.grid(True, alpha=0.25)


def save(fig, path):
    fig.tight_layout(pad=2.0)
    fig.savefig(path + ".png", dpi=220, bbox_inches="tight", pad_inches=0.12)
    fig.savefig(path + ".svg", bbox_inches="tight", pad_inches=0.12)
    plt.close(fig)
    print("wrote", path + ".png")
    print("wrote", path + ".svg")


def main():
    targets = load_targets(TARGETS_YAML)
    if not targets:
        raise RuntimeError("no targets found in %s" % TARGETS_YAML)
    os.makedirs(OUT_DIR, exist_ok=True)

    target_points = []
    for target in targets:
        pts = dragon_fk.fk_points(target["q_star"])
        target["points"] = pts
        target_points.append(pts)
    all_points = np.vstack(target_points)
    colors = ["#2070b4", "#d55e00", "#009e73"]

    for i, target in enumerate(targets):
        fig = plt.figure(figsize=(6.2, 5.2))
        ax = fig.add_subplot(111, projection="3d")
        draw_shape(ax, target["points"], target["name"], target["q_star"], colors[i % len(colors)])
        set_equal_axes(ax, all_points)
        safe_name = re.sub(r"[^A-Za-z0-9]+", "_", target["name"]).strip("_").lower()
        save(fig, os.path.join(OUT_DIR, safe_name))

    fig = plt.figure(figsize=(15, 5))
    for i, target in enumerate(targets):
        ax = fig.add_subplot(1, len(targets), i + 1, projection="3d")
        draw_shape(ax, target["points"], target["name"], target["q_star"], colors[i % len(colors)])
        set_equal_axes(ax, all_points)
    save(fig, os.path.join(OUT_DIR, "overview"))


if __name__ == "__main__":
    main()
