#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Metrics computation for the shape-target reaching task (offline, no ROS).

Reads a trial_XXX_raw.csv written by task_recorder.py and computes:

  success / completion_time  : from the trial_end event when available,
                               otherwise re-derived from the hold condition
  final_E_q [rad]            : RMS joint error at completion (or last sample)
  final_E_s [m]              : RMS link-node position error at the same time
  total_input_I [rad]        : sum_k ||q_exo[k] - q_exo[k-1]||
  response_delay_T_d [s]     : lag maximizing the cross-correlation between
                               |dq_exo/dt| and |dq_target/dt| (fallback q_current)
  mu_min                     : min of the normalized safety margin mu
  gate_count_N_s             : count of gate_rejected False->True transitions
  gate_time [s]              : cumulative time with gate_rejected True
  path_efficiency            : ||q_star - q_current[0]|| / joint-space path length

Missing topics are NaN in the raw CSV; every metric degrades to NaN instead of
raising. Usable as a module (task_recorder) or a CLI:

  ./compute_metrics.py trial_001_raw.csv [-o trial_001_metrics.csv]
"""

import argparse
import csv
import os
from collections import OrderedDict

import numpy as np

METRIC_KEYS = [
    "trial", "target_name", "success", "reason", "completion_time",
    "final_E_q", "final_E_s", "total_input_I", "response_delay_T_d",
    "mu_min", "gate_count_N_s", "gate_time", "path_efficiency",
    "duration", "n_samples",
]


def _to_float(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return float("nan")


def load_raw(path):
    """CSV -> dict of column name -> np.array (float) / list (str columns)."""
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return {}
    data = {}
    for key in rows[0].keys():
        if key in ("target_name", "gate_state"):
            data[key] = [r.get(key, "") for r in rows]
        else:
            data[key] = np.array([_to_float(r.get(key)) for r in rows])
    return data


def _matrix(data, prefix, n):
    cols = []
    for i in range(n):
        key = "%s_%d" % (prefix, i)
        if key not in data:
            return None
        cols.append(data[key])
    return np.stack(cols, axis=1)


def _path_length(q):
    """Sum of L2 norms of consecutive differences, skipping NaN rows."""
    valid = ~np.any(np.isnan(q), axis=1)
    qv = q[valid]
    if len(qv) < 2:
        return float("nan")
    return float(np.sum(np.linalg.norm(np.diff(qv, axis=0), axis=1)))


def _speed_signal(t, q):
    """|dq/dt| resampled onto a uniform grid; returns (t_uniform, speed) or None."""
    valid = ~np.any(np.isnan(q), axis=1) & ~np.isnan(t)
    if np.count_nonzero(valid) < 10:
        return None
    tv, qv = t[valid], q[valid]
    dt = np.diff(tv)
    good = dt > 1e-6
    if np.count_nonzero(good) < 10:
        return None
    speed = np.zeros(len(tv))
    speed[1:][good] = np.linalg.norm(np.diff(qv, axis=0), axis=1)[good] / dt[good]
    step = float(np.median(dt[good]))
    t_uni = np.arange(tv[0], tv[-1], step)
    if len(t_uni) < 10:
        return None
    return t_uni, np.interp(t_uni, tv, speed), step


def estimate_delay(t, q_in, q_out, max_delay=2.0):
    """Input->output delay via cross-correlation of joint-speed magnitudes."""
    sig_in = _speed_signal(t, q_in)
    sig_out = _speed_signal(t, q_out)
    if sig_in is None or sig_out is None:
        return float("nan")
    t_in, v_in, step = sig_in
    v_out = np.interp(t_in, sig_out[0], sig_out[1])
    v_in = v_in - np.mean(v_in)
    v_out = v_out - np.mean(v_out)
    if np.std(v_in) < 1e-9 or np.std(v_out) < 1e-9:
        return float("nan")
    max_lag = int(round(max_delay / step))
    best_lag, best_corr = 0, -np.inf
    for lag in range(0, max_lag + 1):
        if lag >= len(v_in) - 10:
            break
        a = v_in[:len(v_in) - lag]
        b = v_out[lag:]
        denom = np.std(a) * np.std(b)
        corr = float(np.mean(a * b) / denom) if denom > 1e-12 else -np.inf
        if corr > best_corr:
            best_corr, best_lag = corr, lag
    if not np.isfinite(best_corr):
        return float("nan")
    return best_lag * step


def _recompute_success(t, e_q, e_s, mu, params):
    """Fallback when no trial_end info: first time the hold condition completes."""
    eq_th = params.get("E_q_threshold", 0.15)
    es_th = params.get("E_s_threshold", 0.05)
    use_es = params.get("use_shape_error", False)
    t_hold = params.get("t_hold", 1.0)
    require_mu = params.get("require_safety_margin", True)
    ok = e_q < eq_th
    if use_es:
        ok &= e_s < es_th
    if require_mu:
        ok &= mu >= 0.0
    hold_start = None
    for i in range(len(t)):
        if ok[i] and not np.isnan(t[i]):
            if hold_start is None:
                hold_start = t[i]
            elif t[i] - hold_start >= t_hold:
                return True, float(t[i] - t[0])
        else:
            hold_start = None
    return False, float("nan")


def compute(data, info=None, params=None):
    """raw dict (+ trial_end info + threshold params) -> OrderedDict metrics."""
    info = info or {}
    params = params or {}
    m = OrderedDict((k, float("nan")) for k in METRIC_KEYS)
    if not data:
        return m

    t = data.get("time", np.array([]))
    n = len(t)
    m["n_samples"] = n
    m["trial"] = info.get("trial", float("nan"))
    names = data.get("target_name", [])
    m["target_name"] = info.get("target_name") or (names[-1] if names else "")
    m["reason"] = info.get("reason", "")
    m["duration"] = float(t[-1] - t[0]) if n > 1 else float("nan")

    n_joints = len(params.get("joint_names", [])) or 6
    q_star = _matrix(data, "q_star", n_joints)
    q_cur = _matrix(data, "q_current", n_joints)
    q_tgt = _matrix(data, "q_target", n_joints)
    q_exo = _matrix(data, "q_exo", 7)
    e_q = data.get("E_q", np.full(n, np.nan))
    e_s = data.get("E_s", np.full(n, np.nan))
    mu = data.get("mu", np.full(n, np.nan))

    # --- success / completion_time -----------------------------------------
    if "success" in info:
        m["success"] = bool(info["success"])
        m["completion_time"] = info.get("completion_time", float("nan"))
    else:
        m["success"], m["completion_time"] = _recompute_success(t, e_q, e_s, mu, params)

    # --- final errors: at completion when successful, else at the last sample
    if m["success"] and not np.isnan(m["completion_time"]) and n > 0:
        idx = int(np.searchsorted(t, t[0] + m["completion_time"], side="right")) - 1
        idx = max(0, min(idx, n - 1))
    else:
        idx = n - 1
    if n > 0:
        m["final_E_q"] = float(e_q[idx])
        m["final_E_s"] = float(e_s[idx])

    # --- operation input / delay --------------------------------------------
    if q_exo is not None:
        m["total_input_I"] = _path_length(q_exo)
        out = q_tgt if q_tgt is not None else q_cur
        if out is not None:
            m["response_delay_T_d"] = estimate_delay(
                t, q_exo, out, params.get("max_delay", 2.0))

    # --- safety --------------------------------------------------------------
    if np.any(~np.isnan(mu)):
        m["mu_min"] = float(np.nanmin(mu))
    gate = data.get("gate_rejected", np.full(n, np.nan))
    valid = ~np.isnan(gate)
    if np.any(valid):
        g = gate[valid].astype(int)
        m["gate_count_N_s"] = int(np.count_nonzero(np.diff(g) == 1) +
                                  (1 if len(g) and g[0] == 1 else 0))
        tv = t[valid]
        if len(tv) > 1:
            dt = np.diff(tv)
            m["gate_time"] = float(np.sum(dt[g[1:] == 1]))
        else:
            m["gate_time"] = 0.0

    # --- path efficiency -------------------------------------------------------
    if q_cur is not None and q_star is not None:
        end = idx + 1
        actual = _path_length(q_cur[:end])
        first_valid = np.where(~np.any(np.isnan(q_cur), axis=1))[0]
        star_valid = np.where(~np.any(np.isnan(q_star), axis=1))[0]
        if len(first_valid) and len(star_valid) and actual and actual > 1e-6:
            shortest = float(np.linalg.norm(
                q_star[star_valid[0]] - q_cur[first_valid[0]]))
            m["path_efficiency"] = shortest / actual
    return m


def write_metrics(metrics, path):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(list(metrics.keys()))
        writer.writerow([metrics[k] for k in metrics])


def append_summary(metrics, path):
    new_file = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow(list(metrics.keys()))
        writer.writerow([metrics[k] for k in metrics])


def compute_and_write(raw_path, metrics_path=None, info=None, params=None,
                      summary_path=None):
    data = load_raw(raw_path)
    metrics = compute(data, info=info, params=params)
    if metrics_path is None:
        metrics_path = raw_path.replace("_raw.csv", "_metrics.csv")
    write_metrics(metrics, metrics_path)
    if summary_path:
        append_summary(metrics, summary_path)
    return metrics


def main():
    parser = argparse.ArgumentParser(description="shape task trial metrics")
    parser.add_argument("raw_csv")
    parser.add_argument("-o", "--out", default=None, help="metrics CSV path")
    parser.add_argument("--eq-th", type=float, default=0.15)
    parser.add_argument("--es-th", type=float, default=0.05)
    parser.add_argument("--use-es", action="store_true",
                        help="include E_s in the recomputed success condition")
    parser.add_argument("--t-hold", type=float, default=1.0)
    parser.add_argument("--no-mu", action="store_true",
                        help="exclude mu>=0 from the recomputed success condition")
    args = parser.parse_args()
    params = {"E_q_threshold": args.eq_th, "E_s_threshold": args.es_th,
              "use_shape_error": args.use_es,
              "t_hold": args.t_hold, "require_safety_margin": not args.no_mu}
    metrics = compute_and_write(args.raw_csv, args.out, params=params)
    for k, v in metrics.items():
        print("%s: %s" % (k, v))


if __name__ == "__main__":
    main()
