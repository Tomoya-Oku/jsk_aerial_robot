#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import random
from pathlib import Path

try:
    import yaml
except ImportError as e:
    raise SystemExit("pyyaml が必要です: pip install pyyaml") from e


DEFAULT_WORKSPACE = {
    "x": (-0.450, 0.450),
    "y": (-0.450, 0.450),
    "z": (0.000, 0.500),
}

def gen_points(n: int, ws):
    pts = []
    for _ in range(n):
        x = random.uniform(*ws["x"])
        y = random.uniform(*ws["y"])
        z = random.uniform(*ws["z"])
        pts.append([float(x), float(y), float(z)])
    return pts

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--num", type=int, default=20, help="点の個数")
    ap.add_argument("-o", "--out", type=str, default="random_waypoints.yaml", help="出力yamlファイル名")

    ap.add_argument("--xmin", type=float, default=DEFAULT_WORKSPACE["x"][0])
    ap.add_argument("--xmax", type=float, default=DEFAULT_WORKSPACE["x"][1])
    ap.add_argument("--ymin", type=float, default=DEFAULT_WORKSPACE["y"][0])
    ap.add_argument("--ymax", type=float, default=DEFAULT_WORKSPACE["y"][1])
    ap.add_argument("--zmin", type=float, default=DEFAULT_WORKSPACE["z"][0])
    ap.add_argument("--zmax", type=float, default=DEFAULT_WORKSPACE["z"][1])

    ap.add_argument("--seed", type=int, default=None, help="乱数シード（再現性）")
    ap.add_argument("--style", choices=["list", "dict"], default="list",
                    help="YAMLの点表現: list -> [x,y,z], dict -> {x:..,y:..,z:..}")
    ap.add_argument("--digits", type=int, default=3, help="小数点以下桁数")

    args = ap.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    ws = {
        "x": (args.xmin, args.xmax),
        "y": (args.ymin, args.ymax),
        "z": (args.zmin, args.zmax),
    }

    pts = gen_points(args.num, ws)

    # 丸める
    d = int(args.digits)
    if args.style == "list":
        points = [[round(x, d), round(y, d), round(z, d)] for x, y, z in pts]
    else:
        points = [{"x": round(x, d), "y": round(y, d), "z": round(z, d)} for x, y, z in pts]

    data = {
        "frame_id": "world",
        "workspace": {"x": list(ws["x"]), "y": list(ws["y"]), "z": list(ws["z"])},
        "points": points,
    }

    out = Path(args.out)
    out.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print(f"Wrote: {out.resolve()}  (N={args.num}, style={args.style}, seed={args.seed})")

if __name__ == "__main__":
    main()
