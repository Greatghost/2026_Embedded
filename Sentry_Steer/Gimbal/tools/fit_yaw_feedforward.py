#!/usr/bin/env python3
"""Fit Sentry big-yaw J/B/C feedforward coefficients from a CSV log.

CSV units:
  yaw_speed_dps       actual yaw speed, deg/s
  yaw_accel_dps2      actual yaw acceleration, deg/s^2 (J/all modes)
  yaw_torque_model_mnm required t_ff command in model-positive direction, mN*m

The script uses only the Python standard library so it can run on the tuning PC.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable


def solve_linear(matrix: list[list[float]], vector: list[float]) -> list[float]:
    """Solve a small dense system with partial-pivot Gaussian elimination."""
    n = len(vector)
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    for column in range(n):
        pivot = max(range(column, n), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1.0e-12:
            raise ValueError("拟合矩阵奇异：正负速度/加减速激励不足")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(n):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(augmented[row], augmented[column])
            ]
    return [augmented[row][-1] for row in range(n)]


def least_squares(features: list[list[float]], targets: list[float]) -> list[float]:
    width = len(features[0])
    normal = [[0.0] * width for _ in range(width)]
    rhs = [0.0] * width
    for feature, target in zip(features, targets):
        for i in range(width):
            rhs[i] += feature[i] * target
            for j in range(width):
                normal[i][j] += feature[i] * feature[j]
    return solve_linear(normal, rhs)


def friction_ratio(speed_dps: float, blend_dps: float) -> float:
    if blend_dps <= 0.0:
        raise ValueError("--blend-dps 必须大于0")
    return max(-1.0, min(1.0, speed_dps / blend_dps))


def finite_float(row: dict[str, str], column: str) -> float:
    try:
        value = float(row[column])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"CSV缺少有效列 {column!r}") from error
    if not math.isfinite(value):
        raise ValueError(f"列 {column!r} 含NaN/Inf")
    return value


def read_rows(path: Path) -> Iterable[dict[str, str]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        yield from csv.DictReader(stream)


def fit(args: argparse.Namespace) -> tuple[list[str], list[float], list[float], list[float]]:
    names: list[str]
    features: list[list[float]] = []
    targets: list[float] = []

    if args.mode == "bc":
        names = ["B", "C", "bias"]
    elif args.mode == "j":
        if args.b is None or args.c is None:
            raise ValueError("j模式必须同时提供 --b 和 --c")
        names = ["J", "bias"]
    else:
        names = ["J", "B", "C", "bias"]

    for row in read_rows(args.csv):
        speed = finite_float(row, args.speed_column)
        torque = finite_float(row, args.torque_column)
        blend = friction_ratio(speed, args.blend_dps)

        if abs(speed) < args.min_speed:
            continue
        if args.saturation > 0.0 and abs(torque) >= args.saturation * 0.98:
            continue

        if args.mode == "bc":
            feature = [speed, blend, 1.0]
            target = torque
        else:
            accel = finite_float(row, args.accel_column)
            if abs(accel) < args.min_accel:
                continue
            if args.mode == "j":
                feature = [accel, 1.0]
                target = torque - args.b * speed - args.c * blend
            else:
                feature = [accel, speed, blend, 1.0]
                target = torque
        features.append(feature)
        targets.append(target)

    if len(features) < len(names) + 2:
        raise ValueError(f"有效样本仅{len(features)}个，激励或筛选条件不足")

    coefficients = least_squares(features, targets)
    predictions = [
        sum(value * coefficient for value, coefficient in zip(feature, coefficients))
        for feature in features
    ]
    return names, coefficients, targets, predictions


def print_result(
    names: list[str], coefficients: list[float], targets: list[float], predictions: list[float]
) -> None:
    residuals = [target - prediction for target, prediction in zip(targets, predictions)]
    rmse = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    mean = sum(targets) / len(targets)
    total = sum((value - mean) ** 2 for value in targets)
    r_squared = 1.0 - sum(value * value for value in residuals) / total if total else float("nan")

    print(f"samples={len(targets)}  rmse={rmse:.6g} mN*m  R2={r_squared:.6f}")
    result = dict(zip(names, coefficients))
    for name in names:
        unit = {
            "J": "mN*m/(deg/s^2)",
            "B": "mN*m/(deg/s)",
            "C": "mN*m",
            "bias": "mN*m",
        }[name]
        print(f"{name}={result[name]:.9g}  [{unit}]")

    print("\n建议写回（只写本次拟合得到的项）：")
    if "J" in result:
        print(f"#define GIMBAL_BIG_YAW_MODEL_FF_J {result['J']:.9g}f")
    if "B" in result:
        print(f"#define GIMBAL_BIG_YAW_MODEL_FF_B {result['B']:.9g}f")
    if "C" in result:
        print(f"#define GIMBAL_BIG_YAW_MODEL_FF_C {result['C']:.9g}f")
    if abs(result.get("bias", 0.0)) > max(20.0, rmse * 2.0):
        print("警告：bias较大；检查力矩符号、线缆预紧、零速偏置和正反不对称。")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--mode", choices=("bc", "j", "all"), default="bc")
    parser.add_argument("--blend-dps", type=float, default=6.0)
    parser.add_argument("--b", type=float, help="j模式中已标定的B")
    parser.add_argument("--c", type=float, help="j模式中已标定的C")
    parser.add_argument("--min-speed", type=float, default=10.0)
    parser.add_argument("--min-accel", type=float, default=10.0)
    parser.add_argument(
        "--saturation", type=float, default=0.0,
        help="命令绝对限幅(mN*m)；设置后剔除达到98%%限幅的样本",
    )
    parser.add_argument("--speed-column", default="yaw_speed_dps")
    parser.add_argument("--accel-column", default="yaw_accel_dps2")
    parser.add_argument("--torque-column", default="yaw_torque_model_mnm")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        names, coefficients, targets, predictions = fit(args)
        print_result(names, coefficients, targets, predictions)
    except (OSError, ValueError) as error:
        print(f"error: {error}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
