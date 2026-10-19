#!/usr/bin/env python3
"""
run_error_split.py -- 오차를 알고리즘, 센서, 위치 추정, 기계로 나눠 재기 (계획서 6-라)

  python run_error_split.py                          # 무작위 지도 6개, 우리 차
  python run_error_split.py --map arena --count 10   # car_specs 의 시험 공간 크기로 만든 무작위 배치
  python run_error_split.py --map layouts/day1.txt   # 실제로 잰 배치 파일 (형식은 error_split.load_layout)
  python run_error_split.py --map layouts/day1.txt --real-track tracks/day1_run1.txt
                                                     # 실차 궤적까지 붙여 4 단계를 채움
  python run_error_split.py --save                   # 궤적 그림을 recordings/error_split 에 저장

주요 옵션
  --car       ours(기본, car_specs.py 의 우리 차) 또는 sim(시뮬레이터 기본 차)
  --count     지도 몇 개를 돌릴지
  --seed      첫 지도 번호
"""

import argparse
import os

import numpy as np

from rccar import car_specs
from rccar import error_split as E
from rccar import scenarios as S
from rccar.vehicle import VehicleParams
from rccar.world import World

OUT_DIR = os.path.join("recordings", "error_split")


def arena_layout(seed):
    """car_specs 의 시험 공간 크기로 계획서대로 기둥 3~6개를 무작위로 놓음. 출발은 왼쪽, 목표는 오른쪽."""
    rng = np.random.default_rng(50_000 + seed)
    width, length = car_specs.arena_m()
    radius = car_specs.pillar_radius_m()
    start = np.array([0.15 * width, rng.uniform(0.3, 0.7) * length, 0.0])
    goal = np.array([0.85 * width, rng.uniform(0.3, 0.7) * length])
    count = int(rng.integers(3, 7))
    circles = []
    while len(circles) < count:
        center = rng.uniform([0.3 * width, 0.1 * length], [0.7 * width, 0.9 * length])
        if any(np.hypot(*(center - np.asarray(other))) < 2 * radius + 0.05 for other, _ in circles):
            continue
        circles.append((tuple(center), radius))
    return World(circles=circles, bounds=(-16, -16, 16, 16)), start, goal


def maps_for(args):
    if args.map == "random":
        return [(f"random{seed}", *S.make("random", seed))
                for seed in range(args.seed, args.seed + args.count)]
    if args.map == "arena":
        return [(f"arena{seed}", *arena_layout(seed))
                for seed in range(args.seed, args.seed + args.count)]
    return [(os.path.basename(args.map), *E.load_layout(args.map))]


def print_map(name, metrics, extra=None):
    print(f"\n{name}")
    print(f"  {'단계':<10} {'성공':>4} {'길이 비':>8} {'평균 거리(m)':>12} {'최대 거리(m)':>12} {'도달 오차(m)':>12}")
    rows = list(zip(E.STAGES, metrics))
    if extra is not None:
        rows.append(("4 실차", extra))
    for label, m in rows:
        print(f"  {label:<10} {'O' if m['success'] else 'X':>4} {m['length_ratio']:>8.3f} "
              f"{m['gap_mean']:>12.3f} {m['gap_max']:>12.3f} {m['final_error']:>12.3f}")


def print_summary(all_metrics, real=None):
    """모든 단계가 성공한 지도만 모아 평균을 내고, 앞 단계보다 늘어난 만큼을 원인별로 나눔."""
    usable = [m for m in all_metrics if all(stage["success"] for stage in m)]
    print(f"\n== 요약: 지도 {len(all_metrics)}개 중 모든 단계가 도착한 {len(usable)}개로 계산 ==")
    for k, label in enumerate(E.STAGES):
        rate = np.mean([m[k]["success"] for m in all_metrics])
        print(f"  {label:<10} 도달 성공률 {rate * 100:5.1f}%")
    if not usable:
        return
    ratio = np.array([[m[k]["length_ratio"] for k in range(len(E.STAGES))] for m in usable])
    gap = np.array([[m[k]["gap_mean"] for k in range(len(E.STAGES))] for m in usable])
    final = np.array([[m[k]["final_error"] for k in range(len(E.STAGES))] for m in usable])
    print(f"\n  {'원인':<10} {'길이 비 증가':>12} {'평균 거리 증가(m)':>18} {'도달 오차 증가(m)':>18}")
    for k, cause in enumerate(E.CAUSES, start=1):
        print(f"  {cause:<10} {np.mean(ratio[:, k] - ratio[:, k - 1]):>+12.4f} "
              f"{np.mean(gap[:, k] - gap[:, k - 1]):>+18.4f} {np.mean(final[:, k] - final[:, k - 1]):>+18.4f}")
    if real is not None:
        last = usable[-1][-1]
        print(f"  {'기계·제어':<10} {real['length_ratio'] - last['length_ratio']:>+12.4f} "
              f"{real['gap_mean'] - last['gap_mean']:>+18.4f} {real['final_error'] - last['final_error']:>+18.4f}")
    total = np.mean(ratio[:, -1] - 1.0)
    print(f"\n  기준보다 길어진 전체 {total * 100:+.2f}% 중"
          + "".join(f"  {cause} {np.mean(ratio[:, k] - ratio[:, k - 1]) * 100:+.2f}%p"
                    for k, cause in enumerate(E.CAUSES, start=1)))


def save_figure(name, world, start, goal, runs, real_track=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    installed = {font.name for font in font_manager.fontManager.ttflist}
    korean = [font for font in ("Malgun Gothic", "AppleGothic", "NanumGothic") if font in installed]
    plt.rcParams["font.family"] = korean[:1] + ["sans-serif"]
    figure, axis = plt.subplots(figsize=(7, 7))
    for center, radius in world.circles:
        axis.add_patch(plt.Circle(center, radius, color="0.55"))
    for a, b in world.walls:
        axis.plot([a[0], b[0]], [a[1], b[1]], color="0.3", lw=2)
    colors = ("#222222", "#2a9d8f", "#e9c46a", "#d1495b")
    for run, label, color in zip(runs, E.STAGES, colors):
        axis.plot(*run["trail"].T, color=color, lw=2.2 if run["stage"] == 0 else 1.4,
                  ls="-" if run["stage"] == 0 else "--", label=label)
    if real_track is not None:
        axis.plot(*real_track.T, color="#1f6fb2", lw=1.8, label="4 실차")
    axis.plot(start[0], start[1], "o", color="#2a9d8f", ms=8)
    axis.plot(goal[0], goal[1], "*", color="#e9c46a", ms=14, mec="0.3")
    every = np.vstack([run["trail"] for run in runs])
    low, high = every.min(axis=0) - 0.5, every.max(axis=0) + 0.5
    axis.set_xlim(low[0], high[0])
    axis.set_ylim(low[1], high[1])
    axis.set_aspect("equal")
    axis.grid(alpha=0.2)
    axis.legend(loc="best", fontsize=9)
    axis.set_title(f"{name}: 단계별 궤적", fontsize=11)
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, f"{name}.png")
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)
    print("  그림 저장:", path)


def main():
    parser = argparse.ArgumentParser(description="오차 분해")
    parser.add_argument("--map", default="random", help="random, arena, 또는 배치 파일 경로")
    parser.add_argument("--count", type=int, default=6)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--car", default="ours", choices=("ours", "sim"))
    parser.add_argument("--real-track", default=None, help="실차 궤적 파일 (배치 파일과 함께)")
    parser.add_argument("--max-time", type=float, default=120.0)
    parser.add_argument("--save", action="store_true")
    args = parser.parse_args()

    make_vehicle = car_specs.vehicle_params if args.car == "ours" else VehicleParams
    all_metrics, real = [], None
    for name, world, start, goal in maps_for(args):
        runs, metrics = E.split_one_map(world, start, goal, args.seed, make_vehicle, args.max_time)
        track = None
        if args.real_track:
            track = E.load_track(args.real_track)
            real = E.real_track_metrics(track, runs[0], goal)
        print_map(name, metrics, real)
        all_metrics.append(metrics)
        if args.save:
            save_figure(name, world, start, goal, runs, track)
    print_summary(all_metrics, real)


if __name__ == "__main__":
    main()
