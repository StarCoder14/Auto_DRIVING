#!/usr/bin/env python3
"""
run_circle_model.py -- 처음 생각한 기하 모델(계획서의 알고리즘 A) 실행

  python run_circle_model.py                       # 고정 배치 전부를 표로 봄
  python run_circle_model.py --layout cup --show   # 한 배치를 그림으로 봄
  python run_circle_model.py --save               # 고정 배치 그림을 recordings/circle_model 에 저장
  python run_circle_model.py --geogebra           # GeoGebra 로 맞춰 볼 숫자
  python run_circle_model.py --search 300         # 무작위 배치 300개에서 실패하는 배치 찾기

주요 옵션
  --sensor-range   원 표면까지 이 거리 안에 들어와야 보임 (m). 기본은 전부 보임
  --no-baseline    기준선 규칙을 끔 (반복이 끝나는 보장이 없어짐)
  --half-width     차 반폭 (m). 기둥 반지름에 더해서 부풀림
"""

import argparse
import os
from collections import Counter
from math import degrees, inf

from rccar import circle_model as M
from rccar import circle_layouts as L
from rccar.circle_avoider import AvoiderSettings, drive

OUT_DIR = os.path.join("recordings", "circle_model")

RESULT_LABEL = {
    "arrived": "도착",
    "stuck": "갇힘",
    "loop": "맴돎",
    "timeout": "걸음 초과",
    "goal_blocked": "목표가 장애물 안",
    "collision": "충돌",
}


def run_layout(layout, settings, half_width):
    return drive(layout.start, layout.goal, layout.inflated(half_width), settings)


def print_table(layouts, settings, half_width):
    print(f"{'배치':<12} {'결과':<14} {'경로(m)':>8} {'직선(m)':>8} {'비':>6} {'방향 바꿈':>6}  설명")
    for layout in layouts:
        trip = run_layout(layout, settings, half_width)
        ratio = f"{trip.length / trip.straight:.3f}" if trip.arrived else "-"
        print(f"{layout.name:<12} {RESULT_LABEL[trip.result]:<14} {trip.length:>8.3f} "
              f"{trip.straight:>8.3f} {ratio:>6} {trip.side_flips:>6}  {layout.description}")


def draw(layout, trip, half_width, axis):
    """실제 기둥은 회색, 부풀린 경계는 붉은 점선, 기준선은 회색 점선, 경로는 파란 선."""
    import matplotlib.patches as patches

    for pillar in layout.pillars:
        axis.add_patch(patches.Circle((pillar.x, pillar.y), pillar.r, color="0.55"))
        axis.add_patch(patches.Circle((pillar.x, pillar.y), pillar.r + half_width,
                                      fill=False, ls="--", color="#d1495b", lw=1))
    axis.plot([layout.start[0], layout.goal[0]], [layout.start[1], layout.goal[1]],
              ls=":", color="0.6", lw=1)
    xs, ys = zip(*trip.path)
    axis.plot(xs, ys, color="#1f6fb2", lw=2)
    axis.plot(*layout.start, "o", color="#2a9d8f", ms=8)
    axis.plot(*layout.goal, "*", color="#e9c46a", ms=14, mec="0.3")
    for step_number, kind, _ in trip.events:
        if kind in ("hit", "leave", "switch") and step_number < len(trip.path):
            point = trip.path[min(step_number, len(trip.path) - 1)]
            axis.plot(*point, "x" if kind == "hit" else "^", color="0.2", ms=6)
    axis.set_xlim(-0.05, L.ARENA + 0.05)
    axis.set_ylim(-0.05, L.ARENA + 0.05)
    axis.set_aspect("equal")
    axis.grid(alpha=0.2)
    axis.set_title(f"{layout.name}: {RESULT_LABEL[trip.result]}, 경로 {trip.length:.2f} m "
                   f"(직선의 {trip.length / trip.straight:.2f}배)", fontsize=10)


def show_or_save(layouts, settings, half_width, save):
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    installed = {font.name for font in font_manager.fontManager.ttflist}
    korean = [name for name in ("Malgun Gothic", "AppleGothic", "NanumGothic") if name in installed]
    plt.rcParams["font.family"] = korean[:1] + ["sans-serif"]
    plt.rcParams["axes.unicode_minus"] = False
    os.makedirs(OUT_DIR, exist_ok=True)
    for layout in layouts:
        trip = run_layout(layout, settings, half_width)
        figure, axis = plt.subplots(figsize=(6, 6))
        draw(layout, trip, half_width, axis)
        figure.tight_layout()
        if save:
            path = os.path.join(OUT_DIR, f"{layout.name}.png")
            figure.savefig(path, dpi=120)
            plt.close(figure)
            print("저장:", path)
    if not save:
        plt.show()


def print_geogebra(half_width):
    """장애물 하나짜리 예제를 손으로 작도할 수 있게 숫자로 풀어 씀."""
    layout = L.BY_NAME["one"]
    circle = layout.inflated(half_width)[0]
    start, goal = layout.start, layout.goal
    print("GeoGebra 로 맞춰 볼 숫자 (배치 one)\n")
    print(f"출발점 P = {start},  목표 G = {goal}")
    print(f"부풀린 원: (x - {circle.x:g})^2 + (y - {circle.y:g})^2 = {circle.r:.3f}^2"
          f"   (기둥 반지름 {layout.pillars[0].r:g} + 차 반폭 {half_width:g})")
    line = M.line_through(start, goal)
    print(f"\n직선 PG: {line}")
    print(f"판별식 D = {M.discriminant(line, circle):.6f}  (양수면 직선이 원을 뚫음)")
    foot = M.foot_of_perpendicular((circle.x, circle.y), line)
    print(f"중심에서 PG 에 내린 수선의 발 = ({foot[0]:.4f}, {foot[1]:.4f}), "
          f"P 와 G 사이인가: {M.is_between(foot, start, goal)}")
    print("\nP 에서 그은 접선")
    for line in M.tangent_lines_from(start, circle):
        touch = M.tangent_point(circle, line)
        print(f"  {line}   접점 ({touch[0]:.4f}, {touch[1]:.4f})")
    for side in ("left", "right"):
        touch = M.tangent_points_by_side(start, circle)[side]
        turn = M.TURN_OF_SIDE[side]
        leave = M.leave_point(circle, goal, turn)
        arc_angle = degrees(M.central_angle(circle, touch, leave, turn))
        side_name = "왼쪽" if side == "left" else "오른쪽"
        print(f"\n{side_name}으로 비켜 가기 ({'시계' if turn == 'clockwise' else '반시계'} 방향으로 돎)")
        print(f"  접점 T = ({touch[0]:.4f}, {touch[1]:.4f}),  떠나는 점 E = ({leave[0]:.4f}, {leave[1]:.4f})")
        print(f"  PT = {M.distance(start, touch):.4f},  호 TE = {M.arc_length(circle, touch, leave, turn):.4f}"
              f" (중심각 {arc_angle:.2f}도),  EG = {M.distance(leave, goal):.4f}")
        print(f"  합계 = {M.detour_length(start, goal, circle, side):.4f}")
    trip = drive(start, goal, [circle], AvoiderSettings(step=0.001))
    print(f"\n0.001 m 걸음으로 실제로 달린 길이 = {trip.length:.4f}")


def search_failures(count, settings, half_width):
    """무작위 배치에서 도착하지 못하는 경우를 모음 (계획서 목표 5)."""
    results = Counter()
    failures = []
    ratios = []
    for seed in range(count):
        layout = L.random_layout(seed)
        trip = run_layout(layout, settings, half_width)
        results[trip.result] += 1
        if trip.arrived:
            ratios.append(trip.length / trip.straight)
        else:
            failures.append((seed, trip.result, trip.events[-1][2] if trip.events else ""))
    print(f"무작위 배치 {count}개")
    for result, number in results.most_common():
        print(f"  {RESULT_LABEL[result]}: {number}")
    if ratios:
        ratios.sort()
        print(f"  도착한 경우 경로/직선 비: 중앙 {ratios[len(ratios) // 2]:.3f}, 최대 {ratios[-1]:.3f}")
    for seed, result, detail in failures[:15]:
        print(f"  random{seed}: {RESULT_LABEL[result]} — {detail}")
    return failures


def main():
    parser = argparse.ArgumentParser(description="처음 기하 모델 실행")
    parser.add_argument("--layout", default=None, help="배치 이름, 또는 random숫자 (예 random12)")
    parser.add_argument("--show", action="store_true", help="그림 창으로 봄")
    parser.add_argument("--save", action="store_true", help="그림을 파일로 저장")
    parser.add_argument("--geogebra", action="store_true")
    parser.add_argument("--search", type=int, default=0, help="무작위 배치 몇 개를 돌려 실패를 찾을지")
    parser.add_argument("--sensor-range", type=float, default=inf)
    parser.add_argument("--no-baseline", action="store_true")
    parser.add_argument("--half-width", type=float, default=L.CAR_HALF_WIDTH)
    parser.add_argument("--step", type=float, default=0.01)
    args = parser.parse_args()

    settings = AvoiderSettings(step=args.step, sensor_range=args.sensor_range,
                               use_baseline_rule=not args.no_baseline)
    if args.geogebra:
        print_geogebra(args.half_width)
        return
    if args.search:
        search_failures(args.search, settings, args.half_width)
        return

    if args.layout is None:
        layouts = L.FIXED
    elif args.layout.startswith("random"):
        layouts = [L.random_layout(int(args.layout[len("random"):]))]
    else:
        layouts = [L.BY_NAME[args.layout]]
    print_table(layouts, settings, args.half_width)
    if args.show or args.save:
        show_or_save(layouts, settings, args.half_width, save=args.save)


if __name__ == "__main__":
    main()
