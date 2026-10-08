"""처음 기하 모델(계획서의 알고리즘 A) 검증.

계산 함수는 무작위 값을 수만 번 넣어 성질이 깨지는지 보고,
주행은 고정 배치와 무작위 배치에서 결과가 설명한 대로 나오는지 봄.

    python -m tests.test_circle_model
"""
from __future__ import annotations

import random
import sys
from math import hypot

from rccar import circle_layouts as L
from rccar import circle_model as M
from rccar.circle_avoider import AvoiderSettings, drive

RANDOM = random.Random(20261008)


def _random_circle():
    return M.Circle(RANDOM.uniform(-5, 5), RANDOM.uniform(-5, 5), RANDOM.uniform(0.05, 2.0))


def _point_outside(circle):
    while True:
        point = (RANDOM.uniform(-10, 10), RANDOM.uniform(-10, 10))
        if M.distance(point, (circle.x, circle.y)) > circle.r * 1.001:
            return point


def test_tangent_lines(n=20000):
    """접선은 중심에서 거리가 r 이고, 접점은 원 위에 있고, 반지름과 수직이어야 함."""
    for _ in range(n):
        circle = _random_circle()
        point = _point_outside(circle)
        lines = M.tangent_lines_from(point, circle)
        assert len(lines) == 2, f"접선이 2개가 아님: {len(lines)}"
        for line in lines:
            touch = M.tangent_point(circle, line)
            off_circle = abs(M.distance(touch, (circle.x, circle.y)) - circle.r)
            assert off_circle < 1e-6 * max(1.0, circle.r), f"접점이 원 위에 없음 ({off_circle:.2e})"
            radius = (touch[0] - circle.x, touch[1] - circle.y)
            along = (point[0] - touch[0], point[1] - touch[1])
            size = hypot(*radius) * hypot(*along)
            if size > 1e-9:
                cosine = abs(radius[0] * along[0] + radius[1] * along[1]) / size
                assert cosine < 1e-5, f"접선이 반지름과 수직이 아님 (cos={cosine:.2e})"


def test_vertical_tangent():
    """a^2 = r^2 이면 접선 하나가 수직선이라 기울기로 못 나타냄. 따로 처리한 게 맞는지 봄."""
    circle = M.Circle(0.0, 0.0, 1.0)
    lines = M.tangent_lines_from((1.0, 5.0), circle)
    assert any(line.is_vertical and abs(line.vertical_x - 1.0) < 1e-12 for line in lines), lines
    sloped = [line for line in lines if not line.is_vertical][0]
    assert abs(sloped.slope - 2.4) < 1e-12, sloped
    for line in lines:
        touch = M.tangent_point(circle, line)
        assert abs(M.distance(touch, (0.0, 0.0)) - 1.0) < 1e-9


def test_segment_check_matches_sampling(n=20000):
    """판별식 + 수선의 발로 낸 답이 선분 위 점을 촘촘히 찍어 본 답과 같아야 함."""
    disagree = 0
    for _ in range(n):
        circle = _random_circle()
        start, end = _point_outside(circle), _point_outside(circle)
        fast = M.segment_hits_circle(start, end, circle)
        samples = 400
        slow = any(M.is_inside(((start[0] * (samples - i) + end[0] * i) / samples,
                                (start[1] * (samples - i) + end[1] * i) / samples), circle, margin=1e-3)
                   for i in range(samples + 1))
        # 아슬아슬하게 스치는 경우는 촘촘히 찍어도 놓칠 수 있어서, 확실히 뚫는 경우만 비교함
        if slow and not fast:
            disagree += 1
    assert disagree == 0, f"확실히 뚫는데 안 뚫는다고 한 경우 {disagree}건"


def test_foot_is_perpendicular(n=20000):
    for _ in range(n):
        point = (RANDOM.uniform(-5, 5), RANDOM.uniform(-5, 5))
        start = (RANDOM.uniform(-5, 5), RANDOM.uniform(-5, 5))
        end = (RANDOM.uniform(-5, 5), RANDOM.uniform(-5, 5))
        if M.distance(start, end) < 1e-3:
            continue
        foot = M.foot_of_perpendicular(point, M.line_through(start, end))
        along = (end[0] - start[0], end[1] - start[1])
        drop = (point[0] - foot[0], point[1] - foot[1])
        dot = along[0] * drop[0] + along[1] * drop[1]
        assert abs(dot) < 1e-7 * max(1.0, hypot(*along) * hypot(*drop)), f"수선이 수직이 아님 ({dot:.2e})"


def test_circle_intersections(n=10000):
    """두 원의 교점은 두 원 위에 모두 있어야 함."""
    found = 0
    for _ in range(n):
        first, second = _random_circle(), _random_circle()
        for point in M.circle_intersections(first, second):
            found += 1
            for circle in (first, second):
                gap = abs(M.distance(point, (circle.x, circle.y)) - circle.r)
                assert gap < 1e-6, f"교점이 원 위에 없음 ({gap:.2e})"
    assert found > 1000, "교점이 있는 경우가 너무 적게 뽑힘"


def test_detour_matches_driving():
    """손으로 계산한 '접선 + 호 + 접선' 길이와 실제로 걸어 본 길이가 같아야 함."""
    layout = L.BY_NAME["one"]
    circle = layout.inflated()[0]
    expected = min(M.detour_length(layout.start, layout.goal, circle, side) for side in ("left", "right"))
    trip = drive(layout.start, layout.goal, [circle], AvoiderSettings(step=0.001))
    assert trip.arrived
    assert abs(trip.length - expected) < 2e-3, f"계산 {expected:.4f} 와 주행 {trip.length:.4f} 이 다름"


def test_leave_rule_matches_tangent_from_goal():
    """수선의 발이 넘어가는 순간이 목표에서 그은 접점과 같은 곳이어야 함."""
    for _ in range(2000):
        circle = M.Circle(0.0, 0.0, RANDOM.uniform(0.1, 1.0))
        goal = _point_outside(circle)
        for turn in ("clockwise", "counterclockwise"):
            exact = M.leave_point(circle, goal, turn)
            angle = M.angle_on_circle(exact, circle)
            step = 1e-4 / circle.r
            before = M.point_at_angle(circle, angle - step if turn == "counterclockwise" else angle + step)
            after = M.point_at_angle(circle, angle + step if turn == "counterclockwise" else angle - step)
            assert not M.leaving_is_clear(before, goal, circle), "접점 직전인데 벌써 트였다고 함"
            assert M.leaving_is_clear(after, goal, circle), "접점을 지났는데 아직 막혔다고 함"


def test_tie_turns_right():
    trip = drive(L.BY_NAME["tie"].start, L.BY_NAME["tie"].goal, L.BY_NAME["tie"].inflated())
    first_hit = next(detail for _, kind, detail in trip.events if kind == "hit")
    assert first_hit.startswith("right"), f"길이가 같은데 오른쪽으로 안 감: {first_hit}"


def test_fixed_layouts():
    """고정 배치 결과가 문서에 적은 대로 나오는지. 바뀌면 README 의 0단계 표도 고칠 것."""
    expected = {"one": "arrived", "tie": "arrived", "scattered": "arrived",
                "close_pair": "collision", "wall": "collision", "cup": "collision",
                "hidden_goal": "arrived", "goal_inside": "goal_blocked"}
    for name, result in expected.items():
        layout = L.BY_NAME[name]
        trip = drive(layout.start, layout.goal, layout.inflated())
        assert trip.result == result, f"{name}: {trip.result} (예상 {result})"


def test_random_failures_need_overlap(count=400):
    """부풀린 원끼리 안 겹치는 배치는 전부 도착해야 함. 실패는 겹친 원에서만 나와야 함."""
    for seed in range(count):
        layout = L.random_layout(seed)
        circles = layout.inflated()
        overlap = any(M.distance((a.x, a.y), (b.x, b.y)) < a.r + b.r
                      for i, a in enumerate(circles) for b in circles[i + 1:])
        trip = drive(layout.start, layout.goal, circles)
        if not overlap:
            assert trip.arrived, f"random{seed}: 원이 안 겹치는데 {trip.result}"
        assert trip.result != "timeout", f"random{seed}: 걸음 수 상한에 걸림"


TESTS = [
    ("접선 (기울기 이차방정식)", test_tangent_lines),
    ("수직 접선 따로 처리", test_vertical_tangent),
    ("선분과 원 (판별식 + 수선의 발)", test_segment_check_matches_sampling),
    ("수선의 발", test_foot_is_perpendicular),
    ("두 원의 교점 (공통현)", test_circle_intersections),
    ("우회 길이 계산 = 주행 길이", test_detour_matches_driving),
    ("떠나는 순간 = 목표에서 그은 접점", test_leave_rule_matches_tangent_from_goal),
    ("길이가 같으면 오른쪽", test_tie_turns_right),
    ("고정 배치 결과", test_fixed_layouts),
    ("실패는 겹친 원에서만", test_random_failures_need_overlap),
]


def main() -> int:
    failed = 0
    for name, fn in TESTS:
        try:
            fn()
        except AssertionError as exc:
            print(f"  [실패] {name}: {exc}")
            failed += 1
        else:
            print(f"  [통과] {name}")
    if failed:
        print(f"\n{failed}개 실패")
        return 1
    print("\n처음 기하 모델 전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
