"""접선 그래프 + A* 계획기 검증.

기하 함수가 맞아도 그래프를 잘못 이으면 경로가 장애물을 통과한다.
그래서 여기서는 '나온 경로가 실제로 안전한가'를 무작위 환경으로 확인한다.

    python -m tests.test_planner
"""
from __future__ import annotations

import sys
import time

import numpy as np

from rccar import geometry as g
from rccar.obstacles import ObstacleGroup, inflate_and_merge
from rccar.planner import PlannerParams, plan

RNG = np.random.default_rng(20260909)

START = np.array([0.0, 0.0])
GOAL = np.array([20.0, 0.0])
RHO = 0.45


def _params(**kw) -> PlannerParams:
    """시험용 기본값. 넓게 보고, 벌점 없이 순수 거리만 센다.

    plan_radius 와 max_circles 를 키우지 않으면 계획기가 먼 장애물을 아예
    후보에서 빼기 때문에, 그걸 '관통했다'고 잘못 판정하게 된다.
    """
    base = dict(w_clear=0.0, w_turn=0.0, w_unknown=0.0,
                plan_radius=200.0, max_circles=400)
    base.update(kw)
    return PlannerParams(**base)


def _blobs(n: int, rng: np.random.Generator) -> list:
    """출발점과 목표점을 비워 둔 채 무작위 볼록 덩어리를 뿌린다."""
    shapes = []
    while len(shapes) < n:
        c = np.array([rng.uniform(1.0, 19.0), rng.uniform(-7.0, 7.0)])
        if np.hypot(*(c - START)) < 2.0 or np.hypot(*(c - GOAL)) < 2.0:
            continue
        k = int(rng.integers(1, 6))
        shapes.append(c + rng.uniform(-0.6, 0.6, (k, 2)))
    return shapes


def _penetration(poly: np.ndarray, groups: list, max_step: float = 0.10) -> float:
    """경로가 장애물 안으로 파고든 최대 깊이. 안전하면 0 이하.

    폴리라인은 호를 현으로 근사하므로 모서리를 안쪽으로 조금 자른다.
    그 양은 step^2 / (8*rho) 로 위가 막히므로, 그만큼은 봐준다.
    """
    sag = max_step * max_step / (8.0 * RHO)
    worst = 0.0
    for grp in groups:
        d = grp.distance_many(poly)
        worst = max(worst, float(-d.min()) - sag)
    return max(worst, 0.0)


def _length(poly: np.ndarray) -> float:
    return float(np.hypot(*np.diff(poly, axis=0).T).sum())


# --------------------------------------------------------------------------
def test_no_penetration(trials: int = 60, n_obstacles: int = 22) -> None:
    """무작위 환경에서 나온 경로는 어떤 장애물도 관통하지 않는다."""
    rng = np.random.default_rng(11)
    found = 0
    for _ in range(trials):
        groups = inflate_and_merge(_blobs(n_obstacles, rng), RHO)
        poly, cost, _ = plan(START, GOAL, groups, RHO, _params())
        if poly is None:
            continue                      # 정말 막힌 환경일 수 있다
        found += 1
        assert np.allclose(poly[0], START, atol=1e-6), "경로가 출발점에서 시작하지 않는다"
        assert np.allclose(poly[-1], GOAL, atol=1e-6), "경로가 목표점에서 끝나지 않는다"
        assert np.isfinite(cost), "비용이 무한대인데 경로가 나왔다"

        depth = _penetration(poly, groups)
        assert depth < 1e-3, f"경로가 장애물을 {depth * 1000:.1f} mm 관통했다"

    assert found >= trials // 2, \
        f"{trials}번 중 {found}번만 경로를 찾았다. 너무 자주 실패한다"


def test_freespace_is_straight(trials: int = 200) -> None:
    """장애물이 없으면 최단 경로는 직선이다."""
    rng = np.random.default_rng(22)
    for _ in range(trials):
        s = rng.uniform(-20.0, 20.0, 2)
        t = rng.uniform(-20.0, 20.0, 2)
        if np.hypot(*(t - s)) < 1.0:
            continue
        poly, cost, _ = plan(s, t, [], RHO, _params())
        assert poly is not None, "빈 공간인데 경로를 못 찾았다"
        straight = float(np.hypot(*(t - s)))
        assert abs(_length(poly) - straight) < 1e-6, "빈 공간 경로가 직선이 아니다"
        assert abs(cost - straight) < 1e-6, "빈 공간 비용이 직선 거리와 다르다"


def test_single_circle_is_tangent_arc_tangent(trials: int = 300) -> None:
    """장애물 하나를 정면에 두면 경로 길이는 접선-호-접선의 해석해와 같다.

    이게 맞으면 접점 계산, 회전 방향 선택, 호 길이가 전부 맞물려 있다는 뜻이다.
    """
    rng = np.random.default_rng(33)
    checked = 0
    for _ in range(trials):
        # 꼭짓점 하나짜리 장애물 = 반지름 RHO 인 원
        c = np.array([rng.uniform(6.0, 14.0), rng.uniform(-0.3, 0.3)])
        grp = ObstacleGroup(c[None, :], RHO, gid=0)
        R = RHO

        ds = float(np.hypot(*(START - c)))
        dg = float(np.hypot(*(GOAL - c)))
        if ds <= R or dg <= R:
            continue

        theta = abs(g.wrap_pi(np.arctan2(*(GOAL - c)[::-1])
                              - np.arctan2(*(START - c)[::-1])))
        arc = theta - np.arccos(R / ds) - np.arccos(R / dg)
        if arc <= 1e-3:
            continue                       # 직선이 원을 안 스친다. 다른 시험 소관
        expect = (np.sqrt(ds * ds - R * R) + np.sqrt(dg * dg - R * R) + R * arc)

        poly, cost, _ = plan(START, GOAL, [grp], RHO, _params())
        assert poly is not None, "원 하나를 못 돌아갔다"
        checked += 1
        assert abs(cost - expect) < 2e-3, \
            f"경로 길이가 해석해와 다르다: {cost:.6f} vs {expect:.6f}"
        assert _penetration(poly, [grp]) < 1e-3, "원을 관통했다"

    assert checked >= 50, f"검사한 경우가 {checked}개뿐이다"


def test_goal_inside_obstacle() -> None:
    """목표점이 장애물 안이면 경로가 없다고 답해야 한다. 뚫고 들어가면 안 된다."""
    grp = ObstacleGroup(GOAL[None, :], RHO, gid=0)
    assert grp.contains(GOAL), "시험 전제가 틀렸다. 목표점이 장애물 안이 아니다"
    poly, cost, _ = plan(START, GOAL, [grp], RHO, _params())
    assert poly is None, "목표점이 장애물 안인데 경로를 냈다"
    assert not np.isfinite(cost), "경로가 없는데 비용이 유한하다"


def test_start_inside_obstacle() -> None:
    """출발점이 장애물 안이면 기본은 실패, soft_gids 를 주면 빠져나온다."""
    grp = ObstacleGroup(START[None, :], RHO, gid=7)
    assert grp.contains(START), "시험 전제가 틀렸다. 출발점이 장애물 안이 아니다"

    poly, _, _ = plan(START, GOAL, [grp], RHO, _params())
    assert poly is None, "출발점이 장애물 안인데 그냥 경로를 냈다"

    # 이미 안에 갇혔을 때의 탈출 경로. 그 그룹만 검사를 면제한다.
    poly, cost, _ = plan(START, GOAL, [grp], RHO, _params(), soft_gids=(7,))
    assert poly is not None, "soft_gids 를 줬는데도 못 빠져나왔다"
    assert np.isfinite(cost)
    assert np.allclose(poly[-1], GOAL, atol=1e-6), "탈출 경로가 목표점에 닿지 않는다"


def test_cost_matches_polyline_length() -> None:
    """벌점을 끄면 A* 가 낸 비용은 실제 경로 길이와 같아야 한다."""
    rng = np.random.default_rng(44)
    checked = 0
    for _ in range(40):
        groups = inflate_and_merge(_blobs(15, rng), RHO)
        poly, cost, _ = plan(START, GOAL, groups, RHO, _params())
        if poly is None:
            continue
        checked += 1
        # 폴리라인은 호를 현으로 근사하므로 실제보다 아주 조금 짧다.
        assert _length(poly) <= cost + 1e-6, "폴리라인이 보고된 비용보다 길다"
        assert cost - _length(poly) < 0.05, \
            f"비용과 경로 길이 차이가 크다: {cost:.4f} vs {_length(poly):.4f}"
    assert checked >= 10


def test_clearance_penalty_widens_path() -> None:
    """여유 벌점을 켜면 좁은 틈 대신 돌아가는 쪽을 고른다."""
    # x=10 에 기둥 두 개. 사이 간격이 겨우 지나갈 정도.
    gap = 2.0 * RHO + 0.25
    shapes = [np.array([[10.0, +gap / 2.0]]), np.array([[10.0, -gap / 2.0]])]
    groups = inflate_and_merge(shapes, RHO, merge_gap=0.0)

    poly_tight, _, _ = plan(START, GOAL, groups, RHO, _params())
    poly_safe, _, _ = plan(START, GOAL, groups, RHO,
                           _params(w_clear=4.0, clear_ref=1.5))
    if poly_tight is None or poly_safe is None:
        return                              # 병합돼서 틈이 사라진 경우

    assert _penetration(poly_tight, groups) < 1e-3
    assert _penetration(poly_safe, groups) < 1e-3
    # 벌점을 켠 쪽이 더 멀리 비껴간다.
    assert np.max(np.abs(poly_safe[:, 1])) >= np.max(np.abs(poly_tight[:, 1])) - 1e-6, \
        "여유 벌점을 켰는데 더 좁은 쪽으로 갔다"


def test_lazy_evaluation(trials: int = 20) -> None:
    """지연 평가가 살아 있는지 본다. 속도의 90배는 전부 여기서 나온다.

    그래프를 만들 때는 회전 방향만 보고 길이를 하한으로 큐에 넣고, 꺼낼 때만
    실제 충돌 검사를 한다. 그래서 만들어진 엣지 중 극히 일부만 검사된다.
    이 성질이 깨지면 전수 검사로 돌아가 수십 배 느려진다.

    벽시계 시간과 달리 이 비율은 PC 성능과 무관하므로 회귀를 확실히 잡는다.
    """
    rng = np.random.default_rng(66)
    ratios = []
    for _ in range(trials):
        groups = inflate_and_merge(_blobs(18, rng), RHO)
        poly, _, gr = plan(START, GOAL, groups, RHO, _params())
        if poly is None or len(gr.edges) < 200:
            continue
        ratios.append(len(gr._eval) / len(gr.edges))

    assert ratios, "판정할 표본을 못 모았다"
    worst = max(ratios)
    print(f"        엣지 검사 비율 평균 {100 * np.mean(ratios):.1f}% / "
          f"최악 {100 * worst:.1f}%")
    assert worst < 0.35, \
        f"엣지의 {100 * worst:.0f}% 를 검사했다. 지연 평가가 깨졌다"


def test_timing(trials: int = 20) -> None:
    """계획 한 번에 걸리는 시간. 참고용으로 재서 찍는다.

    절대 시간은 PC 성능에 따라 몇 배씩 달라지므로 판정 기준으로 쓰기 어렵다.
    여기서는 '완전히 망가졌다'만 걸러내고, 실제 회귀 판정은
    test_lazy_evaluation 이 맡는다.
    """
    rng = np.random.default_rng(55)
    times = []
    for _ in range(trials):
        groups = inflate_and_merge(_blobs(25, rng), RHO)
        t0 = time.perf_counter()
        plan(START, GOAL, groups, RHO, PlannerParams())
        times.append((time.perf_counter() - t0) * 1000.0)

    avg = float(np.mean(times))
    worst = float(np.max(times))
    print(f"        계획 시간 평균 {avg:.0f} ms / 최악 {worst:.0f} ms "
          f"(PC 성능에 따라 달라짐)")
    assert avg < 10000.0, f"계획 평균이 {avg:.0f} ms 다. 무언가 크게 망가졌다"


TESTS = [
    ("장애물 관통 없음", test_no_penetration),
    ("빈 공간은 직선", test_freespace_is_straight),
    ("원 하나 = 접선-호-접선", test_single_circle_is_tangent_arc_tangent),
    ("목표점이 장애물 안", test_goal_inside_obstacle),
    ("출발점이 장애물 안", test_start_inside_obstacle),
    ("비용 = 경로 길이", test_cost_matches_polyline_length),
    ("여유 벌점", test_clearance_penalty_widens_path),
    ("지연 평가", test_lazy_evaluation),
    ("계획 시간", test_timing),
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
    print("\n계획기 전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
