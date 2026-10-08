"""하이브리드 A* 계획기 검증.

경로가 '짧은가'보다 '차가 실제로 따라갈 수 있고 부딪히지 않는가'를 먼저 본다.

    python -m tests.test_hybrid
"""
from __future__ import annotations

import sys
from math import hypot, pi

import numpy as np

from rccar.grid import OccupancyGrid
from rccar.hybrid import HybridAStar, HybridParams, distance_field, wavefront
from rccar.hybrid_sim import integrate_fast
from rccar.vehicle import VehicleParams

VEH = VehicleParams()


def _grid(segments=(), circles=()):
    """전부 '비어 있다고 아는' 지도에 선분과 원 테두리를 막힌 칸으로 그린다."""
    g = OccupancyGrid((-16, -16, 16, 16), 0.12)
    g.L[:] = -1.0
    xs = g.origin[0] + (np.arange(g.nx) + 0.5) * g.res
    ys = g.origin[1] + (np.arange(g.ny) + 0.5) * g.res
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    for (ax, ay), (bx, by) in segments:
        n = int(hypot(bx - ax, by - ay) / 0.05) + 2
        for t in np.linspace(0.0, 1.0, n):
            i, j = g.to_idx((ax + t * (bx - ax), ay + t * (by - ay)))[0]
            g.L[i, j] = 3.0
    for (cx, cy), r in circles:
        g.L[np.abs(np.hypot(X - cx, Y - cy) - r) < 0.07] = 3.0
    return g


def _plan(g, start, goal, **kw):
    H = HybridAStar(VEH, HybridParams(**kw))
    D = distance_field(g.occupied_mask(), g.res, 1.2)
    return H, H.plan(start, goal, g, D)


def _check_feasible(H, r):
    """차가 따라갈 수 있는 경로인가: 곡률 한계, 진행 방향 일치, 여유."""
    P = r.poses
    d = np.diff(P[:, :2], axis=0)
    ds = np.hypot(d[:, 0], d[:, 1])
    dth = np.abs((np.diff(P[:, 2]) + pi) % (2 * pi) - pi)
    move = ds > 1e-6
    # 두 점 사이 직선 거리(현)는 호보다 짧다. 호 길이로 되돌려 곡률을 잰다.
    arc = ds * (1.0 + dth * dth / 24.0)
    assert np.all(dth[move] <= H.kmax * arc[move] + 1e-4), \
        f"곡률이 한계를 넘는다: 최대 {np.max(dth[move] / arc[move]):.3f} > {H.kmax:.3f}"
    # 전진이면 앞으로, 후진이면 뒤로 움직여야 한다 (옆으로 미끄러지면 안 된다)
    heading = np.stack([np.cos(P[:-1, 2]), np.sin(P[:-1, 2])], axis=1)
    along = np.sum(heading * d, axis=1) * r.dirs[1:]
    assert np.all(along[move] > 0.5 * ds[move]), "진행 방향과 차의 방향이 어긋난다"
    m = H.clearance_poses(P[:, 0], P[:, 1], P[:, 2])
    assert m.min() >= H.r_hard - 1e-3, \
        f"차체 여유 {m.min():.3f} m 가 기준 {H.r_hard:.3f} m 보다 작다"


# --------------------------------------------------------------------------
def test_distance_field_exact() -> None:
    """거리장은 전수 계산과 같다."""
    rng = np.random.default_rng(1)
    occ = rng.random((70, 55)) < 0.02
    D = distance_field(occ, 0.12, 0.9)
    I, J = np.meshgrid(np.arange(70), np.arange(55), indexing="ij")
    brute = np.full(occ.shape, np.inf)
    for a, b in zip(*np.nonzero(occ)):
        brute = np.minimum(brute, np.hypot(I - a, J - b) * 0.12)
    assert np.abs(D - np.minimum(brute, 0.9)).max() < 1e-5, "거리장이 전수 계산과 다르다"


def test_wavefront_does_not_leak() -> None:
    """두께 한 칸짜리 벽을 휴리스틱이 뚫고 지나가지 않는다 (실제로 있었던 버그)."""
    free = np.ones((40, 40), bool)
    free[20, 1:39] = False                    # 가로로 긴 벽, 양 끝만 열려 있다
    d = wavefront(free, (10, 20))
    assert d[30, 20] > 20.0, f"벽 너머까지 {d[30, 20]:.1f} 칸. 벽을 통과했다"


def test_free_space_is_straight() -> None:
    """빈 공간에서 정면 목적지는 직선이다."""
    H, r = _plan(_grid(), (0.0, 0.0, 0.0), (8.0, 0.0))
    assert r is not None, "빈 공간인데 경로가 없다"
    L = float(np.hypot(*np.diff(r.xy, axis=0).T).sum())
    assert abs(L - 8.0) < 0.05, f"직선 8 m 인데 경로가 {L:.2f} m"


def test_goal_behind_is_feasible() -> None:
    """목적지가 뒤에 있어도 제자리에서 돌지 않는다. 곡률 한계를 지킨다."""
    H, r = _plan(_grid(), (0.0, 0.0, 0.0), (-6.0, 0.0))
    assert r is not None
    _check_feasible(H, r)


def test_narrow_gap_passable() -> None:
    """표면 사이 0.8 m 틈은 지나간다. 예전 계획기는 1.69 m 가 필요했다."""
    half = 0.4 + 0.45
    g = _grid(circles=[((3.0, half), 0.45), ((3.0, -half), 0.45)],
              segments=[((3.0, half + 0.45), (3.0, 12.0)),
                        ((3.0, -half - 0.45), (3.0, -12.0))])
    H, r = _plan(g, (0.0, 0.0, 0.0), (6.0, 0.0))
    assert r is not None, "0.8 m 틈을 못 지나간다"
    L = float(np.hypot(*np.diff(r.xy, axis=0).T).sum())
    assert L < 7.0, f"틈으로 안 가고 {L:.1f} m 를 돌아갔다"
    _check_feasible(H, r)


def test_dead_end_escape() -> None:
    """ㄷ자 골목 안에서 골목 안쪽을 보고 출발해도 빠져나가는 길을 찾는다."""
    g = _grid(segments=[((-1, -5), (-1, 5)), ((-1, 5), (7, 5)), ((-1, -5), (7, -5))])
    H, r = _plan(g, (3.0, 0.0, np.radians(160)), (-10.0, 0.0))
    assert r is not None, "골목을 못 빠져나간다"
    assert r.poses[:, 0].max() > 7.0, "골목 입구로 나가지 않았다"
    _check_feasible(H, r)


def test_corridor_needs_reverse() -> None:
    """돌 수 없는 좁은 복도 끝에서는 후진으로 나온다."""
    g = _grid(segments=[((0, 0.8), (6, 0.8)), ((0, -0.8), (6, -0.8)),
                        ((6, -0.8), (6, 0.8))])
    H, r = _plan(g, (4.5, 0.0, 0.0), (-3.0, 0.0))
    assert r is not None, "복도에서 못 나온다"
    assert (r.dirs < 0).any(), "후진 없이 나왔다고 한다. 복도 폭에서는 불가능하다"
    _check_feasible(H, r)


def test_random_maps_safe() -> None:
    """무작위 기둥 밭에서 나온 경로는 전부 따라갈 수 있고 여유를 지킨다."""
    rng = np.random.default_rng(7)
    found = 0
    for _ in range(12):
        cs = []
        while len(cs) < 14:
            c = rng.uniform([1.0, -5.0], [11.0, 5.0])
            if hypot(*c) < 1.8 or hypot(c[0] - 12.0, c[1]) < 1.8:
                continue
            cs.append((tuple(c), float(rng.uniform(0.2, 0.5))))
        H, r = _plan(_grid(circles=cs), (0.0, 0.0, 0.0), (12.0, 0.0))
        if r is None:
            continue
        found += 1
        _check_feasible(H, r)
    assert found >= 8, f"12번 중 {found}번만 경로를 찾았다"


def test_integrate_fast_marks_rays() -> None:
    """광선이 지나간 칸은 비었음, 끝점은 막혔음으로 기록된다."""
    g = OccupancyGrid((-5, -5, 5, 5), 0.12)
    hits = np.array([[3.0, 0.0], [0.0, 2.0]])
    integrate_fast(g, np.array([0.0, 0.0]), hits)
    occ = g.occupied_mask()
    for h in hits:
        i, j = g.to_idx(h)[0]
        assert occ[i, j], "끝점이 막힌 칸으로 기록되지 않았다"
    i, j = g.to_idx((1.5, 0.0))[0]
    assert g.L[i, j] < 0, "광선이 지나간 칸이 비었음으로 기록되지 않았다"
    i, j = g.to_idx((-3.0, -3.0))[0]
    assert g.L[i, j] == 0, "광선이 안 지나간 칸이 바뀌었다"


TESTS = [
    ("거리장 = 전수 계산", test_distance_field_exact),
    ("휴리스틱이 벽을 안 뚫음", test_wavefront_does_not_leak),
    ("빈 공간은 직선", test_free_space_is_straight),
    ("뒤쪽 목적지도 곡률 한계 준수", test_goal_behind_is_feasible),
    ("0.8 m 틈 통과", test_narrow_gap_passable),
    ("막다른 골목 탈출", test_dead_end_escape),
    ("좁은 복도는 후진", test_corridor_needs_reverse),
    ("무작위 지도 안전", test_random_maps_safe),
    ("광선 기록", test_integrate_fast_marks_rays),
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
    print("\n하이브리드 A* 전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
