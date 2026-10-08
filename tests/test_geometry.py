"""기하 계산 검증. 무작위 값을 수만 번 넣어 성질이 깨지는지 본다.

특히 원 경계에 아슬아슬하게 걸치는 경우를 확인한다.
pytest 없이 그냥 실행해도 된다.

    python -m tests.test_geometry
"""
from __future__ import annotations

import sys

import numpy as np

from rccar import geometry as g

RNG = np.random.default_rng(20260909)
TWO_PI = 2.0 * np.pi


# --------------------------------------------------------------------------
def test_tangent_from_point(n: int = 25000) -> None:
    """접점은 (1) 원 위에 있고 (2) 반지름과 직각이고 (3) 회전 방향이 맞아야 한다."""
    for _ in range(n):
        c = RNG.uniform(-10.0, 10.0, 2)
        r = RNG.uniform(0.05, 3.0)
        # 절반은 원 표면에 아슬아슬하게 붙여 수치 안정성을 본다.
        if RNG.random() < 0.5:
            d = r * (1.0 + 10.0 ** RNG.uniform(-9.0, 0.5))
        else:
            d = r + RNG.uniform(0.001, 20.0)
        th = RNG.uniform(0.0, TWO_PI)
        P = c + d * np.array([np.cos(th), np.sin(th)])

        out = g.tangent_points_from_point(P, c, r)
        if d <= r:
            assert not out, "원 안의 점인데 접점이 나왔다"
            continue
        assert len(out) == 2, f"접점이 2개가 아니다: {len(out)}"

        for T, sigma in out:
            assert abs(np.hypot(*(T - c)) - r) < 1e-6, "접점이 원 위에 없다"
            radial = T - c
            along = P - T
            # 접선과 반지름은 직교한다.
            cos = abs(float(radial @ along)) / (
                np.hypot(*radial) * np.hypot(*along) + 1e-18)
            assert cos < 1e-4, f"접선이 반지름과 직각이 아니다 (cos={cos:.2e})"
            # sigma 는 P->T 진행이 그 회전 방향으로 이어지는지를 말한다.
            step = g.tangent_dir(T, c, sigma)
            assert float(step @ g.unit(T - P)) > -1e-4, \
                "sigma 회전 방향이 진행과 어긋난다"

        # 접선 길이는 피타고라스와 맞아야 한다.
        expect = np.sqrt(max(d * d - r * r, 0.0))
        assert abs(g.tangent_length(P, c, r) - expect) < 1e-6 * max(1.0, expect), \
            "tangent_length 가 피타고라스와 다르다"


def test_bitangents_equal_radius(n: int = 25000) -> None:
    """공통접선의 두 접점은 각자의 원 위에 있고, 이은 선분은 두 반지름과 직각이다."""
    for _ in range(n):
        c1 = RNG.uniform(-10.0, 10.0, 2)
        r = RNG.uniform(0.05, 3.0)
        d = RNG.uniform(0.001, 25.0)
        th = RNG.uniform(0.0, TWO_PI)
        c2 = c1 + d * np.array([np.cos(th), np.sin(th)])

        out = g.bitangents_equal_radius(c1, c2, r)
        if d < 1e-9:
            assert not out
            continue
        # 외접선 2개는 항상, 내접선 2개는 두 원이 떨어져 있을 때만.
        expect_n = 4 if d > 2.0 * r + 1e-9 else 2
        assert len(out) == expect_n, f"접선 개수가 {len(out)} (기대 {expect_n})"

        for T1, T2 in out:
            assert abs(np.hypot(*(T1 - c1)) - r) < 1e-6, "접점 T1 이 원1 위에 없다"
            assert abs(np.hypot(*(T2 - c2)) - r) < 1e-6, "접점 T2 가 원2 위에 없다"
            seg = T2 - T1
            L = np.hypot(*seg)
            if L < 1e-6:
                continue
            u = seg / L
            for T, c in ((T1, c1), (T2, c2)):
                cos = abs(float(g.unit(T - c) @ u))
                assert cos < 1e-5, f"접선이 반지름과 직각이 아니다 (cos={cos:.2e})"


def test_outer_tangent_is_shifted_center_line(n: int = 8000) -> None:
    """반지름이 같으면 외접선은 중심선을 옆으로 r 만큼 민 것이다.

    모든 장애물을 같은 반지름으로 부풀리는 설계가 주는 이득이라,
    이 성질이 깨지면 그래프 구축 전체가 틀어진다.
    """
    for _ in range(n):
        c1 = RNG.uniform(-10.0, 10.0, 2)
        r = RNG.uniform(0.05, 3.0)
        d = RNG.uniform(0.01, 25.0)
        th = RNG.uniform(0.0, TWO_PI)
        c2 = c1 + d * np.array([np.cos(th), np.sin(th)])

        out = g.bitangents_equal_radius(c1, c2, r)
        p = g.perp(g.unit(c2 - c1))
        for sign in (+1.0, -1.0):
            want = (c1 + sign * r * p, c2 + sign * r * p)
            hit = any(np.allclose(T1, want[0], atol=1e-9)
                      and np.allclose(T2, want[1], atol=1e-9)
                      for T1, T2 in out)
            assert hit, "외접선이 중심선의 평행이동과 다르다"


def test_arc_direction(n: int = 12000) -> None:
    """호는 지정된 회전 방향을 따라간다. 짧은 쪽을 고르면 장애물을 관통한다."""
    for _ in range(n):
        c = RNG.uniform(-10.0, 10.0, 2)
        r = RNG.uniform(0.05, 3.0)
        th_a, th_b = RNG.uniform(0.0, TWO_PI, 2)
        sigma = int(RNG.choice((-1, 1)))
        T_a = c + r * np.array([np.cos(th_a), np.sin(th_a)])
        T_b = c + r * np.array([np.cos(th_b), np.sin(th_b)])

        sweep = g.arc_sweep(th_a, th_b, sigma)
        assert -1e-12 <= sweep < TWO_PI + 1e-12, f"sweep 이 [0,2pi) 밖: {sweep}"

        L = g.arc_length(c, r, T_a, T_b, sigma)
        assert abs(L - r * sweep) < 1e-6 * max(1.0, r * sweep), \
            "호 길이가 r*sweep 과 다르다"

        pts = g.arc_points(c, r, T_a, T_b, sigma, max_step=0.05)
        assert len(pts) >= 2
        # 표본은 전부 원 위에 있어야 한다.
        rad = np.hypot(*(pts - c).T)
        assert np.max(np.abs(rad - r)) < 1e-6, "호 표본이 원에서 벗어났다"
        assert np.allclose(pts[0], T_a, atol=1e-6), "호가 T_a 에서 시작하지 않는다"
        assert np.allclose(pts[-1], T_b, atol=1e-6), "호가 T_b 에서 끝나지 않는다"
        # 인접 표본 사이의 각 진행은 전부 sigma 부호를 따른다.
        ang = np.arctan2((pts - c)[:, 1], (pts - c)[:, 0])
        step = g.wrap_pi(np.diff(ang)) * sigma
        assert np.all(step > -1e-9), "호 표본이 반대 방향으로 돈다"


def _crosses(p1, p2, q1, q2) -> bool:
    """두 선분이 실제로 교차하는가. 표본 대조와 독립인 부호 판정."""
    def side(a, b, c):
        return np.sign((b[0] - a[0]) * (c[1] - a[1])
                       - (b[1] - a[1]) * (c[0] - a[0]))
    d1, d2 = side(p1, p2, q1), side(p1, p2, q2)
    d3, d4 = side(q1, q2, p1), side(q1, q2, p2)
    return d1 * d2 < 0 and d3 * d4 < 0


def test_segment_distance_matches_bruteforce(n: int = 3000) -> None:
    """선분 거리 함수를 조밀 표본으로 얻은 값과 대조한다.

    표본은 최소값을 절대 과소평가하지 못하므로 위쪽 한계는 엄격하게 본다.
    아래쪽은 표본 간격의 절반만큼 오차가 생길 수 있어 그만큼만 봐준다.
    교차하는 두 선분은 참값이 0 인데 표본은 교차점을 비껴가므로 따로 처리한다.
    """
    m = 1200
    ts = np.linspace(0.0, 1.0, m)[:, None]
    for _ in range(n):
        p1, p2, q1, q2 = (RNG.uniform(-6.0, 6.0, 2) for _ in range(4))

        # 점-선분
        p = RNG.uniform(-6.0, 6.0, 2)
        samp = q1 + ts * (q2 - q1) - p
        brute = float(np.min(np.hypot(samp[:, 0], samp[:, 1])))
        slack = 0.5 * float(np.hypot(*(q2 - q1))) / (m - 1) + 1e-9
        got = g.point_seg_distance(p, q1, q2)
        assert got <= brute + 1e-9, "point_seg_distance 가 표본 최소값보다 크다"
        assert got >= brute - slack, f"점-선분 거리 불일치: {got} vs {brute}"

        # 선분-선분
        A = p1 + ts * (p2 - p1)
        B = q1 + ts * (q2 - q1)
        diff = A[:, None, :] - B[None, :, :]
        brute = float(np.min(np.hypot(diff[:, :, 0], diff[:, :, 1])))
        got = g.seg_seg_distance(p1, p2, q1, q2)
        assert got <= brute + 1e-9, "seg_seg_distance 가 표본 최소값보다 크다"
        if _crosses(p1, p2, q1, q2):
            assert got == 0.0, f"교차하는 선분인데 거리가 {got} 이다"
        else:
            slack = 0.5 * (float(np.hypot(*(p2 - p1)))
                           + float(np.hypot(*(q2 - q1)))) / (m - 1) + 1e-9
            assert got >= brute - slack, f"선분-선분 거리 불일치: {got} vs {brute}"


def test_convex_hull(n: int = 4000) -> None:
    """껍질은 반시계이고, 모든 원래 점을 품어야 한다."""
    for _ in range(n):
        k = int(RNG.integers(1, 40))
        pts = RNG.uniform(-5.0, 5.0, (k, 2))
        if RNG.random() < 0.25:      # 일직선 같은 퇴화 입력도 섞는다
            base = RNG.uniform(-5.0, 5.0, 2)
            direction = RNG.uniform(-1.0, 1.0, 2)
            pts = base + RNG.uniform(-3.0, 3.0, (k, 1)) * direction

        hull = g.convex_hull(pts)
        assert len(hull) >= 1, "껍질이 비었다"
        if len(hull) < 3:
            continue

        area = 0.0
        for i in range(len(hull)):
            a, b = hull[i], hull[(i + 1) % len(hull)]
            area += a[0] * b[1] - b[0] * a[1]
        assert area > -1e-9, "껍질이 시계 방향이다"

        # poly_distance 는 안쪽이 음수. 원래 점은 전부 껍질 안이거나 경계.
        for p in pts:
            assert g.poly_distance(p, hull) < 1e-6, "원래 점이 껍질 밖으로 나갔다"


def test_poly_distance(n: int = 3000) -> None:
    """다각형까지의 부호 거리와, 부풀린 외곽선의 두께를 확인한다."""
    for _ in range(n):
        k = int(RNG.integers(3, 12))
        hull = g.convex_hull(RNG.uniform(-4.0, 4.0, (k, 2)))
        if len(hull) < 3:
            continue

        # 부호: 무게중심은 안(음수), 충분히 먼 점은 밖(양수).
        c = hull.mean(axis=0)
        assert g.poly_distance(c, hull) < 0.0, "무게중심이 바깥으로 판정됐다"
        far = c + np.array([1e3, 1e3])
        assert g.poly_distance(far, hull) > 0.0, "먼 점이 안쪽으로 판정됐다"

        # 변 위의 점은 거리 0.
        i = int(RNG.integers(0, len(hull)))
        a, b = hull[i], hull[(i + 1) % len(hull)]
        mid = a + RNG.random() * (b - a)
        assert abs(g.poly_distance(mid, hull)) < 1e-9, "경계 위 점의 거리가 0이 아니다"

        # 선분-다각형 거리는 양 끝점 거리보다 클 수 없다.
        p, q = RNG.uniform(-8.0, 8.0, 2), RNG.uniform(-8.0, 8.0, 2)
        sd = g.seg_poly_distance(p, q, hull)
        assert sd <= max(g.poly_distance(p, hull), 0.0) + 1e-9
        assert sd <= max(g.poly_distance(q, hull), 0.0) + 1e-9

        # 부풀린 외곽선은 원본에서 정확히 rho 만큼 떨어져 있다.
        rho = RNG.uniform(0.05, 1.5)
        ring = g.offset_polygon(hull, rho, arc_step=0.15)
        dist = np.array([g.poly_distance(p, hull) for p in ring])
        assert np.max(np.abs(dist - rho)) < 1e-6, \
            "부풀린 외곽선의 두께가 rho 가 아니다"


def test_ray_hits(n: int = 12000) -> None:
    """라이다용 반직선 교차. 맞은 지점이 실제로 도형 위에 있어야 한다."""
    for _ in range(n):
        o = RNG.uniform(-8.0, 8.0, 2)
        th = RNG.uniform(0.0, TWO_PI)
        d = np.array([np.cos(th), np.sin(th)])

        c = RNG.uniform(-8.0, 8.0, 2)
        r = RNG.uniform(0.05, 3.0)
        t = g.ray_circle(o, d, c, r)
        if t is not None:
            assert t > 0.0, "반직선 뒤쪽을 맞췄다"
            assert abs(np.hypot(*(o + t * d - c)) - r) < 1e-6, "교점이 원 위에 없다"
        elif np.hypot(*(o - c)) > r + 1e-6:
            # 안 맞았다면 반직선이 원에 닿지 않아야 한다.
            far = o + 1e4 * d
            assert g.point_seg_distance(c, o, far) > r - 1e-6, \
                "닿는데 None 을 반환했다"

        a, b = RNG.uniform(-8.0, 8.0, 2), RNG.uniform(-8.0, 8.0, 2)
        t = g.ray_segment(o, d, a, b)
        if t is not None:
            assert t > 0.0, "반직선 뒤쪽을 맞췄다"
            assert g.point_seg_distance(o + t * d, a, b) < 1e-6, \
                "교점이 선분 위에 없다"


def test_angle_wrapping(n: int = 60000) -> None:
    """각도 정규화의 치역. 각 자체는 바뀌지 않아야 한다."""
    a = RNG.uniform(-200.0, 200.0, n)

    w = g.wrap_pi(a)
    assert np.all(w >= -np.pi - 1e-12) and np.all(w <= np.pi + 1e-12), \
        "wrap_pi 치역 위반"
    assert np.max(np.abs(np.sin(a) - np.sin(w))) < 1e-9, "wrap_pi 가 각을 바꿨다"
    assert np.max(np.abs(np.cos(a) - np.cos(w))) < 1e-9, "wrap_pi 가 각을 바꿨다"

    w2 = g.wrap_2pi(a)
    assert np.all(w2 >= -1e-12) and np.all(w2 < TWO_PI + 1e-12), \
        "wrap_2pi 치역 위반"
    assert np.max(np.abs(np.sin(a) - np.sin(w2))) < 1e-9, "wrap_2pi 가 각을 바꿨다"
    assert np.max(np.abs(np.cos(a) - np.cos(w2))) < 1e-9, "wrap_2pi 가 각을 바꿨다"


def test_pose_composition(n: int = 1000) -> None:
    """SE(2) 합성과 역변환. 센서 좌표를 세계 좌표로 옮길 때 쓴다."""
    for _ in range(n):
        A = g.se2(*RNG.uniform(-5.0, 5.0, 2), RNG.uniform(-10.0, 10.0))
        B = g.se2(*RNG.uniform(-5.0, 5.0, 2), RNG.uniform(-10.0, 10.0))
        p = RNG.uniform(-5.0, 5.0, (7, 2))

        # 합성 = 차례로 적용
        lhs = g.compose(g.compose_pose(A, B), p)
        rhs = g.compose(A, g.compose(B, p))
        assert np.max(np.abs(lhs - rhs)) < 1e-9, "합성이 순차 적용과 다르다"

        # 역변환은 원래 점으로 되돌린다.
        back = g.compose(g.invert_pose(A), g.compose(A, p))
        assert np.max(np.abs(back - p)) < 1e-9, "역변환이 원점으로 돌아오지 않는다"


TESTS = [
    ("점 -> 원 접점", test_tangent_from_point),
    ("원 <-> 원 공통접선", test_bitangents_equal_radius),
    ("바깥접선 = 평행이동", test_outer_tangent_is_shifted_center_line),
    ("호의 회전 방향", test_arc_direction),
    ("선분 거리", test_segment_distance_matches_bruteforce),
    ("볼록 껍질", test_convex_hull),
    ("다각형 거리와 부풀림", test_poly_distance),
    ("반직선 교차", test_ray_hits),
    ("각도 정규화", test_angle_wrapping),
    ("SE(2) 합성", test_pose_composition),
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
    print("\n기하 계산 전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
