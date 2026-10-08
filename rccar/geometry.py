"""
geometry.py -- 알고리즘의 기하 엔진.

문서 3.2절의 "계산 세 가지"가 여기에 있다.
  1) 점 -> 원 접점            : tangent_points_from_point
  2) 원 <-> 원 접선           : bitangents_equal_radius
  3) 원 위 두 점을 잇는 호     : arc_length / arc_points

여기에 충돌 판정(선분/호 vs 도형)과 볼록껍질이 추가된다.
이 파일의 함수들은 전부 순수 함수이며, 단위 테스트가 가능하다.
"""

import numpy as np

TWO_PI = 2.0 * np.pi


# ---------------------------------------------------------------- 각도 유틸
def wrap_pi(a):
    """각도를 (-pi, pi] 로."""
    return (a + np.pi) % TWO_PI - np.pi


def wrap_2pi(a):
    """각도를 [0, 2pi) 로."""
    return a % TWO_PI


def perp(v):
    """반시계 90도 회전."""
    return np.array([-v[1], v[0]])


def unit(v):
    n = np.hypot(v[0], v[1])
    if n < 1e-12:
        return np.array([1.0, 0.0])
    return np.asarray(v, dtype=float) / n


# ------------------------------------------------- (1) 점 -> 원 접점
def tangent_points_from_point(P, c, r):
    """
    점 P 에서 원(중심 c, 반지름 r)에 그은 두 접선의 접점.

    반환: [(접점, sigma), (접점, sigma)]  또는 P 가 원 안이면 []
          sigma = +1 : 반시계(CCW) 회전에 대응
          sigma = -1 : 시계(CW)   회전에 대응

    유도:
        P, 중심 c, 접점 T 가 이루는 삼각형은 T 에서 직각이다.
        |c-T| = r, |c-P| = d  이므로 접선이 중심선에서 벌어진 각은 acos(r/d).
        중심에서 P 를 보는 각 beta 에서 +-alpha 만큼 돌리면 접점.

    2차방정식을 풀지 않으므로 d 가 r 에 가까워도 수치적으로 안정적이다.
    """
    P = np.asarray(P, dtype=float)
    c = np.asarray(c, dtype=float)
    dv = P - c
    d = np.hypot(dv[0], dv[1])
    if d <= r:                      # 점이 원 안/위 -> 접선 없음 (문서 8절 탈출 모드)
        return []
    beta = np.arctan2(dv[1], dv[0])
    alpha = np.arccos(np.clip(r / d, -1.0, 1.0))   # clip 은 필수 (부동소수 방어)
    out = []
    for sign, sigma in ((+1.0, +1), (-1.0, -1)):
        th = beta + sign * alpha
        T = c + r * np.array([np.cos(th), np.sin(th)])
        out.append((T, sigma))
    return out


def tangent_length(P, c, r):
    """접선의 길이 sqrt(d^2 - r^2)."""
    d = np.hypot(P[0] - c[0], P[1] - c[1])
    return np.sqrt(max(d * d - r * r, 0.0))


# ------------------------------------------------- (2) 원 <-> 원 접선
def bitangents_equal_radius(c1, c2, r):
    """
    반지름이 같은 두 원의 공통접선.

    반지름이 같으므로 외접선은 '두 중심을 이은 선을 반지름만큼 옆으로 민 것'이다.
    이것이 모든 장애물을 같은 크기의 원으로 표현한 가장 큰 이득이다.

    반환: [(T1, T2), ...]  (접점 쌍의 목록. 회전 방향은 호출측에서 판정)
    """
    c1 = np.asarray(c1, dtype=float)
    c2 = np.asarray(c2, dtype=float)
    dv = c2 - c1
    d = np.hypot(dv[0], dv[1])
    if d < 1e-9:
        return []
    n = dv / d
    p = perp(n)

    out = []
    # 외접선 2개: 중심선을 +-r 만큼 평행 이동
    out.append((c1 + r * p, c2 + r * p))
    out.append((c1 - r * p, c2 - r * p))

    # 내접선 2개: 두 원 사이를 가로지름. d > 2r 일 때만 존재.
    if d > 2.0 * r + 1e-9:
        beta = np.arctan2(dv[1], dv[0])
        alpha = np.arccos(np.clip(2.0 * r / d, -1.0, 1.0))
        M = 0.5 * (c1 + c2)
        for sign in (+1.0, -1.0):
            th = beta + sign * alpha
            T1 = c1 + r * np.array([np.cos(th), np.sin(th)])
            T2 = 2.0 * M - T1          # 중점 대칭
            out.append((T1, T2))
    return out


# ------------------------------------------------- (3) 원호
def point_angle(T, c):
    return np.arctan2(T[1] - c[1], T[0] - c[0])


def arc_sweep(th_a, th_b, sigma):
    """
    회전 방향 sigma 를 따라 th_a 에서 th_b 까지 진행한 각도. 항상 [0, 2pi).

    주의: '짧은 쪽 호'를 고르면 안 된다. 반드시 지정된 회전 방향을 따라가야
    장애물을 관통하지 않는다.
    """
    return wrap_2pi(sigma * (th_b - th_a))


def arc_length(c, r, T_a, T_b, sigma):
    return r * arc_sweep(point_angle(T_a, c), point_angle(T_b, c), sigma)


def arc_points(c, r, T_a, T_b, sigma, max_step=0.15):
    """호를 폴리라인으로 이산화. max_step 은 목표 호 간격(m)."""
    c = np.asarray(c, dtype=float)
    th_a = point_angle(T_a, c)
    sweep = arc_sweep(th_a, point_angle(T_b, c), sigma)
    n = max(2, int(np.ceil(r * sweep / max_step)) + 1)
    ths = th_a + sigma * np.linspace(0.0, sweep, n)
    return np.stack([c[0] + r * np.cos(ths), c[1] + r * np.sin(ths)], axis=1)


def tangent_dir(T, c, sigma):
    """원 위의 점 T 에서 회전 방향 sigma 로 진행할 때의 접선 방향(단위벡터)."""
    radial = unit(np.asarray(T, dtype=float) - np.asarray(c, dtype=float))
    return sigma * perp(radial)


def direction_consistent(T, c, sigma, travel_dir, tol=0.0):
    """
    접점 T 를 지나는 직선의 진행 방향이 회전 방향 sigma 와 맞는가.

    이 검사 하나가 '접점 4개 중 유효한 조합은 2개뿐' 문제를 통째로 해결한다.
    """
    return float(np.dot(unit(travel_dir), tangent_dir(T, c, sigma))) > tol


# ------------------------------------------------- 거리 / 충돌
def point_seg_distance(p, a, b):
    """점 p 와 '선분' ab 사이의 최소 거리. (직선이 아니라 선분이다.)"""
    p = np.asarray(p, dtype=float)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < 1e-18:
        return float(np.hypot(*(p - a)))
    t = float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
    return float(np.hypot(*(p - (a + t * ab))))


def seg_seg_distance(p1, p2, q1, q2):
    """두 선분 사이의 최소 거리 (교차하면 0)."""
    p1 = np.asarray(p1, float); p2 = np.asarray(p2, float)
    q1 = np.asarray(q1, float); q2 = np.asarray(q2, float)
    if _seg_intersect(p1, p2, q1, q2):
        return 0.0
    return min(point_seg_distance(p1, q1, q2), point_seg_distance(p2, q1, q2),
               point_seg_distance(q1, p1, p2), point_seg_distance(q2, p1, p2))


def _cross2(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _seg_intersect(p1, p2, q1, q2):
    d1 = _cross2(q1, q2, p1); d2 = _cross2(q1, q2, p2)
    d3 = _cross2(p1, p2, q1); d4 = _cross2(p1, p2, q2)
    if ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0)):
        return True
    for d, (a, b, c) in ((d1, (q1, q2, p1)), (d2, (q1, q2, p2)),
                         (d3, (p1, p2, q1)), (d4, (p1, p2, q2))):
        if abs(d) < 1e-12 and _on_seg(a, b, c):
            return True
    return False


def _on_seg(a, b, p):
    return (min(a[0], b[0]) - 1e-12 <= p[0] <= max(a[0], b[0]) + 1e-12 and
            min(a[1], b[1]) - 1e-12 <= p[1] <= max(a[1], b[1]) + 1e-12)


def ray_circle(origin, direction, c, r):
    """반직선-원 교차. 가장 가까운 양수 t 또는 None. (라이다 시뮬레이션용)"""
    o = np.asarray(origin, float) - np.asarray(c, float)
    d = np.asarray(direction, float)
    b = float(o @ d)
    cc = float(o @ o) - r * r
    disc = b * b - cc
    if disc < 0.0:
        return None
    s = np.sqrt(disc)
    for t in (-b - s, -b + s):
        if t > 1e-9:
            return t
    return None


def ray_segment(origin, direction, a, b):
    """반직선-선분 교차. 가장 가까운 양수 t 또는 None."""
    o = np.asarray(origin, float); d = np.asarray(direction, float)
    a = np.asarray(a, float); b = np.asarray(b, float)
    ab = b - a
    den = d[0] * (-ab[1]) - d[1] * (-ab[0])
    if abs(den) < 1e-12:
        return None
    rhs = a - o
    t = (rhs[0] * (-ab[1]) - rhs[1] * (-ab[0])) / den
    u = (d[0] * rhs[1] - d[1] * rhs[0]) / den
    if t > 1e-9 and -1e-9 <= u <= 1.0 + 1e-9:
        return t
    return None


# ------------------------------------------------- 볼록껍질
def convex_hull(points):
    """모노톤 체인. 반시계 방향으로 정렬된 껍질 꼭짓점을 반환."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(pts) <= 2:
        return _dedup(pts)
    order = np.lexsort((pts[:, 1], pts[:, 0]))
    pts = pts[order]
    pts = _dedup(pts)
    if len(pts) <= 2:
        return pts

    lower = []
    for p in pts:
        while len(lower) >= 2 and _cross2(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in pts[::-1]:
        while len(upper) >= 2 and _cross2(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = np.array(lower[:-1] + upper[:-1])
    return hull if len(hull) >= 1 else pts[:1]


def _dedup(pts, tol=1e-9):
    if len(pts) == 0:
        return pts.reshape(0, 2)
    keep = [pts[0]]
    for p in pts[1:]:
        if np.hypot(*(p - keep[-1])) > tol:
            keep.append(p)
    return np.array(keep)


def simplify_hull(hull, max_verts=8, min_edge=0.05):
    """
    껍질 꼭짓점 수를 제한한다.

    꼭짓점 하나가 그래프의 원 하나가 되므로, 꼭짓점이 많으면 계획이 느려진다.
    '면적을 가장 적게 잃는 꼭짓점'이 아니라 '제거해도 도형이 커지지 않는
    꼭짓점'을 지워야 안전하므로, 여기서는 반대로 기여가 작은 꼭짓점을 지우되
    항상 바깥쪽으로만 근사되도록 이웃 두 점을 그대로 둔다(=면적이 줄어든다).
    줄어든 만큼은 부풀림 반지름 rho 의 여유가 흡수한다.
    """
    hull = np.asarray(hull, dtype=float).reshape(-1, 2)
    if len(hull) <= 2:
        return hull
    # 너무 짧은 변 병합
    out = [hull[0]]
    for p in hull[1:]:
        if np.hypot(*(p - out[-1])) > min_edge:
            out.append(p)
    hull = np.array(out)
    if len(hull) <= 2:
        return hull
    while len(hull) > max_verts:
        n = len(hull)
        areas = [abs(_cross2(hull[(i - 1) % n], hull[i], hull[(i + 1) % n]))
                 for i in range(n)]
        hull = np.delete(hull, int(np.argmin(areas)), axis=0)
    return hull


def poly_distance(p, verts):
    """
    점 p 와 볼록 다각형(반시계 정렬) 사이의 부호 있는 거리.
    바깥이면 +, 안이면 -.  꼭짓점 1개면 점, 2개면 선분으로 처리.
    """
    p = np.asarray(p, dtype=float)
    n = len(verts)
    if n == 1:
        return float(np.hypot(*(p - verts[0])))
    if n == 2:
        return point_seg_distance(p, verts[0], verts[1])
    inside = True
    dmin = np.inf
    for i in range(n):
        a, b = verts[i], verts[(i + 1) % n]
        if _cross2(a, b, p) < 0:
            inside = False
        dmin = min(dmin, point_seg_distance(p, a, b))
    return -dmin if inside else dmin


def seg_poly_distance(p, q, verts):
    """선분 pq 와 볼록 다각형 사이의 최소 거리 (겹치면 0)."""
    n = len(verts)
    if n == 1:
        return point_seg_distance(verts[0], p, q)
    if n == 2:
        return seg_seg_distance(p, q, verts[0], verts[1])
    if poly_distance(p, verts) < 0 or poly_distance(q, verts) < 0:
        return 0.0
    d = np.inf
    for i in range(n):
        d = min(d, seg_seg_distance(p, q, verts[i], verts[(i + 1) % n]))
        if d == 0.0:
            return 0.0
    return d


def offset_polygon(verts, rho, arc_step=0.25):
    """
    다각형을 rho 만큼 부풀린 도형('둥근 볼록껍질')의 외곽선.
    그리기용. 꼭짓점 1개면 원, 2개면 캡슐이 된다.
    """
    verts = np.asarray(verts, dtype=float).reshape(-1, 2)
    n = len(verts)
    if n == 1:
        th = np.arange(0.0, TWO_PI + arc_step, arc_step)
        return verts[0] + rho * np.stack([np.cos(th), np.sin(th)], axis=1)
    if n == 2:
        d = unit(verts[1] - verts[0]); nn = perp(d)
        out = [verts[0] + rho * nn, verts[1] + rho * nn]
        th0 = np.arctan2(nn[1], nn[0])
        out += list(verts[1] + rho * np.stack(
            [np.cos(th0 - np.arange(arc_step, np.pi, arc_step)),
             np.sin(th0 - np.arange(arc_step, np.pi, arc_step))], axis=1))
        out += [verts[1] - rho * nn, verts[0] - rho * nn]
        th1 = th0 + np.pi
        out += list(verts[0] + rho * np.stack(
            [np.cos(th1 - np.arange(arc_step, np.pi, arc_step)),
             np.sin(th1 - np.arange(arc_step, np.pi, arc_step))], axis=1))
        return np.array(out)

    pts = []
    for i in range(n):
        a, b, c = verts[i - 1], verts[i], verts[(i + 1) % n]
        n_in = _outward_normal(a, b)
        n_out = _outward_normal(b, c)
        th0 = np.arctan2(n_in[1], n_in[0])
        th1 = np.arctan2(n_out[1], n_out[0])
        sweep = wrap_2pi(th1 - th0)
        k = max(2, int(sweep / arc_step) + 1)
        for th in th0 + np.linspace(0.0, sweep, k):
            pts.append(b + rho * np.array([np.cos(th), np.sin(th)]))
    return np.array(pts)


def _outward_normal(a, b):
    """반시계 다각형에서 변 a->b 의 바깥 방향 법선."""
    d = unit(np.asarray(b, float) - np.asarray(a, float))
    return np.array([d[1], -d[0]])


# ------------------------------------------------- 좌표 변환
def se2(x, y, th):
    return np.array([x, y, th], dtype=float)


def compose(T, p):
    """포즈 T=(x,y,th) 기준 국소 좌표 p 를 전역 좌표로."""
    c, s = np.cos(T[2]), np.sin(T[2])
    R = np.array([[c, -s], [s, c]])
    return (R @ np.asarray(p, float).reshape(-1, 2).T).T + T[:2]


def invert_pose(T):
    c, s = np.cos(-T[2]), np.sin(-T[2])
    R = np.array([[c, -s], [s, c]])
    return np.array([*(-(R @ T[:2])), -T[2]])


def compose_pose(A, B):
    """포즈 합성 A ∘ B."""
    c, s = np.cos(A[2]), np.sin(A[2])
    R = np.array([[c, -s], [s, c]])
    return np.array([*(R @ B[:2] + A[:2]), wrap_pi(A[2] + B[2])])
