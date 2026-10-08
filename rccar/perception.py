"""
perception.py -- 인식 (문서 4장)

세 가지 일을 한다.

1) 점을 묶는다 (4.1)
   라이다는 멀수록 점 간격이 벌어진다. 그래서 '몇 cm 이내면 같은 물체'라는
   고정 기준을 쓰면 안 된다. 거리에 비례해 기준을 늘린다.

2) 뒷면을 추측하지 않는다 (4.2)
   센서는 앞면만 본다. 살짝만 보인 호에 원을 맞추면 반지름이 수십 미터로
   발산해서, 로봇 눈에는 지도 절반이 장애물이 된다.
   그래서 충분히 넓게(기본 90도 이상) 보였을 때만 원으로 보고,
   그렇지 않으면 '보인 점들만' 그대로 쓴다. 뒷면은 모르는 채로 남긴다.

3) 같은 물체를 기억한다 (4.3)
   노이즈 때문에 가만히 있는 기둥도 매 순간 중심이 몇 cm 씩 흔들린다.
   이번에 본 것과 저번에 본 것을 짝지어 누적하면 흔들림이 줄고,
   차가 돌아가면서 뒷면이 저절로 채워진다.

   지우는 기준이 중요하다.
     * 시야 안인데 여러 번 연속 안 보이면  -> 지운다 (잘못 봤거나 치워졌다)
     * 시야 밖이면                        -> 절대 지우지 않는다
"""

import numpy as np
from . import geometry as G


# ------------------------------------------------------------------ 1) 묶기
def cluster(points, ranges, d0=0.16, k=2.6, dphi=0.02, min_pts=2):
    """
    연속된 점 사이 간격이 임계값을 넘으면 끊는다.
    임계값 = d0 + r * dphi * k   (거리에 비례해 커진다)
    """
    if len(points) == 0:
        return []
    out, cur = [], [0]
    for i in range(1, len(points)):
        gap = float(np.hypot(*(points[i] - points[i - 1])))
        thr = d0 + ranges[i] * dphi * k
        if gap <= thr:
            cur.append(i)
        else:
            if len(cur) >= min_pts:
                out.append(np.array(cur))
            cur = [i]
    if len(cur) >= min_pts:
        out.append(np.array(cur))
    return out


# ------------------------------------------------------ 2) 보수적 형상 추정
def fit_circle(pts):
    """대수적 원 피팅(Kasa). 반환 (중심, 반지름, 잔차)."""
    x, y = pts[:, 0], pts[:, 1]
    A = np.stack([x, y, np.ones_like(x)], axis=1)
    b = x ** 2 + y ** 2
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r2 = sol[2] + cx ** 2 + cy ** 2
    if r2 <= 0:
        return None
    r = float(np.sqrt(r2))
    res = float(np.mean(np.abs(np.hypot(x - cx, y - cy) - r)))
    return np.array([cx, cy]), r, res


def split_convex(pts, tol=0.08, min_pts=2):
    """
    한 묶음을 '볼록한 조각들'로 쪼갠다.  ★ 이 단계가 없으면 큰 사고가 난다.

    ㄷ자 벽을 한 묶음으로 잡아 볼록껍질을 씌우면 입구가 메워져서,
    그 안에 있던 자동차가 '장애물 내부에 있다'고 판정된다. 실제로 그렇게 된다.

    그래서 점렬을 훑어 현 chord 에서 가장 많이 벗어난 점에서 잘라내는 것을
    벗어남이 tol 이하가 될 때까지 반복한다(IEPF).
    잘린 조각들은 끝점을 공유하므로, 부풀리면 모서리에서 서로 맞닿는다.
    그 공유 꼭짓점의 바깥 호는 두 조각 모두의 경계 위에 정확히 놓이기 때문에
    모서리를 돌아가는 경로가 자연스럽게 생긴다.
    """
    pts = np.asarray(pts, float).reshape(-1, 2)
    n = len(pts)
    if n <= min_pts + 1:
        return [pts]
    out, stack = [], [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j - i < min_pts:
            out.append((i, j))
            continue
        a, b = pts[i], pts[j]
        ab = b - a
        L = float(np.hypot(*ab))
        if L < 1e-9:
            out.append((i, j))
            continue
        seg = pts[i:j + 1] - a
        dev = np.abs(seg[:, 0] * ab[1] - seg[:, 1] * ab[0]) / L
        k = int(np.argmax(dev))
        if dev[k] > tol and 0 < k < (j - i):
            stack.append((i, i + k))
            stack.append((i + k, j))
        else:
            out.append((i, j))
    out.sort()
    return [pts[i:j + 1] for i, j in out if j > i]


def shape_from_cluster(pts, sensor_origin, min_span_deg=90.0,
                       max_radius=2.5, res_tol=0.06, split_tol=0.08):
    """
    한 묶음에서 '점 집합' 목록을 만든다. 이것이 그대로 장애물의 재료가 된다.

    원으로 인정하는 조건 (전부 만족해야 함)
      * 센서에서 본 호의 각도 폭이 충분히 넓을 것
      * 피팅 잔차가 작을 것
      * 반지름이 상식적일 것
    인정되면 원 둘레를 점으로 흩뿌려 '뒷면까지' 채운다.
    아니면 보인 점을 볼록한 조각들로 쪼개서 쓴다. (뒷면은 모르는 채로 남는다)
    """
    if len(pts) < 3:
        return [pts]

    # 물체를 바라보는 각도 폭이 아니라, 물체 표면이 도는 각도 폭을 본다.
    fit = fit_circle(pts)
    if fit is None:
        return split_convex(pts, split_tol)
    c, r, res = fit
    if not (0.03 < r < max_radius) or res > res_tol:
        return split_convex(pts, split_tol)

    ang = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    span = float(np.max(G.wrap_pi(ang - ang.mean())) -
                 np.min(G.wrap_pi(ang - ang.mean())))
    if np.degrees(span) < min_span_deg:
        return split_convex(pts, split_tol)   # 조금만 보였다 -> 추측하지 않는다

    # 앞면을 본 게 맞는지 확인한다.
    # 센서에서 중심으로 가는 벡터를 u, 중심에서 관측점 평균으로 가는 벡터를 v 라 하면,
    # 앞면(센서 쪽 면)을 봤을 때 v 는 u 의 반대를 향하므로 내적이 음수다.
    # 양수면 관측점이 중심 너머에 있다는 뜻이라 피팅을 믿을 수 없다.
    if np.dot(c - np.asarray(sensor_origin), pts.mean(axis=0) - c) > 0:
        return split_convex(pts, split_tol)

    th = np.linspace(0, 2 * np.pi, 10, endpoint=False)
    return [c + r * np.stack([np.cos(th), np.sin(th)], axis=1)]


# ------------------------------------------------------------------ 3) 추적
class Track:
    """지도에 올라간 물체 하나. 세계 좌표계의 점 집합으로 유지한다."""

    _next_id = 0

    def __init__(self, pts):
        self.pts = np.asarray(pts, float).reshape(-1, 2)
        self.hits = 1
        self.misses = 0
        self.confirmed = False
        self.id = Track._next_id
        Track._next_id += 1
        self._refresh()

    def _refresh(self):
        self.hull = G.simplify_hull(G.convex_hull(self.pts), max_verts=6)
        self.center = self.hull.mean(axis=0)
        self.radius = float(np.max(np.hypot(*(self.hull - self.center).T))) \
            if len(self.hull) > 1 else 0.0

    def merge(self, pts, voxel=0.10, max_pts=140):
        """새 관측을 누적. 복셀 격자로 솎아 개수를 제한한다."""
        allp = np.vstack([self.pts, np.asarray(pts, float).reshape(-1, 2)])
        key = np.round(allp / voxel).astype(np.int64)
        _, idx = np.unique(key, axis=0, return_index=True)
        allp = allp[np.sort(idx)]
        if len(allp) > max_pts:
            allp = allp[np.linspace(0, len(allp) - 1, max_pts).astype(int)]
        self.pts = allp
        self._refresh()

    def distance_to(self, p):
        return G.poly_distance(p, self.hull)


class Tracker:
    def __init__(self, match_dist=0.55, confirm_hits=2, max_misses=4):
        self.tracks = []
        self.match_dist = match_dist
        self.confirm_hits = confirm_hits
        self.max_misses = max_misses

    def update(self, shapes_world, sensor_pos, max_range):
        """shapes_world: 이번 스캔에서 나온 점 집합들 (세계 좌표계)."""
        matched = set()
        for pts in shapes_world:
            pts = np.asarray(pts, float).reshape(-1, 2)
            if len(pts) == 0:
                continue
            ctr = pts.mean(axis=0)
            best, bd = None, np.inf
            for t in self.tracks:
                d = t.distance_to(ctr)
                if d < bd:
                    best, bd = t, d
            if best is not None and bd < self.match_dist:
                best.merge(pts)
                best.hits += 1
                best.misses = 0
                if best.hits >= self.confirm_hits:
                    best.confirmed = True
                matched.add(id(best))
            else:
                self.tracks.append(Track(pts))

        # 시야 안인데 못 본 것만 벌점. 시야 밖은 건드리지 않는다.
        alive = []
        for t in self.tracks:
            if id(t) in matched:
                alive.append(t)
                continue
            in_view = float(np.hypot(*(t.center - sensor_pos))) < max_range * 0.85
            if in_view:
                t.misses += 1
                if t.misses > self.max_misses:
                    continue          # 삭제
            alive.append(t)
        self.tracks = alive
        return self.tracks

    def shapes(self):
        """계획기에 넘길 점 집합 목록 (확정된 것만)."""
        return [t.pts for t in self.tracks if t.confirmed or t.hits >= 1]
