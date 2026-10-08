"""
obstacles.py -- 장애물 표현.

문서 2.2절: "장애물 하나 = 크기가 똑같은 원 여러 개를 이어붙인 덩어리"

    센서 점  ->  볼록껍질  ->  꼭짓점마다 반지름 rho 인 원

이 표현의 이득
  * 점이 1개면 그냥 원          (기둥)
  * 벽은 가늘고 긴 덩어리       (거대한 원이 생기지 않는다)
  * 겹치면 점 집합만 합치면 됨  (병합이 자동)
  * 모든 원의 반지름이 같음      (원-원 접선이 평행 이동 한 번으로 끝난다)

거리 계산은 numpy 로 벡터화되어 있다. 계획기가 1초에 수만 번 부르기 때문이다.
"""

import numpy as np
from . import geometry as G


class ObstacleGroup:
    """부풀린 둥근 볼록껍질 하나. 계획기가 보는 장애물의 단위."""

    def __init__(self, verts, rho, gid=0):
        self.verts = np.ascontiguousarray(
            np.asarray(verts, dtype=float).reshape(-1, 2))
        self.rho = float(rho)
        self.gid = int(gid)
        self.circles = self.verts              # 껍질 꼭짓점 = 그래프에 올릴 원
        self.center = self.verts.mean(axis=0)
        self.bound_r = float(
            np.max(np.hypot(*(self.verts - self.center).T))) + rho
        self._prep()

    def _prep(self):
        n = len(self.verts)
        if n >= 3:
            self.A = self.verts
            self.E = np.roll(self.verts, -1, axis=0) - self.verts
        elif n == 2:
            self.A = self.verts[:1]
            self.E = (self.verts[1:] - self.verts[:1])
        else:
            self.A = self.verts
            self.E = np.zeros((1, 2))
        self.L2 = np.maximum((self.E ** 2).sum(axis=1), 1e-18)
        self.closed = n >= 3

    # ----------------------------------------------------------- 거리
    def distance_many(self, pts):
        """
        여러 점에 대한 부호 있는 거리 (바깥 +, 안 -). 벡터화.
        꼭짓점 1개면 원, 2개면 캡슐, 3개 이상이면 둥근 다각형.
        """
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if len(self.verts) == 1:
            return np.hypot(*(pts - self.verts[0]).T) - self.rho

        d = pts[:, None, :] - self.A[None, :, :]              # (m, k, 2)
        t = np.clip((d * self.E[None]).sum(-1) / self.L2[None], 0.0, 1.0)
        proj = self.A[None] + t[..., None] * self.E[None]
        dist = np.hypot(*(pts[:, None, :] - proj).transpose(2, 0, 1))  # (m, k)
        dmin = dist.min(axis=1)

        if self.closed:
            cross = (self.E[None, :, 0] * (pts[:, None, 1] - self.A[None, :, 1])
                     - self.E[None, :, 1] * (pts[:, None, 0] - self.A[None, :, 0]))
            inside = (cross >= 0.0).all(axis=1)
            dmin = np.where(inside, -dmin, dmin)
        return dmin - self.rho

    def distance(self, p):
        return float(self.distance_many(np.asarray(p, float)[None, :])[0])

    def contains(self, p, tol=0.0):
        return self.distance(p) < -tol

    # ----------------------------------------------------------- 충돌
    def blocks_segment(self, a, b, tol=2e-3):
        """
        선분 ab 가 이 도형을 침범하는가.

        '직선'이 아니라 '선분' 기준이다. 직선으로 검사하면 옆으로 한참
        비껴 있는 장애물까지 피하게 되어 차가 이상하게 돌아간다.
        tol 은 접선이 자기 접점에서 자기 자신과 충돌 판정되는 것을 막는다.
        """
        if G.point_seg_distance(self.center, a, b) > self.bound_r:
            return False   # 경계원으로 1차 기각 (대부분 여기서 걸러진다)
        return G.seg_poly_distance(a, b, self.verts) < self.rho - tol

    def blocks_points(self, pts, tol=2e-3):
        """폴리라인/호 샘플점들이 이 도형을 침범하는가."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if np.min(np.hypot(*(pts - self.center).T)) > self.bound_r:
            return False
        return bool((self.distance_many(pts) < -tol).any())

    def outline(self):
        """그리기용 외곽선."""
        return G.offset_polygon(self.verts, self.rho)


# --------------------------------------------------------------- 부풀림 + 병합
def inflate_and_merge(shapes, rho, max_verts=6, merge_gap=0.0, max_pass=10,
                      is_free=None):
    """
    문서 2.1 ~ 2.2: 부풀리고, 겹치는 것끼리 합친다.

    shapes    : 점 배열의 리스트 (각각 하나의 관측 물체)
    rho       : 부풀림 반지름
                (차폭/2 + 센서오차 + 추종오차, 하한은 차의 최소 회전 반경)
    merge_gap : 이만큼 이내로 붙어 있으면 지나갈 수 없다고 보고 병합

    두 도형이 겹치면 점 집합을 합치기만 하면 되고, 합친 뒤에 또 겹칠 수
    있으므로 변화가 없을 때까지 반복한다.

    is_free : 점들이 '이미 가봐서 비어 있다고 아는 곳'인지 알려주는 함수.
              주면, 합쳤을 때 그 빈 곳을 삼켜버리는 병합은 거부한다.
              ㄷ자 벽 세 조각을 합치면 입구가 메워지는데, 그걸 막는 장치다.
              (조각들이 꼭짓점을 공유해 서로 맞닿는 경우는 병합하지 않아도
               공유 꼭짓점의 바깥 호로 모서리를 돌 수 있으므로 문제없다.)
    """
    hulls = []
    for s in shapes:
        pts = np.asarray(s, dtype=float).reshape(-1, 2)
        if len(pts):
            hulls.append(G.simplify_hull(G.convex_hull(pts), max_verts=max_verts))

    def bound(V):
        c = V.mean(axis=0)
        return c, float(np.max(np.hypot(*(V - c).T)))

    for _ in range(max_pass):
        merged = False
        out, used = [], [False] * len(hulls)
        for i in range(len(hulls)):
            if used[i]:
                continue
            acc, used[i] = hulls[i], True
            changed = True
            while changed:
                changed = False
                for j in range(len(hulls)):
                    if used[j]:
                        continue
                    ca, ra = bound(acc)
                    cb, rb = bound(hulls[j])
                    if np.hypot(*(ca - cb)) > ra + rb + 2.0 * rho + merge_gap:
                        continue          # 경계원으로 1차 기각
                    if _gap(acc, hulls[j]) >= 2.0 * rho + merge_gap:
                        continue
                    if _swallows_free(acc, hulls[j], rho, is_free):
                        continue
                    if True:
                        acc = G.simplify_hull(
                            G.convex_hull(np.vstack([acc, hulls[j]])),
                            max_verts=max_verts)
                        used[j] = True
                        changed = merged = True
            out.append(acc)
        hulls = out
        if not merged:
            break

    return [ObstacleGroup(v, rho, gid=k) for k, v in enumerate(hulls)]


def _gap(A, B):
    """두 볼록 다각형 사이의 최소 거리."""
    ea, eb = _edges(A), _edges(B)
    if not ea:
        return min(G.point_seg_distance(A[0], *e) for e in eb) if eb \
            else float(np.hypot(*(A[0] - B[0])))
    if not eb:
        return min(G.point_seg_distance(B[0], *e) for e in ea)
    d = np.inf
    for a1, a2 in ea:
        for b1, b2 in eb:
            d = min(d, G.seg_seg_distance(a1, a2, b1, b2))
            if d == 0.0:
                return 0.0
    return d


def _edges(V):
    n = len(V)
    if n < 2:
        return []
    if n == 2:
        return [(V[0], V[1])]
    return [(V[i], V[(i + 1) % n]) for i in range(n)]


def clearance(p, groups, cap=12.0, exclude=()):
    """점 p 에서 가장 가까운 장애물까지의 여유 거리."""
    d = cap
    for g in groups:
        if g.gid in exclude:
            continue
        d = min(d, g.distance(p))
    return d


def _swallows_free(A, B, rho, is_free, step=0.25):
    """A 와 B 를 하나로 합치면, 이미 비어 있다고 아는 공간을 삼키는가."""
    if is_free is None:
        return False
    H = G.convex_hull(np.vstack([A, B]))
    if len(H) < 3:
        return False
    # 값싼 사전 검사: 새로 삼켜지는 면적이 작으면 검사할 것도 없다
    if _area(H) - _area(A) - _area(B) < (2.0 * rho) ** 2:
        return False
    lo, hi = H.min(axis=0), H.max(axis=0)
    if np.any(hi - lo < step):
        return False
    xs = np.arange(lo[0], hi[0] + step, step)
    ys = np.arange(lo[1], hi[1] + step, step)
    P = np.stack(np.meshgrid(xs, ys, indexing="ij"), axis=-1).reshape(-1, 2)
    if len(P) == 0:
        return False
    keep = np.array([G.poly_distance(p, H) < 0 for p in P])
    P = P[keep]
    if len(P) == 0:
        return False
    # 두 도형 어느 쪽에도 속하지 않는(=새로 삼켜지는) 점만 본다
    gA, gB = ObstacleGroup(A, rho, -1), ObstacleGroup(B, rho, -2)
    new = (gA.distance_many(P) > 0) & (gB.distance_many(P) > 0)
    P = P[new]
    return bool(len(P) and np.any(is_free(P)))


def _area(V):
    V = np.asarray(V, float).reshape(-1, 2)
    if len(V) < 3:
        return 0.0
    x, y = V[:, 0], V[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
