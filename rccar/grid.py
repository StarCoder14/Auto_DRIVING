"""
grid.py -- 점유 격자와 비상용 격자 A*  (문서 2.3, 5장)

접선 그래프는 빠르지만 구조적 한계가 하나 있다.
ㄷ자 막다른 골목에 들어가면 스스로 못 나온다. 빠져나오려면 먼저 목적지
반대 방향으로 한참 후퇴해야 하는데, '목적지 쪽 최단 경로'만 보는
알고리즘은 그런 판단을 못 하기 때문이다.

그래서 격자 지도를 따로 유지한다. 평소에는 쓰지 않는다.
갇혔다고 판단될 때만 격자 위에서 칸을 하나씩 따라가는 방식으로
탈출로를 찾는다. 느리지만 길이 있으면 반드시 찾는다.

  평소엔 빠른 방식(접선 그래프), 위기엔 확실한 방식(격자). 이중 구조.
"""

import heapq
import numpy as np

L_FREE, L_OCC = -0.45, 0.95
L_MIN, L_MAX = -4.0, 4.0
OCC_THRESH, FREE_THRESH = 0.8, -0.4


class OccupancyGrid:
    def __init__(self, bounds, res=0.10):
        x0, y0, x1, y1 = bounds
        self.res = float(res)
        self.origin = np.array([x0, y0], float)
        self.nx = int(np.ceil((x1 - x0) / res))
        self.ny = int(np.ceil((y1 - y0) / res))
        self.L = np.zeros((self.nx, self.ny), dtype=np.float32)

    # ------------------------------------------------------------ 좌표
    def to_idx(self, p):
        q = (np.asarray(p, float).reshape(-1, 2) - self.origin) / self.res
        return np.clip(q.astype(np.int32), [0, 0], [self.nx - 1, self.ny - 1])

    def to_world(self, i, j):
        return self.origin + (np.array([i, j], float) + 0.5) * self.res

    def inside(self, p):
        q = np.asarray(p, float) - self.origin
        return 0 <= q[0] < self.nx * self.res and 0 <= q[1] < self.ny * self.res

    # ------------------------------------------------------------ 갱신
    def integrate(self, origin, hits, max_range_ends=None):
        """
        광선을 따라 '비었음', 끝점에 '막혔음'을 기록한다.
        max_range_ends 는 아무것도 못 맞힌 광선의 끝점 (비었음만 기록).
        """
        o = self.to_idx(origin)[0]
        for p in np.asarray(hits, float).reshape(-1, 2):
            q = self.to_idx(p)[0]
            for i, j in _bresenham(o[0], o[1], q[0], q[1]):
                self.L[i, j] = max(self.L[i, j] + L_FREE, L_MIN)
            self.L[q[0], q[1]] = min(self.L[q[0], q[1]] + L_OCC - L_FREE, L_MAX)
        if max_range_ends is not None:
            for p in np.asarray(max_range_ends, float).reshape(-1, 2):
                q = self.to_idx(p)[0]
                for i, j in _bresenham(o[0], o[1], q[0], q[1]):
                    self.L[i, j] = max(self.L[i, j] + L_FREE, L_MIN)

    # ------------------------------------------------------------ 조회
    def is_unknown(self, p):
        i, j = self.to_idx(p)[0]
        return abs(float(self.L[i, j])) < 1e-6

    def is_free_many(self, pts):
        """이미 가봐서 비어 있다고 아는 칸인가."""
        idx = self.to_idx(pts)
        return self.L[idx[:, 0], idx[:, 1]] < FREE_THRESH

    def count_unknown(self, pts):
        idx = self.to_idx(pts)
        return int(np.sum(np.abs(self.L[idx[:, 0], idx[:, 1]]) < 1e-6))

    def occupied_mask(self):
        return self.L > OCC_THRESH

    def free_mask(self):
        return self.L < FREE_THRESH

    # ------------------------------------------------------------ 프론티어
    def frontiers(self, min_cluster=3):
        """자유 공간과 미지 영역의 경계 칸들 (탐험 목표 후보)."""
        free = self.free_mask()
        unknown = np.abs(self.L) < 1e-6
        nb = np.zeros_like(unknown)
        nb[1:, :] |= unknown[:-1, :]
        nb[:-1, :] |= unknown[1:, :]
        nb[:, 1:] |= unknown[:, :-1]
        nb[:, :-1] |= unknown[:, 1:]
        fi, fj = np.where(free & nb)
        if len(fi) < min_cluster:
            return np.zeros((0, 2))
        return self.origin + (np.stack([fi, fj], axis=1) + 0.5) * self.res

    # ------------------------------------------------------------ 격자 A*
    def astar(self, start, goal, inflate_cells=3):
        """
        8방향 격자 A*. 막힌 칸을 inflate_cells 만큼 부풀려서 피한다.
        미지 영역은 통행 가능으로 본다(낙관적 가정). 느리지만 완전하다.
        """
        occ = _dilate(self.occupied_mask(), inflate_cells)
        s = tuple(self.to_idx(start)[0])
        g = tuple(self.to_idx(goal)[0])
        if occ[s]:
            occ = _dilate(self.occupied_mask(), max(0, inflate_cells - 2))
            if occ[s]:
                occ[s] = False
        if occ[g]:
            g = _nearest_free(occ, g)
            if g is None:
                return None

        nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
        h = lambda a: np.hypot(a[0] - g[0], a[1] - g[1])
        openq = [(h(s), 0.0, s)]
        came, gsc, closed = {}, {s: 0.0}, set()
        while openq:
            _, c0, u = heapq.heappop(openq)
            if u in closed:
                continue
            closed.add(u)
            if u == g:
                path = [u]
                while u in came:
                    u = came[u]
                    path.append(u)
                pts = np.array([self.to_world(i, j) for i, j in path[::-1]])
                return _shortcut(pts, occ, self)
            for di, dj, w in nbrs:
                v = (u[0] + di, u[1] + dj)
                if not (0 <= v[0] < self.nx and 0 <= v[1] < self.ny):
                    continue
                if occ[v]:
                    continue
                ng = c0 + w
                if ng < gsc.get(v, np.inf):
                    gsc[v] = ng
                    came[v] = u
                    heapq.heappush(openq, (ng + h(v), ng, v))
        return None


def _bresenham(x0, y0, x1, y1):
    pts = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = int(x0), int(y0)
    for _ in range(int(dx + dy) + 2):
        if x == x1 and y == y1:
            break
        pts.append((x, y))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return pts


def _dilate(mask, k):
    if k <= 0:
        return mask.copy()
    out = mask.copy()
    for _ in range(k):
        cur = out.copy()
        out[1:, :] |= cur[:-1, :]
        out[:-1, :] |= cur[1:, :]
        out[:, 1:] |= cur[:, :-1]
        out[:, :-1] |= cur[:, 1:]
    return out


def _nearest_free(occ, g, max_r=40):
    for r in range(1, max_r):
        for di in range(-r, r + 1):
            for dj in (-r, r):
                for a, b in ((g[0] + di, g[1] + dj), (g[0] + dj, g[1] + di)):
                    if 0 <= a < occ.shape[0] and 0 <= b < occ.shape[1] \
                            and not occ[a, b]:
                        return (a, b)
    return None


def _shortcut(pts, occ, grid, max_iter=200):
    """격자 경로의 계단 모양을 펴서 추종하기 쉽게 만든다."""
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1:
            if _visible(pts[i], pts[j], occ, grid):
                break
            j -= 1
        out.append(pts[j])
        i = j
    return np.array(out)


def _visible(a, b, occ, grid):
    n = max(2, int(np.hypot(*(b - a)) / grid.res) + 1)
    for p in a + np.outer(np.linspace(0, 1, n), (b - a)):
        i, j = grid.to_idx(p)[0]
        if occ[i, j]:
            return False
    return True
