"""
world.py -- 실제 세계(정답)와 라이다 시뮬레이션.

이 파일만 '정답'을 안다. 알고리즘 쪽 코드는 여기를 절대 들여다보지 않고,
오직 lidar.scan() 이 돌려주는 노이즈 섞인 거리값만 본다.

라이다는 광선 투사로 흉내 낸다. 가장 가까운 물체에서 멈추므로
'앞면만 보이고 뒷면과 그림자는 안 보이는' 성질이 자동으로 생긴다.
"""

import numpy as np
from . import geometry as G


class World:
    """정답 지도. 원기둥과 벽(선분)으로 구성."""

    def __init__(self, circles=(), walls=(), bounds=(-16, -16, 16, 16)):
        self.circles = [(np.asarray(c, float), float(r)) for c, r in circles]
        self.walls = [(np.asarray(a, float), np.asarray(b, float))
                      for a, b in walls]
        self.bounds = bounds

    def add_box(self, cx, cy, w, h):
        """직사각형 벽 (테두리 4개 선분)."""
        x0, y0, x1, y1 = cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2
        pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
        for i in range(4):
            self.walls.append((np.array(pts[i]), np.array(pts[(i + 1) % 4])))

    def raycast(self, origin, direction, max_range):
        """가장 가까운 교차까지의 거리. 없으면 max_range."""
        best = max_range
        for c, r in self.circles:
            t = G.ray_circle(origin, direction, c, r)
            if t is not None and t < best:
                best = t
        for a, b in self.walls:
            t = G.ray_segment(origin, direction, a, b)
            if t is not None and t < best:
                best = t
        return best

    def min_distance(self, p):
        """점 p 에서 가장 가까운 실제 장애물 표면까지 (충돌 판정용)."""
        d = np.inf
        for c, r in self.circles:
            d = min(d, float(np.hypot(*(np.asarray(p) - c))) - r)
        for a, b in self.walls:
            d = min(d, G.point_seg_distance(p, a, b))
        return d


class Lidar:
    """
    2D 라이다.

    n_beams   : 한 스캔의 광선 수
    max_range : 최대 거리 (이보다 먼 것은 안 보인다)
    fov       : 시야각 (2*pi 면 전방위)
    sigma_r   : 거리 노이즈 표준편차
    dropout   : 각 광선이 반환되지 않을 확률
    """

    def __init__(self, n_beams=270, max_range=8.0, fov=2 * np.pi,
                 sigma_r=0.02, dropout=0.01, rng=None):
        self.n_beams = n_beams
        self.max_range = max_range
        self.fov = fov
        self.sigma_r = sigma_r
        self.dropout = dropout
        self.rng = rng or np.random.default_rng(0)
        self.angles = np.linspace(-fov / 2, fov / 2, n_beams, endpoint=(fov < 6.28))
        self.dphi = float(self.angles[1] - self.angles[0])

    def scan(self, world, pose):
        """
        반환: (angles, ranges)  --  차체 좌표계 기준.
        맞은 게 없으면 ranges 는 inf.
        """
        x, y, th = pose
        out = np.full(self.n_beams, np.inf)
        for i, a in enumerate(self.angles):
            ga = th + a
            d = np.array([np.cos(ga), np.sin(ga)])
            r = world.raycast((x, y), d, self.max_range)
            if r >= self.max_range - 1e-9:
                continue
            if self.rng.random() < self.dropout:
                continue
            out[i] = max(0.05, r + self.rng.normal(0.0, self.sigma_r))
        return self.angles, out

    @staticmethod
    def to_points(angles, ranges):
        """(각도, 거리) -> 차체 좌표계의 점 목록."""
        m = np.isfinite(ranges)
        a, r = angles[m], ranges[m]
        return np.stack([r * np.cos(a), r * np.sin(a)], axis=1), m
