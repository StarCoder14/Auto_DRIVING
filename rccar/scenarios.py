"""
scenarios.py -- 시험용 환경 모음.

각 시나리오는 (World, 시작 포즈, 목적지) 를 돌려준다.
"""

import numpy as np
from .world import World

BOUNDS = (-16.0, -16.0, 16.0, 16.0)


# def pillars(seed=0):
#     """기본: 흩어진 원기둥. 접선-호-접선이 가장 잘 보인다."""
#     w = World(bounds=BOUNDS)
#     w.circles = [(np.array(c), r) for c, r in [
#         ((-2.0, 1.0), 0.55), ((1.5, -1.5), 0.40), ((3.5, 2.5), 0.70),
#         ((-4.5, -3.0), 0.45), ((0.0, 4.0), 0.50), ((5.5, -2.0), 0.60),
#         ((-1.0, -4.5), 0.35), ((6.0, 4.0), 0.45),
#     ]]
#     return w, np.array([-9.0, -8.0, np.radians(35)]), np.array([9.0, 8.0])


# def pillars(seed=0):
#     """기본: 흩어진 원기둥. 접선-호-접선이 가장 잘 보인다."""
#     w = World(bounds=BOUNDS)
#     w.circles = [(np.array(c), r) for c, r in [
#         ((-3.0, 2.0), 0.55),   # 원래 (-2.0, 1.0) → 살짝 위로 이동
#         ((2.0, -2.0), 0.40),   # 원래 (1.5, -1.5) → 오른쪽 아래로 이동
#         ((4.0, 3.0), 0.70),    # 원래 (3.5, 2.5) → 조금 위로
#         ((-5.0, -2.5), 0.45),  # 원래 (-4.5, -3.0) → 왼쪽으로
#         ((0.5, 5.0), 0.50),    # 원래 (0.0, 4.0) → 위로
#         ((6.0, -1.5), 0.60),   # 원래 (5.5, -2.0) → 오른쪽 위로
#         ((-2.0, -5.0), 0.35),  # 원래 (-1.0, -4.5) → 왼쪽 아래로
#         ((7.0, 3.5), 0.45),    # 원래 (6.0, 4.0) → 오른쪽으로
#     ]]
#     return w, np.array([-9.0, -8.0, np.radians(35)]), np.array([9.0, 8.0])

def pillars(seed=0):
    """다양한 배치: 경로가 여러 갈래로 나뉘는 원 장애물 환경."""
    w = World(bounds=BOUNDS)
    w.circles = [(np.array(c), r) for c, r in [
        ((-3.5, 2.5), 0.55),   # 왼쪽 위
        ((1.0, -2.5), 0.45),   # 중앙 아래
        ((3.5, 3.0), 0.60),    # 오른쪽 위
        ((-5.0, -3.0), 0.50),  # 왼쪽 아래
        ((0.0, 4.5), 0.55),    # 중앙 위
        ((5.5, -1.0), 0.50),   # 오른쪽 중간
        ((-2.0, -5.5), 0.40),  # 왼쪽 아래쪽
        ((6.5, 3.5), 0.45),    # 오른쪽 위쪽
        ((2.0, 0.5), 0.35),    # 중앙 근처
    ]]
    return w, np.array([-9.0, -8.0, np.radians(35)]), np.array([9.0, 8.0])

def walls(seed=0):
    """벽이 있는 환경. 벽을 원 하나로 감싸면 안 되는 이유가 드러난다."""
    w = World(bounds=BOUNDS)
    w.walls = [(np.array(a, float), np.array(b, float)) for a, b in [
        ((-6.0, -1.0), (2.0, -1.0)),
        ((4.5, 2.0), (4.5, 9.0)),
        ((-3.0, 3.5), (3.0, 3.5)),
        ((-8.0, 4.0), (-8.0, 9.5)),
    ]]
    w.circles = [(np.array((0.5, 6.0)), 0.6), (np.array((-4.0, 0.8)), 0.45)]
    return w, np.array([-9.0, -6.0, np.radians(20)]), np.array([9.0, 8.0])


def trap(seed=0):
    """
    ㄷ자 막다른 골목. 접선 그래프만으로는 못 나온다.
    격자 A* 폴백이 실제로 동작하는지 보는 시나리오.
    """
    w = World(bounds=BOUNDS)
    w.walls = [(np.array(a, float), np.array(b, float)) for a, b in [
        ((-1.0, -5.0), (-1.0, 5.0)),     # 안쪽 벽
        ((-1.0, 5.0), (7.0, 5.0)),       # 위쪽 날개
        ((-1.0, -5.0), (7.0, -5.0)),     # 아래쪽 날개
    ]]
    w.circles = [(np.array((5.0, 0.0)), 0.5)]
    return w, np.array([3.0, 0.0, np.radians(160)]), np.array([-10.0, 0.0])


def forest(seed=1):
    """기둥이 빽빽한 숲. 계획 시간과 병합 동작을 본다."""
    rng = np.random.default_rng(seed)
    w = World(bounds=BOUNDS)
    cs = []
    while len(cs) < 26:
        p = rng.uniform([-8.0, -8.0], [8.0, 8.0])
        if np.hypot(*(p - np.array([-9.5, -9.5]))) < 3.0:
            continue
        if np.hypot(*(p - np.array([9.5, 9.5]))) < 3.0:
            continue
        if any(np.hypot(*(p - c)) < 1.7 for c, _ in cs):
            continue
        cs.append((p, float(rng.uniform(0.25, 0.6))))
    w.circles = cs
    return w, np.array([-9.5, -9.5, np.radians(45)]), np.array([9.5, 9.5])


def narrow(seed=0):
    """
    좁은 통로. 부풀린 원이 서로 겹쳐 병합되면서
    '여기는 못 지나간다'가 자동으로 판정되는지 본다.
    """
    w = World(bounds=BOUNDS)
    w.circles = [(np.array(c), r) for c, r in [
        ((0.0, 1.2), 0.45), ((0.0, -1.2), 0.45),      # 좁은 문 (통과 불가)
        ((0.0, 4.9), 0.45), ((0.0, 8.6), 0.45),       # 넓은 문 (통과 가능)
        ((0.0, 3.4), 0.45), ((0.0, -3.6), 0.45),
        ((0.0, -6.0), 0.45), ((0.0, -8.4), 0.45),
    ]]
    return w, np.array([-6.0, 0.0, 0.0]), np.array([6.0, 0.5])


# ============================================================== 무작위 기둥 공간
#
# 고정된 지도 다섯 개로만 재고 설정을 고치면, 알고리즘이 좋아진 건지 그 다섯 개에
# 맞춰진 건지 구분할 수 없다. 그래서 실제로 시험할 공간과 닮은 지도를 매번 새로 만든다.
#
# 실제 시험 공간 (사용자에게 확인): 원통형 기둥만 있고, 벽은 달리는 곳에서 멀리 떨어져 있다.
#
# 지도 번호를 두 묶음으로 나눈다.
#   RANDOM_TUNE : 설정을 고칠 때 보는 지도
#   RANDOM_TEST : 채점할 때만 보는 지도. 이걸 보고 설정을 고치면 안 된다.
RANDOM_TUNE = tuple(range(0, 40))
RANDOM_TEST = tuple(range(1000, 1040))
# 1000 번대는 채점하다가 실패한 지도(1026)를 들여다보고 지도 기록 방식을 고쳤다.
# 그래서 더 이상 '처음 보는 지도'가 아니다. 그 뒤의 최종 채점은 2000 번대로 했다.
# 2000 번대를 보고 설정을 또 고치면, 다음 채점은 3000 번대처럼 새 묶음으로 해야 한다.
RANDOM_TEST2 = tuple(range(2000, 2040))

# 풀 수 있는지 검사할 때 차 중심이 장애물 표면에서 떨어져야 하는 거리.
# 차 폭 절반(0.14 m)에 0.10 m 여유. 즉 표면 사이 0.48 m 틈이면 지나갈 수 있다고 본다.
SOLVE_CLEARANCE = 0.24


def _solvable(world, start, goal, clearance=SOLVE_CLEARANCE, res=0.08):
    """실제 지도에서 차 중심이 지나갈 길이 있는가 (방향은 무시한 격자 탐색)."""
    from .hybrid import wavefront
    x0, y0, x1, y1 = world.bounds
    xs = np.arange(x0, x1, res) + res / 2
    ys = np.arange(y0, y1, res) + res / 2
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    free = np.ones(X.shape, bool)
    for c, r in world.circles:
        free &= np.hypot(X - c[0], Y - c[1]) > r + clearance
    for a, b in world.walls:
        ab = b - a
        t = np.clip(((X - a[0]) * ab[0] + (Y - a[1]) * ab[1]) / float(ab @ ab), 0, 1)
        free &= np.hypot(X - (a[0] + t * ab[0]), Y - (a[1] + t * ab[1])) > clearance
    si = (int((start[0] - x0) / res), int((start[1] - y0) / res))
    gi = (int((goal[0] - x0) / res), int((goal[1] - y0) / res))
    if not (free[si] and free[gi]):
        return False
    return bool(np.isfinite(wavefront(free, gi)[si]))


def random_pillars(seed=0):
    """
    무작위 기둥 공간. 같은 seed 면 언제나 같은 지도가 나온다.

      달리는 공간 : 가로 5~10 m, 세로 5~10 m
      기둥        : 반지름 0.05~0.30 m (가정), 공간 1 m^2 당 0.10~0.35 개
      벽          : 달리는 공간 바깥 1.5~3.0 m 에 네모로 두른다
      출발, 목적지: 공간의 서로 반대쪽 끝. 출발 방향은 목적지 쪽에서 최대 90 도 틀어진다
    풀 수 없는 지도가 나오면 같은 seed 안에서 다시 뽑는다.
    """
    rng = np.random.default_rng(10_000 + int(seed))
    for _ in range(200):
        W, H = rng.uniform(5.0, 10.0, 2)
        margin = rng.uniform(1.5, 3.0)
        # 출발과 목적지는 긴 쪽 방향의 양 끝
        if W >= H:
            start = np.array([-W / 2 + 0.6, rng.uniform(-H / 2 + 0.8, H / 2 - 0.8)])
            goal = np.array([W / 2 - 0.6, rng.uniform(-H / 2 + 0.8, H / 2 - 0.8)])
        else:
            start = np.array([rng.uniform(-W / 2 + 0.8, W / 2 - 0.8), -H / 2 + 0.6])
            goal = np.array([rng.uniform(-W / 2 + 0.8, W / 2 - 0.8), H / 2 - 0.6])
        if rng.random() < 0.5:
            start, goal = goal, start

        n_target = int(W * H * rng.uniform(0.10, 0.35))
        circles = []
        tries = 0
        while len(circles) < n_target and tries < 4000:
            tries += 1
            r = float(rng.uniform(0.05, 0.30))
            c = rng.uniform([-W / 2 + r, -H / 2 + r], [W / 2 - r, H / 2 - r])
            if np.hypot(*(c - start)) < 1.0 + r or np.hypot(*(c - goal)) < 1.0 + r:
                continue
            if any(np.hypot(*(c - c2)) < r + r2 + 0.25 for c2, r2 in circles):
                continue
            circles.append((c, r))

        bx, by = W / 2 + margin, H / 2 + margin
        B = 16.0
        w = World(bounds=(-B, -B, B, B))
        w.circles = circles
        corners = [(-bx, -by), (bx, -by), (bx, by), (-bx, by)]
        w.walls = [(np.array(corners[i], float), np.array(corners[(i + 1) % 4], float))
                   for i in range(4)]

        to_goal = np.arctan2(*(goal - start)[::-1])
        heading = to_goal + rng.uniform(-np.pi / 2, np.pi / 2)
        if _solvable(w, start, goal):
            return w, np.array([start[0], start[1], heading]), goal
    raise RuntimeError(f"seed {seed}: 풀 수 있는 지도를 만들지 못했다")


def arena(seed=0):
    """
    우리 시험장 크기 (car_specs 의 시험 공간, 안 쟀으면 계획서의 1.5 m) 에 기둥 3~6개를 무작위로 놓음.
    출발은 왼쪽, 목표는 오른쪽. 벽은 없음. 원래 run_error_split.py 안에 있던 것을 주행 시험도 쓰게 옮김.
    """
    from . import car_specs
    rng = np.random.default_rng(50_000 + seed)
    width, length = car_specs.arena_m()
    radius = car_specs.pillar_radius_m()
    start = np.array([0.15 * width, rng.uniform(0.3, 0.7) * length, 0.0])
    goal = np.array([0.85 * width, rng.uniform(0.3, 0.7) * length])
    count = int(rng.integers(3, 7))
    circles = []
    while len(circles) < count:
        center = rng.uniform([0.3 * width, 0.1 * length], [0.7 * width, 0.9 * length])
        if any(np.hypot(*(center - np.asarray(other))) < 2 * radius + 0.05 for other, _ in circles):
            continue
        # 범퍼 바로 앞에 기둥이 있으면 후진 없이는 못 피함 (예전 2, 10, 11, 18 번 지도: 범퍼에서 1~8 cm).
        # 그래서 출발할 때 범퍼에서 차 길이 하나 안에는 안 놓음. 실제 시험장에 놓을 때도 같은 규칙을 씀
        if np.hypot(*(center - start[:2])) - radius < car_specs.START_CLEAR_M:
            continue
        circles.append((tuple(center), radius))
    return World(circles=circles, bounds=BOUNDS), start, goal


ALL = {
    "arena": arena,
    "pillars": pillars,
    "walls": walls,
    "trap": trap,
    "forest": forest,
    "narrow": narrow,
    "random": random_pillars,
}


def make(name, seed=0):
    if name not in ALL:
        raise SystemExit(f"unknown scenario: {name}. 선택지: {list(ALL)}")
    return ALL[name](seed)
