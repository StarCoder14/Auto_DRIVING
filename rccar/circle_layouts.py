"""
circle_layouts.py -- 처음 기하 모델을 시험할 장애물 배치

계획서의 실험 환경을 따름: 가로세로 1.5 m 평면, 지름 60~100 mm 원기둥.
반지름은 실제 기둥 반지름이고, 시험할 때 차 반폭을 더해 부풀림.
"""

import random

from . import car_specs
from .circle_model import Circle, distance

ARENA = 1.5                 # 실험 평면 한 변 (m)
CAR_HALF_WIDTH = car_specs.half_width_m(fallback=0.07)   # 차 반폭 (m). car_specs.py 에 전폭을 재 넣으면 따라감


class Layout:
    def __init__(self, name, description, start, goal, pillars):
        self.name = name
        self.description = description
        self.start = start
        self.goal = goal
        self.pillars = pillars      # 실제 기둥 (부풀리기 전)

    def inflated(self, half_width=CAR_HALF_WIDTH):
        """차를 점으로 보기 위해 기둥 반지름에 차 반폭을 더함."""
        return [Circle(p.x, p.y, p.r + half_width) for p in self.pillars]


def _row(x_from, x_to, y, count, r):
    gap = (x_to - x_from) / (count - 1)
    return [Circle(x_from + gap * i, y, r) for i in range(count)]


def _column(x, y_from, y_to, count, r):
    gap = (y_to - y_from) / (count - 1)
    return [Circle(x, y_from + gap * i, r) for i in range(count)]


FIXED = [
    Layout("one", "기둥 하나. GeoGebra 로 작도해서 맞춰 볼 기본 예제",
           (0.0, 0.70), (1.5, 0.75), [Circle(0.75, 0.75, 0.04)]),
    Layout("tie", "기둥 중심이 출발-목표 직선 위. 좌우 길이가 같아 오른쪽 규칙이 쓰임",
           (0.0, 0.75), (1.5, 0.75), [Circle(0.75, 0.75, 0.04)]),
    Layout("scattered", "흩어진 기둥 다섯 개",
           (0.05, 0.10), (1.45, 1.40),
           [Circle(0.35, 0.30, 0.04), Circle(0.60, 0.65, 0.05), Circle(0.95, 0.80, 0.03),
            Circle(1.10, 1.15, 0.05), Circle(0.40, 1.00, 0.04)]),
    Layout("close_pair", "붙어 있는 기둥 둘. 부풀리면 겹쳐서 한 덩어리가 됨",
           (0.0, 0.75), (1.5, 0.80), [Circle(0.70, 0.70, 0.04), Circle(0.80, 0.82, 0.04)]),
    Layout("wall", "기둥 여러 개가 벽처럼 늘어섬. 틈은 차가 못 지나갈 만큼 좁음",
           (0.75, 0.10), (0.75, 1.40), _row(0.35, 1.15, 0.75, 6, 0.04)),
    Layout("cup", "목표 앞을 컵 모양으로 둘러싼 기둥. 입구는 출발점 쪽",
           (0.10, 0.75), (1.40, 0.75),
           _column(1.05, 0.35, 1.15, 6, 0.04) + _row(0.65, 0.89, 0.35, 3, 0.04)
           + _row(0.65, 0.89, 1.15, 3, 0.04)),
    Layout("hidden_goal", "목표가 기둥 바로 뒤에 숨어 있음",
           (0.0, 0.75), (0.95, 0.76), [Circle(0.80, 0.75, 0.04)]),
    Layout("goal_inside", "목표가 부풀린 기둥 안. 도달할 수 없음",
           (0.0, 0.75), (0.82, 0.75), [Circle(0.75, 0.75, 0.04)]),
]

BY_NAME = {layout.name: layout for layout in FIXED}


def random_layout(seed, count=None):
    """계획서대로 1.5 m 평면에 지름 60~100 mm 기둥을 3~6개 무작위로 놓음.

    출발점과 목표는 왼쪽과 오른쪽 가장자리. 기둥이 출발점이나 목표를 덮으면 다시 뽑음.
    """
    rng = random.Random(seed)
    start = (0.05, rng.uniform(0.2, 1.3))
    goal = (1.45, rng.uniform(0.2, 1.3))
    count = count or rng.randint(3, 6)
    pillars = []
    while len(pillars) < count:
        pillar = Circle(rng.uniform(0.25, 1.25), rng.uniform(0.1, 1.4), rng.uniform(0.03, 0.05))
        room = pillar.r + CAR_HALF_WIDTH + 0.02
        center = (pillar.x, pillar.y)
        if distance(center, start) < room or distance(center, goal) < room:
            continue
        pillars.append(pillar)
    return Layout(f"random{seed}", f"무작위 배치 {seed}번 (기둥 {count}개)", start, goal, pillars)
