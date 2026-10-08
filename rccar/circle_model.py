"""
circle_model.py -- 처음 생각한 기하 모델의 계산 함수 (계획서 6-가)

계획서에 적은 수식을 교과서(공통수학2 도형의 방정식) 방식 그대로 옮김.
geometry.py 는 각도(acos)로 접점을 구하지만, 여기서는 일부러 기울기 이차방정식을 씀.
처음 모델이 어디까지 버티고 어디서 무너지는지 보려는 것이라 계산 방식도 처음 그대로 둠.

  점   (x, y) 튜플
  원   Circle(x, y, r)  -- 중심과 반지름. 차의 반폭은 이미 더해 둔 반지름
  직선 Line            -- y = slope * x + intercept, 또는 수직선 x = vertical_x
"""

from collections import namedtuple
from math import atan2, cos, hypot, pi, sin, sqrt

Circle = namedtuple("Circle", "x y r")

# 계산 오차 때문에 딱 붙은 경우를 '만난다'로 볼지 '안 만난다'로 볼지 흔들리지 않게 하는 여유
TINY = 1e-9


class Line:
    """y = slope * x + intercept. 수직선이면 slope 대신 vertical_x 를 씀."""

    def __init__(self, slope=None, intercept=None, vertical_x=None):
        self.slope = slope
        self.intercept = intercept
        self.vertical_x = vertical_x

    @property
    def is_vertical(self):
        return self.vertical_x is not None

    def __repr__(self):
        if self.is_vertical:
            return f"x = {self.vertical_x:.4f}"
        return f"y = {self.slope:.4f}x + {self.intercept:.4f}"


def distance(a, b):
    """두 점 사이의 거리."""
    return hypot(b[0] - a[0], b[1] - a[1])


def line_through(p, q):
    """두 점을 지나는 직선. x 좌표가 같으면 기울기가 없으므로 수직선으로 따로 만듦."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    if abs(dx) < TINY * max(1.0, abs(dy)):
        return Line(vertical_x=p[0])
    slope = dy / dx
    return Line(slope=slope, intercept=p[1] - slope * p[0])


def line_with_slope(point, slope):
    """한 점과 기울기로 정한 직선. 접선을 만들 때 씀."""
    return Line(slope=slope, intercept=point[1] - slope * point[0])


# ── 충돌 여부 판정 ────────────────────────────────────────────────────

def discriminant(line, circle):
    """직선의 식을 원의 방정식에 대입한 이차방정식의 판별식 D.

    y = mx + k 를 (x - a)^2 + (y - b)^2 = r^2 에 넣으면
      (1 + m^2) x^2 + 2(m(k - b) - a) x + a^2 + (k - b)^2 - r^2 = 0
    수직선 x = x0 이면 (y - b)^2 = r^2 - (x0 - a)^2 라서 D = 4(r^2 - (x0 - a)^2) 로 둠.
    """
    a, b, r = circle
    if line.is_vertical:
        return 4.0 * (r * r - (line.vertical_x - a) ** 2)
    m, k = line.slope, line.intercept
    quad = 1.0 + m * m
    lin = 2.0 * (m * (k - b) - a)
    const = a * a + (k - b) ** 2 - r * r
    return lin * lin - 4.0 * quad * const


def line_cuts_circle(line, circle):
    """직선이 원을 두 점에서 뚫고 지나가는가 (D > 0). 접하는 것(D = 0)은 안 뚫는 것으로 봄.

    D 는 기울기가 클수록 같이 커져서 0 근처를 그대로 비교하면 흔들림.
    D 를 4(1 + m^2) 로 나누면 r^2 - (중심과 직선 사이 거리)^2 이 되므로, 이 값으로 비교함.
    """
    scale = 4.0 if line.is_vertical else 4.0 * (1.0 + line.slope ** 2)
    return discriminant(line, circle) / scale > TINY * circle.r * circle.r


def foot_of_perpendicular(point, line):
    """점에서 직선에 내린 수선의 발.

    y = mx + k 에 수직이고 (c, d) 를 지나는 직선은 y = -(x - c)/m + d.
    두 식을 연립하면 x = (c + m(d - k)) / (1 + m^2). 이 식은 m = 0 (수평선)일 때도 그대로 맞음.
    """
    c, d = point
    if line.is_vertical:
        return (line.vertical_x, d)
    m, k = line.slope, line.intercept
    x = (c + m * (d - k)) / (1.0 + m * m)
    return (x, m * x + k)


def _position_along(point, start, end):
    """start 에서 end 쪽으로 point 가 얼마나 나아갔는지 (start 는 0, end 는 1).

    셋이 한 직선 위에 있다고 보고, 더 많이 변하는 좌표 하나만으로 비율을 셈.
    """
    dx, dy = end[0] - start[0], end[1] - start[1]
    if abs(dx) >= abs(dy):
        return (point[0] - start[0]) / dx if dx else 0.0
    return (point[1] - start[1]) / dy


def is_between(point, start, end):
    """한 직선 위의 point 가 start 와 end 사이에 있는가."""
    position = _position_along(point, start, end)
    return -TINY <= position <= 1.0 + TINY


def is_inside(point, circle, margin=TINY):
    """점이 원 안쪽에 있는가. 경계 위는 안쪽이 아님."""
    return distance(point, (circle.x, circle.y)) < circle.r - margin


def segment_hits_circle(start, end, circle):
    """선분 start-end 가 원을 지나가는가 (계획서 6-가-1).

    판별식만 보면 직선 전체를 보므로 차 뒤쪽이나 목표 너머의 원까지 걸림.
    그래서 원의 중심에서 내린 수선의 발이 두 점 사이에 있는지도 같이 봄.
    수선의 발이 바깥이면 원에 가장 가까운 곳은 양 끝점이므로, 끝점이 원 안인지만 보면 됨.
    """
    if is_inside(start, circle) or is_inside(end, circle):
        return True
    line = line_through(start, end)
    if not line_cuts_circle(line, circle):
        return False
    foot = foot_of_perpendicular((circle.x, circle.y), line)
    return is_between(foot, start, end)


# ── 접선과 접점 ──────────────────────────────────────────────────────

def tangent_lines_from(point, circle):
    """원 밖의 점에서 원에 그은 두 접선 (계획서 6-가-2).

    점 (p, q) 를 지나고 기울기가 m 인 직선과 중심 사이의 거리가 r 이 되는 조건을 세우면
      (a^2 - r^2) m^2 - 2ab m + (b^2 - r^2) = 0     (a, b 는 점에서 중심까지의 x, y 차이)
    이 되고, 두 근이 두 접선의 기울기임. 판별식/4 = r^2 (a^2 + b^2 - r^2) 라서 원 밖이면 늘 양수.
    a^2 = r^2 이면 m^2 항이 사라짐. 이때 접선 하나는 기울기가 없는 수직선이라 따로 처리함.
    """
    p, q = point
    a, b, r = circle.x - p, circle.y - q, circle.r
    if a * a + b * b <= r * r * (1.0 + TINY):
        return []
    square_term = a * a - r * r
    if abs(square_term) < TINY * r * r:
        vertical = Line(vertical_x=p)
        other_slope = (b * b - r * r) / (2.0 * a * b)
        return [vertical, line_with_slope(point, other_slope)]
    root = r * sqrt(a * a + b * b - r * r)
    return [line_with_slope(point, (a * b + root) / square_term),
            line_with_slope(point, (a * b - root) / square_term)]


def tangent_point(circle, tangent_line):
    """접점 = 중심에서 접선에 내린 수선의 발."""
    return foot_of_perpendicular((circle.x, circle.y), tangent_line)


def side_of(start, end, point):
    """start 에서 end 를 바라볼 때 point 가 왼쪽이면 양수, 오른쪽이면 음수, 직선 위면 0.

    두 기울기를 비교하는 식에서 분모를 없앤 것. 분모가 0 인 수직선도 따로 다루지 않아도 됨.
    """
    return ((end[0] - start[0]) * (point[1] - start[1])
            - (end[1] - start[1]) * (point[0] - start[0]))


def tangent_points_by_side(point, circle):
    """접점 두 개를 '왼쪽으로 비켜 가는 접점'과 '오른쪽으로 비켜 가는 접점'으로 나눔."""
    touch = [tangent_point(circle, line) for line in tangent_lines_from(point, circle)]
    if len(touch) != 2:
        return {}
    center = (circle.x, circle.y)
    if side_of(point, center, touch[0]) > 0:
        return {"left": touch[0], "right": touch[1]}
    return {"left": touch[1], "right": touch[0]}


# 왼쪽으로 비켜 가면 원이 차의 오른쪽에 있으니 원을 시계 방향으로 돌게 됨
TURN_OF_SIDE = {"left": "clockwise", "right": "counterclockwise"}


# ── 호 ──────────────────────────────────────────────────────────────

def angle_on_circle(point, circle):
    """원 위의 점이 중심에서 몇 도(라디안) 방향에 있는가."""
    return atan2(point[1] - circle.y, point[0] - circle.x)


def point_at_angle(circle, angle):
    return (circle.x + circle.r * cos(angle), circle.y + circle.r * sin(angle))


def central_angle(circle, start, end, turn):
    """start 에서 end 까지 정해진 방향(turn)으로 돈 중심각. 0 이상 2파이 미만.

    짧은 쪽 호를 고르면 안 됨. 반대 방향으로 돌면 장애물을 뚫고 지나가는 길이 나옴.
    """
    begin, finish = angle_on_circle(start, circle), angle_on_circle(end, circle)
    swept = finish - begin if turn == "counterclockwise" else begin - finish
    return swept % (2.0 * pi)


def arc_length(circle, start, end, turn):
    """호의 길이 = 원 둘레 x (중심각 / 한 바퀴)."""
    circumference = 2.0 * pi * circle.r
    return circumference * central_angle(circle, start, end, turn) / (2.0 * pi)


def leave_point(circle, goal, turn):
    """원을 turn 방향으로 돌다가 목표 쪽으로 곧장 떠날 수 있게 되는 접점.

    목표에서 원에 그은 두 접점 중, 그 점에서 도는 방향이 목표를 향하는 쪽을 고름.
    도는 방향의 진행 방향은 반지름에 수직이므로 (중심에서 점으로 가는 방향을 90도 돌린 것).
    """
    candidates = [tangent_point(circle, line) for line in tangent_lines_from(goal, circle)]
    for touch in candidates:
        radial_x, radial_y = touch[0] - circle.x, touch[1] - circle.y
        if turn == "counterclockwise":
            heading = (-radial_y, radial_x)
        else:
            heading = (radial_y, -radial_x)
        toward_goal = (goal[0] - touch[0], goal[1] - touch[1])
        if heading[0] * toward_goal[0] + heading[1] * toward_goal[1] > 0:
            return touch
    return None


def detour_length(start, goal, circle, side):
    """'접선 + 호 + 접선' 우회 경로의 전체 길이 (계획서 6-가-3)."""
    touch = tangent_points_by_side(start, circle).get(side)
    if touch is None:
        return None
    turn = TURN_OF_SIDE[side]
    leave = leave_point(circle, goal, turn)
    if leave is None:
        return None
    return (distance(start, touch) + arc_length(circle, touch, leave, turn)
            + distance(leave, goal))


def leaving_is_clear(point, goal, circle):
    """원 위의 점에서 목표로 곧장 가도 이 원을 다시 지나지 않는가 (계획서 6-가-3).

    점이 이미 원 위에 있어서 중심과 직선 사이 거리는 언제나 r 이하라 거리로는 판단이 안 됨.
    그래서 중심에서 '점-목표' 직선에 내린 수선의 발이 점보다 뒤로 넘어갔는지를 봄.
    수선의 발이 점과 목표 사이에 있으면 앞으로 원 안을 지나가야 하고, 점 뒤에 있으면 트인 것임.
    """
    foot = foot_of_perpendicular((circle.x, circle.y), line_through(point, goal))
    return _position_along(foot, point, goal) <= TINY


# ── 두 원의 교점 ─────────────────────────────────────────────────────

def circle_intersections(first, second):
    """두 원의 교점. 두 원의 방정식을 빼면 공통현의 직선이 나오고, 그 직선과 원의 교점을 구함."""
    a1, b1, r1 = first
    a2, b2, r2 = second
    gap = distance((a1, b1), (a2, b2))
    if gap >= r1 + r2 or gap <= abs(r1 - r2) or gap == 0:
        return []
    # 두 식을 빼면 2(a2 - a1)x + 2(b2 - b1)y = r1^2 - r2^2 - a1^2 + a2^2 - b1^2 + b2^2
    coef_x, coef_y = 2.0 * (a2 - a1), 2.0 * (b2 - b1)
    right = r1 * r1 - r2 * r2 - a1 * a1 + a2 * a2 - b1 * b1 + b2 * b2
    if abs(coef_y) < TINY:
        chord = Line(vertical_x=right / coef_x)
    else:
        chord = Line(slope=-coef_x / coef_y, intercept=right / coef_y)
    middle = foot_of_perpendicular((a1, b1), chord)
    half = sqrt(max(0.0, r1 * r1 - distance(middle, (a1, b1)) ** 2))
    # 공통현 방향의 단위 벡터로 중점에서 양쪽으로 half 만큼 감
    along_x, along_y = -(b2 - b1) / gap, (a2 - a1) / gap
    return [(middle[0] + half * along_x, middle[1] + half * along_y),
            (middle[0] - half * along_x, middle[1] - half * along_y)]
