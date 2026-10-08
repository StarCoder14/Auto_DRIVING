"""
circle_avoider.py -- 처음 생각한 기하 모델로 길 찾기 (계획서의 알고리즘 A)

눈앞에 보이는 원만 보고, 목표를 막는 가장 가까운 원 하나를 접선으로 비켜 감.
그 원을 따라 돌다가 목표 쪽이 트이면 떠나서 다시 같은 판단을 반복함.

  곧장 가기   목표까지 선분을 막는 원이 없으면 목표로 감
  비켜 가기   가장 가까운 원에 그은 두 접선 중 '접선 + 호 + 접선' 이 짧은 쪽 접점으로 감
              길이가 같으면 오른쪽 (계획서에서 미리 정한 규칙)
  따라 돌기   접점에 닿으면 그 원을 따라 돎. 수선의 발이 넘어가면 목표 쪽으로 떠남
  기준선 규칙 출발점-목표를 잇는 선을 기억해 두고, 처음 부딪힌 곳보다 목표에 가까울 때만 떠남
              (반복이 끝나도록 하려는 장치. Bug2 알고리즘의 종료 조건과 같은 생각)

계획서에 없던 빈 곳을 메운 규칙 하나
  원에서 떠나는 순간 차는 원 위에 서 있음. 다음 원을 비켜 가는 접선이 방금 떠난 원을 다시
  지나면 그 원을 피해야 하는데, 원 위의 점에서는 그 원에 접선 두 개를 그을 수 없음.
  계획서 규칙으로는 할 일이 정해지지 않으므로, 이때는 떠나지 않고 그 원을 계속 따라 돌게 함.
  처음엔 한 걸음 목표 쪽으로 갔다가 다시 판단하게 했는데, 그러면 같은 원에 반대쪽으로 다시
  부딪혀 떠났다 부딪히기를 끝없이 되풀이했음 (무작위 배치 140번).

차는 크기 없는 점이고, 회전 반경도 센서 잡음도 없음. 그게 이 모델의 가정이고,
나중에 실제 차와 비교해서 그 가정이 얼마나 손해를 보는지 재는 것이 목적임.
"""

from math import inf

from . import circle_model as M


class AvoiderSettings:
    def __init__(self,
                 step=0.01,               # 한 번에 움직이는 거리 (m)
                 sensor_range=inf,        # 원 표면까지 이 거리 안에 들어와야 보임 (m)
                 use_baseline_rule=True,  # 기준선 규칙을 쓸지
                 max_steps=20000):        # 이만큼 움직여도 못 가면 실패로 봄
        self.step = step
        self.sensor_range = sensor_range
        self.use_baseline_rule = use_baseline_rule
        self.max_steps = max_steps


class Trip:
    """한 번 달린 기록."""

    def __init__(self, start, goal):
        self.start = start
        self.goal = goal
        self.path = [start]
        self.events = []         # (몇 번째 걸음, 종류, 설명)
        self.result = None       # 'arrived', 'stuck', 'loop', 'timeout', 'goal_blocked', 'collision'
        self.side_flips = 0      # 비켜 갈 방향을 바꾼 횟수 (갈팡질팡한 정도)

    @property
    def length(self):
        return sum(M.distance(a, b) for a, b in zip(self.path, self.path[1:]))

    @property
    def straight(self):
        return M.distance(self.start, self.goal)

    @property
    def arrived(self):
        return self.result == "arrived"

    def note(self, step_number, kind, detail=""):
        self.events.append((step_number, kind, detail))


def visible_circles(position, circles, settings):
    """센서 거리 안에 들어온 원만 보임. 원 표면까지의 거리로 셈."""
    return [c for c in circles
            if M.distance(position, (c.x, c.y)) - c.r <= settings.sensor_range]


def nearest_blocker(start, end, circles, ignore=None):
    """선분 start-end 를 막는 원 중 가장 가까운 것."""
    blockers = [c for c in circles if c is not ignore and M.segment_hits_circle(start, end, c)]
    if not blockers:
        return None
    return min(blockers, key=lambda c: M.distance(start, (c.x, c.y)) - c.r)


def choose_side(position, goal, circle):
    """좌우 중 '접선 + 호 + 접선' 이 짧은 쪽. 길이가 같으면 오른쪽."""
    left = M.detour_length(position, goal, circle, "left")
    right = M.detour_length(position, goal, circle, "right")
    if left is None and right is None:
        return None
    if right is None:
        return "left"
    if left is None:
        return "right"
    # 장애물 중심이 경로 위에 놓이면 두 길이가 같아짐. 이때 흔들리지 않게 오른쪽으로 정해 둠
    if left < right - M.TINY * max(1.0, right):
        return "left"
    return "right"


def plan_detour(position, goal, circles):
    """다음 경유점(접점)과 그때 돌 원과 방향을 정함 (계획서: 다음 경유점을 내놓는 함수).

    목표를 막는 가장 가까운 원을 골라 접점을 구함. 그런데 접점으로 가는 길을 또 다른 원이
    막을 수 있음. 그러면 그 원이 '가장 가까운 원'이 되므로 그 원으로 바꿔서 다시 구함.
    돌려주는 마지막 값은 접점까지 가는 길이 정말 비었는지. 원끼리 겹쳐 있으면 두 원 사이를
    오가기만 하고 빈 길을 못 찾음. 그래도 이 모델은 원 하나만 보고 가므로 그대로 가게 둠.
    """
    circle = nearest_blocker(position, goal, circles)
    if circle is None:
        return None
    side = touch = None
    for _ in range(len(circles) + 1):
        side = choose_side(position, goal, circle)
        if side is None:
            return circle, None, None, False
        touch = M.tangent_points_by_side(position, circle)[side]
        closer = nearest_blocker(position, touch, circles, ignore=circle)
        if closer is None:
            return circle, side, touch, True
        circle = closer
    return circle, side, touch, False


def move_toward(position, target, step):
    gap = M.distance(position, target)
    if gap <= step:
        return target
    ratio = step / gap
    return (position[0] + (target[0] - position[0]) * ratio,
            position[1] + (target[1] - position[1]) * ratio)


def crossed_baseline(start, goal, before, after):
    """출발점-목표 기준선을 이번 걸음에 건넜는가. 왼쪽/오른쪽 부호가 바뀌면 건넌 것."""
    side_before = M.side_of(start, goal, before)
    side_after = M.side_of(start, goal, after)
    return side_before * side_after <= 0 and (side_before != 0 or side_after != 0)


def drive(start, goal, circles, settings=None):
    """처음 모델로 start 에서 goal 까지 달려 본 기록을 돌려줌."""
    settings = settings or AvoiderSettings()
    trip = Trip(start, goal)
    for circle in circles:
        if M.is_inside(goal, circle):
            trip.result = "goal_blocked"
            trip.note(0, "goal_blocked", "목표가 부풀린 원 안에 있음")
            return trip

    position = start
    following = None          # 따라 도는 중이면 (원, 도는 방향, 처음 부딪힌 곳에서 목표까지 거리)
    hit_position = None       # 처음 부딪힌 곳. 따라 돌다가 여기로 돌아오면 갇힌 것
    walked_since_hit = 0.0
    last_side = None
    hits_on = {}              # 원마다 부딪힌 횟수. 같은 원에 계속 다시 부딪히면 맴도는 것
    just_left = None          # 방금 떠난 원과 그때 따라 돌던 정보. 떠나자마자 다시 막히면 되돌아감
    keep_going = False        # 되돌아가 따라 돌 때는 한 걸음은 돌고 나서 떠날지 봄
    buried_plan = None        # 가려던 접점이 겹친 원 안에 묻혀 있던 순간 (충돌 원인 설명용)

    for step_number in range(1, settings.max_steps + 1):
        if any(M.is_inside(position, c, margin=1e-6) for c in circles):
            trip.result = "collision"
            trip.note(step_number, "collision", buried_plan or "부풀린 원 안으로 들어감")
            return trip
        seen = visible_circles(position, circles, settings)

        if following is None:
            if M.distance(position, goal) <= settings.step:
                trip.path.append(goal)
                trip.result = "arrived"
                trip.note(step_number, "arrived")
                return trip
            detour = plan_detour(position, goal, seen)
            if detour is None:
                position = move_toward(position, goal, settings.step)
                trip.path.append(position)
                just_left = None
                continue
            circle, side, touch, path_is_clear = detour
            if side is None:
                # 원 위에 서 있는데 그 원이 또 앞을 막으면 그 원에는 접선을 그을 수 없음
                if just_left is not None and just_left[0] is circle:
                    following, keep_going = just_left, True
                    just_left = None
                    trip.note(step_number, "resume", "떠나려던 원이 다시 길을 막아 계속 따라 돎")
                    continue
                trip.result = "stuck"
                trip.note(step_number, "stuck", "서 있는 원이 다시 앞을 막아 접선을 그을 수 없음")
                return trip
            just_left = None
            buried_plan = None if path_is_clear else "가려던 접점이 겹친 다른 원 안에 묻혀 있었음"
            if last_side is not None and side != last_side:
                trip.side_flips += 1
            last_side = side
            if M.distance(position, touch) > settings.step:
                position = move_toward(position, touch, settings.step)
                trip.path.append(position)
                continue
            # 접점에 닿음. 여기서부터 원을 따라 돎
            position = touch
            trip.path.append(position)
            following = (circle, M.TURN_OF_SIDE[side], M.distance(position, goal))
            hit_position, walked_since_hit = position, 0.0
            hits_on[circle] = hits_on.get(circle, 0) + 1
            if hits_on[circle] > 20:
                trip.result = "loop"
                trip.note(step_number, "loop", "같은 원에서 떠났다가 다시 부딪히기를 되풀이함")
                return trip
            trip.note(step_number, "hit", f"{side} 쪽으로 비켜 감, 원 중심 ({circle.x:.2f}, {circle.y:.2f})")
            continue

        circle, turn, hit_distance = following
        if not keep_going and _can_leave(position, goal, circle, hit_distance, settings):
            just_left = following
            following = None
            trip.note(step_number, "leave", f"목표까지 {M.distance(position, goal):.3f} m 에서 떠남")
            continue

        keep_going = False
        # 원을 따라 한 걸음. 걸음 길이만큼의 중심각을 돎
        angle_step = settings.step / circle.r
        angle_now = M.angle_on_circle(position, circle)
        angle_next = angle_now + (angle_step if turn == "counterclockwise" else -angle_step)
        next_position = M.point_at_angle(circle, angle_next)

        # 따라 돌던 원이 다른 원과 겹쳐 있으면 그 원 안으로 들어가게 됨. 겹친 곳에서 그 원으로 옮겨 탐
        other = next((c for c in seen if c is not circle and M.is_inside(next_position, c)), None)
        if other is not None:
            crossing = _first_crossing(circle, other, position, turn)
            if crossing is None:
                trip.result = "collision"
                trip.note(step_number, "collision", "겹친 원의 교점을 못 찾음")
                return trip
            next_position = crossing
            following = (other, turn, hit_distance)
            trip.note(step_number, "switch", f"겹친 원 ({other.x:.2f}, {other.y:.2f}) 으로 옮겨 탐")

        if (settings.use_baseline_rule
                and crossed_baseline(start, goal, position, next_position)
                and M.distance(next_position, goal) < hit_distance - M.TINY):
            trip.path.append(next_position)
            position = next_position
            just_left = following
            following = None
            trip.note(step_number, "leave", "기준선을 다시 만나서 떠남")
            continue

        walked_since_hit += M.distance(position, next_position)
        position = next_position
        trip.path.append(position)
        # 한 바퀴 돌아 처음 부딪힌 곳으로 돌아왔는데 아직 못 떠났으면 갇힌 것.
        # 겹친 원 여러 개를 이어 돌면 중심각 합이 한 바퀴를 넘을 수 있어서, 각도 대신 위치로 봄
        if walked_since_hit > 4 * settings.step and M.distance(position, hit_position) < settings.step:
            trip.result = "stuck"
            trip.note(step_number, "stuck", "장애물을 한 바퀴 돌아도 떠날 곳이 없음")
            return trip

    trip.result = "timeout"
    trip.note(settings.max_steps, "timeout", "걸음 수 상한을 넘김")
    return trip


def _can_leave(position, goal, circle, hit_distance, settings):
    """지금 원을 떠나도 되는가.

    수선의 발이 넘어가서 목표 쪽이 트였으면 떠날 수 있음.
    기준선 규칙을 쓰면, 거기에 더해 처음 부딪힌 곳보다 목표에 가까워야 함.
    이게 없으면 원 두 개 사이를 왔다 갔다 하며 끝나지 않을 수 있음.
    """
    if not M.leaving_is_clear(position, goal, circle):
        return False
    if settings.use_baseline_rule:
        return M.distance(position, goal) < hit_distance - M.TINY
    return True


def _first_crossing(circle, other, position, turn):
    """circle 을 turn 방향으로 돌 때 other 와 처음 만나는 교점."""
    points = M.circle_intersections(circle, other)
    if not points:
        return None
    return min(points, key=lambda p: M.central_angle(circle, position, p, turn))
