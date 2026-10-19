"""
robot.py -- 실제 차에서 도는 주행 프로그램 (오렌지파이 쪽)

시뮬레이터(hybrid_sim.py)와 같은 알고리즘을 쓴다. 바뀌는 것은 세상을 보는 창구뿐이다.

  시뮬레이터                        실제 차
  ----------------------------------------------------------------
  라이다/ToF 흉내 -> 지도          아두이노가 보낸 거리 -> 같은 지도
  Vehicle 의 오도메트리            자이로 + 명령 속도로 직접 추정
  Vehicle.step 으로 차를 움직임    아두이노에 속도와 조향을 보냄
  시간이 멈춘 채 계획              계획하는 동안에도 차는 움직인다

엔코더가 없다
  거리는 '보낸 속도 x 시간' 으로 짐작한다. 15 초 직진에 1 m 가까이 틀릴 수 있다.
  그래서 지도는 오래 쌓을수록 어긋난다. 짧은 주행을 전제로 한다.

세 개의 루프를 한 반복문에서 돌린다
  들어온 상태마다  : 자세 추정과 지도 갱신
  50 Hz            : 경로 추종, 속도와 조향 전송
  10 Hz            : 거리장 갱신과 필요할 때만 다시 계획

안전
  - 아두이노가 스스로 멈추는 감시 장치가 1차 방어선이다. 파이가 죽어도 차는 선다.
  - 여기서도 예측 정지를 한다. 지금 명령대로 제동 거리만큼 가면 차체가 닿는지 본다.
  - 사람이 조종기로 가져가면(manual) 계획을 멈추고 자세 추정만 계속한다.
"""

import math
import time

import numpy as np

from . import geometry as G
from .grid import OccupancyGrid
from .hybrid import HybridAStar, HybridParams, distance_field
from .hybrid_sim import HybridConfig, SegmentTracker, integrate_rays
from .link import Status
from .sensors import Measurement, RangeSensorArray
from .vehicle import VehicleParams


class RobotConfig:
    def __init__(self, bounds=(-12.0, -12.0, 12.0, 12.0), grid_res=0.12,
                 control_hz=50.0, plan_hz=10.0, goal_tol=0.45,
                 max_speed=1.0, rev_speed=0.25, max_time=120.0, has_rear_sensor=True):
        self.bounds = bounds          # 지도 범위. 출발점이 (0, 0) 이다
        self.grid_res = grid_res
        self.control_hz = control_hz
        self.plan_hz = plan_hz
        self.goal_tol = goal_tol
        self.max_speed = max_speed    # 실물에서는 시뮬레이터보다 낮춰 시작한다
        self.rev_speed = rev_speed
        self.max_time = max_time
        # 뒤 센서가 없으면 뒤를 모르므로 후진 탈출을 하지 않음. 갇히면 멈추고 사람이 꺼냄
        self.has_rear_sensor = has_rear_sensor


class Robot:
    def __init__(self, link, goal, veh=None, cfg=None, hp=None, hc=None, ctrl=None):
        from .controller import ControllerParams
        self.link = link
        self.goal = np.asarray(goal, float)
        self.V = veh or VehicleParams()
        self.C = cfg or RobotConfig()
        self.HP = hp or HybridParams(allow_reverse=True)
        self.HC = hc or HybridConfig()
        self.planner = HybridAStar(self.V, self.HP)
        hp_fwd = HybridParams(**{**self.HP.__dict__, "allow_reverse": False})
        self.planner_fwd = HybridAStar(self.V, hp_fwd)
        self.follower = SegmentTracker(self.V, ctrl or ControllerParams())
        self.follower.rev_speed = self.C.rev_speed

        self.grid = OccupancyGrid(self.C.bounds, self.C.grid_res)
        self.D = None
        self.pose = np.zeros(3)        # 출발점이 원점, 처음 방향이 x 축
        self.speed = 0.0
        self.path_poses = None
        self.path_cum = None
        self.arrived = False
        self.manual = False

        # 센서 장착 위치와 원뿔. sensors.make_sensor 의 tof_sweep_rear 과 같은 값
        fx = self.V.front_sensor_x_m
        rx = self.V.rear_sensor_x_m
        cone = math.radians(27)
        self.front = RangeSensorArray("front", [[fx, 0.0]], [0.0], 3.0,
                                      cone=cone, subrays=9, map_arc=True)
        self.rear = RangeSensorArray("rear", [[rx, 0.0]], [math.pi], 3.0,
                                     cone=cone, subrays=9, map_arc=True)

        self._last_status_ms = None
        self._plan_t = -1e9
        self._force_replan = False
        self._fwd_fail_pos = None
        self._fwd_fail_t = -1e9
        self.t = 0.0
        self.stats = {"plans": 0, "replans": 0, "stops": 0, "bad_frames": 0, "escapes": 0}

        # 출발 전 한 번 훑기: 앞 센서가 좌우를 다 본 뒤에 움직인다
        self._sweep_min = math.inf
        self._sweep_max = -math.inf
        self.ready = False
        # 뒤 센서가 방금 잰 거리 (탈출할 때 지도 대신 이것을 믿는다)
        self.rear_m = math.inf
        self._rear_t = -1e9
        # 계속 멈추면 조금 후진해서 빠져나온다
        self._stop_times = []
        self._escape = None
        self.stuck = False             # 뒤 센서 없이 갇혀서 스스로 멈춘 상태
        self._still_since = None
        self.last_status = None
        self._log = None

    # ------------------------------------------------------------ 자세와 지도
    def on_status(self, s: Status):
        """아두이노가 보낸 한 줄을 반영한다."""
        if self._last_status_ms is None:
            self._last_status_ms = s.t_ms
            return
        dt = (s.t_ms - self._last_status_ms) / 1000.0
        self._last_status_ms = s.t_ms
        if not (0.0 < dt < 0.5):           # 시계가 튀면 버린다
            return
        self.manual = s.manual
        self.speed = s.speed_mps
        self.last_status = s

        # 자세: 방향은 자이로, 거리는 아두이노가 실제로 준 속도
        dth = math.radians(s.gyro_dps) * dt
        ds = s.speed_mps * dt
        x, y, th = self.pose
        self.pose = np.array([x + ds * math.cos(th + 0.5 * dth),
                              y + ds * math.sin(th + 0.5 * dth),
                              G.wrap_pi(th + dth)])

        # 지도: 앞 센서는 지금 보고 있는 방향, 뒤 센서는 고정
        self._add_measurement(self.front, s.sweep_rad, s.front_m)
        if self.C.has_rear_sensor:
            self._add_measurement(self.rear, math.pi, s.rear_m)
            self.rear_m, self._rear_t = s.rear_m, self.t

        # 출발 준비: 앞 센서가 좌우 끝까지 한 번씩 봤는가
        #
        # 켜자마자 달리면 아직 못 본 기둥 쪽으로 경로를 잡는다. 서보가 180 도를 한 번
        # 훑는 데 1 초 남짓 걸리는데, 무작위 지도 17 번과 35 번에서 둘 다 출발 1 초 만에
        # 기둥에 붙었다. 하나는 갇혔고 하나는 닿았다.
        if not self.ready:
            a = math.degrees(s.sweep_rad)
            self._sweep_min = min(self._sweep_min, a)
            self._sweep_max = max(self._sweep_max, a)
            if self._sweep_min <= -70.0 and self._sweep_max >= 70.0:
                self.ready = True
                self._force_replan = True

    def _add_measurement(self, sensor, angle, rng_m):
        m = Measurement(np.zeros(1, int), sensor.mounts, np.array([angle]),
                        np.array([rng_m]))
        (fo, fe), (oo, oe) = sensor.rays_for_map(m, self.grid.res)
        if len(fe):
            integrate_rays(self.grid, G.compose(self.pose, fo),
                           G.compose(self.pose, fe), mark_free=True, occupied=False)
        if len(oe):
            integrate_rays(self.grid, G.compose(self.pose, oo),
                           G.compose(self.pose, oe), mark_free=False, occupied=True)

    # ------------------------------------------------------------ 계획
    def _search(self, start, goal, reason):
        r = self.planner_fwd.plan(start, goal, self.grid, self.D)
        if r is not None or reason not in ("none", "blocked"):
            return r
        here = np.array(start[:2])
        near = (self._fwd_fail_pos is not None
                and self.t - self._fwd_fail_t < self.HC.stuck_memory
                and float(np.hypot(*(here - self._fwd_fail_pos))) < self.HC.stuck_radius)
        if not near:
            self._fwd_fail_pos, self._fwd_fail_t = here, self.t
        return self.planner.plan(start, goal, self.grid, self.D)

    def plan_cycle(self):
        """10 Hz. 거리장을 새로 만들고 필요하면 다시 계획한다."""
        self.D = distance_field(self.grid.occupied_mask(), self.grid.res,
                                self.HC.field_cap)
        self.planner.prepare(self.grid, self.D)
        pos, th = self.pose[:2], float(self.pose[2])

        reason = None
        if self.path_poses is None:
            reason = "none"
        elif self._force_replan:
            reason = "blocked"
        else:
            i0 = self.follower.global_index()
            s = self.follower.s
            j = int(np.searchsorted(s, s[i0] + self.HC.check_ahead))
            ahead = self.path_poses[i0:max(j, i0 + 1) + 1]
            m = self.planner.clearance_poses(ahead[:, 0], ahead[:, 1], ahead[:, 2])
            if len(m) and float(m.min()) < self.planner.r_hard - self.HC.blocked_slack:
                reason = "blocked"
            elif self.follower.err > self.HC.deviate_dist:
                reason = "deviate"
            elif self.t - self._plan_t > self.HC.replan_period:
                reason = "refresh"
        if reason is None:
            return

        self._force_replan = False
        self._plan_t = self.t
        self.stats["plans"] += 1
        # 계획하는 동안에도 차는 움직인다. 끝났을 때의 자리에서 시작하도록 조금 앞을 잡는다.
        lead = self.speed * 0.15
        start = (pos[0] + lead * math.cos(th), pos[1] + lead * math.sin(th), th)
        res = self._search(start, self.goal, reason)
        if res is None:
            if reason in ("none", "blocked"):
                self.path_poses = self.path_cum = None
                self.follower.set_path(None, None)
            return
        if reason == "refresh" and self.path_poses is not None:
            i0 = self.follower.global_index()
            remaining = float(self.path_cum[-1] - self.path_cum[i0])
            if res.cost >= remaining * self.HC.adopt_ratio:
                return
        self.path_poses, self.path_cum = res.poses, res.cum_cost
        self.follower.set_path(res.poses, res.dirs)
        self.stats["replans"] += 1

    # ------------------------------------------------------------ 추종
    def control_cycle(self):
        """50 Hz. 속도와 조향을 정해 아두이노로 보낸다."""
        if self.arrived or self.manual or not self.ready or self.D is None:
            self._send(0.0, 0.0)
            return
        if self._escape is not None:
            self._escape_step()
            return
        if self.path_poses is None:
            self._send(0.0, 0.0)
            self._note_stop()
            return
        f = self.follower
        f.update_progress(self.pose[:2])
        i0 = f.global_index()
        j = min(len(self.path_poses) - 1, i0 + 15)
        ahead = self.path_poses[i0:j + 1]
        clear = float(self.planner.clearance_poses(ahead[:, 0], ahead[:, 1],
                                                   ahead[:, 2]).min()) \
            - self.planner.fp.radius
        v, s, _ = f.compute(self.pose, self.speed, self.goal, clear)
        v = float(np.clip(v, -self.C.rev_speed, self.C.max_speed))

        # 예측 정지: 이 명령대로 제동 거리만큼 가면 차체가 닿는가
        sgn = np.sign(v) if abs(v) > 1e-3 else np.sign(self.speed)
        if sgn != 0:
            dist = max(abs(self.speed), abs(v)) ** 2 / (2.0 * self.V.max_decel) + 0.12
            kap = math.tan(s) / self.V.wheelbase
            sig = sgn * np.linspace(0.04, dist, 6)
            x, y, th = self.pose
            dth = kap * sig
            if abs(kap) < 1e-6:
                dx, dy = sig, np.zeros_like(sig)
            else:
                dx, dy = np.sin(dth) / kap, (1.0 - np.cos(dth)) / kap
            X = x + math.cos(th) * dx - math.sin(th) * dy
            Y = y + math.sin(th) * dx + math.cos(th) * dy
            if float(self.planner.clearance_poses(X, Y, th + dth).min()) < self._stop_limit():
                v = 0.0
                self.stats["stops"] += 1
                self._force_replan = True
                self._note_stop()
        self._send(v, s)

    # ------------------------------------------------------------ 멈춤과 탈출
    def _stop_limit(self):
        """예측 정지 기준 (차체 원 중심에서 지도상 장애물 칸까지).

        지도는 칸 중심끼리 거리를 재서, 실제 표면은 반 칸 대각선만큼 더 가까울 수 있다.
        계획기(r_hard)는 이걸 더하는데 예측 정지만 빼먹고 있었다. 그래서 지도상 여유
        0.9 cm 를 믿고 멈추지 않았다가 실제로 9 mm 닿았다 (무작위 지도 35 번).
        """
        return self.planner.fp.radius + 0.5 * math.sqrt(2.0) * self.grid.res \
            + self.HC.touch_margin

    def _note_stop(self):
        """멈춘 시각을 기록하고, 계속 멈춰 있으면 탈출을 시작한다."""
        self._stop_times.append(self.t)
        self._stop_times = [t for t in self._stop_times if self.t - t < 1.0]
        if len(self._stop_times) >= 25 and abs(self.speed) < 0.05 and self.C.has_rear_sensor:
            self._start_escape()

    # 뒤 센서가 없을 때 이만큼 계속 서 있으면 갇힌 것으로 봄 (초).
    # 처음엔 '1 초에 25 번 멈춤' 으로 판정했는데, 출발 직후 첫 경로를 찾는 동안 잠깐 서는 것까지
    # 갇힘으로 봐서 무작위 지도 17 번이 2.4 초 만에 끝났음. 뒤 센서가 있으면 후진 없이 도착하는 지도였음
    STUCK_SECONDS = 5.0

    def _check_stuck(self):
        if self.C.has_rear_sensor or not self.ready or self.manual or self.arrived:
            self._still_since = None
            return
        if abs(self.speed) >= 0.05:
            self._still_since = None
            return
        if self._still_since is None:
            self._still_since = self.t
        elif self.t - self._still_since > self.STUCK_SECONDS and not self.stuck:
            self.stuck = True
            self.stats["stuck"] = self.stats.get("stuck", 0) + 1

    def _start_escape(self):
        """
        기둥에 붙어 갇혔을 때 곧게 천천히 조금 후진한다.

        지도가 아니라 뒤 센서가 방금 잰 거리를 믿는다. 지도는 서보 각도 오차 같은 것
        때문에 틀릴 수 있지만, 방금 잰 거리는 그렇지 않다. 뒤가 비어 있지 않으면 하지 않는다.
        무작위 지도 17 번에서 멈춤과 재계획을 12 초에 549 번 반복하던 것을 끊는다.
        """
        fresh = (self.t - self._rear_t) < 0.3
        if not (fresh and self.rear_m > self.ESCAPE_REAR_NEED):
            return
        self._escape = {"start": self.pose[:2].copy(), "until": self.t + 3.0}
        self._stop_times = []
        self.stats["escapes"] += 1

    ESCAPE_BACK = 0.25        # 후진할 거리 (m)
    ESCAPE_REAR_NEED = 0.45   # 뒤가 이만큼은 비어 있어야 후진한다 (m)
    ESCAPE_SPEED = 0.15

    def _escape_step(self):
        e = self._escape
        moved = float(np.hypot(*(self.pose[:2] - e["start"])))
        fresh = (self.t - self._rear_t) < 0.3
        blocked = (not fresh) or self.rear_m < self.ESCAPE_REAR_NEED - self.ESCAPE_BACK
        if moved >= self.ESCAPE_BACK or blocked or self.t > e["until"]:
            self._escape = None
            self._force_replan = True
            self._send(0.0, 0.0)
            return
        self._send(-self.ESCAPE_SPEED, 0.0)

    # ------------------------------------------------------------ 실행
    def step(self, dt):
        """시간을 dt 만큼 진행한다. 실제로는 run() 이 실시간으로 부른다."""
        for s in self.link.poll():
            self.on_status(s)
        self.t += dt
        self._check_stuck()
        if self.stuck:
            self.link.send(0.0, 0.0)
            return False
        if np.hypot(*(self.pose[:2] - self.goal)) < self.C.goal_tol:
            self.arrived = True
            self.link.send(0.0, 0.0)
            return False
        if self.t > self.C.max_time:
            self.link.send(0.0, 0.0)
            return False
        return True

    @property
    def end_reason(self):
        if self.arrived:
            return "도착"
        if self.stuck:
            return "갇힘 (뒤 센서가 없어 후진하지 않고 멈춤)"
        if self.t > self.C.max_time:
            return "시간 초과"
        return "중단"

    # ------------------------------------------------------------ 주행 기록
    LOG_COLUMNS = ("t_s", "x_m", "y_m", "heading_deg", "speed_mps", "cmd_speed_mps",
                   "cmd_steer_deg", "sweep_deg", "front_m", "rear_m", "mode", "plans", "stops")

    def open_log(self, path):
        """추종 주기(50 Hz)마다 한 줄씩 CSV 로 남김. 위치는 차가 믿는 위치 (출발점이 0, 처음 방향이 x 축).

        실차 실험에서 바닥 궤적(진짜 위치)과 이 기록(믿은 위치)을 겹치면
        기계 오차를 위치 추정 오차와 조향 제어 오차로 한 번 더 나눌 수 있음.
        """
        self._log = open(path, "w", encoding="utf-8", newline="")
        self._log.write(",".join(self.LOG_COLUMNS) + "\n")

    def close_log(self):
        if self._log is not None:
            self._log.close()
            self._log = None

    def _mode(self):
        if self.manual:
            return "manual"
        if self.stuck:
            return "stuck"
        if self._escape is not None:
            return "escape"
        if not self.ready:
            return "waiting"
        if self.path_poses is None:
            return "no_path"
        return "drive"

    def _send(self, speed, steer):
        """아두이노로 명령을 보내고, 기록을 켰으면 그 순간을 한 줄 남김."""
        self.link.send(speed, steer)
        if self._log is None:
            return
        s = self.last_status
        sweep = math.degrees(s.sweep_rad) if s else float("nan")
        front = s.front_m if s else float("inf")
        rear = s.rear_m if s else float("inf")
        x, y, th = self.pose
        self._log.write(f"{self.t:.3f},{x:.4f},{y:.4f},{math.degrees(th):.2f},{self.speed:.3f},"
                        f"{speed:.3f},{math.degrees(steer):.2f},{sweep:.1f},{front:.3f},{rear:.3f},"
                        f"{self._mode()},{self.stats['plans']},{self.stats['stops']}\n")

    def run(self, on_tick=None):
        """실시간 주행. 50 Hz 로 돌며 10 Hz 마다 계획한다."""
        dt = 1.0 / self.C.control_hz
        plan_every = max(1, int(round(self.C.control_hz / self.C.plan_hz)))
        k = 0
        t0 = time.monotonic()
        try:
            while True:
                if not self.step(dt):
                    break
                if k % plan_every == 0:
                    self.plan_cycle()
                self.control_cycle()
                if on_tick:
                    on_tick(self, k)
                k += 1
                nxt = t0 + (k + 1) * dt
                slack = nxt - time.monotonic()
                if slack > 0:
                    time.sleep(slack)
        finally:
            self.link.send(0.0, 0.0)
        return self.arrived
