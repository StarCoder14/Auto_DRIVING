"""
simulator.py -- 전체를 묶는 루프 (문서 7장)

서로 다른 속도로 도는 세 개의 루프가 있다.

  계획 루프 (10 Hz) : 본다 -> 이해한다 -> 기억한다 -> 부풀린다
                      -> 노선도를 만든다 -> 고른다 -> 고집을 부린다
  추종 루프 (50 Hz) : 경로 위 앞쪽 지점을 향해 핸들과 속도를 조절한다
  안전 루프(100 Hz) : 바로 앞에 뭔가 있으면 무조건 멈춘다.
                      계획이 무엇이든 상관없이 이 판단이 최우선이다.

알고리즘은 오도메트리 좌표계(차가 믿는 좌표계)에서만 산다.
정답(World)은 라이다 시뮬레이션과 충돌 판정에만 쓰인다.
"""

import time
import numpy as np

from . import geometry as G
from . import obstacles as OB
from . import perception as PC
from . import planner as PL
from . import grid as GR
from .controller import PurePursuit, ControllerParams, path_curvature_ahead
from .vehicle import Vehicle, VehicleParams
from .world import Lidar


class SimConfig:
    def __init__(self,
                 dt=0.01,                 # 안전 루프 주기 (100 Hz)
                 control_every=2,         # 50 Hz
                 plan_every=10,           # 10 Hz
                 margin_sensor=0.10,      # 센서/피팅 오차 여유 (m)
                 margin_control=0.14,     # 추종 오차 여유 (m)
                 turn_slack=1.6,          # rho >= 이 값 x 최소 회전 반경
                 recover_time=0.5,        # 탈출을 시작하면 최소 이만큼은 유지 (s)
                 recover_time_max=1.6,    # 같은 자리에서 반복되면 여기까지 늘린다
                 deadlock_window=7.0,     # 이 시간 동안 진전 없으면 갇힘 (s)
                 deadlock_progress=0.35,  # '진전'의 기준 (m)
                 grid_res=0.12,
                 hysteresis=0.90,         # 새 경로가 이보다 싸야 갈아탄다
                 swerve_angle=np.radians(35.0),  # 이만큼 방향이 벌어지면 '꺾음'
                 swerve_hysteresis=0.65,  # 꺾는 교체는 이만큼 더 싸야 한다
                 max_verts=4,             # 부풀린 도형의 최대 꼭짓점 수
                 max_time=180.0):
        self.dt = dt
        self.control_every = control_every
        self.plan_every = plan_every
        self.margin_sensor = margin_sensor
        self.margin_control = margin_control
        self.turn_slack = turn_slack
        self.recover_time = recover_time
        self.recover_time_max = recover_time_max
        self.deadlock_window = deadlock_window
        self.deadlock_progress = deadlock_progress
        self.grid_res = grid_res
        self.hysteresis = hysteresis
        self.swerve_angle = swerve_angle
        self.swerve_hysteresis = swerve_hysteresis
        self.max_verts = max_verts
        self.max_time = max_time


class Simulator:
    def __init__(self, world, start_pose, goal, cfg=None,
                 veh=None, lidar=None, plan_params=None, ctrl=None, seed=0):
        self.world = world
        self.cfg = cfg or SimConfig()
        self.rng = np.random.default_rng(seed)

        self.VP = veh or VehicleParams()
        self.vehicle = Vehicle(self.VP, start_pose, rng=self.rng)
        self.lidar = lidar or Lidar(rng=self.rng)
        self.PP = plan_params or PL.PlannerParams()
        self.ctrl = PurePursuit(ctrl or ControllerParams(), self.VP)

        # ---- 부풀림 반지름 rho (문서 2.1)
        #   안전 여유 = 차체 외접원 + 센서 오차 + 추종 오차
        #   하한      = 최소 회전 반경 x turn_slack
        #
        # 하한을 최소 회전 반경과 '같게' 두면 안 된다. 핸들을 끝까지 꺾어야
        # 겨우 도는 호가 되어 추종 오차가 그대로 침범으로 이어진다.
        # 여유 있게 돌 수 있는 크기로 잡아야 한다.
        self.margin = (self.VP.body_radius
                       + self.cfg.margin_sensor + self.cfg.margin_control)
        self.rho = max(self.margin, self.cfg.turn_slack * self.VP.min_turn_radius)

        # 계획용 반경(rho)과 안전용 반경(rho_hard)은 다르다.
        #   rho      : 경로를 그릴 때 쓰는 넉넉한 여유. 여기 들어가는 건 '정상'이다.
        #              코너를 조금 잘라먹으면 늘 일어난다.
        #   rho_hard : 실제로 위험한 선. 여기를 넘으면 그때 탈출한다.
        # 이 둘을 같게 두면 탈출과 주행이 무한히 토글한다.
        self.rho_hard = self.VP.body_radius + self.cfg.margin_sensor

        self.goal = np.asarray(goal, float)      # 오도메트리 좌표계 기준
        self.tracker = PC.Tracker()
        self.grid = GR.OccupancyGrid(world.bounds, self.cfg.grid_res)

        # 상태
        self.t = 0.0
        self.k = 0
        self.mode = "drive"
        self.path = None
        self.path_cost = np.inf
        self._path_len0 = 0.0
        self.groups = []
        self.graph = None
        self.cmd = (0.0, 0.0)
        self.escape_dir = None
        self._recover_until = -1.0
        self._recover_at = None
        self._recover_streak = 0
        self.trail = [self.vehicle.pose[:2].copy()]
        self.scan_pts_world = np.zeros((0, 2))
        self.front_clear = 10.0
        self.rear_clear = 10.0
        self.plan_ms = 0.0
        self.collided = False
        self.arrived = False
        self.log = []
        self._best_dgoal = np.inf
        self._best_t = 0.0
        self._grid_path_until = -1.0
        self._replans = 0
        self._heading_released = 0
        self._swerves = 0
        self._events = []

    # ================================================================ 한 스텝
    def step(self):
        cfg = self.cfg
        if self.arrived or self.collided or self.t > cfg.max_time:
            return False

        # ---------------- 계획 루프 (10 Hz)
        if self.k % cfg.plan_every == 0:
            self._sense_and_plan()

        # ---------------- 추종 루프 (50 Hz)
        if self.k % cfg.control_every == 0:
            self._control()

        # ---------------- 안전 루프 (100 Hz) : 최상위 권한
        # 단, 후진 중에는 전방 정지 조건을 걸지 않는다.
        # 걸면 장애물에 코를 박은 채 영원히 못 빠져나온다.
        v_cmd, s_cmd = self.cmd
        if v_cmd > 0.0 and self.front_clear < self.VP.body_radius + 0.05:
            v_cmd = 0.0
            self.mode = "estop"

        self.vehicle.step(v_cmd, s_cmd, cfg.dt)
        self.t += cfg.dt
        self.k += 1

        p = self.vehicle.pose[:2]
        if np.hypot(*(p - self.trail[-1])) > 0.05:
            self.trail.append(p.copy())

        # 실제 충돌 여부 (정답 기준. 알고리즘은 이 정보를 못 본다)
        if self.world.min_distance(p) < self.VP.body_radius * 0.55:
            self.collided = True
            self._events.append((self.t, "COLLISION"))

        # 도착 판정은 차가 믿는 좌표계에서
        if np.hypot(*(self.vehicle.odom[:2] - self.goal)) < self.ctrl.P.goal_tol:
            self.arrived = True
            self._events.append((self.t, "ARRIVED"))
        return True

    # ================================================================ 인식+계획
    def _sense_and_plan(self):
        t0 = time.perf_counter()
        cfg = self.cfg

        # 1) 본다 --------------------------------------------------
        angles, ranges = self.lidar.scan(self.world, self.vehicle.pose)
        pts_body, mask = Lidar.to_points(angles, ranges)
        odom = self.vehicle.odom
        pts_odom = G.compose(odom, pts_body) if len(pts_body) else np.zeros((0, 2))

        # 그림용: 실제 세계 좌표의 스캔점
        self.scan_pts_world = G.compose(self.vehicle.pose, pts_body) \
            if len(pts_body) else np.zeros((0, 2))

        # 전방 여유 (속도 제어와 비상 정지에 쓴다)
        front = np.abs(G.wrap_pi(angles)) < np.radians(35)
        fr = ranges[front]
        self.front_clear = float(np.min(fr)) if np.isfinite(fr).any() else 10.0
        rear = np.abs(G.wrap_pi(angles - np.pi)) < np.radians(35)
        rr = ranges[rear]
        self.rear_clear = float(np.min(rr)) if np.isfinite(rr).any() else 10.0

        # 2) 이해한다 ----------------------------------------------
        r_valid = ranges[mask]
        shapes = []
        for idx in PC.cluster(pts_body, r_valid, dphi=self.lidar.dphi):
            for sh in PC.shape_from_cluster(pts_body[idx], np.zeros(2)):
                if len(sh):
                    shapes.append(G.compose(odom, sh))

        # 3) 기억한다 ----------------------------------------------
        self.tracker.update(shapes, odom[:2], self.lidar.max_range)

        # 격자 지도도 함께 채운다 (비상용)
        if len(pts_odom):
            self.grid.integrate(odom[:2], pts_odom)

        # 4) 부풀린다 ----------------------------------------------
        # max_verts 는 모양의 충실도와 '계획기의 시야'를 맞바꾸는 손잡이다.
        #
        # 4 로 두면 원기둥도 사각형으로 깎여 화면이 각져 보인다. 올리면 둥글어진다.
        # 그런데 계획기는 가까운 원을 max_circles(24) 개까지만 쓴다. 꼭짓점이
        # 늘면 그 24 개가 덮는 장애물 수가 줄어 계획기가 근시가 된다.
        #   4  -> 원 24 개로 장애물 약 6 개를 본다
        #   10 -> 장애물 2~3 개밖에 못 본다
        # trap 처럼 골목 전체를 봐야 빠져나오는 환경은 이때 길을 못 찾고
        # 탐색 모드로 기어다닌다. 실제로 6 이상에서 trap 이 제한 시간을 넘겼다.
        #
        # 그래서 기본은 4 로 둔다. 원기둥만 있는 pillars 를 예쁘게 보여줘야 하면
        # run.py --verts 10 처럼 실행할 때만 올린다. 상시로 올리려면
        # PlannerParams.max_circles 도 같은 비율로 키워야 한다.
        self.groups = OB.inflate_and_merge(self.tracker.shapes(), self.rho,
                                           max_verts=self.cfg.max_verts,
                                           is_free=self.grid.is_free_many)

        pos = odom[:2]

        # ---- 예외 1: 안전선을 넘었다 (문서 8장)
        # g.distance = (껍질까지 거리) - rho 이므로,
        # 껍질까지 거리 < rho_hard  <=>  g.distance < rho_hard - rho
        hard = self.rho_hard - self.rho
        exit_hard = hard + 0.20                      # 나갈 때는 더 여유 있게
        escaping = self.mode == "escape"
        thr = exit_hard if escaping else hard
        inside = [g for g in self.groups if g.distance(pos) < thr]

        # 비상 정지는 '멈추라'고만 할 뿐, 빠져나오게 해주지는 않는다.
        # 코를 박은 채로 정지하면 그대로 영구 동결된다. 그래서 코가 막힌 채
        # 멈춰 있으면 탈출(후진)로 넘긴다.
        nose_blocked = (self.front_clear < self.VP.body_radius + 0.12
                        and abs(self.vehicle.v) < 0.10)

        # 탈출은 '한 번 시작하면 최소 시간은 유지'한다.
        # 0.1 m 물러나자마자 주행으로 돌아가면 같은 자리로 다시 밀고 들어가서
        # 탈출<->주행이 무한히 반복된다. 실제로 그렇게 된다.
        if (inside or nose_blocked) and not escaping:
            # 같은 자리에서 자꾸 다시 걸리면 후진 시간을 늘린다.
            # 짧게만 물러나면 곧장 같은 곳으로 밀고 들어가 한계 순환에 빠진다.
            if self._recover_at is not None and \
                    np.hypot(*(pos - self._recover_at)) < 1.0 and \
                    self.t - self._recover_until < 4.0:
                self._recover_streak += 1
            else:
                self._recover_streak = 0
            self._recover_at = pos.copy()
            hold = min(self.cfg.recover_time * (1 + self._recover_streak),
                       self.cfg.recover_time_max)
            self._recover_until = self.t + hold
        if inside or nose_blocked or (escaping and self.t < self._recover_until):
            self.mode = "escape"
            if inside:
                self.escape_dir = self._escape_dir(pos, inside)
            elif nose_blocked or self.escape_dir is None:
                # 막힌 게 앞이면 나가야 할 방향은 뒤다
                th = self.vehicle.odom[2]
                self.escape_dir = -np.array([np.cos(th), np.sin(th)])
            self.path = None
            self.plan_ms = (time.perf_counter() - t0) * 1e3
            return
        self.escape_dir = None

        # 부풀린 영역 안(그러나 안전선 밖)이면, 그 그룹은 출발점 한정으로 면제한다
        soft = tuple(g.gid for g in self.groups if g.contains(pos, tol=1e-4))

        # ---- 갇힘 감지 (문서 5장)
        d_goal = float(np.hypot(*(pos - self.goal)))
        if d_goal < self._best_dgoal - cfg.deadlock_progress:
            self._best_dgoal = d_goal
            self._best_t = self.t
        deadlocked = (self.t - self._best_t) > cfg.deadlock_window

        if deadlocked and self.t > self._grid_path_until:
            gp = self.grid.astar(pos, self.goal)
            if gp is not None and len(gp) >= 2:
                self.mode = "grid"
                self.path = _densify(gp)
                self.path_cost = np.inf
                self._path_len0 = _remaining_length(self.path, pos)
                self._grid_path_until = self.t + 6.0
                self._best_t = self.t          # 유예를 준다
                self._events.append((self.t, "DEADLOCK -> grid fallback"))
                self.plan_ms = (time.perf_counter() - t0) * 1e3
                return

        if self.t < self._grid_path_until and self.path is not None:
            self.plan_ms = (time.perf_counter() - t0) * 1e3
            return                              # 격자 경로를 따라가는 중

        # 5~6) 노선도를 만들고 고른다 -------------------------------
        goal = self.goal
        gexc = [g for g in self.groups if g.contains(goal, tol=1e-3)]
        if gexc:                                # 예외 2: 목적지가 장애물 안
            goal = self._nearest_reachable(goal)

        # 차의 진행 방향을 알려준다. 뒤로 꺾여 나가는 첫 구간에 벌점이 붙는다.
        # 계획기가 출발점을 점으로만 보면 '제자리에서 돌아 뒤로 가는' 경로를
        # 짧다는 이유로 고른다. 차는 그렇게 못 움직인다.
        # 금지가 아니라 벌점이라, 막다른 골목에서는 여전히 돌아 나올 수 있다.
        new_path, new_cost, graph = PL.plan(pos, goal, self.groups,
                                            self.rho, self.PP, self.grid, soft,
                                            heading=float(odom[2]))


        if new_path is None:                    # 예외 3: 어떤 경로도 없다
            fr_goal = self._frontier_goal(pos)
            if fr_goal is not None:
                new_path, new_cost, graph = PL.plan(pos, fr_goal, self.groups,
                                                    self.rho, self.PP, self.grid,
                                                    soft)
                if new_path is not None:
                    self.mode = "explore"

        self.graph = graph

        # 7) 고집을 부린다 (문서 3.5) -------------------------------
        #
        # 두 가지를 지킨다.
        #
        # (가) 같은 기준으로 비교한다.
        #     path_cost 는 그 경로를 채택하던 '그때 위치'에서 잰 값이고,
        #     new_cost 는 '지금 위치'에서 잰 값이다. 차가 전진할수록 앞의 값만
        #     낡아서 커지므로, 그대로 비교하면 새 경로가 늘 싸 보인다. 그래서
        #     고집 장치가 사실상 매 주기 뚫리고 경로가 계속 갈아치워졌다.
        #     남은 길이의 비율로 깎아 같은 시점의 값으로 맞춘다.
        #
        # (나) 가던 방향을 함부로 꺾지 않는다.
        #     장애물을 왼쪽으로 돌기로 해놓고 다음 주기에 오른쪽으로 트는 뒤집힘이
        #     제일 위험하다. 호는 부풀린 경계에 붙어 있어 여유가 없는데, 그 위에서
        #     방향을 바꾸면 추종 오차가 그대로 침범이 된다. 방향이 크게 벌어지는
        #     교체는 훨씬 큰 이득을 요구한다.
        #
        # 단, 가던 길이 막혔으면 위 둘을 모두 무시하고 즉시 버린다. 안전이 우선이다.
        cur_blocked = self._path_blocked(self.path)
        if new_path is not None:
            if self.path is None or cur_blocked:
                take = True
            else:
                rem = _remaining_length(self.path, pos)
                if rem < 1e-6 or not np.isfinite(self.path_cost):
                    take = True
                else:
                    frac = min(max(rem / max(self._path_len0, 1e-9), 0.0), 1.0)
                    take = new_cost < self.path_cost * frac * cfg.hysteresis
            swerve = False

            if take:
                self.path = new_path
                self.path_cost = new_cost
                self._path_len0 = _remaining_length(new_path, pos)
                self._replans += 1
                if swerve:
                    self._swerves += 1
                if self.mode not in ("explore",):
                    self.mode = "drive"
        elif cur_blocked:
            self.path = None
            self.mode = "no_path"

        self.plan_ms = (time.perf_counter() - t0) * 1e3
        self.log.append((self.t, self.plan_ms, len(self.groups),
                         0 if self.graph is None else len(self.graph.pos)))

    # ================================================================ 제어
    def _control(self):
        if self.mode == "escape" and self.escape_dir is not None:
            self.cmd = self._escape_cmd()
            return
        if self.path is None:
            self.cmd = (0.0, 0.0)
            return
        kappa = path_curvature_ahead(self.path, self.vehicle.odom[:2])
        v, s, info = self.ctrl.compute(
            self.path, self.vehicle.odom, self.vehicle.v,
            self.front_clear - self.VP.body_radius, self.goal, kappa)
        self.cmd = (v, s)

    # ================================================================ 보조
    def _escape_dir(self, pos, inside):
        """장애물들에서 벗어나는 방향 (척력의 합)."""
        push = np.zeros(2)
        for g in inside:
            d = pos - g.center
            n = np.hypot(*d)
            push += (d / n if n > 1e-6 else np.array([1.0, 0.0])) \
                * max(self.rho_hard - (g.distance(pos) + self.rho), 0.05)
        return G.unit(push)

    def _escape_cmd(self):
        """
        부풀린 도형 안에 들어와 있을 때의 전용 제어.

        실제 충돌이 아니라 안전 영역 침범이다. 넉넉히 부풀릴수록 자주 생기므로
        반드시 빠져나올 수 있어야 한다.

        나가야 할 방향이 앞쪽이면 그냥 천천히 전진하고,
        뒤쪽이면 후진한다. 차는 제자리에서 못 돌기 때문에, 코를 박은 상태에서는
        후진이 유일한 답이다.
        """
        th = self.vehicle.odom[2]
        e_fwd = G.wrap_pi(np.arctan2(*self.escape_dir[::-1]) - th)
        smax = self.VP.max_steer
        if (abs(e_fwd) < np.radians(55) and self.front_clear > 0.6) \
                or self.rear_clear < self.VP.body_radius + 0.15:
            return 0.45, float(np.clip(1.5 * e_fwd, -smax, smax))
        # 후진: 차 뒤쪽이 탈출 방향을 향하도록 조향한다 (후진 시 회전 부호가 뒤집힌다)
        e_rev = G.wrap_pi(e_fwd + np.pi)
        return -0.40, float(np.clip(-1.5 * e_rev, -smax, smax))

    def _nearest_reachable(self, goal, radius=3.0, n=48):
        """목적지가 장애물 안이면 가장 가까운 갈 수 있는 점으로 대체."""
        best, bd = None, np.inf
        for r in np.linspace(0.3, radius, 10):
            for th in np.linspace(0, 2 * np.pi, n, endpoint=False):
                p = goal + r * np.array([np.cos(th), np.sin(th)])
                if all(not g.contains(p, tol=0.02) for g in self.groups):
                    d = float(np.hypot(*(p - goal)))
                    if d < bd:
                        best, bd = p, d
            if best is not None:
                break
        return best if best is not None else goal

    def _frontier_goal(self, pos):
        """
        경로가 없으면, 아직 안 가본 영역 중 목적지 방향에 가장 가까운 곳을
        임시 목표로 삼는다. 가보면 새 정보가 생기고 답이 나올 수 있다.
        """
        F = self.grid.frontiers()
        if len(F) == 0:
            return None
        if len(F) > 160:                       # 너무 많으면 솎아낸다
            F = F[np.linspace(0, len(F) - 1, 160).astype(int)]
        d = np.full(len(F), 99.0)
        for g in self.groups:                  # 벡터화된 거리 계산
            d = np.minimum(d, g.distance_many(F))
        F = F[d > 0.05]
        if len(F) == 0:
            return None
        score = np.hypot(*(F - self.goal).T) + 0.4 * np.hypot(*(F - pos).T)
        return F[int(np.argmin(score))]

    def _path_blocked(self, path):
        if path is None or len(path) < 2:
            return True
        for g in self.groups:
            if g.blocks_points(path, tol=0.02):
                return True
        return False

    # ================================================================ 실행
    def run(self, max_steps=None):
        n = max_steps or int(self.cfg.max_time / self.cfg.dt)
        for _ in range(n):
            if not self.step():
                break
        return self.summary()

    def summary(self):
        L = np.array(self.trail)
        dist = float(np.sum(np.hypot(*np.diff(L, axis=0).T))) if len(L) > 1 else 0.0
        ms = [r[1] for r in self.log] or [0.0]
        return {
            "arrived": self.arrived,
            "collided": self.collided,
            "time": round(self.t, 2),
            "distance": round(dist, 2),
            "replans": self._replans,
            "lock_released": self._heading_released,
            "swerves": self._swerves,
            "plan_ms_mean": round(float(np.mean(ms)), 1),
            "plan_ms_p95": round(float(np.percentile(ms, 95)), 1),
            "events": self._events,
            "rho": round(self.rho, 3),
            "min_turn_radius": round(self.VP.min_turn_radius, 3),
        }


def _remaining_length(path, pos):
    """현재 위치에서 경로 끝까지 남은 길이.

    경로 비용은 '지금부터 목적지까지'를 뜻한다. 그런데 채택할 때 재둔 비용은
    그때의 위치 기준이라, 차가 전진할수록 실제보다 크다. 두 값을 그냥 비교하면
    새 경로가 언제나 싸 보여서 매 주기 갈아타게 된다. 이 함수가 그 보정에 쓰인다.
    """
    if path is None or len(path) < 2:
        return 0.0
    d = np.hypot(*(path - pos).T)
    i0 = int(np.argmin(d))
    if i0 >= len(path) - 1:
        return 0.0
    seg = np.hypot(*np.diff(path[i0:], axis=0).T)
    return float(seg.sum())


def _path_heading(path, pos, ahead=1.5):
    """현재 위치에서 경로를 ahead 미터 따라갔을 때의 진행 방향(라디안).

    두 경로가 '같은 쪽으로 도는지'를 판정하는 데 쓴다. 장애물을 왼쪽으로 돌다가
    오른쪽으로 트는 뒤집힘은 이 각도가 크게 벌어지는 것으로 나타난다.
    """
    if path is None or len(path) < 2:
        return None
    d = np.hypot(*(path - pos).T)
    i0 = int(np.argmin(d))
    if i0 >= len(path) - 1:
        return None
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
    tgt = s[i0] + ahead
    j = int(np.searchsorted(s, tgt))
    j = min(max(j, i0 + 1), len(path) - 1)
    v = path[j] - path[i0]
    n = float(np.hypot(*v))
    if n < 1e-9:
        return None
    return float(np.arctan2(v[1], v[0]))


def _densify(poly, step=0.10):
    out = [poly[0]]
    for a, b in zip(poly[:-1], poly[1:]):
        L = float(np.hypot(*(b - a)))
        n = max(2, int(L / step) + 1)
        out.extend(a + np.outer(np.linspace(0, 1, n)[1:], (b - a)))
    return np.array(out)
