"""
hybrid_sim.py -- 하이브리드 A* 로 도는 시뮬레이터

차체, 라이다, 오도메트리, 충돌 판정, 안전 루프는 simulator.py 와 똑같이 쓴다.
바뀌는 것은 '보고 나서 무엇을 하느냐'뿐이다.

  계획 루프 (10 Hz)
    본다       라이다
    기억한다   점유 격자에 광선을 기록한다 (numpy 로 한꺼번에)
    잰다       거리장: 칸마다 가장 가까운 장애물까지의 거리
    판단한다   지금 경로를 계속 탈지, 다시 찾을지
    찾는다     필요할 때만 하이브리드 A*

  추종 루프 (50 Hz)
    경로를 '전진 구간'과 '후진 구간'으로 나눠 순서대로 따라간다.
    방향이 바뀌는 지점에서는 멈춘 뒤 반대로 출발한다.
    명령대로 제동 거리만큼 갔을 때 차체가 닿으면 멈추고 다시 찾게 한다.

다시 찾는 조건 (이것이 '지난 트리를 매번 다시 훑는' 낭비를 없앤다)
  없음    : 경로가 없다
  막힘    : 앞으로 갈 구간에서 차체 여유가 기준 아래로 떨어졌다 (새 장애물 발견)
  이탈    : 차가 경로에서 너무 멀어졌다
  갱신    : 마지막 계획 후 일정 시간이 지났다. 새로 보인 공간에 더 좋은 길이 있을 수 있다.
            이때는 남은 비용보다 충분히 싸야만 갈아탄다 (같은 기준, 같은 시점으로 비교).
"""

import copy
import time
from math import atan2, cos, sin, hypot, pi, sqrt

import numpy as np

from . import geometry as G
from . import grid as GR
from .hybrid import HybridAStar, HybridParams, distance_field
from .simulator import Simulator
from .world import Lidar


class HybridConfig:
    def __init__(self,
                 replan_period=1.5,    # 이 시간이 지나면 더 좋은 길이 있는지 본다 (s)
                 adopt_ratio=0.85,     # 갱신 때는 남은 비용의 이 비율보다 싸야 갈아탄다
                 deviate_dist=0.45,    # 경로에서 이만큼 벗어나면 다시 찾는다 (m)
                 check_ahead=6.0,      # 막힘 검사를 할 앞쪽 길이 (m)
                 blocked_slack=0.05,   # 기준 여유보다 이만큼 더 줄어야 '막힘'으로 본다 (m)
                 field_cap=1.2,        # 거리장 상한 (m)
                 fail_backoff=0.8,     # 계획 실패 후 다시 시도하기까지 (s)
                 reverse_max_expand=5000,  # 후진까지 넣은 탐색의 확장 상한
                 stuck_radius=0.5,     # 이 반경 안에서 전진이 또 실패하면 전진을 건너뛴다 (m)
                 stuck_memory=5.0,     # 전진 실패를 기억하는 시간 (s)
                 touch_margin=0.03):   # 예측 정지: 차체 원에서 이만큼 안으로 들어오면 정지 (m)
        self.replan_period = replan_period
        self.adopt_ratio = adopt_ratio
        self.deviate_dist = deviate_dist
        self.check_ahead = check_ahead
        self.blocked_slack = blocked_slack
        self.field_cap = field_cap
        self.fail_backoff = fail_backoff
        self.reverse_max_expand = reverse_max_expand
        self.stuck_radius = stuck_radius
        self.stuck_memory = stuck_memory
        self.touch_margin = touch_margin


# ============================================================== 지도 갱신
def integrate_fast(grid, origin, hits):
    """
    grid.integrate 와 같은 기록을 numpy 로 한꺼번에 한다.

    광선 하나마다 지나간 칸은 '비었음' 한 번, 끝점 칸은 '막혔음'.
    파이썬 반복문으로 칸을 하나씩 따라가면 스캔 한 번에 수만 번 돈다.
    여기서는 광선을 반 칸 간격으로 한꺼번에 샘플링하고, 광선마다 칸을 중복 제거한다.

    아무것도 못 맞힌 광선은 기록하지 않는다. 라이다가 '못 맞힘'과 '반사 누락'을
    구별하지 못해서, 비었다고 기록하면 실제 장애물 칸을 지워버릴 수 있다.
    """
    integrate_rays(grid, origin, hits, mark_free=True, occupied=True)


def integrate_rays(grid, origins, ends, mark_free=True, occupied=True,
                   occ_weight=1.0, groups=None):
    """
    광선 여러 개를 격자에 기록한다. 광선마다 시작점이 달라도 된다 (센서 여러 개).

    mark_free : 끝점 칸을 뺀 지나간 칸을 '비었음'으로
    occupied  : 끝점 칸을 '막혔음'으로
    occ_weight: 막혔음 기록의 세기. 1 이면 한 번에 막힌 칸이 된다. 광선마다 달라도 된다
    groups    : 광선마다 측정 번호. 주면 같은 측정이 한 칸에 두 번 찍지 않는다
    """
    E = np.asarray(ends, float).reshape(-1, 2)
    if len(E) == 0:
        return
    O = np.asarray(origins, float).reshape(-1, 2)
    if len(O) == 1 and len(E) > 1:
        O = np.broadcast_to(O, E.shape)
    res = grid.res
    nx, ny = grid.nx, grid.ny
    e = grid.to_idx(E)
    eid = e[:, 0].astype(np.int64) * ny + e[:, 1]
    Lf = grid.L.reshape(-1)
    if mark_free:
        d = E - O
        r = np.hypot(d[:, 0], d[:, 1])
        u = d / np.maximum(r, 1e-9)[:, None]
        step = 0.5 * res
        t = np.arange(int(r.max() / step) + 1) * step
        P = O[:, None, :] + u[:, None, :] * t[None, :, None]
        fi = np.clip(((P[..., 0] - grid.origin[0]) / res).astype(np.int64), 0, nx - 1)
        fj = np.clip(((P[..., 1] - grid.origin[1]) / res).astype(np.int64), 0, ny - 1)
        cid = fi * ny + fj
        m = (t[None, :] < r[:, None]) & (cid != eid[:, None])
        ncell = nx * ny
        rays = np.broadcast_to(np.arange(len(E), dtype=np.int64)[:, None], cid.shape)
        cells = np.unique(rays[m] * ncell + cid[m]) % ncell
        np.add.at(Lf, cells, GR.L_FREE)
        np.maximum(Lf, GR.L_MIN, out=Lf)
    if occupied:
        inc = (GR.L_OCC - GR.L_FREE) * np.broadcast_to(np.asarray(occ_weight, float), eid.shape)
        if groups is not None:
            key = np.asarray(groups, np.int64) * (nx * ny) + eid
            _, first = np.unique(key, return_index=True)
            np.add.at(Lf, eid[first], inc[first])
        else:
            np.add.at(Lf, eid, inc)
        np.minimum(Lf, GR.L_MAX, out=Lf)


# ============================================================== 추종
class SegmentTracker:
    """
    전진/후진이 섞인 경로를 따라간다.

    경로를 방향이 바뀌는 지점(꺾임점)에서 구간으로 나눈다.
    한 구간 안에서는 순수 추종을 쓰고, 후진 구간에서는 차 뒤쪽을 앞으로 삼아
    조향 부호를 뒤집는다. 꺾임점에 가까워지면 속도를 줄여 멈춘 뒤 다음 구간으로 넘어간다.

    가장 가까운 점은 '지금 구간 안, 지난번 위치 근처'에서만 찾는다. 후진 구간은
    같은 자리를 되짚는 경우가 많아서 전체에서 찾으면 엉뚱한 구간으로 건너뛴다.
    """

    def __init__(self, veh, ctrl_params, clear_v0=0.70, clear_gain=4.0, clear_zero=0.05):
        self.V = veh
        self.P = ctrl_params
        # 여유에 비례한 속도 상한: v <= clear_v0 + clear_gain * (여유 - clear_zero)
        #
        # 약하게 둔다. 처음에는 0.30 / 2.2 / 0.15 로 세게 걸었는데, 벽을 따라 나란히
        # 달릴 때마다 감속해서 walls 가 24.7 초로 예전 계획기(17.6 초)보다 느렸다.
        # 제한을 아예 꺼도 차체 여유는 줄지 않았다 (walls 0.21 -> 0.44 m).
        # 부딪힘은 _control 의 예측 정지가 차체 기준으로 막고 있기 때문이다.
        # 그래서 평소에는 걸리지 않고, 여유가 거의 없을 때만 묶이는 값으로 둔다.
        self.clear_v0 = clear_v0
        self.clear_gain = clear_gain
        self.clear_zero = clear_zero
        # 후진 최고 속도 (m/s). None 이면 차량 후진 한계의 75%.
        # 뒤를 센서 하나로만 보는 차는 느리게 두어야 옆 뒤 사각지대에서 멈출 틈이 생긴다.
        self.rev_speed = None
        self.lookahead_pt = None
        self.err = 0.0
        self.set_path(None, None)

    def set_path(self, poses, dirs):
        self.poses, self.dirs = poses, dirs
        self.segs, self.seg, self.idx = [], 0, 0
        if poses is None:
            return
        start = 0
        for i in range(1, len(dirs)):
            if dirs[i] != dirs[i - 1]:
                self.segs.append((start, i - 1, int(dirs[i - 1])))
                start = i - 1
        self.segs.append((start, len(dirs) - 1, int(dirs[-1])))
        ds = np.hypot(*np.diff(poses[:, :2], axis=0).T)
        dth = np.abs((np.diff(poses[:, 2]) + pi) % (2 * pi) - pi)
        kap = np.where(ds > 1e-6, dth / np.maximum(ds, 1e-6), 0.0)
        self.kap = np.append(kap, 0.0)
        self.s = np.concatenate([[0.0], np.cumsum(ds)])

    @property
    def direction(self):
        return self.segs[self.seg][2] if self.segs else 1

    def global_index(self):
        return self.idx

    def update_progress(self, pos):
        s, e, _ = self.segs[self.seg]
        lo = max(s, self.idx - 2)
        hi = min(e, self.idx + 60)
        seg = self.poses[lo:hi + 1, :2]
        dd = np.hypot(*(seg - pos).T)
        k = int(np.argmin(dd))
        self.idx = lo + k
        self.err = float(dd[k])

    def compute(self, pose, v, goal, clearance_ahead):
        """반환: (목표속도(부호 있음), 목표조향, 정보)."""
        V, P = self.V, self.P
        if self.poses is None:
            return 0.0, 0.0, {"mode": "no_path"}
        pos, th = pose[:2], pose[2]
        self.update_progress(pos)

        # ---- 꺾임점에서 다음 구간으로
        for _ in range(2):
            s, e, d = self.segs[self.seg]
            last = self.seg == len(self.segs) - 1
            rem = float(self.s[e] - self.s[self.idx])
            if last:
                break
            mdir = self.poses[e, 2] + (0.0 if d > 0 else pi)
            ex, ey = pos - self.poses[e, :2]
            passed = cos(mdir) * ex + sin(mdir) * ey > -0.05
            if (rem < 0.12 or passed) and abs(v) < 0.10:
                self.seg += 1
                self.idx = self.segs[self.seg][0]
                self.update_progress(pos)
                continue
            break
        s, e, d = self.segs[self.seg]
        last = self.seg == len(self.segs) - 1
        rem = float(self.s[e] - self.s[self.idx])

        # ---- 전방주시점
        if d > 0:
            Ld = float(np.clip(P.k_lookahead * max(abs(v), 0.4), P.ld_min, P.ld_max))
        else:
            Ld = 0.55
        k_ahead = float(self.kap[self.idx:min(e, self.idx + 25) + 1].max())
        if k_ahead > 1e-3:
            Ld = min(Ld, max(0.45, 0.70 / k_ahead))
        target_s = self.s[self.idx] + Ld
        if target_s <= self.s[e]:
            j = int(np.searchsorted(self.s, target_s))
            j = min(max(j, self.idx), e)
            target = self.poses[j, :2]
        else:
            over = target_s - self.s[e]
            mdir = self.poses[e, 2] + (0.0 if d > 0 else pi)
            target = self.poses[e, :2] + over * np.array([cos(mdir), sin(mdir)])
        self.lookahead_pt = target

        # ---- 순수 추종 (후진이면 차 뒤쪽이 앞)
        fwd = th if d > 0 else th + pi
        dv = target - pos
        eta = (atan2(dv[1], dv[0]) - fwd + pi) % (2 * pi) - pi
        dist = max(float(hypot(*dv)), 1e-3)
        steer = atan2(2.0 * V.wheelbase * sin(eta), dist)
        if d < 0:
            steer = -steer
        steer = float(np.clip(steer, -V.max_steer, V.max_steer))

        # ---- 속도
        rev_v = self.rev_speed if self.rev_speed is not None else 0.75 * V.max_reverse
        vmax = V.max_speed if d > 0 else rev_v
        vc = vmax
        kap = max(k_ahead, abs(np.tan(steer) / V.wheelbase))
        if kap > 1e-3:
            vc = min(vc, sqrt(V.max_lat_acc / kap))
        vc = min(vc, self.clear_v0
                 + self.clear_gain * max(clearance_ahead - self.clear_zero, 0.0))
        if last:
            d_goal = float(hypot(*(goal - pos)))
            vc = min(vc, max(0.25, 1.4 * d_goal))
        else:
            # 꺾임점에서 멈출 수 있게 미리 줄인다
            vc = min(vc, sqrt(2.0 * 1.5 * max(rem - 0.06, 0.0)))
        # 크게 어긋나 있으면 천천히
        if abs(eta) > np.radians(60):
            vc = min(vc, 0.35)
        return float(d * max(vc, 0.0)), steer, {
            "mode": "track", "dir": d, "seg": self.seg, "rem": rem, "eta": eta}


# ============================================================== 시뮬레이터
class HybridSimulator(Simulator):
    planner_name = "hybrid"

    def __init__(self, world, start_pose, goal, cfg=None, veh=None, lidar=None,
                 plan_params=None, ctrl=None, seed=0, hparams=None, hcfg=None):
        super().__init__(world, start_pose, goal, cfg=cfg, veh=veh, lidar=lidar,
                         plan_params=plan_params, ctrl=ctrl, seed=seed)
        self.HP = hparams or HybridParams()
        self.HC = hcfg or HybridConfig()
        self.planner = HybridAStar(self.VP, self.HP)
        # 후진이 허용되면 전진 전용 계획기를 하나 더 둔다 (_search 참조)
        self.planner_fwd = None
        if self.HP.allow_reverse:
            hp_fwd = copy.copy(self.HP)
            hp_fwd.allow_reverse = False
            self.planner_fwd = HybridAStar(self.VP, hp_fwd)
        self.follower = SegmentTracker(self.VP, self.ctrl.P)
        self.D = None
        self.path_poses = None
        self.path_cum = None
        self._last_plan_t = -1e9
        self._fail_until = -1e9
        self._force_replan = False
        self._plans = 0
        self._plan_fail = 0
        self._stops = 0
        self._reverse_plans = 0
        self._fwd_fail_pos = None
        self._fwd_fail_t = -1e9
        self._search_ms = []
        self._reasons = {}
        self.search_ms = 0.0
        # 화면 표시용. 장애물 표면에서 차축까지 반드시 비우는 거리.
        self.rho = self.planner.r_hard_base + 0.5 * sqrt(2.0) * self.grid.res
        self.rho_hard = self.planner.fp.radius

    # ------------------------------------------------------------ 인식 + 계획
    def _sense_and_plan(self):
        t0 = time.perf_counter()
        HC = self.HC

        odom = self.vehicle.odom
        if hasattr(self.lidar, "measure"):
            self._sense_array(odom)
        else:
            angles, ranges = self.lidar.scan(self.world, self.vehicle.pose)
            pts_body, _ = Lidar.to_points(angles, ranges)
            if len(pts_body):
                pts_odom = G.compose(odom, pts_body)
                self.scan_pts_world = G.compose(self.vehicle.pose, pts_body)
            else:
                pts_odom = np.zeros((0, 2))
                self.scan_pts_world = np.zeros((0, 2))
            front = np.abs(G.wrap_pi(angles)) < np.radians(35)
            fr = ranges[front]
            self.front_clear = float(np.min(fr)) if np.isfinite(fr).any() else 10.0
            rear = np.abs(G.wrap_pi(angles - np.pi)) < np.radians(35)
            rr = ranges[rear]
            self.rear_clear = float(np.min(rr)) if np.isfinite(rr).any() else 10.0
            if len(pts_odom):
                integrate_fast(self.grid, odom[:2], pts_odom)
        self.D = distance_field(self.grid.occupied_mask(), self.grid.res, HC.field_cap)
        self.planner.prepare(self.grid, self.D)

        pos, th = odom[:2], float(odom[2])
        goal = self._free_goal(self.goal)

        # ---- 다시 찾을지 판단
        reason = None
        if self.path_poses is None:
            reason = "none"
        elif self._force_replan:
            reason = "blocked"
        else:
            i0 = self.follower.global_index()
            s = self.follower.s
            j = int(np.searchsorted(s, s[i0] + HC.check_ahead))
            ahead = self.path_poses[i0:max(j, i0 + 1) + 1]
            m = self.planner.clearance_poses(ahead[:, 0], ahead[:, 1], ahead[:, 2])
            if len(m) and float(m.min()) < self.planner.r_hard - HC.blocked_slack:
                reason = "blocked"
            elif self.follower.err > HC.deviate_dist:
                reason = "deviate"
            elif self.t - self._last_plan_t > HC.replan_period:
                reason = "refresh"
        if reason in ("none", "refresh") and self.t < self._fail_until:
            reason = None

        if reason is not None:
            self._force_replan = False
            self._reasons[reason] = self._reasons.get(reason, 0) + 1
            ts = time.perf_counter()
            res = self._search((pos[0], pos[1], th), goal, reason)
            explore = False
            if res is None:
                fg = self._frontier_goal(pos)
                if fg is not None:
                    res = self._search((pos[0], pos[1], th), fg, reason)
                    explore = res is not None
            self.search_ms = (time.perf_counter() - ts) * 1e3
            self._search_ms.append(self.search_ms)
            self._plans += 1
            self._last_plan_t = self.t

            if res is not None:
                self.graph = res
                if reason == "refresh" and self.path_poses is not None:
                    i0 = self.follower.global_index()
                    remaining = float(self.path_cum[-1] - self.path_cum[i0])
                    take = res.cost < remaining * HC.adopt_ratio
                else:
                    take = True
                if take:
                    self._adopt(res)
                    self.mode = "explore" if explore else "drive"
            else:
                self._plan_fail += 1
                self._fail_until = self.t + HC.fail_backoff
                if reason in ("none", "blocked"):
                    self._adopt(None)
                    self.mode = "no_path"

        self.plan_ms = (time.perf_counter() - t0) * 1e3
        self.log.append((self.t, self.plan_ms, 0,
                         0 if self.graph is None else len(self.graph.tree)))

    def _sense_array(self, odom):
        """sensors.RangeSensorArray 로 재고 격자에 기록한다."""
        S = self.lidar
        m = S.measure(self.world, self.vehicle.pose)
        self.last_measure = m                      # 화면에 센서 원뿔을 그릴 때 쓴다
        n = len(S.angles)
        if not hasattr(self, "_beam_r"):
            self._beam_r = np.full(n, np.inf)
            self._beam_t = np.full(n, -1e9)
        self._beam_r[m.idx] = m.ranges
        self._beam_t[m.idx] = self.t

        (fo, fe), (oo, oe) = S.rays_for_map(m, self.grid.res)
        if len(fe):
            integrate_rays(self.grid, G.compose(odom, fo), G.compose(odom, fe),
                           mark_free=True, occupied=False)
        if getattr(S, "map_arc", False):
            ao, ae, aw, ak = S.occupied_for_map(m, self.grid.res)
            if len(ae):
                integrate_rays(self.grid, G.compose(odom, ao), G.compose(odom, ae),
                               mark_free=False, occupied=True, occ_weight=aw, groups=ak)
        elif len(oe):
            integrate_rays(self.grid, G.compose(odom, oo), G.compose(odom, oe),
                           mark_free=False, occupied=True)
        self.scan_pts_world = G.compose(self.vehicle.pose, oe) if len(oe) else np.zeros((0, 2))

        # 최근 0.3 초 안에 잰 값으로만 앞뒤 여유를 잡는다 (뒷바퀴 축에서의 거리)
        fresh = (self.t - self._beam_t) < 0.3
        r = self._beam_r
        ends = S.mounts + np.where(np.isfinite(r), r, 0.0)[:, None] \
            * np.stack([np.cos(S.angles), np.sin(S.angles)], axis=1)
        dist = np.where(fresh & np.isfinite(r), np.hypot(ends[:, 0], ends[:, 1]), np.inf)
        front = np.abs(G.wrap_pi(S.angles)) < np.radians(35)
        rear = np.abs(G.wrap_pi(S.angles - np.pi)) < np.radians(35)
        self.front_clear = float(dist[front].min()) if np.isfinite(dist[front]).any() else 10.0
        self.rear_clear = float(dist[rear].min()) if np.isfinite(dist[rear]).any() else 10.0

    def _search(self, start, goal, reason):
        """
        전진만으로 먼저 찾고, 정말 막혔을 때만 후진을 넣어 한 번 더 찾는다.

        후진은 기능으로 남겨 두되 평소 주행에 짐이 되지 않게 한다.
          평소      : 조각 수가 절반인 전진 전용 탐색만 돈다.
          정기 갱신 : 전진으로 못 찾으면 그냥 가던 길을 간다. 후진을 넣지 않는다.
          없음/막힘 : 그때만 후진을 넣는다.
          같은 자리 : 방금 여기서 전진이 실패했으면 전진을 건너뛰고 바로 후진을 넣는다.

        같은 자리 규칙이 없으면 막다른 곳에서 매 시도마다 전진 탐색이 상한까지 헛돈다.
        처음에는 후진 탐색 상한도 절반으로 줄였는데, 그러면 탈출로를 끝내 못 찾아
        trap 이 25 분 동안 끝나지 않았다. 후진 탐색은 제대로 하고, 헛도는 전진을 뺀다.

        후진을 넣은 탐색은 길이 없을 때 전진과 후진을 끝까지 다 뒤진다. 막다른 곳이
        아니어도, 벽이 띄엄띄엄 찍혀 잠깐 막힌 것처럼 보이면 그런 일이 생긴다.
        """
        if self.planner_fwd is None:
            return self.planner.plan(start, goal, self.grid, self.D)
        here = np.array(start[:2])
        stuck_here = (self._fwd_fail_pos is not None
                      and self.t - self._fwd_fail_t < self.HC.stuck_memory
                      and float(np.hypot(*(here - self._fwd_fail_pos))) < self.HC.stuck_radius)
        if not stuck_here:
            r = self.planner_fwd.plan(start, goal, self.grid, self.D)
            if r is not None or reason not in ("none", "blocked"):
                return r
            self._fwd_fail_pos, self._fwd_fail_t = here, self.t
        P = self.planner.P
        saved = P.max_expand
        P.max_expand = min(saved, self.HC.reverse_max_expand)
        try:
            r = self.planner.plan(start, goal, self.grid, self.D)
        finally:
            P.max_expand = saved
        if r is not None:
            self._reverse_plans += 1
        return r

    def _adopt(self, res):
        if res is None:
            self.path_poses = self.path_cum = self.path = None
            self.follower.set_path(None, None)
            return
        self.path_poses = res.poses
        self.path_cum = res.cum_cost
        self.path = res.poses[:, :2]
        self.path_dirs = res.dirs
        self.follower.set_path(res.poses, res.dirs)
        self._replans += 1

    # ------------------------------------------------------------ 제어
    def _control(self):
        if self.path_poses is None or self.D is None:
            self.cmd = (0.0, 0.0)
            return
        f = self.follower
        pose = self.vehicle.odom
        f.update_progress(pose[:2])
        i0 = f.global_index()
        j = min(len(self.path_poses) - 1, i0 + 15)
        ahead = self.path_poses[i0:j + 1]
        clear = float(self.planner.clearance_poses(ahead[:, 0], ahead[:, 1],
                                                   ahead[:, 2]).min()) \
            - self.planner.fp.radius
        v, s, info = f.compute(pose, self.vehicle.v, self.goal, clear)
        self.ctrl.lookahead_pt = f.lookahead_pt

        # ---- 예측 정지: 이 명령대로 제동 거리만큼 가면 차체가 닿는가
        vm = self.vehicle.v
        sgn = np.sign(v) if abs(v) > 1e-3 else np.sign(vm)
        if sgn != 0:
            dist = max(abs(vm), abs(v)) ** 2 / (2.0 * self.VP.max_decel) + 0.12
            kap = np.tan(self.vehicle.steer) / self.VP.wheelbase
            sig = sgn * np.linspace(0.04, dist, 6)
            x, y, th = pose
            dth = kap * sig
            if abs(kap) < 1e-6:
                dx, dy = sig, np.zeros_like(sig)
            else:
                dx, dy = np.sin(dth) / kap, (1.0 - np.cos(dth)) / kap
            X = x + cos(th) * dx - sin(th) * dy
            Y = y + sin(th) * dx + cos(th) * dy
            m = self.planner.clearance_poses(X, Y, th + dth)
            # 기준 안으로 들어가는 움직임은 멀어지는 방향이어도 전부 막는다.
            #
            # '더 가까워지는 움직임만 막기'로 바꿔 본 적이 있다 (무작위 지도 1026).
            # 가는 기둥에 붙어 갇힌 차를 풀어 주려던 것인데, 오히려 차체가 기둥에 닿았다.
            # 예측 정지가 보는 것은 실제 기둥이 아니라 지도에 찍힌 점이고, 싼 센서는 가는
            # 기둥을 원뿔 가운데에 찍어 위치가 어긋난다. 지도로는 멀어지는데 실제로는
            # 가까워졌다. 갇히는 것보다 닿는 것이 나쁘므로 되돌렸다.
            if float(m.min()) < self.planner.fp.radius + self.HC.touch_margin:
                if self.mode != "estop":
                    self._stops += 1
                v = 0.0
                self.mode = "estop"
                self._force_replan = True
            elif self.mode == "estop":
                self.mode = "drive"
        self.cmd = (v, s)

    # ------------------------------------------------------------ 보조
    def _free_goal(self, goal):
        """목적지가 장애물에 너무 붙어 있으면 가장 가까운 넉넉한 점으로 옮긴다."""
        g = self.grid
        i, j = g.to_idx(goal)[0]
        if self.D[i, j] >= self.planner.r_hard:
            return goal
        for r in np.linspace(0.2, 3.0, 15):
            for a in np.linspace(0, 2 * pi, 36, endpoint=False):
                p = goal + r * np.array([cos(a), sin(a)])
                ii, jj = g.to_idx(p)[0]
                if self.D[ii, jj] >= self.planner.r_soft:
                    return p
        return goal

    def _frontier_goal(self, pos):
        """목적지로 가는 길이 없으면, 목적지 쪽 미지 영역의 경계로 먼저 간다."""
        F = self.grid.frontiers()
        if len(F) == 0:
            return None
        idx = self.grid.to_idx(F)
        F = F[self.D[idx[:, 0], idx[:, 1]] >= self.planner.r_soft]
        F = F[np.hypot(*(F - pos).T) > 1.0]
        if len(F) == 0:
            return None
        score = np.hypot(*(F - self.goal).T) + 0.4 * np.hypot(*(F - pos).T)
        return F[int(np.argmin(score))]

    def hud_text(self):
        s = self
        err = np.hypot(*(s.vehicle.pose[:2] - s.vehicle.odom[:2]))
        d = s.follower.direction if s.path_poses is not None else 0
        dtxt = {1: "fwd", -1: "rev", 0: "---"}[int(d)]
        return (f"t {s.t:6.1f}s   mode {s.mode:<7s} {dtxt}\n"
                f"v {s.vehicle.v:+5.2f} m/s  steer {np.degrees(s.vehicle.steer):+5.1f}deg\n"
                f"hybrid A*  body r {s.planner.fp.radius:.2f}  clear {s.rho:.2f} m\n"
                f"cycle {s.plan_ms:5.1f} ms  search {s.search_ms:6.1f} ms\n"
                f"plans {s._plans:3d}   odom err {err:4.2f} m")

    def summary(self):
        r = super().summary()
        sm = self._search_ms or [0.0]
        r.update({
            "planner": "hybrid",
            "plans": self._plans,
            "plan_fail": self._plan_fail,
            "safety_stops": self._stops,
            "reverse_plans": self._reverse_plans,
            "search_ms_mean": round(float(np.mean(sm)), 1),
            "search_ms_max": round(float(np.max(sm)), 1),
            "replan_reasons": dict(self._reasons),
            "rho": round(self.rho, 3),
        })
        return r
