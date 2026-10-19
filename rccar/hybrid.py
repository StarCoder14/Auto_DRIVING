"""
hybrid.py -- 하이브리드 A* 계획기  (접선 그래프의 대체)

왜 바꿨나
  접선 그래프는 '제자리에서 방향을 바꿀 수 있는 점'이 '미리 알려진 도형' 사이를
  가는 문제에 맞는 방법이다. 우리 차는 제자리에서 못 돌고, 지도는 달리면서
  조금씩 알게 된다. 그 어긋남이 측정에서 그대로 드러났다.

    1) 차 뒤로 가는 경로를 태연히 고른다. 계획기가 차의 방향을 모른다.
    2) 부풀림 반지름이 회전 반경을 겸해 0.85 m 가 된다. 폭 0.28 m 인 차가
       1.69 m 보다 좁은 틈을 못 지나간다. 방법 구조상 둘을 나눌 수 없다.
    3) 0.1 초마다 노선도를 처음부터 만들고 처음부터 찾는다.

하이브리드 A* 가 이 셋을 구조적으로 없애는 방식
  1) 상태가 (x, y, 방향) 이다. '차가 실제로 그릴 수 있는 곡선 조각'만 이어
     붙이므로 제자리 회전 같은 경로는 애초에 후보가 되지 않는다.
     후진도 조각의 하나라서 막다른 골목에서 자연스럽게 되돌아 나온다.
  2) 충돌은 실제 차체로 판정한다. 차체 사각형을 원 세 개로 덮고, 격자 지도에서
     만든 거리장(가장 가까운 장애물까지의 거리)으로 검사한다. 회전 반경은 조각의
     곡률로만 들어가므로 통로 폭과 섞이지 않는다.
  3) 다시 찾을지 말지는 시뮬레이터가 정한다 (hybrid_sim.py). 경로가 멀쩡하면
     다시 찾지 않는다.

'하이브리드'인 이유
  탐색은 격자 칸(x, y, 방향 구간)으로 중복을 거르지만, 노드가 가진 위치와
  방향은 칸의 중심으로 반올림하지 않은 연속값이다. 그래서 칸 크기와 무관하게
  차가 그대로 따라갈 수 있는 매끈한 경로가 나온다.

속도를 위한 장치
  휴리스틱 : 목적지에서 장애물을 피해 퍼져 나가는 거리(웨이브프런트).
             '벽 너머 목적지'가 가깝다고 착각하지 않아 탐색이 헤매지 않는다.
  직결 시도: 몇 번 확장할 때마다 '한 번 꺾고 곧장 가기'로 목적지에 닿는지 본다.
             트인 곳에서는 탐색이 수십 번 만에 끝난다.
  가중치   : 휴리스틱에 1보다 큰 배율을 곱한다. 최단이 조금 양보되는 대신 빠르다.
"""

import heapq
from math import atan2, acos, cos, sin, hypot, pi, sqrt, tau

import numpy as np


class HybridParams:
    def __init__(self,
                 xy_res=0.24,          # 중복 판정 칸 크기 (m)
                 theta_bins=72,        # 방향 구간 수 (5도)
                 step=0.40,            # 조각 하나의 길이 (m). 칸 대각선보다 길어야 한다
                 samples=4,            # 조각당 충돌 검사 지점 수 (0.1 m 간격)
                 turn_slack=1.15,      # 계획 회전 반경 = 최소 회전 반경 x 이 값
                 allow_reverse=True,
                 rev_mult=2.5,         # 후진 거리 비용 배율
                 switch_cost=1.5,      # 전진<->후진 전환 1회 비용 (m 환산)
                 w_steer=0.15,         # 핸들을 꺾은 채 가는 비용
                 w_steer_change=0.35,  # 핸들을 바꾸는 비용
                 hard_margin=0.06,     # 차체 원 바깥으로 반드시 비워야 할 여유 (m)
                 soft_margin=0.40,     # 이 폭 안으로 들어가면 벌점이 커진다 (m)
                 w_clear=4.0,          # 여유 벌점 크기
                 w_unknown=0.25,       # 미지 영역 1 m 당 벌점
                 h_weight=1.6,         # 휴리스틱 배율
                 max_expand=5000,      # 확장 상한 (계산 시간 상한)
                 shot_every=5,         # 직결 시도 주기
                 shot_max_len=14.0,    # 직결을 시도할 최대 길이 (m)
                 goal_tol=0.30):       # 이 안에 들어오면 도착으로 본다 (m)
        self.xy_res = xy_res
        self.theta_bins = theta_bins
        self.step = step
        self.samples = samples
        self.turn_slack = turn_slack
        self.allow_reverse = allow_reverse
        self.rev_mult = rev_mult
        self.switch_cost = switch_cost
        self.w_steer = w_steer
        self.w_steer_change = w_steer_change
        self.hard_margin = hard_margin
        self.soft_margin = soft_margin
        self.w_clear = w_clear
        self.w_unknown = w_unknown
        self.h_weight = h_weight
        self.max_expand = max_expand
        self.shot_every = shot_every
        self.shot_max_len = shot_max_len
        self.goal_tol = goal_tol


# ============================================================== 차체
class Footprint:
    """
    차체 사각형을 축 위의 원 세 개로 덮는다.

    기준점은 뒷바퀴 축 중심이다 (vehicle.py 의 pose 와 같다).
    원 세 개면 사각형 모서리까지 덮으면서, 사각형 하나로 검사하는 것보다 훨씬 싸다.
    """

    def __init__(self, veh, n=3):
        rear = veh.rear_overhang_m                          # vehicle.footprint 와 같은 값
        seg = veh.length / n
        self.offsets = np.array([-rear + seg * (k + 0.5) for k in range(n)])
        self.radius = float(hypot(seg / 2, veh.width / 2))


# ============================================================== 거리장
def distance_field(occ, res, cap):
    """
    각 칸에서 가장 가까운 막힌 칸까지의 유클리드 거리 (m). cap 에서 잘린다.

    numpy 만으로 정확한 거리를 얻는다. 거리 제곱은 축별로 나눌 수 있다는 성질을 쓴다.
      1) 행마다 '같은 행에서 가장 가까운 막힌 칸까지의 칸 수' g 를 누적 연산으로 구한다.
      2) D^2(i, j) = min_k  g(i, j+k)^2 + k^2   (k 는 -cap..cap)
    전체 지도(270 x 270)에서 수 ms 걸린다. 계획 주기마다 새로 만들어도 된다.
    """
    nx, ny = occ.shape
    big = int(np.ceil(cap / res)) + 1
    idx = np.broadcast_to(np.arange(nx)[:, None], (nx, ny))
    last = np.maximum.accumulate(np.where(occ, idx, -10 ** 6), axis=0)
    nxt = np.minimum.accumulate(np.where(occ, idx, 10 ** 6)[::-1], axis=0)[::-1]
    g = np.minimum(np.minimum(idx - last, nxt - idx), big).astype(np.float32)
    g2 = g * g
    d2 = g2.copy()
    for k in range(1, big):
        kk = float(k * k)
        np.minimum(d2[:, :-k], g2[:, k:] + kk, out=d2[:, :-k])
        np.minimum(d2[:, k:], g2[:, :-k] + kk, out=d2[:, k:])
    return np.minimum(np.sqrt(d2) * res, cap).astype(np.float32)


def wavefront(free, goal_ij, max_iter=900):
    """
    목적지에서 출발해 막히지 않은 칸으로 퍼져 나간 8방향 거리 (칸 단위).
    못 닿는 칸은 inf. 휴리스틱으로 쓴다.
    """
    INF = np.float32(1e9)
    d = np.full(free.shape, INF, dtype=np.float32)
    gi, gj = goal_ij
    blocked = ~free
    blocked[gi, gj] = False
    d[gi, gj] = 0.0
    s2 = np.float32(sqrt(2.0))
    # (목적지 칸, 이웃 칸, 한 칸 비용)
    #
    # 막힌 칸은 방향 하나를 퍼뜨릴 때마다 다시 막는다.
    # 여덟 방향을 다 퍼뜨린 뒤 한 번만 막으면, 그 사이에 값이 얇은 벽 칸을 타고
    # 건너편으로 새어 나간다. 실제로 그래서 ㄷ자 골목 안에서 '벽 너머 목적지까지
    # 직선 거리'가 휴리스틱으로 나왔고, 탐색이 벽을 향해 헤매다 상한에 걸렸다.
    sweeps = (
        ((slice(1, None), slice(None)), (slice(None, -1), slice(None)), 1.0),
        ((slice(None, -1), slice(None)), (slice(1, None), slice(None)), 1.0),
        ((slice(None), slice(1, None)), (slice(None), slice(None, -1)), 1.0),
        ((slice(None), slice(None, -1)), (slice(None), slice(1, None)), 1.0),
        ((slice(1, None), slice(1, None)), (slice(None, -1), slice(None, -1)), s2),
        ((slice(None, -1), slice(None, -1)), (slice(1, None), slice(1, None)), s2),
        ((slice(1, None), slice(None, -1)), (slice(None, -1), slice(1, None)), s2),
        ((slice(None, -1), slice(1, None)), (slice(1, None), slice(None, -1)), s2),
    )
    for it in range(max_iter):
        prev = d.copy() if it % 6 == 0 else None
        for dst, src, w in sweeps:
            np.minimum(d[dst], d[src] + w, out=d[dst])
            d[blocked] = INF
        if prev is not None and np.array_equal(prev, d):
            break
    d[d >= INF * 0.5] = np.inf
    return d


# ============================================================== 결과
class HybridResult:
    def __init__(self, poses, dirs, cum_cost, cost, expanded, tree):
        self.poses = poses          # (N, 3)  x, y, 방향
        self.dirs = dirs            # (N,)    +1 전진, -1 후진
        self.cum_cost = cum_cost    # (N,)    출발점부터 누적 비용
        self.cost = cost
        self.expanded = expanded
        self.tree = tree            # 그리기용 탐색 나무 (선분 목록)

    @property
    def xy(self):
        return self.poses[:, :2]

    def edges_for_draw(self, limit=500):
        """viz.py 가 부르는 이름. 탐색이 실제로 뻗어 나간 조각들을 돌려준다."""
        return self.tree[-limit:] if len(self.tree) > limit else self.tree

    @property
    def pos(self):                  # simulator 로그 호환용
        return self.tree


# ============================================================== 계획기
class HybridAStar:
    def __init__(self, veh, params=None):
        self.V = veh
        self.P = params or HybridParams()
        self.fp = Footprint(veh)
        P = self.P

        self.r_turn = veh.min_turn_radius * P.turn_slack
        self.kmax = 1.0 / self.r_turn
        self.r_hard_base = self.fp.radius + P.hard_margin

        # ---- 조각 표. 방향과 무관한 상대 이동량을 미리 계산해 둔다.
        ks = [-self.kmax, -0.5 * self.kmax, 0.0, 0.5 * self.kmax, self.kmax]
        dirs = [1, -1] if P.allow_reverse else [1]
        self.prim_k = np.array([k for d in dirs for k in ks])
        self.prim_d = np.array([d for d in dirs for _ in ks])
        s = P.step * (np.arange(P.samples) + 1.0) / P.samples      # (K,)
        sig = self.prim_d[:, None] * s[None, :]                    # (P, K) 부호 있는 이동거리
        k = self.prim_k[:, None]
        dth = k * sig
        small = np.abs(k) < 1e-9
        kk = np.where(small, 1.0, k)
        self.rel_dx = np.where(small, sig, np.sin(dth) / kk)
        self.rel_dy = np.where(small, 0.0, (1.0 - np.cos(dth)) / kk)
        self.rel_dth = dth

    # ------------------------------------------------------------ 준비
    def prepare(self, grid, D):
        """계획 한 번에 쓸 지도 정보를 묶는다."""
        self.grid = grid
        self.D = D
        self.res = grid.res
        self.ox, self.oy = float(grid.origin[0]), float(grid.origin[1])
        self.nx, self.ny = grid.nx, grid.ny
        # 거리장은 칸 중심 기준이라 실제 표면은 반 칸 대각선만큼 더 가까울 수 있다
        self.r_hard = self.r_hard_base + 0.5 * sqrt(2.0) * grid.res
        self.r_soft = self.r_hard + self.P.soft_margin
        self.unknown = np.abs(grid.L) < 1e-6

    def clearance_poses(self, X, Y, TH):
        """자세 배열에서 차체 원들의 최소 여유(거리장 값). 지도 밖은 -1."""
        offs = self.fp.offsets
        CX = X[..., None] + np.cos(TH)[..., None] * offs
        CY = Y[..., None] + np.sin(TH)[..., None] * offs
        fi = (CX - self.ox) / self.res
        fj = (CY - self.oy) / self.res
        inside = (fi >= 0) & (fj >= 0) & (fi < self.nx) & (fj < self.ny)
        i = np.clip(fi.astype(np.int64), 0, self.nx - 1)
        j = np.clip(fj.astype(np.int64), 0, self.ny - 1)
        d = np.where(inside, self.D[i, j], -1.0)
        return d.min(axis=-1)

    def _penalty(self, m, X, Y, length):
        """여유 벌점 + 미지 영역 벌점. m 은 지점별 최소 여유."""
        P = self.P
        n = m.shape[-1]
        short = np.clip((self.r_soft - m) / P.soft_margin, 0.0, None)
        pen = P.w_clear * (length / n) * np.sum(short * short, axis=-1)
        i = np.clip(((X - self.ox) / self.res).astype(np.int64), 0, self.nx - 1)
        j = np.clip(((Y - self.oy) / self.res).astype(np.int64), 0, self.ny - 1)
        pen = pen + P.w_unknown * (length / n) * np.sum(self.unknown[i, j], axis=-1)
        return pen

    # ------------------------------------------------------------ 휴리스틱
    def build_heuristic(self, goal):
        """칸 두 개를 하나로 묶은 거친 격자에서 목적지까지의 우회 거리."""
        f = 2
        D = self.D
        nx2, ny2 = self.nx // f, self.ny // f
        blk = D[:nx2 * f, :ny2 * f].reshape(nx2, f, ny2, f).max(axis=(1, 3))
        # 낙관적으로: 거친 칸 안에 지나갈 수 있는 곳이 하나라도 있으면 통행 가능
        free = blk >= self.r_hard
        gi = int(np.clip((goal[0] - self.ox) / (self.res * f), 0, nx2 - 1))
        gj = int(np.clip((goal[1] - self.oy) / (self.res * f), 0, ny2 - 1))
        self.h_grid = wavefront(free, (gi, gj)) * (self.res * f)
        self.h_f = f
        self.h_shape = (nx2, ny2)

    def h(self, x, y, gx, gy):
        f = self.h_f
        i = int((x - self.ox) / (self.res * f))
        j = int((y - self.oy) / (self.res * f))
        if not (0 <= i < self.h_shape[0] and 0 <= j < self.h_shape[1]):
            return np.inf
        v = float(self.h_grid[i, j])
        # 거친 격자의 반올림 오차를 직선 거리로 보정한다
        return max(v - self.res * f, hypot(gx - x, gy - y))

    # ------------------------------------------------------------ 직결
    def _shot(self, x, y, th, gx, gy):
        """
        '한 번 꺾은 뒤 곧장 가기' 로 목적지에 닿는가.
        반환: (X, Y, TH, 길이, 호 길이) 또는 None.
        """
        R = self.r_turn
        best = None
        for side in (1, -1):                        # +1 왼쪽(반시계), -1 오른쪽
            cx = x - side * R * sin(th)
            cy = y + side * R * cos(th)
            dx, dy = gx - cx, gy - cy
            dist = hypot(dx, dy)
            if dist <= R + 1e-6:
                continue
            alpha = atan2(dy, dx)
            beta = acos(R / dist)
            phi_t = alpha - side * beta             # 떠나는 접점의 각도
            phi0 = atan2(y - cy, x - cx)
            sweep = (side * (phi_t - phi0)) % tau
            if sweep > tau - 1e-6:
                sweep = 0.0
            arc = R * sweep
            straight = sqrt(dist * dist - R * R)
            total = arc + straight
            if total > self.P.shot_max_len:
                continue
            if best is None or total < best[0]:
                best = (total, side, arc, straight, cx, cy, phi0)
        if best is None:
            return None
        total, side, arc, straight, cx, cy, phi0 = best
        ds = 0.10
        # 올림으로 나눠 간격이 0.1 m 를 넘지 않게 한다. 내림으로 나누면 짧은 호가
        # 점 하나로 찍혀 추종기가 보는 곡률이 실제보다 커진다.
        na = int(np.ceil(arc / ds - 1e-9))
        sa = np.linspace(0.0, arc, na + 1)[1:] if na > 0 else np.zeros(0)
        phi = phi0 + side * sa / R
        ax = cx + R * np.cos(phi)
        ay = cy + R * np.sin(phi)
        ath = phi + side * pi / 2
        th1 = th + side * arc / R
        x1 = cx + R * cos(phi0 + side * arc / R)
        y1 = cy + R * sin(phi0 + side * arc / R)
        nl = max(1, int(straight / ds))
        sl = np.linspace(0.0, straight, nl + 1)[1:]
        lx = x1 + cos(th1) * sl
        ly = y1 + sin(th1) * sl
        lth = np.full(nl, th1)
        X = np.concatenate([ax, lx])
        Y = np.concatenate([ay, ly])
        TH = np.concatenate([ath, lth])
        return X, Y, TH, total, arc

    # ------------------------------------------------------------ 탐색
    def plan(self, start, goal, grid, D, collect_tree=True):
        """
        start = (x, y, 방향), goal = (x, y).
        반환: HybridResult 또는 None.
        """
        P = self.P
        self.prepare(grid, D)
        gx, gy = float(goal[0]), float(goal[1])
        self.build_heuristic(goal)

        sx, sy, sth = float(start[0]), float(start[1]), float(start[2])
        h0 = self.h(sx, sy, gx, gy)
        if not np.isfinite(h0):
            return None

        c0 = float(self.clearance_poses(np.array([sx]), np.array([sy]),
                                        np.array([sth]))[0])
        # 노드: x, y, th, dir, kappa, g, parent, prim, req
        #   req = 이 노드의 자손이 지켜야 할 최소 여유.
        #   보통은 r_hard. 출발점이 이미 여유 안쪽에 들어와 있으면 그 값에서 시작해
        #   '여유가 줄지 않는 방향'으로만 움직이게 한다. 그래야 갇힌 자리에서도 빠져나온다.
        req0 = self.r_hard if c0 >= self.r_hard else c0
        nodes = [(sx, sy, sth, 0, 0.0, 0.0, -1, -1, req0)]
        openq = [(P.h_weight * h0, 0.0, 0)]
        best_g = {}
        closed = set()
        tree = []
        bins_th = P.theta_bins / tau
        expanded = 0
        goal_node, shot = None, None

        rel_dx, rel_dy, rel_dth = self.rel_dx, self.rel_dy, self.rel_dth
        prim_k, prim_d = self.prim_k, self.prim_d
        base_len = P.step * np.where(prim_d > 0, 1.0, P.rev_mult)
        steer_cost = P.w_steer * np.abs(prim_k) / self.kmax * P.step

        while openq and expanded < P.max_expand:
            f, g, nid = heapq.heappop(openq)
            x, y, th, d0, k0, g0, _, _, req = nodes[nid]
            key = (int((x - self.ox) / P.xy_res), int((y - self.oy) / P.xy_res),
                   int((th % tau) * bins_th), d0)
            if key in closed:
                continue
            closed.add(key)
            expanded += 1

            # ---- 직결 시도
            if expanded % P.shot_every == 1 or hypot(gx - x, gy - y) < 2.5:
                s = self._shot(x, y, th, gx, gy)
                if s is not None:
                    X, Y, TH, total, arc = s
                    m = self.clearance_poses(X, Y, TH)
                    if len(m) and float(m.min()) >= min(self.r_hard, req) - 1e-3:
                        pen = float(self._penalty(m[None, :], X[None, :],
                                                  Y[None, :], total)[0])
                        cost = g0 + total + P.w_steer * arc + pen \
                            + (P.switch_cost if d0 < 0 else 0.0)
                        goal_node, shot = nid, (X, Y, TH, cost)
                        break

            # ---- 조각 10개를 한꺼번에 굴린다
            c, s_ = cos(th), sin(th)
            X = x + c * rel_dx - s_ * rel_dy
            Y = y + s_ * rel_dx + c * rel_dy
            TH = th + rel_dth
            m = self.clearance_poses(X, Y, TH)                     # (P, K)
            mmin = m.min(axis=1)
            thr = self.r_hard if req >= self.r_hard else req - 0.02
            ok = mmin >= thr
            if not ok.any():
                continue
            pen = self._penalty(m, X, Y, P.step)
            dk = np.abs(prim_k - k0) / (2.0 * self.kmax)
            sw = np.where((prim_d != d0) & (d0 != 0), P.switch_cost, 0.0)
            gc = g0 + base_len + steer_cost + P.w_steer_change * dk + pen + sw

            for p in np.nonzero(ok)[0]:
                cx_, cy_, cth = float(X[p, -1]), float(Y[p, -1]), float(TH[p, -1])
                cg = float(gc[p])
                ck = (int((cx_ - self.ox) / P.xy_res), int((cy_ - self.oy) / P.xy_res),
                      int((cth % tau) * bins_th), int(prim_d[p]))
                if ck in closed or cg >= best_g.get(ck, np.inf):
                    continue
                hv = self.h(cx_, cy_, gx, gy)
                if not np.isfinite(hv):
                    continue
                best_g[ck] = cg
                creq = self.r_hard if mmin[p] >= self.r_hard else float(mmin[p])
                nodes.append((cx_, cy_, cth, int(prim_d[p]), float(prim_k[p]),
                              cg, nid, int(p), creq))
                cid = len(nodes) - 1
                if collect_tree and len(tree) < 6000:
                    tree.append(np.array([[x, y], [cx_, cy_]]))
                if hypot(gx - cx_, gy - cy_) <= P.goal_tol:
                    goal_node = cid
                    break
                heapq.heappush(openq, (cg + P.h_weight * hv, cg, cid))
            if goal_node is not None:
                break

        if goal_node is None:
            return None
        return self._reconstruct(nodes, goal_node, shot, expanded, tree)

    # ------------------------------------------------------------ 복원
    def _reconstruct(self, nodes, gid, shot, expanded, tree):
        chain = []
        n = gid
        while n >= 0:
            chain.append(n)
            n = nodes[n][6]
        chain.reverse()

        sx, sy, sth = nodes[chain[0]][:3]
        poses = [(sx, sy, sth)]
        dirs = [1]
        cum = [0.0]
        K = self.rel_dx.shape[1]
        for a, b in zip(chain[:-1], chain[1:]):
            x, y, th = nodes[a][:3]
            p = nodes[b][7]
            c, s = cos(th), sin(th)
            g_a, g_b = nodes[a][5], nodes[b][5]
            for k in range(K):
                dx, dy = self.rel_dx[p, k], self.rel_dy[p, k]
                poses.append((x + c * dx - s * dy, y + s * dx + c * dy,
                              th + self.rel_dth[p, k]))
                dirs.append(int(self.prim_d[p]))
                cum.append(g_a + (g_b - g_a) * (k + 1) / K)
        cost = nodes[gid][5]
        if shot is not None:
            X, Y, TH, cost = shot
            g_a = nodes[gid][5]
            for k in range(len(X)):
                poses.append((float(X[k]), float(Y[k]), float(TH[k])))
                dirs.append(1)
                cum.append(g_a + (cost - g_a) * (k + 1) / len(X))
        poses = np.array(poses)
        poses[:, 2] = (poses[:, 2] + pi) % tau - pi
        dirs = np.array(dirs)
        if len(dirs) > 1:
            dirs[0] = dirs[1]           # 첫 점은 첫 조각의 방향을 따른다
        return HybridResult(poses, dirs, np.array(cum), float(cost), expanded, tree)
