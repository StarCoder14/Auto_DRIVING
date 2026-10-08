"""
planner.py -- 접선 그래프 + A*   (문서 3장)

핵심 발상
  갈림길을 하나씩 따라가면 장애물 n 개에 경로가 2^n 개로 폭발한다.
  대신 '갈 수 있는 지점'을 전부 역(node)으로 놓고 지하철 노선도를 그린 뒤,
  최단 경로를 한 번에 찾는다.

역(node)
  출발점, 목적지, 그리고 모든 접점.
  접점은 위치뿐 아니라 '어느 원의 어느 회전 방향인지'까지 갖는다.
  같은 위치라도 회전 방향이 다르면 다른 역이다.
  이 정보 덕분에 접점을 잘못 조합해 원을 뚫는 경로가 아예 만들어지지 않는다.

노선(edge)  -- 전부 방향이 있다
  직선 : 생성된 접선 그 자체만. 모든 노드 쌍을 잇지 않는다.
         한 접선 선분은 진행 방향에 따라 회전 방향이 반대가 되므로
         서로 다른 두 개의 방향 엣지를 만든다.
  호   : 같은 원 / 같은 회전 방향에서 각도상 '이웃한' 노드끼리만.
         A->C 를 직접 잇지 않아도 A->B->C 로 같은 비용이 나오므로 결과는
         같고, 호 엣지 수가 O(n^2) 에서 O(n) 으로 줄어든다.

지연 충돌 검사 (lazy A*)
  엣지 수만 개를 전부 미리 충돌 검사하면 1초가 넘어간다.
  그래서 그래프를 만들 때는 '회전 방향 일관성'만 보고(거의 공짜),
  충돌 검사와 정확한 비용은 A* 가 그 엣지를 실제로 꺼낼 때 계산한다.
  대부분의 엣지는 끝까지 검사되지 않으므로 20~50배 빨라진다.
  하한 비용(길이)만으로 먼저 밀어 넣고, 꺼낼 때 실제 비용으로 고쳐
  다시 넣기 때문에 결과는 전수 검사와 같다.

비용 (문서 3.4)
  거리만 쓰면 경로가 장애물에 딱 붙는다(붙을수록 짧으므로).
  그래서 길이 + 통로 여유 + 꺾임 + 미지영역을 함께 따진다.

  여유를 잴 때는 '지금 돌고 있는 그 장애물'은 제외한다.
  호는 정의상 부풀린 경계에 붙어 있고(그 여유가 곧 rho 다), 실제로 위험한
  것은 옆에 있는 다른 장애물까지의 거리, 즉 통로 폭이기 때문이다.
"""

import heapq
from math import atan2, cos, hypot, sin, tau
import numpy as np

from . import geometry as G

STRAIGHT, ARC = 0, 1


class PlannerParams:
    def __init__(self,
                 w_clear=0.9,       # 좁은 통로에 주는 벌점의 크기
                 clear_ref=0.9,     # 이 정도 여유가 있으면 벌점 없음 (m)
                 clear_cap=12.0,    # 벌점 배율 상한
                 w_turn=0.30,       # 구간이 바뀔 때마다(핸들 꺾임) 벌점
                 w_unknown=0.30,    # 아직 안 가본 영역을 지나는 구간에 벌점
                 w_back=120.0,      # 출발점에서 뒤로 꺾여 나가는 첫 구간의 벌점
                                    # 12~60 은 어중간해서 오히려 나쁘다.
                                    # 포기시킬 만큼 세지 않으면서 계획만 왜곡한다.
                 max_circles=24,    # 계획에 쓰는 최대 원 개수 (계산 시간 상한)
                 plan_radius=10.0,  # 이 반경 밖 장애물은 계획에서 제외 (m)
                 samples=3,         # 엣지당 여유 계산 샘플 수
                 tol=2e-3,
                 max_expand=60000):
        self.w_clear = w_clear
        self.clear_ref = clear_ref
        self.clear_cap = clear_cap
        self.w_turn = w_turn
        self.w_unknown = w_unknown
        self.w_back = w_back
        self.max_circles = max_circles
        self.plan_radius = plan_radius
        self.samples = samples
        self.tol = tol
        self.max_expand = max_expand


class TangentGraph:
    """접선 그래프. 매 계획 주기마다 새로 만든다."""

    def __init__(self, start, goal, groups, rho, params: PlannerParams,
                 grid=None, soft_gids=(), lock=None, heading=None):
        self.start = np.asarray(start, float)
        self.goal = np.asarray(goal, float)
        self.rho = float(rho)
        self.P = params
        self.grid = grid
        self.groups = groups

        # 회전 방향 잠금 (문서 3.5 보강)
        #
        # 한 장애물을 왼쪽으로 돌기로 정했으면, 그 결정을 무를 때까지 오른쪽으로
        # 도는 선택지를 '비싸게' 만드는 게 아니라 아예 만들지 않는다. 값을 매기는
        # 방식은 상황이 조금만 바뀌어도 뒤집히는데, 뒤집히는 순간이 하필 호 위라서
        # 위험하다. 호는 부풀린 경계에 붙어 있어 여유가 없기 때문이다.
        #
        # lock = (gid, sigma) 또는 None.
        # 해당 장애물의 접점 중 sigma 와 반대 방향인 것은 노드로 만들지 않는다.
        # 노드가 없으면 그 노드를 지나는 직선도 호도 저절로 생기지 않는다.
        #
        # 잠금을 푸는 책임은 호출측(simulator)에 있다. 여기서는 시키는 대로만 한다.
        self.lock_gid = -1 if lock is None else int(lock[0])
        self.lock_sig = 0 if lock is None else int(lock[1])

        # 출발 방향 제약 (문서 3장 보강)
        #
        # 계획기는 출발점을 '점 하나'로만 본다. 그래서 차 뒤로 가는 경로가 더
        # 짧으면 그것을 고른다. 차가 제자리에서 돌 수 있다고 착각하는 셈이다.
        # 실제 차는 자전거 모델이라 최소 회전 반경이 있어 그럴 수 없다.
        #
        # 그 결과가 '전방주시점이 차 뒤에 놓이는' 상황이다. 추종기는 핸들을
        # 끝까지 꺾고, 그 순간이 장애물에 가장 가까이 붙는 순간이 된다.
        #
        # heading 을 주면 출발점에서 그 방향과 back_deg 이상 어긋나게 나가는
        # 첫 구간은 아예 만들지 않는다. 물리적으로 못 하는 경로이므로 없애도
        # 잃는 것이 없다. (반대쪽으로 도는 우회로를 막는 것과는 다르다.
        #  그쪽은 멀쩡한 선택지라 막으면 손해가 난다.)
        self.heading = None if heading is None else float(heading)
        # 출발점이 이미 안에 들어와 있는 그룹.
        # 출발점에서 나가는 엣지에 한해 이 그룹의 충돌 검사를 면제한다.
        # 그렇지 않으면 '이미 안에 있어서 아무 데도 못 간다'가 되어 버린다.
        self.soft_gids = set(soft_gids)

        self._select_circles(groups)
        self.pos = [self.start, self.goal]
        # 내부 루프는 numpy 대신 순수 스칼라로 돈다 (수만 번 호출되므로)
        self.px = [float(self.start[0]), float(self.goal[0])]
        self.py = [float(self.start[1]), float(self.goal[1])]
        self.cir = [-1, -1]
        self.sig = [0, 0]
        self.Cx = self.C[:, 0].tolist() if len(self.C) else []
        self.Cy = self.C[:, 1].tolist() if len(self.C) else []
        self.adj = {}            # u -> [(v, lower_bound, edge_index)]
        self.edges = []          # (kind, u, v, ci, sigma, exclude, length)
        self._cache = {}
        self._eval = {}          # edge_index -> (ok, true_cost)
        self.path_edges = []
        self._build()

    # ------------------------------------------------------------ 원 선별
    def _select_circles(self, groups):
        cs, owner = [], []
        for g in groups:
            for v in g.circles:
                cs.append(v)
                owner.append(g.gid)
        if not cs:
            self.C = np.zeros((0, 2))
            self.owner = np.zeros(0, dtype=int)
            return
        C = np.array(cs)
        # 지금 있는 자리 주변만 본다. 멀리 있는 장애물은 다가가면서 다시 본다.
        # ("보이는 데까지 계획해서 조금 가고, 또 계획한다")
        d = np.hypot(*(C - self.start).T)
        keep = np.where(d < self.P.plan_radius)[0]
        if len(keep) > self.P.max_circles:
            keep = keep[np.argsort(d[keep])[:self.P.max_circles]]
        self.C = C[keep]
        self.owner = np.array(owner, dtype=int)[keep]

    # ------------------------------------------------------------ 노드
    def _allowed(self, ci, sigma):
        """잠긴 장애물을 반대 방향으로 도는 접점인가? 그렇다면 만들지 않는다."""
        if self.lock_gid < 0:
            return True
        if int(self.owner[ci]) != self.lock_gid:
            return True
        return int(sigma) == self.lock_sig

    def _back_cost(self, u, v):
        """출발점에서 뒤로 꺾여 나가는 첫 구간에 매기는 벌점.

        금지가 아니라 벌점인 이유가 있다. 막다른 골목에서는 돌아 나오는 것이
        유일한 정답이라, 아예 막으면 빠져나올 수 없다. 비싸게 만들어 두면
        평소에는 고르지 않다가 그것밖에 없을 때만 고른다.

        정면(코사인 >= 0)은 0. 뒤로 갈수록 커져 정반대에서 w_back * rho 가 된다.
        """
        if self.heading is None or u != 0:
            return 0.0
        dx, dy = self.px[v] - self.px[0], self.py[v] - self.py[0]
        n = hypot(dx, dy)
        if n < 1e-9:
            return 0.0
        c = (cos(self.heading) * dx + sin(self.heading) * dy) / n
        if c >= 0.0:
            return 0.0
        return self.P.w_back * self.rho * (-c)

    def _node(self, tx, ty, ci, sigma):
        key = (ci, sigma, int(tx * 1e5), int(ty * 1e5))
        idx = self._cache.get(key)
        if idx is None:
            idx = len(self.pos)
            self.pos.append(np.array([tx, ty]))
            self.px.append(tx)
            self.py.append(ty)
            self.cir.append(ci)
            self.sig.append(sigma)
            self._cache[key] = idx
        return idx

    @staticmethod
    def _sigma_of(tx, ty, cx, cy, dx, dy):
        """
        접점을 지나는 진행 방향 (dx,dy) 에 부합하는 회전 방향.
        반시계 접선 방향은 perp(radial) 이므로 부호만 보면 된다.
        """
        s = (tx - cx) * dy - (ty - cy) * dx
        if -1e-12 < s < 1e-12:
            return 0
        return 1 if s > 0 else -1

    # ------------------------------------------------------------ 구성
    def _build(self):
        r = self.rho
        sx, sy = self.px[0], self.py[0]
        gx, gy = self.px[1], self.py[1]

        # (a) 출발점 -> 목적지 직행
        self._straight(0, 1, ())

        # (b) 출발점 -> 원,  원 -> 목적지
        for ci in range(len(self.C)):
            cx, cy = self.Cx[ci], self.Cy[ci]
            gid = int(self.owner[ci])
            for T, _ in G.tangent_points_from_point((sx, sy), (cx, cy), r):
                tx, ty = float(T[0]), float(T[1])
                sg = self._sigma_of(tx, ty, cx, cy, tx - sx, ty - sy)
                if sg and self._allowed(ci, sg):
                    self._straight(0, self._node(tx, ty, ci, sg), (gid,))
            for T, _ in G.tangent_points_from_point((gx, gy), (cx, cy), r):
                tx, ty = float(T[0]), float(T[1])
                sg = self._sigma_of(tx, ty, cx, cy, gx - tx, gy - ty)
                if sg and self._allowed(ci, sg):
                    self._straight(self._node(tx, ty, ci, sg), 1, (gid,))

        # (c) 원 -> 원 (양방향, 회전 방향은 서로 반대로 붙는다)
        nC = len(self.C)
        for i in range(nC):
            ix, iy = self.Cx[i], self.Cy[i]
            gi = int(self.owner[i])
            for j in range(i + 1, nC):
                jx, jy = self.Cx[j], self.Cy[j]
                gj = int(self.owner[j])
                for T1, T2 in G.bitangents_equal_radius((ix, iy), (jx, jy), r):
                    ax, ay = float(T1[0]), float(T1[1])
                    bx, by = float(T2[0]), float(T2[1])
                    dx, dy = bx - ax, by - ay
                    if dx * dx + dy * dy < 1e-18:
                        continue
                    s1 = self._sigma_of(ax, ay, ix, iy, dx, dy)
                    s2 = self._sigma_of(bx, by, jx, jy, dx, dy)
                    if s1 == 0 or s2 == 0:
                        continue
                    if self._allowed(i, s1) and self._allowed(j, s2):
                        self._straight(self._node(ax, ay, i, s1),
                                       self._node(bx, by, j, s2), (gi, gj))
                    if self._allowed(j, -s2) and self._allowed(i, -s1):
                        self._straight(self._node(bx, by, j, -s2),
                                       self._node(ax, ay, i, -s1), (gi, gj))

        # (d) 호
        self._arcs()

    def _straight(self, u, v, exclude):
        px, py, cir, sig = self.px, self.py, self.cir, self.sig
        dx = px[v] - px[u]
        dy = py[v] - py[u]
        L = hypot(dx, dy)
        if L < 1e-9:
            return
        # 회전 방향 일관성. 이 검사 하나가
        # '접점 4개 중 유효한 조합은 2개뿐' 문제를 통째로 해결한다.
        for n in (u, v):
            ci = cir[n]
            if ci >= 0:
                if sig[n] * ((px[n] - self.Cx[ci]) * dy
                             - (py[n] - self.Cy[ci]) * dx) <= 0.0:
                    return
        self._edge(STRAIGHT, u, v, -1, 0, exclude, L)

    def _arcs(self):
        by = {}
        for n in range(2, len(self.pos)):
            by.setdefault((self.cir[n], self.sig[n]), []).append(n)
        px, py = self.px, self.py
        for (ci, sgm), nodes in by.items():
            if len(nodes) < 2:
                continue
            cx, cy = self.Cx[ci], self.Cy[ci]
            gid = int(self.owner[ci])
            th = [atan2(py[n] - cy, px[n] - cx) for n in nodes]
            seq = [nodes[k] for k in
                   sorted(range(len(nodes)), key=lambda k: sgm * th[k])]
            ths = dict(zip(nodes, th))
            m = len(seq)
            for k in range(m):
                u, v = seq[k], seq[(k + 1) % m]
                if u == v:
                    continue
                sweep = (sgm * (ths[v] - ths[u])) % tau
                self._edge(ARC, u, v, ci, sgm, (gid,), self.rho * sweep)

    def _edge(self, kind, u, v, ci, sigma, exclude, length):
        idx = len(self.edges)
        self.edges.append((kind, u, v, ci, sigma, exclude, length))
        lb = length + self.P.w_turn + self._back_cost(u, v)
        self.adj.setdefault(u, []).append((v, lb, idx))

    # ------------------------------------------------------------ 엣지 평가
    def edge_points(self, idx, max_step=0.15):
        kind, u, v, ci, sgm, _, _ = self.edges[idx]
        if kind == ARC:
            return G.arc_points(self.C[ci], self.rho,
                                self.pos[u], self.pos[v], sgm, max_step)
        p0, p1 = self.pos[u], self.pos[v]
        n = max(2, int(np.hypot(*(p1 - p0)) / max_step) + 1)
        return p0 + np.outer(np.linspace(0.0, 1.0, n), (p1 - p0))

    def evaluate(self, idx):
        """엣지의 충돌 여부와 실제 비용. 한 번만 계산하고 캐시한다."""
        hit = self._eval.get(idx)
        if hit is not None:
            return hit
        kind, u, v, ci, sgm, excl, L = self.edges[idx]
        tol = self.P.tol
        gs = [g for g in self.groups if g.gid not in self.soft_gids] \
            if (u == 0 and self.soft_gids) else self.groups

        if kind == STRAIGHT:
            a, b = self.pos[u], self.pos[v]
            ok = not any(g.blocks_segment(a, b, tol) for g in gs)
            pts = np.stack([a, b])
        else:
            pts = self.edge_points(idx)
            # 호도 반드시 충돌 검사한다. (직선만 검사하는 실수가 흔하다)
            # 자기 그룹도 대상이다. 꼭짓점 원의 '안쪽' 호는 도형 내부를
            # 지나므로 여기서 걸러진다.
            ok = not any(g.blocks_points(pts, tol) for g in gs)

        res = (ok, self._cost(pts, L, excl) + self._back_cost(u, v)
               if ok else np.inf)
        self._eval[idx] = res
        return res

    def _cost(self, pts, length, exclude):
        P = self.P
        cost = length + P.w_turn
        k = max(1, P.samples)
        ds = length / k
        ts = (np.arange(k) + 0.5) / k          # 끝점(=접점, 여유 0)은 피한다
        sample = pts[np.clip((ts * (len(pts) - 1)).astype(int),
                             0, len(pts) - 1)]
        clear = np.full(len(sample), 12.0)
        for g in self.groups:
            if g.gid in exclude:
                continue
            clear = np.minimum(clear, g.distance_many(sample))
        clear = np.maximum(clear, 1e-2)
        ratio = np.clip((P.clear_ref / clear) ** 2 - 1.0, 0.0, P.clear_cap)
        cost += P.w_clear * ds * float(ratio.sum())
        if self.grid is not None:
            cost += P.w_unknown * ds * float(self.grid.count_unknown(sample))
        return cost

    # ------------------------------------------------------------ 탐색
    def search(self):
        """
        지연 평가 A*.
        꺼낼 때 그 엣지를 검사하고, 실제 비용이 하한보다 크면 다시 넣는다.
        """
        gp = self.goal
        h = lambda n: float(np.hypot(*(self.pos[n] - gp)))
        cnt = 0
        openq = [(h(0), 0.0, cnt, 0, -1, -1)]
        closed = {}
        came = {}
        expanded = 0

        while openq:
            _, g0, _, u, parent, eidx = heapq.heappop(openq)
            if u in closed:
                continue
            if parent >= 0:
                ok, true_cost = self.evaluate(eidx)
                if not ok:
                    continue
                g_true = closed[parent] + true_cost
                if g_true > g0 + 1e-9:          # 하한보다 비쌌다 -> 재삽입
                    cnt += 1
                    heapq.heappush(openq,
                                   (g_true + h(u), g_true, cnt, u, parent, eidx))
                    continue
                g0 = g_true
            closed[u] = g0
            came[u] = (parent, eidx)
            expanded += 1
            if u == 1:
                return self._trace(came, 1), g0
            if expanded > self.P.max_expand:
                break
            for v, lb, idx in self.adj.get(u, ()):
                if v in closed:
                    continue
                ng = g0 + lb
                cnt += 1
                heapq.heappush(openq, (ng + h(v), ng, cnt, v, u, idx))
        return None, np.inf

    def _trace(self, came, n):
        nodes, eids = [n], []
        while came[n][0] >= 0:
            eids.append(came[n][1])
            n = came[n][0]
            nodes.append(n)
        self.path_edges = eids[::-1]
        return nodes[::-1]

    # ------------------------------------------------------------ 경로 복원
    def polyline(self, node_path, max_step=0.10):
        """노드 경로를 촘촘한 폴리라인으로. 추종기가 쓰는 형식."""
        if node_path is None or len(node_path) < 2:
            return None
        pts = [self.pos[node_path[0]][None, :]]
        for idx in self.path_edges:
            seg = self.edge_points(idx, max_step)
            pts.append(seg[1:])
        return np.vstack(pts)

    def first_arc_lock(self):
        """고른 경로가 '처음으로 감아 도는' 장애물과 그 회전 방향.

        반환: (gid, sigma) 또는 None(도는 장애물이 없는 직선 경로).
        이것이 다음 주기에 잠글 대상이 된다.
        """
        for idx in self.path_edges:
            kind, u, v, ci, sgm, excl, L = self.edges[idx]
            if kind == ARC:
                return (int(self.owner[ci]), int(sgm))
        return None

    def edges_for_draw(self, limit=1200):
        """접선 그래프 시각화용 폴리라인 목록 (검사 통과한 것만)."""
        out = []
        for idx in range(len(self.edges)):
            ok = self._eval.get(idx)
            if ok is None or not ok[0]:
                continue
            out.append(self.edge_points(idx, 0.4))
            if len(out) >= limit:
                break
        return out


def plan(start, goal, groups, rho, params, grid=None, soft_gids=(), lock=None,
         heading=None):
    """한 번의 전역 계획. 반환: (폴리라인, 비용, 그래프).

    lock = (gid, sigma) 를 주면 그 장애물을 반대로 도는 경로는 아예 만들지 않는다.
    """
    gr = TangentGraph(start, goal, groups, rho, params, grid, soft_gids, lock,
                      heading)
    node_path, cost = gr.search()
    return gr.polyline(node_path), cost, gr
