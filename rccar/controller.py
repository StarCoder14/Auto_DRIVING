"""
controller.py -- 경로 추종 (문서 6장)

찾아낸 경로에는 문제가 있다. 직선 구간에서 호 구간으로 넘어가는 순간
핸들이 중립에서 꺾인 상태로 순간이동해야 한다. 물리적으로 불가능하다.

그래서 경로를 정확히 밟으려 하지 않는다.

  경로 위에서 내 앞 1미터쯤 되는 지점을 하나 찍는다.
  그 지점을 향해 부드러운 곡선으로 간다.
  차가 나아가면 그 지점도 앞으로 밀린다. 계속 쫓아간다.

말 앞에 당근을 매달아 놓은 것과 같다. 차가 항상 저 앞의 한 점만 보고
있으므로 경로의 각진 부분이 저절로 뭉개진다.

대신 코너를 살짝 안쪽으로 잘라먹는다. 그만큼의 여유를 장애물을 부풀릴 때
미리 넣어 두었다(문서 2.1). 여기서 두 부분이 연결된다.
"""

import numpy as np
from . import geometry as G


class ControllerParams:
    def __init__(self,
                 k_lookahead=1.05,   # 전방주시거리 = k * 속도
                 ld_min=0.55,        # 전방주시거리 하한 (m)
                 ld_max=1.10,        # 상한 (m)
                 k_clear=1.30,       # 앞이 트인 정도에 비례한 속도
                 goal_tol=0.45,      # 도착 판정 반경 (m)
                 align_thresh=np.radians(75.0),  # 이보다 어긋나면 제자리 정렬 필요
                 reverse_speed=0.35):
        self.k_lookahead = k_lookahead
        self.ld_min = ld_min
        self.ld_max = ld_max
        self.k_clear = k_clear
        self.goal_tol = goal_tol
        self.align_thresh = align_thresh
        self.reverse_speed = reverse_speed


class PurePursuit:
    def __init__(self, params: ControllerParams, veh_params):
        self.P = params
        self.V = veh_params
        self.lookahead_pt = None
        self.path_error = 0.0

    def lookahead_distance(self, v):
        return float(np.clip(self.P.k_lookahead * v, self.P.ld_min, self.P.ld_max))

    # ------------------------------------------------------------ 전방점
    def find_target(self, path, pos, Ld):
        """
        경로 위에서 현재 위치로부터 Ld 만큼 앞선 점.
        가장 가까운 점부터 앞으로 훑어 거리가 Ld 를 넘는 첫 점을 쓴다.
        """
        d = np.hypot(*(path - pos).T)
        i0 = int(np.argmin(d))
        self.path_error = float(d[i0])
        for i in range(i0, len(path)):
            if d[i] >= Ld:
                return path[i], i
        return path[-1], len(path) - 1

    # ------------------------------------------------------------ 명령
    def compute(self, path, pose, v, clearance_ahead, goal, curvature_ahead=None):
        """
        반환: (목표속도, 목표조향, 정보 dict)
        """
        P, V = self.P, self.V
        pos, th = pose[:2], pose[2]
        info = {"mode": "track"}

        if path is None or len(path) < 2:
            return 0.0, 0.0, {"mode": "no_path"}

        # 전방주시거리: 속도에 비례하되, 코너가 급하면 줄인다.
        # 크게 잡으면 부드럽지만 코너를 많이 잘라먹는다.
        Ld = self.lookahead_distance(max(v, 0.4))
        if curvature_ahead is not None and curvature_ahead > 1e-3:
            Ld = min(Ld, max(self.P.ld_min, 0.70 / curvature_ahead))
        target, idx = self.find_target(path, pos, Ld)
        self.lookahead_pt = target

        dv = target - pos
        eta = G.wrap_pi(np.arctan2(dv[1], dv[0]) - th)
        dist = float(np.hypot(*dv))

        # 처음 방향이 크게 어긋나면 전진만으로는 못 맞춘다 (문서 6.3)
        if abs(eta) > P.align_thresh and v < 0.25:
            info["mode"] = "align"
            return P.reverse_speed, np.sign(eta) * V.max_steer, info

        # 순수 추종 조향
        steer = np.arctan2(2.0 * V.wheelbase * np.sin(eta), max(dist, 1e-3))
        steer = float(np.clip(steer, -V.max_steer, V.max_steer))

        # ---- 속도: 세 가지 중 가장 작은 값 (문서 6.2)
        v_cmd = V.max_speed
        v_cmd = min(v_cmd, P.k_clear * max(clearance_ahead, 0.05))

        kappa = abs(np.tan(steer) / V.wheelbase)
        if curvature_ahead is not None:
            kappa = max(kappa, curvature_ahead)
        if kappa > 1e-3:
            # 코너를 '들어가서' 줄이면 늦다. 앞으로 몇 미터 안에 나올 가장 급한
            # 굽이를 미리 보고 지금부터 줄인다.
            v_cmd = min(v_cmd, np.sqrt(V.max_lat_acc / kappa))

        # 목적지 근처에서 감속
        d_goal = float(np.hypot(*(goal - pos)))
        v_cmd = min(v_cmd, max(0.25, 1.4 * d_goal))

        info.update(lookahead=Ld, eta=eta, kappa=kappa,
                    path_error=self.path_error, target_idx=idx)
        return float(max(v_cmd, 0.0)), steer, info


def path_curvature_ahead(path, pos, horizon=4.0):
    """
    앞으로 horizon 미터 구간의 최대 곡률. 코너 전에 미리 감속하기 위한 값.
    세 점을 지나는 원의 곡률(멩거 곡률)로 근사한다.
    """
    if path is None or len(path) < 3:
        return 0.0
    d = np.hypot(*(path - pos).T)
    i0 = int(np.argmin(d))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(path, axis=0).T))])
    sel = np.where((s >= s[i0]) & (s <= s[i0] + horizon))[0]
    if len(sel) < 3:
        return 0.0
    p = path[sel]
    a, b, c = p[:-2], p[1:-1], p[2:]
    ab = np.hypot(*(b - a).T)
    bc = np.hypot(*(c - b).T)
    ca = np.hypot(*(c - a).T)
    area = np.abs((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1])
                  - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])) * 0.5
    den = ab * bc * ca
    k = np.where(den > 1e-9, 4.0 * area / np.maximum(den, 1e-9), 0.0)
    return float(np.max(k)) if len(k) else 0.0
