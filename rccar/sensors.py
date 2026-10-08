"""
sensors.py -- 싼 거리 센서 흉내  (360도 라이다를 살 수 없을 때)

실제로 살 수 있는 센서는 라이다와 성질이 다르다. 시뮬레이터가 그 차이를 흉내 내지
않으면, 시뮬레이션에서 되던 것이 실제 차에서 안 된다.

흉내 내는 성질
  원뿔       : 센서는 가는 광선이 아니라 원뿔 안을 본다. 원뿔 안의 가장 가까운 물체
               거리 하나만 돌려준다. 그 물체가 원뿔의 어느 방향에 있는지는 모른다.
  장착 위치  : 센서는 뒷바퀴 축이 아니라 앞 범퍼에 달린다. 차 옆과 뒤는 안 보인다.
  측정 주기  : 초음파는 여러 개를 동시에 쏘면 서로 간섭해서 한 번에 몇 개씩만 잰다.
               서보로 돌리는 센서는 한 번에 몇 방향씩만 잰다.
  반사       : 초음파는 비스듬한 면에 맞으면 소리가 옆으로 튀어 돌아오지 않는다.
  못 맞힘    : ToF 와 초음파는 '앞에 아무것도 없음'을 따로 알려준다. 라이다처럼
               반사 누락과 헷갈리지 않으므로, 최대 거리까지 비었다고 기록할 수 있다.

지도에 기록하는 방식 (실제 차에서 그대로 쓸 수 있는 단순한 방식)
  비었음 : 원뿔 안에서 잰 거리보다 가까운 곳은 전부 비어 있다. 가장 가까운 물체까지의
           거리를 쟀으니 확실하다.
  막혔음 : 원뿔 가운데 방향, 잰 거리의 점 하나. 실제 물체는 원뿔 옆쪽에 있을 수 있어서
           원뿔이 넓을수록 장애물이 엉뚱한 방향에 찍힌다. 이 오차도 실험에 그대로 드러난다.

센서 사양 값은 제품 설명서의 대략적인 값을 쓴 가정이다. 실제 부품을 받으면 고친다.
"""

from math import cos, radians, sin

import numpy as np


# ============================================================== 광선 계산
def raycast_world(world, origins, dirs, max_range):
    """
    여러 광선을 한꺼번에 세계에 쏜다.
    반환: (거리 (R,), 맞은 면의 법선 (R, 2)). 못 맞히면 거리는 inf.
    world.raycast 와 같은 결과를 numpy 로 한꺼번에 낸다.
    """
    O = np.asarray(origins, float).reshape(-1, 2)
    D = np.asarray(dirs, float).reshape(-1, 2)
    R = len(O)
    best = np.full(R, np.inf)
    normal = np.zeros((R, 2))

    if world.circles:
        C = np.array([c for c, _ in world.circles])
        r = np.array([rr for _, rr in world.circles])
        oc = O[:, None, :] - C[None, :, :]
        b = np.einsum("rkd,rd->rk", oc, D)
        cc = np.einsum("rkd,rkd->rk", oc, oc) - r * r
        disc = b * b - cc
        s = np.sqrt(np.maximum(disc, 0.0))
        t1, t2 = -b - s, -b + s
        t = np.where(t1 > 1e-9, t1, np.where(t2 > 1e-9, t2, np.inf))
        t = np.where(disc < 0.0, np.inf, t)
        k = np.argmin(t, axis=1)
        tk = t[np.arange(R), k]
        better = tk < best
        if better.any():
            hit = O[better] + tk[better, None] * D[better]
            normal[better] = (hit - C[k[better]]) / r[k[better], None]
            best[better] = tk[better]

    if world.walls:
        A = np.array([a for a, _ in world.walls])
        B = np.array([b for _, b in world.walls])
        ab = B - A
        den = -D[:, 0:1] * ab[None, :, 1] + D[:, 1:2] * ab[None, :, 0]
        rhs = A[None, :, :] - O[:, None, :]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (-rhs[..., 0] * ab[None, :, 1] + rhs[..., 1] * ab[None, :, 0]) / den
            u = (D[:, 0:1] * rhs[..., 1] - D[:, 1:2] * rhs[..., 0]) / den
        ok = (np.abs(den) > 1e-12) & (t > 1e-9) & (u >= -1e-9) & (u <= 1.0 + 1e-9)
        t = np.where(ok, t, np.inf)
        k = np.argmin(t, axis=1)
        tk = t[np.arange(R), k]
        better = tk < best
        if better.any():
            n = np.stack([-ab[:, 1], ab[:, 0]], axis=1)
            n = n / np.hypot(n[:, 0], n[:, 1])[:, None]
            normal[better] = n[k[better]]
            best[better] = tk[better]

    best[best > max_range] = np.inf
    return best, normal


# ============================================================== 센서 묶음
class Measurement:
    def __init__(self, idx, mounts, angles, ranges):
        self.idx = idx            # (M,) 이번에 잰 센서 방향 번호
        self.mounts = mounts      # (M, 2) 차체 좌표계 장착 위치 (뒷바퀴 축 기준)
        self.angles = angles      # (M,) 차체 좌표계 방향
        self.ranges = ranges      # (M,) 센서에서 물체까지. 못 맞히면 inf


class RangeSensorArray:
    """
    같은 종류 센서 여러 방향. 서보로 돌리는 센서는 '방향 여러 개를 가진 센서 하나'로 표현한다.

    mounts, angles : 방향마다 장착 위치와 바라보는 각도 (차체 좌표계)
    cone           : 원뿔 전체 각도 (라디안). 0 이면 가는 광선
    per_cycle      : 계획 주기 한 번에 잴 수 있는 방향 수. None 이면 전부
    sweep          : True 면 방향을 왕복하며 차례로 잰다 (서보). False 면 돌아가며 잰다
    specular_deg   : 이 각도보다 비스듬한 면은 반사가 안 돌아온다. None 이면 무시
    reports_empty  : '아무것도 없음'을 따로 알려주는가
    """

    def __init__(self, name, mounts, angles, max_range, cone=0.0, subrays=1,
                 per_cycle=None, sweep=False, noise_abs=0.02, noise_rel=0.0,
                 dropout=0.0, specular_deg=None, reports_empty=True, rng=None,
                 map_arc=False, arc_near=0.8, arc_far_weight=0.43):
        self.name = name
        self.mounts = np.asarray(mounts, float).reshape(-1, 2)
        self.angles = np.asarray(angles, float).reshape(-1)
        self.max_range = float(max_range)
        self.cone = float(cone)
        self.subrays = int(subrays) if cone > 0 else 1
        self.per_cycle = per_cycle
        self.sweep = sweep
        self.noise_abs = noise_abs
        self.noise_rel = noise_rel
        self.dropout = dropout
        self.specular_cos = None if specular_deg is None else cos(radians(specular_deg))
        self.reports_empty = reports_empty
        # 막힌 곳을 원뿔 끝 호 전체에 찍을지 (occupied_for_map 참조)
        self.map_arc = bool(map_arc and cone > 0)
        self.arc_near = arc_near
        self.arc_far_weight = arc_far_weight
        self.rng = rng or np.random.default_rng(0)
        self._next = 0
        self._step = 1
        # 방향마다 원뿔 각도와 최대 거리 (그리기용. 묶음에서도 같은 이름으로 쓴다)
        self.cones = np.full(len(self.angles), self.cone)
        self.max_ranges = np.full(len(self.angles), self.max_range)
        # 호환용 (simulator.py 가 기대하는 이름)
        self.n_beams = len(self.angles)
        self.fov = float(np.ptp(self.angles)) + self.cone
        self.dphi = self.fov / max(self.n_beams, 1)

    # ------------------------------------------------------------ 순서
    def _schedule(self):
        n = len(self.angles)
        k = n if self.per_cycle is None else min(self.per_cycle, n)
        if k == n:
            return np.arange(n)
        out = []
        for _ in range(k):
            out.append(self._next)
            if self.sweep and n > 1:
                nxt = self._next + self._step
                if nxt < 0 or nxt >= n:
                    self._step = -self._step
                    nxt = self._next + self._step
                self._next = nxt
            else:
                self._next = (self._next + 1) % n
        return np.array(out)

    def cone_offsets(self):
        if self.subrays <= 1:
            return np.zeros(1)
        return np.linspace(-self.cone / 2, self.cone / 2, self.subrays)

    # ------------------------------------------------------------ 측정
    def measure(self, world, pose):
        """pose 는 실제 세계 좌표의 차 자세. 반환: Measurement."""
        idx = self._schedule()
        x, y, th = pose
        c, s = cos(th), sin(th)
        mounts = self.mounts[idx]
        angles = self.angles[idx]
        offs = self.cone_offsets()
        M, S = len(idx), len(offs)

        wm = np.stack([x + c * mounts[:, 0] - s * mounts[:, 1],
                       y + s * mounts[:, 0] + c * mounts[:, 1]], axis=1)
        ga = th + angles[:, None] + offs[None, :]                 # (M, S)
        dirs = np.stack([np.cos(ga), np.sin(ga)], axis=-1)        # (M, S, 2)
        origins = np.repeat(wm, S, axis=0)
        t, n = raycast_world(world, origins, dirs.reshape(-1, 2), self.max_range)
        t = t.reshape(M, S)
        if self.specular_cos is not None:
            inc = np.abs(np.sum(dirs.reshape(-1, 2) * n, axis=1)).reshape(M, S)
            t = np.where(inc >= self.specular_cos, t, np.inf)
        r = t.min(axis=1)

        hit = np.isfinite(r)
        noise = self.rng.normal(0.0, 1.0, M) * (self.noise_abs + self.noise_rel * np.where(hit, r, 0.0))
        r = np.where(hit, np.maximum(0.02, r + noise), np.inf)
        r[r > self.max_range] = np.inf
        if self.dropout > 0:
            r[self.rng.random(M) < self.dropout] = np.inf
        return Measurement(idx, mounts, angles, r)

    # ------------------------------------------------------------ 지도용 광선
    def rays_for_map(self, m, res):
        """
        반환: (비었음 광선 시작점, 끝점), (막혔음 점의 시작점, 끝점). 모두 차체 좌표계.
        """
        offs = self.cone_offsets()
        free_o, free_e, occ_o, occ_e = [], [], [], []
        for mount, a, r in zip(m.mounts, m.angles, m.ranges):
            if np.isfinite(r):
                L = r - 0.5 * res
            elif self.reports_empty:
                L = self.max_range
            else:
                continue
            if L > 0.05:
                for o in offs:
                    free_o.append(mount)
                    free_e.append(mount + L * np.array([cos(a + o), sin(a + o)]))
            if np.isfinite(r):
                occ_o.append(mount)
                occ_e.append(mount + r * np.array([cos(a), sin(a)]))
        z = np.zeros((0, 2))
        return ((np.array(free_o) if free_o else z, np.array(free_e) if free_e else z),
                (np.array(occ_o) if occ_o else z, np.array(occ_e) if occ_e else z))

    def occupied_for_map(self, m, res):
        """
        막힌 곳 기록. 반환: (시작점, 끝점, 무게, 측정 번호). 무게 1 은 한 번에 막힌 칸이 되는 세기.

        map_arc 가 꺼져 있으면 원뿔 가운데 점 하나 (rays_for_map 과 같다).
        켜져 있으면 원뿔 끝 호 전체에 찍는다.
          가까이 (arc_near 안) : 무게 1. 호가 짧아 과하게 막는 부작용이 작고, 코앞의 가는
                                 기둥을 놓치는 것이 훨씬 위험하다.
          멀리                 : 약한 무게. 한 번으로는 막힌 칸이 안 되고, 여러 방향에서 본
                                 원뿔이 겹치는 곳만 막힌 칸이 된다. 잘못 찍힌 부분은 다른
                                 원뿔이 지나가며 비었다고 지운다.
        같은 측정에서 한 칸에 여러 번 찍히지 않도록 측정 번호를 함께 준다.
        """
        return _occ_arc(self, m)


def _occ_arc(sensor, m):
    """RangeSensorArray 한 개의 막힌 점 목록. 반환: (시작점, 끝점, 무게, 측정 번호)."""
    offs = sensor.cone_offsets()
    O, E, W, K = [], [], [], []
    for k, (mount, a, r) in enumerate(zip(m.mounts, m.angles, m.ranges)):
        if not np.isfinite(r):
            continue
        if sensor.map_arc:
            w = 1.0 if r < sensor.arc_near else sensor.arc_far_weight
            for o in offs:
                O.append(mount); E.append(mount + r * np.array([cos(a + o), sin(a + o)]))
                W.append(w); K.append(k)
        else:
            O.append(mount); E.append(mount + r * np.array([cos(a), sin(a)]))
            W.append(1.0); K.append(k)
    z = np.zeros((0, 2))
    return (np.array(O) if O else z, np.array(E) if E else z,
            np.array(W, float), np.array(K, np.int64))


class SensorGroup:
    """
    측정 방식이 다른 센서 묶음을 하나처럼 쓴다.
    예: 서보로 돌리는 앞 센서(한 번에 몇 방향씩) + 고정된 뒤 센서(매번).
    방향 번호는 앞 센서부터 이어 붙인다.
    """

    def __init__(self, name, parts):
        self.name = name
        self.parts = list(parts)
        self.mounts = np.concatenate([p.mounts for p in self.parts])
        self.angles = np.concatenate([p.angles for p in self.parts])
        self.max_range = max(p.max_range for p in self.parts)
        self.cones = np.concatenate([p.cones for p in self.parts])
        self.max_ranges = np.concatenate([p.max_ranges for p in self.parts])
        self._offset = np.cumsum([0] + [len(p.angles) for p in self.parts])[:-1]
        self.n_beams = len(self.angles)
        self.fov = 2 * np.pi
        self.dphi = self.fov / self.n_beams

    def measure(self, world, pose):
        subs = [p.measure(world, pose) for p in self.parts]
        m = Measurement(np.concatenate([s.idx + o for s, o in zip(subs, self._offset)]),
                        np.concatenate([s.mounts for s in subs]),
                        np.concatenate([s.angles for s in subs]),
                        np.concatenate([s.ranges for s in subs]))
        m.subs = subs
        return m

    @property
    def map_arc(self):
        return any(p.map_arc for p in self.parts)

    def occupied_for_map(self, m, res):
        O, E, W, K = [], [], [], []
        base = 0
        for p, s in zip(self.parts, m.subs):
            o, e, w, k = p.occupied_for_map(s, res)
            O.append(o); E.append(e); W.append(w); K.append(k + base)
            base += len(s.idx)
        return np.concatenate(O), np.concatenate(E), np.concatenate(W), np.concatenate(K)

    def rays_for_map(self, m, res):
        fo, fe, oo, oe = [], [], [], []
        for p, s in zip(self.parts, m.subs):
            (a, b), (c, d) = p.rays_for_map(s, res)
            fo.append(a); fe.append(b); oo.append(c); oe.append(d)
        return ((np.concatenate(fo), np.concatenate(fe)),
                (np.concatenate(oo), np.concatenate(oe)))


# ============================================================== 조합 목록
def _bumper(veh):
    """앞 범퍼 위치 (뒷바퀴 축 기준 앞쪽 거리)."""
    rear = 0.5 * (veh.length - veh.wheelbase) + 0.02
    return veh.length - rear - 0.02


SENSOR_SETS = ("lidar360", "tof1", "tof3", "tof_sweep", "tof_sweep_rear", "us1", "us3")

SENSOR_LABEL = {
    "lidar360": "360도 라이다 (비교 기준)",
    "tof1": "ToF 1개 정면",
    "tof3": "ToF 3개 앞 부채꼴",
    "tof_sweep": "ToF 1개 서보 회전",
    "tof_sweep_rear": "ToF 1개 서보 회전 + 뒤 ToF 1개",
    "us1": "초음파 1개 정면",
    "us3": "초음파 3개 앞 부채꼴",
}


def make_sensor(name, veh, rng=None, map_arc=True):
    """
    가정한 사양 (제품 설명서 수준의 대략값)
      ToF VL53L1X : 원뿔 27도, 실내 3.0 m, 오차 2 cm + 거리의 1%, 없음을 알려줌
      초음파 HC-SR04: 원뿔 30도, 3.0 m, 오차 1 cm, 40도보다 비스듬한 면은 못 봄,
                      간섭 때문에 0.1 초에 한 개씩만 쏨
      서보 회전    : 앞 -90~+90 도를 15 도 간격으로 왕복, 0.1 초에 세 방향
    """
    # map_arc 기본값은 켬. 처음 보는 무작위 지도 40 개에서 서보 ToF + 뒤 ToF 조합의
    # 가장 작은 차체 여유가 가운데 점 방식 0.07 m, 호 방식 0.19 m 였다 (둘 다 40/40 도착).
    # 라이다처럼 원뿔이 없는 센서에는 영향이 없다.
    rng = rng or np.random.default_rng(0)
    fx = _bumper(veh)
    if name == "lidar360":
        n = 270
        ang = np.linspace(-np.pi, np.pi, n, endpoint=False)
        return RangeSensorArray(name, np.zeros((n, 2)), ang, 8.0, cone=0.0,
                                noise_abs=0.02, dropout=0.01, reports_empty=False, rng=rng)
    fan3 = np.radians([30.0, 0.0, -30.0])
    mounts3 = [[fx - 0.02, 0.07], [fx, 0.0], [fx - 0.02, -0.07]]
    if name == "tof1":
        return RangeSensorArray(name, [[fx, 0.0]], [0.0], 3.0, cone=radians(27), subrays=9,
                                noise_abs=0.02, noise_rel=0.01, rng=rng, map_arc=map_arc)
    if name == "tof3":
        return RangeSensorArray(name, mounts3, fan3, 3.0, cone=radians(27), subrays=9,
                                noise_abs=0.02, noise_rel=0.01, rng=rng, map_arc=map_arc)
    if name in ("tof_sweep", "tof_sweep_rear"):
        ang = np.radians(np.arange(-90.0, 90.1, 15.0))
        mounts = np.tile([[fx - 0.04, 0.0]], (len(ang), 1))
        front = RangeSensorArray("tof_sweep", mounts, ang, 3.0, cone=radians(27), subrays=9,
                                 per_cycle=3, sweep=True, noise_abs=0.02, noise_rel=0.01,
                                 rng=rng, map_arc=map_arc)
        if name == "tof_sweep":
            return front
        rx = -(0.5 * (veh.length - veh.wheelbase) + 0.02) + 0.02     # 뒤 범퍼
        rear = RangeSensorArray("tof_rear", [[rx, 0.0]], [np.pi], 3.0, cone=radians(27),
                                subrays=9, noise_abs=0.02, noise_rel=0.01, rng=rng, map_arc=map_arc)
        return SensorGroup(name, [front, rear])
    if name == "us1":
        return RangeSensorArray(name, [[fx, 0.0]], [0.0], 3.0, cone=radians(30), subrays=11,
                                noise_abs=0.01, specular_deg=40.0, rng=rng, map_arc=map_arc)
    if name == "us3":
        return RangeSensorArray(name, mounts3, fan3, 3.0, cone=radians(30), subrays=11,
                                per_cycle=1, noise_abs=0.01, specular_deg=40.0, rng=rng, map_arc=map_arc)
    raise ValueError(f"unknown sensor set: {name}. 선택지: {SENSOR_SETS}")
