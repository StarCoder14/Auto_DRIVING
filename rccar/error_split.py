"""
error_split.py -- 이론값에서 벗어난 만큼을 원인별로 나눠 재기 (계획서 6-라)

전체 오차를 한 번에 재면 원인을 구분할 수 없음. 그래서 같은 지도, 같은 출발점과 목표에서
한 번에 한 가지만 실제에 가깝게 바꿔 가며 달리고, 앞 단계보다 늘어난 만큼을 그 원인의 몫으로 봄.

  0 다 아는 차 지도를 처음부터 다 앎. 위치 추정 완벽                      (계획서의 알고리즘 C)
  1 알고리즘  보이는 것만 앎. 센서는 이상적 (1도 간격, 잡음 없음, 원뿔 없음)
  2 센서      실제 ToF 흉내 (서보로 15도씩 왕복, 원뿔 27도, 잡음 2 cm + 거리의 1%)
  3 위치 추정 엔코더 없이 '보낸 속도 x 시간' 과 자이로로 위치를 짐작
  4 실차      실제 차를 달려 바닥에 남긴 궤적. 3 과의 차이가 기계와 제어의 몫

0 단계는 '이론적 최적'이 아님. 계획기가 줄이는 것은 길이가 아니라 비용이고, 비용에는
장애물에 가까울수록 붙는 벌점이 들어 있음. 그래서 다 아는 차는 좁은 틈을 피해 넓게 돌아가고,
보이는 것만 아는 차는 못 본 기둥을 빈 땅으로 여겨 바짝 붙어 더 짧게 갈 때가 많음
(무작위 지도 12 개 중 8 개, 2026-10-19). 길이만 보면 '모르는 게 낫다' 는 틀린 결론이 나와서,
길이와 여유와 도달 오차를 늘 함께 봄.

지표 (계획서의 네 가지에 차체 여유와 위치 추정 오차를 더함)
  경로 길이 비        다 아는 차가 달린 길이에 대한 비
  기준 경로까지 거리  실제 궤적 위 점에서 기준 경로까지 최단거리의 평균과 최댓값
                      (점과 선분 사이 거리. 충돌 판정에 쓰는 것과 같은 공식)
  최종 도달 오차      차가 '도착했다' 고 멈춘 순간 진짜 위치와 진짜 목표 사이 거리
                      (도착 판정 반경 goal_tol 안이면 도착으로 치므로 0 이 되지는 않음)
  도달 성공률         부딪히지 않고 시간 안에 도착한 비율
  차체 최소 여유      달리는 동안 차체 사각형 둘레와 실제 장애물 사이 가장 가까웠던 거리
                      (뒷바퀴 축 한 점으로 재면 차 앞머리가 파고들어도 모름. README 5단계 참고)
  위치 추정 오차      멈춘 순간 차가 믿은 위치와 진짜 위치 사이 거리
"""

from math import radians

import numpy as np

from . import car_specs
from .hybrid import HybridParams
from .hybrid_sim import HybridSimulator
from .grid import L_MAX, L_MIN
from .sensors import RangeSensorArray, make_sensor
from .simulator import SimConfig

STAGES = ("0 다 아는 차", "1 알고리즘", "2 센서", "3 위치 추정")
CAUSES = ("알고리즘", "센서", "위치 추정")


class KnowsEverything(HybridSimulator):
    """0 단계. 센서로 잰 것과 상관없이 격자에 정답 지도를 덮어씀.

    센서가 원뿔 가장자리로 막힌 칸을 '비었음' 쪽으로 깎을 수 있어서, 매 주기 다시 덮어씀.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._truth = _true_log_odds(self.world, self.grid)

    def _sense_array(self, odom):
        super()._sense_array(odom)
        self.grid.L[:] = self._truth


def _true_log_odds(world, grid):
    """칸 중심이 실제 장애물 안이거나 반 칸 안쪽으로 붙어 있으면 막힘, 나머지는 비었음."""
    xs = grid.origin[0] + (np.arange(grid.nx) + 0.5) * grid.res
    ys = grid.origin[1] + (np.arange(grid.ny) + 0.5) * grid.res
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    nearest = np.full(X.shape, np.inf)
    for center, radius in world.circles:
        nearest = np.minimum(nearest, np.hypot(X - center[0], Y - center[1]) - radius)
    for a, b in world.walls:
        ab = b - a
        t = np.clip(((X - a[0]) * ab[0] + (Y - a[1]) * ab[1]) / float(ab @ ab), 0.0, 1.0)
        nearest = np.minimum(nearest, np.hypot(X - (a[0] + t * ab[0]), Y - (a[1] + t * ab[1])))
    return np.where(nearest < 0.5 * grid.res, L_MAX, L_MIN).astype(grid.L.dtype)


def ideal_sensor(veh):
    """1 단계 센서. 보는 범위는 실제 ToF 와 같고(앞쪽 반원 + 원뿔 반각, 3 m), 보이는 것은 정확히 앎."""
    half_view = 90.0 + 27.0 / 2.0
    angles = np.radians(np.arange(-half_view, half_view + 0.01, 1.0))
    mounts = np.tile([[veh.front_sensor_x_m, 0.0]], (len(angles), 1))
    return RangeSensorArray("ideal", mounts, angles, 3.0, cone=0.0,
                            noise_abs=0.0, noise_rel=0.0, rng=np.random.default_rng(0))


def stage_vehicle(stage, base):
    """단계에 맞게 위치 추정을 완벽하게 하거나 실제처럼 틀리게 만든 차."""
    vehicle = car_specs.vehicle_params() if base is None else base
    if stage < 3:
        vehicle.odom_drift = 0.0
        vehicle.odom_yaw_drift = 0.0
        vehicle.odom_yaw_noise = 0.0
        vehicle.speed_scale_err = 0.0
        vehicle.gyro_bias_dps = 0.0
        vehicle.dist_source = "encoder"
    else:
        # 실물과 같음: 엔코더가 없어 명령 속도로 거리를 짐작하고, 자이로는 조금 틀어짐
        vehicle.dist_source = "command"
        vehicle.speed_scale_err = 0.08
        vehicle.gyro_bias_dps = 0.2
    return vehicle


def simulate(stage, world, start, goal, seed, make_vehicle, max_time=120.0):
    """한 단계를 한 번 달려서 진짜 궤적과 결과를 돌려줌."""
    vehicle = stage_vehicle(stage, make_vehicle())
    rng = np.random.default_rng(seed)
    if stage <= 1:
        sensor = ideal_sensor(vehicle)
    else:
        sensor = make_sensor("tof_sweep_rear" if car_specs.has_rear_sensor() else "tof_sweep",
                             vehicle, rng=rng)
    # 뒤를 못 보면 후진 경로를 만들지 않음. 모든 단계가 같은 규칙이어야 비교가 됨
    params = HybridParams(allow_reverse=car_specs.has_rear_sensor())
    simulator_class = KnowsEverything if stage == 0 else HybridSimulator
    sim = simulator_class(world, start, goal, cfg=SimConfig(max_time=max_time), veh=vehicle,
                          lidar=sensor, seed=seed, hparams=params)
    # sim.run() 과 같지만, 차체 여유를 재려고 0.05 초마다 차의 진짜 자세를 들여다봄
    closest = np.inf
    body = BodyOutline(vehicle)
    while sim.step():
        if sim.k % 5 == 0:
            closest = min(closest, body.clearance(world, sim.vehicle.pose))
    closest = min(closest, body.clearance(world, sim.vehicle.pose))
    summary = sim.summary()
    trail = np.array(sim.trail + [sim.vehicle.pose[:2].copy()])
    return {
        "stage": stage,
        "success": bool(summary["arrived"] and not summary["collided"]),
        "collided": bool(summary["collided"]),
        "trail": trail,
        "length": path_length(trail),
        "final_error": float(np.hypot(*(sim.vehicle.pose[:2] - np.asarray(goal)[:2]))),
        "localization_error": float(np.hypot(*(sim.vehicle.pose[:2] - sim.vehicle.odom[:2]))),
        "body_clearance": float(closest),
        "time": summary["time"],
    }


class BodyOutline:
    """차체 사각형 둘레에 촘촘히 찍은 점. 차 기준 좌표(뒷바퀴 축 가운데가 0)로 미리 만들어 둠."""

    def __init__(self, vehicle, spacing=0.02):
        back, front = -vehicle.rear_overhang_m, vehicle.front_bumper_x
        half = vehicle.width / 2.0
        corners = [(back, -half), (front, -half), (front, half), (back, half), (back, -half)]
        points = []
        for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
            count = max(2, int(np.hypot(x1 - x0, y1 - y0) / spacing))
            for t in np.linspace(0.0, 1.0, count, endpoint=False):
                points.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        self.points = np.array(points)

    def clearance(self, world, pose):
        """차체 둘레에서 실제 장애물 표면까지 가장 가까운 거리. 음수면 파고든 것."""
        x, y, heading = pose
        c, s = np.cos(heading), np.sin(heading)
        px = x + c * self.points[:, 0] - s * self.points[:, 1]
        py = y + s * self.points[:, 0] + c * self.points[:, 1]
        nearest = np.inf
        for center, radius in world.circles:
            nearest = min(nearest, float(np.min(np.hypot(px - center[0], py - center[1]))) - radius)
        for a, b in world.walls:
            ab = b - a
            t = np.clip(((px - a[0]) * ab[0] + (py - a[1]) * ab[1]) / float(ab @ ab), 0.0, 1.0)
            nearest = min(nearest, float(np.min(np.hypot(px - (a[0] + t * ab[0]), py - (a[1] + t * ab[1])))))
        return nearest


def path_length(points):
    points = np.asarray(points, float)
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.hypot(*np.diff(points, axis=0).T)))


def distances_to_path(points, path):
    """각 점에서 경로(꺾은선)까지의 최단거리. 경로의 선분마다 점과 선분 사이 거리를 구해 가장 작은 값."""
    points = np.asarray(points, float)[:, None, :]
    start, end = np.asarray(path[:-1], float), np.asarray(path[1:], float)
    along = end - start
    length_sq = np.maximum(np.sum(along * along, axis=1), 1e-12)
    # 점에서 선분에 내린 수선의 발이 선분 밖이면 가까운 끝점을 씀
    t = np.clip(np.sum((points - start) * along, axis=2) / length_sq, 0.0, 1.0)
    foot = start + t[..., None] * along
    return np.min(np.hypot(*(points - foot).transpose(2, 0, 1)), axis=1)


def compare(run, baseline):
    """기준 단계와 비교한 지표."""
    gaps = distances_to_path(run["trail"], baseline["trail"])
    return {
        "length_ratio": run["length"] / baseline["length"] if baseline["length"] > 0 else np.nan,
        "gap_mean": float(np.mean(gaps)),
        "gap_max": float(np.max(gaps)),
        "final_error": run["final_error"],
        "success": run["success"],
        "body_clearance": run.get("body_clearance", np.nan),
        "localization_error": run.get("localization_error", np.nan),
    }


def split_one_map(world, start, goal, seed, make_vehicle, max_time=120.0):
    """지도 하나에서 0~3 단계를 다 달림."""
    runs = [simulate(stage, world, start, goal, seed, make_vehicle, max_time)
            for stage in range(len(STAGES))]
    return runs, [compare(run, runs[0]) for run in runs]


def real_track_metrics(track_points, baseline, goal):
    """4 단계. 바닥에 남은 궤적을 재서 넣은 점들(m)로 같은 지표를 냄."""
    track = np.asarray(track_points, float)
    run = {"trail": track, "length": path_length(track),
           "final_error": float(np.hypot(*(track[-1] - np.asarray(goal)[:2]))), "success": True}
    return compare(run, baseline)


# ── 실측 배치 파일 ───────────────────────────────────────────────────

def load_layout(path):
    """시험 공간을 잰 파일을 읽음. 단위 cm, 각도는 도. # 뒤는 메모.

        start, 30, 75, 0        # 출발: 뒷바퀴 축 가운데 위치와 차가 보는 방향
        goal, 270, 75           # 목표
        pillar, 120, 60, 8      # 기둥: 중심과 지름. 줄마다 하나
        wall, 0, 0, 300, 0      # 벽(있으면): 양 끝점
    """
    from .world import World
    start = goal = None
    circles, walls = [], []
    with open(path, encoding="utf-8") as file:
        for raw in file:
            line = raw.split("#")[0].strip()
            if not line:
                continue
            kind, *numbers = [part.strip() for part in line.split(",")]
            values = [float(number) for number in numbers]
            if kind == "start":
                start = np.array([values[0] / 100, values[1] / 100, radians(values[2])])
            elif kind == "goal":
                goal = np.array([values[0] / 100, values[1] / 100])
            elif kind == "pillar":
                circles.append(((values[0] / 100, values[1] / 100), values[2] / 200))
            elif kind == "wall":
                walls.append(((values[0] / 100, values[1] / 100), (values[2] / 100, values[3] / 100)))
            else:
                raise ValueError(f"모르는 줄: {raw.strip()}")
    if start is None or goal is None:
        raise ValueError("start 와 goal 줄이 있어야 함")
    return World(circles=circles, walls=walls, bounds=(-16, -16, 16, 16)), start, goal


def load_track(path):
    """바닥 궤적을 잰 점들. 한 줄에 'x, y' (cm). 출발에서 도착 순서대로."""
    points = []
    with open(path, encoding="utf-8") as file:
        for raw in file:
            line = raw.split("#")[0].strip()
            if line:
                x, y = (float(part) for part in line.split(",")[:2])
                points.append((x / 100, y / 100))
    return np.array(points)
