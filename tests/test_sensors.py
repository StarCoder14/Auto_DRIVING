"""싼 거리 센서 흉내 검증.

    python -m tests.test_sensors
"""
from __future__ import annotations

import sys
from math import radians

import numpy as np

from rccar.sensors import SENSOR_SETS, RangeSensorArray, make_sensor, raycast_world
from rccar.vehicle import VehicleParams
from rccar.world import World


def test_raycast_matches_world() -> None:
    """numpy 로 한꺼번에 쏜 광선이 world.raycast 와 같은 거리를 낸다."""
    rng = np.random.default_rng(3)
    w = World(circles=[((2.0, 1.0), 0.5), ((-1.0, 3.0), 0.8), ((4.0, -2.0), 0.3)],
              walls=[((-3.0, -3.0), (3.0, -3.0)), ((5.0, 0.0), (5.0, 4.0))])
    O = rng.uniform(-2.0, 2.0, (300, 2))
    a = rng.uniform(-np.pi, np.pi, 300)
    D = np.stack([np.cos(a), np.sin(a)], axis=1)
    t, _ = raycast_world(w, O, D, 8.0)
    for i in range(300):
        ref = w.raycast(O[i], D[i], 8.0)
        ref = np.inf if ref >= 8.0 - 1e-9 else ref
        assert (np.isinf(t[i]) and np.isinf(ref)) or abs(t[i] - ref) < 1e-9, \
            f"{i}번 광선: {t[i]} != {ref}"


def test_cone_sees_off_axis() -> None:
    """원뿔 센서는 정면에서 비껴 있는 물체도 잡는다. 가는 광선은 못 잡는다."""
    w = World(circles=[((2.0, 0.6), 0.2)])
    cone = RangeSensorArray("c", [[0, 0]], [0.0], 3.0, cone=radians(30), subrays=15,
                            noise_abs=0.0)
    thin = RangeSensorArray("t", [[0, 0]], [0.0], 3.0, cone=0.0, noise_abs=0.0)
    rc = cone.measure(w, (0.0, 0.0, 0.0)).ranges[0]
    rt = thin.measure(w, (0.0, 0.0, 0.0)).ranges[0]
    assert np.isfinite(rc) and rc < 2.2, f"원뿔이 옆 물체를 못 잡았다: {rc}"
    assert np.isinf(rt), "가는 광선이 정면에 없는 물체를 잡았다"


def test_ultrasonic_misses_slanted_wall() -> None:
    """초음파는 거의 나란한 벽을 놓치고, 마주 보는 벽은 잡는다."""
    us = RangeSensorArray("u", [[0, 0]], [0.0], 3.0, cone=radians(10), subrays=5,
                          noise_abs=0.0, specular_deg=40.0)
    facing = World(walls=[((2.0, -2.0), (2.0, 2.0))])
    slanted = World(walls=[((0.5, -1.0), (3.0, 0.3))])        # 정면과 약 27도
    assert np.isfinite(us.measure(facing, (0, 0, 0)).ranges[0]), "마주 보는 벽을 놓쳤다"
    assert np.isinf(us.measure(slanted, (0, 0, 0)).ranges[0]), "비스듬한 벽이 보였다"


def test_sweep_and_round_robin() -> None:
    """서보 센서는 왕복하며, 초음파 묶음은 돌아가며 잰다."""
    V = VehicleParams()
    sw = make_sensor("tof_sweep", V)
    w = World()
    seen = [int(i) for _ in range(10) for i in sw.measure(w, (0, 0, 0)).idx]
    assert max(seen) == len(sw.angles) - 1 and min(seen) == 0, "끝까지 훑지 않는다"
    assert all(abs(a - b) == 1 for a, b in zip(seen, seen[1:]) if a != b), "건너뛴다"
    us = make_sensor("us3", V)
    order = [int(us.measure(w, (0, 0, 0)).idx[0]) for _ in range(6)]
    assert order == [0, 1, 2, 0, 1, 2], f"돌아가며 재지 않는다: {order}"


def test_all_sets_build() -> None:
    """모든 센서 조합이 만들어지고 한 번씩 잴 수 있다."""
    V = VehicleParams()
    w = World(circles=[((2.0, 0.0), 0.4)])
    for name in SENSOR_SETS:
        s = make_sensor(name, V)
        m = s.measure(w, (0.0, 0.0, 0.0))
        (fo, fe), (oo, oe) = s.rays_for_map(m, 0.12)
        assert len(m.idx) > 0, f"{name}: 잰 방향이 없다"
        assert len(fe) > 0 or len(oe) > 0, f"{name}: 지도에 기록할 광선이 없다"


TESTS = [
    ("광선 계산 = world.raycast", test_raycast_matches_world),
    ("원뿔은 옆 물체를 잡음", test_cone_sees_off_axis),
    ("초음파는 비스듬한 벽을 놓침", test_ultrasonic_misses_slanted_wall),
    ("서보 왕복, 초음파 순번", test_sweep_and_round_robin),
    ("모든 센서 조합 생성", test_all_sets_build),
]


def main() -> int:
    failed = 0
    for name, fn in TESTS:
        try:
            fn()
        except AssertionError as exc:
            print(f"  [실패] {name}: {exc}")
            failed += 1
        else:
            print(f"  [통과] {name}")
    if failed:
        print(f"\n{failed}개 실패")
        return 1
    print("\n센서 전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
