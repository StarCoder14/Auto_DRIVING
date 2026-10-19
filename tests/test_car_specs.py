"""차 사양 파일 검증.

    python -m tests.test_car_specs
"""
from __future__ import annotations

import sys
from math import degrees, hypot, radians, tan

from rccar import car_specs
from rccar.vehicle import VehicleParams


def _with_measured(values, fn):
    """MEASURED 를 잠깐 바꿔서 fn 을 돌리고 원래대로 돌려놓음."""
    saved = dict(car_specs.MEASURED)
    car_specs.MEASURED.update(values)
    try:
        return fn()
    finally:
        car_specs.MEASURED.clear()
        car_specs.MEASURED.update(saved)


def test_turn_diameter_round_trip():
    """조향각에서 바깥 앞바퀴 원 지름을 만들고, 그 지름으로 다시 조향각을 구하면 같아야 함."""
    for steer_deg in (15.0, 22.5, 32.0, 40.0):
        wheelbase, track = 0.26, 0.17
        rear_center_radius = wheelbase / tan(radians(steer_deg))
        outer_radius = hypot(rear_center_radius + track / 2, wheelbase)
        back = car_specs.steer_from_turn_diameter(outer_radius * 200.0, wheelbase, track)
        assert abs(degrees(back) - steer_deg) < 1e-9, f"{steer_deg}도 -> {degrees(back)}도"


def test_nothing_measured_keeps_simulator_car():
    """아무것도 안 쟀으면 시뮬레이터 기본 차와 똑같아야 함. 예전 측정값이 그대로 유지되는 근거."""
    empty = {name: None for name in car_specs.MEASURED if name != "has_rear_tof"}
    ours, default = _with_measured(empty, car_specs.vehicle_params), VehicleParams()
    for field in ("length", "width", "wheelbase", "max_steer", "rear_overhang_m",
                  "front_sensor_x_m", "rear_sensor_x_m"):
        assert getattr(ours, field) == getattr(default, field), field


def test_measured_values_flow_into_vehicle():
    values = {"length_cm": 40.0, "width_cm": 20.0, "wheelbase_cm": 26.0, "track_cm": 17.0,
              "rear_overhang_cm": 6.0, "front_tof_from_rear_axle_cm": 30.0,
              "turn_diameter_left_cm": 150.0, "turn_diameter_right_cm": 160.0}
    car = _with_measured(values, car_specs.vehicle_params)
    assert abs(car.length - 0.40) < 1e-12 and abs(car.width - 0.20) < 1e-12
    assert abs(car.rear_overhang_m - 0.06) < 1e-12
    assert abs(car.front_bumper_x - 0.34) < 1e-12
    assert abs(car.front_sensor_x_m - 0.30) < 1e-12
    # 덜 꺾이는 쪽(지름이 큰 오른쪽)에 맞춰야 함
    right = car_specs.steer_from_turn_diameter(160.0, 0.26, 0.17)
    assert abs(car.max_steer - right) < 1e-12
    assert car.min_turn_radius > 0


def test_half_width_follows_width():
    assert _with_measured({"width_cm": 18.0}, lambda: car_specs.half_width_m(0.07)) == 0.09
    assert _with_measured({"width_cm": None}, lambda: car_specs.half_width_m(0.07)) == 0.07


TESTS = [
    ("회전 지름 <-> 조향각 왕복", test_turn_diameter_round_trip),
    ("안 쟀으면 기본 차 그대로", test_nothing_measured_keeps_simulator_car),
    ("잰 값이 차에 들어감", test_measured_values_flow_into_vehicle),
    ("기하 모델 반폭", test_half_width_follows_width),
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
    print("\n차 사양 전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
