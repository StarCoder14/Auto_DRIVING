"""
car_specs.py -- 우리 차를 잰 값을 모아 두는 곳

차를 재면 아래 MEASURED 의 None 을 잰 값으로 바꾸면 됨. 길이는 전부 cm, 각도는 도.
실물 주행(run_robot.py), 시뮬레이터(run.py --car ours), 오차 분해, 처음 기하 모델이 이 값을 씀.
아직 안 잰 값은 시뮬레이터 기본 차의 값을 대신 씀.

  python -m rccar.car_specs     # 어떤 값이 비었는지, 계산된 값, 펌웨어에 옮길 값을 보여줌

위치는 전부 뒷바퀴 축 가운데를 0 으로 잼. 시뮬레이터가 차의 위치를 그 점으로 다루기 때문임.
"""

from math import atan, degrees, sqrt

from .vehicle import VehicleParams

MEASURED = {
    # 차체 (2026-10-21 실측)
    "length_cm": 22.0,              # 전장. 범퍼 포함 가장 앞에서 가장 뒤까지
    "width_cm": 12.0,               # 전폭. 바퀴 포함 가장 바깥끼리
    "wheelbase_cm": 15.0,           # 축거. 앞바퀴 중심에서 뒷바퀴 중심까지
    "track_cm": None,               # 윤거. 왼쪽 바퀴 중심에서 오른쪽 바퀴 중심까지 (아직)
    "rear_overhang_cm": 4.0,        # 뒷바퀴 축에서 차 뒤끝까지 (앞끝 18 cm = 22 - 4, 자동 계산)
    "wheel_diameter_cm": 5.5,

    # 조향. 핸들을 끝까지 꺾고 손으로 천천히 밀어 한 바퀴 돌 때, 바깥 앞바퀴 자국 원의 지름
    "turn_diameter_left_cm": None,
    "turn_diameter_right_cm": None,
    "steer_center_deg": None,       # 바퀴가 똑바를 때 조향 서보에 준 각도 (펌웨어 STEER_CENTER_DEG)

    # 센서. 앞 ToF 하나를 서보에 올려 좌우로 훑음. 뒤 센서는 없음
    "front_tof_from_rear_axle_cm": None,   # 센서 서보 회전축이 뒷바퀴 축에서 앞으로 몇 cm
    "front_tof_height_cm": None,           # 바닥에서 센서까지. 기둥이 이보다 높아야 보임
    "has_rear_tof": False,
    "rear_tof_from_rear_axle_cm": None,    # 뒤 센서를 달면 뒷바퀴 축에서 뒤로 몇 cm (양수로 적음)

    # 시험 공간
    "arena_width_cm": None,
    "arena_length_cm": None,
    "pillar_diameter_cm": None,
    "pillar_height_cm": None,
}

# 아직 안 잰 시험 공간 값은 계획서에 적은 값으로 둠
PLANNED_ARENA_CM = 150.0
PLANNED_PILLAR_DIAMETER_CM = 8.0

# 앞 거리 센서 (샤프 적외선 GP2Y0A21). 펌웨어의 SHARP_MIN_MM, SHARP_MAX_MM 과 같은 값이어야 함
FRONT_SENSOR = {
    "min_range_m": 0.10,     # 이보다 가까우면 펌웨어가 '못 맞힘'으로 보냄
    "max_range_m": 0.80,     # 이보다 멀면 펌웨어가 '못 맞힘'으로 보냄
    # 빛이 퍼지는 폭. 아직 안 잼. 레이저(27도)보다 훨씬 좁다고 알려져 있어 10도로 짐작함
    "cone_deg": 10.0,
}

# 펌웨어의 속도 -> 모터 힘 변환. rccar_arduino.ino 의 MAX_SPEED_MMS, MOTOR_MIN_PWM 과 같아야 함
FIRMWARE_MAX_SPEED_MPS = 1.4     # 이 속도를 보내면 모터 힘 255 (끝까지)
FIRMWARE_MOTOR_MIN_PWM = 60      # 힘이 이보다 약하면 펌웨어가 모터를 끔


def min_drive_speed_mps():
    """이보다 느린 명령은 펌웨어가 모터를 꺼 버림. 그래서 주행 코드는 이보다 느리게 보내지 않음."""
    # 255 로 나눴다 다시 곱하면서 버림이 생겨 경계에서 꺼질 수 있음. 1 을 더해 확실히 넘김
    return (FIRMWARE_MOTOR_MIN_PWM + 1) / 255.0 * FIRMWARE_MAX_SPEED_MPS


def _meters(name):
    value = MEASURED[name]
    return None if value is None else value / 100.0


def steer_from_turn_diameter(diameter_cm, wheelbase_m, track_m):
    """바깥 앞바퀴가 그린 원의 지름으로 최대 조향각을 거꾸로 구함 (라디안).

    자전거 모델에서 뒷바퀴 축 가운데가 그리는 원의 반지름을 R 이라 하면 tan(조향각) = 축거 / R.
    바깥 앞바퀴는 뒷바퀴 축 가운데에서 옆으로 윤거/2, 앞으로 축거만큼 떨어져 있으므로
      (바깥 앞바퀴 반지름)^2 = (R + 윤거/2)^2 + 축거^2
    이 식을 R 에 대해 풀면 R = sqrt(바깥 반지름^2 - 축거^2) - 윤거/2.
    각도기로 바퀴 각도를 직접 재는 것보다 원 지름을 재는 쪽이 오차가 훨씬 작음.
    """
    outer_radius = diameter_cm / 200.0
    rear_center_radius = sqrt(max(outer_radius ** 2 - wheelbase_m ** 2, 0.0)) - track_m / 2.0
    if rear_center_radius <= 0:
        raise ValueError("회전 지름이 축거에 비해 너무 작음. 잰 값을 다시 확인할 것")
    return atan(wheelbase_m / rear_center_radius)


def vehicle_params(**overrides):
    """우리 차의 VehicleParams. 잰 값은 잰 값으로, 안 잰 값은 시뮬레이터 기본값으로 채움."""
    default = VehicleParams()
    settings = {}
    for field, name in (("length", "length_cm"), ("width", "width_cm"),
                        ("wheelbase", "wheelbase_cm"), ("rear_overhang", "rear_overhang_cm"),
                        ("front_sensor_x", "front_tof_from_rear_axle_cm")):
        if MEASURED[name] is not None:
            settings[field] = MEASURED[name] / 100.0
    if MEASURED["has_rear_tof"] and MEASURED["rear_tof_from_rear_axle_cm"] is not None:
        settings["rear_sensor_x"] = -MEASURED["rear_tof_from_rear_axle_cm"] / 100.0

    steers = max_steer_by_side(settings.get("wheelbase", default.wheelbase))
    if steers:
        # 계획기는 좌우가 같다고 봄. 덜 꺾이는 쪽에 맞춰야 계획한 곡선을 양쪽 다 따라갈 수 있음
        settings["max_steer"] = min(steers.values())
    settings.update(overrides)
    return VehicleParams(**settings)


def max_steer_by_side(wheelbase_m):
    """잰 회전 지름이 있으면 좌우 최대 조향각 (라디안)."""
    track_m = _meters("track_cm")
    if track_m is None:
        return {}
    steers = {}
    for side in ("left", "right"):
        diameter = MEASURED[f"turn_diameter_{side}_cm"]
        if diameter is not None:
            steers[side] = steer_from_turn_diameter(diameter, wheelbase_m, track_m)
    return steers


def half_width_m(fallback):
    """처음 기하 모델에서 기둥 반지름에 더할 차 반폭."""
    width = _meters("width_cm")
    return fallback if width is None else width / 2.0


def has_rear_sensor():
    return bool(MEASURED["has_rear_tof"])


def arena_m():
    """시험 공간 (가로, 세로) m. 안 쟀으면 계획서의 1.5 m."""
    width = MEASURED["arena_width_cm"] or PLANNED_ARENA_CM
    length = MEASURED["arena_length_cm"] or PLANNED_ARENA_CM
    return width / 100.0, length / 100.0


def pillar_radius_m():
    return (MEASURED["pillar_diameter_cm"] or PLANNED_PILLAR_DIAMETER_CM) / 200.0


def report():
    """빈칸, 계산된 값, 펌웨어에 옮길 값을 보여줌."""
    missing = [name for name, value in MEASURED.items()
               if value is None and not (name == "rear_tof_from_rear_axle_cm" and not MEASURED["has_rear_tof"])]
    print("잰 값")
    for name, value in MEASURED.items():
        mark = "아직 안 잼" if name in missing else value
        print(f"  {name:<30} {mark}")

    car = vehicle_params()
    print("\n시뮬레이터가 쓸 값 (안 잰 것은 기본 차의 값)")
    print(f"  전장 {car.length:.3f} m, 전폭 {car.width:.3f} m, 축거 {car.wheelbase:.3f} m")
    print(f"  뒷바퀴 축에서 뒤끝 {car.rear_overhang_m:.3f} m, 앞끝 {car.front_bumper_x:.3f} m")
    print(f"  앞 센서 위치 {car.front_sensor_x_m:.3f} m, 뒤 센서 {'있음' if has_rear_sensor() else '없음'}")
    print(f"  최대 조향각 {degrees(car.max_steer):.1f}도, 최소 회전 반경(뒷바퀴 축 가운데) {car.min_turn_radius:.3f} m")
    steers = max_steer_by_side(car.wheelbase)
    if len(steers) == 2:
        left, right = degrees(steers["left"]), degrees(steers["right"])
        print(f"  좌 {left:.1f}도, 우 {right:.1f}도 (차이 {abs(left - right):.1f}도"
              f"{'. 서보 중앙이 틀어졌을 수 있음' if abs(left - right) > 2 else ''})")
    width, length = arena_m()
    print(f"  시험 공간 {width:.2f} x {length:.2f} m, 기둥 반지름 {pillar_radius_m():.3f} m")
    if MEASURED["pillar_height_cm"] and MEASURED["front_tof_height_cm"]:
        if MEASURED["pillar_height_cm"] <= MEASURED["front_tof_height_cm"]:
            print("  [주의] 기둥이 센서보다 낮음. 센서가 기둥을 못 봄")

    print("\n펌웨어 (firmware/rccar_arduino/rccar_arduino.ino) 에 손으로 옮길 값")
    print(f"  const float MAX_STEER_DEG = {degrees(car.max_steer):.1f};")
    if MEASURED["steer_center_deg"] is not None:
        print(f"  const float STEER_CENTER_DEG = {MEASURED['steer_center_deg']:.1f};")
    print(f"  뒤 센서: {'있음' if has_rear_sensor() else '없음 (펌웨어에서 뒤 센서를 끄는 설정이 필요함)'}")

    if missing:
        print(f"\n아직 안 잰 값 {len(missing)}개: " + ", ".join(missing))


if __name__ == "__main__":
    report()
