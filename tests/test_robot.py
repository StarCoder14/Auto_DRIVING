"""실물용 통신 규약과 주행 프로그램 검증.

부품이 없어도 여기까지는 확인할 수 있다. 가짜 아두이노가 같은 문장을 만들어 준다.

    python -m tests.test_robot
"""
from __future__ import annotations

import math
import sys

import numpy as np

from rccar import scenarios as S
from rccar.fake_arduino import FakeArduino
from rccar.link import (FLAG_FRONT_OK, FLAG_REAR_OK, encode_cmd, encode_run, parse_status,
                        _checksum)
from rccar.robot import Robot, RobotConfig


def _status_line(**kw):
    f = FLAG_FRONT_OK | FLAG_REAR_OK
    body = "S,{ms},{gyro},{sweep},{front},{rear},{cmd},{flags}".format(
        ms=kw.get("ms", 1000), gyro=kw.get("gyro", -1520), sweep=kw.get("sweep", 450),
        front=kw.get("front", 842), rear=kw.get("rear", 1200),
        cmd=kw.get("cmd", 850), flags=kw.get("flags", f))
    return f"{body}*{_checksum(body):02X}"


def test_cmd_round_trip() -> None:
    """명령 문장은 아두이노가 읽는 형식 그대로여야 한다."""
    b = encode_cmd(0.85, math.radians(-12.3)).decode()
    assert b.startswith("C,850,-123*"), f"명령 형식이 다르다: {b}"
    assert b.endswith("\n"), "줄바꿈이 없다"


def test_status_parse() -> None:
    """상태 문장을 숫자로 옳게 푼다."""
    st = parse_status(_status_line())
    assert st is not None, "정상 문장을 못 읽었다"
    assert st.t_ms == 1000
    assert abs(st.gyro_dps + 15.2) < 1e-6, f"자이로 {st.gyro_dps}"
    assert abs(st.sweep_rad - math.radians(45.0)) < 1e-9
    assert abs(st.front_m - 0.842) < 1e-9 and abs(st.rear_m - 1.2) < 1e-9
    assert abs(st.speed_mps - 0.85) < 1e-9


def test_bad_frames_rejected() -> None:
    """검사값이 틀리거나 잘린 문장은 버린다. 선이 흔들리면 실제로 생긴다."""
    good = _status_line()
    assert parse_status(good[:-2] + "00") is None, "검사값이 틀린데 받아들였다"
    assert parse_status(good[: len(good) // 2]) is None, "잘린 문장을 받아들였다"
    assert parse_status("쓰레기") is None
    assert parse_status(good) is not None, "정상 문장을 버렸다"


def test_no_signal_means_no_range() -> None:
    """유효 플래그가 없으면 거리는 '못 맞힘'이어야 한다. 0 으로 읽으면 코앞에 벽이 생긴다."""
    st = parse_status(_status_line(flags=0))
    assert math.isinf(st.front_m) and math.isinf(st.rear_m)


def test_goal_restarts_map() -> None:
    """출발 신호(G 줄)를 받으면 기다리는 동안 칠한 지도를 지우고, 좌우를 다시 다 본 뒤 출발한다.
    손으로 출발시키면 센서 앞에 댄 손이 '차 코앞 기둥'으로 남기 때문."""
    sweeps = [0, -75, -60, 60, 75, 60]
    lines = [_status_line(ms=1000 + 20 * i, sweep=a * 10, front=120, flags=FLAG_FRONT_OK, cmd=0, gyro=0)
             for i, a in enumerate(sweeps)]
    link = _ScriptedLink(lines)
    rb = Robot(link, np.array([99.0, 0.0]), cfg=RobotConfig(bounds=(-3, -3, 3, 3), grid_res=0.04))
    rb.step(0.02)
    assert rb.ready and rb.grid.occupied_mask().any(), "기다리는 동안 본 것이 지도에 있어야 함"
    link.goal_cm = 100
    rb.step(0.02)
    assert not rb.ready, "출발 신호 뒤에도 예전 훑기로 바로 출발하려 함"
    assert not rb.grid.occupied_mask().any(), "출발 신호 뒤에도 기다리는 동안 본 손이 지도에 남음"
    assert abs(rb.goal[0] - (1.0 + rb.C.goal_tol)) < 1e-9, f"목적지 {rb.goal}"


def test_run_command_format() -> None:
    """시험용 R 명령은 펌웨어가 읽는 'R,1*검사값' 형식이어야 한다."""
    on, off = encode_run(True).decode(), encode_run(False).decode()
    assert on == f"R,1*{_checksum('R,1'):02X}\n", f"R 명령 형식이 다르다: {on!r}"
    assert off.startswith("R,0*") and off.endswith("\n")


class _ScriptedLink:
    """정해 둔 상태 줄만 내보내는 가짜 통신. 보낸 명령은 모아 둠."""

    def __init__(self, lines):
        self.lines = list(lines)
        self.sent = []

    def poll(self):
        out = [parse_status(line) for line in self.lines]
        self.lines = []
        return out

    def send(self, speed, steer):
        self.sent.append((speed, steer))


def test_silent_arduino_no_planning() -> None:
    """아두이노가 아무 말도 안 하면 길을 찾지 않는다. 2026-10-28 실물에서 얼어붙은 아두이노 때문에
    보드가 99 m 길을 찾느라 힘을 다 썼음."""
    rb = Robot(_ScriptedLink([]), np.array([99.0, 0.0]), cfg=RobotConfig(bounds=(-3, -3, 3, 3), grid_res=0.04))
    for _ in range(20):
        rb.step(0.02)
        rb.plan_cycle()
        rb.control_cycle()
    assert rb.stats["plans"] == 0, f"아무 말도 없는데 길을 {rb.stats['plans']} 번 찾았다"
    assert all(v == 0.0 for v, _ in rb.link.sent), "아무 말도 없는데 움직이라고 했다"


def test_gyro_error_blocks_start() -> None:
    """자이로 이상(16) 표시가 오면 센서를 다 훑었어도 출발하지 않는다."""
    gyro_bad = 16 | FLAG_FRONT_OK
    lines = [_status_line(ms=1000 + 20 * i, sweep=sweep * 10, front=-1, flags=gyro_bad, cmd=0, gyro=0)
             for i, sweep in enumerate([0, -75, -60, 60, 75, 60])]
    st = parse_status(lines[0])
    assert st.gyro_error, "16 플래그를 자이로 이상으로 못 읽었다"
    rb = Robot(_ScriptedLink(lines), np.array([1.0, 0.0]), cfg=RobotConfig(bounds=(-3, -3, 3, 3), grid_res=0.04))
    rb.step(0.02)
    assert rb.ready, "센서는 다 훑었는데 준비가 안 됐다 (75 도 기준 확인)"
    rb.plan_cycle()
    rb.control_cycle()
    assert rb.stats["plans"] == 0 and rb.link.sent[-1][0] == 0.0, "자이로 이상인데 출발하려 했다"
    assert rb._mode() == "gyro_error"


def test_fake_hardware_drive() -> None:
    """가짜 아두이노로 무작위 지도를 실제로 주행한다."""
    world, start, goal = S.make("random", 2005)
    fake = FakeArduino(world, start, rng=np.random.default_rng(1))
    d = goal - start[:2]
    c, s = math.cos(-start[2]), math.sin(-start[2])
    goal_local = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
    rb = Robot(fake, goal_local, cfg=RobotConfig(max_time=60))
    dt = 1.0 / rb.C.control_hz
    k = 0
    while rb.step(dt):
        if k % 5 == 0:
            rb.plan_cycle()
        rb.control_cycle()
        fake.advance(dt)
        k += 1
    assert rb.arrived, f"도착하지 못했다 (t={rb.t:.1f}s)"
    true_err = float(np.hypot(*(fake.vehicle.pose[:2] - goal)))
    assert true_err < 1.0, f"실제 위치가 목적지에서 {true_err:.2f} m 떨어졌다"


def test_buzzer_command_round_trip() -> None:
    """부저 명령 B,1 / B,0 형식과 검사값이 맞아야 한다."""
    from rccar.link import encode_buzzer
    on = encode_buzzer(True).decode()
    off = encode_buzzer(False).decode()
    assert on.startswith("B,1*") and on.endswith("\n"), f"켜기 형식: {on}"
    assert off.startswith("B,0*"), f"끄기 형식: {off}"
    body = "B,1"
    assert on == f"{body}*{_checksum(body):02X}\n"


def test_countdown_beeps_without_moving() -> None:
    """카운트다운은 부저를 여러 번 울리고, 그 동안 차를 움직이지 않는다 (watchdog 유지)."""
    import run_robot
    world, start, _ = S.make("random", 2005)
    fake = FakeArduino(world, start, rng=np.random.default_rng(0))
    # 명령을 한 번도 안 보낸 상태에서 카운트다운만 돌린다 (시간을 아끼려 2초)
    run_robot.countdown(fake, seconds=2, quiet=True)
    for _ in range(50):                      # 카운트다운 뒤 1초를 흘려 본다
        fake.advance(0.02)
    assert fake.beeps >= 2, f"부저가 거의 안 울림: {fake.beeps}회"
    assert abs(fake.vehicle.v) < 1e-6, "카운트다운 중 차가 움직였다 (부저가 watchdog 을 건드림)"


def test_watchdog_stops_car() -> None:
    """명령을 끊으면 차가 스스로 선다. 파이가 죽어도 차는 서야 한다."""
    world, start, _ = S.make("random", 2005)
    fake = FakeArduino(world, start, rng=np.random.default_rng(0))
    fake.send(0.8, 0.0)
    for _ in range(50):                      # 1 초 동안 명령을 계속 준다
        fake.send(0.8, 0.0)
        fake.advance(0.02)
    assert fake.vehicle.v > 0.3, "명령을 주는데 차가 안 움직인다"
    for _ in range(100):                     # 2 초 동안 명령을 끊는다
        fake.advance(0.02)
    assert abs(fake.vehicle.v) < 0.05, f"명령이 끊겼는데 아직 {fake.vehicle.v:.2f} m/s 로 간다"


def test_manual_override() -> None:
    """사람이 조종기로 가져가면 주행 프로그램은 명령을 멈춘다."""
    world, start, goal = S.make("random", 2005)
    fake = FakeArduino(world, start, rng=np.random.default_rng(0))
    rb = Robot(fake, np.array([3.0, 0.0]), cfg=RobotConfig(max_time=10))
    dt = 1.0 / rb.C.control_hz
    for k in range(60):
        rb.step(dt)
        if k % 5 == 0:
            rb.plan_cycle()
        rb.control_cycle()
        fake.advance(dt)
    fake.manual = True
    for k in range(30):
        rb.step(dt)
        rb.control_cycle()
        fake.advance(dt)
    assert rb.manual, "사람이 가져간 것을 알아채지 못했다"
    assert abs(fake._cmd[0]) < 1e-9, "사람 조종 중인데 속도를 보냈다"


def _fake_drive(seed, has_rear, log_path=None, max_time=60):
    """가짜 아두이노로 한 판 달리고, 보낸 속도 명령을 전부 모아 돌려줌."""
    from rccar.hybrid import HybridParams
    world, start, goal = S.make("random", seed)
    fake = FakeArduino(world, start, rng=np.random.default_rng(1), has_rear=has_rear)
    sent = []
    original_send = fake.send
    fake.send = lambda v, s: (sent.append(v), original_send(v, s))
    d = goal - start[:2]
    c, s = math.cos(-start[2]), math.sin(-start[2])
    goal_local = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
    rb = Robot(fake, goal_local, cfg=RobotConfig(max_time=max_time, has_rear_sensor=has_rear),
               hp=HybridParams(allow_reverse=has_rear))
    if log_path:
        rb.open_log(log_path)
    k = 0
    while rb.step(0.02):
        if k % 5 == 0:
            rb.plan_cycle()
        rb.control_cycle()
        fake.advance(0.02)
        k += 1
    rb.close_log()
    return rb, sent


def test_no_rear_sensor_never_reverses() -> None:
    """뒤 센서가 없으면 후진 명령을 한 번도 보내면 안 됨. 갇히면 서서 '갇힘' 으로 끝나야 함."""
    for seed in (2005, 2016, 17, 4023):
        rb, sent = _fake_drive(seed, has_rear=False)
        assert min(sent) >= 0.0, f"지도 {seed}: 뒤를 못 보는데 후진 명령 {min(sent):.2f} m/s"
        assert rb.arrived or rb.stuck, f"지도 {seed}: 도착도 갇힘도 아닌 채 끝남 ({rb.end_reason})"
    # 예전엔 4023 번이 갇혔는데, 지도 칠하는 버그를 고친 뒤 (2026-10-27) 빠져나가서 4000 번으로 바꿈
    rb, _ = _fake_drive(4000, has_rear=False)
    assert rb.stuck, "지도 4000 은 뒤 센서 없이 갇혀야 함 (갇힘 판정이 안 켜짐)"


def test_drive_log_written() -> None:
    """주행 기록 CSV 에 머리글과 50 Hz 줄이 쌓여야 함."""
    import os
    import tempfile
    path = os.path.join(tempfile.mkdtemp(), "run.csv")
    rb, _ = _fake_drive(2005, has_rear=False, log_path=path)
    with open(path, encoding="utf-8") as file:
        lines = file.read().splitlines()
    assert lines[0].split(",") == list(Robot.LOG_COLUMNS), f"머리글이 다름: {lines[0]}"
    rows = [line.split(",") for line in lines[1:]]
    assert all(len(row) == len(Robot.LOG_COLUMNS) for row in rows), "칸 수가 다른 줄이 있음"
    expected = rb.t * 50
    assert len(rows) > 0.8 * expected, f"줄이 너무 적음: {len(rows)} (약 {expected:.0f} 예상)"
    modes = {row[Robot.LOG_COLUMNS.index("mode")] for row in rows}
    assert "drive" in modes and "waiting" in modes, f"주행 상태가 안 남음: {modes}"


def test_small_car_arena() -> None:
    """22 cm 차 설정으로 1.5 m 시험장 지도 0~9 번을 다 도착하고, 기둥에 안 닿아야 함."""
    from run_arena_check import drive_one
    for seed in range(10):
        robot, closest, miss = drive_one(seed, 0.5)
        assert robot.arrived, f"시험장 {seed}: 도착 못 함 ({robot.end_reason})"
        assert closest > 0.0, f"시험장 {seed}: 기둥에 {-closest * 100:.1f} cm 닿음"
        assert miss < 0.12, f"시험장 {seed}: 목적지에서 {miss * 100:.1f} cm 떨어져 섬"


TESTS = [
    ("명령 문장 형식", test_cmd_round_trip),
    ("상태 문장 해석", test_status_parse),
    ("깨진 문장 버리기", test_bad_frames_rejected),
    ("신호 없으면 거리 없음", test_no_signal_means_no_range),
    ("가짜 장치로 실제 주행", test_fake_hardware_drive),
    ("명령이 끊기면 정지", test_watchdog_stops_car),
    ("사람 조종 우선", test_manual_override),
    ("뒤 센서 없으면 후진 안 함", test_no_rear_sensor_never_reverses),
    ("주행 기록 CSV", test_drive_log_written),
    ("부저 명령 형식", test_buzzer_command_round_trip),
    ("카운트다운 삑삑, 차는 멈춤", test_countdown_beeps_without_moving),
    ("22 cm 차, 1.5 m 시험장 10 개", test_small_car_arena),
    ("아두이노가 조용하면 길 안 찾음", test_silent_arduino_no_planning),
    ("자이로 이상이면 출발 안 함", test_gyro_error_blocks_start),
    ("시험용 R 명령 형식", test_run_command_format),
    ("출발 신호를 받으면 지도를 새로 시작", test_goal_restarts_map),
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
    print("\n실물 준비 전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(main())
