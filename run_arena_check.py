#!/usr/bin/env python3
"""
run_arena_check.py -- 가짜 아두이노로 1.5 m 시험장 지도 여러 개를 실제 주행 코드 그대로 돌려 봄

  python run_arena_check.py                 # 시험장 지도 30 개, 최고 속도 0.5 m/s
  python run_arena_check.py --count 10 --max-speed 0.4

run_robot.py 와 같은 설정(car_specs.small_car_settings)과 같은 차(car_specs.vehicle_params)를 씀.
지도마다 도착했는지, 차체가 기둥에 가장 가까이 간 거리(음수면 닿음), 실제로 멈춘 곳과 목적지 사이 거리를 찍음.
보고서의 '실차 전에 가상으로 확인한 결과' 표에 씀.
"""

import argparse
import math

import numpy as np

from rccar import car_specs
from rccar import scenarios as S
from rccar.error_split import BodyOutline
from rccar.fake_arduino import FakeArduino
from rccar.robot import Robot


def drive_one(seed, max_speed):
    world, start, goal = S.make("arena", seed)
    car = car_specs.vehicle_params()
    fake = FakeArduino(world, start, veh=car, rng=np.random.default_rng(seed),
                       has_rear=car_specs.has_rear_sensor(), front_spec=car_specs.FRONT_SENSOR,
                       motor_min_speed=car_specs.min_drive_speed_mps())
    # 목적지를 출발할 때의 차 기준 좌표로 바꿈 (run_robot.py 와 같음)
    offset = goal - start[:2]
    c, s = math.cos(-start[2]), math.sin(-start[2])
    goal_local = np.array([c * offset[0] - s * offset[1], s * offset[0] + c * offset[1]])
    robot = Robot(fake, goal_local, veh=car,
                  **car_specs.small_car_settings(max_speed=max_speed, max_time=40.0))

    outline = BodyOutline(car)
    dt = 1.0 / robot.C.control_hz
    plan_every = max(1, int(robot.C.control_hz / robot.C.plan_hz))
    closest = math.inf
    tick = 0
    while robot.step(dt):
        if tick % plan_every == 0:
            robot.plan_cycle()
        robot.control_cycle()
        fake.advance(dt)
        closest = min(closest, outline.clearance(world, fake.vehicle.pose))
        tick += 1
    miss = float(np.hypot(*(fake.vehicle.pose[:2] - goal)))
    return robot, closest, miss


def main():
    ap = argparse.ArgumentParser(description="가상 1.5 m 시험장 주행 확인")
    ap.add_argument("--count", type=int, default=30)
    ap.add_argument("--seed", type=int, default=0, help="첫 지도 번호")
    ap.add_argument("--max-speed", type=float, default=car_specs.DRIVE_MAX_SPEED_MPS)
    args = ap.parse_args()

    arrived = touched = 0
    closest_all = math.inf
    for seed in range(args.seed, args.seed + args.count):
        robot, closest, miss = drive_one(seed, args.max_speed)
        arrived += robot.arrived
        touched += closest < 0
        closest_all = min(closest_all, closest)
        print(f"지도 {seed:3d}  {robot.end_reason:<26} {robot.t:5.1f}초  "
              f"기둥까지 최소 {closest * 100:+5.1f} cm  목적지와 {miss * 100:5.1f} cm", flush=True)
    print(f"\n도착 {arrived}/{args.count}, 기둥에 닿음 {touched}/{args.count}, "
          f"가장 가까이 간 거리 {closest_all * 100:+.1f} cm")


if __name__ == "__main__":
    main()
