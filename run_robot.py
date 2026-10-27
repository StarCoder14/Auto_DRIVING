#!/usr/bin/env python3
"""
run_robot.py -- 실제 차를 움직이는 진입점 (오렌지파이에서 실행)

  python run_robot.py --port /dev/ttyUSB0 --goal 1 0     # 앞으로 1 m
  python run_robot.py --fake --seed 3                    # 부품 없이 1.5 m 시험장 3 번 지도로 시험
  python run_robot.py --port /dev/ttyUSB0 --goal 1.2 0.3 --max-speed 0.4

목적지는 '출발할 때의 차 기준' 좌표다. x 는 차가 향한 방향, y 는 왼쪽이다.
차는 자기 위치를 자이로와 명령 속도로 짐작한다. 엔코더가 없으므로 오래 달릴수록 어긋난다.

안전
  - 아두이노가 스스로 멈추는 감시 장치가 1차 방어선이다. 이 프로그램을 Ctrl+C 로 끊어도 차는 선다.
  - 처음에는 --max-speed 0.4 처럼 낮춰서 시작할 것.
  - 사람이 RC 조종기로 가져가면 이 프로그램은 명령을 멈춘다.
"""

import argparse
import os
import sys
import time

import numpy as np


def countdown(link, seconds, quiet=False):
    """출발 전 삑삑 카운트다운. 사람이 키보드와 모니터 선을 뽑을 시간을 준다.

    마지막 3 초는 빠르게 울려 곧 출발함을 알린다. 부저 명령은 watchdog 을 건드리지 않아
    이 동안에도 '명령이 끊긴' 상태라 모터는 돌지 않는다. 가짜 아두이노면 소리 대신 횟수만 센다.
    """
    def beep(duration):
        link.beep(True)
        time.sleep(duration)
        link.beep(False)

    for remaining in range(int(seconds), 0, -1):
        if not quiet:
            print(f"  출발까지 {remaining}초", flush=True)
        if remaining > 3:
            beep(0.1)                      # 1 초에 한 번 짧게
            time.sleep(0.9)
        else:
            for _ in range(3):             # 마지막 3 초는 빠르게 삑삑삑
                beep(0.07)
                time.sleep(0.093)
            time.sleep(0.5)
    beep(0.6)                              # 길게 한 번 울리고 출발
    if not quiet:
        print("  출발!", flush=True)


def main():
    ap = argparse.ArgumentParser(description="실제 RC카 주행")
    ap.add_argument("--port", default=None, help="아두이노 포트. 예 /dev/ttyUSB0, COM3")
    ap.add_argument("--fake", action="store_true", help="부품 없이 가짜 아두이노로 시험")
    ap.add_argument("--seed", type=int, default=0, help="--fake 일 때 쓸 지도 번호")
    ap.add_argument("--map", default="arena",
                    help="--fake 일 때 지도 종류. arena(1.5 m 시험장, 기본) 또는 random(5~10 m 방, 예전 큰 차용)")
    ap.add_argument("--goal", type=float, nargs=2, default=None,
                    metavar=("X", "Y"), help="출발 기준 목적지 (m)")
    ap.add_argument("--max-speed", type=float, default=0.5, help="최고 속도 (m/s). 가상 시험장 30 개를 0.5 로 확인함")
    ap.add_argument("--max-time", type=float, default=90.0)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--log", default=None, help="주행 기록 CSV 경로. 안 주면 logs/ 에 시각으로 이름을 지음")
    ap.add_argument("--no-log", action="store_true", help="주행 기록을 남기지 않음")
    ap.add_argument("--countdown", type=float, default=10.0,
                    help="출발 전 삑삑 카운트다운 시간(초). 0 이면 바로 시작")
    args = ap.parse_args()

    from rccar import car_specs
    from rccar.robot import Robot
    has_rear = car_specs.has_rear_sensor()
    # 22 cm 차, 1.5 m 시험장에 맞춘 설정. 뒤를 못 보면 후진 경로도 만들지 않음 (car_specs 안에서 처리)
    settings = car_specs.small_car_settings(max_speed=args.max_speed, max_time=args.max_time)
    cfg = settings["cfg"]
    # 실제 차를 움직이는 프로그램이라 늘 잰 값을 씀. 가짜 아두이노도 같은 차로 흉내 내야 시험이 의미 있음
    veh = car_specs.vehicle_params()

    if args.fake:
        from rccar import scenarios as S
        from rccar.fake_arduino import FakeArduino
        world, start, goal = S.make(args.map, args.seed)
        link = FakeArduino(world, start, veh=veh, rng=np.random.default_rng(args.seed),
                           has_rear=has_rear, front_spec=car_specs.FRONT_SENSOR,
                           motor_min_speed=car_specs.min_drive_speed_mps())
        if args.goal is None:                      # 지도의 목적지를 차 기준으로 바꾼다
            d = goal - start[:2]
            c, s = np.cos(-start[2]), np.sin(-start[2])
            goal_local = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
        else:
            goal_local = np.array(args.goal, float)
    else:
        if args.port is None:
            raise SystemExit("--port 를 주거나 --fake 로 시험할 것")
        if args.goal is None:
            raise SystemExit("--goal X Y 로 목적지를 줄 것")
        from rccar.link import SerialLink
        link = SerialLink(args.port)
        goal_local = np.array(args.goal, float)

    rb = Robot(link, goal_local, veh=veh, **settings)
    log_path = None
    if not args.no_log:
        log_path = args.log
        if log_path is None:
            os.makedirs("logs", exist_ok=True)
            stamp = time.strftime("run_%Y%m%d_%H%M%S") + ("_fake" if args.fake else "")
            log_path = os.path.join("logs", stamp + ".csv")
        rb.open_log(log_path)

    def tick(r, k):
        if args.quiet or k % 25:
            return
        p = r.pose
        print(f"t {r.t:5.1f}s  위치 ({p[0]:+.2f},{p[1]:+.2f}) 방향 {np.degrees(p[2]):+6.1f}도  "
              f"속도 {r.speed:+.2f} m/s  계획 {r.stats['plans']:3d}  정지 {r.stats['stops']:2d}"
              f"{'  사람조종' if r.manual else ''}", flush=True)

    # 출발 전 삑삑 카운트다운. 이 동안 차는 멈춰 있고, 사람은 키보드와 모니터 선을 뽑는다.
    # 가짜 아두이노는 소리 대신 울린 횟수만 세므로, 시험을 빠르게 하려면 --countdown 0 으로 끈다.
    if args.countdown > 0:
        if not args.quiet:
            print(f"출발 전 {int(args.countdown)}초 카운트다운 (선을 뽑으세요)", flush=True)
        countdown(link, args.countdown, quiet=args.quiet)

    if args.fake:
        dt = 1.0 / cfg.control_hz                  # 시계를 직접 돌린다 (실시간 아님)
        k = 0
        while rb.step(dt):
            if k % max(1, int(cfg.control_hz / cfg.plan_hz)) == 0:
                rb.plan_cycle()
            rb.control_cycle()
            tick(rb, k)
            link.advance(dt)
            k += 1
        ok = rb.arrived
    else:
        ok = rb.run(on_tick=tick)

    rb.close_log()
    print(f"\n결과: {rb.end_reason}  시간 {rb.t:.1f}s  계획 {rb.stats['plans']}회  "
          f"경로채택 {rb.stats['replans']}회  예측정지 {rb.stats['stops']}회")
    if log_path:
        print(f"주행 기록: {log_path}")
    link.close()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
