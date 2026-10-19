#!/usr/bin/env python3
"""
run.py -- 실행 진입점

  python run.py                          # pillars 시나리오를 창으로 본다
  python run.py --scenario trap           # 다른 시나리오
  python run.py --planner tangent         # 예전 접선 그래프 계획기로 본다
  python run.py --sensor tof_sweep_rear   # 싼 센서 조합으로 본다 (--list 로 목록)
  python run.py --record out.mp4          # 동영상으로 저장 (recordings 폴더에)
  python run.py --bench                   # 창 없이 전 시나리오 성능만 측정
  python run.py --list                    # 시나리오 목록

주요 옵션
  --planner      hybrid(기본) 또는 tangent
  --sensor       lidar(기본) 또는 sensors.py 의 센서 조합. hybrid 에서만 쓸 수 있다
  --speed        1.0 이 실시간. 2.0 이면 2배속
  --no-graph     접선 그래프를 그리지 않는다 (훨씬 가볍다)
  --seed         노이즈 시드
  --car          sim(기본, 시뮬레이터 기본 차) 또는 ours(rccar/car_specs.py 에 잰 우리 차)
"""

import argparse
import os
import sys
import numpy as np

from rccar import car_specs
from rccar import scenarios as S
from rccar.simulator import Simulator, SimConfig
from rccar.hybrid_sim import HybridSimulator
from rccar.vehicle import VehicleParams
from rccar.world import Lidar
from rccar.planner import PlannerParams
from rccar.hybrid import HybridParams
from rccar.sensors import SENSOR_LABEL, SENSOR_SETS, make_sensor

# 앞만 보는 센서는 뒤를 못 보므로 후진 경로를 만들지 않는다.
# 뒤 센서가 있는 조합만 후진을 허용하되 천천히 간다.
REVERSE_OK = {"lidar360": None, "tof_sweep_rear": 0.25}


REC_DIR = "recordings"          # 동영상과 정지 화면을 모아 두는 폴더


def out_path(name):
    """폴더 없이 파일 이름만 주면 recordings 폴더에 저장한다."""
    if name is None or os.path.dirname(name):
        return name
    os.makedirs(REC_DIR, exist_ok=True)
    return os.path.join(REC_DIR, name)


def build(args):
    world, start, goal = S.make(args.scenario, args.seed)
    rng = np.random.default_rng(args.seed)
    veh = car_specs.vehicle_params() if args.car == "ours" else VehicleParams()
    cfg = SimConfig(max_time=args.max_time, max_verts=args.verts)

    if args.sensor != "lidar":
        if args.planner != "hybrid":
            raise SystemExit("--sensor 는 --planner hybrid 에서만 쓸 수 있다")
        sensor = make_sensor(args.sensor, veh, rng=rng)
        hp = HybridParams(allow_reverse=args.sensor in REVERSE_OK)
        sim = HybridSimulator(world, start, goal, cfg=cfg, veh=veh, lidar=sensor,
                              plan_params=PlannerParams(), seed=args.seed, hparams=hp)
        if REVERSE_OK.get(args.sensor) is not None:
            sim.follower.rev_speed = REVERSE_OK[args.sensor]
        return sim

    lidar = Lidar(n_beams=args.beams, max_range=args.range,
                  sigma_r=args.noise, rng=rng)
    cls = HybridSimulator if args.planner == "hybrid" else Simulator
    return cls(world, start, goal, cfg=cfg, veh=veh, lidar=lidar,
               plan_params=PlannerParams(), seed=args.seed)


def main():
    ap = argparse.ArgumentParser(description="지도 없는 환경의 RC카 자율주행 시뮬레이터")
    ap.add_argument("--scenario", default="pillars", help="시나리오 이름")
    ap.add_argument("--planner", default="hybrid", choices=("hybrid", "tangent"),
                    help="hybrid: 하이브리드 A* (기본). tangent: 예전 접선 그래프")
    ap.add_argument("--sensor", default="lidar", choices=("lidar",) + SENSOR_SETS,
                    help="lidar: 기존 360도 라이다 (기본). 나머지는 싼 센서 조합")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--car", default="sim", choices=("sim", "ours"),
                    help="sim: 시뮬레이터 기본 차, ours: car_specs.py 에 잰 우리 차")
    ap.add_argument("--speed", type=float, default=5.0, help="재생 배속")
    ap.add_argument("--record", default=None, help="저장할 mp4/gif 경로")
    ap.add_argument("--snapshot", default=None, help="정지 화면 저장 경로(png)")
    ap.add_argument("--snapshot-at", type=float, default=6.0, help="정지 화면 시각(s)")
    ap.add_argument("--no-graph", action="store_true", help="접선 그래프 숨김")
    ap.add_argument("--verts", type=int, default=4,
                    help="부풀린 도형의 최대 꼭짓점 수. 올리면 둥글어 보이지만 "
                         "계획기 시야가 좁아진다. pillars 시연에만 10 권장")
    ap.add_argument("--headless", action="store_true", help="창 없이 실행")
    ap.add_argument("--bench", action="store_true", help="전 시나리오 성능 측정")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--max-time", type=float, default=120.0)
    ap.add_argument("--beams", type=int, default=270)
    ap.add_argument("--range", type=float, default=8.0)
    ap.add_argument("--noise", type=float, default=0.02)
    ap.add_argument("--fps", type=int, default=40)
    args = ap.parse_args()

    if args.list:
        print("사용 가능한 시나리오:")
        for k in S.ALL:
            print("  ", k)
        print("사용 가능한 센서 조합 (--sensor):")
        print("   lidar            기존 360도 라이다 (기본)")
        for k in SENSOR_SETS:
            print(f"   {k:16s} {SENSOR_LABEL[k]}")
        return

    if args.bench:
        print(f"planner: {args.planner}")
        print(f"{'scenario':10s} {'arrive':>6s} {'coll':>5s} {'time':>7s} "
              f"{'dist':>7s} {'plan avg':>9s} {'p95':>7s}")
        for name in S.ALL:
            args.scenario = name
            sim = build(args)
            r = sim.run()
            print(f"{name:10s} {str(r['arrived']):>6s} {str(r['collided']):>5s} "
                  f"{r['time']:6.1f}s {r['distance']:6.1f}m "
                  f"{r['plan_ms_mean']:8.1f}ms {r['plan_ms_p95']:6.1f}ms")
        return

    sim = build(args)

    args.record = out_path(args.record)
    args.snapshot = out_path(args.snapshot)

    if args.snapshot:
        import matplotlib
        matplotlib.use("Agg")
        from rccar import viz
        while sim.t < args.snapshot_at and sim.step():
            pass
        viz.snapshot(sim, args.snapshot, show_graph=not args.no_graph)
        print("saved:", args.snapshot)
        return

    if args.headless:
        r = sim.run()
        for k, v in r.items():
            print(f"{k:16s} {v}")
        return

    import matplotlib
    if args.record:
        matplotlib.use("Agg")
    from rccar import viz

    spf = max(1, int(round(4 * args.speed)))
    r = viz.animate(sim, steps_per_frame=spf, save=args.record, fps=args.fps,
                    show_graph=not args.no_graph)
    print()
    for k, v in r.items():
        print(f"{k:16s} {v}")
    if args.record:
        print("saved:", args.record)


if __name__ == "__main__":
    sys.exit(main())
