#!/usr/bin/env python3
"""
run_forever.py -- 무작위 지도를 끝없이 이어서 보여준다

  python3 run_forever.py                       # 싼 센서 조합으로 계속 돌린다
  python3 run_forever.py --speed 2             # 2배속
  python3 run_forever.py --sensor lidar360     # 다른 센서로

창 하나를 계속 다시 써서, 지도가 바뀌어도 창이 새로 뜨지 않는다.
한 판이 끝나면 결과를 잠깐 보여주고 다음 지도로 넘어간다.

  끝내는 법 : 터미널에서 Ctrl+C, 또는 창을 닫는다.

시드는 매번 무작위로 뽑는다. 풀 수 없는 지도는 scenarios 쪽에서 알아서
다시 뽑으므로, 아무 시드나 넣어도 된다.
"""

import argparse
import random
import sys
from argparse import Namespace

import matplotlib.pyplot as plt

import run as runner
from rccar import viz


def maximize(fig):
    """창을 화면 가득 채운다. 뒷단(Tk/Qt/GTK)마다 방법이 달라 차례로 시도한다."""
    win = getattr(getattr(fig.canvas, "manager", None), "window", None)
    if win is None:
        return
    for attempt in (lambda: win.showMaximized(),          # Qt
                    lambda: win.state("zoomed"),          # Tk (윈도우)
                    lambda: win.attributes("-zoomed", 1), # Tk (리눅스)
                    lambda: win.maximize()):              # GTK
        try:
            attempt()
            return
        except Exception:
            continue


def make_args(a, seed):
    """run.build 가 읽는 항목만 채운 꾸러미."""
    return Namespace(scenario="random", seed=seed, sensor=a.sensor,
                     planner="hybrid", max_time=a.max_time, verts=4,
                     beams=270, range=8.0, noise=0.02)


def one_map(a, fig, seed):
    """지도 하나를 끝까지 보여준다. 창이 닫혔으면 False 를 준다."""
    sim = runner.build(make_args(a, seed))
    view = viz.Viewer(sim, show_graph=not a.no_graph, fig=fig)
    spf = max(1, int(round(4 * a.speed)))

    running = True
    while running:
        if not plt.fignum_exists(fig.number):
            return None
        for _ in range(spf):
            if not sim.step():
                running = False
                break
        view.update()
        plt.pause(0.001)

    # 결과를 보여주며 쉰다. 이 동안 Ctrl+C 로 편하게 끝낼 수 있다.
    r = sim.summary()
    left = a.hold
    print("  다음 지도까지 %.0f초 쉰다. 끝내려면 Ctrl+C." % left)
    while left > 0:
        if not plt.fignum_exists(fig.number):
            return None
        plt.pause(0.1)
        left -= 0.1
    return r


def main():
    ap = argparse.ArgumentParser(description="무작위 지도를 끝없이 보여준다")
    ap.add_argument("--sensor", default="tof_sweep_rear",
                    help="센서 조합 (기본: 실제 차에 쓸 싼 조합)")
    ap.add_argument("--speed", type=float, default=2.0, help="재생 배속")
    ap.add_argument("--hold", type=float, default=5.0,
                    help="한 판이 끝나고 다음 지도까지 쉬는 시간(초). "
                         "이 동안 Ctrl+C 로 끝낼 수 있다")
    ap.add_argument("--max-time", type=float, default=90.0,
                    help="한 판의 제한 시간(초)")
    ap.add_argument("--no-graph", action="store_true")
    ap.add_argument("--seed", type=int, default=None,
                    help="시드 뽑는 순서를 고정하고 싶을 때만 쓴다")
    a = ap.parse_args()

    rng = random.Random(a.seed)
    fig = plt.figure(figsize=(9.2, 9.2))
    plt.show(block=False)
    maximize(fig)

    n = 0
    arrived = 0
    print("무작위 지도를 계속 보여준다. 끝내려면 Ctrl+C 또는 창 닫기.\n")
    print(f"{'번호':>4s} {'시드':>6s} {'도착':>5s} {'충돌':>5s} "
          f"{'시간':>7s} {'거리':>7s}   누적")
    try:
        while True:
            seed = rng.randrange(1_000_000)
            r = one_map(a, fig, seed)
            if r is None:
                print("\n창이 닫혔다. 끝낸다.")
                break
            n += 1
            arrived += bool(r["arrived"])
            print(f"{n:4d} {seed:6d} {str(r['arrived']):>5s} "
                  f"{str(r['collided']):>5s} {r['time']:6.1f}s "
                  f"{r['distance']:6.1f}m   {arrived}/{n}")
    except KeyboardInterrupt:
        print("\n멈춘다.")
    finally:
        if n:
            print(f"총 {n} 판, 도착 {arrived} 판 ({100.0*arrived/n:.0f}%)")
        plt.close("all")


if __name__ == "__main__":
    sys.exit(main())
