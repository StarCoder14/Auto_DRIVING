# 무작위 지도를 끝없이 보여준다. 끝내려면 Ctrl+C.
import random, matplotlib.pyplot as plt, run
from argparse import Namespace
from rccar import viz

while True:
    sd = random.randrange(1000000)
    a = Namespace(scenario="random", seed=sd, sensor="tof_sweep_rear",
                  planner="hybrid", max_time=90.0, verts=4,
                  beams=270, range=8.0, noise=0.02)
    sim = run.build(a)
    v = viz.Viewer(sim)
    go = True
    while go:
        for _ in range(8):
            if not sim.step():
                go = False
                break
        v.update()
        plt.pause(0.001)
    plt.pause(1.5)
    r = sim.summary()
    print(sd, r["arrived"], r["collided"], round(r["time"], 1))
    plt.close(v.fig)
