"""
viz.py -- 시각화

한 화면에 다음을 겹쳐 그린다.

  회색 실선   실제 세계 (정답). 알고리즘은 이걸 못 본다.
  연한 점     이번 스캔의 라이다 점
  주황 영역   차가 인식하고 부풀린 장애물 (계획이 실제로 보는 세계)
  얇은 회색선 접선 그래프에서 실제로 검사된 노선들
  파란 굵은선 지금 따라가는 경로
  빨간 점     전방주시점 (당근)
  차체        조향각까지 그린다

차가 믿는 위치(오도메트리)는 실제 위치와 서서히 벌어진다.
그림은 실제 좌표계에 그리므로, 인식 결과는 그 오차만큼 밀려 보인다.
그게 정상이고, 그 오차까지 견디는 게 목표다.
"""

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.font_manager as _fm


def _setup_font():
    """한글 폰트가 있으면 쓰고, 없으면 영문 라벨로 자동 전환한다."""
    have = {f.name for f in _fm.fontManager.ttflist}
    for name in ("NanumGothic", "Malgun Gothic", "AppleGothic",
                 "Noto Sans CJK KR", "Noto Sans CJK JP", "NanumBarunGothic"):
        if name in have:
            matplotlib.rcParams["font.family"] = name
            matplotlib.rcParams["axes.unicode_minus"] = False
            return True
    matplotlib.rcParams["axes.unicode_minus"] = False
    return False


KOREAN = _setup_font()


def _L(ko, en):
    return ko if KOREAN else en
from matplotlib.patches import Polygon as MplPolygon, Circle as MplCircle
from matplotlib.collections import LineCollection

from . import geometry as G

C_TRUE = "#9aa0a6"
C_SCAN = "#4dd0e1"
C_OBST = "#ff9800"
C_PATH = "#1e88e5"
C_GRAPH = "#c8c8c8"
C_CAR = "#212121"
C_GOAL = "#2e7d32"
C_TRAIL = "#90a4ae"
C_LOOK = "#e53935"
C_REV = "#8e24aa"

MODE_COLOR = {
    "drive": "#1e88e5", "explore": "#8e24aa", "grid": "#f57c00",
    "escape": "#e53935", "estop": "#b71c1c", "no_path": "#616161",
}


class Viewer:
    def __init__(self, sim, show_graph=True, show_grid=False, figsize=(9.2, 9.2),
                 fig=None):
        self.sim = sim
        self.show_graph = show_graph
        self.show_grid = show_grid

        # 하이브리드 A* 시뮬레이터는 planner_name 이 'hybrid' 다.
        # 둘은 '계획기가 보는 세계'가 다르다. 접선 그래프는 부풀린 도형을, 하이브리드 A* 는
        # 점유 격자를 본다. 그림도 각자 실제로 보는 것을 그린다.
        self.hybrid = getattr(sim, "planner_name", "tangent") == "hybrid"
        # fig 를 주면 그 창을 비우고 다시 쓴다. 지도를 이어서 보여줄 때
        # 창이 매번 새로 뜨지 않도록 하기 위한 것이다.
        if fig is None:
            self.fig, self.ax = plt.subplots(figsize=figsize)
        else:
            self.fig = fig
            self.fig.clf()
            self.ax = self.fig.add_subplot(111)
        mgr = getattr(self.fig.canvas, "manager", None)
        if mgr is not None:
            mgr.set_window_title(
                "RC car - hybrid A* navigation" if self.hybrid
                else "RC car - tangent graph navigation")
        # 실제 활동 범위에 맞춰 화면을 잡는다 (지도 경계 전체는 너무 넓다)
        pts = [np.array(sim.vehicle.pose[:2]), np.asarray(sim.goal, float)]
        for c, r in sim.world.circles:
            pts += [c - r, c + r]
        for a, b in sim.world.walls:
            pts += [a, b]
        P = np.array(pts)
        lo, hi = P.min(axis=0) - 2.2, P.max(axis=0) + 2.2
        span = float(max(hi - lo))
        mid = 0.5 * (lo + hi)
        self.ax.set_xlim(mid[0] - span / 2, mid[0] + span / 2)
        self.ax.set_ylim(mid[1] - span / 2, mid[1] + span / 2)
        self.ax.set_aspect("equal")
        self.ax.set_facecolor("#fbfbfb")
        self.ax.grid(alpha=0.15, linewidth=0.5)
        self.ax.tick_params(labelsize=8)

        self._draw_truth()

        # 갱신되는 것들
        self.grid_im = None
        self.scan = self.ax.plot([], [], ".", ms=3.0, color=C_SCAN, zorder=3)[0]
        self.graph_lc = LineCollection([], colors=C_GRAPH, linewidths=0.35,
                                       alpha=0.55, zorder=2)
        self.ax.add_collection(self.graph_lc)
        self.obst_lc = LineCollection([], colors=C_OBST, linewidths=1.2,
                                      alpha=0.9, zorder=4)
        self.ax.add_collection(self.obst_lc)
        self.obst_patches = []
        self.occ_pts = self.ax.plot([], [], "s", ms=2.2, color=C_OBST,
                                    alpha=0.75, zorder=4)[0]
        # 싼 거리 센서의 원뿔 (sensors.py). 라이다는 광선이 많아 그리지 않는다.
        self.cone_sensor = hasattr(sim.lidar, "measure") and len(sim.lidar.angles) <= 40
        self.sensor_lc = LineCollection([], linewidths=0.9, alpha=0.8, zorder=5)
        self.ax.add_collection(self.sensor_lc)
        self.path_line = self.ax.plot([], [], "-", lw=2.4, color=C_PATH,
                                      alpha=0.9, zorder=6)[0]
        self.rev_line = self.ax.plot([], [], "-", lw=2.4, color=C_REV,
                                     alpha=0.9, zorder=6)[0]
        self.trail_line = self.ax.plot([], [], "-", lw=1.2, color=C_TRAIL,
                                       alpha=0.8, zorder=5)[0]
        self.look_pt = self.ax.plot([], [], "o", ms=5, color=C_LOOK, zorder=8)[0]
        self.body = MplPolygon(np.zeros((4, 2)), closed=True, fc="#eceff1",
                               ec=C_CAR, lw=1.6, zorder=9)
        self.ax.add_patch(self.body)
        self.wheels = [self.ax.plot([], [], "-", lw=2.6, color=C_CAR,
                                    solid_capstyle="round", zorder=10)[0]
                       for _ in range(2)]
        self.heading = self.ax.plot([], [], "-", lw=1.0, color=C_CAR,
                                    alpha=0.5, zorder=9)[0]
        self.goal_mk = self.ax.plot(*sim.goal, "*", ms=17, color=C_GOAL,
                                    zorder=7)[0]
        self.goal_ring = MplCircle(sim.goal, sim.ctrl.P.goal_tol, fill=False,
                                   ec=C_GOAL, lw=1.0, ls="--", alpha=0.6, zorder=7)
        self.ax.add_patch(self.goal_ring)

        self.hud = self.ax.text(
            0.015, 0.985, "", transform=self.ax.transAxes, va="top", ha="left",
            fontsize=8.5, family="monospace", zorder=12,
            bbox=dict(boxstyle="round,pad=0.45", fc="white", ec="#cfd8dc",
                      alpha=0.92))
        self.legend()

    # ------------------------------------------------------------ 정답 지도
    def _draw_truth(self):
        for c, r in self.sim.world.circles:
            self.ax.add_patch(MplCircle(c, r, fc="#e8eaed", ec=C_TRUE,
                                        lw=1.3, zorder=1))
        segs = [np.stack([a, b]) for a, b in self.sim.world.walls]
        if segs:
            self.ax.add_collection(LineCollection(segs, colors=C_TRUE,
                                                  linewidths=2.6, zorder=1))

    def legend(self):
        import matplotlib.lines as mlines
        h = [
            mlines.Line2D([], [], color=C_TRUE, lw=2.4,
                          label=_L("실제 장애물 (정답)", "ground truth")),
            mlines.Line2D([], [], color=C_SCAN, marker=".", ls="",
                          label=_L("라이다 점", "lidar points")),
        ]
        if self.hybrid and self.cone_sensor:
            h += [mlines.Line2D([], [], color=C_SCAN, lw=1.2,
                                label=_L("센서 시야 (회색: 아무것도 없음)",
                                         "sensor cone (grey: nothing)"))]
        if self.hybrid:
            h += [
                mlines.Line2D([], [], color=C_OBST, marker="s", ls="", ms=4,
                              label=_L("점유 격자 (계획기가 보는 장애물)",
                                       "occupancy grid")),
                mlines.Line2D([], [], color=C_GRAPH, lw=1.0,
                              label=_L("탐색 나무", "search tree")),
                mlines.Line2D([], [], color=C_PATH, lw=2.4,
                              label=_L("계획 경로 (전진)", "planned path (forward)")),
                mlines.Line2D([], [], color=C_REV, lw=2.4,
                              label=_L("계획 경로 (후진)", "planned path (reverse)")),
            ]
        else:
            h += [
                mlines.Line2D([], [], color=C_OBST, lw=2.0,
                              label=_L("인식 + 부풀린 장애물", "perceived + inflated")),
                mlines.Line2D([], [], color=C_GRAPH, lw=1.0,
                              label=_L("접선 그래프", "tangent graph")),
                mlines.Line2D([], [], color=C_PATH, lw=2.4,
                              label=_L("계획 경로", "planned path")),
            ]
        h += [
            mlines.Line2D([], [], color=C_LOOK, marker="o", ls="",
                          label=_L("전방주시점", "lookahead point")),
        ]
        lg = self.ax.legend(handles=h, loc="lower right", fontsize=7.5,
                            framealpha=0.92)
        lg.set_zorder(12)

    # ------------------------------------------------------------ 갱신
    def update(self):
        sim = self.sim
        veh = sim.vehicle
        T = veh.belief_to_world()          # 믿음 -> 실제 (그리기 전용)

        # 라이다
        if len(sim.scan_pts_world):
            self.scan.set_data(*sim.scan_pts_world.T)
        else:
            self.scan.set_data([], [])

        # 인식된 장애물 (믿음 좌표계 -> 실제 좌표계)
        for p in self.obst_patches:
            p.remove()
        self.obst_patches = []
        outlines = []
        for g in sim.groups:
            o = G.compose(T, g.outline())
            outlines.append(np.vstack([o, o[:1]]))
            patch = MplPolygon(o, closed=True, fc=C_OBST, alpha=0.13, ec="none",
                               zorder=3)
            self.ax.add_patch(patch)
            self.obst_patches.append(patch)
        self.obst_lc.set_segments(outlines)

        # 센서 원뿔 (실제 세계 좌표. 센서는 실제 자세에서 잰다)
        if self.cone_sensor and getattr(sim, "last_measure", None) is not None:
            segs, cols = self._sensor_cones(sim)
            self.sensor_lc.set_segments(segs)
            self.sensor_lc.set_color(cols)

        # 점유 격자 (하이브리드 A* 가 실제로 보는 장애물)
        if self.hybrid:
            occ = sim.grid.occupied_mask()
            ii, jj = np.nonzero(occ)
            if len(ii):
                pts = sim.grid.origin + (np.stack([ii, jj], axis=1) + 0.5) * sim.grid.res
                self.occ_pts.set_data(*G.compose(T, pts).T)
            else:
                self.occ_pts.set_data([], [])

        # 접선 그래프
        if self.show_graph and sim.graph is not None:
            self.graph_lc.set_segments(
                [G.compose(T, e) for e in sim.graph.edges_for_draw(500)])
        else:
            self.graph_lc.set_segments([])

        # 경로
        self.rev_line.set_data([], [])
        if sim.path is not None and len(sim.path) > 1:
            path, dirs = sim.path, None
            if self.hybrid and getattr(sim, "path_poses", None) is not None:
                # 이미 지나온 부분은 빼고 차 앞의 남은 경로만 그린다
                i0 = sim.follower.global_index()
                path, dirs = sim.path[i0:], sim.path_dirs[i0:]
            W = G.compose(T, path) if len(path) > 1 else path
            if dirs is not None and len(dirs) == len(W) and (dirs < 0).any():
                # 전진 구간과 후진 구간을 따로 그린다. 끊어지는 곳은 NaN 으로 나눈다.
                fwd = W.copy()
                rev = W.copy()
                fwd_mask = dirs > 0
                rev_mask = ~fwd_mask
                # 구간 경계의 점은 양쪽에 모두 남겨 선이 이어져 보이게 한다
                edge = np.zeros(len(dirs), bool)
                edge[:-1] |= dirs[:-1] != dirs[1:]
                fwd[~(fwd_mask | edge)] = np.nan
                rev[~(rev_mask | edge)] = np.nan
                self.path_line.set_data(*fwd.T)
                self.rev_line.set_data(*rev.T)
            else:
                self.path_line.set_data(*W.T)
        else:
            self.path_line.set_data([], [])

        # 전방주시점
        lp = sim.ctrl.lookahead_pt
        if lp is not None and sim.mode not in ("escape", "estop"):
            w = G.compose(T, lp[None, :])[0]
            self.look_pt.set_data([w[0]], [w[1]])
        else:
            self.look_pt.set_data([], [])

        # 지나온 길
        tr = np.array(sim.trail)
        self.trail_line.set_data(tr[:, 0], tr[:, 1])

        # 차체와 바퀴
        self.body.set_xy(veh.footprint())
        self._draw_wheels(veh)
        x, y, th = veh.pose
        self.heading.set_data([x, x + 1.2 * np.cos(th)], [y, y + 1.2 * np.sin(th)])

        self.hud.set_text(self._hud_text())
        self.hud.get_bbox_patch().set_edgecolor(
            MODE_COLOR.get(sim.mode, "#cfd8dc"))
        return self._artists()

    def _sensor_cones(self, sim):
        S, m = sim.lidar, sim.last_measure
        x, y, th = sim.vehicle.pose
        c, s_ = np.cos(th), np.sin(th)
        segs, cols = [], []
        for k, i in enumerate(m.idx):
            mx, my = S.mounts[i]
            o = np.array([x + c * mx - s_ * my, y + s_ * mx + c * my])
            r = m.ranges[k]
            hit = np.isfinite(r)
            L = r if hit else S.max_ranges[i]
            half = 0.5 * S.cones[i]
            a = th + S.angles[i] + np.linspace(-half, half, 5)
            arc = o + L * np.stack([np.cos(a), np.sin(a)], axis=1)
            col = C_SCAN if hit else "#b0b0b0"
            segs += [np.stack([o, arc[0]]), np.stack([o, arc[-1]])]
            segs += [arc[j:j + 2] for j in range(len(arc) - 1)]
            cols += [col] * (2 + len(arc) - 1)
        return segs, cols

    def _draw_wheels(self, veh):
        P = veh.P
        x, y, th = veh.pose
        for k, side in enumerate((-1, 1)):
            hub = G.compose(veh.pose,
                            np.array([[P.wheelbase, side * P.width / 2]]))[0]
            a = th + veh.steer
            d = np.array([np.cos(a), np.sin(a)]) * 0.09
            self.wheels[k].set_data([hub[0] - d[0], hub[0] + d[0]],
                                    [hub[1] - d[1], hub[1] + d[1]])

    def _hud_text(self):
        s = self.sim
        if hasattr(s, "hud_text"):
            return s.hud_text()
        err = np.hypot(*(s.vehicle.pose[:2] - s.vehicle.odom[:2]))
        return (f"t {s.t:6.1f}s   mode {s.mode:<7s}\n"
                f"v {s.vehicle.v:+5.2f} m/s  steer {np.degrees(s.vehicle.steer):+5.1f}deg\n"
                f"rho {s.rho:.2f} m  (safe {s.rho_hard:.2f} m)\n"
                f"groups {len(s.groups):3d}   plan {s.plan_ms:5.1f} ms\n"
                f"front {s.front_clear:5.2f} m  odom err {err:4.2f} m")

    def _artists(self):
        return ([self.scan, self.graph_lc, self.obst_lc, self.occ_pts, self.sensor_lc,
                 self.rev_line, self.path_line,
                 self.trail_line, self.look_pt, self.body, self.heading,
                 self.hud] + self.wheels + self.obst_patches)


def animate(sim, steps_per_frame=4, interval=25, save=None, fps=40,
            max_frames=4000, show_graph=True, dpi=110):
    """
    실시간 창을 띄우거나(save=None) 동영상 파일로 저장한다.
    steps_per_frame 은 한 프레임당 시뮬레이션 스텝 수 (기본 4 = 25 fps 실시간).
    """
    from matplotlib import animation
    view = Viewer(sim, show_graph=show_graph)

    state = {"done": False}

    def frame(_):
        for _ in range(steps_per_frame):
            if not sim.step():
                state["done"] = True
                break
        return view.update()

    def frames():
        """차가 도착하거나 멈추면 거기서 끝낸다.

        예전에는 max_frames(4000) 장을 무조건 만들었다. 영상으로 저장하면 도착한 뒤에도
        같은 화면이 계속 찍혀서, 파일이 커지고 뒷부분이 정지 화면으로 채워졌다.
        """
        n = 0
        while n < max_frames and not state["done"]:
            yield n
            n += 1

    anim = animation.FuncAnimation(view.fig, frame, frames=frames,
                                   interval=interval, blit=False, repeat=False,
                                   cache_frame_data=False, save_count=max_frames)
    if save:
        writer = ("ffmpeg" if save.endswith(".mp4") else "pillow")
        anim.save(save, writer=writer, fps=fps, dpi=dpi)
        plt.close(view.fig)
    else:
        plt.show()
    return sim.summary()


def snapshot(sim, path, show_graph=True, dpi=130):
    """현재 상태를 그림 파일로 저장."""
    view = Viewer(sim, show_graph=show_graph)
    view.update()
    view.fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(view.fig)
