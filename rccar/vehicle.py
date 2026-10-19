"""
vehicle.py -- 차량 운동학 (자전거 모델)

RC카는 제자리에서 못 돈다. 앞바퀴를 꺾어서 돌기 때문에
  * 최소 회전 반경이 있고
  * 핸들도 순간이동하지 않는다 (조향 속도 제한)

이 두 가지가 '경로는 예쁜데 차가 못 따라가는' 문제의 원인이다.
그래서 시뮬레이터도 이 제약을 그대로 갖는다. 제약 없는 점 로봇을
움직이면 알고리즘의 진짜 문제가 안 보인다.

오도메트리(자기 위치 추정)도 흉내 낸다. 바퀴가 미끄러지고 자이로가
조금씩 틀어지므로, 차가 믿는 위치는 실제 위치와 서서히 벌어진다.
"""

import numpy as np
from . import geometry as G


class VehicleParams:
    def __init__(self,
                 wheelbase=0.33,      # 축거 (앞뒤 바퀴 간 거리, m)
                 width=0.28,          # 차폭 (m)
                 length=0.48,         # 전장 (m)
                 max_steer=np.radians(32.0),   # 최대 조향각
                 steer_rate=np.radians(180.0), # 조향 속도 (rad/s)
                 max_speed=1.4,       # 최대 전진 속도 (m/s)
                 max_reverse=0.6,     # 최대 후진 속도 (m/s)
                 max_accel=1.8,       # 최대 가속 (m/s^2)
                 max_decel=3.0,       # 최대 감속 (m/s^2)
                 max_lat_acc=2.2,     # 최대 횡가속도 (m/s^2)
                 odom_drift=0.006,    # 이동거리당 위치 오차 비율 (엔코더 기준)
                 dist_source="encoder",  # 'encoder' 또는 'command'
                 speed_scale_err=0.08,   # 명령 속도 대비 실제 속도의 비율 오차
                 gyro_bias_dps=0.0,      # 자이로 편향 (도/초). 0 이면 편향 없음
                 odom_yaw_drift=0.004,   # 회전량당 방위 오차 비율
                 rear_overhang=None,     # 뒷바퀴 축에서 차 뒤끝까지 (m). None 이면 앞뒤가 고르게 튀어나왔다고 봄
                 front_sensor_x=None,    # 앞 센서(서보 회전축) 위치, 뒷바퀴 축에서 앞으로 (m)
                 rear_sensor_x=None):    # 뒤 센서 위치, 뒷바퀴 축 기준 (m, 뒤쪽이 음수)
        self.wheelbase = wheelbase
        self.width = width
        self.length = length
        self.max_steer = max_steer
        self.steer_rate = steer_rate
        self.max_speed = max_speed
        self.max_reverse = max_reverse
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_lat_acc = max_lat_acc
        self.odom_drift = odom_drift
        # 거리를 어디서 얻는가.
        #   encoder : 바퀴가 실제로 굴러간 거리를 잰다
        #   command : 엔코더가 없어 '보낸 속도 x 시간'으로 짐작한다.
        #             배터리가 닳거나 바닥이 미끄러우면 실제 속도가 명령과 달라지고,
        #             가속하는 동안에는 명령보다 덜 간다. 그 오차가 그대로 쌓인다.
        self.dist_source = dist_source
        self.speed_scale_err = speed_scale_err
        self.gyro_bias_dps = gyro_bias_dps
        self.odom_yaw_drift = odom_yaw_drift
        self.rear_overhang = rear_overhang
        self.front_sensor_x = front_sensor_x
        self.rear_sensor_x = rear_sensor_x

    # 차체와 센서 위치는 전부 뒷바퀴 축 가운데를 0 으로 잼. 시뮬레이터의 pose 가 그 점임.
    # 예전엔 아래 기본식이 파일 다섯 곳에 복사돼 있었음. 실제 차를 재서 넣으려고 여기로 모음
    @property
    def rear_overhang_m(self):
        if self.rear_overhang is not None:
            return self.rear_overhang
        return 0.5 * (self.length - self.wheelbase) + 0.02

    @property
    def front_bumper_x(self):
        """앞 범퍼가 뒷바퀴 축에서 앞으로 얼마나 나가 있는가 (m)."""
        return self.length - self.rear_overhang_m

    @property
    def front_sensor_x_m(self):
        if self.front_sensor_x is not None:
            return self.front_sensor_x
        return self.front_bumper_x - 0.06

    @property
    def rear_sensor_x_m(self):
        if self.rear_sensor_x is not None:
            return self.rear_sensor_x
        return -self.rear_overhang_m + 0.02

    @property
    def min_turn_radius(self):
        """
        최소 회전 반경. 부풀림 반지름 rho 의 하한이 되는 값이다.
        이보다 작은 호는 아무리 핸들을 꺾어도 못 돌기 때문에,
        계획해봐야 실행할 수 없다.
        """
        return self.wheelbase / np.tan(self.max_steer)

    @property
    def body_radius(self):
        """차체 외접원 반지름. 부풀림의 출발점."""
        return 0.5 * float(np.hypot(self.length, self.width))


class Vehicle:
    def __init__(self, params: VehicleParams, pose, rng=None):
        self.P = params
        self.pose = np.array(pose, dtype=float)      # 실제 위치 (정답)
        self.odom = np.array(pose, dtype=float)      # 차가 믿는 위치
        self.v = 0.0
        self.steer = 0.0
        self.rng = rng or np.random.default_rng(0)
        # 이번 주행 동안 변하지 않는 오차 (배터리 상태, 바닥, 자이로 개체차)
        self._scale = 1.0 + self.rng.normal(0.0, params.speed_scale_err)
        self._gyro_bias = np.radians(self.rng.normal(0.0, params.gyro_bias_dps))

    # ------------------------------------------------------------ 갱신
    def step(self, cmd_speed, cmd_steer, dt):
        P = self.P
        # 조향은 순간이동하지 않는다
        cmd_steer = float(np.clip(cmd_steer, -P.max_steer, P.max_steer))
        d = np.clip(cmd_steer - self.steer, -P.steer_rate * dt, P.steer_rate * dt)
        self.steer += float(d)

        # 가감속 제한 (후진 허용: 좁은 곳에서 빠져나오려면 반드시 필요하다)
        cmd_speed = float(np.clip(cmd_speed, -P.max_reverse, P.max_speed))
        dv = cmd_speed - self.v
        lim = (P.max_accel if dv > 0 else P.max_decel) * dt
        self.v += float(np.clip(dv, -lim, lim))

        # 자전거 모델
        x, y, th = self.pose
        ds = self.v * dt
        dth = ds / P.wheelbase * np.tan(self.steer)
        self.pose = np.array([x + ds * np.cos(th + 0.5 * dth),
                              y + ds * np.sin(th + 0.5 * dth),
                              G.wrap_pi(th + dth)])

        # 오도메트리: 미끄러짐과 자이로 편차가 누적된다
        if P.dist_source == "command":
            n_ds = cmd_speed * dt * self._scale     # 엔코더 없음. 명령대로 갔다고 믿는다
        else:
            n_ds = ds * (1.0 + self.rng.normal(0.0, P.odom_drift))
        n_dth = dth * (1.0 + self.rng.normal(0.0, P.odom_yaw_drift)) \
            + self.rng.normal(0.0, 2e-4) + self._gyro_bias * dt
        ox, oy, oth = self.odom
        self.odom = np.array([ox + n_ds * np.cos(oth + 0.5 * n_dth),
                              oy + n_ds * np.sin(oth + 0.5 * n_dth),
                              G.wrap_pi(oth + n_dth)])

    # ------------------------------------------------------------ 보조
    @property
    def curvature(self):
        return np.tan(self.steer) / self.P.wheelbase

    def belief_to_world(self):
        """
        차의 믿음(오도메트리 좌표계)을 실제 세계 좌표계로 옮기는 변환.
        그림을 그릴 때만 쓴다. 알고리즘은 절대 이 정보를 못 본다.
        """
        return G.compose_pose(self.pose, G.invert_pose(self.odom))

    def footprint(self, pose=None):
        """차체 사각형 꼭짓점 (그리기용)."""
        P = self.P
        pose = self.pose if pose is None else pose
        # 뒷축 기준점에서 앞으로 치우친 차체
        rear = P.rear_overhang_m
        pts = np.array([[-rear, -P.width / 2], [P.length - rear, -P.width / 2],
                        [P.length - rear, P.width / 2], [-rear, P.width / 2]])
        return G.compose(pose, pts)
