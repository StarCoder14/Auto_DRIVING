"""
fake_arduino.py -- 부품 없이 주행 프로그램을 시험하는 가짜 아두이노

robot.py 는 아두이노에서 온 문장만 보고 판단한다. 그래서 문장을 똑같이 만들어 주면
부품이 없어도 전체를 돌려볼 수 있다. 안쪽에서는 시뮬레이터의 세계와 차량 모델을 쓴다.

무엇을 흉내 내는가
  - 50 Hz 로 상태 문장을 만든다 (실제 펌웨어와 같은 주기)
  - 앞 ToF 를 서보로 훑는다. 서보가 멈춘 뒤에 잰다
  - 자이로는 편향과 잡음이 있다
  - 명령이 끊기면 스스로 멈춘다 (감시 장치)
  - 통신 지연을 흉내 낸다

  - 센서 서보가 명령한 각도로 즉시 가지 않는다 (속도 제한과 떨림).
    펌웨어는 명령한 각도를 보고하므로, 아직 안 도착했으면 장애물이 엉뚱한 방향에 찍힌다.

무엇을 흉내 내지 못하는가
  변속기의 후진 지연, 바퀴 미끄러짐, 센서가 유리나 검은 면을 놓치는 것.
  이런 것은 실물에서 처음 만난다.
"""

import math
from collections import deque

import numpy as np

from .link import FLAG_FRONT_OK, FLAG_MANUAL, FLAG_REAR_OK, FLAG_WATCHDOG, Status
from .sensors import RangeSensorArray
from .vehicle import Vehicle, VehicleParams


class FakeArduino:
    """robot.py 가 쓰는 link 와 같은 모양(send, poll, close)을 가진 가짜 장치."""

    def __init__(self, world, start_pose, veh=None, rng=None, dt=0.002,
                 send_hz=50.0, latency_s=0.02, watchdog_ms=300,
                 gyro_bias_dps=0.2, gyro_noise_dps=0.3,
                 sweep_deg=(-90, 90), sweep_step=15, settle_s=0.090,
                 servo_rate_dps=300.0, servo_jitter_deg=1.5, has_rear=True):
        self.V = veh or VehicleParams()
        self.rng = rng or np.random.default_rng(0)
        self.world = world
        self.vehicle = Vehicle(self.V, start_pose, rng=self.rng)
        self.dt = dt
        self.send_period = 1.0 / send_hz
        self.latency = latency_s
        self.watchdog = watchdog_ms / 1000.0
        self.gyro_bias = math.radians(self.rng.normal(0.0, gyro_bias_dps))
        self.gyro_noise = math.radians(gyro_noise_dps)

        fx = self.V.front_sensor_x_m
        rx = self.V.rear_sensor_x_m
        cone = math.radians(27)
        self.front = RangeSensorArray("front", [[fx, 0.0]], [0.0], 3.0,
                                      cone=cone, subrays=9, noise_abs=0.02,
                                      noise_rel=0.01, rng=self.rng)
        self.rear = RangeSensorArray("rear", [[rx, 0.0]], [math.pi], 3.0,
                                     cone=cone, subrays=9, noise_abs=0.02,
                                     noise_rel=0.01, rng=self.rng)

        self.sweep_angles = np.radians(np.arange(sweep_deg[0], sweep_deg[1] + 1, sweep_step))
        self.sweep_i = 0
        self.sweep_dir = 1
        self.settle_s = settle_s
        self._settle_left = settle_s
        # 서보는 명령한 각도로 즉시 가지 않는다. 정해진 속도로 따라가고 조금 떤다.
        # 펌웨어는 '명령한 각도'를 보고하므로, 아직 안 도착했으면 그 차이만큼
        # 장애물이 엉뚱한 방향에 찍힌다.
        self.servo_rate = math.radians(servo_rate_dps)
        self.servo_jitter = math.radians(servo_jitter_deg)
        self.servo_actual = float(self.sweep_angles[0])
        self.servo_target = float(self.sweep_angles[0])
        self.sweep_err = []

        self.t = 0.0
        self._last_cmd_t = -1e9
        self._cmd = (0.0, 0.0)
        self._next_send = 0.0
        self._out = deque()          # (도착 시각, Status)
        self._front_m = math.inf
        self._rear_m = math.inf
        self.manual = False          # 사람이 조종기로 가져간 상황을 시험하려면 True
        self.has_rear = has_rear     # 뒤 센서를 안 달았으면 False. 펌웨어 USE_REAR_TOF 와 같음

    # ------------------------------------------------------------ link 흉내
    def send(self, speed_mps, steer_rad):
        self._cmd = (float(speed_mps), float(steer_rad))
        self._last_cmd_t = self.t

    def poll(self):
        out = []
        while self._out and self._out[0][0] <= self.t:
            out.append(self._out.popleft()[1])
        return out

    def close(self):
        self.send(0.0, 0.0)

    # ------------------------------------------------------------ 안쪽 진행
    def advance(self, seconds):
        """가짜 아두이노를 seconds 만큼 돌린다. robot.step 과 번갈아 부른다."""
        n = max(1, int(round(seconds / self.dt)))
        for _ in range(n):
            late = (self.t - self._last_cmd_t) > self.watchdog
            v, s = (0.0, 0.0) if (late or self.manual) else self._cmd
            self.vehicle.step(v, s, self.dt)
            self.t += self.dt

            # 서보가 목표 각도로 따라간다 (속도 제한)
            d = self.servo_target - self.servo_actual
            step = self.servo_rate * self.dt
            self.servo_actual += max(-step, min(step, d))

            # 앞 센서: 펌웨어가 정한 시간만큼 기다린 뒤 잰다
            self._settle_left -= self.dt
            if self._settle_left <= 0.0:
                # 실제로 보고 있는 방향 = 서보가 지금 가 있는 곳 + 떨림
                true_a = self.servo_actual + self.rng.normal(0.0, self.servo_jitter)
                self.front.angles[0] = true_a
                m = self.front.measure(self.world, self.vehicle.pose)
                self._front_m = float(m.ranges[0])
                self._front_a = self.servo_target        # 펌웨어가 아는 각도(명령값)
                # 잴 때의 각도 오차. 이 값이 크면 장애물이 엉뚱한 방향에 찍힌다
                self.sweep_err.append(true_a - self._front_a)
                self.servo_target = float(self.sweep_angles[self.sweep_i])
                self.sweep_i += self.sweep_dir
                if self.sweep_i >= len(self.sweep_angles):
                    self.sweep_i, self.sweep_dir = len(self.sweep_angles) - 2, -1
                elif self.sweep_i < 0:
                    self.sweep_i, self.sweep_dir = 1, 1
                self._settle_left = self.settle_s

            if self.t >= self._next_send:
                self._next_send = self.t + self.send_period
                if self.has_rear:
                    r = self.rear.measure(self.world, self.vehicle.pose)
                    self._rear_m = float(r.ranges[0])
                dth_true = self.vehicle.curvature * self.vehicle.v
                gyro = dth_true + self.gyro_bias + self.rng.normal(0.0, self.gyro_noise)
                flags = 0
                if np.isfinite(self._front_m):
                    flags |= FLAG_FRONT_OK
                if np.isfinite(self._rear_m):
                    flags |= FLAG_REAR_OK
                if self.manual:
                    flags |= FLAG_MANUAL
                if late:
                    flags |= FLAG_WATCHDOG
                st = Status(
                    t_ms=int(self.t * 1000),
                    gyro_dps=math.degrees(gyro),
                    sweep_rad=getattr(self, "_front_a", 0.0),
                    front_m=self._front_m if flags & FLAG_FRONT_OK else math.inf,
                    rear_m=self._rear_m if flags & FLAG_REAR_OK else math.inf,
                    speed_mps=self.vehicle.v,
                    manual=self.manual,
                    watchdog=late,
                )
                self._out.append((self.t + self.latency, st))
