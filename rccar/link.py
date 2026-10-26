"""
link.py -- 오렌지파이와 아두이노 사이의 통신

왜 글자로 주고받나
  숫자를 압축해서 보내면 빠르지만, 사람이 눈으로 볼 수 없다. 시리얼 모니터에 찍어 보며
  고칠 수 있는 편이 훨씬 중요하다. 주고받는 양이 초당 2 KB 도 안 되므로 글자로 충분하다.

문장 형식 (줄바꿈으로 끝난다)

  아두이노 -> 파이   S,<ms>,<자이로>,<센서각>,<앞거리>,<뒷거리>,<실제명령>,<플래그>*<검사값>
    ms       아두이노가 켜진 뒤 지난 시간 (밀리초)
    자이로   z 축 각속도 (도/초 의 100 배, 정수)
    센서각   앞 ToF 가 지금 보고 있는 방향 (도의 10 배, 정수). 서보가 멈춘 뒤 잰 값이다
    앞거리   앞 ToF 거리 (mm). 못 맞히면 -1
    뒷거리   뒤 ToF 거리 (mm). 못 맞히면 -1
    실제명령 아두이노가 실제로 모터에 준 속도 (mm/s). 비상 정지가 걸리면 0 이 된다
    플래그   1 앞거리 유효, 2 뒷거리 유효, 4 사람이 조종기로 뺏음, 8 감시 정지

  파이 -> 아두이노   C,<속도>,<조향>*<검사값>
    속도     mm/s. 음수면 후진
    조향     도의 10 배, 정수. 왼쪽이 양수

  파이 -> 아두이노   B,<on>*<검사값>      부저. 1 이면 켜고 0 이면 끈다
    출발 전 카운트다운 삑삑에 쓴다. 이 명령은 watchdog 을 건드리지 않아 정지에 영향이 없다.

검사값
  * 앞의 글자들을 XOR 한 값을 두 자리 16 진수로 적는다. 선이 흔들려 글자가 깨지면 버린다.

감시 정지
  파이가 멈추거나 선이 빠지면 명령이 끊긴다. 아두이노는 마지막 명령을 받은 지
  watchdog_ms 가 지나면 스스로 속도를 0 으로 만든다. 이 판단은 파이와 무관하게 아두이노가 한다.
"""

from dataclasses import dataclass


BAUD = 115200
WATCHDOG_MS = 300

FLAG_FRONT_OK = 1
FLAG_REAR_OK = 2
FLAG_MANUAL = 4
FLAG_WATCHDOG = 8


def _checksum(payload: str) -> int:
    c = 0
    for ch in payload:
        c ^= ord(ch)
    return c & 0xFF


def encode_cmd(speed_mps: float, steer_rad: float) -> bytes:
    """파이 -> 아두이노 한 줄."""
    import math
    body = f"C,{int(round(speed_mps * 1000))},{int(round(math.degrees(steer_rad) * 10))}"
    return f"{body}*{_checksum(body):02X}\n".encode("ascii")


def encode_buzzer(on: bool) -> bytes:
    """부저를 켜거나 끄는 한 줄. 출발 전 카운트다운 삑삑에 쓴다."""
    body = f"B,{1 if on else 0}"
    return f"{body}*{_checksum(body):02X}\n".encode("ascii")


@dataclass
class Status:
    t_ms: int
    gyro_dps: float          # z 축 각속도 (도/초)
    sweep_rad: float         # 앞 ToF 가 보고 있는 방향 (라디안, 차체 기준)
    front_m: float           # 앞 거리 (m). 못 맞히면 inf
    rear_m: float            # 뒤 거리 (m). 못 맞히면 inf
    speed_mps: float         # 아두이노가 실제로 모터에 준 속도
    manual: bool             # 사람이 조종기로 가져갔는가
    watchdog: bool           # 명령이 끊겨 스스로 멈췄는가


def parse_status(line: str):
    """아두이노 한 줄을 Status 로. 형식이 틀리거나 검사값이 안 맞으면 None."""
    import math
    line = line.strip()
    if not line.startswith("S,") or "*" not in line:
        return None
    body, _, chk = line.rpartition("*")
    try:
        if _checksum(body) != int(chk, 16):
            return None
        _, ms, gyro, sweep, front, rear, cmd, flags = body.split(",")
        f = int(flags)
        fm = int(front) / 1000.0
        rm = int(rear) / 1000.0
        return Status(
            t_ms=int(ms),
            gyro_dps=int(gyro) / 100.0,
            sweep_rad=math.radians(int(sweep) / 10.0),
            front_m=fm if (f & FLAG_FRONT_OK) and fm > 0 else math.inf,
            rear_m=rm if (f & FLAG_REAR_OK) and rm > 0 else math.inf,
            speed_mps=int(cmd) / 1000.0,
            manual=bool(f & FLAG_MANUAL),
            watchdog=bool(f & FLAG_WATCHDOG),
        )
    except (ValueError, IndexError):
        return None


class SerialLink:
    """실제 USB 시리얼. pyserial 이 필요하다 (pip install pyserial)."""

    def __init__(self, port, baud=BAUD, timeout=0.02):
        try:
            import serial
        except ImportError as e:
            raise SystemExit("pyserial 이 없다. pip install pyserial") from e
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self._buf = b""

    def send(self, speed_mps, steer_rad):
        self.ser.write(encode_cmd(speed_mps, steer_rad))

    def beep(self, on):
        self.ser.write(encode_buzzer(on))

    def poll(self):
        """지금까지 도착한 줄들을 Status 목록으로. 깨진 줄은 버린다."""
        self._buf += self.ser.read(4096)
        out = []
        while b"\n" in self._buf:
            raw, _, self._buf = self._buf.partition(b"\n")
            line = raw.decode("ascii", "ignore").strip()
            if line.startswith("G,") and "*" in line:
                body, cs = line.split("*", 1)
                if f"{_checksum(body):02X}" == cs.upper():
                    self.goal_cm = int(body[2:])
                continue
            st = parse_status(raw.decode("ascii", "ignore"))
            if st is not None:
                out.append(st)
        return out

    def close(self):
        try:
            self.ser.write(encode_cmd(0.0, 0.0))
        finally:
            self.ser.close()
