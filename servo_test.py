"""
servo_test.py -- 조향 서보(D10)만 테스트한다. 바퀴가 좌우로 꺾이는지 본다.

구동 모터는 안 건드린다(속도 0). 그래서 모터 배터리가 없어도 된다.
watchdog(0.3초) 때문에 손으로 한 줄씩 치면 서보가 곧 중앙으로 돌아가므로,
이 스크립트가 50 Hz 로 계속 보내면서 왼 -> 중앙 -> 오른 -> 중앙 을 반복한다.

사용법 (프로젝트 루트에서, rccar 폴더 있는 곳)
  python servo_test.py COM9          (윈도우)
  python3 servo_test.py /dev/ttyUSB0 (오렌지파이)

보는 법
  바퀴(앞바퀴)가 왼쪽 -> 가운데 -> 오른쪽 으로 꺾였다 돌아오면 성공.
  - 아예 안 움직이면: 서보 신호선이 D10 인지, 서보 전원(빨강=5V, 갈/검=GND)이 연결됐는지 확인.
  - 움직이다 아두이노가 꺼지거나 포트 끊기면: 서보가 USB 5V 전류를 너무 먹는 것. 별도 5V 필요.
  - 차 뒤에서 앞을 볼 때 '왼쪽' 이라고 찍힐 때 앞바퀴가 오른쪽으로 꺾이면: 펌웨어 STEER_DIR = -1 로 바꿔 다시 올림.
    (예전엔 여기 '반대면 정상' 이라고 적혀 있었는데 틀렸음. 코드는 왼쪽을 양수로 씀)
  - 중앙이 안 맞으면 STEER_CENTER_DEG, 각도가 모자라면 STEER_GAIN 을 나중에 조정.

  python servo_test.py COM9 2    처럼 뒤에 숫자를 주면 그만큼만 반복하고 끝남 (안 주면 Ctrl+C 까지)

중단은 Ctrl+C. 끝나면 중앙으로 되돌리고 포트를 닫는다.
"""

import sys
import time
import math

from rccar.link import SerialLink


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"
    repeats = int(sys.argv[2]) if len(sys.argv) > 2 else None

    link = SerialLink(port, show_ir=False)
    print(f"[{port}] 조향 서보: 왼30도 -> 중앙 -> 오른30도 -> 중앙 반복. 중단 Ctrl+C.")
    print("앞바퀴가 좌우로 꺾이는지 보세요. (구동 모터는 안 움직임)")

    # (조향각도 deg, 유지 시간 s).  양수 = 왼쪽
    seq = [(30, 1.2), (0, 0.8), (-30, 1.2), (0, 0.8)]
    names = {30: "왼쪽", 0: "가운데", -30: "오른쪽"}
    # 펌웨어는 켜지면 리모컨을 기다리며 조향도 가운데로 묶어 둠. 시험 동안만 풀어 줌
    link.run(True)
    done = 0
    try:
        while repeats is None or done < repeats:
            done += 1
            for deg, hold in seq:
                print(f"  -> 조향 {deg:+d}도 ({names[deg]})", flush=True)
                t0 = time.time()
                while time.time() - t0 < hold:
                    link.send(0.0, math.radians(deg))   # 속도 0, 조향만. 50 Hz 로 계속
                    link.poll()                          # 상태줄 비우기(안 하면 버퍼 참)
                    time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n사용자 중단")
    finally:
        link.run(False)
        link.close()                                     # 중앙(C,0,0) 보내고 닫음
        print("중앙으로 되돌리고 종료.")


if __name__ == "__main__":
    main()
