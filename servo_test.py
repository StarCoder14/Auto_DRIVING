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
  - 좌우가 반대면: 정상(펌웨어상 왼쪽이 +). 중앙이 안 맞으면 STEER_CENTER_DEG, 각도가 모자라면 STEER_GAIN 을 나중에 조정.

중단은 Ctrl+C. 끝나면 중앙으로 되돌리고 포트를 닫는다.
"""

import sys
import time
import math

from rccar.link import SerialLink


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM9"

    link = SerialLink(port)
    print(f"[{port}] 조향 서보: 왼30도 -> 중앙 -> 오른30도 -> 중앙 반복. 중단 Ctrl+C.")
    print("앞바퀴가 좌우로 꺾이는지 보세요. (구동 모터는 안 움직임)")

    # (조향각도 deg, 유지 시간 s).  양수 = 왼쪽
    seq = [(30, 1.2), (0, 0.8), (-30, 1.2), (0, 0.8)]
    try:
        while True:
            for deg, hold in seq:
                print(f"  -> 조향 {deg:+d}도")
                t0 = time.time()
                while time.time() - t0 < hold:
                    link.send(0.0, math.radians(deg))   # 속도 0, 조향만. 50 Hz 로 계속
                    link.poll()                          # 상태줄 비우기(안 하면 버퍼 참)
                    time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n사용자 중단")
    finally:
        link.close()                                     # 중앙(C,0,0) 보내고 닫음
        print("중앙으로 되돌리고 종료.")


if __name__ == "__main__":
    main()
