"""
motor_test.py -- 바퀴 띄우고 구동 모터만 벤치 테스트한다.

왜 이 스크립트가 필요한가
  시리얼 모니터에서 손으로 C,500 한 줄만 치면 watchdog(0.3초) 때문에 모터가
  곧 멈춘다. 이 스크립트는 50 Hz 로 명령을 계속 보내 watchdog 을 안 걸리게 하고,
  아두이노가 '실제로 모터에 준 속도'와 플래그를 그대로 찍어 준다.
  그래서 "펌웨어가 모터에 보내고는 있는데 안 도는가(=하드 문제)"를 바로 가린다.

쓰는 법 (프로젝트 루트에서)
  python motor_test.py <포트> [속도 m/s] [시간 초]
  예) 윈도우 노트북 :  python motor_test.py COM9 0.5 3
      오렌지파이    :  python3 motor_test.py /dev/ttyUSB0 0.5 3

읽는 법
  applied= 가 500 근처로 뜨고 watchdog=False 인데도 모터가 안 돌면 -> 배선/드라이버/모터(하드) 문제.
  applied= 가 계속 0 이면 -> 명령이 안 먹음(watchdog=True 면 더 자주 보내야 함, 아니면 체크섬/포트).

반드시 바퀴를 띄우고 돌릴 것. 끝나면 정지 명령을 보내고 포트를 닫는다.
"""

import sys
import time

from rccar.link import SerialLink


def main():
    if len(sys.argv) < 2:
        print("사용법: python motor_test.py <포트> [속도 m/s] [시간 초]")
        print("예)    python motor_test.py COM9 0.5 3")
        return

    port = sys.argv[1]
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5   # m/s
    dur = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0     # 초

    link = SerialLink(port, show_ir=False)
    print(f"[{port}] {speed} m/s 를 {dur}초 동안 50 Hz 로 보낸다. 중단은 Ctrl+C.")
    print("바퀴 띄웠는지 확인!  (applied=실제 모터에 준 mm/s)")
    print("차 뒤에서 앞을 볼 때 두 바퀴가 다 앞으로 구르면 맞음.")
    print("  둘 다 뒤로 -> 펌웨어 DRIVE_DIR = -1 / 한쪽만 반대 -> 그 모터 선 두 가닥을 바꿔 꽂기")
    print("-" * 56)

    # 펌웨어는 켜지면 리모컨을 기다리며 모터를 막아 둠. 시험 동안만 풀어 줌
    link.run(True)
    t0 = time.time()
    last_print = 0.0
    try:
        while time.time() - t0 < dur:
            # 리모컨 잡음이 '아무 버튼 = 정지' 로 읽혀 기다림이 다시 걸리므로 매번 다시 풂.
            # 모터가 돌기 시작하자 잡음이 늘어 첫 0.1 초 만에 멈췄음 (2026-10-28)
            link.run(True)
            link.send(speed, 0.0)            # 조향 0, 전진만. 50 Hz 로 계속 보냄
            now = time.time()
            for st in link.poll():
                # 너무 많이 찍히지 않게 0.1초에 한 번만 출력
                if now - last_print >= 0.1:
                    last_print = now
                    print(f"applied={st.speed_mps * 1000:5.0f} mm/s   "
                          f"watchdog={st.watchdog}   manual={st.manual}   "
                          f"front={st.front_m if st.front_m != float('inf') else '-'}")
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n사용자 중단")
    finally:
        link.run(False)
        link.close()                         # 정지(C,0,0) 보내고 포트 닫음
        print("-" * 56)
        print("정지 명령 보내고 종료.")


if __name__ == "__main__":
    main()
