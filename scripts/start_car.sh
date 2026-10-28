#!/bin/bash
# start_car.sh -- 오렌지파이가 켜지면 주행 프로그램을 띄워 두고 리모컨을 기다림
#
# 직접 등록하지 말고 scripts/install_autostart.sh 를 한 번 실행하면 됨 (부팅 예약까지 해 줌).
# 끄기:  pkill -f start_car.sh; pkill -f run_robot.py
# 기록:  ~/car.log  (이 파일이 찍은 것과 주행 프로그램 화면 출력이 다 쌓임)
#
# 켜진 뒤 리모컨 숫자(cm) + ▶|| 를 누르면 출발함. 누르기 전에는 움직이지 않음.
# 도착하거나 끝나면 3 초 뒤 다시 기다림. 포트를 다시 열 때 나노가 다시 켜지며(부저 삑삑) 자이로 영점을
# 2 초 동안 다시 잡으므로, 그동안 차를 가만히 둘 것.
# 이게 도는 동안은 아두이노 포트를 쓰고 있어 업로드나 cat 이 안 됨. 먼저 끌 것.

# 이 파일이 있는 곳의 한 칸 위가 저장소. 저장소를 어디 두든 동작함
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO" || exit 1

# 모니터 없이 부팅 예약으로 켜지면 파이썬이 한글을 못 찍고 죽음 (출력 방식이 ASCII 로 잡힘)
export PYTHONIOENCODING=utf-8

while true; do
  P=$(ls /dev/ttyUSB* 2>/dev/null | head -1)          # USB 를 다시 꽂으면 0 번, 1 번이 바뀜
  if [ -z "$P" ]; then
    sleep 3
    continue
  fi
  echo "$(date '+%F %T') 주행 프로그램 시작 (포트 $P)"
  started=$(date +%s)
  # 목적지 99 m 는 '아직 없음' 자리표시. 리모컨에서 숫자를 받으면 그걸로 바뀜
  # -u: 출력을 모아 두지 않고 바로 car.log 에 씀. 안 그러면 노트북에서 기록을 봐도 몇 분 늦은 내용만 보임
  # --wait-goal: 99 m 는 자리표시일 뿐이고 손 출발 신호를 받아야 달림. 켜지면 '삐이' 하고 기다림
  python3 -u run_robot.py --port "$P" --goal 99 0 --wait-goal --max-speed 0.5 --countdown 0 --max-time 3600
  code=$?
  echo "$(date '+%F %T') 주행 프로그램 끝 (종료 코드 $code)"
  # 켜자마자 죽으면 (코드 오류 등) 3 초마다 다시 켜며 삑삑거림. 그래서 10 초 안에 죽었으면 30 초 쉬고 다시 해 봄.
  # 단 USB 를 뽑아서 끝난 거면 쉬지 않음. 예전엔 업로드하려고 뽑았다 꽂는 동안 30 초를 쉬어서,
  # 그 사이에 한 손 출발 신호가 사라졌음 (2026-10-28)
  if [ -z "$(ls /dev/ttyUSB* 2>/dev/null)" ]; then
    echo "$(date '+%F %T') USB 가 빠졌음. 다시 꽂히길 기다림"
  elif [ $(( $(date +%s) - started )) -lt 10 ]; then
    echo "$(date '+%F %T') 너무 빨리 끝남. 30 초 뒤 다시 시도 (위 오류를 볼 것)"
    sleep 30
  else
    sleep 3
  fi
done
