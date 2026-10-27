#!/bin/bash
# start_car.sh -- 오렌지파이가 켜지면 주행 프로그램을 띄워 두고 리모컨을 기다린다
#
# 등록 (한 번만):  chmod +x ~/start_car.sh
#                 crontab -e  에 아래 한 줄
#                 @reboot /home/orangepi/start_car.sh >> /home/orangepi/car.log 2>&1
# 끄기:           pkill -f start_car.sh; pkill -f run_robot.py
#
# 켜진 뒤 리모컨 숫자(cm) + ▶|| 를 누르면 출발한다. 누르기 전에는 움직이지 않는다.
# 도착하거나 끝나면 3 초 뒤 다시 기다린다. 포트를 다시 열 때 나노가 재부팅되며 자이로 영점을
# 다시 잡으므로, 출발 전 2 초는 차를 가만히 둘 것.
# 이게 도는 동안은 아두이노 포트를 쓰고 있어 업로드나 cat 이 안 된다. 먼저 끌 것.
cd ~/Auto_DRIVING
while true; do
  P=$(ls /dev/ttyUSB* 2>/dev/null | head -1)          # USB 를 다시 꽂으면 0 번, 1 번이 바뀐다
  if [ -n "$P" ]; then
    python3 run_robot.py --port $P --goal 99 0 --max-speed 0.5 --countdown 0 --max-time 3600 --quiet
  fi
  sleep 3
done
