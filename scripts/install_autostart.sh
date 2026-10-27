#!/bin/bash
# install_autostart.sh -- 오렌지파이가 켜질 때 start_car.sh 가 저절로 돌게 예약함 (한 번만 실행)
#
#   bash scripts/install_autostart.sh            # 예약 넣기
#   bash scripts/install_autostart.sh remove     # 예약 빼기
#
# 리눅스의 부팅 예약 목록(crontab)에 "@reboot ... start_car.sh" 한 줄을 넣음.
# 이미 들어 있으면 또 넣지 않음. 다른 예약 줄은 건드리지 않음.

REPO="$(cd "$(dirname "$0")/.." && pwd)"
SCRIPT="$REPO/scripts/start_car.sh"
LINE="@reboot \"$SCRIPT\" >> $HOME/car.log 2>&1"

# 지금 예약 목록. 아직 없으면 빈 목록
current="$(crontab -l 2>/dev/null)"
# start_car.sh 가 들어간 줄만 뺀 나머지
others="$(printf '%s\n' "$current" | grep -v 'start_car.sh')"

if [ "$1" = "remove" ]; then
  printf '%s\n' "$others" | sed '/^$/d' | crontab -
  echo "부팅 예약에서 start_car.sh 를 뺐음. 지금 돌고 있는 건 따로 꺼야 함:"
  echo "  pkill -f start_car.sh; pkill -f run_robot.py"
  exit 0
fi

chmod +x "$SCRIPT"
printf '%s\n%s\n' "$others" "$LINE" | sed '/^$/d' | crontab -
echo "부팅 예약을 넣었음. 지금 예약 목록:"
crontab -l
echo
echo "다음에 오렌지파이를 켜면 저절로 시작함. 기록은 $HOME/car.log"
echo "지금 바로 켜 보려면:  nohup $SCRIPT >> $HOME/car.log 2>&1 &"
