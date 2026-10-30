# 코드 요약 (1쪽)

> 팀 Tesla / 사전 지도 없이 달리면서 지도를 만들어 목적지까지 가는 RC카. 자세한 내용은 [코드 보고서](CODE_REPORT.md)

## 구조

```text
Sharp 센서(서보로 ±75° 훑기) -> Arduino Nano -> Orange Pi 5 -> 주행 지도(4 cm 격자) -> 하이브리드 A*(10 Hz) -> 경로 따라가기(50 Hz) -> Arduino -> 모터, 조향
```

시뮬레이터와 실제 차가 **같은 코드**(`rccar/robot.py`)를 쓴다. 통신 상대만 진짜 USB(`SerialLink`)와 가짜 Arduino(`FakeArduino`)로 바꿔 끼운다.

## 변천사

| 단계 | 파일 | 핵심 | 한계 또는 결과 |
|---|---|---|---|
| 1 기하 모델 | `circle_model.py` | 판별식과 수선의 발로 원을 지나는지 판정, 접선으로 비켜 감 | 원끼리 겹치면 실패 (1000개 중 19개, 전부 겹침) |
| 2 접선 그래프 | `planner.py` | 접점을 역으로 한 노선도에서 A\*. 꺼낼 때만 충돌 검사 | 2,253 ms에서 24 ms. 회전 반경을 못 다룸 |
| 3 하이브리드 A\* | `hybrid.py` | 차가 그릴 수 있는 곡선 조각만 이어 붙임 | 가상 시험장 27/30 도착, 접촉 0, 8.3 ms |
| 4 실제 차 | `robot.py`, 펌웨어 | 실물에서 드러난 버그 수정 | 아래 표 |

```python
# 1단계의 핵심: 공통수학2의 판별식 + 수선의 발
line = line_through(start, end)
if not line_cuts_circle(line, circle):                  # 판별식 D > 0 이 아니면 안 지남
    return False
foot = foot_of_perpendicular((circle.x, circle.y), line)
return is_between(foot, start, end)                     # 수선의 발이 선분 위에 있어야 지남
```

## 실제 차에서 고친 것 (일부)

| 증상 | 원인 | 수정 |
|---|---|---|
| 지도가 좌우로 뒤집힘 | 센서 서보 방향이 반대 | `SWEEP_SERVO_DIR = -1` |
| 도는 양이 8배 | 자이로 범위 미설정, 다른 눈금으로 나눔 | 범위 ±500 설정, 눈금 65.5 |
| 나선을 그리며 감김 | 조향 방향이 반대 | `STEER_DIR = -1` |
| 3 cm 가고 96 cm 갔다고 믿음 | 시뮬레이터 속도(1.4 m/s)를 그대로 씀 | 바닥에서 재서 0.1 m/s로 |
| 자이로 고장 | 판이 I2C 선을 붙잡음 | Arduino가 $\omega = v\tan\delta / L$ 로 계산해 대신 보냄 |

**교훈:** 실물 버그 대부분은 알고리즘이 아니라 **부호와 단위**였다.

## 실행

```bash
python run_arena_check.py     # 가상 시험장 30개
python -m pytest tests        # 자동 시험 67개
```
