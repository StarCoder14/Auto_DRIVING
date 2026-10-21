/*
  sharp_calib.ino -- 샤프 적외선 거리 센서(GP2Y0A21) 교정용 임시 스케치

  목적: 알려진 거리에서 센서의 아날로그 생값(ADC)을 재서, 거리 변환식을 정한다.
        (계획서 6-다-4. 정식 펌웨어는 rccar_arduino 이고, 교정 끝나면 그걸 다시 올린다.)

  이 스케치가 정식 펌웨어와 다른 점
    - 센서 서보를 정면(0도)에 고정한다. 안 돌려야 같은 방향을 계속 잴 수 있다.
    - 모터와 조향은 건드리지 않는다 (안전).
    - 거리로 바꾸지 않고 아날로그 생값(0~1023)을 그대로 보낸다.
      생값을 32번 평균해 잡음을 줄인다.

  배선은 정식 펌웨어와 같다: 샤프 신호 A0, 센서 서보 D9.

  출력: 한 줄에 "RAW,<adc평균>" 를 10 Hz 로 보낸다.
*/

#include <Servo.h>

const uint8_t PIN_SHARP_FRONT = A0;
const uint8_t PIN_SWEEP_SERVO = 9;
const float   SWEEP_CENTER_DEG = 90.0;     // 정식 펌웨어와 같은 값. 정면이 90도

Servo sweepServo;

void setup() {
  Serial.begin(115200);
  sweepServo.attach(PIN_SWEEP_SERVO);
  sweepServo.write(SWEEP_CENTER_DEG);       // 정면에 고정
  pinMode(PIN_SHARP_FRONT, INPUT);
}

void loop() {
  long sum = 0;
  for (uint8_t i = 0; i < 32; i++) { sum += analogRead(PIN_SHARP_FRONT); delay(1); }
  int adc = sum / 32;
  Serial.print("RAW,");
  Serial.println(adc);
  delay(70);
}
