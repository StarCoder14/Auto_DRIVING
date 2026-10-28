/*
  speed_test.ino -- 6V 건전지로 바꾼 뒤 바닥 속도를 재는 임시 스케치

  오렌지파이와 상관없이 아두이노 혼자 똑바로 2 초 달림. 오렌지파이가 보내는 말은 다 무시함.
  전기는 오렌지파이 USB 에서 받으면 되고, 노트북 선이 필요 없음.

  쓰는 법
    차 뒤 끝에 테이프를 붙임 -> 센서 정면 20 cm 안에 손을 2 초 대면 '삐'
    -> 손을 치우면 1 초마다 '삑' 세 번 -> 똑바로 2 초 달리고 멈추며 '삐이'
    -> 차 뒤 끝이 테이프에서 몇 cm 갔는지 잼
  출발할 때마다 힘이 100% -> 57% -> 100% -> 57% ... 로 바뀜.
  57% 는 예전 5V 때 쟀던 힘(145)과 같은 값이라 비교할 수 있음.

  확인이 끝나면 rccar_arduino 를 다시 올릴 것.
*/

#include <Servo.h>

const uint8_t PIN_SHARP_FRONT = A0;
const uint8_t PIN_SWEEP_SERVO = 9;
const uint8_t PIN_STEER_SERVO = 10;
const uint8_t PIN_MOTOR_A1A = 6;
const uint8_t PIN_MOTOR_A1B = 7;
const uint8_t PIN_BUZZER = 13;

// rccar_arduino 와 같은 값. 20 cm 안에 손이 있으면 이보다 큼
const int HAND_ADC_MIN = 276;
const unsigned long HAND_START_MS = 2000;
const unsigned long RUN_MS = 2000;
const int RUN_POWERS[2] = {255, 145};

Servo sweepServo;
Servo steerServo;
uint8_t runIndex = 0;

void beep(unsigned long ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

// rccar_arduino 의 전진과 같은 핀 쓰기. 두 모터가 같은 힘을 받음
void drive(int pwm) {
  analogWrite(PIN_MOTOR_A1A, pwm); analogWrite(5, pwm);
  analogWrite(PIN_MOTOR_A1B, 0); analogWrite(4, 0);
}

// 다섯 번 읽어 가운데 값을 씀. 샤프는 한 번씩 튀는 값이 많음
int readHandAdc() {
  int s[5];
  for (uint8_t i = 0; i < 5; i++) s[i] = analogRead(PIN_SHARP_FRONT);
  for (uint8_t i = 1; i < 5; i++)
    for (uint8_t j = i; j > 0 && s[j - 1] > s[j]; j--) { int t = s[j]; s[j] = s[j - 1]; s[j - 1] = t; }
  return s[2];
}

bool handIsThere() {
  return readHandAdc() >= HAND_ADC_MIN;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_MOTOR_A1A, OUTPUT);
  pinMode(PIN_MOTOR_A1B, OUTPUT);
  drive(0);
  sweepServo.attach(PIN_SWEEP_SERVO);
  steerServo.attach(PIN_STEER_SERVO);
  sweepServo.write(90);
  steerServo.write(90);
  beep(150); delay(150); beep(150);
  Serial.println("speed_test ready");
}

void loop() {
  // 손을 2 초 계속 대야 출발 준비가 됨. 중간에 0.3 초 넘게 비면 처음부터 셈
  unsigned long handSince = 0, handLastAt = 0;
  while (true) {
    unsigned long now = millis();
    if (handIsThere()) {
      if (handSince == 0) handSince = now;
      handLastAt = now;
      if (now - handSince >= HAND_START_MS) break;
    } else if (handSince != 0 && now - handLastAt > 300) {
      handSince = 0;
    }
    delay(20);
  }
  beep(200);

  // 손을 치울 때까지 기다림. 손이 있는 채로 출발하면 손을 치다 넘어질 수 있음
  unsigned long clearSince = millis();
  while (millis() - clearSince < 300) {
    if (handIsThere()) clearSince = millis();
    delay(20);
  }
  for (uint8_t i = 0; i < 3; i++) { beep(100); delay(900); }

  int pwm = RUN_POWERS[runIndex % 2];
  Serial.print("RUN "); Serial.print(runIndex + 1);
  Serial.print(" pwm="); Serial.println(pwm);
  drive(pwm);
  delay(RUN_MS);
  drive(0);
  beep(400);
  runIndex++;
}
