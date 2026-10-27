/*
  rccar_arduino.ino -- RC카 아래쪽 담당 (아두이노 우노/나노)

  이 보드가 하는 일
    1. 서보 위에 올린 앞 거리 센서(샤프 적외선)로 좌우를 훑는다. 뒤 센서는 없다.
    2. 자이로(MPU6050)의 z 축 각속도를 읽는다.
    3. 오렌지파이가 보낸 속도와 조향을 서보와 변속기에 넘긴다.
    4. 명령이 끊기면 스스로 차를 세운다.   <- 이것이 이 보드의 가장 중요한 일

  이 보드가 하지 않는 일
    경로 계획, 지도 만들기. 그건 전부 오렌지파이가 한다.

  통신 형식은 rccar/link.py 머리말에 적혀 있다. 글자로 주고받아서 시리얼 모니터로 볼 수 있다.

  배선 (핀 번호는 아래 상수에서 바꾼다)
    I2C : SDA A4, SCL A5   (자이로 MPU6050)
    앞 거리 센서  A0       샤프 적외선 GP2Y0A21 의 신호선 (아날로그)
    센서 서보     D9      앞 거리 센서를 좌우로 돌린다
    조향 서보     D10
    DC 모터 2개, 드라이버 L9110S : B-1A D4, B-1B D5, A-1A D6, A-1B D7
                                 (VCC 는 모터 배터리, GND 공통)
    적외선 리모컨 수신기 D2   OUT 선. VCC 5V, GND
    RC 수신기     D2      사람이 조종기로 뺏을 때 (선택)

  센서 서보 고를 때
    속도가 0.1 초/60 도 이하인 것을 쓸 것. 시뮬레이션에서 150 도/초짜리 느린 서보는
    20 판 중 11 판에서 기둥에 박았다. 잴 때 각도가 평균 34 도 어긋나기 때문이다.

  필요한 라이브러리 (아두이노 IDE 라이브러리 매니저에서 설치)
    Servo          (기본 제공)
    Wire           (기본 제공)
    (샤프 거리 센서는 아날로그라 라이브러리가 필요 없다)

  안전 규칙
    - 최고 속도를 소프트웨어로 제한한다. 실제 RC카는 훨씬 빠르다.
    - 명령이 WATCHDOG_MS 동안 안 오면 무조건 멈춘다.
    - RC 조종기 채널이 올라오면 사람 조종이 우선이다.
*/

#include <Wire.h>
#include <Servo.h>
#include <IRremote.hpp>                // arduino-cli lib install IRremote (4.x)

// ---- 적외선 리모컨 (Car mp3 21키 리모컨, NEC)
//   숫자: 갈 거리(cm) 입력.  ▶|| (0x43): 출발.  EQ (0x09): 숫자 지우기.  달리는 중 아무 버튼: 정지.
//   켜면 멈춘 상태(irPaused). 멈춘 동안은 '사람 조종' 플래그(4)를 올려 파이가 기다리게 한다.
//   출발할 때 숫자가 있으면 G,<cm> 줄을 파이에 보낸다. 파이는 그 순간 차가 보는 방향으로 그 거리를 목적지로 잡는다.
const uint8_t PIN_IR = 2;
bool irPaused = true;
bool irHasGoal = false;
int irNum = 0;
const uint8_t IR_DIGITS[10] = {0x16,0x0C,0x18,0x5E,0x08,0x1C,0x5A,0x42,0x52,0x4A};   // 0~9

// ---------------------------------------------------------------- 설정
// 앞 거리 센서는 샤프 적외선 GP2Y0A21YK0F (아날로그, 10~80 cm). 신호선을 A0 에 꽂는다.
// 전에는 레이저 ToF(VL53L1X)였는데, 이 차는 계획서대로 샤프를 쓴다 (2026-10-21).
const uint8_t PIN_SHARP_FRONT = A0;
const uint8_t PIN_SWEEP_SERVO = 9;
const uint8_t PIN_STEER_SERVO = 10;
// 구동 모터는 DC 모터 2개 + L9110S 드라이버 (A, B 채널). 두 모터는 늘 같은 명령을 받는다.
// L9110: 두 입력의 차이로 돈다. 둘 다 LOW 면 정지. 입력이 떠 있으면 HIGH 로 읽혀 모터가 돈다
//   (2026-10-26 실제로 겪음: 모터 2개 배선인데 펌웨어가 D5 D6 만 써서 D4 D7 이 떠 바퀴가 계속 돌았다).
// A 채널: A-1A D6 (세기), A-1B D7. B 채널: B-1B D5 (세기), B-1A D4. 아래 출력부에서 A 와 같이 쓴다.
// D4 D7 은 PWM 이 안 되는 핀이라 후진(세기 핀 반대편)은 제대로 안 된다. 뒤 센서가 없어 후진을 꺼 두었으므로 지금은 문제없다.
const uint8_t PIN_MOTOR_A1A = 6;      // 드라이버 A-1A (PWM 핀)
const uint8_t PIN_MOTOR_A1B = 7;      // 드라이버 A-1B (보통 핀)
const uint8_t PIN_RC          = 2;      // 쓰지 않으면 USE_RC 를 0 으로
const uint8_t PIN_BUZZER      = 13;     // 이 차의 부저는 D13 (나노의 L LED 와 같은 핀)

// 능동형 부저 가정: 핀을 HIGH 로 두면 알아서 삑 운다. HIGH 만으로 소리가 안 나면
// 수동형이므로 beepOn() 을 tone(PIN_BUZZER, 2000) 으로 바꿀 것 (아래 함수 한 곳만 고치면 됨)
const uint8_t USE_BUZZER = 1;

const uint8_t USE_RC = 0;
const uint8_t USE_REAR_TOF = 0;        // 뒤 거리 센서 없음

// 샤프 적외선은 거리를 전압으로 준다. 1/거리 가 ADC 값에 선형이라, 거리(mm) = K / (adc - B).
// 이 두 상수는 2026-10-21 교정값이다 (계획서 6-다-4). firmware/sharp_calib 로 재서 정했다.
// 측정: 20cm->276, 30cm->187, 40cm->146, 50cm->113, 60cm->93 (ADC). 20~50cm 오차 몇 mm.
// 센서를 바꾸거나 전원 전압이 달라지면 다시 교정할 것.
const float SHARP_K = 54428.0;
const float SHARP_B = 5.16;
const int   SHARP_MIN_MM = 100;        // 이보다 가까우면 값이 뒤집혀 믿을 수 없다 (10 cm)
const int   SHARP_MAX_MM = 800;        // 이보다 멀면 '없음'으로 본다 (80 cm)
const uint8_t SHARP_SAMPLES = 5;       // 아날로그 잡음이 커서 여러 번 읽어 중앙값을 쓴다

const unsigned long WATCHDOG_MS = 300;  // 명령이 이 시간 안 오면 정지
const int   MAX_SPEED_MMS = 1400;       // 최고 속도 제한 (시뮬레이터와 같은 1.4 m/s)
// 뒤를 못 보면 후진 명령은 받아도 0 으로 자름. 오렌지파이 쪽도 후진을 안 보내지만 한 겹 더 막음
const int   MAX_REV_MMS   = USE_REAR_TOF ? 450 : 0;
const float MAX_STEER_DEG = 32.0;       // 차의 최대 조향각

// 조향 서보 보정: 바퀴가 정면일 때의 서보 각도와, 1 도 꺾을 때 서보가 움직이는 양
const float STEER_CENTER_DEG = 90.0;
const float STEER_GAIN       = 1.0;     // 실측해서 고칠 것

// DC 모터는 아주 느린 속도에서 안 돌고 소리만 난다. 이 PWM(0~255) 아래로는 0 으로 본다.
const int MOTOR_MIN_PWM = 60;

// 앞 센서가 훑는 각도 (도). 시뮬레이터의 tof_sweep 과 같은 범위
const int SWEEP_MIN_DEG = -90;
const int SWEEP_MAX_DEG =  90;
const int SWEEP_STEP_DEG = 15;
// 서보가 멈추길 기다리는 시간.
// 45 ms 로 두면 15 도 옮기는 동안 다 못 가서, 잴 때 평균 6.7 도 어긋난다.
// 90 ms 면 1.2 도로 줄고, 무작위 지도 20 개에서 차체 여유가 0.068 m -> 0.109 m 로 넓어졌다.
// 대가는 주행 시간 0.4 초다. 서보가 느리면 이 값을 더 키워야 한다.
const unsigned long SWEEP_SETTLE_MS = 90;

// 센서 서보 보정: 차 정면을 볼 때의 서보 각도
const float SWEEP_CENTER_DEG = 90.0;

// ---------------------------------------------------------------- 상태
Servo sweepServo, steerServo;

int   sweepDeg = 0;
int   sweepDir = SWEEP_STEP_DEG;
// frontMM 을 잰 방향. 재자마자 서보를 다음 방향으로 돌리므로 sweepDeg 와 따로 둬야 함.
// 예전엔 sweepDeg 를 보내서 거리가 15 도 옆 방향에 찍혔음. 999 는 '아직 한 번도 안 잼' 표시 (link.py 와 같음)
int   measuredDeg = 999;
unsigned long sweepMovedAt = 0;
bool  sweepSettled = false;

int   cmdSpeedMMS = 0;
float cmdSteerDeg = 0.0;
unsigned long lastCmdAt = 0;

int   frontMM = -1, rearMM = -1;
float gyroDps = 0.0;
float gyroBias = 0.0;

char  rxBuf[48];
uint8_t rxLen = 0;

unsigned long lastSendAt = 0;
const unsigned long SEND_PERIOD_MS = 20;   // 50 Hz

// ---------------------------------------------------------------- 자이로
const uint8_t MPU = 0x68;

void mpuBegin() {
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); Wire.write(0);          // 잠자기 해제
  Wire.endTransmission(true);
  // 처음 2 초 동안 가만히 두고 편향을 잰다. 이걸 빼면 차가 가만히 있어도 방향이 돈다.
  long sum = 0;
  for (int i = 0; i < 200; i++) { sum += mpuRawZ(); delay(10); }
  gyroBias = sum / 200.0;
}

int16_t mpuRawZ() {
  Wire.beginTransmission(MPU);
  Wire.write(0x47);                          // GYRO_ZOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU, (uint8_t)2, (uint8_t)true);
  int16_t v = (Wire.read() << 8) | Wire.read();
  return v;
}

// ---------------------------------------------------------------- 센서
void sharpBegin() {
  pinMode(PIN_SHARP_FRONT, INPUT);           // 아날로그 입력. 따로 켤 것이 없다
}

// 샤프 센서 한 번 읽어 거리(mm). 범위 밖이거나 못 믿을 값이면 -1.
// 아날로그 값이 튀어서 여러 번 읽어 중앙값을 쓴다 (버블 정렬, 개수가 적어 충분하다).
int readSharp() {
  int s[SHARP_SAMPLES];
  for (uint8_t i = 0; i < SHARP_SAMPLES; i++) s[i] = analogRead(PIN_SHARP_FRONT);
  for (uint8_t i = 0; i < SHARP_SAMPLES - 1; i++)
    for (uint8_t j = 0; j < SHARP_SAMPLES - 1 - i; j++)
      if (s[j] > s[j + 1]) { int t = s[j]; s[j] = s[j + 1]; s[j + 1] = t; }
  int adc = s[SHARP_SAMPLES / 2];
  if (adc <= SHARP_B) return -1;             // 전압이 바닥 — 아주 멀거나 아무것도 없음
  int mm = (int)(SHARP_K / (adc - SHARP_B));
  if (mm < SHARP_MIN_MM || mm > SHARP_MAX_MM) return -1;
  return mm;
}

// ---------------------------------------------------------------- 통신
uint8_t checksum(const char* s, uint8_t n) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < n; i++) c ^= (uint8_t)s[i];
  return c;
}

void handleLine(char* line) {
  char* star = strchr(line, '*');
  if (!star) return;
  *star = 0;
  uint8_t want = (uint8_t)strtol(star + 1, NULL, 16);
  if (checksum(line, strlen(line)) != want) return;       // 깨진 줄은 버린다
  if (line[0] == 'B') {                                    // 부저: B,1 켜고 B,0 끈다
    int on;
    if (sscanf(line, "B,%d", &on) == 1) beepSet(on != 0);
    return;                                                // 부저는 lastCmdAt 을 건드리지 않는다
  }                                                        // (삑삑 소리가 watchdog 정지를 막으면 안 됨)
  if (line[0] != 'C') return;
  int sp; int st;
  if (sscanf(line, "C,%d,%d", &sp, &st) != 2) return;
  cmdSpeedMMS = constrain(sp, -MAX_REV_MMS, MAX_SPEED_MMS);
  cmdSteerDeg = constrain(st / 10.0, -MAX_STEER_DEG, MAX_STEER_DEG);
  lastCmdAt = millis();
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen) { rxBuf[rxLen] = 0; handleLine(rxBuf); rxLen = 0; }
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;                                          // 너무 길면 버린다
    }
  }
}

void sendStatus(int appliedMMS, uint8_t flags) {
  char body[64];
  int n = snprintf(body, sizeof(body), "S,%lu,%d,%d,%d,%d,%d,%u",
                   millis(), (int)(gyroDps * 100.0), measuredDeg * 10,
                   frontMM, rearMM, appliedMMS, flags);
  Serial.print(body);
  Serial.print('*');
  char hex[4]; snprintf(hex, sizeof(hex), "%02X", checksum(body, n));
  Serial.println(hex);
}

// ---------------------------------------------------------------- 부저
// 능동형 부저를 가정해 핀만 켜고 끈다. 수동형이면 이 두 함수만 tone/noTone 으로 바꾼다.
void beepSet(bool on) {
  if (USE_BUZZER) digitalWrite(PIN_BUZZER, on ? HIGH : LOW);
}

// ---------------------------------------------------------------- 출력
// 돌려주는 값은 모터에 실제로 준 속도(mm/s). 너무 느려서 모터를 껐으면 0 을 돌려줌
int applyOutputs(int speedMMS, float steerDeg) {
  steerServo.write(constrain(STEER_CENTER_DEG + STEER_GAIN * steerDeg, 0, 180));

  // 속도(mm/s)를 PWM(0~255)으로. 부호가 방향.
  int pwm = (int)((long)abs(speedMMS) * 255 / MAX_SPEED_MMS);
  pwm = constrain(pwm, 0, 255);
  if (pwm < MOTOR_MIN_PWM) pwm = 0;        // 너무 느리면 안 돌고 소리만 나므로 끈다
  if (pwm == 0) {                          // 정지
    analogWrite(PIN_MOTOR_A1A, 0); analogWrite(5, 0);
    analogWrite(PIN_MOTOR_A1B, 0); analogWrite(4, 0);
  } else if (speedMMS > 0) {               // 전진
    analogWrite(PIN_MOTOR_A1A, pwm); analogWrite(5, pwm);
    analogWrite(PIN_MOTOR_A1B, 0); analogWrite(4, 0);
  } else {                                 // 후진
    analogWrite(PIN_MOTOR_A1A, 0); analogWrite(5, 0);
    analogWrite(PIN_MOTOR_A1B, pwm); analogWrite(4, pwm);
  }
  return pwm == 0 ? 0 : speedMMS;
}

// ---------------------------------------------------------------- 시작
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  sweepServo.attach(PIN_SWEEP_SERVO);
  steerServo.attach(PIN_STEER_SERVO);
  pinMode(PIN_MOTOR_A1A, OUTPUT);
  pinMode(PIN_MOTOR_A1B, OUTPUT);
  applyOutputs(0, 0);                       // 모터 정지 상태로 시작
  sweepServo.write(SWEEP_CENTER_DEG);
  if (USE_RC) pinMode(PIN_RC, INPUT);
  if (USE_BUZZER) { pinMode(PIN_BUZZER, OUTPUT); beepSet(false); }

  sharpBegin();
  mpuBegin();                                // 이 동안 차를 가만히 둘 것
  IrReceiver.begin(PIN_IR, DISABLE_LED_FEEDBACK);   // LED 피드백은 D13(부저)이라 끈다
  lastCmdAt = millis();
  sweepMovedAt = millis();
}

// ---------------------------------------------------------------- 반복
void irBeep(int ms) { beepSet(true); delay(ms); beepSet(false); }

// 리모컨으로 정한 거리를 파이에 알린다: G,<cm>*<검사값>
void sendGoal(int cm) {
  char b[16];
  int n = snprintf(b, sizeof(b), "G,%d", cm);
  Serial.print(b);
  Serial.print('*');
  char h[4];
  snprintf(h, sizeof(h), "%02X", checksum(b, n));
  Serial.println(h);
}

void loop() {
  readSerial();

  // ---- 자이로 (도/초). 2000 도/초 범위 기준 눈금 16.4
  gyroDps = (mpuRawZ() - gyroBias) / 16.4;

  // ---- 앞 센서: 서보가 멈춘 뒤에 잰다. 움직이면서 재면 엉뚱한 방향에 찍힌다
  unsigned long now = millis();
  if (!sweepSettled && now - sweepMovedAt >= SWEEP_SETTLE_MS) sweepSettled = true;
  if (sweepSettled) {
    frontMM = readSharp();                   // 샤프는 늘 바로 값을 준다 (ToF 처럼 '준비중'이 없다)
    measuredDeg = sweepDeg;
    {
      sweepDeg += sweepDir;                  // 다음 방향으로
      if (sweepDeg > SWEEP_MAX_DEG) { sweepDeg = SWEEP_MAX_DEG - SWEEP_STEP_DEG; sweepDir = -SWEEP_STEP_DEG; }
      if (sweepDeg < SWEEP_MIN_DEG) { sweepDeg = SWEEP_MIN_DEG + SWEEP_STEP_DEG; sweepDir =  SWEEP_STEP_DEG; }
      sweepServo.write(constrain(SWEEP_CENTER_DEG + sweepDeg, 0, 180));
      sweepMovedAt = now; sweepSettled = false;
    }
  }
  // 뒤 거리 센서는 없다. rearMM 은 -1 로 남아 '못 맞힘'으로 보고된다.

  // ---- 안전 판단: 명령이 끊겼거나 사람이 가져갔으면 멈춘다
  uint8_t flags = 0;
  if (frontMM >= 0) flags |= 1;
  if (rearMM  >= 0) flags |= 2;
  bool manual = false;
  if (USE_RC) {
    unsigned long w = pulseIn(PIN_RC, HIGH, 25000);
    manual = (w > 1700);                     // 조종기 스위치를 올리면 사람이 운전
    if (manual) flags |= 4;
  }
  if (IrReceiver.decode()) {
    uint8_t c = IrReceiver.decodedIRData.command;
    if (IrReceiver.decodedIRData.protocol != UNKNOWN && !(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      int d = -1;
      for (int i = 0; i < 10; i++) if (IR_DIGITS[i] == c) d = i;
      if (!irPaused) { irPaused = true; irBeep(400); }                         // 달리는 중: 아무 버튼이나 정지
      else if (d >= 0) { irNum = (irNum * 10 + d) % 1000; irBeep(30); }       // 숫자
      else if (c == 0x09) { irNum = 0; irBeep(200); }                          // EQ: 지우기
      else if (c == 0x43 && (irNum > 0 || irHasGoal)) {                        // ▶||: 출발
        if (irNum > 0) { sendGoal(irNum); irNum = 0; irHasGoal = true; }
        irPaused = false; irBeep(100);
      }
    }
    IrReceiver.resume();
  }
  if (irPaused) { manual = true; flags |= 4; }
  bool late = (millis() - lastCmdAt) > WATCHDOG_MS;
  if (late) flags |= 8;

  // 예전엔 모터를 꺼 놓고도 명령 속도를 그대로 보고해서, 파이가 서 있는 차를 가고 있다고 믿었음
  int applied = applyOutputs((late || manual) ? 0 : cmdSpeedMMS,
                             (late || manual) ? 0.0 : cmdSteerDeg);

  // ---- 상태 보고 (50 Hz)
  if (now - lastSendAt >= SEND_PERIOD_MS) {
    lastSendAt = now;
    sendStatus(applied, flags);
  }
}
