/*
  rccar_arduino.ino -- RC카 아래쪽 담당 (아두이노 우노/나노)

  이 보드가 하는 일
    1. 서보 위에 올린 앞 ToF 로 좌우를 훑는다. 뒤 ToF 는 USE_REAR_TOF 가 1 일 때만 읽는다.
    2. 자이로(MPU6050)의 z 축 각속도를 읽는다.
    3. 오렌지파이가 보낸 속도와 조향을 서보와 변속기에 넘긴다.
    4. 명령이 끊기면 스스로 차를 세운다.   <- 이것이 이 보드의 가장 중요한 일

  이 보드가 하지 않는 일
    경로 계획, 지도 만들기. 그건 전부 오렌지파이가 한다.

  통신 형식은 rccar/link.py 머리말에 적혀 있다. 글자로 주고받아서 시리얼 모니터로 볼 수 있다.

  배선 (핀 번호는 아래 상수에서 바꾼다)
    I2C : SDA A4, SCL A5   (ToF 2개와 자이로가 같은 선을 쓴다)
    앞 ToF XSHUT  D7      주소를 바꾸려고 하나씩 켠다
    뒤 ToF XSHUT  D8
    센서 서보     D9      앞 ToF 를 좌우로 돌린다
    조향 서보     D10
    변속기(ESC)   D11
    RC 수신기     D2      사람이 조종기로 뺏을 때 (선택)

  센서 서보 고를 때
    속도가 0.1 초/60 도 이하인 것을 쓸 것. 시뮬레이션에서 150 도/초짜리 느린 서보는
    20 판 중 11 판에서 기둥에 박았다. 잴 때 각도가 평균 34 도 어긋나기 때문이다.

  필요한 라이브러리 (아두이노 IDE 라이브러리 매니저에서 설치)
    VL53L1X        by Pololu
    Servo          (기본 제공)
    Wire           (기본 제공)

  안전 규칙
    - 최고 속도를 소프트웨어로 제한한다. 실제 RC카는 훨씬 빠르다.
    - 명령이 WATCHDOG_MS 동안 안 오면 무조건 멈춘다.
    - RC 조종기 채널이 올라오면 사람 조종이 우선이다.
*/

#include <Wire.h>
#include <Servo.h>
#include <VL53L1X.h>

// ---------------------------------------------------------------- 설정
const uint8_t PIN_XSHUT_FRONT = 7;
const uint8_t PIN_XSHUT_REAR  = 8;
const uint8_t PIN_SWEEP_SERVO = 9;
const uint8_t PIN_STEER_SERVO = 10;
const uint8_t PIN_ESC         = 11;
const uint8_t PIN_RC          = 2;      // 쓰지 않으면 USE_RC 를 0 으로
const uint8_t PIN_BUZZER      = 13;     // 이 차의 부저는 D13 (나노의 L LED 와 같은 핀)

// 능동형 부저 가정: 핀을 HIGH 로 두면 알아서 삑 운다. HIGH 만으로 소리가 안 나면
// 수동형이므로 beepOn() 을 tone(PIN_BUZZER, 2000) 으로 바꿀 것 (아래 함수 한 곳만 고치면 됨)
const uint8_t USE_BUZZER = 1;

const uint8_t USE_RC = 0;
// 뒤 ToF 를 달았으면 1. 지금 차는 앞 ToF 하나뿐이라 0 (2026-10-19).
// 없는 센서를 켜려 하면 시간 초과를 기다리느라 루프가 늦어질 수 있어서 아예 건드리지 않음.
// rccar/car_specs.py 의 has_rear_tof 와 맞출 것
const uint8_t USE_REAR_TOF = 0;

const unsigned long WATCHDOG_MS = 300;  // 명령이 이 시간 안 오면 정지
const int   MAX_SPEED_MMS = 1400;       // 최고 속도 제한 (시뮬레이터와 같은 1.4 m/s)
// 뒤를 못 보면 후진 명령은 받아도 0 으로 자름. 오렌지파이 쪽도 후진을 안 보내지만 한 겹 더 막음
const int   MAX_REV_MMS   = USE_REAR_TOF ? 450 : 0;
const float MAX_STEER_DEG = 32.0;       // 차의 최대 조향각

// 조향 서보 보정: 바퀴가 정면일 때의 서보 각도와, 1 도 꺾을 때 서보가 움직이는 양
const float STEER_CENTER_DEG = 90.0;
const float STEER_GAIN       = 1.0;     // 실측해서 고칠 것

// 변속기 보정: 정지일 때 1500 us. 앞뒤로 얼마나 벌릴지
const int ESC_NEUTRAL_US = 1500;
const int ESC_SPAN_US    = 400;         // 최고 속도에서 1900 us

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
Servo sweepServo, steerServo, esc;
VL53L1X tofFront, tofRear;

int   sweepDeg = 0;
int   sweepDir = SWEEP_STEP_DEG;
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
void tofBegin() {
  pinMode(PIN_XSHUT_FRONT, OUTPUT); digitalWrite(PIN_XSHUT_FRONT, LOW);
  pinMode(PIN_XSHUT_REAR,  OUTPUT); digitalWrite(PIN_XSHUT_REAR,  LOW);
  delay(10);
  // 두 센서가 같은 주소로 나오므로, 하나씩 켜서 주소를 바꾼다
  digitalWrite(PIN_XSHUT_FRONT, HIGH); delay(10);
  tofFront.setTimeout(100);
  tofFront.init();
  tofFront.setAddress(0x30);
  if (USE_REAR_TOF) {
    digitalWrite(PIN_XSHUT_REAR, HIGH); delay(10);
    tofRear.setTimeout(100);
    tofRear.init();
    tofRear.setAddress(0x31);
  }

  // AVR 컴파일러는 { } 목록을 도는 for 를 받지 못한다. 배열로 쓴다.
  VL53L1X* sensors[2] = { &tofFront, &tofRear };
  uint8_t sensorCount = USE_REAR_TOF ? 2 : 1;
  for (uint8_t i = 0; i < sensorCount; i++) {
    VL53L1X* s = sensors[i];
    s->setDistanceMode(VL53L1X::Long);
    s->setMeasurementTimingBudget(33000);    // 33 ms
    s->startContinuous(33);
  }
}

int readTof(VL53L1X& s) {
  if (!s.dataReady()) return -2;             // 아직 안 나옴. 지난 값을 유지
  int mm = s.read(false);
  if (s.timeoutOccurred() || s.ranging_data.range_status != VL53L1X::RangeValid) return -1;
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
                   millis(), (int)(gyroDps * 100.0), (int)(sweepDeg * 10),
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
void applyOutputs(int speedMMS, float steerDeg) {
  steerServo.write(constrain(STEER_CENTER_DEG + STEER_GAIN * steerDeg, 0, 180));
  int us = ESC_NEUTRAL_US + (int)((long)speedMMS * ESC_SPAN_US / MAX_SPEED_MMS);
  esc.writeMicroseconds(constrain(us, ESC_NEUTRAL_US - ESC_SPAN_US,
                                  ESC_NEUTRAL_US + ESC_SPAN_US));
}

// ---------------------------------------------------------------- 시작
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  sweepServo.attach(PIN_SWEEP_SERVO);
  steerServo.attach(PIN_STEER_SERVO);
  esc.attach(PIN_ESC);
  applyOutputs(0, 0);                       // 변속기가 중립을 보고 준비되도록
  sweepServo.write(SWEEP_CENTER_DEG);
  if (USE_RC) pinMode(PIN_RC, INPUT);
  if (USE_BUZZER) { pinMode(PIN_BUZZER, OUTPUT); beepSet(false); }

  tofBegin();
  mpuBegin();                                // 이 동안 차를 가만히 둘 것
  lastCmdAt = millis();
  sweepMovedAt = millis();
}

// ---------------------------------------------------------------- 반복
void loop() {
  readSerial();

  // ---- 자이로 (도/초). 2000 도/초 범위 기준 눈금 16.4
  gyroDps = (mpuRawZ() - gyroBias) / 16.4;

  // ---- 앞 센서: 서보가 멈춘 뒤에 잰다. 움직이면서 재면 엉뚱한 방향에 찍힌다
  unsigned long now = millis();
  if (!sweepSettled && now - sweepMovedAt >= SWEEP_SETTLE_MS) sweepSettled = true;
  if (sweepSettled) {
    int v = readTof(tofFront);
    if (v != -2) {
      frontMM = v;
      sweepDeg += sweepDir;                  // 다음 방향으로
      if (sweepDeg > SWEEP_MAX_DEG) { sweepDeg = SWEEP_MAX_DEG - SWEEP_STEP_DEG; sweepDir = -SWEEP_STEP_DEG; }
      if (sweepDeg < SWEEP_MIN_DEG) { sweepDeg = SWEEP_MIN_DEG + SWEEP_STEP_DEG; sweepDir =  SWEEP_STEP_DEG; }
      sweepServo.write(constrain(SWEEP_CENTER_DEG + sweepDeg, 0, 180));
      sweepMovedAt = now; sweepSettled = false;
    }
  }
  if (USE_REAR_TOF) {
    int r = readTof(tofRear);
    if (r != -2) rearMM = r;
  }                                          // 뒤 센서가 없으면 rearMM 은 -1 로 남아 '못 맞힘'으로 보고됨

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
  bool late = (millis() - lastCmdAt) > WATCHDOG_MS;
  if (late) flags |= 8;

  int applied = (late || manual) ? 0 : cmdSpeedMMS;
  applyOutputs(applied, (late || manual) ? 0.0 : cmdSteerDeg);

  // ---- 상태 보고 (50 Hz)
  if (now - lastSendAt >= SEND_PERIOD_MS) {
    lastSendAt = now;
    sendStatus(applied, flags);
  }
}
