/*
  i2c_scan.ino -- 자이로(MPU6050)가 살아 있는지 보는 임시 스케치

  2 초마다 I2C 주소 1~126 을 다 불러 보고, 대답한 주소를 찍음.
  자이로가 대답하면 '나 누구야' 레지스터(WHO_AM_I, 0x75)와 z 축 회전값도 찍음.
  확인이 끝나면 rccar_arduino 를 다시 올릴 것.

  읽는 법
    아무 주소도 없음        선이나 전원 문제 (VCC, GND, SDA=A4, SCL=A5)
    0x69 에서 찾음          자이로 AD0 핀이 높게 걸려 주소가 바뀜
    0x68 에서 찾음          자이로는 살아 있음. WHO_AM_I 가 0x68 이면 진짜 MPU6050
*/

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  // 대답이 없을 때 무한히 기다리지 않게 함 (rccar_arduino 와 같음)
  Wire.setWireTimeout(3000, true);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);                    // D13 은 부저라 꺼 둠
}

int readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom(addr, (uint8_t)1, (uint8_t)true);
  if (Wire.available() < 1) return -1;
  return Wire.read();
}

void loop() {
  Serial.print("SCAN ");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (Wire.getWireTimeoutFlag()) {
      Wire.clearWireTimeoutFlag();
      Serial.print("timeout@0x"); Serial.print(addr, HEX); Serial.print(' ');
      continue;
    }
    if (err == 0) {
      Serial.print("0x"); Serial.print(addr, HEX); Serial.print(' ');
      found++;
    }
  }
  Serial.print("| found="); Serial.println(found);

  for (uint8_t addr = 0x68; addr <= 0x69; addr++) {
    int who = readReg(addr, 0x75);
    if (who < 0) continue;
    // 잠자기를 풀고 z 축 회전값을 몇 번 읽어 봄. 값이 바뀌면 센서가 실제로 재고 있는 것
    Wire.beginTransmission(addr); Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
    delay(50);
    Serial.print("MPU@0x"); Serial.print(addr, HEX);
    Serial.print(" WHO_AM_I=0x"); Serial.print(who, HEX);
    Serial.print(" gyroZ=");
    for (int i = 0; i < 5; i++) {
      int hi = readReg(addr, 0x47), lo = readReg(addr, 0x48);
      if (hi < 0 || lo < 0) { Serial.print("? "); continue; }
      Serial.print((int16_t)((hi << 8) | lo)); Serial.print(' ');
      delay(20);
    }
    Serial.println();
  }
  delay(2000);
}
