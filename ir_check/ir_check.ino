/*
  ir_check.ino -- IR 리모콘 버튼 코드 확인용 (임시 스케치)

  목적
    리모콘 버튼을 누를 때마다 그 버튼의 코드(Command)를 시리얼 모니터에 찍는다.
    여기서 본 Command 값들을 적어서 알려주면, 최종 차 펌웨어의 버튼 매핑에 쓴다.
    (이건 코드 확인용 임시 스케치다. 확인 끝나면 원래 차 펌웨어로 되돌린다.)

  배선
    IR 수신기 신호선(OUT) -> 아두이노 D2
    IR 수신기 VCC         -> 5V
    IR 수신기 GND         -> GND

  라이브러리 (아두이노 IDE -> 라이브러리 매니저)
    "IRremote" (by Armin Joachimsmeyer) 설치. 버전 4.x 권장.

  쓰는 법
    1. 이 스케치를 아두이노에 업로드
    2. 시리얼 모니터 열고 속도 115200
    3. 리모콘을 수신기에 향하게 하고 버튼을 하나씩 누른다
    4. 각 버튼마다 뜨는 "Command = 0x.." 값을 적는다 (숫자 0~9, OK/▶, 정지 등)

  참고
    - 대부분의 싼 리모콘은 NEC 프로토콜이라 잘 잡힌다. Protocol 이 UNKNOWN 으로만 뜨면 알려줘.
    - 버튼을 꾹 누르면 'repeat' 가 뜬다 (정상).
    - LED 피드백은 껐다. 이 차의 D13 은 부저라서, 켜면 버튼마다 삑 울리기 때문.
*/

#include <IRremote.hpp>        // IRremote 4.x

const int IR_RECEIVE_PIN = 2; 

void setup() {
  Serial.begin(115200);
  delay(200);
  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);   // D13=부저라 LED 피드백 끔
  Serial.println(F("IR 준비됨. 리모콘 버튼을 하나씩 눌러보세요."));
}

void loop() {
  if (IrReceiver.decode()) {
    // 한 줄 요약: 프로토콜, 주소(Address), 명령(Command), 원시값
    IrReceiver.printIRResultShort(&Serial);
    // 우리가 버튼 매핑에 쓸 값은 Command 다. 크게 한 번 더 찍어 준다.
    if (IrReceiver.decodedIRData.protocol != UNKNOWN) {
      Serial.print(F("   >>> 이 버튼 Command = 0x"));
      Serial.println(IrReceiver.decodedIRData.command, HEX);
    } else {
      Serial.println(F("   (UNKNOWN - 프로토콜을 못 알아봄. 리모콘을 더 가까이, 또는 다시 눌러보기)"));
    }
    IrReceiver.resume();       // 다음 신호 받을 준비
  }
}
