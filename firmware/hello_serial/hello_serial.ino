/*
  업로드가 정말 되었는지 눈이 아니라 숫자로 확인하는 코드.
  LED 는 실드에 가려 안 보일 수 있으므로, 시리얼로 직접 말하게 한다.
  115200 으로 0.2 초마다 한 줄씩 보낸다.
*/
unsigned long n = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, n % 2);
  Serial.print("HELLO ");
  Serial.println(n++);
  delay(200);
}
