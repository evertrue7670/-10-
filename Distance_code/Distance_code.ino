#include <Wire.h>

#define SENSOR_ADRS 0x40 // GP2Y0E03의 I2C 주소
#define DISTANCE_ADRS 0x5E // 거리 값이 저장된 테이블의 주소

void setup() {
  // 시리얼 통신 초기화
  Wire.begin(); // I2C 초기화
  delay(1000); // 1초 후에 시작
  Serial.begin(9600);
}

void loop() {
  int ans;
  byte c[2];

  Wire.beginTransmission(SENSOR_ADRS); // 통신 시작
  Wire.write(DISTANCE_ADRS); // 거리 값이 저장된 테이블의 주소 지정
  ans = Wire.endTransmission(); // 데이터 전송 및 통신 종료
  delay(200);

  if (ans == 0) {
    ans = Wire.requestFrom(SENSOR_ADRS, 2);
    c[0] = Wire.read(); // 데이터의 11번째부터 4번째 비트 읽기 c[1]
    c[1] = Wire.read(); // 데이터의 3번째와 0번째 비트 읽기
    ans = ((c[0] * 16 + c[1]) / 16) / 4; // 거리
    Serial.print(ans);
    Serial.println("cm"); // 시리얼 모니터에 표시
    // Serial.println(millis()); // 주석 처리된 부분 제거
  } else {
    Serial.print("ERROR NO. = "); // GP2Y0E03과 통신할 수 없음
    Serial.println(ans);
  }

  delay(200); // 200ms 후에 반복
}