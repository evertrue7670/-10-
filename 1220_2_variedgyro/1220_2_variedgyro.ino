#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>
#define SENSOR_ADRS 0x40 // GP2Y0E03의 I2C 주소
#define DISTANCE_ADRS 0x5E // 거리 값이 저장된 테이블의 주소

Servo sus1;
Servo sus2;
Servo sus3;
Servo sus4;
MPU6050 mpu(Wire);

const int dislimit = 10;        // 경사로로 판단하는 임계값
const int angchange = 20;         // 서보모터를 고정시키는 각도 변화 임계값
long timer = 0;
float x = 2.0;
unsigned long slopedetect = 0;
bool onslope = false;

void self() {
   if (millis() - timer > 100) {
        Serial.print("\tx축: ");
        Serial.print(mpu.getAngleX());
        Serial.print(",");
        Serial.print("\ty축: ");
        Serial.println(mpu.getAngleY());
        sus1.write(constrain(x*mpu.getAngleX() + x*mpu.getAngleY() + 90, 0, 180));
        sus2.write(constrain(-x*mpu.getAngleX() + x*mpu.getAngleY() + 90, 0, 180));
        sus3.write(constrain(-x*mpu.getAngleX() + x*mpu.getAngleY() + 90, 0, 180));
        sus4.write(constrain(x*mpu.getAngleX() + x*mpu.getAngleY() + 90, 0, 180));
        timer = millis();
      }
}
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  delay(1000);
  mpu.calcGyroOffsets();
  sus1.attach(8);  // 앞왼
  sus2.attach(9);  // 앞오
  sus3.attach(10); // 뒤왼
  sus4.attach(12); // 뒤오

  sus1.write(90); //우리가 원하는 수평방향 초기값 90
  sus2.write(90);
  sus3.write(90);
  sus4.write(90);
}
void loop() {
  mpu.update();
  uint16_t dis;
  byte c[2];

  Wire.beginTransmission(SENSOR_ADRS);  // 적외선 센서 통신 시작
  Wire.write(DISTANCE_ADRS);  // 거리 값이 저장된 테이블의 주소 지정
  Wire.endTransmission();  // 데이터 전송 및 통신 종료
  Wire.requestFrom(SENSOR_ADRS, 2);
  delay(200);

  if (Wire.available()) {
    c[0] = Wire.read();  // 데이터의 11번째부터 4번째 비트 읽기 c[1]
    c[1] = Wire.read();  // 데이터의 3번째와 0번째 비트 읽기
    dis = ((c[0] * 16 + c[1]) / 16) / 4;  // 거리
    Serial.print(dis);
    Serial.println("cm");
  }

  if (dis < dislimit) {
    // 처음 경사로에 진입할 때
    slopedetect = millis() + 3000;   // 여기서 3초는 추후에 속도로 시간 구하는 식 추가 예정
    delay(3000);
    Serial.println("경사로 진입");
    onslope = true; 
  }

  if (onslope) {
    dis = dislimit-1;// 경사로에 진입한 후 지속적으로 감지
    Serial.println("경사로 내 진행 중");
    Serial.print("각도변화 = ");
    Serial.println(mpu.getGyroX());

    // 여기서 서보모터 초기화 또는 원하는 동작 수행
    if (abs(mpu.getGyroX()) < angchange) {
      sus1.write(90); // 초기값으로 변경
      sus2.write(90);
      sus3.write(90);
      sus4.write(90);
      Serial.println(abs(mpu.getGyroX()));
      Serial.println("차고 높이 초기화");
    } else {
      self();
      Serial.println("경사로 들어간지 얼마 안됐을 때 레벨링");
    }

    if (millis() - slopedetect > 10000) {
      onslope = false;
    }
  } 
  else {
    self();
    Serial.println("일반주행");
    slopedetect = 0;
  }
}

