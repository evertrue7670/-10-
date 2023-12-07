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

const int dislimit = 10;        // 경사로로 판단하는 기준값
const int angchange = 5;         // 서보모터를 고정시키는 각도 변화 기준값
long timer = 0;
float x = 3.0;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();
  sus1.attach(8);  // 앞왼
  sus2.attach(9);  // 앞오
  sus3.attach(10); // 뒤왼
  sus4.attach(11); // 뒤오

  sus1.write(90);
  sus2.write(90);
  sus3.write(90);
  sus4.write(90);
}

void loop() {
  mpu.update();
  uint16_t dis;
  byte c[2];

  Wire.beginTransmission(SENSOR_ADRS); // 적외선 센서 통신 시작
  Wire.write(DISTANCE_ADRS); // 거리 값이 저장된 테이블의 주소 지정
  Wire.endTransmission(); // 데이터 전송 및 통신 종료
  Wire.requestFrom(SENSOR_ADRS, 2);
  delay(200);

  if (Wire.available()) {
    
    c[0] = Wire.read(); // 데이터의 11번째부터 4번째 비트 읽기 
    c[1] = Wire.read(); // 데이터의 3번째와 0번째 비트 읽기
    dis = ((c[0] * 16 + c[1]) / 16) / 4; // 거리
    Serial.println(dis);
    Serial.println("cm");
  }

  if (dis < dislimit) {
    // 경사로 진입
    dis = dislimit-1;
    Serial.println("경사로 진입");
    delay(5000);

    // 서보모터 초기화 + 원하는 동작 수행
    if (abs(mpu.getGyroX()) < angchange) {
      Serial.println(abs(mpu.getGyroX()));
      Serial.println("차고 높이 초기화");
      sus1.write(90);
      delay(1);
      sus2.write(90);
      delay(1);
      sus3.write(90);
      delay(1);
      sus4.write(90);
      delay(3000);
      
    } else {
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
  } else {                                      // 경사로 아님
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
        Serial.println("경사로 아님");

        timer = millis();
      }
  }
}