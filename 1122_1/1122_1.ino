#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>
#define SENSOR_ADRS 0x40 // GP2Y0E03의 I2C 주소
#define DISTANCE_ADRS 0x5E // 거리 값이 저장된 테이블의 주소

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
MPU6050 mpu(Wire);

const int dislimit = 10;        // 경사로로 판단하는 임계값
const int angchange = 5;         // 서보모터를 고정시키는 각도 변화 임계값
long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  delay(1000);
  mpu.calcGyroOffsets();
  myservo1.attach(8);  // 앞왼
  myservo2.attach(9);  // 앞오
  myservo3.attach(10); // 뒤왼
  myservo4.attach(12); // 뒤오

  myservo1.write(90); //우리가 원하는 수평방향 초기값 90
  myservo2.write(90);
  myservo3.write(90);
  myservo4.write(90);
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
    
    c[0] = Wire.read(); // 데이터의 11번째부터 4번째 비트 읽기 c[1]
    c[1] = Wire.read(); // 데이터의 3번째와 0번째 비트 읽기
    dis = ((c[0] * 16 + c[1]) / 16) / 4; // 거리
    Serial.println(dis);
    Serial.println("cm");
  }

  if (dis < dislimit) {
    // 경사로 진입
    dis = dislimit-1;
    Serial.println("경사로 진입");

    // 여기서 서보모터 초기화 또는 원하는 동작 수행
    if (abs(mpu.getGyroX()) < angchange) {
      myservo1.write(90); //초기값으로 변경
      myservo2.write(90);
      myservo3.write(90);
      myservo4.write(90);
      Serial.println(abs(mpu.getGyroX()));
      Serial.println("차고 높이 초기화");
    } else {
        if (millis() - timer > 100) {
          Serial.print("\tx축: ");
          Serial.print(mpu.getAngleX());
          Serial.print(",");
          Serial.print("\ty축: ");
          Serial.println(mpu.getAngleY());

          myservo1.write(constrain(mpu.getAngleX() + mpu.getAngleY() + 90, 0, 180));
          myservo2.write(constrain(mpu.getAngleX() - mpu.getAngleY() + 90, 0, 180));
          myservo3.write(constrain(-mpu.getAngleX() + mpu.getAngleY() + 90, 0, 180));
          myservo4.write(constrain(-mpu.getAngleX() - mpu.getAngleY() + 90, 0, 180));

          timer = millis();
        }
      }
  } else {
      if (millis() - timer > 100) {
        Serial.print("\tx축: ");
        Serial.print(mpu.getAngleX());
        Serial.print(",");
        Serial.print("\ty축: ");
        Serial.println(mpu.getAngleY());
        myservo1.write(constrain(mpu.getAngleX() + mpu.getAngleY() + 90, 0, 180));
        myservo2.write(constrain(mpu.getAngleX() - mpu.getAngleY() + 90, 0, 180));
        myservo3.write(constrain(-mpu.getAngleX() + mpu.getAngleY() + 90, 0, 180));
        myservo4.write(constrain(-mpu.getAngleX() - mpu.getAngleY() + 90, 0, 180));
        Serial.println("경사로 아님");

        timer = millis();
      }
  }
}
