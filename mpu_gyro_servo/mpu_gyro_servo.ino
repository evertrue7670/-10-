/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */
 /*
  * 예제2 (스몰프로젝트) 자이로센서로
  * 2축팬틸트 프레임을 제어해보라!
  * 바닥쪽 서보모터는 D9에 연결했고
  * 머리쪽 서보모터는 D10에 연결했다.
  */

#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>

Servo myservo1; 
Servo myservo2; 
MPU6050 mpu(Wire);

long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  //Serial.println(F("Calculating gyro offset, do not move MPU6050"));
  delay(1000);
  mpu.calcGyroOffsets();
  //Serial.println("Done!\n");
  myservo1.attach(9); //바닥
  myservo2.attach(10); //머리
}

void loop() {
  mpu.update();

  if(millis() - timer > 100){ // print data every second
    //Serial.print(F("TEMPERATURE  : "));Serial.println(mpu.getTemp());
    //Serial.print(F("ACCELERO   X : "));
    //가속도센서값
    //Serial.print(mpu.getAccX());
    //Serial.print(",");
    //Serial.print(mpu.getAccY());
    //Serial.print(",");
    //Serial.println(mpu.getAccZ());
  
    //Serial.print(F("GYRO       X : "));
    //자이로센서
    //Serial.print(mpu.getGyroX());
    //Serial.print(",");
    //Serial.print(mpu.getGyroY());
    //Serial.print(",");
    //Serial.println(mpu.getGyroZ());
  
    //Serial.print(F("ACC ANGLE  X : "));Serial.print(mpu.getAccAngleX());
    //Serial.print("\tY : ");Serial.println(mpu.getAccAngleY());
    
    //Serial.print(F("ANGLE      X : "));
    //각도
    myservo2.write(mpu.getAngleX());
    //Serial.print(constrain(mpu.getAngleX(),0,180));
    //Serial.print(",");
    //Serial.print(mpu.getAngleY());
    //Serial.print(",");
    myservo1.write(mpu.getAngleY());
    //Serial.println(constrain(mpu.getAngleZ()+90,0,180));
    //Serial.println(F("=======================================================\n"));
    
    timer = millis();
  }

}