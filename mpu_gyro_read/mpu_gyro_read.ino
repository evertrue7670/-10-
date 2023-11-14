/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */
 /*
  * 예제1-1 가속도센서의 값을 시리얼 플로터로 관찰하자
  * 예제1-2 자이로센서의 값을 시리얼 플로터로 관찰하자
  * 예제1-3 각도값을 시리얼 플로터로 관찰하자
  */

#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  //Serial.println(F("Calculating gyro offset, do not move MPU6050"));
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println("Done!\n");
  
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
    Serial.print("\tx축: ");Serial.print(mpu.getAngleX());
    Serial.print(",");
    Serial.print("\ty축: ");Serial.println(mpu.getAngleY());
    //Serial.print(",");
    //Serial.print("\tz축: ");Serial.println(mpu.getAngleZ());
    //Serial.println(F("\n"));
    
    timer = millis();
  }

}