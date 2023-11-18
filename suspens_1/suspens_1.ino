#include "Wire.h"
#include <MPU6050_light.h>
#include <Servo.h>

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
MPU6050 mpu(Wire);

long timer = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.begin();
  delay(1000);
  mpu.calcGyroOffsets();
  myservo1.attach(8); // 앞왼
  myservo2.attach(9); // 앞오
  myservo3.attach(10); // 뒤왼
  myservo4.attach(12); // 뒤오

  myservo1.write(0);
  myservo2.write(0);
  myservo3.write(0);
  myservo4.write(0);

}

void loop() {
  mpu.update();

  if (millis() - timer > 100) { // print data every second
    //myservo1.write(constrain(mpu.getAngleX()+90, 0, 180));
    //myservo2.write(constrain(mpu.getAngleX()+90, 0, 180));
    //myservo3.write(constrain(-mpu.getAngleX()+90, 0, 180));
    //myservo4.write(constrain(-mpu.getAngleX()+90, 0, 180));

    Serial.print("\tx축: ");
    Serial.print(mpu.getAngleX());
    Serial.print(",");
    Serial.print("\ty축: ");
    Serial.println(mpu.getAngleY());

    myservo1.write(constrain(mpu.getAngleX()+mpu.getAngleY()+90, 0, 180));
    myservo2.write(constrain(mpu.getAngleX()-mpu.getAngleY()+90, 0, 180));
    myservo3.write(constrain(-mpu.getAngleX()+mpu.getAngleY()+90, 0, 180));
    myservo4.write(constrain(-mpu.getAngleX()-mpu.getAngleY()+90, 0, 180));
    Serial.print("Servo 4 Angle: ");
    Serial.println(myservo4.read()); 
   
    timer = millis();
  }
}

