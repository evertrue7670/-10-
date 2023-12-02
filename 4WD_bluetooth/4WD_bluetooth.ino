//4휠 방향전환 자동차 with 블루투스
#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(12, 13); //블루투스 설정 BTSerial(Tx, Rx)
Servo servo;

int servoPin = 8;
int initAngle = 80;
int currentAngle;
int Rmax = initAngle+40, Lmin = initAngle-40;

int motorL[2] = {3, 11};
int motorR[2] = {5, 6};

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600); //블루투스 통신 시작
  for(int i=0; i<2; i++){
    pinMode(motorR[i], OUTPUT);
    pinMode(motorL[i], OUTPUT);
  }
  servo.attach(servoPin);
  servo.write(initAngle);
}

void loop() {
  //delay(10);
  currentAngle = servo.read();
//  Serial.println(currentAngle);
  
  if(BTSerial.available()) {
    char bt;
    bt = BTSerial.read();
    Serial.println(bt);
    if(bt == 'f') {
      analogWrite(motorL[0], 255);
      analogWrite(motorL[1], 0);
      analogWrite(motorR[0], 255);
      analogWrite(motorR[1], 0);
    }
    else if(bt == 's') {
      analogWrite(motorL[0], 0);
      analogWrite(motorL[1], 0);
      analogWrite(motorR[0], 0);
      analogWrite(motorR[1], 0);
    }
    else if(bt == 'b') {
      analogWrite(motorL[0], 0);
      analogWrite(motorL[1], 255);
      analogWrite(motorR[0], 0);
      analogWrite(motorR[1], 255);
    }
    
    if(bt == 'r') {
      if (currentAngle < Rmax){
        currentAngle+=10;
      }
    }
    else if(bt == 'l') {
      if (currentAngle > Lmin){
         currentAngle-=10;
      }   
    }
  }
  servo.write(currentAngle);
}


