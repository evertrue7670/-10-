#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;

Servo frontServo; // 앞바퀴용 서보모터
Servo rearServo; // 뒷바퀴용 서보모터

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 sensor!");
    while (1);
  }
  frontServo.attach(FRONT_SERVO_PIN); // 앞바퀴 서보모터에 연결된 핀 번호 입력
  rearServo.attach(REAR_SERVO_PIN); // 뒷바퀴 서보모터에 연결된 핀 번호 입력
}

void loop() {
  // IMU 센서에서 가속도 값을 읽어옵니다.
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 가속도 값으로 차체의 기울기를 계산합니다.
  float pitch = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;

  // 기울기를 기반으로 차체의 높이를 조절합니다.
  int frontServoAngle = map(pitch, -90, 90, 0, 180);
  int rearServoAngle = 180 - frontServoAngle;

  // 서보모터를 사용하여 차체의 높이를 조절합니다.
  frontServo.write(frontServoAngle);
  rearServo.write(rearServoAngle);

  delay(10); // 일정 시간 동안 기다립니다.
}
