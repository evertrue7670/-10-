#include <IRremote.hpp>  // IR리모콘을 사용하기 위한 헤더파일 호출
#define IR_RECEIVE_PIN  A0 // 적외선 수신센서 모듈, 아두이노 우노 보드의 아날로그 A0 핀과 연결
IRrecv irrecv(IR_RECEIVE_PIN);  // IR읽어오는 핀 지정
decode_results results;  // IR리모콘 데이터 저장변수 객체 선언
#define EA 3  // 모터드라이버 EA 핀, 아두이노 디지털 3번 핀에 연결
#define EB 10  // 모터드라이버 EB 핀, 아두이노 디지털 11번 핀에 연결
#define M_IN1 4  // 모터드라이버 IN1 핀, 아두이노 디지털 4번 핀에 연결
#define M_IN2 5  // 모터드라이버 IN2 핀, 아두이노 디지털 5번 핀에 연결
#define M_IN3 12  // 모터드라이버 IN3 핀, 아두이노 디지털 13번 핀에 연결
#define M_IN4 11  // 모터드라이버 IN4 핀, 아두이노 디지털 12번 핀에 연결
int motorA_vector = 1;  // 모터의 회전방향이 반대일 시 0을 1로, 1을 0으로 바꿔주면 회전방향이 바뀜.
int motorB_vector = 1;  // 모터의 회전방향이 반대일 시 0을 1로, 1을 0으로 바꿔주면 회전방향이 바뀜.
int motor_speed = 255;  // 모터 스피드
int readIR;  // 리모콘으로 부터 들어오는 값을 저장하는 변수
void setup()  // 초기화
{
  Serial.begin(9600);
  pinMode(EA, OUTPUT);
  pinMode(EB, OUTPUT);
  pinMode(M_IN1, OUTPUT);
  pinMode(M_IN2, OUTPUT);
  pinMode(M_IN3, OUTPUT);
  pinMode(M_IN4, OUTPUT);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void loop()
{
  if (IrReceiver.decode()) {
    Serial.println(readIR);
    Serial.println(IrReceiver.decodedIRData.decodedRawData);

    switch (IrReceiver.decodedIRData.decodedRawData) { //읽어온 IR값과 비교
      case 3877175040: // Key2
        motor_con(-motor_speed, -motor_speed); //motor_con(motor_speed, motor_speed); //전진
        readIR = 2;
        break;
      case 4144561920: // Key 4
        motor_con(-255, 0); //좌회전
        readIR = 4;
        break;
      case 3810328320: // Key 5
        motor_con(0, 0); //정지
        readIR = 5;
        break;
      case 2774204160: // Key 6
        motor_con(0, -255); //우회전
        readIR = 6;
        break;
      case 2907897600: // Key 8
        motor_con(motor_speed, motor_speed); //후진
        readIR = 8;
        break;
      default:
        break;
    }
    irrecv.resume();
  }
}

void motor_con(int M1, int M2) { //모터 함수

  if (M1 > 0) { //모터A 정회전
    digitalWrite(EA, HIGH);
    digitalWrite(M_IN1, motorA_vector);
    digitalWrite(M_IN2, !motorA_vector);
  }
  else if (M1 < 0) { //모터A 역회전
    digitalWrite(EA, HIGH);
    digitalWrite(M_IN1, !motorA_vector);
    digitalWrite(M_IN2, motorA_vector);
  }
  else {       //모터 A 정지
    digitalWrite(EA, LOW);
    digitalWrite(M_IN1, LOW);
    digitalWrite(M_IN2, LOW);
  }

  if (M2 > 0) { //모터B 정회전
    digitalWrite(EB, HIGH);
    digitalWrite(M_IN3, motorB_vector);
    digitalWrite(M_IN4, !motorB_vector);
  }
  else if (M2 < 0) { //모터B 역회전
    digitalWrite(EB, HIGH);
    digitalWrite(M_IN3, !motorB_vector);
    digitalWrite(M_IN4, motorB_vector);
  }
  else {         //모터 B 정지
    digitalWrite(EB, LOW);
    digitalWrite(M_IN3, LOW);
    digitalWrite(M_IN4, LOW);
  }
}
