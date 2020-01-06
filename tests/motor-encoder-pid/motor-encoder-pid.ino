#include <PIDController.h>

#define MOTOR_1

#ifdef MOTOR_1
  // MOTOR 1 RIGHT
  #define MOTOR_ENA 5
  #define MOTOR_IN1 4
  #define MOTOR_IN2 7
  
  #define ENCODER_A 3
  #define ENCODER_B 17
#else
  // MOTOR 2 LEFT
  #define MOTOR_ENA 6
  #define MOTOR_IN1 15
  #define MOTOR_IN2 8
  
  #define ENCODER_A 2
  #define ENCODER_B 14
#endif

volatile long int encoder_pos = 0;
PIDController pos_pid;
int motor_value = 255;
int motor_min = 70;
unsigned int integerValue=0;  // Max value is 65535
char incomingByte;

void setup() {
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder, RISING);

  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
 
  pos_pid.begin();
  pos_pid.tune(40, 2.5, 5000); // 15, 2.5, 3000
  pos_pid.limit(-255, 255);
}

void loop() {
  
  if (Serial.available() > 0) {
    integerValue = 0;
    while(1) {
      incomingByte = Serial.read();
      if (incomingByte == '\n') break;
      if (incomingByte == -1) continue;
      integerValue *= 10;
      integerValue = ((incomingByte - 48) + integerValue);
      pos_pid.setpoint(integerValue);
    }
  }

  motor_value = pos_pid.compute(encoder_pos);
  
  if (motor_value > 0) {
    MotorCounterClockwise(motor_value);
  } else {
    MotorClockwise(abs(motor_value));
  }
  
  Serial.println(encoder_pos);
  delay(10);
}



void encoder(){
  if (digitalRead(ENCODER_B) == HIGH) {
    encoder_pos++;
  } else {
    encoder_pos--;
  }
}

void MotorClockwise(int power) {
  if (power > motor_min) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, power);
  } else {
    //digitalWrite(MOTOR_ENA, LOW);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  }
}

void MotorCounterClockwise(int power) {
  if (power > motor_min) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_ENA, power);
  } else {
    //digitalWrite(MOTOR_ENA, LOW);
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
  }
}
