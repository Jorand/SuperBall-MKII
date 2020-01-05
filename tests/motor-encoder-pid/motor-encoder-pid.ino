#include <PIDController.h>

const int encoderA = 3;
const int encoderB = 17;

const int enA = 5;
const int in1 = 4;
const int in2 = 7;

volatile long int encoder_pos = 0;
PIDController pos_pid;
int motor_value = 255;
int motor_min = 70;
unsigned int integerValue=0;  // Max value is 65535
char incomingByte;

void setup() {

  Serial.begin(9600);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoder, RISING);

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
  if (digitalRead(encoderB) == HIGH) {
    encoder_pos++;
  } else {
    encoder_pos--;
  }
}

void MotorClockwise(int power) {
  if (power > motor_min) {
    analogWrite(enA, power);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else {
    digitalWrite(enA, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void MotorCounterClockwise(int power) {
  if (power > motor_min) {
    analogWrite(enA, power);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(enA, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
