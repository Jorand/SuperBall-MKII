#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <PIDController.h>
#include <L298N.h>

#include <U8g2lib.h>

#define RATE 20 // 1000/20 = 50 time per seconde

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

RF24 radio(9, 10); // CE, CSN

struct package {
  int type;   // 0
  int ch1;    // P
  int ch2;    // I
  int ch3;    // D
} remPackage;

const byte addresses[6] = "00001";
uint32_t timeoutTimer = 0;
bool recievedData = false;

/* MOTOR */
#define M1_ENA 5
#define M1_IN1 4
#define M1_IN2 7

#define M2_IN3 8
#define M2_IN4 15
#define M2_ENB 6

#define M1_ENCODER_A 3
#define M1_ENCODER_B 17

#define M2_ENCODER_A 2
#define M2_ENCODER_B 14

L298N m1(M1_ENA, M1_IN1, M1_IN2);

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
float Kp = 1;
float Ki = 0;
float Kd = 0;

volatile long int encoder_pos = 0;
PIDController pos_pid; 
PIDController speed_pid; 
long int last_encoder_pos = 0;
long int last_millis = 0;
int motor_value = 255;

int motor_min = 70;

unsigned int integerValue = 0;
char incomingByte;

void encoder(){
  if (digitalRead(M1_ENCODER_B) == HIGH) {
    encoder_pos++;
  } else {
    encoder_pos--;
  }
}

void MotorClockwise(int power){
  if(power > 100){
  digitalWrite(M1_IN1, HIGH);
  digitalWrite(M1_IN2, LOW);
  analogWrite(M1_ENA, power);
  }else{
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
  }
}

void MotorCounterClockwise(int power){
  if(power > 100){
  digitalWrite(M1_IN2, HIGH);
  digitalWrite(M1_IN1, LOW);
  analogWrite(M1_ENA, power);
  
  }else{
    digitalWrite(M1_IN2, LOW);
    digitalWrite(M1_IN1, LOW);
  }
}

void drawScreen(void) {
 
  u8g2.clearBuffer();
  
    u8g2.setFont(u8g2_font_profont12_tr);
    int offset = 0;
    
    u8g2.setCursor(0,offset+8);
    u8g2.print("P:");
    u8g2.print(Kp);
    u8g2.print(" I:");
    u8g2.print(Ki);
    u8g2.print(" D:");
    u8g2.print(Kd);

    offset += 13;

  u8g2.sendBuffer();
}

void setup(void) {
  Serial.begin(115200);
  Serial.println("Start");

  attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), encoder, RISING);

  u8g2.begin();

  pinMode(M1_ENA, OUTPUT);
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);

  pinMode(M1_ENCODER_A, INPUT);
  pinMode(M1_ENCODER_B, INPUT);

  m1.setSpeed(0);
  m1.stop();

  radio.begin();
  radio.openReadingPipe(1, addresses);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  
  //turn the PID on
  speed_pid.begin();   
  speed_pid.tune(10, 0, 100);    
  speed_pid.limit(-255, 255);
  speed_pid.setpoint(0);
}

void loop(void) {

  while (radio.available())
  {
    // Read and store the received package
    radio.read( &remPackage, sizeof(remPackage) );
    //DEBUG_PRINT( uint64ToAddress(pipe) + " - New package: '" + (String)remPackage.type + "-" + (String)remPackage.ch1 + "-" + (String)remPackage.ch2 + "-" + (String)remPackage.ch3 + "-" + (String)remPackage.ch4 + "-" + (String)remPackage.ch5 + "-" + (String)remPackage.ch6 + "-" + (String)remPackage.ch7 + "'" );
    
    if( remPackage.type <= 2 ){
      timeoutTimer = millis();
      recievedData = true;
    }
  }

  if (recievedData == true) {
    Kp = (float)remPackage.ch1 / 10;
    Ki = (float)remPackage.ch2 / 10;
    Kd = (float)remPackage.ch3;

    speed_pid.tune(Kp, Ki, Kd);    
  }

  if (Serial.available() > 0) {
    integerValue = 0;
    while(1) {
      incomingByte = Serial.read();
      if (incomingByte == '\n') break;
      if (incomingByte == -1) continue;
      integerValue *= 10;
      integerValue = ((incomingByte - 48) + integerValue);
      speed_pid.setpoint(integerValue); 
    }
  }
  
  int rpm_speed = (((float)encoder_pos - (float)last_encoder_pos) / 374.0) *60.0 * (1000/(millis()-last_millis));
  int motor_value = speed_pid.compute(rpm_speed);

  Serial.print(integerValue);
  Serial.print(", ");
  Serial.print(rpm_speed);
  Serial.print(", ");
  Serial.println(motor_value);
  
//  if (motor_value > 0) {
//    if (motor_value > motor_min) {
//      m1.backward();
//      m1.setSpeed(motor_value);
//    }
//    else {
//      m1.stop();
//      m1.setSpeed(0);
//    }
//  } 
//  else {
//    if (abs(motor_value) > motor_min) {
//      m1.forward();
//      m1.setSpeed(abs(motor_value));
//    } 
//    else {
//      m1.stop();
//      m1.setSpeed(0);
//    }
//  }

  if(motor_value > 0){
    MotorCounterClockwise(motor_value);
  }else{
    MotorClockwise(abs(motor_value));
  }

  last_encoder_pos = encoder_pos;
  last_millis = millis();

  drawScreen();

  //Serial.println(encoder_pos);
  delay(10);
}
