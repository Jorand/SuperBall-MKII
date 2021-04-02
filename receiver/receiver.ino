/**************************************
  Author Jorand Le Pape

  ** Upload Settings **
  Board: "Arduino Nano"
  Processor: "ATmega328P"
 **************************************/

#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#include <L298N.h>

#include <PID_v1.h>

/*
 * Ressources
How to Build an Arduino Self-Balancing Robot - https://maker.pro/arduino/projects/build-arduino-self-balancing-robot/
The PIDDYBOT - DIY Arduino Balancing Robot - https://youtu.be/eyG5GFpSRSM
How to Visually Tune PID Control Loops - https://youtu.be/-bQdrvSLqpg
BB8 Droid -  http://www.instructables.com/id/3D-Printed-Remote-Controlled-BB8-Droid/

*/


#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.println (x)
  #include "printf.h"
#else
  #define DEBUG_PRINT(x)
#endif

struct vescValues {
  float battVoltage;
};

// Transmit and receive package
struct package {  // | cmd        |
  uint8_t type;   // | 0          |
  uint8_t ch1;    // | X          |
  uint8_t ch2;    // | Y          |
  uint8_t ch3;    // | Up         |
  uint8_t ch4;    // | Right      |
  uint8_t ch5;    // | Down       |
  uint8_t ch6;    // | Left       |
  uint8_t ch7;    // | JoyButton  |
};

// Defining struct to handle callback data (auto ack)
struct callback {
  float battVoltage;
  int controlMode;
};

float batteryVoltage = 0.0;

struct callback returnData;
struct package remPackage;

// Defining variables for NRF24 communication
const uint64_t pipe = 0xE8E8F0F0E1AA;
const uint8_t defaultChannel = 108;
uint32_t timeoutTimer = 0;
bool recievedData = false;

uint8_t statusMode = 0;
#define CONNECTED 0
#define TIMEOUT 1

// Last time data was check
unsigned long lastBatteryCheck;


const uint8_t defaultInputCenter = 127;
const short timeoutMax = 500;

// Initiate RF24 class
RF24 radio(9, 10);

// motors
#define enA 5
#define in1 4
#define in2 7

#define in3 8
#define in4 15
#define enB 6

L298N m1(enA, in1, in2);
L298N m2(enB, in4, in3);

int m1D = 1, m2D = 1;
bool motorStop = false;

//initial speed
unsigned short theSpeed = 0;
int inputCenter = 127;
int outMin      = 0;
int motorMaxV   = 10;
int inputCenterDeadzone = 4;

// Battery monitering
const int batteryMeasurePin = A2;
float batteryDividerR1 = 100000.0; // resistance of R1 (100K)
float batteryDividerR2 = 9970.0; // resistance of R2 (10K)
const float refVoltage = 5.0;

// Buzzer
char buzzerPin = 17;

// BNO055
#define RATE 20 // 1000/20 = 50 time per seconde
#define BNO055_SAMPLERATE_DELAY_MS (RATE)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
unsigned long sensorLastMillis;

float offsetCompass = 0;
float orientationX = 0;
float orientationY = 0;
float orientationZ = 0;

// PID
int motor1PWM = 0, motor2PWM = 0; //set motor starting point
int motor1Offset = 60, motor2Offset = 64; //motor offset, when they start to spin
int motor1Start = motor1Offset + 60;
int motor2Start = motor2Offset + 56; // make sure both motors start at same value

int rawJoyY = 127, joyY, rawJoyX = 127, joyX, joyXLeft, joyXRight;

float roll;
float pitch;
int batt_attenuation;

int current_pos_drive;  // variables for smoothing main drive
int target_pos_drive;
int pot_drive;   // target position/inout
int diff_drive; // difference of position
double easing_drive;

// PID 1
double Setpoint1, Input1, Output1, Output1a, Output2a; // Variables for PID controller

double Pk1 = 2.2; // 10
double Ik1 = 0;  //1
double Dk1 = 0;  // 0.35

PID myPID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1, Dk1, DIRECT); // PID Setup - PITCH stability

// PID 2
double Setpoint2, Input2, Output2; // Variables for PID controller

double Pk2 = 1;
double Ik2 = 0;
double Dk2 = 0.01;

PID myPID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2, Dk2, DIRECT); // PID Setup

int controlMode = 0; // 0 = disable radio lost, 1 = manual+stabY, 2 = heading/stabY, 3 = disable by radio
int remoteModeSetting = 1;
int nbModes = 4;

void setup() {
  tone(buzzerPin, 2637, 1000/12); // Hello

  #ifdef DEBUG
    Serial.begin(9600);
    DEBUG_PRINT(F("** LionBall-remote receiver **"));
    printf_begin();
  #endif

  // RADIO SETUP
  radio.begin();
  radio.setChannel(defaultChannel);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, pipe);
  radio.startListening();

  #ifdef DEBUG
    DEBUG_PRINT(F("Printing receiver details"));
    radio.printDetails();
  #endif

  // PIN SETUP
  pinMode(batteryMeasurePin, INPUT);

  //pinMode(statusLedPin, OUTPUT);

  // MOTORS SETUP
  resetMotor();

  // IMU SETUP
  /* Initialise the sensor */
  initSensor();

  setCompassOffset();

  tone(buzzerPin, 2637, 1000/12);   delay((1000/12) * 1.30);
  tone(buzzerPin, 2637, 1000/12);   delay((1000/12) * 1.30);
  tone(buzzerPin, 2637, 1000/12);   delay((1000/12) * 1.30);

  // PID
  myPID1.SetMode(AUTOMATIC); // set output limits for PID controller
  myPID1.SetOutputLimits(-255, 255);
  myPID1.SetSampleTime(RATE); // sample time for PID controller

  // PID 2
  myPID2.SetMode(AUTOMATIC); // set output limits for PID controller
  myPID2.SetOutputLimits(-255, 255);
  myPID2.SetSampleTime(RATE); // sample time for PID controller
}

void loop() {

  /* Begin listen for transmission */
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
  /* End listen for transmission */

  getData();

  /* Begin data handling */
  if (recievedData == true) {

    statusMode = CONNECTED;

    switch (remPackage.type) {
      case 0:

        rawJoyY = remPackage.ch2;
        //joyY = map(rawJoyY, joyInputMin, joyInputMax, -255, 255);
        //if ( abs(joyY) < inputCenterDeadzone ) { joyY = 0; }

        rawJoyX = remPackage.ch1;
        //rawJoyX = map(rawJoyX, 0, 255, -255, 255);
        //joyX = map(rawJoyX, joyInputMin, joyInputMax, -255, 255);

        joyXLeft = map(remPackage.ch1, 127, 0, 0, 255);
        joyXLeft = constrain(joyXLeft, 0, 255);
        if ( abs(joyXLeft) < inputCenterDeadzone ) { joyXLeft = 0; }

        joyXRight = map(remPackage.ch1, 127, 255, 0, 255);
        joyXRight = constrain(joyXRight, 0, 255);
        if ( abs(joyXRight) < inputCenterDeadzone ) { joyXRight = 0; }

        //Serial.print("Remote: " + (String)remPackage.ch1 + " , " + (String)remPackage.ch2 + "-" + (String)remPackage.ch3 + "-" + (String)remPackage.ch4 + "-" + (String)remPackage.ch5 + "-" + (String)remPackage.ch6 + "-" + (String)remPackage.ch7 + "'" );
        //Serial.println("  ");

        doActions();

        controlMode = remoteModeSetting;

        returnData.controlMode = controlMode;

        // The next time a transmission is received, the returnData will be sent back in acknowledgement
        radio.writeAckPayload(1, &returnData, sizeof(returnData));
        break;
      case 1:
        controlMode = 0;
        resetMotor();
        break;
    }

    recievedData = false;
  }
  /* End data handling */

  /* Begin timeout handling */
  if ( timeoutMax <= ( millis() - timeoutTimer ) )
  {
    // No speed is received within the timeout limit.
    statusMode = TIMEOUT;
    resetMotor();
    controlMode = 0;
    timeoutTimer = millis();

    DEBUG_PRINT( uint64ToAddress(pipe) + " - Timeout");
  }
  /* End timeout handling */

  if (millis() - sensorLastMillis > BNO055_SAMPLERATE_DELAY_MS) {
    sensorLastMillis = millis();

    updateSensor();

    switch (controlMode) {
      case 0:
        resetMotor();
        break;
      case 1:
        motorControlPIDandHeading();
        //motorControlPID();
        break;
      case 2:
        motorControlPIDandHeading();
        break;
      case 3:
        motorControlPID();
        break;
      case 4:
        resetMotor();
        break;
    }


    //updateMotor();
  }

}

void updateSensor() {

  /* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);

  orientationX = event.orientation.x;
  orientationY = event.orientation.y;
  orientationZ = event.orientation.z;

  /* Display the floating point data */
  //diplayData(event);

  /* Optional: Display calibration status */
  //displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  //Serial.println("");
    //DEBUG_PRINT("");

}

void diplayData(sensors_event_t event) {
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  Serial.print("\tRemote: " + (String)remPackage.ch1 + " , " + (String)remPackage.ch2 + " , " + (String)remPackage.ch3 + " , " + (String)remPackage.ch4 + " , " + (String)remPackage.ch5 + " , " + (String)remPackage.ch6 + " , " + (String)remPackage.ch7 + "" );
}

void setCompassOffset() {
  sensors_event_t event;
  bno.getEvent(&event);
  offsetCompass = event.orientation.x;
  Serial.println("\nOffset : ");
  Serial.print(offsetCompass);
  Serial.println("\nCentering Done!");
  Serial.println("");
  tone(buzzerPin, 2637, 1000/12);
}

unsigned long lastModeButtonPress;

void doActions()
{

  // Set front side
  if (remPackage.ch3 && remPackage.ch6) {
    Serial.println("Place the light in front of you …");
    setCompassOffset();
  }

  else if (remPackage.ch4) {

    //ghost busters
    tone(buzzerPin, 987, 133);   delay(133 * 1.30);
    tone(buzzerPin, 2489, 267); delay(267 * 1.30);
    tone(buzzerPin, 987, 267);  delay(267 * 1.30);
    tone(buzzerPin, 2217, 267); delay(267 * 1.30);
    tone(buzzerPin, 880, 267);  delay(267 * 1.30);
       delay(1071);
    tone(buzzerPin, 987, 133);  delay(133 * 1.30);
    tone(buzzerPin, 987, 133);  delay(133 * 1.30);
    tone(buzzerPin, 987, 133);  delay(133 * 1.30);
    tone(buzzerPin, 987, 133);  delay(133 * 1.30);
    tone(buzzerPin, 880, 267);  delay(267 * 1.30);
    tone(buzzerPin, 987, 267);  delay(267 * 1.30);
       delay(1071);
  }

  else if (remPackage.ch5) {

    if (millis() - lastModeButtonPress >= 250 ) {

      remoteModeSetting ++;

      if (remoteModeSetting > nbModes) {
        remoteModeSetting = 1;
      }

      lastModeButtonPress = millis();
    }
  }

}

void motorControlPIDandHeading(){

  float x = map(rawJoyX, 0, 255, -255, 255);
  float y = map(rawJoyY, 0, 255, -255, 255);
  //if ( abs(x) < inputCenterDeadzone ) { x = 0; }

  // DIRECTION SPEED
  int directionSpeed = max( abs( x ), abs( y ) );

  //Serial.print("directionSpeed: ");
  //Serial.print(directionSpeed);

  // DIRECTION DEGREE
  float directionDeg = 0;
  directionDeg = atan2(x, y) * 57;

  //Serial.print(" directionDeg: ");
  //Serial.print(directionDeg);

  // DIRECTION ANGLE
  float directionAngle = directionDeg;
  if (directionAngle < 0) {
    directionAngle = directionAngle + 360;
  }

  //Serial.print(" directionAngle: ");
  //Serial.print(directionAngle);

  // ANGLE TO HEAD
  float angleToHead = 0;

  if (orientationX >= offsetCompass) {
    angleToHead = orientationX - offsetCompass;
  }
  else {
    angleToHead = 360 - (offsetCompass - orientationX);
  }

  //Serial.print(" angleToHead: ");
  //Serial.print(angleToHead);

   if (directionSpeed <= 5) {
    directionAngle = angleToHead;
  }

  // DIFF SHORTEST ANGLE
  float error = directionAngle - angleToHead;

  //Serial.print(" rawerror: ");
  //Serial.print(error);
  if (error > 180) {
    error = error - 360;
  }
  else if (error < -180) {
    error = error + 360;
  }

  //Serial.print(" error: ");
  //Serial.print(error);

  Setpoint2 = 0;

  Input2 = error;

  myPID2.Compute();

  //Serial.print(" Output2: ");
  //Serial.print(Output2);

  // PITCH

  target_pos_drive = map(abs(directionSpeed), 0, 255, 0, -80);

  easing_drive = 700;          //modify this value for stick smoothing sensitivity
  easing_drive /= 1000;

  // Work out the required travel.
  diff_drive = target_pos_drive - current_pos_drive;

  // Avoid any strange zero condition
  if( diff_drive != 0.00 ) {
    current_pos_drive += diff_drive * easing_drive;
  }

  Setpoint1 = current_pos_drive;

  pitch = orientationY;

  Input1 = pitch+1;

  myPID1.Compute();

  //Serial.print(" Output1 Pitch: ");
  //Serial.print(Output1);

  float yaw = Output2;

  //Serial.print(" controlMode: ");

  //Serial.println(controlMode);

  if (controlMode == 2) {
    float yaw = map(Output2, -255, 255, Output1, -Output1);
  }

  float m1Speed = Output1 + yaw;
  float m2Speed = Output1 - yaw;

  //Serial.print(" m1Speed: ");
  //Serial.print(m1Speed);

  //Serial.print(" m2Speed: ");
  //Serial.print(m2Speed);

  sendMotorSpeed(m1Speed, m2Speed);


}

float motorDeadzone = 5;

void sendMotorSpeed(float m1Speed, float m2Speed) {
  //mSpeed -255 255

  batt_attenuation = constrain(map(returnData.battVoltage, 0, 8.4, 0, 255), 0, 255);
  batt_attenuation = 255 + (255-batt_attenuation);
  int minMotor1 = map(50, 0, 255, 0, batt_attenuation);
  int minMotor2 = minMotor1; // +5

  if ( m1Speed <= -motorDeadzone ) {
    m1.forward();
    float m1_PWM = constrain(map(abs(m1Speed), 0, 255, minMotor1, batt_attenuation), 0, 255);
    m1.setSpeed(m1_PWM);
  }
  else if (m1Speed > motorDeadzone) {
    m1.backward();
    float m1_PWM = constrain(map(abs(m1Speed), 0, 255, minMotor1, batt_attenuation), 0, 255);
    m1.setSpeed(m1_PWM);
  }
  else {
    m1.stop();
    m1.setSpeed(0);
  }

  if ( m2Speed <= -motorDeadzone ) {
    m2.forward();
    float m2_PWM = constrain(map(abs(m2Speed), 0, 255, minMotor2, batt_attenuation), 0, 255);
    m2.setSpeed(m2_PWM);
  }
  else if (m2Speed > motorDeadzone) {
    m2.backward();
    float m2_PWM = constrain(map(abs(m1Speed), 0, 255, minMotor2, batt_attenuation), 0, 255);
    m2.setSpeed(m2_PWM);
  }
  else {
    m2.stop();
    m2.setSpeed(0);
  }
}

void motorControlPID(){

  //Serial.print("rawJoyY: ");
  //Serial.print(rawJoyY);

  target_pos_drive = map(rawJoyY, 0, 255, 70, -70);

  easing_drive = 800;          //modify this value for stick smoothing sensitivity
  easing_drive /= 1000;

  // Work out the required travel.
  diff_drive = target_pos_drive - current_pos_drive;

  // Avoid any strange zero condition
  if( diff_drive != 0.00 ) {
    current_pos_drive += diff_drive * easing_drive;
  }

  Setpoint1 = current_pos_drive;
  /*
  Serial.print("target_pos_drive: ");
  Serial.print(target_pos_drive);
  Serial.print(" Setpoint: ");
  Serial.print(Setpoint1);
  Serial.print(" pitch: ");
  Serial.print(pitch);
  */

  pitch = orientationY;

  Input1 = pitch+1;

  myPID1.Compute();

  //Serial.print(" Output: ");
  //Serial.print(Output1);

  batt_attenuation = constrain(map(returnData.battVoltage, 0, 8.4, 0, 255), 0, 255);
  batt_attenuation = 255 + (255-batt_attenuation);
  int minMotor1 = map(35, 0, 255, 0, batt_attenuation);
  int minMotor2 = minMotor1; // +5

  //Serial.print(" batt_attenuation: ");
  //Serial.print(batt_attenuation);

  float m1Speed = Output1;
  float m2Speed = Output1;

  float yaw = map(rawJoyX, 0, 255, minMotor1, 255);
  yaw = map(yaw, minMotor1, 255, -100, 100);
/*
  Serial.print(" rawJoyX: ");
  Serial.print(rawJoyX);

  Serial.print(" yaw: ");
  Serial.print(yaw);
*/
  m1Speed = Output1 - yaw;
  m2Speed = Output1 + yaw;
/*
  Serial.print(" minMotor 1: ");
  Serial.print(minMotor1);
  Serial.print(" minMotor 2: ");
  Serial.print(minMotor2);
*/
  if ( m1Speed <= 1 ) {
    Output1a = abs(m1Speed);
    m1.forward();

    float Output1aM1 = constrain(map(Output1a, 0, 255, minMotor1, batt_attenuation), 0, 255);

    //Serial.print(" Output1aM1: ");
    //Serial.print(Output1aM1);

    m1.setSpeed(Output1aM1);
  }
  else if (m1Speed > 1) {
    Output1a = abs(m1Speed);
    m1.backward();

    float Output1aM1 = constrain(map(Output1a, 0, 255, minMotor1, batt_attenuation), 0, 255);

    //Serial.print(" Output1aM1: ");
    //Serial.print(Output1aM1);

    m1.setSpeed(Output1aM1);
  }
  else {
    m1.stop();
    m1.setSpeed(0);
  }

  if ( m2Speed <= 1 ) {
    Output2a = abs(m2Speed);
    m2.forward();

    float Output2aM2 = constrain(map(Output2a, 0, 255, minMotor2, batt_attenuation), 0, 255);

    //Serial.print(" Output2aM2: ");
    //Serial.print(Output2aM2);

    m2.setSpeed(Output2aM2);
  }
  else if (m2Speed > 1) {
    Output2a = abs(m2Speed);
    m2.backward();

    float Output2aM2 = constrain(map(Output2a, 0, 255, minMotor2, batt_attenuation), 0, 255);

    //Serial.print(" Output2aM2: ");
    //Serial.print(Output2aM2);

    m2.setSpeed(Output2aM2);
  }
  else {
    m2.stop();
    m2.setSpeed(0);
  }
}

void updateMotor()
{

  // joyY joyXLeft  joyXRight
  // orientationX offsetCompass

  // reduce noise in center joystick
  if ( abs(joyXLeft) < inputCenterDeadzone ) { joyXLeft = 0; }
  if ( abs(joyXRight) < inputCenterDeadzone ) { joyXRight = 0; }
  if ( abs(joyX) < inputCenterDeadzone ) { joyX = 0; }

  if ( abs(joyY) < inputCenterDeadzone ) { joyY = 0; }

  //DEBUG_PRINT( "remote: 'joyY: " + (String)joyY + " joyX: " + (String)joyX + " joyXLeft: " + (String)joyXLeft + " joyXRight:" + (String)joyXRight );

  // SPEED
  int directionSpeed = max( abs( joyX ), abs( joyY ) );

  // DIRECTION
  // help - https://www.youtube.com/watch?v=9c2HvJDNboE
  float directionDeg = 0;
  directionDeg = atan2(joyX, joyY) * 57;

  float directionAngle = directionDeg;
  if (directionAngle < 0) {
    directionAngle = directionAngle + 360;
  }

  float angleToHead = 0;

  DEBUG_PRINT( "orientationX: " + (String)orientationX + " offsetCompass: " + (String)offsetCompass );

  if (orientationX >= offsetCompass) {
    angleToHead = orientationX - offsetCompass;
  }
  else {
    angleToHead = 360 - (offsetCompass - orientationX);
  }

  float error = directionAngle - angleToHead;
  if (error > 180) {
    error = 360 - error;
  }
  else if (error < -180) {
    error = 360 + error;
  }

  DEBUG_PRINT( "angleToHead: " + (String)angleToHead + " directionAngle: " + (String)directionAngle + " directionDeg: " + (String)directionDeg + " error: " + (String)error );

  int leftSpeed = 0;
  int rightSpeed = 0;

  int deadzone = 25;

  if (error < -deadzone) {
    // counter clockwinse
    float multi = constrain(abs(error)/100, 0, 1);
    leftSpeed  = -directionSpeed; //  * multi
    rightSpeed =  directionSpeed;
    DEBUG_PRINT( "counter clockwinse | multi: " + (String)multi + " leftSpeed: " + (String)leftSpeed + " rightSpeed: " + (String)rightSpeed );
  }
  else if (error > deadzone) {
    // clockwinse
    float multi = constrain(error/100, 0, 1);
    leftSpeed  =  directionSpeed;
    rightSpeed = -directionSpeed;
    DEBUG_PRINT( "clockwinse | multi: " + (String)multi  + " error: " + (String)error + " leftSpeed: " + (String)leftSpeed + " rightSpeed: " + (String)rightSpeed );
  }
  else {
    leftSpeed = directionSpeed;
    rightSpeed = directionSpeed;
  }

  //float motionYaw = map(error, -180, 180, -255, 255);

  // MIXER
  //int v1 = directionSpeed + motionYaw;
  //int v2 = directionSpeed - motionYaw;

  int v1 = leftSpeed;
  int v2 = rightSpeed;

  if (directionSpeed < inputCenterDeadzone) {
    v1 = v2 = 0;
    /*
    if (orientationY > 12) {
      v1 = map(orientationY, 0, 180, 0, 255);
      v2 = map(orientationY, 0, 180, 0, 255);
    }
    else if (orientationY < -12) {
      v1 = -map(orientationY, 0, -180, 0, 255);
      v2 = -map(orientationY, 0, -180, 0, 255);
    }
    */
  }

  DEBUG_PRINT( "vIn: " + (String)v1 + " " + (String)v2 );

  float outMax = 255;

  // Cut the speed to not exceed the max speed
  int v1Max = abs( v1 );
  if ( v1Max > outMax ) {
      float scale = (float)outMax / (float)v1Max;
      v1 = v1 * scale;
   }

  int v2Max = abs( v2 );
  if ( v2Max > outMax ) {
    float scale = (float)outMax / (float)v2Max;
    v2 = v2 * scale;
  }

  // set motors direction

  //if (motorStop) {

    if ( v1 < 0 ) {  m1.backward(); } else if ( v1 > 0 ) { m1.forward(); } else { m1.stop(); }
    if ( v2 < 0 ) {  m2.backward(); } else if ( v2 > 0 ) { m2.forward(); } else { m2.stop(); }

  //}
  //else {
  //  if ( v1 < 0 ) {  m1.backward(); m1D = 0; } else if ( v1 > 0 ) { m1.forward();  m1D = 1; }
  //    else { if (m1D > 0) { m1.forward(); } else { m1.backward(); } }
  //
  //  if ( v2 < 0 ) {  m2.backward(); m2D = 0; } else if ( v2 > 0 ) { m2.forward();  m2D = 1; }
  //    else { if (m2D > 0) { m2.forward(); } else { m2.backward(); } }
  //}

  DEBUG_PRINT( "vOut: " + (String)v1 + " " + (String)v2 );

  // setSpeed take only positive value
  v1 = abs( v1 );
  v2 = abs( v2 );

  v1 = map(v1, 0, 255, motor1Offset, 255);
  v2 = map(v2, 0, 255, motor2Offset, 255);

  m1.setSpeed(v1);
  m2.setSpeed(v2);
}

void resetMotor()
{
  rawJoyY = 127;
  rawJoyX = 127;
  m1.setSpeed(0);
  m2.setSpeed(0);
  m1.stop();
  m2.stop();
}

void getData() {

  if (millis() - lastBatteryCheck >= 250) {

    lastBatteryCheck = millis();

    float batteryVoltage = 0.0;
    int total = 0;

    for (int i = 0; i < 10; i++) {
      total += analogRead(batteryMeasurePin);
    }

    batteryVoltage = (refVoltage / 1024.0) * ((float)total / 10.0);

    batteryVoltage = batteryVoltage / ( batteryDividerR2 / (batteryDividerR1 + batteryDividerR2) );
    //DEBUG_PRINT(batteryVoltage);
    returnData.battVoltage = batteryVoltage;
  }
}

void initSensor() {

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }

  delay(1000);

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /*
  *  Look for the sensor's unique ID at the beginning oF EEPROM.
  *  This isn't foolproof, but it's better than nothing.
  */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
      Serial.println(F("\nNo Calibration Data for this sensor exists in EEPROM"));
      delay(500);
  }
  else
  {
      Serial.println(F("\nFound Calibration for this sensor in EEPROM."));
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      displaySensorOffsets(calibrationData);

      Serial.println(F("\n\nRestoring Calibration data to the BNO055..."));
      bno.setSensorOffsets(calibrationData);

      Serial.println(F("\n\nCalibration data loaded into BNO055"));
      foundCalib = true;
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

 //Crystal must be configured AFTER loading calibration data into BNO055.
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  if (foundCalib){
      Serial.println(F("Move sensor slightly to calibrate magnetometers"));
      while (!bno.isFullyCalibrated())
      {
          bno.getEvent(&event);
          delay(BNO055_SAMPLERATE_DELAY_MS);
      }
  }
  else
  {
      Serial.println(F("Please Calibrate Sensor: "));
      while (!bno.isFullyCalibrated())
      {
          Serial.println(F("Please Calibrate Sensor: "));
          bno.getEvent(&event);

          Serial.print("X: ");
          Serial.print(event.orientation.x, 4);
          Serial.print("\tY: ");
          Serial.print(event.orientation.y, 4);
          Serial.print("\tZ: ");
          Serial.print(event.orientation.z, 4);

          /* Optional: Display calibration status */
          displayCalStatus();

          /* New line for the next sample */
          Serial.println("");

          /* Wait the specified delay before requesting new data */
          delay(BNO055_SAMPLERATE_DELAY_MS);
      }
  }

  Serial.println(F("\nFully calibrated!"));
  Serial.println(F("--------------------------------"));
  Serial.println(F("Calibration Results: "));
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  Serial.println(F("\n\nStoring calibration data to EEPROM..."));

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  Serial.println(F("Data stored to EEPROM."));

  Serial.println(F("\n--------------------------------\n"));
  delay(500);
}

String uint64ToString(uint64_t number)
{
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  if(part1 == 0){
    return String(part2, DEC);
  }

  return String(part1, DEC) + String(part2, DEC);
}

String uint64ToAddress(uint64_t number)
{
  unsigned long part1 = (unsigned long)((number >> 32)); // Bitwise Right Shift
  unsigned long part2 = (unsigned long)((number));

  return String(part1, HEX) + String(part2, HEX);
}


/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
    */
/**************************************************************************/
void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print(F("Sensor:       ")); Serial.println(sensor.name);
    Serial.print(F("Driver Ver:   ")); Serial.println(sensor.version);
    Serial.print(F("Unique ID:    ")); Serial.println(sensor.sensor_id);
    Serial.print(F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print(F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print(F("Resolution:   ")); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
    */
/**************************************************************************/
void displaySensorStatus(void)
{
    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
    */
/**************************************************************************/
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    /* The data should be ignored until the system calibration is > 0 */
    Serial.print("\t");
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);
}

/**************************************************************************/
/*
    Display the raw calibration offset and radius data
    */
/**************************************************************************/
void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print(F("Accelerometer: "));
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print(F("\nGyro: "));
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print(F("\nMag: "));
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print(F("\nAccel Radius: "));
    Serial.print(calibData.accel_radius);

    Serial.print(F("\nMag Radius: "));
    Serial.print(calibData.mag_radius);
}
