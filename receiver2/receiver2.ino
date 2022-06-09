#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <util/atomic.h>
#include <Filters.h>

#include <L298N.h>
#include <PID_v1.h>

#include <FastLED.h>
#include "MegunoLink.h"
TimePlot MyPlot;

#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.println (x)
  #include "printf.h"
#else
  #define DEBUG_PRINT(x)
#endif

/*** Radio ***/
// Initiate RF24 class
RF24 radio(9, 10); // CE, CSN

struct package {  // | cmd        |
  uint8_t type;   // | 0          |
  uint8_t ch1;    // | X          |
  uint8_t ch2;    // | Y          |
  uint8_t ch3;    // | Up         |
  uint8_t ch4;    // | Right      |
  uint8_t ch5;    // | Down       |
  uint8_t ch6;    // | Left       |
  uint8_t ch7;    // | JoyButton  |
} remPackage;

struct callback {
  float battVoltage;
  int controlMode;
} returnData;

const uint64_t pipe = 0xE8E8F0F0E1AA;
//const byte addresses[0][6] = {"00001", "00002"};
const uint8_t defaultChannel = 108;
bool recievedData = false;

uint32_t timeoutTimer = 0;
uint8_t statusMode = 0;
#define CONNECTED 0
#define TIMEOUT 1
const short timeoutMax = 500;

/*** Battery monitering ***/
float batteryVoltage = 0.0;
// Last time data was check
unsigned long lastBatteryCheck;

const int batteryMeasurePin = A6;
float batteryDividerR1 = 100000.0; // resistance of R1 (100K)
float batteryDividerR2 = 10000.0; // resistance of R2 (10K)
const float refVoltage = 1.1;

/*** Motors ***/
#define enA 5
#define in1 4
#define in2 7
#define in3 8
#define in4 15
#define enB 6

L298N m1(enA, in1, in2);
L298N m2(enB, in4, in3);

#define ENCODEROUTPUT 1200

#define M1_ENCODER_A 3
#define M1_ENCODER_B 17

#define M2_ENCODER_A 2
#define M2_ENCODER_B 14

volatile long int m1_encoder_pos = 0;
volatile byte m1_encoder_last = 0;
long m1_pos = 0;
long m1_old_pos = 0;
float m1_speed = 0;
int m1_dir = 0;

volatile long m2_encoder_pos = 0;
volatile byte m2_encoder_last = 0;
long m2_pos = 0;
long m2_old_pos = 0;
long m2_speed = 0;
int m2_dir = 0;

int motor1PWM = 0;

unsigned long encodersNewTime;
unsigned long encodersOldTime;

const byte ppr = 1200, upDatesPerSec = 100;
const float konstant = 60.0 * upDatesPerSec / (ppr * 2);

/*** IMU BNO055 ***/
/* Set the delay between fresh samples */
#define INTERVAL 10 // 10ms per cycle (100Hz)
#define BNO055_SAMPLERATE_DELAY_MS (20) // 1000/20 = 50 time per seconde
Adafruit_BNO055 bno = Adafruit_BNO055(55);
unsigned long sensorLastMillis;

float offsetCompass = 0;
float orientationX = 0;
float orientationY = 0;
float orientationZ = 0;

/*** LED ***/
#define LED_PIN     16
#define NUM_LEDS    8

CRGB leds[NUM_LEDS];

/******/

int controlMode = 0; // 0 = disable radio lost, 1 = manual+stabY, 2 = heading/stabY, 3 = disable by radio
int remoteModeSetting = 1;
int nbModes = 4;

int yaw;
int pitch;
int joybtn;

int rawJoyY = 127, joyY, rawJoyX = 127, joyX, joyXLeft, joyXRight;


// PID yaw
double Pk1 = 10;   // 0.3 | 2
double Ik1 = 0;     // 2 | 0
double Dk1 = 0; // 0.011 | 0

double Setpoint1, Input1, Output1;    // PID variables
PID PID_dir(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

// PID pitch
double Pk2 = 1.4; //1.4
double Ik2 = 0;
double Dk2 = 0;
// double Pk2 = 2.25;
// double Ik2 = 17;
// double Dk2 = 0.024;

double Setpoint2, Input2, Output2;    // PID variables
PID PID_angle(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

// PID encoder M1
double Pk3 = 2.4;     // 1.4    | 2    | 10  | 3 | 1.4
double Ik3 = 0;     // 0.025  | 0    | 0   | 0 | 0
double Dk3 = 0.06;   // 0      | 0.09 | 0.4 | 0.1 | 0.06

double Setpoint3, Input3, Output3;    // PID variables
PID PID_m1(&Input3, &Output3, &Setpoint3, Pk3, Ik3 , Dk3, DIRECT);    // PID Setup

// PID encoder M2
// double Pk4 = 10;
// double Ik4 = 0;
// double Dk4 = 0;

double Setpoint4, Input4, Output4;    // PID variables
PID PID_m2(&Input4, &Output4, &Setpoint4, Pk3, Ik3 , Dk3, DIRECT);    // PID Setup

float output1;
float output2;
float output3;
float output4;

String mySt = "";
char myChar;
boolean stringComplete = false;  // whether the string is complete

double setSpeed1 = 0;
double setSpeed2 = 0;

double lp_freq = 1.5; // 2
FilterOnePole lowpassFilterM1(LOWPASS, lp_freq); //5 Hz
FilterOnePole lowpassFilterM2(LOWPASS, lp_freq); //5 Hz


void setup(void)
{
  analogReference(INTERNAL); // looks like it's more precise for battery voltage reading
  pinMode(batteryMeasurePin, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  FastLED.addLeds<WS2812B, LED_PIN, BRG>(leds, NUM_LEDS);
  FastLED.setBrightness(5);
  setAll(0, 0, 0);
  setPixel(0, 255, 0, 0);
  showStrip();

  #ifdef DEBUG
    Serial.begin(115200);
    //DEBUG_PRINT(F("** SuperBall receiver **"));
    printf_begin();
  #endif

  resetMotor();

  initSensor();
  setCompassOffset();

  // RADIO SETUP
  radio.begin();
  // Set the PA Level low for testing in close proximity. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);

  radio.setChannel(defaultChannel);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openReadingPipe(1, pipe);
  radio.startListening();

  #ifdef DEBUG
    // DEBUG_PRINT(F("Printing receiver details"));
    // radio.printDetails();
  #endif

  setAll(0, 0, 0);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(M1_ENCODER_A, INPUT_PULLUP);
	pinMode(M1_ENCODER_B, INPUT_PULLUP);
  pinMode(M2_ENCODER_A, INPUT_PULLUP);
	pinMode(M2_ENCODER_B, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), detect_m1_a, RISING);
	attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), detect_m2_a, CHANGE);

  PID_dir.SetMode(AUTOMATIC);
  PID_dir.SetOutputLimits(-255, 255);
  PID_dir.SetSampleTime(20);

  PID_angle.SetMode(AUTOMATIC);
  PID_angle.SetOutputLimits(-500, 500);
  PID_angle.SetSampleTime(20);

  PID_m1.SetMode(AUTOMATIC);
  PID_m1.SetOutputLimits(-255, 255);
  PID_m1.SetSampleTime(20);

  PID_m2.SetMode(AUTOMATIC);
  PID_m2.SetOutputLimits(-255, 255);
  PID_m2.SetSampleTime(20);

}

void loop() {

  /* Begin listen for transmission */
  while (radio.available()) {
    radio.read( &remPackage, sizeof(remPackage) );

    // DEBUG_PRINT( uint64ToAddress(pipe) + " - New package: '" + (String)remPackage.type + "-" + (String)remPackage.ch1 + "-" + (String)remPackage.ch2 + "-" + (String)remPackage.ch3 + "-" + (String)remPackage.ch4 + "-" + (String)remPackage.ch5 + "-" + (String)remPackage.ch6 + "-" + (String)remPackage.ch7 + "'" );

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

        joybtn = remPackage.ch7;

        rawJoyY = remPackage.ch2;
        rawJoyX = remPackage.ch1;
        // joyXLeft = map(remPackage.ch1, 127, 0, 0, 255);
        // joyXLeft = constrain(joyXLeft, 0, 255);
        // if ( abs(joyXLeft) < inputCenterDeadzone ) { joyXLeft = 0; }

        // joyXRight = map(remPackage.ch1, 127, 255, 0, 255);
        // joyXRight = constrain(joyXRight, 0, 255);
        // if ( abs(joyXRight) < inputCenterDeadzone ) { joyXRight = 0; }

        remoteAction();

        controlMode = remoteModeSetting;

        returnData.controlMode = controlMode;

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

    //DEBUG_PRINT( uint64ToAddress(pipe) + " - Timeout");
  }
  /* End timeout handling */

  while (Serial.available()) {
		// get the new byte:
		char inChar = (char)Serial.read();
		// add it to the inputString:
		mySt += inChar;
		// if the incoming character is a newline, set a flag so the main loop can
		// do something about it:
		if (inChar == '\n')
			stringComplete = true;
	}

  if (stringComplete) {
		if (mySt.substring(0,2) == "l=")
			setSpeed1 = mySt.substring(2,mySt.length()).toFloat();
    else if (mySt.substring(0,2) == "r=")
			setSpeed2 = mySt.substring(2,mySt.length()).toFloat();
    else if (mySt.substring(0,2) == "p=")
			Pk3 = mySt.substring(2,mySt.length()).toFloat(); //get string after vs_kp
		else if (mySt.substring(0,2) == "i=")
			Ik3 = mySt.substring(2,mySt.length()).toFloat(); //get string after vs_ki
		else if (mySt.substring(0,2) == "d=")
			Dk3 = mySt.substring(2,mySt.length()).toFloat(); //get string after vs_kd

    else if (mySt.substring(0,3) == "ap=")
			Pk2 = mySt.substring(3,mySt.length()).toFloat();
		else if (mySt.substring(0,3) == "ai=")
			Ik2 = mySt.substring(3,mySt.length()).toFloat();
		else if (mySt.substring(0,3) == "ad=")
			Dk2 = mySt.substring(3,mySt.length()).toFloat();

    else if (mySt.substring(0,3) == "dp=")
			Pk1 = mySt.substring(3,mySt.length()).toFloat();
		else if (mySt.substring(0,3) == "di=")
			Ik1 = mySt.substring(3,mySt.length()).toFloat();
		else if (mySt.substring(0,3) == "dd=")
			Dk1 = mySt.substring(3,mySt.length()).toFloat();

		PID_m1.SetTunings(Pk3, Ik3, Dk3);
		PID_m2.SetTunings(Pk3, Ik3, Dk3);
		PID_angle.SetTunings(Pk2, Ik2, Dk2);
		PID_dir.SetTunings(Pk1, Ik1, Dk1);
		// clear the string when COM receiving is completed
		mySt = "";  //note: in code below, mySt will not become blank, mySt is blank until '\n' is received
		stringComplete = false;
	}

  // start timed event
  if (millis() - sensorLastMillis > 20) {
    sensorLastMillis = millis();

    updateSensor();
    // balance();

    // add control mode for disable Magnetometer
    switch (controlMode) {
      case 0:
        resetMotor();
        break;
      case 1:
        // balance();
        balancev2();
        break;
      case 2:
        // balance();
        balancev2();
        break;
      case 3:
        // balance();
        balancev2();
        break;
      case 4:
        resetMotor();
        break;
    }
  }
}

long int last_millis = 0;
long int last_encoder_pos = 0;
long int last_encoder_pos2 = 0;

int current_pos_drive;  // variables for smoothing main drive


void balancev2() {

  joyX = thresholdStick(rawJoyX);
  joyY = thresholdStick(rawJoyY);
  float x = map(rawJoyX, 0, 255, -255, 255);
  float y = map(rawJoyY, 0, 255, -255, 255);

  int directionSpeed = max( abs(x), abs(y) );

  float directionDeg = 0;
  directionDeg = atan2(x, y) * 180 / PI;

  float directionAngle = directionDeg;
  if (directionAngle < 0) {
    directionAngle = directionAngle + 360;
  }

  // float angleToHead = map(orientationX, 0, 360, -180, 180);
  float angleToHead = 0;

  if (orientationX >= offsetCompass) {
    angleToHead = orientationX - offsetCompass;
  }
  else {
    angleToHead = 360 - (offsetCompass - orientationX);
  }

  if (directionSpeed <= 5) {
    directionAngle = angleToHead;
  }

  float error = directionAngle - angleToHead;

  if (error > 180) {
    error = error - 360;
  }
  else if (error < -180) {
    error = error + 360;
  }

  Setpoint1 = 0;
  Input1 = error;
  PID_dir.Compute();

  // MyPlot.SendData("Setpoint1", Setpoint1);
  // MyPlot.SendData("error", error);
  // MyPlot.SendData("angleToHead", angleToHead);
  // MyPlot.SendData("directionSpeed", directionSpeed);

  int target_pos_drive = map(abs(directionSpeed), 0, 255, 0, -80);

  double easing_drive = 700.0;          //modify this value for stick smoothing sensitivity
  easing_drive /= 1000.0;

  // Work out the required travel.
  int diff_drive = target_pos_drive - current_pos_drive;

  // Avoid any strange zero condition
  if( diff_drive != 0.00 ) {
    current_pos_drive += diff_drive * easing_drive;
  }

  Setpoint2 = current_pos_drive;
  Input2 = orientationY+1;
  PID_angle.Compute();

  // MyPlot.SendData("Setpoint2", Setpoint2);
  // MyPlot.SendData("orientationY", orientationY);

  // Motor

  // noInterrupts();
  //   m1_pos = m1_encoder_pos;
  //   m2_pos = m2_encoder_pos;
  // interrupts();
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    m1_pos = m1_encoder_pos;
    m2_pos = m2_encoder_pos;
  }

  int targetSpeed = Output2*-1;

  float yaw = Output1;

  // float yaw = map(Output1, -255, 255, targetSpeed, -targetSpeed);

  float m1TargetSpeed = targetSpeed + yaw;
  float m2TargetSpeed = targetSpeed - yaw;

  int delta = millis()-last_millis;
  int enc_diff = m1_pos;
  float rpm_speed1 = ((float)enc_diff / 300.0) * 60.0 * (1000/delta);

  lowpassFilterM1.input(rpm_speed1);
  float rpm_speed1_f = lowpassFilterM1.output();

  int enc_diff2 = m2_pos;
  float rpm_speed2 = ((float)enc_diff2 * (60000.f / delta)) / 600.f;

  lowpassFilterM2.input(rpm_speed2);
  float rpm_speed2_f = lowpassFilterM2.output();

  Setpoint3 = m1TargetSpeed;
  Input3 = rpm_speed1_f;
  PID_m1.Compute();

  Setpoint4 = m2TargetSpeed;
  Input4 = rpm_speed2_f;
  PID_m2.Compute();

  // Serial.print("target:");
  // Serial.print(targetSpeed);
  // Serial.print(" rpm_speed:");
  // Serial.print(rpm_speed1_f);
  // Serial.print(" rpm_speed2:");
  // Serial.print(rpm_speed2);
  // Serial.print(" rpm_speed2_f:");
  // Serial.print(rpm_speed2_f);
  // duration = 0;
  // Serial.print(" rpm_speed_f:");
  // Serial.print(rpm_speed_f);
  // Serial.println();

  // MyPlot.SendData("target", targetSpeed);
  // MyPlot.SendData("rpm_speed1_f", rpm_speed1_f);
  // MyPlot.SendData("rpm_speed2_f", rpm_speed2_f);

  // MyPlot.SendData("Output3", Output3);
  // MyPlot.SendData("Output4", Output4);

  setMotorSpeed(Output3, Output4);

  last_millis = millis();
  last_encoder_pos = m1_pos;
  last_encoder_pos2 = m2_pos;
  // m1_encoder_pos = 0;
  m1_pos = 0;
  m2_pos = 0;
  m1_encoder_pos = 0;
  m2_encoder_pos = 0;
}

void balance() {
  // ici le code pour l'équilibre

  // 1. get Setpoint
  // 2. get data imu
  // 3. pid from Setpoint
  // 4. output to motor speed

  // 5. yaw offset
  // 6. yaw pid
  // 7. apply yaw correction to motor speed

  // 8. motor speed pid


  // threshold remote data
  joyX = thresholdStick(rawJoyX);
  joyY = thresholdStick(rawJoyY);

  joybtn = remPackage.ch7;

  double easing_drive = 800; //modify this value for stick smoothing sensitivity
  easing_drive /= 1000;

  int target_pos_drive = joyY/2;
  // Work out the required travel.
  int diff_drive = target_pos_drive - current_pos_drive;

  // Avoid any strange zero condition
  // if( diff_drive != 0.00 ) {
  //   current_pos_drive += diff_drive * easing_drive;
  // }

  Setpoint2 = current_pos_drive;
  Input2 = orientationY;
  PID_angle.Compute();
  // Serial.print("Setpoint2:");
  // Serial.print(Setpoint2);
  // Serial.print(" orientationY:");
  // Serial.print(orientationY);
  // Serial.print(" output2:");
  // Serial.print(Output2);
  // Serial.println();
  // MyPlot.SendData("joyY", joyY);
  // MyPlot.SendData("Setpoint2", Setpoint2);
  // MyPlot.SendData("orientationY", orientationY);

  ///

  // int targetSpeed = Output2*-1; //RPM

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    m1_pos = m1_encoder_pos;
    m2_pos = m2_encoder_pos;
  }

  encodersNewTime = millis();
  // m1_speed = (m1_pos - m1_old_pos) * 1000 / (encodersNewTime - encodersOldTime); // encoder ticks per second

  // m1_speed = ((m1_pos - m1_old_pos) * (1000/INTERVAL)) * 60 / ENCODEROUTPUT;
  // m2_speed = ((m2_pos - m2_old_pos) * (1000/INTERVAL)) * 60 / ENCODEROUTPUT;

  // (encoder tick * 1s/INTERVAL) * 60 / encodercount = rpm
  // m1_speed = ((m1_pos - m1_old_pos) * (1000/INTERVAL)) * 60 / ENCODEROUTPUT; // rpm
  // m2_speed = ((m2_pos - m2_old_pos) * (1000/INTERVAL)) * 60 / ENCODEROUTPUT; // rpm

  // targetSpeed = 100; // position
  // targetSpeed = 250*sin(prevT/1e6); // postiton sin
  // targetSpeed = 100/3.0*currT/1.0e6; // speed
  int targetSpeed = 100; //rpm


  //https://www.youtube.com/watch?v=HRaZLCBFVDE&t=79s
  // (64/4) * 18.75 = 300

  float m1_enc_count_sec = (m1_pos - m1_old_pos) * (1000/INTERVAL); // encoder count/s
  float m1_rpm = m1_enc_count_sec / 300.0 * 60.0; // 600 = count per revolution
  //lowpassFilterM1.input(m1_rpm);
  //float m1_rpm_filtered = lowpassFilterM1.output();
  //m1_speed = m1_rpm_filtered;

  float m2_enc_count_sec = (m2_pos - m2_old_pos) * (1000/INTERVAL);
  float m2_rpm = m2_enc_count_sec / 300.0 * 60.0;
  // lowpassFilterM2.input(m2_rpm);
  // m2_speed = lowpassFilterM2.output();

  // float delta = ((float)(encodersNewTime - encodersOldTime)) / 1000.0;
  // float konstant = 60.0 * 100 / (600 * 2);
  // float m1test = (m1_pos - m1_old_pos) * konstant;
  //
  // lowpassFilterM1.input(m1test);
  //
  // float m1testF = lowpassFilterM1.output();

  int rpm_speed = (((float)m1_pos - (float)m1_old_pos) / 300.0) * 60.0 * (1000/(millis()-encodersOldTime));

  // m1_speed = m1_pos;

  // MyPlot.SendData("target", targetSpeed);
  // MyPlot.SendData("m1_pos", m1_pos);
  // MyPlot.SendData("m2_pos", m2_pos);
  // MyPlot.SendData("delta", delta);
  // MyPlot.SendData("m1_test", m1test);
  // MyPlot.SendData("m1testF", m1testF);
  // MyPlot.SendData("rpm_speed", rpm_speed);
  // MyPlot.SendData("m2_rpm", m2_rpm*-1);
  // MyPlot.SendData("m1_speed", m1_speed);
  // MyPlot.SendData("m2_speed", m2_speed*-1);

  Serial.print("target:");
  Serial.print(targetSpeed);
  Serial.print(" rpm_speed:");
  Serial.print(rpm_speed);
  // Serial.print(" m1:");
  // Serial.print(m1_speed);
  // Serial.print(" m2:");
  // Serial.print(m2_speed*-1);
  // Serial.print(" m1_pos:");
  // Serial.print(m1_pos);
  Serial.println();
  m1_old_pos = m1_pos;
  m2_old_pos = m2_pos;
  encodersOldTime = encodersNewTime;

  // MyPlot.SendData("rawJoyX", rawJoyX);
  // MyPlot.SendData("rawJoyY", rawJoyY);

  float yaw = map(joyX, -127, 127, -200, 200);

  // MyPlot.SendData("yaw", yaw);

  float m1SpeedY = Output3 + yaw;
  float m2SpeedY = Output4 + yaw;

  Setpoint3 = targetSpeed;
  Input3 = rpm_speed;
  PID_m1.Compute();

  Setpoint4 = targetSpeed*-1;
  Input4 = m2_speed;
  PID_m2.Compute();

  // MyPlot.SendData("target", targetSpeed);
  // MyPlot.SendData("m1_speed", m1_speed);
  // MyPlot.SendData("Output3", Output3);
  // MyPlot.SendData("m2_speed", Output4*-1);

  // Serial.println();
  // setMotorSpeed(m1SpeedY, m2SpeedY);
  setMotorSpeed(Output3, 0);
}

int inputCenterDeadzone = 4;

int thresholdStick (int pos) {

    // get zero centre position
    pos = pos - 127;

    // threshold value for control sticks
    if (pos > inputCenterDeadzone) {
      pos = pos - inputCenterDeadzone;
    }
    else if (pos < -inputCenterDeadzone) {
      pos = pos + inputCenterDeadzone;
    }
    else {
      pos = 0;
    }

    return pos;
}

void detect_m1_a() {
	// m1_dir = digitalRead(M1_ENCODER_B) ? 1 : -1; //read direction of motor
	// m1_encoder_pos+=m1_dir; //increasing encoder at new pulse
  if (digitalRead(M1_ENCODER_B) == HIGH) {
    m1_encoder_pos++;
  }
  else {
    m1_encoder_pos--;
  }
}


void detect_m2_a() {
	// m2_dir = digitalRead(M2_ENCODER_B) ? 1 : -1; //read direction of motor
	// m2_encoder_pos+=m2_dir; //increasing encoder at new pulse

  int Lstate = digitalRead(M2_ENCODER_A);
  if((m2_encoder_last == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(M2_ENCODER_B);
    if(val == LOW && m2_dir)
    {
      m2_dir = false; //Reverse
    }
    else if(val == HIGH && !m2_dir)
    {
      m2_dir = true;  //Forward
    }
  }
  m2_encoder_last = Lstate;

  if(!m2_dir)  m2_encoder_pos++;
  else  m2_encoder_pos--;
}

void motorControl() {

}

int motorDeadzone = 10;
int motorMin = 100; // 60 no load
int wheel2a = 0;
int wheel2b = 0;

void setMotorSpeed(float m1Speed, float m2Speed) {

  if (m1Speed > motorDeadzone && joybtn == 0) {
    m1Speed = constrain(m1Speed,0,255);
    m1Speed = map(abs(m1Speed), 0, 255, motorMin, 255);
    m1.setSpeed(m1Speed);
    m1.backward();
  }
  else if (m1Speed < -motorDeadzone && joybtn == 0) {
    wheel2a = abs(m1Speed);
    wheel2a = constrain(wheel2a,0,255);
    wheel2a = map(abs(wheel2a), 0, 255, motorMin, 255);
    m1.setSpeed(wheel2a);
    m1.forward();
  }
  else {
    m1.stop();
    m1.setSpeed(0);
  }

  if (m2Speed > motorDeadzone && joybtn == 0) {
    m2Speed = constrain(m2Speed,0,255);
    m2Speed = map(abs(m2Speed), 0, 255, motorMin, 255);
    m2.setSpeed(m2Speed);
    m2.forward();
  }
  else if (m2Speed < -motorDeadzone && joybtn == 0) {
    wheel2b = abs(m2Speed);
    wheel2b = constrain(wheel2b,0,255);
    wheel2b = map(abs(wheel2b), 0, 255, motorMin, 255);
    m2.setSpeed(wheel2b);
    m2.backward();
  }
  else {
    m2.stop();
    m2.setSpeed(0);
  }

  // MyPlot.SendData("m1Speed", m1Speed);
  // MyPlot.SendData("wheel2a", wheel2a);
  // MyPlot.SendData("m2Speed", m2Speed);
  // MyPlot.SendData("wheel2b", wheel2b);
}

unsigned long lastModeButtonPress;

void remoteAction() {
  // here all action triggered by the remote

  // Set front side
  if (remPackage.ch3 && remPackage.ch6) {
    Serial.println("Place the light in front of you …");
    setCompassOffset();
  }

  else if (remPackage.ch4) {
    // do something

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

void updateSensor() {

    /* Get a new sensor event */
    sensors_event_t event;
    bno.getEvent(&event);

    orientationX = event.orientation.x;
    orientationY = event.orientation.y;
    orientationZ = event.orientation.z;

    /* Display the floating point data */
    // Serial.print("X: ");
    // Serial.print(event.orientation.x, 4);
    // Serial.print("\tY: ");
    // Serial.print(event.orientation.y, 4);
    // Serial.print("\tZ: ");
    // Serial.print(event.orientation.z, 4);

    /* Optional: Display calibration status */
    //displayCalStatus();

    /* Optional: Display sensor status (debug only) */
    //displaySensorStatus();

    /* New line for the next sample */
    //Serial.println("");

    /* Wait the specified delay before requesting new data */
    //delay(BNO055_SAMPLERATE_DELAY_MS);
}

void getData() {
  if (millis() - lastBatteryCheck >= 500) {

    lastBatteryCheck = millis();

    float batteryVoltage = 0.0;
    int total = 0;

    for (int i = 0; i < 10; i++) {
      total += analogRead(batteryMeasurePin);
    }

    batteryVoltage = (refVoltage / 1024.0) * ((float)total / 10.0);

    batteryVoltage = batteryVoltage / ( batteryDividerR2 / (batteryDividerR1 + batteryDividerR2) );
    // DEBUG_PRINT(batteryVoltage);
    returnData.battVoltage = batteryVoltage;
  }
}

void resetMotor() {
  rawJoyY = 127;
  rawJoyX = 127;
  m1.setSpeed(0);
  m2.setSpeed(0);
  m1.stop();
  m2.stop();
}

void setCompassOffset() {
  // save current heading position as offset
  sensors_event_t event;
  bno.getEvent(&event);
  offsetCompass = event.orientation.x;
  // Serial.println("\nOffset : ");
  // Serial.print(offsetCompass);
  // Serial.println("\nCentering Done!");
  // Serial.println("");
}

void initSensor() {
  //Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */
      //Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while (1);
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
      //Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      //Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      // displaySensorOffsets(calibrationData);

      //Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      //Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  // delay(1000);

  /* Display some basic information on this sensor */
  //displaySensorDetails();

  /* Optional: Display current status */
  //displaySensorStatus();

 //Crystal must be configured AFTER loading calibration data into BNO055.
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  if (foundCalib){
      // Serial.println("Move sensor slightly to calibrate magnetometers");
      // while (!bno.isFullyCalibrated())
      // {
      //     bno.getEvent(&event);
      //     delay(BNO055_SAMPLERATE_DELAY_MS);
      // }
  }
  else
  {
      Serial.println("Please Calibrate Sensor: ");
      while (!bno.isFullyCalibrated())
      {
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

  // Serial.println("\nFully calibrated!");
  // Serial.println("--------------------------------");
  // Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  // displaySensorOffsets(newCalib);

  // Serial.println("\n\nStoring calibration data to EEPROM...");

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  // Serial.println("Data stored to EEPROM.");

  // Serial.println("\n--------------------------------\n");
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
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
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
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

void showStrip() {
  FastLED.show();
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
  // FastLED
  leds[Pixel].r = red;
  leds[Pixel].g = green;
  leds[Pixel].b = blue;
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue);
  }
  showStrip();
}