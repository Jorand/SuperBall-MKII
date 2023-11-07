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

/*** IMU BNO055 ***/
/* Set the delay between fresh samples */
#define INTERVAL 20
#define BNO055_SAMPLERATE_DELAY_MS (INTERVAL) // 1000/20 = 50 time per seconde
Adafruit_BNO055 bno = Adafruit_BNO055(55);
unsigned long sensorLastMillis;

float offsetCompass = 0;
float orientationX = 0;
float orientationY = 0;
float orientationZ = 0;

/******/

int controlMode = 0; // 0 = disable radio lost, 1 = manual+stabY, 2 = heading/stabY, 3 = disable by radio
int remoteModeSetting = 1;
int nbModes = 4;

int yaw;
int pitch;
int joybtn;

int rawJoyY = 127, joyY, rawJoyX = 127, joyX, joyXLeft, joyXRight;


// PID yaw
double Pk1 = 2;   // 0.3 | 2
double Ik1 = 0;     // 2 | 0
double Dk1 = 0; // 0.011 | 0

double Setpoint1, Input1, Output1;    // PID variables
PID PID_dir(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

// PID pitch
double Pk2 = 0.4; //1.4
double Ik2 = 0;
double Dk2 = 0;
// double Pk2 = 2.25;
// double Ik2 = 17;
// double Dk2 = 0.024;

double Setpoint2, Input2, Output2;    // PID variables
PID PID_angle(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

String mySt = "";
char myChar;
boolean stringComplete = false;  // whether the string is complete

double setSpeed1 = 0;
double setSpeed2 = 0;

void setup(void)
{
  analogReference(INTERNAL); // looks like it's more precise for battery voltage reading
  pinMode(batteryMeasurePin, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

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

  digitalWrite(LED_BUILTIN, LOW);


  PID_dir.SetMode(AUTOMATIC);
  PID_dir.SetOutputLimits(-255, 255);
  PID_dir.SetSampleTime(10);

  PID_angle.SetMode(AUTOMATIC);
  PID_angle.SetOutputLimits(-255, 255);
  PID_angle.SetSampleTime(10);

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

    // DEBUG_PRINT( uint64ToAddress(pipe) + " - Timeout");
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

		PID_angle.SetTunings(Pk2, Ik2, Dk2);
		PID_dir.SetTunings(Pk1, Ik1, Dk1);
		// clear the string when COM receiving is completed
		mySt = "";  //note: in code below, mySt will not become blank, mySt is blank until '\n' is received
		stringComplete = false;
	}

  // start timed event
  if (millis() - sensorLastMillis > 10) {
    sensorLastMillis = millis();

    updateSensor();

    // add control mode for disable Magnetometer
    switch (controlMode) {
      case 0:
        resetMotor();
        break;
      case 1:
        balancev3();
        break;
      case 2:
        balancev3();
        break;
      case 3:
        balancev3();
        break;
      case 4:
        resetMotor();
        break;
    }
  }
}

void balancev3() {

  joyX = thresholdStick(rawJoyX);
  joyY = thresholdStick(rawJoyY);
  float x = map(rawJoyX, 0, 255, -255, 255);
  float y = map(rawJoyY, 0, 255, -255, 255);

  // int directionSpeed = max( abs(x), abs(y) );
  int directionSpeed = abs(y);

  int target_pos_drive = map(abs(directionSpeed), 0, 255, 0, -230);

  Setpoint2 = target_pos_drive;
  Input2 = orientationY+1;
  PID_angle.Compute();

  float yaw = map(rawJoyX, 0, 255, -100, 100);

  float m1Speed = -Output2 + yaw;
  float m2Speed = -Output2 - yaw;

  // Serial.print("Setpoint2:");
  // Serial.print(Setpoint2);
  // Serial.print(",");
  // Serial.print("Input2:");
  // Serial.print(Input2);
  // Serial.print(",");  
  // Serial.print("m1Speed:");
  // Serial.print(m1Speed);
  // Serial.print(",");  
  // Serial.print("m2Speed:");
  // Serial.println(m2Speed);

  setMotorSpeed(m1Speed, m2Speed);
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

void motorControl() {

}

int motorDeadzone = 5;
int motorMin = 70; // 60 no load
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
  Serial.print("m1Speed:");
  Serial.print(m1Speed);
  Serial.print(",");
  Serial.print("wheel2a:");
  Serial.print(wheel2a);
  Serial.print(",");  
  Serial.print("m2Speed:");
  Serial.print(m2Speed);
  Serial.print(",");  
  Serial.print("wheel2b:");
  Serial.println(wheel2b);
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
  // Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */
      // Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
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
      // Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      // Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      displaySensorOffsets(calibrationData);

      // Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      // Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  // delay(1000);

  /* Display some basic information on this sensor */
  // displaySensorDetails();

  /* Optional: Display current status */
  // displaySensorStatus();

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
  displaySensorOffsets(newCalib);

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
