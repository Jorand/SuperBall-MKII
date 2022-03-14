#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

#include <FastLED.h>

#include <L298N.h>


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
#define enA 6
#define in1 7
#define in2 8
#define enB 5
#define in3 2
#define in4 4

L298N m1(enA, in1, in2);
L298N m2(enB, in4, in3);


/*** IMU BNO055 ***/
/* Set the delay between fresh samples */
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

void setup(void)
{
  #ifdef DEBUG
    Serial.begin(115200);
    DEBUG_PRINT(F("** SuperBall receiver **"));
    printf_begin();
  #endif

  analogReference(INTERNAL); // looks like it's more precise for battery voltage reading
  pinMode(batteryMeasurePin, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  FastLED.addLeds<WS2812B, LED_PIN, BRG>(leds, NUM_LEDS);
  FastLED.setBrightness(5);
  setAll(0, 0, 0);
  setPixel(0, 255, 0, 0);
  showStrip();

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
    DEBUG_PRINT(F("Printing receiver details"));
    radio.printDetails();
  #endif

  setAll(0, 0, 0);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  /* Begin listen for transmission */
  while (radio.available()) {
    radio.read( &remPackage, sizeof(remPackage) );
    /*
    Serial.println("New package: '" + (String)remPackage.type + "-" + (String)remPackage.ch1 + "-" + (String)remPackage.ch2 + "-" + (String)remPackage.ch3 + "-" + (String)remPackage.ch4 + "-" + (String)remPackage.ch5 + "-" + (String)remPackage.ch6 + "-" + (String)remPackage.ch7 + "'");
    */
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

    if(remPackage.type == 0) {

      controlMode = remoteModeSetting;

      returnData.controlMode = controlMode;

      radio.writeAckPayload(1, &returnData, sizeof(returnData));
    }
    else {
      controlMode = 0;
      resetMotor();
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
        //motorControlPIDandHeading();
        //motorControlPID();
        break;
      case 2:
        //motorControlPIDandHeading();
        break;
      case 3:
        //motorControlPID();
        break;
      case 4:
        resetMotor();
        break;
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
  //rawJoyY = 127;
  //rawJoyX = 127;
  m1.setSpeed(0);
  m2.setSpeed(0);
  m1.stop();
  m2.stop();
}

void setCompassOffset() {
  sensors_event_t event;
  bno.getEvent(&event);
  offsetCompass = event.orientation.x;
  Serial.println("\nOffset : ");
  Serial.print(offsetCompass);
  Serial.println("\nCentering Done!");
  Serial.println("");
}

void initSensor() {
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
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
      Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
      delay(500);
  }
  else
  {
      Serial.println("\nFound Calibration for this sensor in EEPROM.");
      eeAddress += sizeof(long);
      EEPROM.get(eeAddress, calibrationData);

      displaySensorOffsets(calibrationData);

      Serial.println("\n\nRestoring Calibration data to the BNO055...");
      bno.setSensorOffsets(calibrationData);

      Serial.println("\n\nCalibration data loaded into BNO055");
      foundCalib = true;
  }

  // delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

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

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  Serial.println("\n\nStoring calibration data to EEPROM...");

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  Serial.println("Data stored to EEPROM.");

  Serial.println("\n--------------------------------\n");
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