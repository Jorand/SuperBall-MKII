#include <SPI.h>
/* RADIO */
#include <nRF24L01.h>
#include "RF24.h"
#include "printf.h"
/* OLED */
#include <U8g2lib.h>
/* IMU */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
/* MOTOR */
#include <L298N.h>
/* PID */
#include <PID_v1.h>


#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.println (x)
  #include "printf.h"
#else
  #define DEBUG_PRINT(x)
#endif

/* RADIO */

/* OLED */
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

/* IMU */
#define RATE 20 // 1000/20 = 50 time per seconde
#define BNO055_SAMPLERATE_DELAY_MS (RATE)
Adafruit_BNO055 bno = Adafruit_BNO055(55);
unsigned long sensorLastMillis;

float pitch = 0;

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
L298N m2(M2_ENB, M2_IN4, M2_IN3);

const int motor_min = 70;

/* PID */
double Setpoint, Input, Output;
double Kp = 1;
double Ki = 0;
double Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

/* Let's Go */
String sensor_info;

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData) {
  #ifdef DEBUG
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
  #endif
}

void drawStartScreen(void) {
  u8g2.firstPage();
  do {

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(20, 40, "....");

  } while ( u8g2.nextPage() );
}

void drawScreen(sensors_event_t event) {
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  /* The data should be ignored until the system calibration is > 0 */

  #ifdef DEBUG
    Serial.print("X: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.orientation.z, 4);
    
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.print(mag, DEC);

    Serial.println("");
  #endif

  u8g2.firstPage();
  do {
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
    u8g2.drawStr(0,offset+8,"PITCH:");
    u8g2.setCursor(40,offset+8); u8g2.print(event.orientation.y);

    offset += 10;
    u8g2.drawStr(0,offset+8,"ROLL:");
    u8g2.setCursor(40,offset+8); u8g2.print(event.orientation.z);

    offset += 10;
    u8g2.drawStr(0,offset+8,"YAW:");
    u8g2.setCursor(40,offset+8); u8g2.print(event.orientation.x);

    offset += 10;
    u8g2.setCursor(0,offset+8);
    u8g2.print("Sys:");
    u8g2.print(system, DEC);
    u8g2.print(" G:");
    u8g2.print(gyro, DEC);
    u8g2.print(" A:");
    u8g2.print(accel, DEC);
    u8g2.print(" M:");
    u8g2.print(mag, DEC);

    offset += 13;
    u8g2.setCursor(0,offset+8);
    u8g2.print(sensor_info);
    
  } while ( u8g2.nextPage() );
}

void initSensor(void) {
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    DEBUG_PRINT(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }

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
    DEBUG_PRINT(F("\nNo Calibration Data for this sensor exists in EEPROM"));
    delay(500);
  }
  else
  {
    DEBUG_PRINT(F("\nFound Calibration for this sensor in EEPROM."));
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    displaySensorOffsets(calibrationData);

    DEBUG_PRINT(F("\n\nRestoring Calibration data to the BNO055..."));
    bno.setSensorOffsets(calibrationData);

    DEBUG_PRINT(F("\n\nCalibration data loaded into BNO055"));
    foundCalib = true;
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  if (foundCalib){
    sensor_info = "PLEASE CALIBRATE MAG";
    DEBUG_PRINT(F("Move sensor slightly to calibrate magnetometers"));
    while (!bno.isFullyCalibrated())
    {
      bno.getEvent(&event);
      drawScreen(event);
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }
  else
  {
    sensor_info = "PLEASE CALIBRATE SENSOR";
    DEBUG_PRINT(F("Please Calibrate Sensor: "));
    while (!bno.isFullyCalibrated())
    {
      bno.getEvent(&event);

      drawScreen(event);
      
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  sensor_info = "DONE";

  DEBUG_PRINT(F("\nFully calibrated!"));
  DEBUG_PRINT(F("--------------------------------"));
  DEBUG_PRINT(F("Calibration Results: "));
  adafruit_bno055_offsets_t newCalib;
  bno.getSensorOffsets(newCalib);
  displaySensorOffsets(newCalib);

  DEBUG_PRINT(F("\n\nStoring calibration data to EEPROM..."));

  eeAddress = 0;
  bno.getSensor(&sensor);
  bnoID = sensor.sensor_id;

  EEPROM.put(eeAddress, bnoID);

  eeAddress += sizeof(long);
  EEPROM.put(eeAddress, newCalib);
  DEBUG_PRINT(F("Data stored to EEPROM."));
  sensor_info = "SAVED";

  DEBUG_PRINT(F("\n--------------------------------\n"));
  delay(500);
}

void updateSensor(void) {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);

  pitch = event.orientation.y;

  drawScreen(event);
}

void setup(void) {
  
  #ifdef DEBUG
    Serial.begin(115200);
    DEBUG_PRINT(F("** START **"));
    printf_begin();
  #endif
  
  u8g2.begin();

  drawStartScreen();

  initSensor();

  Input = analogRead(0);
  Setpoint = 0;
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(RATE);
}

void loop(void) {

  // IMU loop without delay
  if (millis() - sensorLastMillis > BNO055_SAMPLERATE_DELAY_MS) {
    sensorLastMillis = millis();

    updateSensor();

    Input = pitch;
    myPID.Compute();

    DEBUG_PRINT(Output);

    if (Output > motor_min) {
      m1.forward();
      m1.setSpeed(Output);
    } 
    else if (Output < motor_min) {
      m1.backward();
      m1.setSpeed(abs(Output));
    } 
    else {
      m1.stop();
      m1.setSpeed(0);
    }
    
  }
}
