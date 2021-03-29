#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

String screen_info;

void displayCalStatus(void) {
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

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData) {
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

void drawScreen(sensors_event_t event) {
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);

  displayCalStatus();

  Serial.println("");

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont12_tr);
    u8g2.drawStr(0,8,"PITCH:");
    u8g2.setCursor(40,8); u8g2.print(event.orientation.y);
    
    u8g2.drawStr(0,18,"ROLL:");
    u8g2.setCursor(40,18); u8g2.print(event.orientation.z);
    
    u8g2.drawStr(0,28,"YAW:");
    u8g2.setCursor(40,28); u8g2.print(event.orientation.x);

    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    u8g2.setCursor(0,38);
    u8g2.print("Sys:");
    u8g2.print(system, DEC);
    u8g2.print(" G:");
    u8g2.print(gyro, DEC);
    u8g2.print(" A:");
    u8g2.print(accel, DEC);
    u8g2.print(" M:");
    u8g2.print(mag, DEC);

    u8g2.setCursor(0,50);
    u8g2.print(screen_info);
    
  } while ( u8g2.nextPage() );
}
 
void setup(void) {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  u8g2.begin();

  u8g2.firstPage();
  do {

    u8g2.setFont(u8g2_font_logisoso22_tr);
    u8g2.drawStr(20, 27, "Hello !");

  } while ( u8g2.nextPage() );
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  int eeAddress = 0;
  long bnoID;
  bool foundCalib = false;

  EEPROM.get(eeAddress, bnoID);

  adafruit_bno055_offsets_t calibrationData;
  sensor_t sensor;

  /* Look for the sensor's unique ID at the beginning oF EEPROM. */
  bno.getSensor(&sensor);
  if (bnoID != sensor.sensor_id)
  {
    screen_info = "NONE";
    Serial.println("\nNo Calibration Data for this sensor exists in EEPROM");
    delay(500);
  }
  else
  {
    screen_info = "RESTORED";
    Serial.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);

    displaySensorOffsets(calibrationData);

    Serial.println("\n\nRestoring Calibration data to the BNO055...");
    bno.setSensorOffsets(calibrationData);

    Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);

  sensors_event_t event;
  bno.getEvent(&event);
  if (foundCalib){
    screen_info = "CALIBRATE MAG";
    Serial.println("Move sensor slightly to calibrate magnetometers");
    while (!bno.isFullyCalibrated())
    {
      bno.getEvent(&event);
      drawScreen(event);
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }
  else
  {
    screen_info = "CALIBRATE ALL";
    Serial.println("Please Calibrate Sensor: ");
    while (!bno.isFullyCalibrated())
    {
      bno.getEvent(&event);

      drawScreen(event);
      
      delay(BNO055_SAMPLERATE_DELAY_MS);
    }
  }

  screen_info = "DONE";

  Serial.println("\nFully calibrated!");
  Serial.println("--------------------------------");
  Serial.println("Calibration Results: ");
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
  screen_info = "SAVED";

  Serial.println(F("\n--------------------------------\n"));
  delay(500);
}
 
void loop(void) 
{

  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  drawScreen(event);
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
