/**************************************
  Author Jorand - 2018
  Inspired by https://github.com/SolidGeek/nRF24-Esk8-Remote/

  ** Upload Settings **
  Board: "Arduino Nano"
  Processor: "ATmega328P (Old Bootloader)"
**************************************/

//#include <avr/pgmspace.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include "RF24.h"

//#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.println (x)
  #include "printf.h"
#else
  #define DEBUG_PRINT(x)
#endif

// Defining the type of display used (128x32)
U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const unsigned char PROGMEM signal_transmitting_bits[] = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x0f,
  0x33, 0x0f, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char PROGMEM signal_connected_bits[] = {
  0x18, 0x00, 0x0c, 0x00, 0xc6, 0x00, 0x66, 0x00, 0x23, 0x06, 0x33, 0x09,
  0x33, 0x09, 0x23, 0x06, 0x66, 0x00, 0xc6, 0x00, 0x0c, 0x00, 0x18, 0x00
};

const unsigned char PROGMEM signal_noconnection_bits[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x09,
  0x00, 0x09, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Defining struct to handle callback data (auto ack)
struct callback {
  float battVoltage;
  int controlMode;
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


struct callback returnData;
struct package remPackage;

// Pin defination
const uint8_t batteryMeasurePin = A2;

const int  analogInputCount     = 2;       // number of joystick input
const uint8_t analogInputPin[]  = {A1,A3}; // Y, X
const bool analogInputReverse[] = { 0, 0}; // Set to 1 for reverse channel

const uint8_t digitalInputPin[]  = {8,7,6,5,4}; // Joy Button, Up, Right, Down, Left
const int  digitalInputCount     = 5;           // number of button input


short analogInputValue[analogInputCount], analogOutputValue[analogInputCount];

unsigned long lastDigitalInputPress[digitalInputCount];
bool digitalInputStatus[digitalInputCount];

// Input and output configuration
int inputMin      = 0;
int inputCenter   = 518; // value when stick is center. ex: 512
int inputMax      = 1023;
int inputCenterDeadzone = 5;

// Battery monitering
const float minVoltage = 3.3;
const float maxVoltage = 4.1;
const float refVoltage = 5.0;
int nbCells = 2;

// Defining variables for NRF24 communication
const uint64_t pipe = 0xE8E8F0F0E1AA; // 0xE8E8F0F0E1LL If you change the pipe, you will need to update it on the receiver to.
const uint8_t defaultChannel = 108; // Above most WiFi frequencies
unsigned long lastTransmission;
long transmitInterval = 40; // Transmit once every 40 millisecond (1000/40 = 25 time per seconde)
bool connected = false;
short failCount;

// Defining variables for OLED display
String tempString;
uint8_t displayData = 0;
unsigned long lastSignalBlink;

bool signalBlink = false;

// Instantiating RF24 object for NRF24 communication
RF24 radio(9, 10);

void setup() {

  #ifdef DEBUG
    Serial.begin(9600);
    printf_begin();
  #endif

  for ( uint8_t i = 0; i < sizeof(analogInputPin); i++ ) {
    pinMode(analogInputPin[i], INPUT);
  }
  for ( uint8_t i = 0; i < sizeof(digitalInputPin); i++ ) {
    pinMode(digitalInputPin[i], INPUT_PULLUP);
  }
  pinMode(batteryMeasurePin, INPUT);

  // Screen setup
  u8g2.begin();
  drawStartScreen();

  // RF24 setup radio communication
  radio.begin();
  radio.setChannel(defaultChannel);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.openWritingPipe(pipe);

  #ifdef DEBUG
    DEBUG_PRINT(F("Printing transmitter details"));
    radio.printDetails();
  #endif

  DEBUG_PRINT(F("Setup complete - begin transmitting"));
}

void loop() {

  readAnalogInputValues();
  readDigitalInputValues();

  //DEBUG_PRINT("INPUT: " + (String)analogInputValue[0] + "-" + (String)analogInputValue[1] + "-" + (String)digitalInputIsActive(digitalInputPin[0]) + "-" + (String)digitalInputStatus[1] + "-" + (String)digitalInputIsActive(digitalInputPin[2]) + "-" + (String)digitalInputIsActive(digitalInputPin[3]) + "-" + (String)digitalInputIsActive(digitalInputPin[4]));
  //DEBUG_PRINT("OUTPUT: " + (String)analogOutputValue[0] + "-" + (String)analogOutputValue[1]);

  remPackage.ch1 = analogOutputValue[0];  // X
  remPackage.ch2 = analogOutputValue[1];  // Y
  remPackage.ch3 = digitalInputIsActive(digitalInputPin[1]); // 1 Up
  remPackage.ch4 = digitalInputIsActive(digitalInputPin[3]); // 3 Right
  remPackage.ch5 = digitalInputIsActive(digitalInputPin[2]); // 2 Down
  remPackage.ch6 = digitalInputIsActive(digitalInputPin[4]); // 4 Left
  remPackage.ch7 = digitalInputStatus[0]; // O Joy button

  DEBUG_PRINT("RC: " + (String)remPackage.ch1 + " , " + (String)remPackage.ch2 + " , " + (String)remPackage.ch3 + " , " + (String)remPackage.ch4 + " , " + (String)remPackage.ch5 + " , " + (String)remPackage.ch6 + " , " + (String)remPackage.ch7 );

  // Transmit to receiver
  transmitToReceiver();

  // Call function to update display
  updateMainDisplay();

}

// Function used to transmit the joysticks values, and receive the telemetry realtime data.
void transmitToReceiver() {
  // Transmit once every 40 millisecond
  if (millis() - lastTransmission > transmitInterval) {
    lastTransmission = millis();

    boolean sendSuccess = false;
    // Transmit the remPackage package
    sendSuccess = radio.write(&remPackage, sizeof(remPackage));

    // Listen for an acknowledgement reponse (return telemetry data).
    while (radio.isAckPayloadAvailable()) {
      radio.read(&returnData, sizeof(returnData));
    }

    if (sendSuccess == true)
    {
      // Transmission was a succes
      failCount = 0;
      sendSuccess = false;

      //DEBUG_PRINT(F("Transmission succes"));
    } else {
      // Transmission was not a succes
      failCount++;

      //DEBUG_PRINT(F("Failed transmission"));
    }

    // If lost more than 5 transmissions, we can assume that connection is lost.
    if (failCount < 5) {
      connected = true;
    } else {
      connected = false;
    }
  }
}

void updateMainDisplay() {

  u8g2.firstPage();
  do {

    drawThrottle();
    drawPage();
    drawBatteryLevel();
    drawSignal();

  } while ( u8g2.nextPage() );
}

// Return true if button is activated, false otherwice
bool digitalInputIsActive(uint8_t pin) {
  if (digitalRead(pin) == LOW)
    return true;
  else
    return false;
}

void readAnalogInputValues() {
  // Joystick reading can be noisy, lets make an average reading.
  unsigned short total[analogInputCount];

  for ( uint8_t i = 0; i < analogInputCount; i++ ) {
    total[i] = 0;
    for ( uint8_t ii = 0; ii < 10; ii++ ) {
      total[i] += analogRead(analogInputPin[i]);
    }
  }

  for ( uint8_t i = 0; i < analogInputCount; i++ ) {
    analogInputValue[i] = total[i]/10;

    if (analogInputReverse[i]) {
      analogInputValue[i] =  map(analogInputValue[i], inputMin, inputMax, inputMax, inputMin);
    }

    if (analogInputValue[i] >= inputCenter + inputCenterDeadzone) {
      analogOutputValue[i] = constrain(map(analogInputValue[i], inputCenter + inputCenterDeadzone, inputMax, 127, 255), 127, 255);
    }
    else if (analogInputValue[i] <= inputCenter - inputCenterDeadzone) {
      analogOutputValue[i] = constrain(map(analogInputValue[i], inputMin, inputCenter - inputCenterDeadzone, 0, 127), 0, 127);
    }
    else {
      // Default value if stick is in deadzone
      analogOutputValue[i] = 127;
    }
  }

}

void readDigitalInputValues() {

  for ( uint8_t i = 0; i < digitalInputCount; i++ ) {

    if (digitalInputIsActive(digitalInputPin[i])) {

      if (millis() - lastDigitalInputPress[i] >= 250 ) {
        digitalInputStatus[i] = !digitalInputStatus[i];
        lastDigitalInputPress[i] = millis();
      }

    }
  }

}

// Function used to indicate the remotes battery level.
int batteryLevel() {
  float voltage = batteryVoltage();

  if (voltage <= minVoltage) {
    return 0;
  } else if (voltage >= maxVoltage) {
    return 100;
  } else {
    return (voltage - minVoltage) * 100 / (maxVoltage - minVoltage);
  }
}

// Function to calculate and return the remotes battery voltage.
float batteryVoltage() {
  float batteryVoltage = 0.0;
  int total = 0;

  for (int i = 0; i < 10; i++) {
    total += analogRead(batteryMeasurePin);
  }

  batteryVoltage = (refVoltage / 1024.0) * ((float)total / 10.0);

  return batteryVoltage;
}

void drawStartScreen() {

  u8g2.firstPage();
  do {

    u8g2.setFont(u8g2_font_logisoso22_tr);
    u8g2.drawStr(20, 27, "Hello !");

  } while ( u8g2.nextPage() );

  delay(1500);
}

void drawTitleScreen(String title) {
  u8g2.firstPage();
  do {
    drawString(title, 20, 12, 20, u8g2_font_helvR10_tr );
  } while ( u8g2.nextPage() );
  delay(1500);
}

void drawPage() {

  float value;
  uint8_t decimals;
  String suffix;
  String prefix;

  short first, last;

  uint8_t x = 0;
  uint8_t y = 16;

  value = returnData.battVoltage;
  suffix = F("v");
  prefix = F("BATTERY");
  decimals = 1;

  if(returnData.controlMode == 0) {
    prefix = F("DISABLE");
  }
  else if (returnData.controlMode == 1) {
    prefix = F("HEADING");
  }
  else if (returnData.controlMode == 2) {
    prefix = F("HEADING 2");
  }
  else if (returnData.controlMode == 3) {
    prefix = F("MANUAL");
  }
  else if (returnData.controlMode == 4) {
    prefix = F("DISABLE");
  }
  else {
    prefix = F("BATTERY");
  }

  // Display prefix (title)
  drawString(prefix, 10, x, y - 1, u8g2_font_profont12_tr );

  // Split up the float value: a number, b decimals.
  first = abs(floor(value));
  last = value * pow(10, 3) - first * pow(10, 3);

  // Add leading zero
  if (first <= 9) {
    tempString = "0" + (String)first;
  } else {
    tempString = (String)first;
  }

  // Display numbers
  drawString(tempString, 10, x + 55, y + 13, u8g2_font_logisoso22_tn );

  // Display decimals
  tempString = "." + (String)last;
  drawString(tempString, decimals + 2, x + 86, y - 1, u8g2_font_profont12_tr);

  // Display suffix
  drawString(suffix, 10, x + 88, y + 13, u8g2_font_profont12_tr);
}

void drawString(String text, uint8_t lenght, uint8_t x, uint8_t y, const uint8_t  *font){

  static char textBuffer[20];

  text.toCharArray(textBuffer, lenght);

  u8g2.setFont(font);
  u8g2.drawStr(x, y, textBuffer);
}

void drawThrottle() {
  int x = 0;
  int y = 18;
  uint8_t width;

  // Draw throttle
  //u8g2.drawHLine(x, y, 52);
  //u8g2.drawVLine(x, y, 10);
  //u8g2.drawVLine(x + 52, y, 10);
  //u8g2.drawHLine(x, y + 10, 5);
  //u8g2.drawHLine(x + 52 - 4, y + 10, 5);
  int lineHeight = 3;
  u8g2.drawHLine(x, y + 11, 52);

  float b = 0;
  if (returnData.battVoltage <= (minVoltage*nbCells)) {
    b = 0;
  } else if (returnData.battVoltage >= (maxVoltage*nbCells)) {
    b = 100;
  } else {
    b = (returnData.battVoltage - (minVoltage*nbCells)) * 100 / ((maxVoltage*nbCells) - (minVoltage*nbCells));
  }

  width = map(b, 0, 100, 0, 49);

  for (int i = 0; i < width; i++) {
    //u8g2.drawVLine(x + i, y + 2, 7);
    u8g2.drawVLine(x + i, y + 9, lineHeight);
  }

  int battDisplay = (int)b;

  drawString( (String)battDisplay +" %", 10, x, y + 7, u8g2_font_helvR08_tr);
}

void drawSignal() {
  // Position on OLED
  uint8_t x = 114;
  uint8_t y = 17;

  if (connected == true) {
    //if (buttonRightStatus) { // triggerActive()
      //u8g2.drawXBMP(x, y, 12, 12, signal_transmitting_bits);
    //} else {
      u8g2.drawXBMP(x, y, 12, 12, signal_connected_bits);
    //}
  } else {
    if (millis() - lastSignalBlink > 500) {
      signalBlink = !signalBlink;
      lastSignalBlink = millis();
    }

    if (signalBlink == true) {
      u8g2.drawXBMP(x, y, 12, 12, signal_connected_bits);
    } else {
      u8g2.drawXBMP(x, y, 12, 12, signal_noconnection_bits);
    }
  }
}

void drawBatteryLevel() {
  // Position on OLED
  uint8_t x = 108;
  uint8_t y = 4;

  int level = batteryLevel();

  u8g2.drawFrame(x + 2, y, 18, 9);
  u8g2.drawBox(x, y + 2, 2, 5);

  for (int i = 0; i < 5; i++) {
    uint8_t p = round((100 / 5) * i);
    if (p <= level)
    {
      u8g2.drawBox(x + 4 + (3 * i), y + 2, 2, 5);
    }
  }
}
