#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"
#include "printf.h"

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

bool recievedData = false;

void setup() {
  Serial.begin(9600);
  Serial.println(F("Hello"));
  printf_begin();

  radio.begin();
  // Set the PA Level low for testing in close proximity. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);

  radio.setChannel(108);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  
  radio.openReadingPipe(1, pipe);
  radio.startListening();

  Serial.println(F("Printing receiver details"));
  radio.printDetails();
}

void loop() {

  if (radio.available()) {
    while (radio.available()) {
      radio.read( &remPackage, sizeof(remPackage) );
      Serial.println("New package: '" + (String)remPackage.type + "-" + (String)remPackage.ch1 + "-" + (String)remPackage.ch2 + "-" + (String)remPackage.ch3 + "-" + (String)remPackage.ch4 + "-" + (String)remPackage.ch5 + "-" + (String)remPackage.ch6 + "-" + (String)remPackage.ch7 + "'");
      
      if( remPackage.type <= 2 ){
        recievedData = true;
      }
    }
  }

  if (recievedData == true) {
    
    if(remPackage.type == 0) {
      
      returnData.controlMode = 1;
    
      //radio.writeAckPayload(1, &returnData, sizeof(returnData));
    }
      
    recievedData = false;
  }
}
