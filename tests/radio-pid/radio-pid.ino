/*
  Arduino uno
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "printf.h"

#define POT_1 A0
#define POT_2 A2
#define POT_3 A3

RF24 radio(9, 10); // CE, CSN

struct package {
  int type;   // 0
  int ch1;    // P
  int ch2;    // I
  int ch3;    // D
} remPackage;

const byte addresses[6] = "00001";

unsigned long lastTransmission;
long transmitInterval = 40;

void setup() {
  Serial.begin(115200);
  Serial.println(F("Hello"));
  printf_begin();

  pinMode(POT_1, INPUT);
  pinMode(POT_2, INPUT);
  pinMode(POT_3, INPUT);

  radio.begin();
  radio.setPALevel(RF24_PA_LOW);

  //radio.setChannel(108);
  
  radio.openWritingPipe(addresses);
  radio.stopListening();

  radio.printDetails();
}

void loop() {

  int val1 = analogRead(POT_1);
  int val2 = analogRead(POT_2);
  int val3 = analogRead(POT_3);
  
  //Serial.println("VAL: " + (String)val1 + " , " + (String)val2 + " , " + (String)val3);
 
  //val1 = map(val1, 0, 1023, 0, 255);
  //val2 = map(val2, 0, 1023, 0, 255);
  //val3 = map(val3, 0, 1023, 0, 255);
  
  remPackage.ch1 = val1;
  remPackage.ch2 = val2;
  remPackage.ch3 = val3;

  Serial.println("RC: " + (String)remPackage.ch1 + " , " + (String)remPackage.ch2 + " , " + (String)remPackage.ch3);

  // Transmit to receiver
  if (millis() - lastTransmission > transmitInterval) {
    lastTransmission = millis();

    radio.write(&remPackage, sizeof(remPackage));
  }
  delay(transmitInterval);
}
