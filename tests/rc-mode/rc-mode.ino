#include <SPI.h>
/* RADIO */
#include <nRF24L01.h>
#include "RF24.h"
#include "printf.h"
/* MOTOR */
#include <L298N.h>


#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.println (x)
  #include "printf.h"
#else
  #define DEBUG_PRINT(x)
#endif


/* RADIO */
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
const uint8_t defaultChannel = 108;
uint32_t timeoutTimer = 0;
bool recievedData = false;

uint8_t statusMode = 0;
#define CONNECTED 0
#define TIMEOUT 1

const uint8_t defaultInputCenter = 127;
const short timeoutMax = 500;

int rawJoyY = 127, joyY, rawJoyX = 127, joyX, joyXLeft, joyXRight;

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

int m1D = 1, m2D = 1;
bool motorStop = false;

//initial speed
unsigned short theSpeed = 0;
int inputCenter = 127;
int outMin      = 0;
int motorMaxV   = 10;
int inputCenterDeadzone = 4;

int controlMode = 0; // 0 = disable radio lost, 1 = manual+stabY, 2 = heading/stabY, 3 = disable by radio
int remoteModeSetting = 1;


void setup() {
  Serial.begin(115200);
  Serial.println(F("Hello"));
  printf_begin();

  radio.begin();
  #ifdef DEBUG
    // Set the PA Level low for testing in close proximity. RF24_PA_MAX is default.
    radio.setPALevel(RF24_PA_LOW);
  #endif

  radio.setChannel(108);
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  
  radio.openReadingPipe(1, pipe);
  radio.startListening();

  Serial.println(F("Printing receiver details"));
  radio.printDetails();

  /* MOTOR */
  resetMotor();
}

void loop() {

  /* Begin listen for transmission */
  if (radio.available()) {
    while (radio.available()) {
      radio.read( &remPackage, sizeof(remPackage) );
      //Serial.println("New package: '" + (String)remPackage.type + "-" + (String)remPackage.ch1 + "-" + (String)remPackage.ch2 + "-" + (String)remPackage.ch3 + "-" + (String)remPackage.ch4 + "-" + (String)remPackage.ch5 + "-" + (String)remPackage.ch6 + "-" + (String)remPackage.ch7 + "'");
      
      if( remPackage.type <= 2 ){
        recievedData = true;
      }
    }
  }
  /* End listen for transmission */

  /* Begin data handling */
  if (recievedData == true) {
    
    switch (remPackage.type) {
      case 0:

        rawJoyY = remPackage.ch2;
        joyY = map(rawJoyY, 0, 255, -255, 255);
        if ( abs(joyY) < inputCenterDeadzone ) { joyY = 0; }

        rawJoyX = remPackage.ch1;
        joyX = map(rawJoyX, 0, 255, -255, 255);
        if ( abs(joyX) < inputCenterDeadzone ) { joyX = 0; }

        joyXLeft = map(remPackage.ch1, 127, 0, 0, 255);
        joyXLeft = constrain(joyXLeft, 0, 255); 
        if ( abs(joyXLeft) < inputCenterDeadzone ) { joyXLeft = 0; }

        joyXRight = map(remPackage.ch1, 127, 255, 0, 255);
        joyXRight = constrain(joyXRight, 0, 255); 
        if ( abs(joyXRight) < inputCenterDeadzone ) { joyXRight = 0; }

        //Serial.print("Remote: " + (String)remPackage.ch1 + " , " + (String)remPackage.ch2 + "-" + (String)remPackage.ch3 + "-" + (String)remPackage.ch4 + "-" + (String)remPackage.ch5 + "-" + (String)remPackage.ch6 + "-" + (String)remPackage.ch7 + "'" );
        //Serial.println("  ");
        
        //doActions();

        controlMode = 1;

        returnData.controlMode = controlMode;
        
        // The next time a transmission is received, the returnData will be sent back in acknowledgement 
        //radio.writeAckPayload(1, &returnData, sizeof(returnData));
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
    
    DEBUG_PRINT(" - Timeout");
  }
  /* End timeout handling */

  updateMotor();
  
}

void updateMotor()
{
  // reduce noise in center joystick
  DEBUG_PRINT( "remote: 'joyY: " + (String)joyY + " joyX: " + (String)joyX + " joyXLeft: " + (String)joyXLeft + " joyXRight:" + (String)joyXRight );

  // MIXER
  //int v1 = directionSpeed + motionYaw;
  //int v2 = directionSpeed - motionYaw;

  int directionSpeed = joyY;

  int motor1_value = directionSpeed - joyX;
  int motor2_value = directionSpeed + joyX;

  if ( motor1_value > 0 ) {
    MotorClockwise(motor1_value, m1);
  }
  else {
    MotorCounterClockwise(abs(motor1_value), m1);
  }

  if ( motor2_value > 0 ) {
    MotorCounterClockwise(motor2_value, m2);
  }
  else {
    MotorClockwise(abs(motor2_value), m2);
  }
  
}

void MotorClockwise(int power, L298N &motor){
  if(power > 100){
    motor.forward();
    motor.setSpeed(power);
  }
  else{
    motor.stop();
  }
}

void MotorCounterClockwise(int power, L298N &motor){
  if(power > 100){
    motor.backward();
    motor.setSpeed(power);
  }
  else{
    motor.stop();
  }
}

void resetMotor()
{
  rawJoyY = 127;
  rawJoyX = 127;
  joyY = 0;
  joyX = 0;
  m1.setSpeed(0);
  m2.setSpeed(0);
  m1.stop();
  m2.stop();
}
