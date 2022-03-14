#include <ELClient.h>
#include <ELClientWebServer.h>
#include <L298N.h>
#include <PID_v1.h>
#include "printf.h"

/* MOTOR */
#define M1_EN 5
#define M1_IN1 4
#define M1_IN2 7
#define M1_ENCODER_A 3
#define M1_ENCODER_B 17

#define M2_EN 6
#define M2_IN1 8
#define M2_IN2 15
#define M2_ENCODER_A 2
#define M2_ENCODER_B 14

L298N m1(M1_EN, M1_IN1, M1_IN2);
L298N m2(M2_EN, M2_IN1, M2_IN2);

int sleep_time = 100;

int m1_step = 0;
int m1_last_step = 0;
int m1_rps = 0;

int m1_dir = 0;

int m2_step = 0;
int m2_dir = 0;


double Setpoint, Input, Output;
double Kp=5, Ki=10, Kd=0.04;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

String mySt = "";
char myChar;
boolean stringComplete = false;  // whether the string is complete

ELClient esp(&Serial);
ELClientWebServer webServer(&esp);

// Callback made form esp-link to notify that it has just come out of a reset. This means we
// need to initialize it!
void resetCb(void) {
  Serial.println("EL-Client (re-)starting!");
  bool ok = false;
  do {
    ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
    if (!ok) Serial.println("EL-Client sync failed!");
  } while(!ok);
  Serial.println("EL-Client synced!");

  webServer.setup();
}

void setup() {
	Serial.begin(115200);
	// m1.forward();
	// m1.setSpeed(42);
	// m1.backward();
	// m1.setSpeed(128);

	// m1.setSpeed(0);
	// m1.stop();



	pinMode(M1_ENCODER_A, INPUT);
	pinMode(M2_ENCODER_A, INPUT);
	attachInterrupt(digitalPinToInterrupt(M1_ENCODER_A), detect_m1_a, RISING);
	attachInterrupt(digitalPinToInterrupt(M2_ENCODER_A), detect_m2_a, RISING);

	Setpoint = 10;
	myPID.SetMode(AUTOMATIC);
	myPID.SetOutputLimits(-256, 255);

  // Esp-link
  pidInit();

  esp.resetCb = resetCb;
  resetCb();

  Serial.println("hello");
}

void detect_m1_a() {
	m1_dir = digitalRead(M1_ENCODER_B) ? 1 : -1; //read direction of motor
	m1_step+=m1_dir; //increasing encoder at new pulse
}

void detect_m2_a() {
	m2_dir = digitalRead(M2_ENCODER_B) ? 1 : -1; //read direction of motor
	m2_step+=m2_dir; //increasing encoder at new pulse
}

void loop() {
  esp.Process();

  pidLoop();


	m1_rps = (m1_step - m1_last_step) / 300.0 / sleep_time*1000;

	m1_last_step = m1_step;


	Input = m1_rps;
	myPID.Compute();
	Serial.print(Input);
	Serial.print("\t");
	Serial.print(Output);

	// Serial.print("\t");
	// Serial.print(Kp);
	// Serial.print("\t");
	// Serial.print(Ki);
	// Serial.print("\t");
	// Serial.print(Kd);

	Serial.print("\t");
	Serial.println(Setpoint);
	//Serial.printf("%d\t%d\n", m1_rps, Output);
	if (Output > 0) {
		m1.backward();
	} else {
		m1.forward();
	}
	m1.setSpeed(abs(Output));

	delay(sleep_time);

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
		if (mySt.substring(0,2) == "s=")
			Setpoint = mySt.substring(2,mySt.length()).toFloat();  //get string after set_speed
		else if (mySt.substring(0,2) == "p=")
			Kp = mySt.substring(2,mySt.length()).toFloat(); //get string after vs_kp
		else if (mySt.substring(0,2) == "i=")
			Ki = mySt.substring(2,mySt.length()).toFloat(); //get string after vs_ki
		else if (mySt.substring(0,2) == "d=")
			Kd = mySt.substring(2,mySt.length()).toFloat(); //get string after vs_kd
		myPID.SetTunings(Kp, Ki, Kd);
		// clear the string when COM receiving is completed
		mySt = "";  //note: in code below, mySt will not become blank, mySt is blank until '\n' is received
		stringComplete = false;
	}

}
