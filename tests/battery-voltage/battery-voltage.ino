const int analogInput = A2;
float vout = 0.0;
float vin = 0.0;
float R1 = 99800.0; // resistance of R1 (100K)
float R2 = 9950.0; // resistance of R2 (10K)
const float refVoltage = 5.0; //
int value = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Hello");

  pinMode(analogInput, INPUT);
}

void loop() {
  value = analogRead(analogInput);
  vout = (value * refVoltage) / 1024.0;
  vin = vout / (R2/(R1+R2)); 
  if (vin < 0.09) {
    vin = 0.0;
  }
  Serial.print("INPUT V= ");
  Serial.println(vin);
  delay(500);
}
