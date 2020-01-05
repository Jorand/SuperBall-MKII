const int enA = 5;
const int in1 = 4;
const int in2 = 7;

int motor_min = 70;

int motor_value = 255;
unsigned int integerValue=0;  // Max value is 65535
char incomingByte;

void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  Serial.println("Hello");
}

String my_name;

void loop() {

  if (Serial.available()) {
      my_name = Serial.readStringUntil('\n');
      Serial.println("> " + my_name);
      if (my_name.toInt() > 0) {
        integerValue = my_name.toInt();
      }
      Serial.println(integerValue);
  }
  
  motor_value = integerValue;
  if(motor_value > 0){
    MotorCounterClockwise(motor_value);
  }else{
    MotorClockwise(abs(motor_value));
  }
  //Serial.println(motor_value);
  delay(10);
}

void MotorClockwise(int power){
  if(power > motor_min){
    analogWrite(enA, power);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }else{
    digitalWrite(enA, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void MotorCounterClockwise(int power){
  if(power > motor_min){
    analogWrite(enA, power);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }else{
    digitalWrite(enA, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}
