#include <Servo.h>
#include <Wire.h>

Servo servo;

const int Servopin = 3;
const int Servo0 = 4;
const int Servo180 = 5;

const int downAngle = 0;
const int depositAngle = 180;
const int holdAngle = 120;

void setup() {
  Serial.begin(115200);
  servo.attach(3);
  pinMode(Servo0, INPUT);
  pinMode(Servo180, INPUT);
}

void loop() {
  if(digitalRead(Servo0) == HIGH && digitalRead(Servo180) == LOW){
    servo.write(downAngle);
    delay(1000);
    }
  if(digitalRead(Servo0) == LOW && digitalRead(Servo180) == HIGH){
    servo.write(depositAngle);
    delay(1000);
    }
  if(digitalRead(Servo0) == LOW && digitalRead(Servo180) == LOW){
    servo.write(holdAngle);
    delay(1000);
    }
}
