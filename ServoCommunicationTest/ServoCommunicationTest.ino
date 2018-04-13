const int Servo0 = 3;
const int Servo180 = 5;
const int SERVODOWN = 0;
const int SERVOUP = 150;
const int SERVORELEASE = 180;


void ServoTurn(int degree, int duration)
{
    Serial.print("startingServo");  
    //servo.write(degree);
    if (degree == SERVODOWN){
      digitalWrite(Servo0,HIGH);
      digitalWrite(Servo180,LOW);
      }
      
    if (degree == SERVOUP){
      digitalWrite(Servo0,LOW);
      digitalWrite(Servo180,LOW);
      }
      
    if (degree == SERVORELEASE){
      digitalWrite(Servo0,LOW);
      digitalWrite(Servo180,HIGH);
      }
    Serial.print("ServoSet");
    delay(duration);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(Servo0,OUTPUT);
  pinMode(Servo180,OUTPUT);
  digitalWrite(Servo0,LOW);
  digitalWrite(Servo180,LOW);

}

void loop() {
  // put your main code here, to run repeatedly:
  ServoTurn(SERVODOWN,1800);
  delay(3000);
  ServoTurn(SERVOUP,1800);
    delay(3000);
  ServoTurn(SERVORELEASE,1800);
    delay(3000);


  
}
