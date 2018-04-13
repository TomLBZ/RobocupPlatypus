/*
 Name:		Rescue2018.ino
 Created:	4/4/2018 5:56:41 PM
 Author:	Li Bozhao
*/

#include <DualVNH5019MotorShield.h>
#include <Wire.h>
//#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


#pragma region BlindSweepVarsAndConsts

//const int INTERRUPT_PIN = 2;  // use pin 2 on Arduino Uno & most boards
const int GyroADO = 13;
//const int SERVOPIN = 9;
const int Servo0 = A2;
const int Servo180 = A3;
const int THETA = 90;
const int VL = 255;
const int VR = 255;
const int TVL = 100;
const int TVR = 100;
const int SERVODOWN = 0;
const int SERVOUP = 120;
const int SERVORELEASE = 180;
const int LEFT = 1;
const int RIGHT = -1;
const int LONGSIDETIME = 4000;
const int SHORTSIDETIME = 3000;
const int LONGTESTTIME = 2000;
const int SHORTTESTTIME = 1000;
const int ALIGNINGTIME = 500;
const int DELTATIME = 10;
const int REVERSETIME = 200;
const int TURNINGTIME = 400;
const int SERVOTIME = 2000;
const int BRAKETIME = 80;
const int BRAKECYCLES = 5;
const int FAR = -100;
const int CLOSE = -10;

//Servo servo;
DualVNH5019MotorShield md;
MPU6050 mpu(0x69); // <-- use for AD0 high

uint8_t mpuIntStatus;   // holds actual  status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
bool IsBlindSweeping = true;
int ZONELOCATION = 0;
#pragma endregion

#pragma region BlindSweepFuncs
void Blindsweep() 
{
  Serial.print("checkingzone");
	CheckZone();
  Serial.print("doneCheckingZone");
	ReturnHome(ZONELOCATION);
	while (IsBlindSweeping)
	{
		int direction;
		switch (ZONELOCATION)
		{
		case 0:
			direction = RIGHT;
			break;
		case 1:
			direction = LEFT;
			break;
		default://case 2
			direction = LEFT;
			break;
		}
		SweepHalfCycle(direction);
		direction = -direction;
	}
	Deposit();
}

void Forward(int vL, int vR, int duration, bool IsBraking)
{
	md.setSpeeds(vR, vL);
	delay(duration);
	if (IsBraking)
	{
		//md.setBrakes(255, 255);
		SETBRAKES(md, vL, vR, BRAKECYCLES);
	}
}

void Turn(float theta, int direction)
{
	float Criterion = GetAngle();
	while (!IsTurned(theta, Criterion))
	{
		if (direction == LEFT)
		{
			Forward(-TVL, TVR, DELTATIME, false);
		}
		else
		{
			Forward(TVL, -TVR, DELTATIME, false);
		}
	}
	Forward(0, 0, 0, true);
}

float GetAngle()
{
	mpu.resetFIFO();
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();
	if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
		// display Euler angles in degrees
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		//Serial.print("turn:\t");
		Serial.println(ypr[0] * 180/M_PI);
		return (ypr[0] * 180 / M_PI);
	}
}

//void dmpDataReady() 
//{
	//mpuInterrupt = true;
//}

bool IsTurned(int theta, float criterion)
{
	return abs(GetAngle() - criterion) >= theta;
}

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

void CheckZone()//0 straight, 1 left, 2 diagonal
{
	ServoTurn(SERVODOWN, SERVOTIME);
	Forward(VL, VR, SHORTTESTTIME, true);
	ServoTurn(SERVOUP, SERVOTIME);
	if (IsZone(FAR))
	{
		ZONELOCATION = 0;
	}
	else
	{
		ServoTurn(SERVODOWN, SERVOTIME);
		Forward(-VL, -VR, SHORTTESTTIME, true);
		ServoTurn(SERVOUP, SERVOTIME);
		Turn(THETA, LEFT);
		ServoTurn(SERVODOWN, SERVOTIME);
		Forward(VL, VR, LONGTESTTIME, true);
		ServoTurn(SERVOUP, SERVOTIME);
		if (IsZone(FAR))
		{
			ZONELOCATION = 1;
		}
		else
		{
			ZONELOCATION = 2;
		}
	}
	ServoTurn(SERVODOWN, SERVOTIME);
}

bool IsZone(int range)
{
	if (range == CLOSE)
	{
		Serial.println('C');
	}
	else
	{
		Serial.println('F');
	}
	while (!Serial.available())
	{
	}
	char chr = Serial.read();
	return chr == '1' ? true : false;
}

void ReturnHome(int zonelocation)
{
	switch (zonelocation)
	{
	case 0:
		Forward(-VL, -VR, SHORTTESTTIME, true);
		ServoTurn(SERVOUP, SERVOTIME);
		Turn(THETA, LEFT);
		ServoTurn(SERVODOWN, SERVOTIME);
		Forward(-VL, -VR, ALIGNINGTIME, false);//aligning
		Forward(VL, VR, LONGSIDETIME, true);
		Forward(-VL, -VR, REVERSETIME, true);
		ServoTurn(SERVOUP, SERVOTIME);
		Forward(VL, VR, TURNINGTIME, true);
		Turn(THETA, RIGHT);
		ServoTurn(SERVODOWN, SERVOTIME);
		Forward(-VL, -VR, ALIGNINGTIME, false);//aligning
		break;
	case 1:
		Forward(-VL, -VR, LONGTESTTIME, true);
		ServoTurn(SERVOUP, SERVOTIME);
		Turn(THETA, RIGHT);
		ServoTurn(SERVODOWN, SERVOTIME);
		break;
	default://case 2
		Forward(-VL, -VR, LONGTESTTIME, true);
		ServoTurn(SERVOUP, SERVOTIME);
		Turn(THETA, RIGHT);
		ServoTurn(SERVODOWN, SERVOTIME);
		break;
	}
}

void SweepHalfCycle(int direction)
{
	Forward(VL, VR, SHORTSIDETIME, true);
	Forward(-VL, -VR, REVERSETIME, true);
	ServoTurn(SERVOUP, SERVOTIME);
	Forward(VL, VR, TURNINGTIME, true);
	Turn(THETA, direction);
	IsBlindSweeping = !IsZone(CLOSE);
	ServoTurn(SERVODOWN, SERVOTIME);
	Forward(VL, VR, TURNINGTIME, true);
	ServoTurn(SERVOUP, SERVOTIME);
	Turn(THETA, direction);
	Forward(-VL, -VR, ALIGNINGTIME, false);//aligning
}

void Deposit()
{
	ServoTurn(SERVOUP, SERVOTIME);
	Turn(2 * THETA, LEFT);
	Forward(-TVL, -TVR, SHORTTESTTIME, false);
	ServoTurn(SERVORELEASE, SERVOTIME);
}

void GyroSetUp()
{
	Wire.begin();
	// put your setup code here, to run once:
	// join I2C bus (I2Cdev library doesn't do this automatically)
	pinMode(GyroADO, OUTPUT);
	digitalWrite(GyroADO, HIGH);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	//pinMode(INTERRUPT_PIN, INPUT);
	// verify connection
	//Serial.println(F("Testing device connections..."));
	//Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	// load and configure the DMP
	//Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(-21); //220
	mpu.setYGyroOffset(95); //76
	mpu.setZGyroOffset(28); //-85
	mpu.setZAccelOffset(1519); // 1788/1688 factory default for my test chip
							   // make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		//Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);
		// enable Arduino interrupt detection
		//Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		//attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		//attachInterrupt(digitalPinToInterrupt(3), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;
		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

void SETBRAKES(DualVNH5019MotorShield motor, int LBrake, int RBrake, int brakeCycles)
{
	int iL = LBrake / 2;
	if (iL < 75) { iL = 75; }
	int diL = iL / brakeCycles;
	int iR = RBrake / 2;
	if (iR < 75) { iR = 75; }
	int diR = iR / brakeCycles;
	int t = BRAKETIME;
	int dt = BRAKETIME / brakeCycles;
	int sign = -1;
	for (int i = 0; i < brakeCycles; i++)
	{
		motor.setSpeeds(-iL * pow(sign, i), -iR * pow(sign, i));
		iL -= diL;
		iR -= diR;
		delay(t);
		t -= dt;
	}
	motor.setSpeeds(1, 1);
	delay(dt);
}

#pragma endregion

// the setup function runs once when you press reset or power the board
void setup() {
	//servo.attach(SERVOPIN);
	Serial.begin(115200);
	bool IsStarting = false;
	while (!IsStarting)
	{
		while (!Serial.available())
		{
		}
		if (Serial.read() == 'S')
		{
			IsStarting = true;
		}
	}
	GyroSetUp();
	md.init();
  pinMode(Servo0,OUTPUT);
  pinMode(Servo180,OUTPUT);
  digitalWrite(Servo0,LOW);
  digitalWrite(Servo180,LOW);
}

// the loop function runs over and over again until power down or reset
void loop() {
	Serial.print("Entering Loop");
	Blindsweep();
 //Turn(90,1);
 //delay(1000);

}
