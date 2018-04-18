/*
 Name:		Rescue2018.ino
 Created:	4/4/2018 5:56:41 PM
 Author:	Li Bozhao
*/

#include <DualVNH5019MotorShield.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#pragma region BlindSweepVarsAndConsts
const int GyroADO = 13;
const int SERVO0 = 5;
const int SERVO180 = 11;
const int THETA = 90;
const int VL = 240;
const int VR = 240;
const int TVL = 110;
const int TVR = 110;
const int VBRAKEMIN = 75;
const int SERVODOWN = 0;
const int SERVOUP = 120;
const int SERVORELEASE = 180;
const int LEFT = 1;
const int RIGHT = -1;
const int TURNINGBIAS = 1;
const int LONGSIDETIME = 3000;
const int SHORTSIDETIME = 2000;
const int LONGTESTTIME = 2000;
const int SHORTTESTTIME = 1000;
const int ALIGNINGTIME = 500;
const int DELTATIME = 10;
const int REVERSETIME = 100;
const int TURNINGTIME = 400;
const int SERVOTIME = 1800;
const int BRAKETIME = 80;
const int BRAKECYCLES = 5;
const int FAR = -100;
const int CLOSE = -10;

DualVNH5019MotorShield md;
MPU6050 mpu(0x69); // <-- use for AD0 high

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
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
	CheckZone();
	ReturnHome(ZONELOCATION);
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
	while (IsBlindSweeping)
	{
		SweepHalfCycle(direction);
		direction = -direction;
	}
	Deposit();
}

void Forward(int vL, int vR, int duration, bool IsBraking, bool IsTurning = false)
{
	//Gryo control: based on current theta. 
	//Larger than Criterion -> adjust to the left.
	//Smaller than Criterion -> adjust to the right.
	if (IsTurning)
	{
		md.setSpeeds(vR, vL);//m1 is right side; m2 is left side.
		delay(duration);
	}
	else 
	{
		float Criterion = GetAngle();
		md.setSpeeds(vR, vL);//m1 is right side; m2 is left side.
		long prevT = millis();
		while (millis() - prevT < duration)
		{
			if (GetAngle() - Criterion > 0)//increase V right, decrease V left
			{
				md.setSpeeds(vR + TVR / 4, vL - TVL / 4); //difference equals to TVL / 2 or TVR / 2
			}
			else 
			{
				md.setSpeeds(vR - TVR / 4, vL + TVL / 4);
			}
			//delay(DELTATIME);
		}
	}
	if (IsBraking)
	{
		md.setBrakes(vR, vL);
		//SETBRAKES(md, vL, vR, BRAKECYCLES);
	}
}

void Turn(float theta, int direction)
{
	float Criterion = GetAngle();
	while (!IsTurned(theta, Criterion, direction))
	{
		if (direction == LEFT)
		{
			Forward(-TVL, TVR, DELTATIME, false, true);
		}
		else
		{
			Forward(TVL, -TVR, DELTATIME, false, true);
		}
	}
	Forward(0, 0, 0, true);
}

float GetAngle()
{
	mpuInterrupt = false;
	mpu.resetFIFO();
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();
	if (mpuIntStatus & 0x02) 
	{
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
		//Serial.println(ypr[0] * 180/M_PI);
		return (ypr[0] * 180 / M_PI);
	}
	else 
	{
		Serial.println(mpuIntStatus);
	}
}

bool IsTurned(int theta, float current, int direction)
{
	float target;	//gryo range is (L, R) <- [-180.0, 180.0]
	float angle = GetAngle();
	if (direction == LEFT)
	{
		target = current - theta;
		if (target >= -180.0)			//normal case
		{
			return !(angle >= target + TURNINGBIAS & angle <= current + TURNINGBIAS); 
			// return true if not in turning range, having a bias to prevent backshaking and overturning.
		}
		else							//target causes discontinuity
		{
			target = 360.0 + target;		//shift lower bound up
			return !((angle >= -180.0 & angle <= current + TURNINGBIAS) | (angle >= target + TURNINGBIAS & angle <= 180.0));
		}
	}
	else if (direction == RIGHT)
	{
		target = current + theta;
		if (target <= 180.0)			//normal case
		{
			return !(angle <= target - TURNINGBIAS & angle >= current - TURNINGBIAS);
			// return true if not in turning range, having a bias to prevent backshaking and overturning.
		}
		else							//target causes discontinuity
		{
			target = target - 360.0;		//shift lower bound up
			return !((angle <= 180.0 & angle >= current - TURNINGBIAS) | (angle <= target - TURNINGBIAS & angle >= -180.0));
		}
	}
	else
	{
		return true;
	}
}

void ServoTurn(int degree, int duration)
{
	if (degree == SERVODOWN) {
		digitalWrite(SERVO0, HIGH);
		digitalWrite(SERVO180, LOW);
	}

	if (degree == SERVOUP) {
		digitalWrite(SERVO0, LOW);
		digitalWrite(SERVO180, LOW);
	}

	if (degree == SERVORELEASE) {
		digitalWrite(SERVO0, LOW);
		digitalWrite(SERVO180, HIGH);
	}
	delay(duration);
}
void CheckZone()//0 straight, 1 left, 2 diagonal
{
	ServoTurn(SERVODOWN, SERVOTIME);
	Forward(VL, VR, SHORTTESTTIME, true);
	Forward(-VL, -VR, REVERSETIME, true);
	ServoTurn(SERVOUP, SERVOTIME);
	if (IsZone(FAR))
	{
		ZONELOCATION = 0;
	}
	else
	{
		ServoTurn(SERVODOWN, SERVOTIME);
		Forward(-VL, -VR, SHORTTESTTIME - REVERSETIME, true);
		ServoTurn(SERVOUP, SERVOTIME);
		Turn(THETA, LEFT);
		ServoTurn(SERVODOWN, SERVOTIME);
		Forward(VL, VR, LONGTESTTIME, true);
		Forward(-VL, -VR, REVERSETIME, true);
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
	return chr == '1';
}

void ReturnHome(int zonelocation)
{
	switch (zonelocation)
	{
	case 0:
		Forward(-VL, -VR, SHORTTESTTIME, true);	//backward to gate
		ServoTurn(SERVOUP, SERVOTIME);			//servo up
		Turn(THETA, LEFT);						//turn left by 90
		ServoTurn(SERVODOWN, SERVOTIME);		//servo down
		Forward(-VL, -VR, ALIGNINGTIME, false);	//aligning with right wall
		Forward(VL, VR, LONGSIDETIME, true);	//go towards left wall
		Forward(-VL, -VR, REVERSETIME, true);	//reverse to have clearance
		ServoTurn(SERVOUP, SERVOTIME);			//servo up
		Forward(VL, VR, TURNINGTIME, true);		//forward to clear machanism
		Forward(-VL, -VR, REVERSETIME, true);	//reverse to have clearance
		Turn(THETA, RIGHT);						//turn right by 90
		ServoTurn(SERVODOWN, SERVOTIME);		//servo down
		Forward(-VL, -VR, ALIGNINGTIME, false);	//aligning with back wall
		break;									//now in ready position
	case 1:
		Forward(-VL, -VR, LONGTESTTIME, true);	//backward to gate
		ServoTurn(SERVOUP, SERVOTIME);			//servo up
		Turn(THETA, RIGHT);						//turn right by 90
		ServoTurn(SERVODOWN, SERVOTIME);		//servo up
		break;									//now in ready position
	default://case 2:
		Forward(-VL, -VR, LONGTESTTIME, true);	//backward to gate
		ServoTurn(SERVOUP, SERVOTIME);			//servo up
		Turn(THETA, RIGHT);						//turn right by 90
		ServoTurn(SERVODOWN, SERVOTIME);		//servo up
		break;									//now in ready position
	}
}

void SweepHalfCycle(int direction)
{
	Forward(VL, VR, SHORTSIDETIME, true);	//forward a short side distance
	Forward(-VL, -VR, REVERSETIME, true);	//reverse to have clearance
	ServoTurn(SERVOUP, SERVOTIME);			//servo up
	Forward(VL, VR, TURNINGTIME, true);		//forward to clear mechanism
	Turn(THETA, direction);					//turn 90 to direction
	IsBlindSweeping = !IsZone(CLOSE);		//check for evacuation zone
	if (IsBlindSweeping)
	{
		ServoTurn(SERVODOWN, SERVOTIME);		//servo down
		Forward(VL, VR, TURNINGTIME, true);		//forward to clear track
		ServoTurn(SERVOUP, SERVOTIME);			//servo up
		Turn(THETA, direction);					//turn 90 to direction
		Forward(-VL, -VR, ALIGNINGTIME, false);	//aligning with a wall
	}										//now ready for next half-cycle
}

void Deposit()
{
	ServoTurn(SERVOUP, SERVOTIME);				//servo up
	Turn(2 * THETA, LEFT);						//turn 180 to the left
	Forward(-VL, -VR, SHORTTESTTIME, false);	//back until aligned with zone
	ServoTurn(SERVORELEASE, SERVOTIME);			//release balls
}

void GyroSetUp()
{
	Wire.begin();
	// join I2C bus (I2Cdev library doesn't do this automatically)
	pinMode(GyroADO, OUTPUT);
	digitalWrite(GyroADO, HIGH);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	//Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	// verify connection
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
	if (iL < VBRAKEMIN) { iL = VBRAKEMIN; }
	int diL = iL / brakeCycles;
	int iR = RBrake / 2;
	if (iR < VBRAKEMIN) { iR = VBRAKEMIN; }
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
	Serial.begin(115200);
	bool IsStarting = false;
	GyroSetUp();
	md.init();
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
	pinMode(SERVO0, OUTPUT);
	pinMode(SERVO180, OUTPUT);
	digitalWrite(SERVO0, LOW);
	digitalWrite(SERVO180, LOW);
}

// the loop function runs over and over again until power down or reset
void loop() {
	Serial.print("Entering Loop");
	ServoTurn(SERVORELEASE, SERVOTIME);
	Forward(VL, VR, SHORTTESTTIME, true, true);
	Blindsweep();
	while (true);
}