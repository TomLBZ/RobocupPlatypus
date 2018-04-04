/*
 Name:		Robocup2018.ino
 Created:	10/17/2017 11:35:59 AM
 Author:	LBZ
 Hi, This is me.
*/
#include <Servo.h>
#include <Adafruit_VL53L0X.h>
#include <DualVNH5019MotorShield.h>
#pragma region defs
//digital output
#define pLED		13
//digital input
//I2C portals
#define pSDA		20
#define pSCL		21
//analog output
#define pSRV		A15
//analog input
//operational constants
#define turnLEFT	-100
#define turnRIGHT	-101
#define GAINONLY	-110
#define BALANCED	-111
#define THRESHOLD	-112
#define SYMMETRIC	-113
#define servoMIN	0
#define servoDELAY	15
#define SECTIONS	30
#define servoMAX	70
#define obstRANGE	75
#define baseSPEED	100
#define blinkFAST	250
#define FULLSPDBRK	400
#define blinkMEDIUM	500
#define blinkSLOW	1000
#define delayTEST	1500
//operational variables
bool ISTRACKING = true;
bool ISRESCUING = false;
const byte numChars = 16;
char receivedChars[numChars]; // an array to store the received data
boolean newData = false;
//operational objects
Servo myservo;
DualVNH5019MotorShield md;
Adafruit_VL53L0X laser = Adafruit_VL53L0X();
#pragma endregion
#pragma region level1Funcs: Programme Entrance
void ini() 
{
	//digital output
	pinMode(pLED, OUTPUT);	
	//digital input
	Serial.begin(115200);
	//I2C portals
	pinMode(pSDA, INPUT);
	pinMode(pSCL, INPUT);
	//analog output
	pinMode(pSRV, OUTPUT);
	//analog input
	//operational objects initiation
	myservo.attach(pSRV);
	md.init();
	if (!laser.begin())
	{
		blink(pLED, blinkFAST);
	}
	//servoUp();
}
#pragma endregion
#pragma region level2Track: Track Structure
void drive(String input)
{
	String message = input.substring(0, 0);
	int msg = message.toInt();
	switch (msg)
	{
	case 2://Point Turn, using BALANCED
		md.setSpeeds(regulateSpeed(getLGain(getBias(input, SECTIONS), BALANCED, SECTIONS), FULLSPDBRK, baseSPEED)
			, regulateSpeed(getRGain(getBias(input, SECTIONS), BALANCED, SECTIONS), FULLSPDBRK, baseSPEED));
		break;
	case 3://HALT
		md.setBrakes(FULLSPDBRK, FULLSPDBRK);
		break;
	case 4://UNIFORM REVERSE
		md.setSpeeds(-baseSPEED, -baseSPEED);
		break;
	case 5://OBSTACLE
		md.setBrakes(FULLSPDBRK, FULLSPDBRK);
		if (isObstacle())
		{
			avoidObstacle();
		}
		else
		{
			md.setSpeeds(regulateSpeed(getLGain(getBias(input, SECTIONS), GAINONLY, SECTIONS), FULLSPDBRK, baseSPEED)
				, regulateSpeed(getRGain(getBias(input, SECTIONS), GAINONLY, SECTIONS), FULLSPDBRK, baseSPEED));
		}
		break;
	case 6://SWITCH THE TASKS
		if (ISTRACKING==true)
		{
			ISTRACKING = false;
			ISRESCUING = true;
		}
		else
		{
			ISTRACKING == true;
			ISRESCUING == false;
		}
		break;
	case 7://R GREEN

		break;
	case 8://L GREEN

		break;
	case 9://rescue zone*

		break;
	default://0 or 1, NORMAL TRACKING
		md.setSpeeds(regulateSpeed(getLGain(getBias(input, SECTIONS), GAINONLY, SECTIONS), FULLSPDBRK, baseSPEED)
			, regulateSpeed(getRGain(getBias(input, SECTIONS), GAINONLY, SECTIONS), FULLSPDBRK, baseSPEED));
		break;
	}
	stopIfFault();
}
void avoidObstacle()
{

}
#pragma endregion
#pragma region level2Rescue: Rescue Structure
void rescue(String input) 
{
	int msg = input.substring(0, 0).toInt();
	double bias = getBias(input, SECTIONS);
	switch (msg)
	{
	case 8://sees blank space
		if (isObstacle())//point turn at a wall
		{
			md.setSpeeds(baseSPEED, -baseSPEED);
		}
		else//go forward blindly
		{
			md.setSpeeds(baseSPEED, baseSPEED);
		}
		break;
	case 9://Sees Rescue Zone: Rectangle
		if (abs(bias)<SECTIONS/3)//facing centre of rescue zone,track with GAINONLY
		{
			md.setSpeeds(regulateSpeed(getLGain(bias, GAINONLY, SECTIONS), FULLSPDBRK, baseSPEED),
				regulateSpeed(getRGain(bias, GAINONLY, SECTIONS), FULLSPDBRK, baseSPEED));
		}
		else//ball not in capture range, point turn with SYMMETRIC
		{
			md.setSpeeds(getLGain(bias, SYMMETRIC, SECTIONS), getRGain(bias, SYMMETRIC, SECTIONS));
		}
		if (isObstacle())//reaches the zone
		{
			depositBALL();
		}
		break;
	default://0 or 1
		if (abs(bias)<SECTIONS/3)//ball in capture range, track with GAINONLY
		{
			md.setSpeeds(regulateSpeed(getLGain(bias, GAINONLY, SECTIONS), FULLSPDBRK, baseSPEED),
				regulateSpeed(getRGain(bias, GAINONLY, SECTIONS), FULLSPDBRK, baseSPEED));
		}
		else//ball not in capture range, point turn with SYMMETRIC
		{
			md.setSpeeds(getLGain(bias, SYMMETRIC, SECTIONS), getRGain(bias, SYMMETRIC, SECTIONS));
		}
		break;
	}
}
void depositBALL()
{
	servoDown(servoMIN, servoMAX, servoDELAY);
}
#pragma endregion
#pragma region level3Funcs: Basic Structures
void blink(int LED, int delaytime) 
{
	pinMode(LED, OUTPUT);
	while (true) {
		digitalWrite(LED, HIGH);
		delay(delaytime);
		digitalWrite(LED, LOW);
		delay(delaytime);
	}
}
void stopIfFault()
{
	if (md.getM1Fault())
	{
		Serial.println("M1 fault");
		blink(pLED,blinkFAST);
	}
	if (md.getM2Fault())
	{
		Serial.println("M2 fault");
		blink(pLED,blinkFAST);
	}
}
int regulateSpeed(double Vin, int Vmax, int Vbase)
{
	if (Vin+Vbase>Vmax)
	{
		return Vmax;
	}
	else
	{
		if (Vin+Vbase<-Vmax)
		{
			return -Vmax;
		}
		else
		{
			return (int)Vin + Vbase;
		}
	}
}
bool isObstacle()
{
	VL53L0X_RangingMeasurementData_t measure;
	laser.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
	int aveDist = measure.RangeMilliMeter;
	for (size_t i = 0; i < 5; i++)
	{
		aveDist = (int)(aveDist + measure.RangeMilliMeter) / 2;
		delay(50);
	}
	if ((measure.RangeStatus != 4)&(aveDist<obstRANGE)) //no failure and in range
	{  // phase failures have incorrect data
		return true;
	}
	else
	{
		return false;
	}
}
void brake(int M1Brake, int M2Brake)
{
	md.setBrakes(M1Brake, M2Brake);
}
void servoDown(int minAngle, int maxAngle, int Tdelay)
{
	for (int pos = minAngle; pos <= maxAngle; pos ++)
	{ // goes from servoMIN to servoMAX in degrees
		myservo.write(pos);
		delay(Tdelay);
	}
}
void servoUp(int minAngle, int maxAngle, int Tdelay)
{
	for (int pos = maxAngle; pos >= minAngle; pos--)
	{ // goes from servoMAX to servoMIN in degrees
		myservo.write(pos);
		delay(Tdelay);
	}
}
#pragma endregion
#pragma region level4Funcs: Data Processing
String getINPUT(bool isChr = false)
{
	if (isChr)
	{
		recvWithEndMarker();
		showNewData();
		return String(receivedChars);
	}
	else
	{
		return "";
	}
}
int getFLAG(String data)
{
	
}
int getPOS(String data)
{

}
void recvWithEndMarker() {
	static byte ndx = 0;
	char endMarker = '\n';
	char rc;
	while (Serial.available() > 0 && newData == false) {
		rc = Serial.read();
		if (rc != endMarker) {
			receivedChars[ndx] = rc;
			ndx++;
			if (ndx >= numChars) {
				ndx = numChars - 1;
			}
		}
		else {
			receivedChars[ndx] = '\0'; // terminate the string
			ndx = 0;
			newData = true;
		}
	}
}
void showNewData() {
	if (newData == true) {
		Serial.print("This just in ... ");
		Serial.println(receivedChars);
		newData = false;
	}
}
double getBias(String data, int sections)
{
	double locLine = data.indexOf("1");
	if (locLine<0)
	{
		locLine = 0;
	}
	locLine -= sections / 2;
	if (sections / 2 == (int)(sections / 2))//EVEN sections
	{
		if (locLine >= 0) 
		{ 
			locLine += 1; //00000001>>4;10000000>>-4;no zero-point
		}
	}
	else//ODD sections
	{
		locLine += 0.5;//0000001>>3;1000000>>-3;0001000>>0
	}
	return locLine;
}
double getLGain(double bias,int mode, int sections)
{
	double mapX;
	if (sections / 2 == (int)(sections / 2))//EVEN sections
	{
		mapX = sections / 2;
	}
	else//ODD sections
	{
		mapX = sections / 2 - 0.5;//1000000>>bias=-3>>LGain~3
	}
	switch (mode)
	{
	case GAINONLY:
		if (bias<0)//NO NEGATIVE GAIN
		{
			return 0.0;
		}//else y against mapX
		else//gain~temp
		{
			return bias * (sections - bias);//POSITIVE,DECREASING RATE
		}
		break;
	case THRESHOLD:
		double temp;
		temp = bias * (sections - bias);
		if (temp<-baseSPEED)
		{
			temp = -baseSPEED;
		}
		return temp;
		break;
	case SYMMETRIC:
		return baseSPEED / mapX * bias;
		break;
	default://BALANCED
		return bias * (sections - bias);
		break;
	}
}
double getRGain(double bias, int mode, int sections)
{
	double mapX;
	double temp = -bias;//make negative bias positive base for gain
	if (sections / 2 == (int)(sections / 2))//EVEN sections
	{
		mapX = sections / 2;
	}
	else//ODD sections
	{
		mapX = sections / 2 - 0.5;//0000001>>bias=3>>RGain~3
	}
	switch (mode)
	{
	case GAINONLY:
		if (temp<0)//NO NEGATIVE GAIN
		{
			return 0.0;
		}//else y against mapX
		else//gain~temp
		{
			return temp * (sections - temp);//POSITIVE,DECREASING RATE
		}
		break;
	case THRESHOLD:
		double tmp;
		tmp = temp * (sections - temp);
		if (tmp<-baseSPEED)
		{
			tmp = -baseSPEED;
		}
		return tmp;
		break;
	case SYMMETRIC:
		return baseSPEED / mapX * temp;
		break;
	default://BALANCED
		return temp * (sections - temp);
		break;
	}
}
#pragma endregion
#pragma region OBSOLETE_FUNCS
void clearArray(int in[], int size)
{
		for (size_t i = 0; i < size; i++)
		{
			in[i] = 0;
		}
		return;
}
double moderateSumArray(double in[], int size)
{
	return (sumArray(in, size) - maxArray(in, size) - minArray(in, size));
}
double moderateAveArray(double in[], int size)
{
	return (moderateSumArray(in, size) / (size - 2));
}
double maxArray(double in[], int size)
{
	double huge = 0;
	for (int i = 0; i < size; i++)
	{
		if (huge<in[i])
		{
			huge = in[i];
		}
	}
	return huge;
}
double minArray(double in[], int size)
{
	double tiny = 10000;
	for (int i = 0; i < size; i++)
	{
		if (tiny>in[i])
		{
			tiny = in[i];
		}
	}
	return tiny;
}
double sumArray(double in[], int size)
{
	double sum = 0;
	for (int i = 0; i < size; i++)
	{
		sum += in[i];
	}
	return sum;
}
double gaussianArray(double in[], int size) 
{
	double result = 0;
	switch (size)
	{
	case 3:
		result = (in[0] + in[1] * 2 + in[2]) / 4;
		break;
	case 5:
		result = (in[0] + in[1] * 4 + in[2] * 7 + in[3] * 4 + in[4]) / 17;
		break;
	default:
		break;
	}
	return result;
}
double averageArray(double in[], int size)
{
	return sumArray(in,size) / size;
}
double midGaussian(double in[5])
{
	double huge, tiny;
	huge = 0;
	tiny = 100000;
	for (int i = 0; i < 5; i++)
	{
		if (in[i]>huge)
		{
			huge = in[i];
		}
		if (true)
		{

		}
	}
}
void readSeries(int pinInput, int outArray[], int arraySize, bool ifDigital = false)
{
		if (ifDigital)
		{
			for (size_t i = 0; i < arraySize; i++)
			{
				outArray[i] = digitalRead(pinInput);
			}
		}
		else
		{
			for (size_t i = 0; i < arraySize; i++)
			{
				outArray[i] = analogRead(pinInput);
			}
		}
		return;
}
void readMultiple(int pinsInput[], int outArray[], int size, bool ifDigital = false)
{
		if (ifDigital)
		{
			for (size_t i = 0; i < size; i++)
			{
				outArray[i] = digitalRead(pinsInput[i]);
			}
		}
		else
		{
			for (size_t i = 0; i < size; i++)
			{
				outArray[i] = analogRead(pinsInput[i]);
			}
		}
		return;
}
void turn(int direction, int basespeed, int speeddifference)
{
	if (direction == turnLEFT)
	{
		//analogWrite(pL, (int)(basespeed - speeddifference / 2));
		//analogWrite(pR, (int)(basespeed + speeddifference / 2));
		return;
	}
	else if (direction == turnRIGHT)
	{
		//analogWrite(pR, (int)(basespeed - speeddifference / 2));
		//analogWrite(pL, (int)(basespeed + speeddifference / 2));
		return;
	}
	else
	{
		return;
	}
}
void straight(int basespeed, bool ifforward = true) 
{
	if (!ifforward)
	{
		basespeed = -basespeed;
		//analogWrite(pR, basespeed);
		//analogWrite(pL, basespeed);
		basespeed = -basespeed;
	}
	else 
	{
		//analogWrite(pR, basespeed);
		//analogWrite(pL, basespeed);
	}
	return;
}
#pragma endregion
#pragma region TEST_FUNCS
void motorTEST()
{
	while (true)
	{
		for (int i = 0; i <= 400; i++)
		{
			md.setM1Speed(i);
			stopIfFault();
			if (i % 200 == 100)
			{
				Serial.print("M1 current: ");
				Serial.println(md.getM1CurrentMilliamps());
			}
			delay(2);
		}

		for (int i = 400; i >= -400; i--)
		{
			md.setM1Speed(i);
			stopIfFault();
			if (i % 200 == 100)
			{
				Serial.print("M1 current: ");
				Serial.println(md.getM1CurrentMilliamps());
			}
			delay(2);
		}

		for (int i = -400; i <= 0; i++)
		{
			md.setM1Speed(i);
			stopIfFault();
			if (i % 200 == 100)
			{
				Serial.print("M1 current: ");
				Serial.println(md.getM1CurrentMilliamps());
			}
			delay(2);
		}

		for (int i = 0; i <= 400; i++)
		{
			md.setM2Speed(i);
			stopIfFault();
			if (i % 200 == 100)
			{
				Serial.print("M2 current: ");
				Serial.println(md.getM2CurrentMilliamps());
			}
			delay(2);
		}

		for (int i = 400; i >= -400; i--)
		{
			md.setM2Speed(i);
			stopIfFault();
			if (i % 200 == 100)
			{
				Serial.print("M2 current: ");
				Serial.println(md.getM2CurrentMilliamps());
			}
			delay(2);
		}

		for (int i = -400; i <= 0; i++)
		{
			md.setM2Speed(i);
			stopIfFault();
			if (i % 200 == 100)
			{
				Serial.print("M2 current: ");
				Serial.println(md.getM2CurrentMilliamps());
			}
			delay(2);
		}
	}
}
void motorNEWtest(DualVNH5019MotorShield motor, int Vmax, int Tdelay)
{
	motor.setSpeeds(Vmax, Vmax);	//Forward Test
	delay(Tdelay);
	motor.setSpeeds(-Vmax, -Vmax);	//Backward Test
	delay(Tdelay);
	motor.setSpeeds(Vmax, -Vmax);	//Right Turn Test
	delay(Tdelay);
	motor.setSpeeds(-Vmax, Vmax);	//Left Turn Test
	delay(Tdelay);
	motor.setSpeeds(Vmax, Vmax);	//Forward
	delay(Tdelay);
	motor.setBrakes(Vmax, Vmax);	//Full Brake Test
	delay(Tdelay);
	motor.setSpeeds(Vmax, Vmax);	//Forward
	delay(Tdelay);
	motor.setSpeeds(0, 0);			//Natural Brake Test
	delay(Tdelay);
}
void servoTEST()
{
	int pos = 0;
	for (int i=0;i<5;i++)
	{
		for (pos = 0; pos <= 180; pos += 1) 
		{ // goes from 0 degrees to 180 degrees
			myservo.write(pos);              // tell servo to go to position in variable 'pos'
			delay(15);                       // waits 15ms for the servo to reach the position
		}
		for (pos = 180; pos >= 0; pos -= 1) 
		{ // goes from 180 degrees to 0 degrees
			myservo.write(pos);              // tell servo to go to position in variable 'pos'
			delay(15);                       // waits 15ms for the servo to reach the position
		}
	}
}
void servoNEWtest(int angle, int Tdelay)
{
		myservo.write(angle);
		delay(Tdelay);
}
#pragma endregion

// the setup function runs once when you press reset or power the board
void setup() {
	ini();
	while (true)
	{
		servoNEWtest(servoMIN, delayTEST);
		motorNEWtest(md, FULLSPDBRK, delayTEST);
		servoNEWtest(servoMAX, delayTEST);
	}
}

// the loop function runs over and over again until power down or reset
void loop() {				//LOGICAL STRUCTURE:
	while(ISTRACKING)		//if silver strip has not been detected
	{
		drive(getINPUT());
	}						//by now silver strip has been detected
	while (ISRESCUING)		//if rescue has not ended
	{	
		rescue(getINPUT());
	}
	while (true) 
	{ 
		blink(pLED,blinkSLOW); 
	}	//blink LED continuously when task conmpleted
}
