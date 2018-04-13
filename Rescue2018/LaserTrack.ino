///Obsolete Code Using LaserTrack(Do Not Delete)

#pragma region Defs

#pragma endregion

#pragma region Vars
const int BUFFERSIZE = 100;
float BUFFER[BUFFERSIZE];
int IncontinuityCounts = 0;
int currentPosition = 0;
float MINIMUM_BALL_DIST = 10;
float balls[10];
int ballCOUNT = 0;
#pragma endregion

#pragma region Level_1_Funcs
///
///This finction checks distance from the laser sensor and returns a float
///
float chkDist(int laser)
{
float dist = 0.0;
if (true)
{
//dosth
return dist;
}
else
{
return 0.0;
}
}
///
///This finction checks distance from the ultrasound sensor and returns a float
///
float chkDist(int ultrasound)
{
float dist = 0.0;
if (true)
{
//dosth
return dist;
}
else
{
return 0.0;
}
}
///
///This finction checks angle from the gyro and returns a float
///
float chkAngle(int gyro)
{
float theta = 0.0;
if (true)
{
//dosth
return theta;
}
else
{
return 0.0;
}
}
#pragma endregion

#pragma region Level_2_Funcs
///
///push the input input into buffer buffer[] at position pos.
///
bool push(float input, float buffer[], int pos)
{
buffer[pos] = input;
return true;
}
///
///checks if buffer[] is continuous at position pos, or times of incontinuity.
///
bool isContinuous(float buffer[], int pos, int count)
{
//making a sequential copy of buffer[] as double data[]
double data[BUFFERSIZE][2];
int iPos = pos;
for (size_t i = 0; i < BUFFERSIZE; i++)
{
data[i][1] = buffer[iPos];
data[i][0] = i;
iPos += 1;
if (iPos >= BUFFERSIZE)
{
iPos -= BUFFERSIZE;
}
}
//operate on data[]
double a, b;
double sp[4];
if (LinearRegression((double*)data,BUFFERSIZE-1-count,&a,&b,sp) == 0)
{
if (abs(a+b*data[BUFFERSIZE-1][0]-data[BUFFERSIZE-1][1])>sqrt(sp[2])*2.0)
{
count += 1;
}
if (count > 3)
{
count = 0;
return false;
}
else
{
return true;
}
}
}

//Linear Regression: Y = a + bx
//dada[rows*2]: X, Y; rows: numers of data pairs; a, b: coefficients
// SquarePoor[4]: 返回方差分析指标: 回归平方和，剩余平方和，回归平方差，剩余平方差
//0:succeeded; -1: failed
int LinearRegression(double *data, int rows, double *a, double *b, double *SquarePoor)
{
int m;
double *p, Lxx = 0.0, Lxy = 0.0, xa = 0.0, ya = 0.0;
if (data == 0 || a == 0 || b == 0 || rows < 1)
return -1;
for (p = data, m = 0; m < rows; m++)
{
xa += *p++;
ya += *p++;
}
xa /= rows;                                     // X平均值
ya /= rows;                                     // Y平均值
for (p = data, m = 0; m < rows; m++, p += 2)
{
Lxx += ((*p - xa) * (*p - xa));             // Lxx = Sum((X - Xa)平方)
Lxy += ((*p - xa) * (*(p + 1) - ya));       // Lxy = Sum((X - Xa)(Y - Ya))
}
*b = Lxy / Lxx;                                 // b = Lxy / Lxx
*a = ya - *b * xa;                              // a = Ya - b*Xa
if (SquarePoor == 0)
return 0;
// 方差分析
SquarePoor[0] = SquarePoor[1] = 0.0;
for (p = data, m = 0; m < rows; m++, p++)
{
Lxy = *a + *b * *p++;
SquarePoor[0] += ((Lxy - ya) * (Lxy - ya)); // U(回归平方和)
SquarePoor[1] += ((*p - Lxy) * (*p - Lxy)); // Q(剩余平方和)
}
SquarePoor[2] = SquarePoor[0];                  // 回归方差
SquarePoor[3] = SquarePoor[1] / (rows - 2);     // 剩余方差
return 0;
}

#pragma endregion

// the setup function runs once when you press reset or power the board
void Lsetup() {
#pragma region LaserTrackCode
	int laser = 0;
	for (size_t i = 0; i < BUFFERSIZE; i++)
	{
	if (push(chkDist(laser),BUFFER,currentPosition))
	{
	currentPosition++;
	}
	}
	if (currentPosition >= BUFFERSIZE)
	{
	currentPosition -= BUFFERSIZE;
	}

#pragma endregion

}

// the loop function runs over and over again until power down or reset
void Lloop() {
#pragma region LaserTrackCode

	while (chkDist(1)>MINIMUM_BALL_DIST)//loop while front is more than MINIMUMBALLDIST fron the wall
	{
	if (isContinuous(BUFFER,currentPosition,IncontinuityCounts))
	{
	//drive();
	}
	else
	{
	balls[ballCOUNT] = chkDist(1);//get front dist from wall and record to list
	ballCOUNT++;
	}
	//push
	}

	//perform collection
	for (size_t i = ballCOUNT-1; i >-1; i--)
	{
	while (chkDist(1) < balls[i]) //1 as the front laser
	{//loops until reaches the distance
	//driveback();
	}
	//turn90degrees();
	while (chkDist(1) > MINIMUM_BALL_DIST)
	{
	//drive();
	}
	while (true)//while servo not down servo down by 1 degree
	{
	//servo.write();
	}
	while (true)
	{
	//goback();
	}
	//turn90degreesback();
	float distancefromWall = chkDist(1);//1 as the front laser, takes down
	while (chkDist(1)<distancefromWall)// 1 as the front laser
	{//loops until reversed back
	}

	}

#pragma endregion

}
