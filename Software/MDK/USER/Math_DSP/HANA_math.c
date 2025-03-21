#include "HANA_math.h"

float my_abs_float(float in)
{
	if(in < 0.0f)
		return -in;
	else
		return in;
}

short my_abs_short(short in)
{
	if(in < 0)
		return -in;
	else
		return in;
}

//快速计算 1/Sqrt(x) 	
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//快速反正切
// a faster varient of atan.  accurate to 6 decimal places for values between -1 ~ 1 but then diverges quickly
float fast_atan(float v)
{
    float v2 = v*v;
    return (v*(1.6867629106f + v2*0.4378497304f)/(1.6867633134f + v2));
}

//求数字长度
uint8_t lenOfNum(uint16_t num)
{
uint8_t i=0;
	while(num>1)
	{
	i++;
	num=num/10;
	}
	return i;
}

//最小二乘法拟合直线
//参数：x：x坐标数组
//y:y坐标数组
//len:数据长度
//k：返回直线k 值
//b: 返回直线b 值
//确保x和y数组大小一样！！！！！！！
void leastSquareMethod(short*x,short*y,uint8_t len,float* k,float* b)
{
	uint8_t i;
	
	int sumXY,sumX,sumY,sumX2=0;
	float aveX,aveY;
	
	sumXY=sumX=sumY=sumX2=0;
	
	for(i=0;i<len;i++)
	{
		sumXY+=x[i]*y[i];
		sumX+=x[i];
		sumY+=y[i];
		sumX2+=x[i]*x[i];
	}
	
  aveX=sumX/len;
	aveY=sumY/len;
	
	*k=(float)(sumXY-sumX*sumY/len)/(sumX2-sumX*sumX/len);
	*b=(float)(aveY-(*k)*aveX);
}

//求最小二乘法拟合误差
//参数：x：x坐标数组
//y:y坐标数组
//len:数据长度
//k：拟合直线k 值
//b: 拟合直线b 值
//确保x和y数组大小一样！！！！！！！
float getLeastSquareMethodERROR(short*x,short*y,uint8_t len,float k,float b)
{
	float fitVal=0;
	uint8_t i;
	float fitErr=0;
	
	for(i=0;i<len;i++)
	{
		fitVal=x[i]*k+b;
		fitErr+=sq(fitVal-y[i]);
	}
	
	return (float)(fitErr/len);
}

//获得kx+b的值
float getLineValue(uint8_t x,float k,float b)
{
	return (k*x+b);
}

//求两点斜率
float getLineK(short x1,short y1,short x2,short y2)
{
	return (y2-y1)/(x2-x1);
}


//float限幅
// constrain a value
float constrain_float(float amt, float low, float high) 
{
	// the check for NaN as a float prevents propogation of
	// floating point errors through any function that uses
	// constrain_float(). The normal float semantics already handle -Inf
	// and +Inf
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

//16bit限幅
// constrain a int16_t value
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

//16bit最大限幅
int16_t constrain_int16_MAX(int16_t amt, int16_t high) {
	return ((amt)>(high)?(high):(amt));
}

//16Bit最小限幅
int16_t constrain_int16_LOW(int16_t amt, int16_t low) {
	return ((amt)<(low)?(low):(amt));
}


//32bit限幅
// constrain a int32_t value
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

//角度转rad
// degrees -> radians
float radians(float deg) {
	return deg * DEG_TO_RAD;
}

//rad转角度
// radians -> degrees
float degrees(float rad) {
	return rad * RAD_TO_DEG;
}

//平方
// square
float sq(float v) {
	return v*v;
}

//2维向量长
// 2D vector length
float pythagorous2(float a, float b) {
	return sqrtf(sq(a)+sq(b));
}

//3维向量长
// 3D vector length
float pythagorous3(float a, float b, float c) {
	return sqrtf(sq(a)+sq(b)+sq(c));
}


//0~360限幅（0-360）
/*
  wrap an angle in centi-degrees to 0..36000
 */
int32_t wrap_360_cd(int32_t error)
{
    while (error > 360) error -= 360;
    while (error < 0) error += 360;
    return error;
}

//-180~+180限幅（-180~180）
/*
  wrap an angle in centi-degrees to -18000..18000
 */
float wrap_180_cd(float error)
{
    while (error > 180) error -= 360;
    while (error < -180) error += 360;
    return error;
}

//-90~90限幅（-90~90）
/*
  wrap an angle in centi-degrees to -90..90
 */
float wrap_90_cd(float error)
{
    while (error > 90) error -= 180;
    while (error < -90) error += 180;
    return error;
}

uint8_t round_division(float x)//y/x
{
	uint8_t x_uint8=x;
	if((x-x_uint8)>0.5f)
		return x_uint8+1;
	else
		return x_uint8;
}


