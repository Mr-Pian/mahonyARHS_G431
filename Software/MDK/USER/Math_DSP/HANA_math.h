#ifndef __HANA_MATH_H__
#define __HANA_MATH_H__
#include "main.h"
#include "math.h"


#define isignof(IntVar) ((IntVar)<0?-1:1)//求值符号
#define isignOrZero(a) ((a)<0?-1:(a)>0?1:0)//求值符号（大于零，小于零，零）

#ifndef ABS
#define ABS(a) (((a)>0) ? (a):(-1*a))
#endif

#ifndef max
#define max(a,b)        ((a) > (b) ? (a) : (b))//求最大
#endif

#ifndef min
#define min(a,b)        ((a) < (b) ? (a) : (b))//求最小
#endif

#ifndef LABS
#define LABS(a)	((a)>=0 ? (a) : -(a))//绝对值
#endif

#ifndef MAX
#define MAX(a,b) ((a)>(b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a,b) ((a)<(b) ? (a) :( b))
#endif

//限幅
#define LIMIT2MAX( a,b)( (a) = (a)>(b)?(b):(a))
#define LIMIT2MIN( a,b)( (a) = (a)<(b)?(b):(a))
#define LIMIT(val,minV,maxV) ((val)=((val)>(maxV))?(maxV) : ( ((val)<(minV)) ? (minV) : (val)))

//元素个数
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a)   (sizeof( (a) ) / sizeof( *(a) ))
#endif

//绝对差值
#ifndef ABS_DIFF
#define ABS_DIFF(a,b) (((a)>(b)) ? ((a)-(b)) : ((b)-(a)))
#endif

//求符号（正负零）
#ifndef SIGN_OF
#define SIGN_OF(a)   ((a)<0?-1:(a)>0?1:0)
#endif

//交换
#ifndef SWAP
#define SWAP(a, b) ((a)^=(b),(b)^=(a),(a)^=(b))
#endif

#define M_2PI 6.283185307179586  //2Pi
#define M_PI 3.141592653589793f  //Pi
#define M_PI_2 1.570796326794897f  //Pi/2

#define DEG_TO_RAD 0.017453292519943295769236907684886f  //degree->rad
#define RAD_TO_DEG 57.295779513082320876798154814105f   //rad->degree


float invSqrt(float x); //快速计算 1/Sqrt(x) 	
float fast_atan(float v);//快速反正切
float constrain_float(float amt, float low, float high);//float限幅
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);//16bit限幅
int32_t wrap_360_cd(int32_t error);//0~360限幅（0-360）
float wrap_180_cd(float error);//-180~+180限幅（-180~180）
float wrap_90_cd(float error);//-90~90限幅（-90~90）
float sq(float v);//平方
float radians(float deg);//角度转rad
float degrees(float rad);//rad转角度
float pythagorous2(float a, float b);//2维向量长
float pythagorous3(float a, float b, float c);//3维向量长

uint8_t lenOfNum(uint16_t num);
void leastSquareMethod(short*x,short*y,uint8_t len,float* k,float* b);
float getLineK(short x1,short y1,short x2,short y2);
float getLeastSquareMethodERROR(short*x,short*y,uint8_t len,float k,float b);
float getLineValue(uint8_t x,float k,float b);

int16_t constrain_int16_LOW(int16_t amt, int16_t low);
int16_t constrain_int16_MAX(int16_t amt, int16_t high);

float my_abs_float(float in);
short my_abs_short(short in);
uint8_t round_division(float x);

#endif
