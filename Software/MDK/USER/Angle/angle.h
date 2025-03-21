/********************************Copyright (c)**********************************
**
**      
**                           All Rights Reserved
**
**                           By(ABO From UESTC)
**    
**
**----------------------------------文件信息------------------------------------
** 文件名称: angle.h
** 创建人员: ABO
** 创建日期: 2025-03-20
** 文档描述: 
**
********************************End of Head************************************/
 
#ifndef __ANGLE_H_
#define __ANGLE_H_



/*头文件----------------------------------------------------------------------*/
#include "delay.h"
#include "icm45686_task.h"
#include "filter.h"
#include "HANA_math.h"
#include "stdio.h" 
/*宏定义-----------------------------------------------------------------------*/
#define G		        9.7923f		      	// m/s^2	chengdu
#define RadtoDeg    	57.29578f				//弧度到角度 (弧度 * 180/3.1415926)
#define DegtoRad    	0.0174533f				//角度到弧度 (角度 * 3.1415/180)
#define Acc_Gain  		0.0004883f				//加速度变成G (初始化加速度满量程-+16g LSBa = 2*16/65536.0)
#define Gyro_Gain 		0.0610352f				//角速度变成度dps (初始化陀螺仪满量程+-2000 LSBg = 2*2000/65536.0)

#define Gyro_Gr	    	0.0010641f			  	//角速度变成弧度(3.1415/180 * LSBg) 
#define VAR_GyX				100.0f		            //1/方差   0.001f
#define VAR_GyY				100.0f		            //1/方差   0.001f
#define VAR_GyZ				100.0f		            //1/方差   0.001f
#define VAR_AcX				200.0f		            //1/方差   0.001f
#define VAR_AcY				200.0f		            //1/方差   0.001f
#define VAR_AcZ				200.0f		            //1/方差   0.001f
#define ALPHA			0.0f					//一阶互补滤波系数


#define ANGLE_TO_RAD(x)    ((x) * PI / 180.0)                                   // 角度转换为弧度
#define RAD_TO_ANGLE(x)    ((x) * 180.0 / PI)                                   // 弧度转换为角度


typedef struct
{
	float x;
	float y;
	float z;
}vector3float_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}vector3int16_t;

typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

typedef struct{
	vector3int16_t acc;// 加速度
	vector3float_t gyro;// 角速度
	vector3float_t mag;// 磁场强度
	vector3int16_t acc_offset;//陀螺仪零偏
	vector3int16_t gyro_offset;//加速度计零偏
	vector3int16_t mag_offset;//磁力计零偏
}IMU_st;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}Angle_st;

extern  IMU_st IMU_Data;
extern  Angle_st Angle_Data;
extern 	uint8_t imu_init_success;
/*函数描述-----------------------------------------------------------*/


/*函数声明*/
void IMU_Calibration(void);
void IMU_DataUpdate(void);
void IMU_GetAngle(float dt);
/********************************End of File************************************/







#endif
