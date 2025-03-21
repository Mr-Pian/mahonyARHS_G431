/********************************Copyright (c)**********************************`
** 文件名称: angle.c
** 创建人员: aBo
** 创建日期: 2025-1-09
** 文档描述: 陀螺仪姿态解算
** 结算相关函数
********************************End of Head************************************/
#include "angle.h"


#define Gyro_NUM	1000 // 角速度采样个数
#define Acc_NUM   100 // 加速度采样个数


IMU_st IMU_Data={0};//IMU惯性测量单元数据
Angle_st Angle_Data={0};//角度数据


/*******************************************************************************
** 函数名称: IMU_Calibration()
** 功能描述: 零偏校正，取20组数据
** 参数说明: : 
** 返回说明: None
** 创建人员: JLB
** 创建日期: 2023-11-29
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
void IMU_Calibration(void)
{
	uint16_t i=0;
	vector3float_t gyro[Gyro_NUM];//角速度计原始数据
	vector3float_t gyro_avg = {0,0,0};//角速度计平均值
	vector3float_t gyro_var 		 = {0,0,0};//存放角速度计方差

	
	vector3float_t acc[Acc_NUM];//加速度计原始数据
	vector3float_t acc_avg  = {0,0,0};//加速度计平均值
	vector3float_t acc_var      = {0,0,0};//存放加速度计方差

	do
	{
		/****数据清零begin****/
		gyro_avg.x = 0;
		gyro_avg.y = 0;
		gyro_avg.z = 0;
		gyro_var.x = 0;
		gyro_var.y = 0;
		gyro_var.z = 0;
		IMU_Data.gyro_offset.x = 0;
		IMU_Data.gyro_offset.y = 0;
		IMU_Data.gyro_offset.z = 0;
		acc_avg.x = 0;
		acc_avg.y = 0;
		acc_avg.z = 0;
		acc_var.x = 0;
		acc_var.y = 0;
		acc_var.z = 0;
		IMU_Data.acc_offset.x = 0;
		IMU_Data.acc_offset.y = 0;
		IMU_Data.acc_offset.z = 0;		
		/****数据清零end****/
		
		for(i=0; i<Gyro_NUM; i++)//采集陀螺仪数据计算均值
		{

			//获取IMU数据
			icm45686_data_update();

			gyro[i].x = myIMU_data.gyro_x;
			gyro[i].y = myIMU_data.gyro_y;
			gyro[i].z = myIMU_data.gyro_z;

			//累加求和
			gyro_avg.x += gyro[i].x;
			gyro_avg.y += gyro[i].y;
			gyro_avg.z += gyro[i].z;

			delay_ms(1);
			
		}
		//求平均
		gyro_avg.x = gyro_avg.x/Gyro_NUM;
		gyro_avg.y = gyro_avg.y/Gyro_NUM;
		gyro_avg.z = gyro_avg.z/Gyro_NUM;

		
		for(i=0;i<Acc_NUM;i++)//采集加速度数据计算均值
		{
			//获取IMU数据
			icm45686_data_update();

			acc[i].x = myIMU_data.acc_x;
			acc[i].y = myIMU_data.acc_y;
			acc[i].z = myIMU_data.acc_z;

			//累加求和
			acc_avg.x += acc[i].x;
			acc_avg.y += acc[i].y;
			acc_avg.z += acc[i].z;

			delay_ms(10);			
	
		}
		//求平均
		acc_avg.x = acc_avg.x/Acc_NUM;
		acc_avg.y = acc_avg.y/Acc_NUM;
		acc_avg.z = acc_avg.z/Acc_NUM;

		
		//计算方差 *确保校准的时候是静止状态的（零偏）
		for(i=0; i<Gyro_NUM; i++)
		{
			gyro_var.x += (float) (1.0f/(Gyro_NUM-1)) * (gyro[i].x - gyro_avg.x) * (gyro[i].x - gyro_avg.x);
			gyro_var.y += (float) (1.0f/(Gyro_NUM-1)) * (gyro[i].y - gyro_avg.y) * (gyro[i].y - gyro_avg.y);
			gyro_var.z += (float)	(1.0f/(Gyro_NUM-1)) * (gyro[i].z - gyro_avg.z) * (gyro[i].z - gyro_avg.z);
		}
		for(i=0; i<Acc_NUM; i++)
		{
			acc_var.x += (float) (1.0f/(Acc_NUM-1)) * (acc[i].x - acc_avg.x) * (acc[i].x - acc_avg.x);
			acc_var.y += (float) (1.0f/(Acc_NUM-1)) * (acc[i].y - acc_avg.y) * (acc[i].y - acc_avg.y);
			acc_var.z += (float) (1.0f/(Acc_NUM-1)) * (acc[i].z - acc_avg.z) * (acc[i].z - acc_avg.z);
		}

		printf("gyro_var\nx:%f\ny:%f\nz:%f\n",gyro_var.x,gyro_var.y,gyro_var.z);
		printf("acc_var\nx:%f\ny:%f\nz:%f\n",acc_var.x,acc_var.y,acc_var.z);
		

		//判断并保存静止时的零偏
		if( gyro_var.x<VAR_GyX && gyro_var.y<VAR_GyY && gyro_var.z<VAR_GyZ&&acc_var.x<VAR_AcX && acc_var.y<VAR_AcY &&acc_var.z<VAR_AcZ)//方差足够小
		{
			/***零漂获取begin***/
			IMU_Data.gyro_offset.x  = (int16_t)gyro_avg.x;
			IMU_Data.gyro_offset.y  = (int16_t)gyro_avg.y;
			IMU_Data.gyro_offset.z  = (int16_t)gyro_avg.z;
			IMU_Data.acc_offset.x		=	(int16_t)acc_avg.x;
			IMU_Data.acc_offset.y  = (int16_t)acc_avg.y;
			IMU_Data.acc_offset.z  = (int16_t)acc_avg.z;
			/***零漂获取end***/
			
			printf("gyro_offset.x:%d\r\n",IMU_Data.gyro_offset.x);
			printf("gyro_offset.y:%d\r\n",IMU_Data.gyro_offset.y);
			printf("gyro_offset.z:%d\r\n",IMU_Data.gyro_offset.z);
			printf("acc_offset.x:%d\r\n",IMU_Data.acc_offset.x);
			printf("acc_offset.y:%d\r\n",IMU_Data.acc_offset.y);
			printf("acc_offset.z:%d\r\n",IMU_Data.acc_offset.z);
			delay_ms(1000);//延时等待姿态解算稳定
			return;
		}

	}while(1);

	
}

/*******************************************************************************
** 函数名称: icm42688_Update()
** 功能描述: 零偏校正，取20组数据
** 参数说明: : 
** 返回说明: None
** 创建人员: JLB
** 创建日期: 2024-1-20
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/

void IMU_DataUpdate(void)
{
//	volatile vector3float_t gyro_trans={0,0,0};
//	volatile vector3float_t gyro_temp = {0,0,0};
//	volatile vector3float_t acc_trans={0,0,0};
//	volatile vector3float_t acc_temp = {0,0,0};
//	int16_t acc_LPF_in[3],acc_LPF_out[3];

	
			//获取IMU数据
			icm45686_data_update();
	
//	static  kalman_1_struct ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};
//	
//	kalman_1(&ekf[0],(float)icm42688_gyro_x);  //对x轴角速度一维卡尔曼滤波
//	gyro_temp.x=(short)ekf[0].out;
//	gyro_temp.x=(gyro_temp.x/10)*10;//降低角速度值的精确度来获得相对稳定的积分值
//  
//	kalman_1(&ekf[1],(float)icm42688_gyro_y);  //对y轴角速度一维卡尔曼滤波
//	gyro_temp.y=(short)ekf[1].out;
//	gyro_temp.y=(gyro_temp.y/10)*10;//降低角速度值的精确度来获得相对稳定的积分值
//  
//	kalman_1(&ekf[2],(float)icm42688_gyro_z);  //对Z轴角速度一维卡尔曼滤波
//	gyro_temp.z=(short)ekf[2].out;
//	gyro_temp.z=(gyro_temp.z/10)*10;//降低角速度值的精确度来获得相对稳定的积分值
  
		
		
	IMU_Data.gyro.x= (myIMU_data.gyro_x-IMU_Data.gyro_offset.x)*Gyro_Gain;//IMU角速度数据更新+去零偏
	IMU_Data.gyro.y= (myIMU_data.gyro_y-IMU_Data.gyro_offset.y)*Gyro_Gain;
	IMU_Data.gyro.z= (myIMU_data.gyro_z-IMU_Data.gyro_offset.z)*Gyro_Gain;
	
	
//	static Filter_LPF_1 LPF1[3]={{100,0,10},{100,0,10},{8330,0,10}};//加速度收敛
//	acc_LPF_in[0] = icm42688_acc_x;
//	acc_LPF_in[1] = icm42688_acc_y;
//	acc_LPF_in[2] = icm42688_acc_z;
//	for(int i=0;i<3;i++)//低通滤波处理
//	{
//		LPF1[i].new_data = acc_LPF_in[i];
//		acc_LPF_out[i] = (int16_t)LPF_1_Filter_2(&LPF1[i],0.005f);
//		LPF1[i].old_data = acc_LPF_in[i];
//	}
	
	IMU_Data.acc.x=myIMU_data.acc_x;//IMU加速度数据更新
	IMU_Data.acc.y=myIMU_data.acc_y;
	IMU_Data.acc.z=myIMU_data.acc_z;
	
}
/*******************************************************************************
** 函数名称: GetAngle(const _IMU_st *pImu,_Angle_st *pAngle, float dt) 
** 功能描述: 计算车身角度信息
** 参数说明:  
					pImu:陀螺仪原始数据
					pAngle:角度数据
					dt:采样时间 单位：s
** 返回说明: None
** 创建人员: JLB
** 创建日期: 2024-3-16
**------------------------------------------------------------------------------
** 修改人员:
** 修改日期:
** 修改描述:
**------------------------------------------------------------------------------
********************************************************************************/
void IMU_GetAngle(float dt) 
{		
	volatile struct V{
				float x;
				float y;
				float z;
				} Gravity,Acc,Gyro,AccGravity;

	static struct V GyroIntegError = {0};
	static  float KpDef = 0.8f ;//融合滤波系数
	static  float KiDef = 0.0003f;
	static Quaternion NumQ = {1, 0, 0, 0};
	float q0_t,q1_t,q2_t,q3_t;
//  float NormAcc;	
	float NormQuat; 
	float HalfTime = dt * 0.5f;

	// 提取等效旋转矩阵中的重力分量 
	Gravity.x = 2*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);								
	Gravity.y = 2*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);						  
	Gravity.z = 1-2*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

	// 加速度归一化
	NormQuat = invSqrt(sq(IMU_Data.acc.x)+ sq(IMU_Data.acc.y) +sq(IMU_Data.acc.z));
	Acc.x =IMU_Data.acc.x * NormQuat;
	Acc.y = IMU_Data.acc.y * NormQuat;
	Acc.z = IMU_Data.acc.z * NormQuat;	
	
 	//向量差乘得出的值
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	
	//再做加速度积分补偿角速度的补偿值
	GyroIntegError.x += AccGravity.x * KiDef;
	GyroIntegError.y += AccGravity.y * KiDef;
	GyroIntegError.z += AccGravity.z * KiDef;
	
	//角速度融合加速度积分补偿值
	Gyro.x = IMU_Data.gyro.x * DegtoRad + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
	Gyro.y = IMU_Data.gyro.y * DegtoRad + KpDef * AccGravity.y  +  GyroIntegError.y;
	Gyro.z = IMU_Data.gyro.z * DegtoRad + KpDef * AccGravity.z  +  GyroIntegError.z;		
	
	// 一阶龙格库塔法, 更新四元数
	q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;
	NumQ.q0 += q0_t;
	NumQ.q1 += q1_t;
	NumQ.q2 += q2_t;
	NumQ.q3 += q3_t;
	
	// 四元数归一化
	NormQuat = invSqrt(sq(NumQ.q0) + sq(NumQ.q1) + sq(NumQ.q2) + sq(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;	


		/*机体坐标系下的Z方向向量*/
	float vecxZ = 2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3 ;/*矩阵(3,1)项*/
	float vecyZ = 2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;/*矩阵(3,2)项*/
	float veczZ =  1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;	/*矩阵(3,3)项*/		 

	//float yaw_G =IMU_Data.gyro.z * Gyro_Gain;//将Z轴角速度陀螺仪值 转换为Z角度/秒      Gyro_G陀螺仪初始化量程+-2000度每秒于1 / (65536 / 4000) = 0.03051756*2		
	if(fabs(Gyro.z) > 0.005f) //数据太小可以认为是干扰，不是偏航动作
	{
		Angle_Data.yaw  += Gyro.z*RadtoDeg* dt;//角速度积分成偏航角			
	}
	Angle_Data.pitch  =  asinf(vecxZ)* RadtoDeg;	 //俯仰角					
	Angle_Data.roll	= atan2f(vecyZ,veczZ) * RadtoDeg;	//横滚角

		//	NormAcc = pImu->accX* vecxZ + pImu->accY * vecyZ + pImu->accZ * veczZ;	/*Z轴垂直方向上的加速度，此值涵盖了倾斜时在Z轴角速度的向量和，不是单纯重力感应得出的值*/				
}


 

