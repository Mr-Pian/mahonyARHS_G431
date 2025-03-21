/**********************ICM42688 by ABO**************/

#include "icm45686_task.h"
#include "delay.h"
#include "spi.h"
#include "stdio.h"

ICM45686_TypeDef myIMU1;
ICM45686_TypeDef myIMU2;
IMUDATA_TypeDef myIMU_data;
/*IMU1 寄存器读取函数实现*/
int si_io_imu1_read_reg(uint8_t reg, uint8_t *buf, uint32_t len)
{
	uint8_t reg_r = reg | IMU_SPI_R;
    IMU1_CS(0);//片选选中
	if(HAL_SPI_Transmit(&hspi1,&reg_r,1,0x1000) !=HAL_OK)
		return INV_IMU_ERROR_TRANSPORT;//寄存器读命令
	if(HAL_SPI_Receive(&hspi1,buf,len,0xffff) != HAL_OK)
		return INV_IMU_ERROR_TRANSPORT;//接收len字节数据
    IMU1_CS(1);//片选释放	
	return INV_IMU_OK;
}
/*IMU1 寄存器写入函数实现*/
int si_io_imu1_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len)
{
	uint8_t reg_w = reg | IMU_SPI_W;
  	IMU1_CS(0);//片选选中
	if(HAL_SPI_Transmit(&hspi1,&reg_w,1,0x1000) != HAL_OK)
		return INV_IMU_ERROR_TRANSPORT;//寄存器写命令
	if(HAL_SPI_Transmit(&hspi1,buf,len,0xffff) != HAL_OK)
		return INV_IMU_ERROR_TRANSPORT;//写入len字节数据
  	IMU1_CS(1);//片选释放
	return INV_IMU_OK;
}
/*IMU2 寄存器读取函数实现*/
int si_io_imu2_read_reg(uint8_t reg, uint8_t *buf, uint32_t len)
{
	uint8_t reg_r = reg | IMU_SPI_R;
    IMU2_CS(0);//片选选中
	if(HAL_SPI_Transmit(&hspi3,&reg_r,1,0x1000) !=HAL_OK)
		return INV_IMU_ERROR_TRANSPORT;//寄存器读命令
	if(HAL_SPI_Receive(&hspi3,buf,len,0xffff) != HAL_OK)
		return INV_IMU_ERROR_TRANSPORT;//接收len字节数
    IMU2_CS(1);//片选释放	
	return INV_IMU_OK;
}
/*IMU2 寄存器写入函数实现*/
int si_io_imu2_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len)
{
	uint8_t reg_w = reg | IMU_SPI_W;
  	IMU2_CS(0);//片选选中
	if(HAL_SPI_Transmit(&hspi3,&reg_w,1,0x1000) != HAL_OK)
		return INV_IMU_ERROR_TRANSPORT;//寄存器写命令
	if(HAL_SPI_Transmit(&hspi3,buf,len,0xffff) != HAL_OK)
		return INV_IMU_ERROR_TRANSPORT;//写入数据
  	IMU2_CS(1);//片选释放
	return INV_IMU_OK;
}
/*us延时函数实现*/
void si_sleep_us(uint32_t us)
{
	delay_us(us);
}

/* Initializes IMU device and apply configuration. */
int imu_init(ICM45686_TypeDef *imu)
{
	int                      rc     = 0;
	uint8_t                  whoami = 0;
	inv_imu_int_pin_config_t int_pin_config;
	inv_imu_int_state_t      int_config;

	/* Init transport layer */
	if(imu==&myIMU1)
	{
		imu->dev.transport.read_reg   = si_io_imu1_read_reg;//链接读取寄存器函数
		imu->dev.transport.write_reg  = si_io_imu1_write_reg;//链接写入寄存器函数
		imu->dev.transport.serif_type = UI_SPI4;//通信接口 4线SPI
		imu->dev.transport.sleep_us   = si_sleep_us;//链接us延时函数
	}
	else if(imu==&myIMU2)
	{
		imu->dev.transport.read_reg   = si_io_imu2_read_reg;//链接读取寄存器函数
		imu->dev.transport.write_reg  = si_io_imu2_write_reg;//链接写入寄存器函数
		imu->dev.transport.serif_type = UI_SPI4;//通信接口 4线SPI
		imu->dev.transport.sleep_us   = si_sleep_us;//链接us延时函数		
	}

	/* Wait 3 ms to ensure device is properly supplied  */
	si_sleep_us(3000);

	/* In SPI, configure slew-rate to prevent bus corruption on DK-SMARTMOTION-REVG */
	if (imu->dev.transport.serif_type == UI_SPI3 || imu->dev.transport.serif_type == UI_SPI4) {
		drive_config0_t drive_config0;
		drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
		rc |= inv_imu_write_reg(&imu->dev, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
		si_sleep_us(2); /* Takes effect 1.5 us after the register is programmed */
	}

	/* Check whoami */
	rc |= inv_imu_get_who_am_i(&imu->dev, &whoami);
	if (whoami != INV_IMU_WHOAMI) {
		return -1;
	}

	rc |= inv_imu_soft_reset(&imu->dev);

	/*
	 * Configure interrupts pins
	 * - Polarity High
	 * - Pulse mode
	 * - Push-Pull drive
	 */
	int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
	int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
	rc |= inv_imu_set_pin_config_int(&imu->dev, INV_IMU_INT1, &int_pin_config);

	/* Interrupts configuration */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_UI_DRDY = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu->dev, INV_IMU_INT1, &int_config);

	/* Set FSR */
	rc |= inv_imu_set_accel_fsr(&imu->dev, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G);
	rc |= inv_imu_set_gyro_fsr(&imu->dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);


	/* Set ODR */
	rc |= inv_imu_set_accel_frequency(&imu->dev, ACCEL_CONFIG0_ACCEL_ODR_1600_HZ);
	rc |= inv_imu_set_gyro_frequency(&imu->dev, GYRO_CONFIG0_GYRO_ODR_1600_HZ);

	/* Set BW = ODR/4 */
	rc |= inv_imu_set_accel_ln_bw(&imu->dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
	rc |= inv_imu_set_gyro_ln_bw(&imu->dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);


	/* Sensor registers are not available in ULP, so select RCOSC clock to use LP mode. */
	rc |= inv_imu_select_accel_lp_clk(&imu->dev, SMC_CONTROL_0_ACCEL_LP_CLK_RCOSC);

	/* Set power modes */
	rc |= inv_imu_set_accel_mode(&imu->dev, PWR_MGMT0_ACCEL_MODE_LN);
	rc |= inv_imu_set_gyro_mode(&imu->dev, PWR_MGMT0_GYRO_MODE_LN);

	return rc;
}
/*获取两个IMU数据并合成为一个数据*/
void icm45686_data_update(void)
{
	
	inv_imu_get_register_data(&myIMU1.dev, &myIMU1.data);
	inv_imu_get_register_data(&myIMU2.dev, &myIMU2.data);
	
	myIMU_data.acc_x = (myIMU1.data.accel_data[0] - myIMU2.data.accel_data[0])>>1;
	myIMU_data.acc_y = (myIMU1.data.accel_data[1] - myIMU2.data.accel_data[1])>>1;
	myIMU_data.acc_z = (myIMU1.data.accel_data[2] + myIMU2.data.accel_data[2])>>1;
	
	myIMU_data.gyro_x = (myIMU1.data.gyro_data[0] - myIMU2.data.gyro_data[0])>>1;
	myIMU_data.gyro_y = (myIMU1.data.gyro_data[1] - myIMU2.data.gyro_data[1])>>1;
	myIMU_data.gyro_z = (myIMU1.data.gyro_data[2] + myIMU2.data.gyro_data[2])>>1;	
	
	myIMU_data.temp = (myIMU1.data.temp_data + myIMU2.data.temp_data)>>1;	
}
