#include "bmp390_task.h"
#include "bmp3.h"
#include "delay.h"
#include "i2c.h"
#include "stdlib.h"
#include "string.h"
extern volatile uint8_t int1_flag;
extern volatile uint8_t int2_flag;

struct bmp3_data aircraft_BMP390_data;

struct bmp3_dev dev;

static uint8_t dev_addr = 0;

void bmp3_check_rslt(const char api_name[], int8_t rslt) {
	switch (rslt) {
	case BMP3_OK:
		//printf("BMP3_OK\r\n");
		/* Do nothing */
		break;
	case BMP3_E_NULL_PTR:
		printf("API [%s] Error [%d] : Null pointer\r\n", api_name, rslt);
		break;
	case BMP3_E_COMM_FAIL:
		printf("API [%s] Error [%d] : Communication failure\r\n", api_name, rslt);
		break;
	case BMP3_E_INVALID_LEN:
		printf("API [%s] Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
		break;
	case BMP3_E_DEV_NOT_FOUND:
		printf("API [%s] Error [%d] : Device not found\r\n", api_name, rslt);
		break;
	case BMP3_E_CONFIGURATION_ERR:
		printf("API [%s] Error [%d] : Configuration Error\r\n", api_name, rslt);
		break;
	case BMP3_W_SENSOR_NOT_ENABLED:
		printf("API [%s] Error [%d] : Warning when Sensor not enabled\r\n", api_name, rslt);
		break;
	case BMP3_W_INVALID_FIFO_REQ_FRAME_CNT:
		printf("API [%s] Error [%d] : Warning when Fifo watermark level is not in limit\r\n", api_name, rslt);
		break;
	default:
		printf("API [%s] Error [%d] : Unknown error code\r\n", api_name, rslt);
		break;
//		while(1){};
	}
}

void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
	/***此处实现微秒延时***/
	delay_us(period);
}
uint8_t bmp390_init_success=0;
BMP3_INTF_RET_TYPE bmp3_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint16_t len, void *intf_ptr)
{
	/***此处实现IIC读寄存器***/
	uint8_t daddr;
	daddr = dev_addr<<1; //device address for SDO high status (0x77<<1)

	HAL_I2C_Master_Transmit(&hi2c1,daddr,&reg_addr,1,0x1000);
	HAL_I2C_Master_Receive(&hi2c1,daddr+1,read_data,len,0x1000);//阻塞读取
	
	return BMP3_INTF_RET_SUCCESS;
}

BMP3_INTF_RET_TYPE bmp3_i2c_write(uint8_t reg_addr, uint8_t *read_data, uint16_t len, void *intf_ptr)
{
	/***此处实现IIC写寄存器***/
	uint8_t daddr; //device address (0x1e<<1)
	uint8_t data_to_send[len+1];
	daddr = dev_addr<<1; //device address for SDO high status (0x77<<1)
	data_to_send[0] = reg_addr;
	memcpy(&data_to_send[1],read_data,len);
	HAL_I2C_Master_Transmit(&hi2c1,daddr,data_to_send,len+1,0xFFFF);
	return BMP3_INTF_RET_SUCCESS;
}

BMP3_INTF_RET_TYPE bmp3_interface_init(struct bmp3_dev *bmp3, uint8_t intf) {
	int8_t rslt = BMP3_OK;

	if (bmp3 != NULL) {
		/* Bus configuration : I2C */
		if (intf == BMP3_I2C_INTF) {
			// printf("I2C Interface\n");
			dev_addr = BMP3_ADDR_I2C_SEC;
			bmp3->read = bmp3_i2c_read;
			bmp3->write = bmp3_i2c_write;
			bmp3->intf = BMP3_I2C_INTF;
		}
//		/* Bus configuration : SPI */
//		else if (intf == BMP3_SPI_INTF) {
//			// printf("SPI Interface\n");
//			dev_addr = 0;
//			bmp3->read = SensorAPI_SPIx_Read;
//			bmp3->write = SensorAPI_SPIx_Write;
//			bmp3->intf = BMP3_SPI_INTF;
//		}

		bmp3->delay_us = bmp3_delay_us;
		bmp3->intf_ptr = &dev_addr;

//		assignment to 'bmp3_write_fptr_t'
//		from incompatible pointer type 'int8_t (*)(uint8_t,  uint8_t *, uint16_t,  void *)'
//
//		{aka 'signed char (*)(unsigned char,  const unsigned char *, long unsigned int,  void *)'}
//		{aka 'signed char (*)(unsigned char,  unsigned char *, short unsigned int,  void *)'}
	} else {
		rslt = BMP3_E_NULL_PTR;
	}

	return rslt;
}

uint8_t BMP390_Init(void) {

	int8_t rslt = 0;

	uint8_t settings_sel;
//	struct bmp3_dev dev; // Original, creats a bug
	struct bmp3_settings settings = { 0 };

	/* Interface reference is given as a parameter
	 *		   For I2C : BMP3_I2C_INTF
	 *		   For SPI : BMP3_SPI_INTF
	 */
#if defined(USE_I2C_INTERFACE)
	rslt = bmp3_interface_init(&dev, BMP3_I2C_INTF);
	#elif defined(USE_SPI_INTERFACE)
	rslt = bmp3_interface_init(&dev, BMP3_SPI_INTF);
#endif
	bmp3_check_rslt("bmp3_interface_init", rslt);

	rslt = bmp3_init(&dev);
	bmp3_check_rslt("bmp3_init", rslt);

	settings.int_settings.drdy_en = BMP3_DISABLE;
	settings.press_en = BMP3_ENABLE;
	settings.temp_en = BMP3_ENABLE;

	settings.odr_filter.press_os = SAMPLING_RATE;
	settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
	settings.odr_filter.odr = OUTPUT_RATE;
	settings.odr_filter.iir_filter = IIR_FILTER_COEFF; // Enable IIR filter, results will be noisy without this

	settings.int_settings.drdy_en = BMP3_ENABLE;
	settings.int_settings.latch = BMP3_INT_PIN_NON_LATCH;
	settings.int_settings.level = BMP3_INT_PIN_ACTIVE_HIGH;
	settings.int_settings.output_mode = BMP3_INT_PIN_PUSH_PULL;
	
	settings_sel = BMP3_SEL_PRESS_EN |//enable pressure measure
	BMP3_SEL_TEMP_EN |//enable temperature measure
	BMP3_SEL_PRESS_OS |//over sampling
	BMP3_SEL_TEMP_OS |//over sampling
	BMP3_SEL_IIR_FILTER | //IIR FILTER
			BMP3_SEL_DRDY_EN | // Data ready interrupt  ->通过外部中断获取数据
			BMP3_SEL_ODR; //output data rate

	rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
	bmp3_check_rslt("bmp3_set_sensor_settings", rslt);

	settings.op_mode = BMP3_MODE_NORMAL;
	rslt = bmp3_set_op_mode(&settings, &dev);
	bmp3_check_rslt("bmp3_set_op_mode", rslt);

	if(rslt==BMP3_OK)
	{
		bmp390_init_success = 1;
		return 0;
	}
	else
	{
		bmp390_init_success=0;
		return 1;
	}
}

/*
 * 
 * Based on "bmp390_getdata" modified to adapt to DMA  --JLB
 */
uint8_t bmp390_data_dma_buf[BMP3_LEN_P_T_DATA];
void bmp390_getregs_DMA() 
{
	//bmp3_get_regs(BMP3_REG_DATA, bmp390_data_dma_buf, BMP3_LEN_P_T_DATA, &dev);//启动DMA接收    数据处理在DMA中断
		uint8_t daddr;
	daddr = dev_addr<<1; //device address for SDO high status (0x77<<1)
	uint8_t red_addr = BMP3_REG_DATA;
	HAL_I2C_Master_Transmit(&hi2c1,daddr,&red_addr,1,0x1000);
	HAL_I2C_Master_Receive_DMA(&hi2c1,daddr+1,bmp390_data_dma_buf,BMP3_LEN_P_T_DATA);
}
extern void parse_sensor_data(const uint8_t *reg_data, struct bmp3_uncomp_data *uncomp_data);
extern int8_t compensate_data(uint8_t sensor_comp,
                              const struct bmp3_uncomp_data *uncomp_data,
                              struct bmp3_data *comp_data,
                              struct bmp3_calib_data *calib_data);
/***DMA数据获取***/															
struct bmp3_data bmp390_getdata_DMA(uint8_t *buf)
{
		struct bmp3_data data = { -1, -1 };
    /* Array to store the pressure and temperature data read from
     * the sensor */
    struct bmp3_uncomp_data uncomp_data = { 0 };

		/* Parse the read data from the sensor */
		parse_sensor_data(buf, &uncomp_data);

		/* Compensate the pressure/temperature/both data read
		 * from the sensor */
		compensate_data(BMP3_PRESS_TEMP, &uncomp_data,&data, &dev.calib_data);
	
		return data;
}
/***阻塞数据获取***/
struct bmp3_data bmp390_getdata() {
	int8_t rslt = 0;

	// Creating data variable
	struct bmp3_data data = { -1, -1 };
	struct bmp3_status status = { { 0 } };

	rslt = bmp3_get_status(&status, &dev);
	bmp3_check_rslt("bmp3_get_status", rslt);

	/* Read temperature and pressure data iteratively based on data ready interrupt */
	if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE)) {
		/*
		 * First parameter indicates the type of data to be read
		 * BMP3_PRESS_TEMP : To read pressure and temperature data
		 * BMP3_TEMP	   : To read only temperature data
		 * BMP3_PRESS	   : To read only pressure data
		 */
		rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
		bmp3_check_rslt("bmp3_get_sensor_data", rslt);

		/* NOTE : Read status register again to clear data ready interrupt status */
		rslt = bmp3_get_status(&status, &dev);
		bmp3_check_rslt("bmp3_get_status", rslt);

//		printf("Data  T: %.2f deg C, P: %.2f Pa\n", (data.temperature), (data.pressure));
	}

	return data;
}
// Convert from mhPa to m
/**************************************************************************/
/*!
 @brief Calculates the altitude (in meters).

 Reads the current atmostpheric pressure (in hPa) from the sensor and
 calculates via the provided sea-level pressure (in hPa).

 @param  seaLevel      Sea-level pressure in hPa
 @return Altitude in meters
 */
/**************************************************************************/
//float Adafruit_BMP3XX::readAltitude(float seaLevel) {
// Equation taken from BMP180 datasheet (page 16): http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
// Note that using the equation from wikipedia can give bad results at high altitude.
// See this thread for more information: http://forums.adafruit.com/viewtopic.php?f=22&t=58064

double convert_Pa_to_meter(double pressure_Pa) {
	double atmospheric_hPa = pressure_Pa / 100.0f;
	double elevation = 44330.0 * (1.0 - pow(atmospheric_hPa / SEA_LEVEL_PRESSURE_HPA, 0.1903));
	if (isnan(elevation) || pressure_Pa == -1) {
		return -1;
	} else {
		return elevation;
	}
}

double convert_mhPa_to_meter(int32_t pressure_mhPa) {
	double atmospheric = pressure_mhPa / 1000.0f;
	double elevation = 44330.0 * (1.0 - pow(atmospheric / SEA_LEVEL_PRESSURE_HPA, 0.1903));
	if (isnan(elevation) || pressure_mhPa == -1) {
		return -1;
	} else {
		return elevation;
	}
}

int32_t convert_Pa_to_mhPa(double pressure_Pa) {
	if(pressure_Pa == -1){
		return -1;
	}
	int32_t pressure_mhPa = pressure_Pa * 10.0f;
	return pressure_mhPa;
}

struct bmp3_data bmp390_data_check(struct bmp3_data data) {
	if(data.temperature == -1){
		data.pressure = -1;
	}
	return data;
}

