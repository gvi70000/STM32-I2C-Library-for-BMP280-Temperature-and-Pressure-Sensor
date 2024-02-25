#include "BMP280.h"

#define BMP280_I2C_ADDRESS  0x76 //or 0x77
#define BMP280_ADDRESS  (BMP280_I2C_ADDRESS << 1) //or 0x77
#define BMP280_CHIP_ID  0x58 /* BMP280 has chip-id 0x58 */

/**
 * BMP280 registers
 */
#define BMP280_REG_TEMP_XLSB   0xFC /* bits: 7-4 */
#define BMP280_REG_TEMP_LSB    0xFB
#define BMP280_REG_TEMP_MSB    0xFA
#define BMP280_REG_TEMP        (BMP280_REG_TEMP_MSB)
#define BMP280_REG_PRESS_XLSB  0xF9 /* bits: 7-4 */
#define BMP280_REG_PRESS_LSB   0xF8
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_PRESSURE    (BMP280_REG_PRESS_MSB)
#define BMP280_REG_CONFIG      0xF5 /* bits: 7-5 t_sb; 4-2 filter; 0 spi3w_en */
#define BMP280_REG_CTRL        0xF4 /* bits: 7-5 osrs_t; 4-2 osrs_p; 1-0 mode */
#define BMP280_REG_STATUS      0xF3 /* bits: 3 measuring; 0 im_update */
#define BMP280_REG_CTRL_HUM    0xF2 /* bits: 2-0 osrs_h; */
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_ID          0xD0
#define BMP280_REG_CALIB       0x88
#define BMP280_REG_HUM_CALIB   0x88
#define BMP280_RESET_VALUE     0xB6

#define BMP280_RESPONCE_TIME   500

uint16_t dig_T1;
int16_t  dig_T2;
int16_t  dig_T3;
uint16_t dig_P1;
int16_t  dig_P2;
int16_t  dig_P3;
int16_t  dig_P4;
int16_t  dig_P5;
int16_t  dig_P6;
int16_t  dig_P7;
int16_t  dig_P8;
int16_t  dig_P9;
uint8_t  id;        /* Chip ID */

bmp280_params_t myParams;

static bool read_register16(uint8_t addr, uint16_t *value) {
	uint8_t rx_buff[2];
	if (HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDRESS, addr, 1, rx_buff, 2, BMP280_RESPONCE_TIME) == HAL_OK) {
		*value = (uint16_t) ((rx_buff[1] << 8) | rx_buff[0]);
		return true;
	} else
		return false;
}

static inline int read_data(uint8_t addr, uint8_t *value, uint8_t len) {
	if (HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDRESS, addr, 1, value, len, BMP280_RESPONCE_TIME) == HAL_OK)
		return 0;
	else
		return 1;
}

static bool read_calibration_data(void) {

	if (read_register16(0x88, &dig_T1) && read_register16(0x8a, (uint16_t *) &dig_T2)
			&& read_register16(0x8c, (uint16_t *) &dig_T3) && read_register16(0x8e, &dig_P1)
			&& read_register16(0x90, (uint16_t *) &dig_P2) && read_register16(0x92, (uint16_t *) &dig_P3)
			&& read_register16(0x94, (uint16_t *) &dig_P4) && read_register16(0x96, (uint16_t *) &dig_P5)
			&& read_register16(0x98, (uint16_t *) &dig_P6) && read_register16(0x9a, (uint16_t *) &dig_P7)
			&& read_register16(0x9c, (uint16_t *) &dig_P8) && read_register16(0x9e,	(uint16_t *) &dig_P9))
	{
		return true;
	}
	return false;
}

static int write_register8(uint8_t addr, uint8_t value) {
	if (HAL_I2C_Mem_Write(&hi2c1, BMP280_ADDRESS, addr, 1, &value, 1, BMP280_RESPONCE_TIME) == HAL_OK)
		return false;
	else
		return true;
}

bool bmp280_init(void) {
/**
 * Initialize default parameters.
 * Default configuration:
 *      mode: NORAML
 *      filter: OFF
 *      oversampling: x4
 *      standby time: 250ms
 */
	myParams.mode = BMP280_MODE_NORMAL;
	myParams.filter = BMP280_FILTER_16;
	myParams.oversampling_pressure = BMP280_ULTRA_HIGH_RES;
	myParams.oversampling_temperature = BMP280_ULTRA_HIGH_RES;
	myParams.standby = BMP280_STANDBY_250;
	
	if (read_data(BMP280_REG_ID, &id, 1)) {
		return false;
	}
	// Soft reset.
	if (write_register8(BMP280_REG_RESET, BMP280_RESET_VALUE)) {
		return false;
	}
	// Wait until finished copying over the NVP data.
	while (1) {
		uint8_t status;
		if (!read_data(BMP280_REG_STATUS, &status, 1)	&& (status & 1) == 0)
			break;
	}
	if (!read_calibration_data()) {
		return false;
	}

	uint8_t config = (myParams.standby << 5) | (myParams.filter << 2);
	if (write_register8(BMP280_REG_CONFIG, config)) {
		return false;
	}
	if (myParams.mode == BMP280_MODE_FORCED) {
		myParams.mode = BMP280_MODE_SLEEP;  // initial mode for forced is sleep
	}
	uint8_t ctrl = (myParams.oversampling_temperature << 5) | (myParams.oversampling_pressure << 2) | (myParams.mode);
	if (write_register8(BMP280_REG_CTRL, ctrl)) {
		return false;
	}
	return true;
}

bool bmp280_force_measurement(void) {
	uint8_t ctrl;
	if (read_data(BMP280_REG_CTRL, &ctrl, 1))
		return false;
	ctrl &= ~0b11;  // clear two lower bits
	ctrl |= BMP280_MODE_FORCED;
	if (write_register8(BMP280_REG_CTRL, ctrl))
		return false;
	return true;
}

bool bmp280_is_measuring(void) {
	uint8_t status;
	if (read_data(BMP280_REG_STATUS, &status, 1))
		return false;
	if (status & (1 << 3)) {
		return true;
	}
	return false;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in degrees Celsius.
 */
static inline int32_t compensate_temperature(int32_t adc_temp,	int32_t *fine_temp) {
	int32_t var1, var2;
	var1 = ((((adc_temp >> 3) - ((int32_t) dig_T1 << 1))) * (int32_t) dig_T2) >> 11;
	var2 = (((((adc_temp >> 4) - (int32_t) dig_T1)	* ((adc_temp >> 4) - (int32_t) dig_T1)) >> 12)	* (int32_t) dig_T3) >> 14;
	*fine_temp = var1 + var2;
	return (*fine_temp * 5 + 128) >> 8;
}

/**
 * Compensation algorithm is taken from BMP280 datasheet.
 *
 * Return value is in Pa, 24 integer bits and 8 fractional bits.
 */
static inline uint32_t compensate_pressure(int32_t adc_press,	int32_t fine_temp) {
	int64_t var1, var2, p;
	var1 = (int64_t) fine_temp - 128000;
	var2 = var1 * var1 * (int64_t) dig_P6;
	var2 = var2 + ((var1 * (int64_t) dig_P5) << 17);
	var2 = var2 + (((int64_t) dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t) dig_P3) >> 8)	+ ((var1 * (int64_t) dig_P2) << 12);
	var1 = (((int64_t) 1 << 47) + var1) * ((int64_t) dig_P1) >> 33;
	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}
	p = 1048576 - adc_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = ((int64_t) dig_P9 * (p >> 13) * (p >> 13)) >> 25;
	var2 = ((int64_t) dig_P8 * p) >> 19;
	p = ((p + var1 + var2) >> 8) + ((int64_t) dig_P7 << 4);
	return p;
}

bool bmp280_read_fixed(int32_t *temperature, uint32_t *pressure) {
	int32_t adc_pressure;
	int32_t adc_temp;
	uint8_t data[6];
	
	if (read_data(BMP280_REG_PRESS_MSB, data, 6)) {
		return false;
	}
	adc_pressure = data[0] << 12 | data[1] << 4 | data[2] >> 4;
	adc_temp = data[3] << 12 | data[4] << 4 | data[5] >> 4;

	int32_t fine_temp;
	*temperature = compensate_temperature(adc_temp, &fine_temp);
	*pressure = compensate_pressure(adc_pressure, fine_temp);
	return true;
}

bool bmp280_read_float(float *temperature, float *pressure) {
	int32_t fixed_temperature;
	uint32_t fixed_pressure;
	uint32_t fixed_humidity;
	if (bmp280_read_fixed(&fixed_temperature, &fixed_pressure)) {
		*temperature = (float) fixed_temperature / 100.0;
		*pressure = (float) fixed_pressure / 25600.0;
		return true;
	}
	return false;
}