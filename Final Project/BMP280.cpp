#include "BMP280.h"
#include "I2C.h"
#include "uart.h"

bmp280_calib_t calib;
int32_t t_fine;

void bmp280_read_calibration(void)
{
	uint8_t data[24];
	i2c_read_multi(BMP280_I2C_ADDR, BMP280_REG_CALIB_START, data, 24);

	calib.dig_T1 = (uint16_t)(data[1] << 8 | data[0]);
	calib.dig_T2 = (int16_t)(data[3] << 8 | data[2]);
	calib.dig_T3 = (int16_t)(data[5] << 8 | data[4]);
	calib.dig_P1 = (uint16_t)(data[7] << 8 | data[6]);
	calib.dig_P2 = (int16_t)(data[9] << 8 | data[8]);
	calib.dig_P3 = (int16_t)(data[11] << 8 | data[10]);
	calib.dig_P4 = (int16_t)(data[13] << 8 | data[12]);
	calib.dig_P5 = (int16_t)(data[15] << 8 | data[14]);
	calib.dig_P6 = (int16_t)(data[17] << 8 | data[16]);
	calib.dig_P7 = (int16_t)(data[19] << 8 | data[18]);
	calib.dig_P8 = (int16_t)(data[21] << 8 | data[20]);
	calib.dig_P9 = (int16_t)(data[23] << 8 | data[22]);
}


int32_t bmp280_compensate_T(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) *
	((int32_t)calib.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
	((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
	((int32_t)calib.dig_T3)) >> 14;

	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;  
	return T;
}

// pressure in Pa (unsigned)
uint32_t bmp280_compensate_P(int32_t adc_P)
{
	double var1, var2, p;

	var1 = ((double)t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)calib.dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)calib.dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)calib.dig_P4) * 65536.0);
	var1 = (((double)calib.dig_P3) * var1 * var1 / 524288.0 +
	((double)calib.dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * (double)calib.dig_P1;

	if (var1 == 0.0)
	return 0;

	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)calib.dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)calib.dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)calib.dig_P7)) / 16.0;

	if (p < 0.0)
	p = 0.0;

	return (uint32_t)(p + 0.5);
}

void bmp280_init(void)
{
	// Soft reset
	i2c_write_reg(BMP280_I2C_ADDR, BMP280_REG_RESET, 0xB6);
	_delay_ms(5);

	// Read chip ID
	uint8_t id = i2c_read_reg(BMP280_I2C_ADDR, BMP280_REG_CHIPID);
	uart_puts("BMP280 ID = 0x");
	const char hex[] = "0123456789ABCDEF";
	uart_putc(hex[(id >> 4) & 0x0F]);
	uart_putc(hex[id & 0x0F]);
	uart_puts("\r\n\n");

	// Read calibration
	bmp280_read_calibration();

	// ctrl_meas: temp x1, press x1, normal mode => 0x27
	i2c_write_reg(BMP280_I2C_ADDR, BMP280_REG_CTRL_MEAS, 0x27);
	// config: standby 500 ms, filter off => 0xA0
	i2c_write_reg(BMP280_I2C_ADDR, BMP280_REG_CONFIG, 0xA0);
}

void bmp280_read_raw(int32_t *adc_T, int32_t *adc_P)
{
	uint8_t buf[6];
	i2c_read_multi(BMP280_I2C_ADDR, BMP280_REG_PRESS_MSB, buf, 6);

	int32_t adc_P_local = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
	int32_t adc_T_local = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

	*adc_P = adc_P_local;
	*adc_T = adc_T_local;
}
