#pragma once

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define BMP280_I2C_ADDR 0x76
#define BMP280_REG_CHIPID      0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB_START 0x88

typedef struct
{
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
} bmp280_calib_t;


void bmp280_read_calibration(void);
int32_t bmp280_compensate_T(int32_t adc_T);
uint32_t bmp280_compensate_P(int32_t adc_P);
void bmp280_init(void);
void bmp280_read_raw(int32_t *adc_T, int32_t *adc_P);

