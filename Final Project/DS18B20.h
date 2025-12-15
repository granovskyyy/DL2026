#pragma once

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define ONEWIRE_PORT PORTD
#define ONEWIRE_DDR  DDRD
#define ONEWIRE_PINR PIND
#define ONEWIRE_BIT  PD4

void ds18b20_io_init(void);
uint8_t ds18b20_reset(void);
void onewire_write_bit(uint8_t bit);
uint8_t onewire_read_bit(void);
void onewire_write_byte(uint8_t byte);
uint8_t onewire_read_byte(void);
int32_t ds18b20_get_temperature_c_x100(void);
void update_mode_leds(uint8_t mode);
	