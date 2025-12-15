#pragma once

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define BLANK_DIGIT  0xFF  // special value for "no segments"

void display_io_init(void);
void set_segments(uint8_t seg, uint8_t dp_on);
void display_timer_init(void);
void update_7SEG_temperature(int32_t temp_c_x100);
void update_7SEG_pressure(uint32_t press_hpa);
void update_7SEG_sonda_temperature(int32_t temp_c_x100);
