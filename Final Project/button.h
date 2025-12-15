#pragma once

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

void button_init(void);
uint8_t button_was_pressed(void);