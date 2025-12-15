#pragma once

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

void uart_init(uint32_t baud);
void uart_putc(char c);
void uart_puts(const char *s);
void uart_print_uint32(uint32_t v);
		