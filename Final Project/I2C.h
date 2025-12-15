#pragma once

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

#ifdef __AVR_ATmega328PB__
#ifndef TWSR
#define TWSR   TWSR0
#endif
#ifndef TWBR
#define TWBR   TWBR0
#endif
#ifndef TWAR
#define TWAR   TWAR0
#endif
#ifndef TWDR
#define TWDR   TWDR0
#endif
#ifndef TWCR
#define TWCR   TWCR0
#endif
#ifndef TWINT
#define TWINT  TWINT0
#endif
#ifndef TWEA
#define TWEA   TWEA0
#endif
#ifndef TWSTA
#define TWSTA  TWSTA0
#endif
#ifndef TWSTO
#define TWSTO  TWSTO0
#endif
#ifndef TWEN
#define TWEN   TWEN0
#endif
#ifndef TWPS0
#ifdef TWPS00
#define TWPS0  TWPS00
#endif
#endif
#ifndef TWPS1
#ifdef TWPS01
#define TWPS1  TWPS01
#endif
#endif
#endif

#define TWI_READ  1
#define TWI_WRITE 0


void i2c_init(void);
void i2c_write_reg(uint8_t dev_addr7, uint8_t reg, uint8_t value);
uint8_t i2c_read_reg(uint8_t dev_addr7, uint8_t reg);
void i2c_read_multi(uint8_t dev_addr7, uint8_t reg, uint8_t *buf, uint8_t len);