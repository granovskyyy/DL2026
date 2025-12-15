#include "I2C.h"

void i2c_init(void)
{
	// Prescaler = 1
	TWSR &= ~((1 << TWPS0) | (1 << TWPS1));
	// 100 kHz @ 16 MHz: TWBR = 72
	TWBR = 72;
	// Enable TWI
	TWCR = (1 << TWEN);
}

static uint8_t i2c_start(uint8_t address_rw)
{
	// Send START
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))) {;}

	// Send SLA+R/W
	TWDR = address_rw;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))) {;}

	return 0;
}

static void i2c_stop(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
}

static uint8_t i2c_write(uint8_t data)
{
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))) {;}
	return 0;
}

static uint8_t i2c_read_ack(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT))) {;}
	return TWDR;
}

static uint8_t i2c_read_nack(void)
{
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT))) {;}
	return TWDR;
}

void i2c_write_reg(uint8_t dev_addr7, uint8_t reg, uint8_t value)
{
	i2c_start((dev_addr7 << 1) | TWI_WRITE);
	i2c_write(reg);
	i2c_write(value);
	i2c_stop();
}

uint8_t i2c_read_reg(uint8_t dev_addr7, uint8_t reg)
{
	uint8_t val;
	i2c_start((dev_addr7 << 1) | TWI_WRITE);
	i2c_write(reg);
	i2c_start((dev_addr7 << 1) | TWI_READ);
	val = i2c_read_nack();
	i2c_stop();
	return val;
}

void i2c_read_multi(uint8_t dev_addr7, uint8_t reg, uint8_t *buf, uint8_t len)
{
	i2c_start((dev_addr7 << 1) | TWI_WRITE);
	i2c_write(reg);
	i2c_start((dev_addr7 << 1) | TWI_READ);

	for (uint8_t i = 0; i < len; i++)
	{
		if (i < (len - 1))
		buf[i] = i2c_read_ack();
		else
		buf[i] = i2c_read_nack();
	}

	i2c_stop();
}
