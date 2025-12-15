#include "DS18B20.h"


static inline void onewire_low(void)
{
	ONEWIRE_DDR  |=  (1 << ONEWIRE_BIT);  // output
	ONEWIRE_PORT &= ~(1 << ONEWIRE_BIT);  // drive low
}

static inline void onewire_release(void)
{
	ONEWIRE_DDR  &= ~(1 << ONEWIRE_BIT);  // input
	ONEWIRE_PORT |=  (1 << ONEWIRE_BIT);  // enable pull-up
}

static inline uint8_t onewire_read_pin(void)
{
	return (ONEWIRE_PINR & (1 << ONEWIRE_BIT)) ? 1 : 0;
}

void ds18b20_io_init(void)
{
	onewire_release();  // line idle high (requires external pull-up)
}

// returns 1 if presence pulse detected
uint8_t ds18b20_reset(void)
{
	uint8_t presence;

	onewire_low();
	_delay_us(480);
	onewire_release();
	_delay_us(70);
	presence = (onewire_read_pin() == 0);  // 0 = presence
	_delay_us(410);

	return presence;
}

void onewire_write_bit(uint8_t bit)
{
	if (bit)
	{
		onewire_low();
		_delay_us(6);
		onewire_release();
		_delay_us(64);
	}
	else
	{
		onewire_low();
		_delay_us(60);
		onewire_release();
		_delay_us(10);
	}
}

uint8_t onewire_read_bit(void)
{
	uint8_t bit;

	onewire_low();
	_delay_us(6);
	onewire_release();
	_delay_us(9);
	bit = onewire_read_pin();
	_delay_us(55);

	return bit;
}

void onewire_write_byte(uint8_t byte)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		onewire_write_bit(byte & 0x01);
		byte >>= 1;
	}
}

uint8_t onewire_read_byte(void)
{
	uint8_t byte = 0;
	for (uint8_t i = 0; i < 8; i++)
	{
		if (onewire_read_bit())
		byte |= (1 << i);
	}
	return byte;
}

// Convert and read DS18B20 temperature, returns in 0.01 °C
int32_t ds18b20_get_temperature_c_x100(void)
{
	if (!ds18b20_reset())
	return 0;  // sensor not present

	// SKIP ROM
	onewire_write_byte(0xCC);
	// CONVERT T
	onewire_write_byte(0x44);

	// wait for conversion (max 750 ms for 12-bit)
	_delay_ms(750);

	if (!ds18b20_reset())
	return 0;

	// SKIP ROM
	onewire_write_byte(0xCC);
	// READ SCRATCHPAD
	onewire_write_byte(0xBE);

	uint8_t lsb = onewire_read_byte();
	uint8_t msb = onewire_read_byte();

	for (uint8_t i = 0; i < 7; i++)
	(void)onewire_read_byte();

	int16_t raw = (int16_t)((msb << 8) | lsb); // 1/16 °C
	int32_t temp_c_x100 = ((int32_t)raw * 100) / 16;

	return temp_c_x100;
}

void update_mode_leds(uint8_t mode)
{
	// turn off all LEDs
	PORTB &= ~((1 << PB2) | (1 << PB3) | (1 << PB4));

	switch(mode)
	{
		case 0: PORTB |= (1 << PB2); break; // BMP temp
		case 1: PORTB |= (1 << PB3); break; // BMP pres
		case 2: PORTB |= (1 << PB4); break; // DS18B20 temp
	}
}
