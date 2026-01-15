#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "I2C.h"
#include "BMP280.h"
#include "DS18B20.h"
#include "7SEG.h"
#include "button.h"

static inline void uart_print_int32(int32_t v)
{
	if (v < 0)
	{
		uart_putc('-');
		v = -v;
	}
	uart_print_uint32((uint32_t)v);
}


int main(void)
{
	// output LEDs
	DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB4);
	PORTB &= ~((1 << PB2) | (1 << PB3) | (1 << PB4)); // turn off all LEDs

	uart_init(9600);
	i2c_init();
	display_io_init();
	display_timer_init();
	button_init();
	ds18b20_io_init();

	sei(); // enable global interrupts (for display timer)

	uart_puts("\r\nBMP280 + DS18B20 temp + pressure with 7-seg display\r\n");
	bmp280_init();

	int32_t adc_T = 0, adc_P = 0;
	int32_t temp_c_x100 = 0;
	uint32_t press_pa = 0, press_hpa = 0;

	int32_t sonda_temp_c_x100 = 0;

	// 7SEG_mode:
	// 0 -> BMP280 temperature
	// 1 -> BMP280 pressure
	// 2 -> DS18B20 temperature (sonda)
	
	uint8_t SEG_mode = 0;

	// Initialize 7SEG with 0.0?C (BMP mode)
	update_7SEG_temperature(0);

	uint16_t ms_counter = 0;        // for 1 s intervals
	uint8_t SEG_second_counter = 0; // for 2 s LCD update period

	while (1)
	{
		// base tick: 10 ms
		_delay_ms(10);
		ms_counter += 10;

		// button toggle: cycle through 3 display modes
		if (button_was_pressed())
		{
			SEG_mode = (SEG_mode + 1) % 3;

			if (SEG_mode == 0)
			update_7SEG_temperature(temp_c_x100);
			else if (SEG_mode == 1)
			update_7SEG_pressure(press_hpa);
			else
			update_7SEG_sonda_temperature(sonda_temp_c_x100);
			
			update_mode_leds(SEG_mode);
		}

		// every ~1 s: read sensors + print via UART
		if (ms_counter >= 1000)
		{
			ms_counter = 0;

			// BMP280 read
			bmp280_read_raw(&adc_T, &adc_P);
			temp_c_x100 = bmp280_compensate_T(adc_T);
			press_pa    = bmp280_compensate_P(adc_P);
			press_hpa   = press_pa / 100U;

			// DS18B20 read
			sonda_temp_c_x100 = ds18b20_get_temperature_c_x100();

			// ---- UART frame for LabVIEW (single-line CSV) ----
			uart_print_int32(temp_c_x100);
			uart_putc(',');
			uart_print_uint32(press_hpa);
			uart_putc(',');
			uart_print_int32(sonda_temp_c_x100);
			uart_putc(',');
			uart_print_uint32(SEG_mode);
			uart_puts("\r\n");

			// 7SEG update every ~2 s
			SEG_second_counter++;
			if (SEG_second_counter >= 2)
			{
				SEG_second_counter = 0;

				if (SEG_mode == 0)
				update_7SEG_temperature(temp_c_x100);
				else if (SEG_mode == 1)
				update_7SEG_pressure(press_hpa);
				else
				update_7SEG_sonda_temperature(sonda_temp_c_x100);
				update_mode_leds(SEG_mode);
			}
		}
	}
}
