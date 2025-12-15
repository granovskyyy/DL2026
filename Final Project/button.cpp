#include "button.h"
void button_init(void)
{
	DDRB &= ~(1 << PB7);  // input
	PORTB |= (1 << PB7);  // pull-up
}

uint8_t button_was_pressed(void)
{
	static uint8_t prev_raw = 1;
	static uint8_t debounced = 1;
	static uint8_t stable_cnt = 0;

	uint8_t raw = (PINB & (1 << PB7)) ? 1 : 0;
	uint8_t pressed_event = 0;

	if (raw == prev_raw)
	{
		if (stable_cnt < 255) stable_cnt++;
	}
	else
	{
		stable_cnt = 0;
	}

	if (stable_cnt >= 5)   // ~5 * 10ms = 50ms debounce
	{
		if (raw != debounced)
		{
			debounced = raw;
			if (debounced == 0)   // active LOW -> pressed
			pressed_event = 1;
		}
	}

	prev_raw = raw;
	return pressed_event;
}

