#include "7SEG.h"
//   a -> PB0
//   b -> PB1
//   c -> PC0
//   d -> PC1
//   e -> PC2
//   f -> PC3
//   g -> PD2
//   DP -> PD3
//   Digit0..3 (common anodes, active LOW) -> PE0..PE3

const uint8_t digit_map[10] = {
	0b00111111, // 0
	0b00000110, // 1
	0b01011011, // 2
	0b01001111, // 3
	0b01100110, // 4
	0b01101101, // 5
	0b01111101, // 6
	0b00000111, // 7
	0b01111111, // 8
	0b01101111  // 9
};


volatile uint8_t display_digits[4] = {BLANK_DIGIT, BLANK_DIGIT, BLANK_DIGIT, BLANK_DIGIT};

volatile uint8_t display_dp_mask = 0;

volatile uint8_t current_digit = 0;

void display_io_init(void)
{
	// segments: PB0, PB1
	DDRB |= (1 << PB0) | (1 << PB1);
	// segments: PC0..PC3
	DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
	// segments: PD2 (g) + PD3 (DP)
	DDRD |= (1 << PD2) | (1 << PD3);
	// digit selects: PE0..PE3
	DDRE |= (1 << PE0) | (1 << PE1) | (1 << PE2) | (1 << PE3);

	// common anode: OFF = 1 on segments, OFF = 1 on digit selects
	PORTB |= (1 << PB0) | (1 << PB1);
	PORTC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
	PORTD |= (1 << PD2) | (1 << PD3);
	PORTE |= (1 << PE0) | (1 << PE1) | (1 << PE2) | (1 << PE3);
}

// set segments (g..a in bits 6..0), DP separately
void set_segments(uint8_t seg, uint8_t dp_on)
{
	// a -> PB0 (bit0)
	if (seg & (1 << 0)) PORTB &= ~(1 << PB0);   // ON = 0
	else                PORTB |=  (1 << PB0);   // OFF = 1

	// b -> PB1 (bit1)
	if (seg & (1 << 1)) PORTB &= ~(1 << PB1);
	else                PORTB |=  (1 << PB1);

	// c -> PC0 (bit2)
	if (seg & (1 << 2)) PORTC &= ~(1 << PC0);
	else                PORTC |=  (1 << PC0);

	// d -> PC1 (bit3)
	if (seg & (1 << 3)) PORTC &= ~(1 << PC1);
	else                PORTC |=  (1 << PC1);

	// e -> PC2 (bit4)
	if (seg & (1 << 4)) PORTC &= ~(1 << PC2);
	else                PORTC |=  (1 << PC2);

	// f -> PC3 (bit5)
	if (seg & (1 << 5)) PORTC &= ~(1 << PC3);
	else                PORTC |=  (1 << PC3);

	// g -> PD2 (bit6)
	if (seg & (1 << 6)) PORTD &= ~(1 << PD2);
	else                PORTD |=  (1 << PD2);

	// DP -> PD3
	if (dp_on) PORTD &= ~(1 << PD3);   // ON = 0
	else       PORTD |=  (1 << PD3);   // OFF = 1
}

// Timer0 ISR: multiplex display, runs at 1 kHz
ISR(TIMER0_COMPA_vect)
{
	// turn all digits OFF (anodes = 1)
	PORTE |= (1 << PE0) | (1 << PE1) | (1 << PE2) | (1 << PE3);

	uint8_t value = display_digits[current_digit];
	uint8_t dp_on = (display_dp_mask & (1 << current_digit)) ? 1 : 0;

	uint8_t seg = 0;
	if (value <= 9)
	seg = digit_map[value];
	else
	seg = 0; // BLANK_DIGIT => all segments off (DP may still be on)

	set_segments(seg, dp_on);

	// enable chosen digit (anode = 0)
	switch (current_digit)
	{
		case 0: PORTE &= ~(1 << PE0); break;
		case 1: PORTE &= ~(1 << PE1); break;
		case 2: PORTE &= ~(1 << PE2); break;
		case 3: PORTE &= ~(1 << PE3); break;
		default: break;
	}

	current_digit++;
	if (current_digit > 3) current_digit = 0;
}

// init Timer0 to generate 1ms interrupts for display refresh
void display_timer_init(void)
{
	// CTC mode
	TCCR0A = (1 << WGM01);
	// 16 MHz / 64 = 250 kHz; 250 kHz / 250 = 1 kHz -> 1 ms
	OCR0A = 249;
	TCCR0B = (1 << CS01) | (1 << CS00); // prescaler 64
	TIMSK0 = (1 << OCIE0A);             // enable compare match A interrupt
}

void update_7SEG_temperature(int32_t temp_c_x100)
{
	if (temp_c_x100 < 0) temp_c_x100 = -temp_c_x100;

	// convert to 0.1°C resolution: 23.4°C -> 234
	int32_t temp_x10 = temp_c_x100 / 10;   // 0.1°C units
	if (temp_x10 > 999) temp_x10 = 999;    // limit to 2-digit int part

	uint16_t t = (uint16_t)temp_x10;
	uint8_t frac = t % 10;                 // decimal digit
	uint8_t integer = t / 10;              // 0..99

	uint8_t tens = integer / 10;           // 0..9
	uint8_t ones = integer % 10;

	display_digits[0] = BLANK_DIGIT;  // leftmost off
	display_digits[1] = tens;
	display_digits[2] = ones;         // DP ON here
	display_digits[3] = frac;

	display_dp_mask = (1 << 2);
}


void update_7SEG_pressure(uint32_t press_hpa)
{
	if (press_hpa > 9999) press_hpa = 9999;

	display_digits[0] = (press_hpa / 1000) % 10;
	display_digits[1] = (press_hpa / 100)  % 10;
	display_digits[2] = (press_hpa / 10)   % 10;
	display_digits[3] = (press_hpa % 10);

	display_dp_mask = 0;
}

void update_7SEG_sonda_temperature(int32_t temp_c_x100)
{
	if (temp_c_x100 < 0) temp_c_x100 = -temp_c_x100;

	int32_t temp_x10 = temp_c_x100 / 10;   // 0.1°C units
	if (temp_x10 > 999) temp_x10 = 999;

	uint16_t t = (uint16_t)temp_x10;
	uint8_t frac = t % 10;
	uint8_t integer = t / 10;              // 0..99

	uint8_t tens = integer / 10;
	uint8_t ones = integer % 10;

	display_digits[0] = 0;        // '0' on the first digit (index 0)
	display_digits[1] = tens;
	display_digits[2] = ones;     // DP ON here
	display_digits[3] = frac;

	display_dp_mask = (1 << 2);   // DP only on digit index 2
}

