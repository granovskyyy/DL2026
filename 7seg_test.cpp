#include <avr/io.h>
#include <util/delay.h>

// digit_map[n] = bity g f e d c b a (1 = segment ma ?wieci? logicznie)
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

// WSPÓLNA ANODA: segment ON = 0, OFF = 1
void set_segments(uint8_t seg)
{
	// a -> PB0 (bit0)
	if (seg & (1<<0)) PORTB &= ~(1<<PB0);   // ON -> 0
	else              PORTB |=  (1<<PB0);   // OFF -> 1

	// b -> PB1 (bit1)
	if (seg & (1<<1)) PORTB &= ~(1<<PB1);
	else              PORTB |=  (1<<PB1);

	// c -> PC0 (bit2)
	if (seg & (1<<2)) PORTC &= ~(1<<PC0);
	else              PORTC |=  (1<<PC0);

	// d -> PC1 (bit3)
	if (seg & (1<<3)) PORTC &= ~(1<<PC1);
	else              PORTC |=  (1<<PC1);

	// e -> PC2 (bit4)
	if (seg & (1<<4)) PORTC &= ~(1<<PC2);
	else              PORTC |=  (1<<PC2);

	// f -> PC3 (bit5)
	if (seg & (1<<5)) PORTC &= ~(1<<PC3);
	else              PORTC |=  (1<<PC3);

	// g -> PD2 (bit6)
	if (seg & (1<<6)) PORTD &= ~(1<<PD2);
	else              PORTD |=  (1<<PD2);
}

// Wy?wietlenie jednej cyfry na danej pozycji (0..3)
void show_digit(uint8_t position, uint8_t value)
{
	// wy??cz wszystkie cyfry (anody w 1 = OFF, je?li aktywne stanem niskim)
	PORTE |= (1<<PE0)|(1<<PE1)|(1<<PE2)|(1<<PE3);

	// ustaw odpowiednie segmenty (wspólna anoda: segment ON = 0)
	set_segments(digit_map[value]);

	// w??cz wybran? cyfr? (anoda w 0 = ON)
	switch (position) {
		case 0: PORTE &= ~(1<<PE0); break; // cyfra tysi?cy
		case 1: PORTE &= ~(1<<PE1); break; // cyfra setek
		case 2: PORTE &= ~(1<<PE2); break; // cyfra dziesi?tek
		case 3: PORTE &= ~(1<<PE3); break; // cyfra jedno?ci
	}

	_delay_ms(3);
}

int main(void)
{
	// segmenty jako wyj?cia
	DDRB |= (1<<PB0)|(1<<PB1);
	DDRC |= (1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3);
	DDRD |= (1<<PD2);

	// wybór cyfr na 7-segu jako wyj?cia
	DDRE |= (1<<PE0)|(1<<PE1)|(1<<PE2)|(1<<PE3);

	// chcemy stale wy?wietla? 1234
	uint8_t digits[4] = {1, 2, 3, 4};

	while (1) {
		// multipleksowanie czterech cyfr
 		for (uint8_t i = 0; i < 4; i++) {
 			show_digit(i, digits[i]);
 		}
	}
}