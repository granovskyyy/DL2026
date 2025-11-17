#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

// mapowanie cyfr 0-9 na segmenty (wspólna anoda)
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

// ustawienie segmentów na podstawie mapy
void set_segments(uint8_t seg) {
	if (seg & (1<<0)) PORTB &= ~(1<<PB0); else PORTB |= (1<<PB0);
	if (seg & (1<<1)) PORTB &= ~(1<<PB1); else PORTB |= (1<<PB1);
	if (seg & (1<<2)) PORTC &= ~(1<<PC0); else PORTC |= (1<<PC0);
	if (seg & (1<<3)) PORTC &= ~(1<<PC1); else PORTC |= (1<<PC1);
	if (seg & (1<<4)) PORTC &= ~(1<<PC2); else PORTC |= (1<<PC2);
	if (seg & (1<<5)) PORTC &= ~(1<<PC3); else PORTC |= (1<<PC3);
	if (seg & (1<<6)) PORTD &= ~(1<<PD2); else PORTD |= (1<<PD2);
}

// wyświetlenie cyfry na konkretnej pozycji (0..3)
void show_digit(uint8_t position, uint8_t value) {
	PORTE |= (1<<PE0)|(1<<PE1)|(1<<PE2)|(1<<PE3); // wyłącz wszystkie cyfry
	set_segments(digit_map[value]);
	switch (position) {
		case 0: PORTE &= ~(1<<PE0); break;
		case 1: PORTE &= ~(1<<PE1); break;
		case 2: PORTE &= ~(1<<PE2); break;
		case 3: PORTE &= ~(1<<PE3); break;
	}
	_delay_ms(3);
}

// inicjalizacja UART 9600 bps
void uart_init(void) {
	uint16_t ubrr = F_CPU/16/9600-1;
	UBRR0H = (ubrr >> 8);
	UBRR0L = ubrr & 0xFF;
	UCSR0B = (1<<TXEN0)|(1<<RXEN0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
}

// odbiór pojedynczego znaku
char uart_receive_char(void) {
	while (!(UCSR0A & (1<<RXC1)));
	return UDR0;
}

void uart_send_char(char c) {
	while (!(UCSR0A & (1<<UDRE0))); // czekaj aż bufor wolny
	UDR0 = c;
}



int main(void) {
	// konfiguracja wyjść dla segmentów i wybór cyfr
	DDRB |= (1<<PB0)|(1<<PB1);
	DDRC |= (1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3);
	DDRD |= (1<<PD2);
	DDRE |= (1<<PE0)|(1<<PE1)|(1<<PE2)|(1<<PE3);

	uart_init();

	uint8_t digits[4] = {0, 5, 3, 1}; // początkowo 1234

	while (1) {
		// multipleksowanie wyświetlacza
		for (uint8_t i=0; i<4; i++) show_digit(i, digits[i]);

		// odbiór znaku z UART
		if (UCSR0A & (1<<RXC1)) {
			char c = uart_receive_char();
			if (c >= '0' && c <= '9') {
				uint8_t val = c - '0';
				for (uint8_t i=0; i<4; i++) digits[i] = val; // ustaw na wszystkich cyfrach
				        uart_send_char(c);   // wyślij znak do terminala
						uart_send_char(' '); // nowa linia
			}
		}
	}
}
