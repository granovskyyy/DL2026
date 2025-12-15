#include "uart.h"
void uart_init(uint32_t baud)
{
	uint16_t ubrr = (F_CPU / (16UL * baud)) - 1;
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)ubrr;
	UCSR0A = 0;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

void uart_putc(char c)
{
	while (!(UCSR0A & (1 << UDRE0))) {;}
	UDR0 = c;
}

void uart_puts(const char *s)
{
	while (*s)
	{
		if (*s == '\n')
		uart_putc('\r');  // CRLF
		uart_putc(*s++);
	}
}

void uart_print_uint32(uint32_t v)
{
	char buf[11];
	uint8_t i = 0;

	if (v == 0)
	{
		uart_putc('0');
		return;
	}

	while (v > 0 && i < sizeof(buf))
	{
		uint8_t digit = v % 10;
		buf[i++] = '0' + digit;
		v /= 10;
	}

	while (i > 0)
	{
		uart_putc(buf[--i]);
	}
}
