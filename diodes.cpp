#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define rx PD0
#define tx PD1
#define I2C1 PC4
#define I2C2 PC5
#define SPI PC1


int main(void)
{
	DDRD |= (1 << rx) | (1 << tx);
	DDRC |= (1 << I2C1) | (1 << I2C2) | (1 << SPI);


    while (1) 
    {
		PORTD|= (1 << rx) |  (1 << tx);
		PORTC|= (1 << I2C1) | (1 << I2C2) | (1 << SPI);
		
		_delay_ms(500);
		
		PORTD &= ~((1 << rx) |  (1 << tx));
		PORTC &= ~((1 << I2C1) | (1 << I2C2) | (1 << SPI));
		
		_delay_ms(500);
    }
}

