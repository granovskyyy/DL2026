#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

// ---------- UART (USART0) ----------

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

// ---------- I2C (TWI0) low level ----------

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

// ---------- BMP280 driver (minimal) ----------

#define BMP280_I2C_ADDR 0x76  // if it doesn't work, try 0x77

// Registers
#define BMP280_REG_CHIPID      0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
#define BMP280_REG_CALIB_START 0x88

typedef struct
{
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
} bmp280_calib_t;

bmp280_calib_t calib;
int32_t t_fine;

void bmp280_read_calibration(void)
{
	uint8_t data[24];
	i2c_read_multi(BMP280_I2C_ADDR, BMP280_REG_CALIB_START, data, 24);

	calib.dig_T1 = (uint16_t)(data[1] << 8 | data[0]);
	calib.dig_T2 = (int16_t)(data[3] << 8 | data[2]);
	calib.dig_T3 = (int16_t)(data[5] << 8 | data[4]);
	calib.dig_P1 = (uint16_t)(data[7] << 8 | data[6]);
	calib.dig_P2 = (int16_t)(data[9] << 8 | data[8]);
	calib.dig_P3 = (int16_t)(data[11] << 8 | data[10]);
	calib.dig_P4 = (int16_t)(data[13] << 8 | data[12]);
	calib.dig_P5 = (int16_t)(data[15] << 8 | data[14]);
	calib.dig_P6 = (int16_t)(data[17] << 8 | data[16]);
	calib.dig_P7 = (int16_t)(data[19] << 8 | data[18]);
	calib.dig_P8 = (int16_t)(data[21] << 8 | data[20]);
	calib.dig_P9 = (int16_t)(data[23] << 8 | data[22]);
}

// temperature in 0.01 °C
int32_t bmp280_compensate_T(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) *
	((int32_t)calib.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)calib.dig_T1)) *
	((adc_T >> 4) - ((int32_t)calib.dig_T1))) >> 12) *
	((int32_t)calib.dig_T3)) >> 14;

	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;   // 0.01 °C
	return T;
}

// pressure in Pa (unsigned)
uint32_t bmp280_compensate_P(int32_t adc_P)
{
	double var1, var2, p;

	var1 = ((double)t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)calib.dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)calib.dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)calib.dig_P4) * 65536.0);
	var1 = (((double)calib.dig_P3) * var1 * var1 / 524288.0 +
	((double)calib.dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * (double)calib.dig_P1;

	if (var1 == 0.0)
	return 0;

	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)calib.dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)calib.dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)calib.dig_P7)) / 16.0;

	if (p < 0.0)
	p = 0.0;

	return (uint32_t)(p + 0.5);
}

void bmp280_init(void)
{
	// Soft reset
	i2c_write_reg(BMP280_I2C_ADDR, BMP280_REG_RESET, 0xB6);
	_delay_ms(5);

	// Read chip ID
	uint8_t id = i2c_read_reg(BMP280_I2C_ADDR, BMP280_REG_CHIPID);
	uart_puts("BMP280 ID = 0x");
	const char hex[] = "0123456789ABCDEF";
	uart_putc(hex[(id >> 4) & 0x0F]);
	uart_putc(hex[id & 0x0F]);
	uart_puts("\r\n");

	// Read calibration
	bmp280_read_calibration();

	// ctrl_meas: temp x1, press x1, normal mode => 0x27
	i2c_write_reg(BMP280_I2C_ADDR, BMP280_REG_CTRL_MEAS, 0x27);
	// config: standby 500 ms, filter off => 0xA0
	i2c_write_reg(BMP280_I2C_ADDR, BMP280_REG_CONFIG, 0xA0);
}

void bmp280_read_raw(int32_t *adc_T, int32_t *adc_P)
{
	uint8_t buf[6];
	i2c_read_multi(BMP280_I2C_ADDR, BMP280_REG_PRESS_MSB, buf, 6);

	int32_t adc_P_local = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[2] >> 4);
	int32_t adc_T_local = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4);

	*adc_P = adc_P_local;
	*adc_T = adc_T_local;
}

// ===== LCD / 7-seg display code (START) =====
// Wiring:
//   a -> PB0
//   b -> PB1
//   c -> PC0
//   d -> PC1
//   e -> PC2
//   f -> PC3
//   g -> PD2
//   DP -> PD3
//   Digit0..3 (common anodes, active LOW) -> PE0..PE3

// digit_map[n] = bits g f e d c b a
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

#define BLANK_DIGIT  0xFF  // special value for "no segments"

// what to show right now
volatile uint8_t display_digits[4] = {BLANK_DIGIT, BLANK_DIGIT, BLANK_DIGIT, BLANK_DIGIT};
// per-digit DP mask: bit n = 1 -> DP on for digit n
volatile uint8_t display_dp_mask = 0;

// which digit is currently active (for ISR multiplex)
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

// *** LCD UPDATE FUNCTIONS (this is what you asked to mark) ***
// BMP280 temperature layout:
// digit0 = blank
// digit1 = tens
// digit2 = ones  <-- DP ON here
// digit3 = decimal digit (0.1°C)
void update_lcd_temperature(int32_t temp_c_x100)
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

	// DP only on digit index 2
	display_dp_mask = (1 << 2);
}

// BMP280 pressure in hPa, integer, 4 digits, no DP
void update_lcd_pressure(uint32_t press_hpa)
{
	if (press_hpa > 9999) press_hpa = 9999;

	display_digits[0] = (press_hpa / 1000) % 10;
	display_digits[1] = (press_hpa / 100)  % 10;
	display_digits[2] = (press_hpa / 10)   % 10;
	display_digits[3] = (press_hpa % 10);

	display_dp_mask = 0;
}

// *** NEW: DS18B20 temperature layout (3rd mode) ***
// Required format: same as BMP temp but
//   digit0 = 0
//   digit1 = tens
//   digit2 = ones (DP ON here)
//   digit3 = decimal digit (0.1°C)
void update_lcd_sonda_temperature(int32_t temp_c_x100)
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

// ===== LCD / 7-seg display code (END) =====

// ---------- User button (PB7) ----------

void button_init(void)
{
	DDRB &= ~(1 << PB7);  // input
	PORTB |= (1 << PB7);  // pull-up
}

// simple polled debounce with edge detection (called every ~10 ms)
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

// ---------- DS18B20 on PD4 (1-Wire) ----------

#define ONEWIRE_PORT PORTD
#define ONEWIRE_DDR  DDRD
#define ONEWIRE_PINR PIND
#define ONEWIRE_BIT  PD4

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

	// we don't care about the rest scratchpad bytes here
	for (uint8_t i = 0; i < 7; i++)
	(void)onewire_read_byte();

	int16_t raw = (int16_t)((msb << 8) | lsb); // 1/16 °C
	int32_t temp_c_x100 = ((int32_t)raw * 100) / 16;

	return temp_c_x100;
}

// ---------- main ----------

int main(void)
{
	uart_init(9600);
	i2c_init();
	display_io_init();
	display_timer_init();
	button_init();
	ds18b20_io_init();

	sei(); // enable global interrupts (for display timer)

	uart_puts("\r\nBMP280 + DS18B20 temp+pressure with 7-seg display\r\n");
	bmp280_init();

	int32_t adc_T = 0, adc_P = 0;
	int32_t temp_c_x100 = 0;
	uint32_t press_pa = 0, press_hpa = 0;

	int32_t sonda_temp_c_x100 = 0;

	// lcd_mode:
	// 0 -> BMP280 temperature
	// 1 -> BMP280 pressure
	// 2 -> DS18B20 temperature (sonda)
	uint8_t lcd_mode = 0;

	// Initialize LCD with 0.0°C (BMP mode)
	update_lcd_temperature(0);

	uint16_t ms_counter = 0;        // for 1 s intervals
	uint8_t lcd_second_counter = 0; // for 2 s LCD update period

	while (1)
	{
		// base tick: 10 ms
		_delay_ms(10);
		ms_counter += 10;

		// button toggle: cycle through 3 display modes
		if (button_was_pressed())
		{
			lcd_mode = (lcd_mode + 1) % 3;

			if (lcd_mode == 0)
			update_lcd_temperature(temp_c_x100);
			else if (lcd_mode == 1)
			update_lcd_pressure(press_hpa);
			else
			update_lcd_sonda_temperature(sonda_temp_c_x100);
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

			// ----- Serial output -----
			// Line 1: BMP280
			uart_puts("T = ");
			int32_t t_print = temp_c_x100;
			if (t_print < 0)
			{
				uart_putc('-');
				t_print = -t_print;
			}
			uart_print_uint32((uint32_t)(t_print / 100));
			uart_putc('.');
			uint8_t frac = (uint8_t)(t_print % 100);
			if (frac < 10) uart_putc('0');
			uart_print_uint32(frac);
			uart_puts(" C, P = ");
			uart_print_uint32(press_hpa);
			uart_puts(" hPa\r\n");

			// Line 2: DS18B20
			uart_puts("Temp. from sonda : ");
			int32_t s_print = sonda_temp_c_x100;
			if (s_print < 0)
			{
				uart_putc('-');
				s_print = -s_print;
			}
			uart_print_uint32((uint32_t)(s_print / 100));
			uart_putc('.');
			uint8_t s_frac = (uint8_t)((s_print / 10) % 10);  // 1 decimal place
			uart_print_uint32(s_frac);
			uart_puts(" C\r\n\r\n");

			// LCD update every ~2 s
			lcd_second_counter++;
			if (lcd_second_counter >= 2)
			{
				lcd_second_counter = 0;

				if (lcd_mode == 0)
				update_lcd_temperature(temp_c_x100);
				else if (lcd_mode == 1)
				update_lcd_pressure(press_hpa);
				else
				update_lcd_sonda_temperature(sonda_temp_c_x100);
			}
		}
	}
}