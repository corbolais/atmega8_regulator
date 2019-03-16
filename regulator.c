/*
Compile: 
 avr-gcc -mmcu=atmega8 -Os regulator.c -o regulator.o
 avr-objcopy -j .text -j .data -O ihex  regulator.o  regulator.hex
Upload:
 sudo uisp -dprog=dapa -dlpt=0x378 --erase
 sudo uisp -dprog=dapa -dlpt=0x378 --upload if=regulator.hex 
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#define F_CPU 16000000UL
#include <util/delay.h>

/* Temperature value from sensor in dC' (i.e. 20'C == 200) */
uint16_t temp_val = 0;

/* Temperature average in cC' (i.e. 20'C == 2000) */
uint16_t temp_avr = 2000;

/* Time keeping */
uint16_t seconds = 0;
uint16_t get_seconds(void);

/* Regulate to get 17.5 'C */
#define TEMP_AVR_GOAL 1750
uint16_t temp_avr_goal = TEMP_AVR_GOAL;

/* Min allowed temp to configure 8 'C */
#define MIN_TEMP_GOAL 800
#define MAX_TEMP_GOAL 2200
/* Max allowed temp to configure 22.9 'C */
#define MAX_MAX_TEMP_GOAL (MAX_TEMP_GOAL + 90)

uint8_t need_eeprom_update = 0;

/* Regulation cycle = 180 sec
   must be multiplication of 3 and less than 255 (regulate() and calc_temp_avr() broke otherwise) */
#define CYCLE 180

/* Button on PB1 - active low, need pull-up */
#define BUTTON_GET	((PINB & (1 << PB1)) == 0)

void init_button(void)
{
	DDRB &= ~(1 << PB1);
	PORTB |= (1 << PB1); // Pull-up
}

/* Generate about 8 ms interrupt for LED display control */
void init_timer2(void)
{
	/* 1024 prescaler, clock = 1024 / 16MHz = 64 us */
	TCCR2 = (1 << CS22) | (1 << CS21) | (1 << CS20);

	/* Clear Timer on Compare mode */
	TCCR2 &= ~(1 << WGM20);
	TCCR2 |= (1 << WGM21);

	/* Clear timer */
	TCNT2 = 0;
 
	/* Generate interrupt every 120 cycles ~= 7.6 ms */
	OCR2 = 120;

	/* Enable timer interrupts */
	TIFR |= (1 << OCF2);
	TIMSK |= (1 << OCIE2);
}

/*
	LED display map:
	D2  D3  D4  A   B   C   D   E   F   G   H
	PD3 PD4 PD5 PD1 PD0 PC5 PC4 PD6 PC2 PC1 PC0
*/

#define SEG_A_ON	(PORTD &= ~(1 << PD1))
#define SEG_A_OFF	(PORTD |= (1 << PD1)) 
#define SEG_B_ON	(PORTD &= ~(1 << PD0))
#define SEG_B_OFF	(PORTD |= (1 << PD0))
#define SEG_C_ON	(PORTC &= ~(1 << PC5))
#define SEG_C_OFF	(PORTC |= (1 << PC5)) 
#define SEG_D_ON	(PORTC &= ~(1 << PC4))
#define SEG_D_OFF	(PORTC |= (1 << PC4))
#define SEG_E_ON	(PORTD &= ~(1 << PD6))
#define SEG_E_OFF	(PORTD |= (1 << PD6)) 
#define SEG_F_ON	(PORTC &= ~(1 << PC2))
#define SEG_F_OFF	(PORTC |= (1 << PC2))
#define SEG_G_ON	(PORTC &= ~(1 << PC1))
#define SEG_G_OFF	(PORTC |= (1 << PC1)) 
#define SEG_H_ON	(PORTC &= ~(1 << PC0))
#define SEG_H_OFF	(PORTC |= (1 << PC0))

#define DIGIT_0_ON	(PORTD &= ~(1 << PD5))
#define DIGIT_0_OFF	(PORTD |= (1 << PD5)) 
#define DIGIT_1_ON	(PORTD &= ~(1 << PD4))
#define DIGIT_1_OFF	(PORTD |= (1 << PD4))
#define DIGIT_2_ON	(PORTD &= ~(1 << PD3))
#define DIGIT_2_OFF	(PORTD |= (1 << PD3))

void init_led_display(void)
{
	/* Set ports as output */
	DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC4) | (1 << PC5);
	DDRD |= (1 << PD0) | (1 << PD1) | (1 << PD3) | (1 << PD4) | (1<< PD5) | (1 << PD6);

	/* Turn off by default */
	DIGIT_0_OFF;
	DIGIT_1_OFF;
	DIGIT_2_OFF;

	SEG_A_OFF;
	SEG_B_OFF;
	SEG_C_OFF;
	SEG_D_OFF;
	SEG_E_OFF;
	SEG_F_OFF;
	SEG_G_OFF;
	SEG_H_OFF;
}

uint8_t print_digit(uint8_t x)
{
	switch (x) {
	case 0:
		SEG_A_ON;
		SEG_B_ON;
		SEG_C_ON;
		SEG_D_ON;
		SEG_E_ON;
		SEG_F_ON;
		SEG_G_OFF;
	break;
	case 1:
		SEG_A_OFF;
		SEG_B_ON;
		SEG_C_ON;
		SEG_D_OFF;
		SEG_E_OFF;
		SEG_F_OFF;
		SEG_G_OFF;
	break;
	case 2:
		SEG_A_ON;
		SEG_B_ON;
		SEG_C_OFF;
		SEG_D_ON;
		SEG_E_ON;
		SEG_F_OFF;
		SEG_G_ON;
	break;
	case 3:
		SEG_A_ON;
		SEG_B_ON;
		SEG_C_ON;
		SEG_D_ON;
		SEG_E_OFF;
		SEG_F_OFF;
		SEG_G_ON;
	break;
	case 4:
		SEG_A_OFF;
		SEG_B_ON;
		SEG_C_ON;
		SEG_D_OFF;
		SEG_E_OFF;
		SEG_F_ON;
		SEG_G_ON;
	break;
	case 5:
		SEG_A_ON;
		SEG_B_OFF;
		SEG_C_ON;
		SEG_D_ON;
		SEG_E_OFF;
		SEG_F_ON;
		SEG_G_ON;
	break;
	case 6:
		SEG_A_ON;
		SEG_B_OFF;
		SEG_C_ON;
		SEG_D_ON;
		SEG_E_ON;
		SEG_F_ON;
		SEG_G_ON;
	break;
	case 7:
		SEG_A_ON;
		SEG_B_ON;
		SEG_C_ON;
		SEG_D_OFF;
		SEG_E_OFF;
		SEG_F_OFF;
		SEG_G_OFF;
	break;
	case 8:
		SEG_A_ON;
		SEG_B_ON;
		SEG_C_ON;
		SEG_D_ON;
		SEG_E_ON;
		SEG_F_ON;
		SEG_G_ON;
	break;
	case 9:
		SEG_A_ON;
		SEG_B_ON;
		SEG_C_ON;
		SEG_D_ON;
		SEG_E_OFF;
		SEG_F_ON;
		SEG_G_ON;
	break;
	default:
		SEG_A_OFF;
		SEG_B_OFF;
		SEG_C_OFF;
		SEG_D_OFF;
		SEG_E_OFF;
		SEG_F_OFF;
		SEG_G_OFF;
	break;
	}
}

void print_number(uint16_t val, uint8_t blink)
{
	static uint8_t digit_no = 0;
	static uint8_t count = 0;
	uint8_t digit;

	/* This called every 8ms, 30*8ms ~ 0.25s */

	if (blink) {
		count++;
		/* 0.25s OFF - 0.5s ON */
		if (count < 30) {
			/* Blink fraction digit */
			if ((blink & 1) && digit_no == 0)
				goto out;
			/* Blink integer digits */
			if ((blink & 2) && digit_no != 0)
				goto out;
		} else if (count > 90) {
			count = 0;
		}
	}

	switch (digit_no) {
	case 0:
		digit = val % 10;
		DIGIT_0_ON;
		break;
	case 1:
		digit = (val / 10) % 10;
		DIGIT_1_ON;
		SEG_H_ON;
		break;
	case 2:
		digit = val / 100;
		DIGIT_2_ON;
		break;
	}

	print_digit(digit);

	/* Changing this value regulate brightness */
	_delay_us(100);

	switch (digit_no) {
	case 0:
		DIGIT_0_OFF;
		break;
	case 1:
		DIGIT_1_OFF;
		SEG_H_OFF;
		break;
	case 2:
		DIGIT_2_OFF;
		break;
	}

out:
	digit_no++;
	if (digit_no > 2)
		digit_no = 0;
}

typedef enum { MODE_PRINT_TEMP, MODE_PRINT_GOAL, MODE_CONFIGURE } ModeEnum;
typedef enum { INCREASE_FRACTION = 1, INCREASE_INTEGER = 2} IncEnum;
uint16_t temp_avr_goal_new;

void change_new_goal(IncEnum inc)
{
	uint8_t rem, div;

	div = temp_avr_goal_new / 100; 
	rem = temp_avr_goal_new % 100;

	switch (inc) {
	default:
	case INCREASE_FRACTION:
		/* Decimal part between 0 and 9 'dC */
		rem += 10;
		if (rem > 90)
			rem = 0;
	break;
	case INCREASE_INTEGER:
		/* Interger part between 8 and 22 'C */
		div++;
		if (div > (MAX_TEMP_GOAL/100))
			div = (MIN_TEMP_GOAL/100);
	break;
	}

	temp_avr_goal_new = ((uint16_t) div) * 100 + rem;
}

ISR(TIMER2_COMP_vect)
{
	/* This called every 8ms, 30*8ms ~ 0.25s */
 	static ModeEnum mode = MODE_PRINT_TEMP;
	static IncEnum inc = INCREASE_FRACTION;
	static uint8_t count = 0;
	static uint8_t press = 0;

	uint8_t button;
	uint8_t blink;
	uint16_t val;
 
	button = BUTTON_GET;

	switch (mode) {
	default:
	case MODE_PRINT_TEMP:
		if (button) {
			count++;
			if (count > 5) {
				count = 0;
				mode = MODE_PRINT_GOAL;
			}
		} else {
			count = 0;
		}

		val = temp_val;
		/* Show blinking goal during init or updating EEPROM */
		if (val == 0 || need_eeprom_update) {
			blink = 3;
			val = temp_avr_goal / 10;
		} else {
			blink = 0;
		}
	break;
	case MODE_PRINT_GOAL:
		if (button) {
			count++;
			if (count > 250) { // 2 seconds
				count = 0;
				press = 0;
				inc = INCREASE_FRACTION;
				temp_avr_goal_new = temp_avr_goal;
				mode = MODE_CONFIGURE;
			}
		} else {
			mode = MODE_PRINT_TEMP;
			count = 0;
		}

		val = temp_avr_goal / 10;
		blink = 0;
	break;
	case MODE_CONFIGURE:
		if (button) {
			if (press == 1) {
				change_new_goal(inc);
				count = 0;
			}
			press = 0;
		} else {
			count++;
			if (count > 250) {
				count = 0;
				if (inc == INCREASE_INTEGER) {
					temp_avr_goal = temp_avr_goal_new;
					need_eeprom_update = 1;
					mode = MODE_PRINT_TEMP;
				} else {
					inc = INCREASE_INTEGER;
				}
			}
			press = 1;
		}
		val = temp_avr_goal_new / 10;
		blink = inc;
	break;
	}

	print_number(val, blink);
}

/* Generate one second interrupt for time keeping */
void init_timer1(void)
{
	/* 1024 prescaler, clock = 1024 / 16MHz = 64 us */
	TCCR1B &= ~(1 << CS12);
	TCCR1B = (1 << CS12) | (1 << CS10);

	/* Clear Timer on Compare (CTC) mode */
	TCCR1A &= ~((1 << WGM11) | (1 << WGM10));
	TCCR1B &= ~(1 << WGM13);
	TCCR1B |= (1 << WGM12);

	/* Generate interrupt every 15625 (125*125) cycles = 1 second */
	OCR1A = 0x3d09;

	/* Clear timer */
	TCNT1 = 0;

	/* Enable timer interrupts */
	TIFR |= (1 << OCF1A);
	TIMSK |= (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{
	seconds++;
}

uint16_t get_seconds(void)
{
	uint16_t sec;

	cli();
	sec = seconds;
	sei();

	return sec;
}

/* One Wire sensor interface on pin PB0 */
#define OW_GET_IN	(PINB & (1 << PB0))
#define OW_OUT_LOW	(PORTB &= ~(1 << PB0))
#define OW_OUT_HIGH	(PORTB |= (1 << PB0))
#define OW_DIR_IN	(DDRB &= ~(1 << PB0))
#define OW_DIR_OUT	(DDRB |= (1 << PB0))

uint8_t ow_reset(void)
{
	uint8_t ret;

	OW_OUT_LOW;
	OW_DIR_OUT;
	_delay_us(480);

	cli();
	OW_DIR_IN;
	_delay_us(64);
	ret = OW_GET_IN;
	sei();

	_delay_us(480 - 64);

	if (ret != 0 || OW_GET_IN == 0)
		return 1;
	else
		return 0;
}

#define OW_DELAY_OFF 7

void ow_wr_bit(uint8_t bit)
{
	cli();
	OW_OUT_LOW;
	OW_DIR_OUT;
	_delay_us(15 - OW_DELAY_OFF);
	if (bit)
		OW_DIR_IN;
	_delay_us(60 - 15 + OW_DELAY_OFF);
	OW_DIR_IN;
	sei();

	/* Recovery */
	_delay_us(1);
}

uint8_t ow_rd_bit(void)
{
	uint8_t bit;

	cli();
	OW_OUT_LOW;
	OW_DIR_OUT;
	_delay_us(2);
	OW_DIR_IN;
	_delay_us(15 - 2 - OW_DELAY_OFF);
	bit = OW_GET_IN;
	_delay_us(60 - 15 + OW_DELAY_OFF);
	sei();

	/* Recovery */
	_delay_us(1);

	return bit;
}

void ow_write(uint8_t byte)
{
	uint8_t i;

	for (i = 0; i < 8; i++) {
		ow_wr_bit(byte & 1);
		byte >>= 1;
	}
}

uint8_t ow_read(void)
{
	uint8_t byte = 0;
	uint8_t bit, i;

	for (i = 0; i < 8; i++) {
		bit = ow_rd_bit();
		byte >>= 1;
		if (bit)
			byte |= 0x80;
	}

	return byte;
}

#define OW_CMD_READ_ROM		0x33
#define OW_CMD_MATCH_ROM	0x55
#define OW_CMD_READ_SCRATCH	0xbe
#define OW_CMD_CONVERT_T	0x44

void ow_command(uint8_t cmd, uint8_t *data, uint8_t len, uint8_t rd)
{
	int i;

	ow_write(cmd);
	for (i = 0; i < len; i++) {
		if (rd)
			data[i] = ow_read();
		else
			ow_write(data[i]);
	}
}

/* Broken RELAY - connects 2 from 3 outputs only if disabled, otherwise all outputs are separated */
#define SWITCH_ON	(PORTD &= ~(1 << PD7))
#define SWITCH_OFF	(PORTD |= (1 << PD7))

uint8_t switch_on;
uint8_t switch_is_on;

/* Time of last relay switch */
uint16_t last_switch = 0;
#define SWITCH_DELAY 10

void init_switch(void)
{
	DDRD |= (1 << PD7);
	_delay_ms(10);

	SWITCH_ON; // broken RELAY is ON by default, even on POWER OFF
	switch_on = 1;
	switch_is_on = 1;
	_delay_ms(10);

	/* We did not make RELAY switch here - allow to switch instantly */
	last_switch = get_seconds() - SWITCH_DELAY;
}

void change_switch(uint16_t sec)
{
	/* Don't switch more often than SWITCH_DELAY seconds */
	if (sec - last_switch < SWITCH_DELAY) 
		return;

	if (switch_on == switch_is_on)
		return;

	if (switch_on)
		SWITCH_ON;
	else
		SWITCH_OFF;

	switch_is_on = switch_on;
	last_switch = sec;
}

/* TODO: moving average */
void calc_temp_avr(uint16_t sec)
{
	static uint8_t n = 0;
	static int32_t temp_avr_sum = 0;
	int16_t temp = temp_val * 10;
 
	if (n == 0) { // Init
		temp_avr = temp;
		temp_avr_sum = temp;
		n = 1;
		return;
	}

	if ((sec % CYCLE) == 0) {
		if (n != 0)
			temp_avr = temp_avr_sum / n;
		else
			temp_avr = temp; // Not happen

		temp_avr_sum = temp;
		n = 1;
	} else {
		if (n < 255) {
			temp_avr_sum += temp;
			n++;
		}
	}
}

void regulate(uint16_t sec)
{
	uint8_t on;
	uint8_t sec_in_cycle = sec % CYCLE;
	uint16_t goal;

	/* Race with MODE_CONFIGURE on timer2 */
	cli();
	goal = temp_avr_goal;
	sei();

	/* Output 0%   if temp <= GOAL
	   Output 33%  if temp > GOAL
	   Output 66%  if temp > GOAL + 0.15 'C
	   Output 100% if temp > GOAL + 0.30 'C
	*/
	if (temp_avr > goal + 30)
		on = 1;
	else if (temp_avr > goal + 15)
		on = (sec_in_cycle < 2*CYCLE/3) ? 1 : 0; 
	else if (temp_avr > goal)
		on = (sec_in_cycle < 1*CYCLE/3) ? 1 : 0;
	else
		on = 0;

	switch_on = on;
}

void init_eeprom(void)
{
 	uint16_t goal;

	eeprom_busy_wait(); // Not needed ?

	goal = eeprom_read_word((uint16_t *) 0);
	if (goal >= MIN_TEMP_GOAL && goal <= MAX_MAX_TEMP_GOAL) {
		cli();
		temp_avr_goal = goal;
		sei();
	}
}

void update_eeprom(void)
{
	uint16_t goal;

	if (need_eeprom_update && eeprom_is_ready()) {
		cli();
		goal = temp_avr_goal;
		sei();

		eeprom_write_word((uint16_t *) 0, goal);

		need_eeprom_update = 0;
	}
}

void main(void)
{
	static uint8_t ds18b20_rom[8];
	static uint8_t scratch[9];

	uint8_t i;
	uint16_t sec, val;
	uint16_t done_second = 0;

	ow_reset();
	ow_command(OW_CMD_READ_ROM, ds18b20_rom, 8, 1);

	init_led_display();
	init_timer1();
	init_timer2();
	sei();
	init_switch();
	init_button();
	init_eeprom();

	/* Print goal for short period after power ON */
	done_second = get_seconds() + 2;
	while ((sec = get_seconds()) != done_second)
		;
	done_second = sec;

	while (1) {
		ow_reset();
		ow_command(OW_CMD_MATCH_ROM, ds18b20_rom, 8, 0);
		ow_write(OW_CMD_CONVERT_T);

		/* 750 ms delay for conversion */
		for (i = 0; i < 75; i++)
			_delay_ms(10);

		ow_reset();
		ow_command(OW_CMD_MATCH_ROM, ds18b20_rom, 8, 0);
		ow_command(OW_CMD_READ_SCRATCH, scratch, 9, 1);

		val = ((int16_t)scratch[1] << 4) + (scratch[0] >> 4);
		val *= 10;
		val += (10 * (scratch[0] & 0xf)) / 16;

		cli(); // Race with Timer2 (LED Display) interrupt
		temp_val = val;
		sei();

		update_eeprom(); // if needed

		/* Wait till 1 second from last iteration */
		while ((sec = get_seconds()) == done_second)
			;
		done_second = sec;

		calc_temp_avr(sec);
		regulate(sec);
		change_switch(sec);
	}
}
