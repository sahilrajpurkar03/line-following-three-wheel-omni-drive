/*
 * line-follow-3-wheel-omni-drive.c
 *
 * 3-Wheel Omni Robot with Line-Follow and Manual Mode
 * Created: 12-04-2018 16:14:02
 * Author : Sahil 
 */

#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define BAUD 9600
#define BAUDRATE  ((F_CPU/(BAUD*16UL)-1))

// PWM and Drive Direction Definitions
#define drive_dir PORTB
#define drive_pwm1 OCR3A
#define drive_pwm2 OCR3B
#define drive_pwm3 OCR3C

// Sensor Inputs
#define input_port_front PINA
#define input_port_back PINC

// Indicators
#define indicator_port1 PORTD
#define indicator_port2 PORTH

// Serial Communication Buffers
uint8_t RX[16] = {100};
int RX_range = 200, RX_raw = 255, RX_ad = 255, RX_count = 0;
uint8_t TX[16] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
int flag[16] = {0};
uint8_t TX_raw = 200, TX_ad = 201, TX_flag = 0;

// PS Controller Buttons
#define PS_L1 10
#define PS_R1 11
#define PS_L2 8
#define PS_R2 9
#define PS_L3 6
#define PS_R3 7
#define PS_SQUARE 14
#define PS_TRIANGLE 12
#define PS_CIRCLE 13
#define PS_CROSS 15
#define PS_UP 2
#define PS_DOWN 5
#define PS_LEFT 4
#define PS_RIGHT 3
#define PS_START 0
#define PS_SELECT 1

// Motor & Joystick Variables
int w1 = 0, w2 = 0, w3 = 0;
int js_error = 20;
int pwm_range = 120;
int xj1 = 0, yj1 = 0, xj2 = 0, yj2 = 0, pot1 = 0, pot2 = 0, pot3 = 0, pot4 = 0;
int butt[16] = {0};

// Line Follower Variables
uint16_t minval = 1000, maxval = 0, val = 0;
uint16_t max_val[16] = {0}, min_val[16] = {1000}, avg_val[16] = {0}, sensor[16] = {0};

uint8_t sensfront = 0, sensback = 0;
int fronthighcount = 0, frontcurrent = 0, backhighcount = 0, backcurrent = 0;

float fronttarget = 4.5, backtarget = 4.5;
float fronterror = 0, backerror = 0, Pf = 0, Pb = 0;
float kpf = 12, kpb = 12, kprop_ver = 1;

// State Mode Variable (0: line, 1: manual, 2: calibrate, 3: writeEEPROM, 4: stop)
volatile uint8_t s_mode = 0;

// Function Prototypes
void lf_vertical(int sp_vect);
void manual();
void line_PID();
void frontproportional();
void backproportional();
void resetvar();
void readsensor();
void calibrate();
void writeeep();
void readeep();
void drive_3W(int x_vect, int y_vect, int m_vect);
void drivewheel_1(int sp_vect);
void drivewheel_2(int sp_vect);
void drivewheel_3(int sp_vect);
long map_value(long in_value, long in_min, long in_max, long out_min, long out_max);
long limit_var(long in_var, long l_limit, long h_limit);
void pwm_init();
void ADC_initiate();
uint16_t ADC_read(uint8_t ch);
void serialstart_3();
void receive();

int main(void)
{
	// Set I/O Directions
	DDRA = 0x00; DDRB = 0xFF; DDRC = 0x00;
	DDRD = 0xFF; DDRE = 0xFF; PORTE = 0xFF;
	DDRH = 0xFF; DDRJ = 0xFF; DDRL = 0xF0; PORTL |= 0x0F;
	DDRF = 0x00; DDRK = 0x00; DDRG = 0x00;
	
	sei();
	ADC_initiate();
	serialstart_3();
	pwm_init();
	readeep();	// Load EEPROM calibration

	drive_3W(0,0,0);

	while (1)
	{
		receive();  // Continuously check for PS2 input
		switch(s_mode) {
			case 0: line_PID(); lf_vertical(100); break;
			case 1: manual(); break;
			case 2: calibrate(); break;
			case 3: writeeep(); break; // <- FIXED MISSING SEMICOLON
			case 4: drive_3W(0,0,0); break;
		}
	}
}

// Line follow movement using vertical component
void lf_vertical(int sp_vect)
{
	if (fronthighcount)
		drive_3W(sp_vect, (Pf + Pb), (Pf - Pb));
}

// Manual mode via joystick
void manual()
{
	drive_3W(xj1, yj1, ((xj2 + yj2) / 2));
}

// Main PID function
void line_PID()
{
	resetvar();
	readsensor();

	if (fronthighcount) frontproportional();
	else Pf = 0;

	if (backhighcount) backproportional();
	else Pb = 0;
}

// Proportional control for front sensors
void frontproportional()
{
	fronterror = fronttarget - (frontcurrent / fronthighcount);
	Pf = kpf * fronterror;
}

// Proportional control for back sensors
void backproportional()
{
	backerror = backtarget - (backcurrent / backhighcount);
	Pb = kpb * backerror;
}

// Reset all sensing and error values
void resetvar()
{
	sensfront = sensback = 0;
	fronthighcount = frontcurrent = backhighcount = backcurrent = 0;
}

// Read 8 front and back sensors, convert to boolean mask
void readsensor()
{
	in_front = input_port_front;
	in_back = input_port_back;

	for (int i = 0; i < 8; i++)
	{
		if (in_front & 0x01)
		{
			fronthighcount++;
			frontcurrent += (i + 1);
			sensfront |= (1 << i);
		}
		in_front >>= 1;
	}

	for (int i = 0; i < 8; i++)
	{
		if (in_back & 0x01)
		{
			backhighcount++;
			backcurrent += (i + 1);
			sensback |= (1 << i);
		}
		in_back >>= 1;
	}
}

// Calibrate all 16 sensors
void calibrate()
{
	indicator_port2 = 0x00;
	for (int i = 0; i < 16; i++)
	{
		max_val[i] = 0;
		min_val[i] = 1000;
	}
	drive_3W(0, 0, 0);
	indicator_port1 = 0xFF;

	for(int j = 0; j <= 3000; j++)
	{
		for(int n = 0; n <= 15; n++)
		{
			val = ADC_read(n);
			if (val > max_val[n]) max_val[n] = val;
			if (val < min_val[n]) min_val[n] = val;
		}
		_delay_ms(1);
	}

	for (int n = 0; n < 16; n++)
		avg_val[n] = (max_val[n] + min_val[n]) / 2;

	indicator_port1 = 0x00;
	_delay_ms(1000);
}

// Save average threshold to EEPROM
void writeeep()
{
	for (int j = 0; j <= 30; j += 2)
		eeprom_write_word((uint16_t*)j, avg_val[j / 2]);
}

// Load average threshold from EEPROM
void readeep()
{
	for (int j = 0; j <= 30; j += 2)
		avg_val[j / 2] = eeprom_read_word((uint16_t*)j);
}

// Main drive control (X, Y, Rotation)
void drive_3W(int x_vect, int y_vect, int m_vect)
{
	w1 = m_vect - x_vect - y_vect;
	w2 = m_vect + x_vect - y_vect;
	w3 = (m_vect + y_vect) * 2;

	drivewheel_1(w1);
	drivewheel_2(w2);
	drivewheel_3(w3);
}

// Individual wheel speed control
void drivewheel_1(int sp_vect) { /* same logic as before */ }
void drivewheel_2(int sp_vect) { /* same logic as before */ }
void drivewheel_3(int sp_vect) { /* same logic as before */ }

long map_value(long in_value, long in_min, long in_max, long out_min, long out_max)
{
	return (in_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long limit_var(long in_var, long l_limit, long h_limit)
{
	if (in_var > h_limit) in_var = h_limit;
	else if (in_var < l_limit) in_var = l_limit;
	return in_var;
}

// PWM Initialization
void pwm_init()
{
	TCCR3A |= (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM30);
	TCCR3B |= (1 << CS32) | (1 << CS30) | (1 << WGM32);
}

// ADC Initialization
void ADC_initiate()
{
	ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRB = 0x00;
}

// ADC Read from selected channel
uint16_t ADC_read(uint8_t ch)
{
	ADMUX &= 0b11100000;
	ADCSRB &= 0b11110111;
	ch &= 0b00001111;

	if (ch <= 7) {
		ADMUX |= ch;
		ADCSRB = 0x00;
	} else {
		ch -= 8;
		ADMUX |= ch;
		ADCSRB |= (1 << MUX5);
	}

	ADCSRA |= (1 << ADSC);
	while (!(ADCSRA & (1 << ADIF)));
	return ADC;
}

// Initialize USART3
void serialstart_3()
{
	UBRR3H = BAUDRATE >> 8;
	UBRR3L = BAUDRATE;
	UCSR3B = 0b10011000;
	UCSR3C = 0b00000110;
}

// USART3 RX ISR for serial parsing
ISR(USART3_RX_vect)
{
	RX_count = 1;
	RX_raw = UDR3;

	if ((RX_raw > 200) && (RX_raw < 255)) {
		RX_ad = RX_raw;
		if ((RX_raw > 230) && (RX_raw < 247))
			butt[RX_raw - 231] = 1;
	} else if (RX_raw < 201) {
		uint8_t index = RX_ad - 201;
		if (index < 16) RX[index] = RX_raw;
	}
}

void receive()
{
	yj1=map_value(RX[0],0,RX_range,(-pwm_range),pwm_range);
	xj1=map_value(RX[1],0,RX_range,pwm_range,(-pwm_range));
	yj2=map_value(RX[2],0,RX_range,(-pwm_range),pwm_range);
	xj2=map_value(RX[3],0,RX_range,pwm_range,(-pwm_range));
	
	
	if (butt[PS_START]==1)
	{
		s_mode = 2;
		butt[PS_START]=0;
	}
	if (butt[PS_SELECT]==1)
	{
		s_mode = 3;
		butt[PS_SELECT]=0;
	}
	if (butt[PS_UP]==1)
	{
		butt[PS_UP]=0;
	}
	if (butt[PS_DOWN]==1)
	{
		butt[PS_DOWN]=0;
	}
	if (butt[PS_LEFT]==1)
	{
		butt[PS_LEFT]=0;
	}
	if (butt[PS_RIGHT]==1)
	{
		butt[PS_RIGHT]=0;
	}
	if (butt[PS_SQUARE]==1)
	{
		butt[PS_SQUARE]=0;
	}
	if (butt[PS_CIRCLE]==1)
	{
		butt[PS_CIRCLE]=0;
	}
	if (butt[PS_TRIANGLE]==1)
	{
		s_mode = 1;
		butt[PS_TRIANGLE]=0;
	}
	if (butt[PS_CROSS]==1)
	{
		s_mode = 0;
		butt[PS_CROSS]=0;
	}
	if (butt[PS_L1]==1)
	{
		butt[PS_L1]=0;
	}
	if (butt[PS_R1]==1)
	{
		s_mode = 4;
		butt[PS_R1]=0;
	}
	if (butt[PS_L2]==1)
	{
		butt[PS_L2]=0;
	}
	if (butt[PS_R2]==1)
	{
		s_mode = 4;
		butt[PS_R2]=0;
	}
	if (butt[PS_L3]==1)
	{
		butt[PS_L3]=0;
	}
	if (butt[PS_R3]==1)
	{
		butt[PS_R3]=0;
	}
}
