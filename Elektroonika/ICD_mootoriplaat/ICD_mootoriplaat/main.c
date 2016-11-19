/*
 * wheelmod.c
 * 
 * eeprom
 * 0 - dir
 * 1 - wheel polarity
 * 2 - id
 * 3 - pgain
 * 4 - igain
 * 5 - dgain
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
//#include <string.h>
#include "usb_serial.h"

#define bit_get(p,m) ((p) & (m)) 
#define bit_set(p,m) ((p) |= (m)) 
#define bit_clear(p,m) ((p) &= ~(m)) 
#define bit_flip(p,m) ((p) ^= (m)) 
#define bit_write(c,p,m) (c ? bit_set(p,m) : bit_clear(p,m)) 
#define BIT(x) (0x01 << (x)) 
#define LONGBIT(x) ((unsigned long)0x00000001 << (x))

//pin definitions
#define PWM PC6
#define DIR1 PB6
#define DIR2 PB7
#define FAULT PC7
#define LED1 PC4
#define LED2 PC5
#define ENCA PD0
#define ENCB PD1
#define BALL PB0

int atoi(const char * str);

union doublebyte {
  unsigned int value;
  unsigned char bytes[2];
};

uint8_t enc_dir;
uint8_t enc_last = 0;
uint8_t enc_now;
union doublebyte wcount;
union doublebyte decoder_count;

uint8_t dir;
uint8_t pid_on = 1;
uint8_t motor_polarity = 1;
uint8_t fail_counter = 0;
uint8_t send_speed = 0;
uint8_t failsafe = 1;
uint8_t leds_on = 1;

int16_t sp, sp_pid, der;
int16_t intgrl, prop;
int16_t count, speed;
int16_t csum;
int16_t err, err_prev;
uint8_t pgain, igain, dgain;
int16_t pwm, pwm_min, pwmmin;
int16_t imax, err_max;
uint8_t pid_multi, update_pid;
uint8_t ball = 0;
uint8_t ir_led;
uint16_t stall_counter = 0;
uint16_t stallCount = 0;
uint16_t prevStallCount = 0;
uint16_t stallWarningLimit = 60;
uint16_t stallErrorLimit = 300;
uint8_t stallLevel = 0;
uint8_t stallChanged = 0;
int16_t currentPWM = 0;


char response[16];

void USART_send(unsigned char* data){
  PORTB |= 0b00001100;
  PORTD |= 0b00010000;
  //data = "derp\n\0";
  while(*data){
    while(!( UCSR1A & (1<<UDRE1)));
    UDR1 = *data;
    data++;
  }
  _delay_ms(5);
  PORTB &= 0b11110011;
  PORTD &= 0b11101111;
}

void forward(uint8_t pwm) {
	if (dir) {
		bit_clear(PORTB, BIT(DIR1));
		bit_set(PORTB, BIT(DIR2));
	} else {
		bit_set(PORTB, BIT(DIR1));
		bit_clear(PORTB, BIT(DIR2));
	}
	OCR1AL = pwm;
	currentPWM = pwm;
}

void backward(uint8_t pwm) {
	if (dir) {
		bit_set(PORTB, BIT(DIR1));
		bit_clear(PORTB, BIT(DIR2));
	} else {
		bit_clear(PORTB, BIT(DIR1));
		bit_set(PORTB, BIT(DIR2));
	}
	OCR1AL = pwm;
	currentPWM = -pwm;
}

void pid() {	
	err_prev = err;	
	err = sp_pid - speed;
	
	if (stallLevel != 2) {
	
		intgrl += (err * pid_multi) / igain;
	
		//constrain integral
		if (intgrl < -imax) intgrl = -imax;
		if (intgrl > imax) intgrl = imax;
	
		if (sp == 0) pwmmin = 0;
		else if (sp < 0) pwmmin = -pwm_min;
		else pwmmin = pwm_min;
	
		pwm = pwmmin + err*pgain + intgrl/pid_multi;
		//constrain pwm
		if (pwm < -255) pwm = -255;
		if (pwm > 255) pwm = 255;
		
		prevStallCount = stallCount;
		if ((speed < 5 && currentPWM == 255 || speed > -5 && currentPWM == -255) && stallCount < stallErrorLimit) {
			stallCount++;
		} else if (stallCount > 0) {
			stallCount--;
		}
	
		if (pwm < 0) {
			pwm *= -1;
			backward(pwm);	
		} else {
			forward(pwm);
		}
		
		if ((stallCount == stallWarningLimit - 1) && (prevStallCount == stallWarningLimit)) {
			stallLevel = 0;
			stallChanged = 1;
		} else if ((stallCount == stallWarningLimit) && (prevStallCount == stallWarningLimit - 1)) {
			stallLevel = 1;
			stallChanged = 1;
		} else if (stallCount == stallErrorLimit) {
			stallLevel = 2;
			stallChanged = 1;
			reset_pid();
		}
	} else {
		stallCount--;
		if (stallCount == 0) {
			stallLevel = 0;
			stallChanged = 1;
		}
	} 	
	//OCR1AL = (pwm >= 0) ? pwm : -pwm;
}

void reset_pid() {
	err = 0;
	err_prev = 0;
	intgrl = 0;
	der = 0;
	sp = 0;
	sp_pid = 0;
	forward(0);
}

//TODO implement UART
void usb_write(const char *str) {
    USART_send(str);
    while (*str) { 
        usb_serial_putchar(*str); 
        str++; 
	}
  
}

uint8_t recv_str(char *buf, uint8_t size) {
	char data;
	uint8_t count=0;
	
	while (count < size) {
		data = usb_serial_getchar();
		//usb_serial_putchar(data);
		if (data == '\r' || data == '\n') {
			*buf = '\0';
			return size;
		}
		if (data >= ' ' && data <= '~') {
			*buf++ = data;
			count++;
		}		
	}
	return count;
}

void decode() {
	enc_now = bit_get(PIND, 3);
	enc_dir = (enc_last & 1) ^ ((enc_now & 2) >> 1);
	enc_last = enc_now;
	if (enc_dir) {
		if (motor_polarity) wcount.value--;
		else wcount.value++;		
		if (leds_on) bit_set(PORTC, BIT(LED2));
	}		
	else {
		if (motor_polarity) wcount.value++;
		else wcount.value--;		
		if (leds_on) bit_clear(PORTC, BIT(LED2));
	}
	if ((wcount.value == 0) && leds_on) bit_flip(PORTC, BIT(LED1));
}

void setup() {
	//uint8_t pid_on_current = pid_on;
	pid_on = 0;
	forward(50);
	wcount.value = 0;
	_delay_ms(500);
	if (speed < 0) {
		motor_polarity ^= 1;
		eeprom_update_byte((uint8_t*)1, motor_polarity);
	}		
	//else motor_polarity = dir;
	sp_pid = 0;
	reset_pid();
	forward(0);
	pid_on = 1;
}

ISR(INT0_vect) {
	decode();
}

ISR(INT1_vect) {
	decode();
}

ISR(TIMER0_COMPA_vect) {
	if (leds_on) bit_flip(PORTC, BIT(LED1));
	decoder_count.value = wcount.value;
	speed = wcount.value;
	wcount.value = 0;
	update_pid = 1;
}

ISR(PCINT0_vect) {
	//PB4 - IR LED
	ir_led = bit_get(PINB, BIT(0));
	if (ir_led & 0b00000001) {
		ball = 1;
		if (leds_on) bit_set(PORTC, BIT(LED2));
	}
	else {
		ball = 0;
		if (leds_on) bit_clear(PORTC, BIT(LED2));
	}
}

volatile unsigned char input_buf[16];
volatile uint8_t i = 0;
volatile uint8_t newData = 0;

ISR(USART1_RX_vect){
  unsigned char ch = UDR1;
  
  if(ch >= ' ' && ch <= '~'){
    input_buf[i] = ch;
    i++;
  }
  
  if(ch == '\n' || ch == '\r'){
    input_buf[i] = '\0';
    i = 0;
    newData = 1;
    return;
  }
}

void parse_and_execute_command(char *buf) {
	uint8_t id = eeprom_read_byte((uint8_t*)2);
  char *command;
	int16_t par1;
	command = buf;
  par1 = atoi(command);
  if(par1 != id){
    return;
  }
  else{
    while (*command != ':'){
      command++;
    }
    command++;
  }        
	if ((command[0] == 's') && (command[1] == 'd')) {
		//set motor speed with pid setpoint
		pid_on = 1;
		par1 = atoi(command+2);
		sp_pid = par1;
		if (sp_pid == 0) reset_pid();
		fail_counter = 0;
	} else if ((command[0] == 'w') && (command[1] == 'l')) {
		//set motor speed with pwm
		pid_on = 0;
		par1 = atoi(command+2);
		if (par1 < 0) {
			par1 *= -1;
			backward(par1);
		} else {
			forward(par1);

		}
		fail_counter = 0;
	} else if ((command[0] == 'g') && (command[1] == 'b')) {
		//get ball
    sprintf(response, "<%d:b:%d>\n", id, ball);
		usb_write(response);
	} else if ((command[0] == 'd') && (command[1] == 'r')) {
		//toggle motor direction
		par1 = atoi(command+2);
		if (dir ^ par1) motor_polarity ^= 1;
		dir = par1;
		eeprom_update_byte((uint8_t*)0, dir);
		eeprom_update_byte((uint8_t*)1, motor_polarity);
	} else if ((command[0] == 's') && (command[1] == 't')) {
		//perform setup
		setup();
	} else if ((command[0] == 'm') && (command[1] == 'p')) {
		//toggle motor polarity
		par1 = atoi(command+2);
		motor_polarity = par1;
		eeprom_update_byte((uint8_t*)1, motor_polarity);
	} else if ((command[0] == 'p') && (command[1] == 'd')) {
		//toggle pid
		par1 = atoi(command+2);
		pid_on = par1;
		if (!pid_on) reset_pid();
	} else if ((command[0] == 'g') && (command[1] == 's')) {
		//toggle get speed on every pid cycle
		par1 = atoi(command+2);
		send_speed = par1;
	} else if ((command[0] == 'f') && (command[1] == 's')) {
		//toggle failsafe
		par1 = atoi(command+2);
		failsafe = par1;
	} else if (command[0] == 's') {
		//get speed
		sprintf(response, "<%d:s:%d>\n", id, speed);
		usb_write(response);
	} else if ((command[0] == 'i') && (command[1] == 'd')) {
		//set id
		par1 = atoi(command+2);
		eeprom_update_byte((uint8_t*)2, par1);
	} else if (command[0] == '?') {
		//get info: id
		par1 = eeprom_read_byte((uint8_t*)2);
		sprintf(response, "<%d:id:%d>\n", id, par1);
		usb_write(response);
	} else if ((command[0] == 'p') && (command[1] == 'g')) {
		//set pgain
		par1 = atoi(command+2);
		eeprom_update_byte((uint8_t*)3, par1);
		pgain = par1;
	} else if ((command[0] == 'i') && (command[1] == 'g')) {
		//set igain
		par1 = atoi(command+2);
		eeprom_update_byte((uint8_t*)4, par1);
		igain = par1;
	} else if ((command[0] == 'd') && (command[1] == 'g')) {
		//set dgain
		par1 = atoi(command+2);
		eeprom_update_byte((uint8_t*)5, par1);
		dgain = par1;
	} else if ((command[0] == 'g') && (command[1] == 'g')) {
		//get pid gains
		sprintf(response, "<%d:pid:%d,%d>\n", id, pgain, igain);
		usb_write(response);
	} else if ((command[0] == 't') && (command[1] == 'l')) {
		//toggle leds
		par1 = atoi(command+2);
		leds_on = par1;
		if (leds_on == 0) {
			bit_clear(PORTC, BIT(LED1));
			bit_clear(PORTC, BIT(LED2));
		}			
	} else {
		//bit_flip(PORTC, BIT(LED1));
		sprintf(response, "%d:%s\n", id, command);
		usb_write(response);
	}
}

int main(void) {	
	CLKPR = 0x80;
	CLKPR = 0x00;
	usb_init();
  
  //TODO Implement and test UART init 
  //UART init
  DDRB &= 0b11110010;
  DDRB |= 0b00001100;
  DDRD &= ~(0b00000010);
  DDRD |= 0b00010100;
  //DDRD |= 0b00000010;   
  //19200
  //8
  UBRR1 = 51;
  //enable Tx and Rx
  UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
  UCSR1C = (1<<USBS1)|(3<<UCSZ10);  
	
	dir = eeprom_read_byte((uint8_t*)0);
	motor_polarity = eeprom_read_byte((uint8_t*)1);
	pgain = eeprom_read_byte((uint8_t*)3);
	igain = eeprom_read_byte((uint8_t*)4);
	dgain = eeprom_read_byte((uint8_t*)5);
	
	if (pgain == 255) {
		pgain = 6;
		eeprom_update_byte((uint8_t*)3, pgain);
	}
	if (igain == 255) {
		igain = 8;
		eeprom_update_byte((uint8_t*)4, igain);
	}
	
	bit_set(DDRC, BIT(LED1));	
	bit_set(DDRC, BIT(LED2));
	bit_set(PORTC, BIT(LED1));
	bit_set(PORTC, BIT(LED2));
	
	//Wait for USB to be configured
	//while (!usb_configured()) /* wait */ ;
	_delay_ms(1000);
	
	//PID
	pid_multi = 32;
	imax = 255 * pid_multi;
	err_max = 4000;
	pwm_min = 25;
	//igain = 8;	//divider
	//pgain = 2;
	//dgain = 0;
	intgrl = 0;
	count = 0;
	speed = 0;
	err = 0;
	
	//timer0
	TCCR0A = 0b00000010;
	TCCR0B = 0b00000101; //prescale 1024
	OCR0A = 250;
	TIMSK0 = 0b00000010;
	TCNT0 = 0;
	
	//init PCINT0
	//DDRB &= 0b11110010;
  
  //UART
  //DDRB |= 0b00001100;
   
	PCICR = 1; //enable pin change interrupt
	PCMSK0 = 0b00000001;	
	
	//quadrature decoder, INT0, INT1 interrupt
	bit_clear(DDRD, BIT(0));
	bit_clear(DDRD, BIT(1));
	EICRA = 0b00000101; //both edges
	EIMSK = 0b00000011; //enable mask
	
	//Mootorite PWM
	bit_set(DDRC, BIT(PWM));	
	
	//TCCR1A = 0b10000001; //phase correct
	//TCCR1B = 0b00000011; //prescale 64, freq 490
	TCCR1A = 0b10000001; //fast pwm
	TCCR1B = 0b00001100; //prescale 64, freq 980		
	
	OCR1AL = 0;
	//OCR1BL = 0;
	
	bit_set(DDRB, BIT(DIR1));
	bit_set(DDRB, BIT(DIR2));
	
	sei();
	
	uint8_t n;
	char buf[16];
	
	while (1) {
		if (update_pid) {
			if (pid_on) {
				pid();
				update_pid = 0;
			}
			if (send_speed) {
				sprintf(response, "<s:%d>\n", speed);
				usb_write(response);
			}
			if ((speed < 10) && (pwm > 250)) {
				stall_counter++;
			} else {
				stall_counter = 0;
			}
			fail_counter++;
		}
		
		if ((fail_counter == 100) && failsafe) {
			sp_pid = 0;
			reset_pid();
			forward(0);
		}
		if (stallChanged) {
			sprintf(response, "<stall:%d>\n", stallLevel);
			usb_write(response);
			stallChanged = 0;
		}	
    //TODO implement UART read
		if (usb_serial_available()) {
			n = recv_str(buf, sizeof(buf));
			if (n == sizeof(buf)) {
				parse_and_execute_command(buf);
			}
		}else if (newData == 1){
      parse_and_execute_command(input_buf);
      newData = 0;
    }	
	}
}