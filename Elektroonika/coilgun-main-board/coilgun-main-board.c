/*
 * coilgun_main_board.c
 *
 * Created: 17-Oct-15 16:01:02
 * Author: Jürgen Laks
 * 
 * TODO:
 *   [ ]	implement DISCHARGE
 *   [ ]	implement CHARGE
 *   [ ]	implement AUTOCHARGE
 *   [ ]	implement KICK
 *   [ ]	implement FALILSAFE
 *   [ ]	implement BALL DETECTION
 *   [ ]	implement BUTTON DETECTION
 *   [ ]	implement DRIBBLER SPEED
 *   [?]	implement SET ID
 *   [?]	implement GET ID
 *   [?]	implement PING
 *   [?]	implement LEDs
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "usb_serial.h"
#include "nrf24.h"

//defines
#define USB_ENABLED /*Uncomment this to use RS485 interface*/
#define BOARD_ID (uint8_t*)0

//pin definitions
#define PIN_LED_1_R 1
#define PIN_LED_1_G 0
#define PIN_LED_1_B 4
#define PIN_LED_2_R 6
#define PIN_LED_2_G 5
#define PIN_LED_2_B 7
#define PIN_MOTOR_1 5
#define PIN_MOTOR_2 6
#define PIN_COILGUN_KICK 5 /*PORTD*/
#define PIN_COILGUN_CHARGE 4 /*PORTD*/
#define PIN_COILGUN_PWM 6 /*PORTC*/
#define PIN_COILGUN_DONE 4 /*PORTB*/

//macros
#define COILGUN_CHARGE_ON PORTD |= (1 << PIN_COILGUN_CHARGE)
#define COILGUN_CHARGE_OFF PORTD &= ~(1 << PIN_COILGUN_CHARGE)

//function prototypes
int atoi(const char * str);
void usb_write(const char *str);
uint8_t recv_str(char *buf, uint8_t size);
void parse_and_execute_command(char *buf, _Bool answer_to_usb);
void init_ports_and_pins();
void init_timers();
void setLED(_Bool led, _Bool r, _Bool g, _Bool b);
void setLEDs(unsigned char color);
void delay_ms(unsigned short time);
void uart_init(void);
void uart_print();
void discharge();

//global variables
char response[64];
uint8_t fail_counter = 0;
char uart_buf[32];
_Bool uart_data_available = 0;
_Bool failsafe = 1;
_Bool autocharge = 1;
_Bool do_discharge_on_fail = 1;
_Bool coilgun_state = 0;
uint8_t ball_in_dribbler_counter = 0;
uint8_t button_states[3];
uint8_t sensorboard_led_states = 0;
_Bool kicking = 0;

//variables used for NRF24L01+ module
uint8_t data_array[10];
uint8_t tx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t no_data_from_nrf_count = 0;

/*********************************
*   __  __           _           *
*  |  \/  |   __ _  (_)  _ __    *
*  | |\/| |  / _` | | | | '_ \   *
*  | |  | | | (_| | | | | | | |  *
*  |_|  |_|  \__,_| |_| |_| |_|  *
*                                *
*********************************/

int main(void){
	CLKPR = 0x80;
	CLKPR = 0x00;
	
	init_ports_and_pins();
	init_timers();
	
	#ifdef USB_ENABLED
		//init USB
		usb_init();
		//while (!usb_configured());
		_delay_ms(1000);
	#endif
	
	uart_init();
	
	sei();
	uint8_t n;
	char buf[16];
	
	//send welcome message
	sprintf(response, "<connected_id:%d>\n", eeprom_read_byte(BOARD_ID));
	usb_write(response);
	uart_print(response);
	
	uint16_t counter = 0;
	uint16_t counter2 = 0;
	
	//initialize the NRF24L01+ module
	nrf24_init();
	nrf24_config(2, 10);//RF_CH 127 => 2.400128GHz, 10 byte packets
	nrf24_tx_address(tx_address);
	nrf24_rx_address(rx_address);
	
	int16_t m1, m2, m3, m4;
	m1 = m2 = m3 = m4 = 0;
	
    while(1){
		if(uart_data_available){
			uart_data_available = 0;
			//sprintf(response, "<sain:%s>\n", uart_buf);
			//uart_print(response);
			parse_and_execute_command(uart_buf, 0);
		}
		if( !(PINB & (1 << PIN_COILGUN_DONE)) ){
			//coilgun has finished charging
			PORTD &= ~(1 << PIN_COILGUN_CHARGE);
			if(!coilgun_state){
				usb_write("CHARGED!");
				uart_print("C\n");
				coilgun_state = 1;
			}
		}else{
			coilgun_state = 1;
		}
		if(usb_serial_available()) {
			n = recv_str(buf, sizeof(buf));
			setLED(1, 1, 0, 0);
			if (n == sizeof(buf)) {
				setLED(1, 0, 0, 1);
				parse_and_execute_command(buf, 1);
			}
		}
		
		no_data_from_nrf_count++;
		if(no_data_from_nrf_count>200) 
			no_data_from_nrf_count = 200;
			
		//setLEDs(nrf24_dataReady());
		
		while(nrf24_dataReady()){
			fail_counter = 0;
			do_discharge_on_fail = 1;
			no_data_from_nrf_count = 0;
			
			nrf24_getData(data_array);
			//int16_t R = data_array[2]-127;//roll
			//int16_t P = data_array[3]-127;//pitch
			//int16_t T = data_array[0]-127;//throttle
			//int16_t Y = data_array[1]-127;//yaw
			setLEDs(data_array[4]);//aux1
			//controls[5] = (data_array[6]&(1<<0))?255:0;//arming
			//controls[6] = (data_array[6]&(1<<1))?255:0;//flight-mode
			//controls[7] = (data_array[6]&(1<<2))?255:0;//airmode
			
			if(data_array[6] & (1 << 0)){
				if(!kicking){
					//pole veel löönud, löön
					COILGUN_CHARGE_OFF;
					_Bool b = autocharge;
					autocharge = 0;
					PORTD |= (1 << PIN_COILGUN_KICK);
					delay_ms(data_array[4]);
					PORTD &= ~(1 << PIN_COILGUN_KICK);
					autocharge = b;
					sprintf(response, "kick_done, %d, f:%d\n", autocharge, data_array[4]);
					usb_write(response);
					if(autocharge)
						COILGUN_CHARGE_ON;
					kicking = 1;
				}else{
					//juba löödud
					sprintf(response, "already kicked\n");
					usb_write(response);
					
				}
			}else{
				//ei taha enam lüüa
				kicking = 0;
				
			}
			
			const int16_t max_motor_power = 190;
			int16_t rotation = data_array[1]-127;
			m1 = (data_array[2] - 127)*max_motor_power/127 + rotation;
			m2 = (127 - data_array[3])*max_motor_power/127 + rotation;
			m3 = (127 - data_array[2])*max_motor_power/127 + rotation;
			m4 = (data_array[3] - 127)*max_motor_power/127 + rotation;
		}
		
		if(counter++ > 100){
			sprintf(response, "1:sd%i\n2:sd%i\n3:sd%i\n4:sd%i\n", m1, m2, m3, m4);
			uart_print(response);
			usb_write(response);
			if(counter2++ % 2 == 0){
				sprintf(response, "3:sd5 %i\n", no_data_from_nrf_count);
			}else{
				sprintf(response, "3:sd-5 %i\n", no_data_from_nrf_count);
			}
			//usb_write(response);
			counter = 0;
		}
        if ((fail_counter >= 50) && failsafe) {
	        fail_counter = 0;
	        sprintf(response, "<failsafe_triggered>\n");
	        usb_write(response);
	        uart_print("F\n");
	        if(do_discharge_on_fail){
		        do_discharge_on_fail = 0;
		        discharge();
	        }
        }
		_delay_ms(1);
    }
}

/*******************************************************************
*   _____                          _     _                         *
*  |  ___|  _   _   _ __     ___  | |_  (_)   ___    _ __    ___   *
*  | |_    | | | | | '_ \   / __| | __| | |  / _ \  | '_ \  / __|  *
*  |  _|   | |_| | | | | | | (__  | |_  | | | (_) | | | | | \__ \  *
*  |_|      \__,_| |_| |_|  \___|  \__| |_|  \___/  |_| |_| |___/  *
*                                                                  *
*******************************************************************/

//Related to initialization
void init_ports_and_pins(){
	//disable JTAG, needed to use some of the pins on the MCU
	MCUCR|= (1<<JTD); //in order to change this value, it is needed to 
	MCUCR|= (1<<JTD); //overwrite this value twice during 4 clock cycles
	   
	DDRF = 0b11110011; //LEDs
	PORTF = 255; //swich LEDs off by default
	
	DDRB |= (1 << PIN_MOTOR_1) | (1 << PIN_MOTOR_2);
	
	//Coilgun
	DDRD |= (1 << PIN_COILGUN_KICK) | (1 << PIN_COILGUN_CHARGE);
	DDRC |= (1 << PIN_COILGUN_PWM);
	
	//UART pins (rs485)
	DDRD |= (1 << 6) | (1 << 7);
	//Enabling receive and transmit (on the rs485 chip)
	PORTD |= (1 << 6);
	PORTD &= ~(1 << 7);
	
}

void init_timers(){
	//init timer 0
	TCCR0A = 0b00000010;
	TCCR0B = 0b00000101; //prescale 1024
	OCR0A = 250;
	TIMSK0 = 0b00000010;
	TCNT0 = 0;
	
	//Setup PWM
	//Configure TIMER1
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);        //NON Inverted PWM
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11)|(1<<CS10); //PRESCALER=64 MODE 14(FAST PWM)

	ICR1=4999;  //fPWM=50Hz (Period = 20ms Standard).

}

//LED control
void setLED(_Bool led, _Bool r, _Bool g, _Bool b){
	if(led){
		if(!r) PORTF |= (1 << PIN_LED_1_R); else PORTF &= ~(1 << PIN_LED_1_R);
		if(!g) PORTF |= (1 << PIN_LED_1_G); else PORTF &= ~(1 << PIN_LED_1_G);
		if(!b) PORTF |= (1 << PIN_LED_1_B); else PORTF &= ~(1 << PIN_LED_1_B);
	}else{
		if(!r) PORTF |= (1 << PIN_LED_2_R); else PORTF &= ~(1 << PIN_LED_2_R);
		if(!g) PORTF |= (1 << PIN_LED_2_G); else PORTF &= ~(1 << PIN_LED_2_G);
		if(!b) PORTF |= (1 << PIN_LED_2_B); else PORTF &= ~(1 << PIN_LED_2_B);
	}
}

void setLEDs(unsigned char color){
	setLED(0, color&1, color&2, color&4);
	setLED(1, color&8, color&16, color&32);
}

void discharge(){
	COILGUN_CHARGE_OFF;
	for(uint8_t i = 1; i > 0; i++){
		PORTD |= (1 << PIN_COILGUN_KICK);
		delay_ms(10);
		PORTD &= ~(1 << PIN_COILGUN_KICK);
		delay_ms(500);
	}
	for(uint8_t i = 1; i > 0; i++){
		PORTD |= (1 << PIN_COILGUN_KICK);
		delay_ms(30);
		PORTD &= ~(1 << PIN_COILGUN_KICK);
		delay_ms(500);
	}
	for(uint8_t i = 1; i > 0; i++){
		PORTD |= (1 << PIN_COILGUN_KICK);
		delay_ms(70);
		PORTD &= ~(1 << PIN_COILGUN_KICK);
		delay_ms(200);
	}
	for(uint8_t i = 1; i > 0; i++){
		PORTD |= (1 << PIN_COILGUN_KICK);
		delay_ms(100);
		PORTD &= ~(1 << PIN_COILGUN_KICK);
		delay_ms(90);
	}
}

//USB related
void usb_write(const char *str) {
	while (*str) {
		usb_serial_putchar(*str);
		str++;
	}
}

uint8_t recv_str(char *buf, uint8_t size) {
	char data;
	uint8_t count = 0;
	
	while (count < size) {
		data = usb_serial_getchar();
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

/** Ootab [time]*10 mikrosekundit ehk 100 on 1 ms (parameeter on kuni 65535) */
void delay_ms(unsigned short time){
	for(unsigned short i = 0; i < time; i++){
		_delay_us(10);
	}
}

//Related to UART
# define USART_BAUDRATE 19200
# define BAUD_PRESCALE ((( F_CPU / ( USART_BAUDRATE * 16UL))) - 1)

// transmit a char to uart
void uart_transmit( unsigned char data ){
	// wait for empty transmit buffer
	while (  !( UCSR1A & (1 << UDRE1))  );
	// put data into buffer, sends data
	UDR1 = data;
}

// init uart
void uart_init(void){
	unsigned int baud = BAUD_PRESCALE;
	UBRR1H = (unsigned char) (baud >> 8 );
	UBRR1L = (unsigned char)baud;
	// enable received and transmitter and interrupt
	UCSR1B = ( 1 << RXEN1 ) | ( 1 << TXEN1 ) | (1 << RXCIE1);
	UCSR1C = (3 << UCSZ10); //8 dta bits, 1 stop bit
}

// check if there are any chars to be read
int uart_dataAvailable(void){
	if ( UCSR1A & ( 1 << RXC1) )
	return 1;
	return 0;
}

// write a string to the uart
void uart_print( char data[] ){
	int c = 0;
	for ( c = 0; c < strlen(data); c++ )
		uart_transmit(data[c]);
}

/************************************************************************************************************************
*    ____                                                       _       ____                         _                  *
*   / ___|   ___    _ __ ___    _ __ ___     __ _   _ __     __| |     |  _ \    __ _   _ __   ___  (_)  _ __     __ _  *
*  | |      / _ \  | '_ ` _ \  | '_ ` _ \   / _` | | '_ \   / _` |     | |_) |  / _` | | '__| / __| | | | '_ \   / _` | *
*  | |___  | (_) | | | | | | | | | | | | | | (_| | | | | | | (_| |     |  __/  | (_| | | |    \__ \ | | | | | | | (_| | *
*   \____|  \___/  |_| |_| |_| |_| |_| |_|  \__,_| |_| |_|  \__,_|     |_|      \__,_| |_|    |___/ |_| |_| |_|  \__, | *
*                                                                                                                |___/  *
************************************************************************************************************************/

void parse_and_execute_command(char *buf, _Bool answer_to_usb) {
	uint8_t id = eeprom_read_byte(BOARD_ID);
	char *command;
	int16_t par1;
	command = buf;
	
	//Vaatan üle globaalsed käsud (ID pole tarvis)
	if (   (command[0] == 'i') && (command[1] == 'd')   ){
		//peaks sättima plaadi ID
		par1 = atoi(command+2); //id
		eeprom_update_byte(BOARD_ID, par1);
		sprintf(response, "<id_changed_from_%d_to_%d>\n", id, par1);
		if(answer_to_usb) usb_write(response); else uart_print(response);
		return;
	} else if (command[0] == '?') {
		//tagastan ID
		sprintf(response, "<connected:%d>\n", id);
		if(answer_to_usb) usb_write(response); else uart_print(response);
		return;
	}else if(command[0] == 't'){
		setLEDs(32);
		//transfer message to other communication endpoint
		sprintf(response, "%s\n", command+1);
		if(!answer_to_usb) usb_write(response); else uart_print(response);
		sprintf(response, "sent!\n");
		if(answer_to_usb) usb_write(response); else uart_print(response);
		return;
	}else if(!isdigit(command[0])){
		sprintf(response, "%d:unknown:%s\n", id, command);
		//sprintf(response, "%s", command);
		if(answer_to_usb) usb_write(response); else uart_print(response);
		return;
	}
	
	//Parsin välja päris käsu (eemaldan ID)
	par1 = atoi(command);
	if(par1 != id){
		return;
	}else{
		while (*command != ':') command++;
		command++;
	}
	
	//uurin ja täidan käske
	if (command[0] == 'g'  &&  command[1] == 'b') {
		//report if ball is in dribbler
		sprintf(response, "<%d:ball:%d>\n", id, !!(PINE & (1 << 6)));
		if(answer_to_usb) usb_write(response); else uart_print(response);
	}else if (command[0] == 'l'  &&  command[1] == 'e'  &&  command[2] == 'd') {
		//set LED state
		par1 = atoi(command+3);
		setLEDs(par1);
	}else if (command[0] == 's'  &&  command[1] == 'b'  &&  command[2] == 'l') {
		//set LED state on sensorboard
		par1 = atoi(command+3);
		sensorboard_led_states = par1 & 0b11;
	}else if (command[0] == 'p'  &&  command[1] == 'i'  &&  command[2] == 'n'  &&  command[3] == 'g') {
		sprintf(response, "<%d:pong>\n", id);
		if(answer_to_usb) usb_write(response); else uart_print(response);
	}else if (command[0] == 'p') {
		//ping command with no answer, also restarts failsafe counter
		fail_counter = 0;
	}else if (command[0] == 'm'  &&  command[1] == '1') {
		par1 = atoi(command+2);
		OCR1A = par1;
	}else if (command[0] == 'm'  &&  command[1] == '2') {
		par1 = atoi(command+2);
		OCR1B = par1;
	}else if (command[0] == 'f'  &&  command[1] == 's') {
		par1 = atoi(command+2);
		failsafe = par1;
	}else if(command[0] == 's'){
		//make a sound
		uint16_t d = atoi(command + 1);
		for(uint8_t i = 1; i > 0; i++){
			PORTD |= (1 << PIN_COILGUN_KICK);
			delay_ms(d);
			PORTD &= ~(1 << PIN_COILGUN_KICK);
			delay_ms(d);
		}
	}else if (command[0] == 'd'  &&  command[1] == '1') {
		//brushelss mootori start
		par1 = atoi(command+2);
		OCR1A = 500;
		_delay_ms(2000);
		OCR1A = 200;
		_delay_ms(2000);
		OCR1A = 500;
		sprintf(response, "<%d:running>\n", id);
		if(answer_to_usb) usb_write(response); else uart_print("R\n");
	}else if (command[0] == 'a'  &&  command[1] == 'c') {
		autocharge = atoi(command+2);
	}else if (command[0] == 'k') {
		COILGUN_CHARGE_OFF;
		_Bool b = autocharge;
		autocharge = 0;
		par1 = atoi(command+1);
		PORTD |= (1 << PIN_COILGUN_KICK);
		delay_ms(par1);
		PORTD &= ~(1 << PIN_COILGUN_KICK);
		autocharge = b;
		sprintf(response, "kick_done, %d\n", autocharge);
		if(answer_to_usb) usb_write(response); else uart_print("K\n");
		if(autocharge)
		COILGUN_CHARGE_ON;
	}else if (command[0] == 'd') {
		//discharge
		discharge();
		sprintf(response, "discharged, %d\n", autocharge);
		if(answer_to_usb) usb_write(response); else uart_print("D\n");
	}else if (command[0] == 'c') {
		//charge
		COILGUN_CHARGE_ON;
		do_discharge_on_fail = 1;
		sprintf(response, "Charging(ac:%d)\n", autocharge);
		if(answer_to_usb) usb_write(response); else uart_print("c\n");
	}else{
		sprintf(response, "%d:unknown:%s\n", id, command);
		//sprintf(response, "%s", command);
		if(answer_to_usb) usb_write(response); else uart_print(response);
	}
}

/************************************************************************
*   ___           _                                          _          *
*  |_ _|  _ __   | |_    ___   _ __   _ __   _   _   _ __   | |_   ___  *
*   | |  | '_ \  | __|  / _ \ | '__| | '__| | | | | | '_ \  | __| / __| *
*   | |  | | | | | |_  |  __/ | |    | |    | |_| | | |_) | | |_  \__ \ *
*  |___| |_| |_|  \__|  \___| |_|    |_|     \__,_| | .__/   \__| |___/ *
*                                                   |_|                 *
************************************************************************/

ISR(TIMER0_COMPA_vect) {
	fail_counter++;
}