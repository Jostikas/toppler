/*
 * coilgun_main_board.c
 *
 * Created: 17-Oct-15 16:01:02
 * Author: Jürgen Laks
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
char uart_buf[32];
_Bool uart_data_available = 0;
uint8_t sensorboard_led_states = 0;
int16_t motor_left_speed_buffer = 10, motor_right_speed_buffer = 10;


//variables used for NRF24L01+ module
uint8_t data_array[10];
//uint8_t tx_address[5]		= {0xE7,0xE7,0xE7,0xE7,0xE7};	//Self
uint8_t tx_address[5]		= {0xD7,0xD7,0xD7,0xD7,0xD7};	//Remote
uint8_t tx_address_l1[5]	= {0xE7,0xE7,0xE7,0xE7,0xE1};	//Lower robot #1
uint8_t tx_address_l2[5]	= {0xE7,0xE7,0xE7,0xE7,0xE2};	//Lower robot #2
uint8_t rx_address[5]		= {0xE7,0xE7,0xE7,0xE7,0xE7};	//Self
uint8_t no_data_from_nrf_count = 0;

uint8_t robot_to_control = 1;
_Bool should_control_lower_robots = 0;
_Bool is_armed = 0;

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
		_delay_ms(100);
	#endif
	
	uart_init();
	
	sei();
	char buf[16];
	
	//send welcome message
	sprintf(response, "<connected_id:%d>\n", eeprom_read_byte(BOARD_ID));
	usb_write(response);
	uart_print(response);
	
	//initialize the NRF24L01+ module
	nrf24_init();
	nrf24_config(2, 10);//RF_CH 127 => 2.400128GHz, 10 byte packets
	nrf24_tx_address(tx_address);
	nrf24_rx_address(rx_address);
	
	setLED(0, 0, 1, 0);
    while(1){
		if(uart_data_available){
			uart_data_available = 0;
			sprintf(response, "<sain:%s>\n", uart_buf);
			usb_write(response);
		}
		if(usb_serial_available()) {
			setLED(1, 1, 0, 0);
			uint16_t n = recv_str(buf, sizeof(buf));
			setLED(1, 0, 1, 0);
			if (n == sizeof(buf)) {
				setLED(1, 0, 0, 1);
				parse_and_execute_command(buf, 1);
				setLED(1, 1, 1, 1);
			
			}
		}
		
		no_data_from_nrf_count++;
		if(no_data_from_nrf_count>200) 
			no_data_from_nrf_count = 200;
			
		while(nrf24_dataReady()){
			nrf24_getData(data_array);
			if(data_array[0] == 0xE1  ||  data_array[0] == 0xE2){
				//Tegemist on maapealse robotiga
				
			}else{
				//tõenäoliselt, kui keegi saadab midagi ja aadress ei klapi on tegemist puldiga
			
				usb_write("Received:\n");
				for(int i = 0; i < 6; i++){
					sprintf(response, "%d. val=%d\n", i, data_array[i]);
					usb_write(response);
				}
				
				
				if(data_array[6] & (1 << 0)){
					//Armed
					is_armed = 1;
					usb_write("Armed\n");
				}else{
					//Not armed
					is_armed = 0;
				}
				
				if(   data_array[7] & (1 << 0)   ){
					//LEDs on
				}else if(   data_array[7] & (1 << 1)   ){
					//LEDs off
				}
				if(   data_array[7] & (1 << 2)   ){
					//FPV on
				}else if(   data_array[7] & (1 << 3)   ){
					//FPV off
				}
				if(   data_array[7] & (1 << 4)   ){
					//Buzzer on
					//Võiks vahetada, kas autode kontrollimisel juhtida üht või teist
					robot_to_control = 1;
				}else if(   data_array[7] & (1 << 5)   ){
					//Buzzer off
					//Võiks vahetada, kas autode kontrollimisel juhtida üht või teist
					robot_to_control = 2;
				}
				if(   data_array[1] < 5   &&  data_array[0] < 5   &&   is_armed){
					//Yaw stick left
					//See võiks tamiili läbi kärsatada
					sprintf(buf, "Tamiil: %d\n", data_array[4]);
					usb_write(buf);
					
					if(robot_to_control == 1){
						//Tahetakse juhtida alumist robotit nr. 1
						nrf24_tx_address(tx_address_l1);
					}else{
						//Tahetakse juhtida alumist robotit nr. 2
						nrf24_tx_address(tx_address_l2);
					}
					
					data_array[0] = 0xE7;	//Sent from main board
					data_array[1] = 0x03;	//Cut the line
					data_array[2] = data_array[4] & 0xFF;
					data_array[3] = 0 >> 8;
					nrf24_send(data_array);
					while(nrf24_isSending());
					uint8_t temp = nrf24_lastMessageStatus();
					uint8_t connection_quality = nrf24_retransmissionCount();
					nrf24_tx_address(tx_address);
					nrf24_powerUpRx();
					if(temp == NRF24_MESSAGE_LOST){
						usb_write("<message:lost>\n");
					}else if(temp == NRF24_TRANSMISSON_OK){
						usb_write("<message:sent>\n");
					}else{
						usb_write("Something is really wrong!!!");
					}
					sprintf(response, "<quality:%d>\n", connection_quality);
					usb_write(response);
				}else{
					//
				}
				if(   data_array[7] & (1 << 7)   ){
					//Not currently used, add something fun here!
				}
				
				if(data_array[6] & (1 << 1)){
					//Flight mode 1
					usb_write("Ulemised\n");
					should_control_lower_robots = 0;
					
					int16_t speed = 0;
					if(data_array[2] < 95){
						speed = data_array[2] - 95;
					}else if(data_array[2] > 105){
						speed = data_array[2] - 105;
					}
					if(!is_armed){
						speed = 0;
					}
					if(robot_to_control == 2){
						int16_t vspeed = 0;
						if(data_array[3] < 95){
							vspeed = data_array[3] - 95;
						}else if(data_array[3] > 105){
							vspeed = data_array[3] - 105;
						}else{
							vspeed = 0;
						}
						if(!is_armed){
							vspeed = 0;
						}
						sprintf(response, "%d:sd%i\n", 3, vspeed);
						uart_print(response);
						usb_write("<sent_to_uart:");
						usb_write(response);
						usb_write(">\n");
					}
					sprintf(response, "%d:sd%i\n", robot_to_control, speed);
					uart_print(response);
					usb_write("<sent_to_uart:");
					usb_write(response);
					usb_write(">\n");
				}else{
					//Flight mode 2
					should_control_lower_robots = 1;
					usb_write("alumised\n");
					
					if(is_armed){
						if(data_array[3] < 95){
							motor_right_speed_buffer = motor_left_speed_buffer = (data_array[2] - 95)*data_array[5]/10; //pmst pitch määrab kiiruse, l-aux määrab kordaja
						}else if(data_array[3] > 105){
							motor_right_speed_buffer = motor_left_speed_buffer = (data_array[2] - 105)*data_array[5]/10; //pmst pitch määrab kiiruse, l-aux määrab kordaja
						}else{
							motor_right_speed_buffer = motor_left_speed_buffer = 0;
						}
					
						if(motor_right_speed_buffer != 0){
							motor_right_speed_buffer *= -1;
					
							motor_right_speed_buffer	+= data_array[3] * data_array[4] / 20;
							motor_left_speed_buffer		+= data_array[3] * data_array[4] / 20; 
						}
					
						if(robot_to_control == 1){
							//Tahetakse juhtida alumist robotit nr. 1
							nrf24_tx_address(tx_address_l1);
						}else{
							//Tahetakse juhtida alumist robotit nr. 2
							nrf24_tx_address(tx_address_l2);
						}
						data_array[0] = 0xE7;	//Sent from main board
						data_array[1] = 0x01;	//Want to change the speed of motors
						data_array[2] = motor_left_speed_buffer & 0xFF;
						data_array[3] = motor_left_speed_buffer >> 8;
						data_array[4] = motor_right_speed_buffer & 0xFF;
						data_array[5] = motor_right_speed_buffer >> 8;
						data_array[6] = 0;		//Don't send anything back
						nrf24_send(data_array);
						while(nrf24_isSending());
						uint8_t temp = nrf24_lastMessageStatus();
						uint8_t connection_quality = nrf24_retransmissionCount();
						nrf24_tx_address(tx_address);
						nrf24_powerUpRx();
						if(temp == NRF24_MESSAGE_LOST){
							usb_write("<message:lost>\n");
						}else if(temp == NRF24_TRANSMISSON_OK){
							usb_write("<message:sent>\n");
						}else{
							usb_write("Something is really wrong!!!");
						}
						sprintf(response, "<quality:%d>\n", connection_quality);
						usb_write(response);
					}
				}		
			}
			
			
		}
		
		_delay_ms(5);
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
	uint8_t linuxfix = 0;
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
		if(++linuxfix == 0){
			return 0xFF;
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
	} else if (command[0] == 'p'  &&  command[1] == 'o'  &&  command[2] == 'n'  &&  command[3] == 'g') {
		//tagastan ID
		if(answer_to_usb) usb_write("pong"); else uart_print("pong");
		return;
	}else if(command[0] == 't'){
		setLEDs(32);
		//transfer message to other communication endpoint
		sprintf(response, "%s\n", command+1);
		if(!answer_to_usb) usb_write(response); else uart_print(response);
		sprintf(response, "<sent!>\n");
		if(answer_to_usb) usb_write(response); else uart_print(response);
		return;
	}else if(command[0] == 'u'){
		//Tahetakse juhtida ülemist robotit
		uint8_t receiver = 0;
		if(command[1] == '1'){
			//Tahetakse juhtida ülemist robotit nr. 1
			receiver = 1;
		}else if(command[1] == '2'){
			//Tahetakse juhtida ülemist robotit nr. 2
			receiver = 2;
		}
		
		if(command[2] == 's'){
			//Tahetakse sättida roboti kiirust
			int16_t speed = atoi(command+3);
			sprintf(response, "%d:sd%i\n", receiver, speed);
			uart_print(response);
			usb_write("<sent_to_uart:");
			usb_write(response);
			usb_write(">\n");
			
		}
	}else if(command[0] == 'l'){
		//Tahetakse juhtida alumist robotit
		
		if(command[2] == 't'){
			//Tahetakse pöörata robotit
			uint16_t time = atoi(command+3);
			if(command[1] == '1'){
				//Tahetakse juhtida alumist robotit nr. 1
				nrf24_tx_address(tx_address_l1);
			}else if(command[1] == '2'){
				//Tahetakse juhtida alumist robotit nr. 2
				nrf24_tx_address(tx_address_l2);
			}
			data_array[0] = 0xE7;	//Sent from main board
			data_array[1] = 0x02;	//turn the robot
			data_array[2] = motor_left_speed_buffer & 0xFF;
			data_array[3] = motor_left_speed_buffer >> 8;
			data_array[4] = motor_right_speed_buffer & 0xFF;
			data_array[5] = motor_right_speed_buffer >> 8;
			data_array[6] = time & 0xFF;
			data_array[7] = time >> 8;
			nrf24_send(data_array);
			while(nrf24_isSending());
			uint8_t temp = nrf24_lastMessageStatus();
			uint8_t connection_quality = nrf24_retransmissionCount();
			nrf24_tx_address(tx_address);
			nrf24_powerUpRx();
			if(temp == NRF24_MESSAGE_LOST){
				usb_write("<message:lost>\n");
			}else if(temp == NRF24_TRANSMISSON_OK){
				usb_write("<message:sent>\n");
			}else{
				usb_write("Something is really wrong!!!");
			}
			sprintf(response, "<quality:%d>\n", connection_quality);
			usb_write(response);
		}else if(command[2] == 'm'){
			//Tahetakse tamiili sulatada
			int16_t melttime = atoi(command+3);
			
			sprintf(response, "<melt:%d>\n", melttime);
			usb_write(response);
			
			if(melttime > 300){
				usb_write("<Too high value!>\n");
			}else{
				if(command[1] == '1'){
					//Tahetakse juhtida alumist robotit nr. 1
					nrf24_tx_address(tx_address_l1);
				}else if(command[1] == '2'){
					//Tahetakse juhtida alumist robotit nr. 2
					nrf24_tx_address(tx_address_l2);
				}
				data_array[0] = 0xE7;	//Sent from main board
				data_array[1] = 0x03;	//Want to melt the line
				data_array[2] = melttime & 0xFF;
				data_array[3] = melttime >> 8;
				nrf24_send(data_array);
				while(nrf24_isSending());
				uint8_t temp = nrf24_lastMessageStatus();
				uint8_t connection_quality = nrf24_retransmissionCount();
				nrf24_tx_address(tx_address);
				nrf24_powerUpRx();
				if(temp == NRF24_MESSAGE_LOST){
					usb_write("<message:lost>\n");
				}else if(temp == NRF24_TRANSMISSON_OK){
					usb_write("<message:sent>\n");
				}else{
					usb_write("Something is really wrong!!!");
				}
				sprintf(response, "<quality:%d>\n", connection_quality);
				usb_write(response);
			}
		}else if(command[2] == 'l'){
			//Tahetakse sättida vasaku ratta kiirust
			motor_left_speed_buffer = atoi(command+3);
			sprintf(response, "<slbt:%d>\n", motor_left_speed_buffer);
			usb_write(response);
		}else if(command[2] == 'r'){
			//Tahetakse sättida parema ratta kiirust
			motor_right_speed_buffer = atoi(command+3);
			sprintf(response, "<srbt:%d>\n", motor_right_speed_buffer);
			usb_write(response);
		}else if(command[2] == 's'){
			//Tahetakse saata rataste kiirusi
			if(command[1] == '1'){
				//Tahetakse juhtida alumist robotit nr. 1
				nrf24_tx_address(tx_address_l1);
			}else if(command[1] == '2'){
				//Tahetakse juhtida alumist robotit nr. 2
				nrf24_tx_address(tx_address_l2);
			}
			data_array[0] = 0xE7;	//Sent from main board
			data_array[1] = 0x01;	//Want to change the speed of motors
			data_array[2] = motor_left_speed_buffer & 0xFF;
			data_array[3] = motor_left_speed_buffer >> 8;
			data_array[4] = motor_right_speed_buffer & 0xFF;
			data_array[5] = motor_right_speed_buffer >> 8;
			data_array[6] = 0;		//Don't send anything back
			nrf24_send(data_array);
			while(nrf24_isSending());
			uint8_t temp = nrf24_lastMessageStatus();
			uint8_t connection_quality = nrf24_retransmissionCount();
			nrf24_tx_address(tx_address);
			nrf24_powerUpRx();
			if(temp == NRF24_MESSAGE_LOST){
				usb_write("<message:lost>\n");
			}else if(temp == NRF24_TRANSMISSON_OK){
				usb_write("<message:sent>\n");
			}else{
				usb_write("Something is really wrong!!!");
			}
			sprintf(response, "<quality:%d>\n", connection_quality);
			usb_write(response);
			
		}
	}else if(command[0] == 't'){	
		setLEDs(32);
		//transfer message to other communication endpoint
		sprintf(response, "%s\n", command+1);
		if(!answer_to_usb) usb_write(response); else uart_print(response);
		sprintf(response, "<uart:sent>\n");
		if(answer_to_usb) usb_write(response); else uart_print(response);
		return;
	}else if(!isdigit(command[0])){
		sprintf(response, "<unknown:%s>\n", command);
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
	}else if (command[0] == 'p'  &&  command[1] == 'i'  &&  command[2] == 'n'  &&  command[3] == 'g') {
		sprintf(response, "<%d:pong>\n", id);
		if(answer_to_usb) usb_write(response); else uart_print(response);
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
	//fail_counter++;
}