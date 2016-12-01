
/*
 * .c
 *
 * Created: 14-Oct-16 11:07:22
 * Author : Myrka
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include "motorDriver.h"
#include "usb_serial.h"
#include "nrf24.h"

//pin definitions
#define PIN_LED_1_R 1
#define PIN_LED_1_G 0
#define PIN_LED_1_B 4
#define PIN_LED_2_R 6
#define PIN_LED_2_G 5
#define PIN_LED_2_B 7

void init_ports_and_pins();
void setLED(_Bool led, _Bool r, _Bool g, _Bool b);
void setLEDs(unsigned char);
void melt(uint16_t time);

//variables used for NRF24L01+ module
uint8_t data_array[10];
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7}; //Main board
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE1}; //Robot nr. 1 (default)
uint8_t rx_address_2[5] = {0xE7,0xE7,0xE7,0xE7,0xE2}; //Robot nr. 2
uint8_t rx_address_remote[5] = {0xE7,0xE7,0xE7,0xE7,0xE7}; //remote

//variables used for USB communication
char buf[64], response[64];

//Other variables
_Bool controlled_by_remote = 0;
uint8_t robot_id = 0;
int16_t meltTime = 0;

int main(void) {
	//remove clkdiv8
	CLKPR = 0x80;
	CLKPR = 0x00;
	
	init_ports_and_pins();
	setLEDs(9);
	
	//init USB
	usb_init();
	//while (!usb_configured());
	
	
	robot_id = eeprom_read_byte((uint8_t*)0);
	
	if(robot_id == 1){
		setLED(1, 0, 1, 0);
	}else if(robot_id == 2){
		setLED(1, 0, 0, 1);
	}else{
		//Midagi on valesti, ilmselt on auto confimata
		for(uint8_t i = 0; i < 10; i++){
			setLED(1, 1, 0, 0);
			_delay_ms(100);
			setLED(1, 0, 0, 0);
			_delay_ms(100);
		}
	}
	_delay_ms(200);
	setLED(1, 0, 0, 0);
	_delay_ms(200);
	
	if((PIND & (1 << 1))){
		controlled_by_remote = 0;
		setLED(1, 0, 1, 0);
	}else{
		controlled_by_remote = 1;
		setLED(1, 0, 0, 1);
	}
	
	usb_write("init motors...");
	motorDriverInit();
	usb_write("Done!\n");
	
	
	//initialize the NRF24L01+ module
	usb_write("init nrf...");
	nrf24_init();
	nrf24_config(2, 10);//RF_CH 127 => 2.400128GHz, 10 byte packets
	nrf24_tx_address(tx_address);
	nrf24_rx_address(controlled_by_remote? rx_address_remote : (robot_id == 2?rx_address_2:rx_address));
	
	usb_write("Done!\n");
	
	int16_t failsafe = 0;
    while (1) {
		if(failsafe < 15){
			setLED(0, 0, 1, 0);
		}else if(failsafe < 80){
			setLED(0, 0, 0, failsafe&8);
		}else{
			setLED(0, failsafe&2, 0, 0);
		}
		
		
		
		while(nrf24_dataReady()){
			if(!controlled_by_remote){
				failsafe = 0;
				nrf24_getData(data_array);
				usb_write("Received:\n");
				for(int i = 0; i < 10; i++){
					sprintf(response, "%d. val=%d\n", i, data_array[i]);
					usb_write(response);
				}
				if(data_array[1] == 1){
					//Tahetakse sättida mootorite kiiruseid
					rpm_to_hold_m1 = (data_array[3]<<8) + data_array[2];
					rpm_to_hold_m2 = (data_array[5]<<8) + data_array[4];
				
					sprintf(response, "kiirused: %d ja %d\n", rpm_to_hold_m1, rpm_to_hold_m2);
					usb_write(response);
				}else if(data_array[1] == 2){
					//Tahetakse pöörata
					rpm_to_hold_m1 = (data_array[3]<<8) + data_array[2];
					rpm_to_hold_m2 = (data_array[5]<<8) + data_array[4];
					uint16_t time  = (data_array[7]<<8) + data_array[6];
					
					
					sprintf(response, "kiirused: %d ja %d\n", rpm_to_hold_m1, rpm_to_hold_m2);
					usb_write(response);
					
					set_speed_m1(rpm_to_hold_m1);
					set_speed_m2(rpm_to_hold_m2);
					
					for(uint16_t i = 0; i < time; i++){
						_delay_us(100);
					}
					rpm_to_hold_m1 = rpm_to_hold_m2 = 0;
					set_speed_m1(0);
					set_speed_m2(0);
					
				}else if(data_array[1] == 3){
					//Tamiili sulatada
					uint16_t time  = (data_array[3]<<8) + data_array[2];
					meltTime = time;				
					//melt(time);
					
				}else{
					sprintf(response, "unknown type: %d\n", data_array[1]);
					usb_write(response);
				}
			
			}else{
			
				///////////////////////////////////////
				//VANA KOOD
				///////////////////////////////////////
				
			
						failsafe = 0;
						nrf24_getData(data_array);
						if(data_array[7] & (1 << 4)){
							//debug_text_counter = 0;
							usb_write("Received:\n");
							for(int i = 0; i < 8; i++){
								sprintf(response, "%d. val=%d\n", i, data_array[i]);
								usb_write(response);
							}
						}
			
						//////////////////////
						usb_write("Received:\n");
						for(int i = 0; i < 6; i++){
							sprintf(response, "%d. val=%d\n", i, data_array[i]);
							usb_write(response);
						}
						////////////////////////
						
						if(data_array[0] == 0  &&  data_array[1] == 0){
							meltTime = data_array[4]*4;
						}
						
						
						if(data_array[3] > 105  ||  data_array[3] < 95){
							//setSpeed(100-data_array[3]);
				
							rpm_to_hold_m1 = (100-data_array[3])*5 * data_array[5]/100;
							rpm_to_hold_m2 = (-1)*((int16_t)((100-data_array[3])*5));
				
							if(data_array[7] & (1 << 4)){
								rpm_to_hold_m1 += (100-data_array[2])*data_array[5]/70;
								rpm_to_hold_m2 += (100-data_array[2])*data_array[5]/70;
							}
				
				
						}else{
							rpm_to_hold_m1 = 0;
							rpm_to_hold_m2 = 0;
				
							//setSpeed(0);
						}
			
						//sprintf(response, "spd: %d\n", rpm_to_hold);
						//usb_write(response);
						rpm_to_hold_m1 *= data_array[0];
						rpm_to_hold_m2 *= data_array[0];
			
			
				
						/*if(data_array[7] & (1 << 4)){
							if(rpm_to_hold_m1 < 200  &&  rpm_to_hold_m1 > -200){
								rpm_to_hold_m1 = 0;
							}
							if(rpm_to_hold_m2 < 200  &&  rpm_to_hold_m2 > -200){
								rpm_to_hold_m2 = 0;
							}
				
						}*/
				///////////////////////////////////////
				//VANA KOOD
				///////////////////////////////////////
			}
		}
		
		if(meltTime > 0){
			meltTime--;
			PORTB |= (1 << 6);
			setLEDs(0xFF);
		}else{
			PORTB &= ~(1 << 6);
			if(controlled_by_remote){
				setLED(1, 0, 0, 1);
			}else{
				setLED(1, 0, 1, 0);
			}
		}
		
		if(failsafe++ < 200){
			set_speed_m1(rpm_to_hold_m1);
			set_speed_m2(rpm_to_hold_m2);
		}else{
			set_speed_m1(0);
			set_speed_m2(0);
		}
		
		uint8_t available = usb_serial_available();
		if (available) {
			uint8_t n = recv_str(buf, sizeof(buf));
			if (n == sizeof(buf)) {
				if (buf[0] == '?') {
					sprintf(response, "pong (robot %d, NRF: 0xE%d)\n", robot_id, robot_id);
					usb_write(response);
				} else if ((buf[0] == 'i') && (buf[1] == 'd')) {
					//set igain
					int16_t par1 = atoi(buf+2);
					eeprom_update_byte((uint8_t*)0, par1);
					sprintf(response, "Set robot ID from %d to %d\n", robot_id, par1);
					robot_id = par1;
					usb_write(response);
					usb_write("Restart required!\n");
				}else {
					sprintf(response, "got(%d): %s\n", available, buf);
					usb_write(response);
				}
			
			}
		}
		
		_delay_ms(10);
		
    }
}

//Related to initialization
void init_ports_and_pins(){
	//disable JTAG, needed to use some of the pins on the MCU
	MCUCR|= (1<<JTD); //in order to change this value, it is needed to 
	MCUCR|= (1<<JTD); //overwrite this value twice during 4 clock cycles
	   
	DDRF = 0b11110011; //LEDs
	PORTF = 255; //swich LEDs off by default
	
	//UART pins (rs485)
	DDRD |= (1 << 6) | (1 << 7);
	//Enabling receive and transmit (on the rs485 chip)
	PORTD |= (1 << 6);
	PORTD &= ~(1 << 7);
	
	//Setup wire melting
	DDRB |= (1 << 6);
	PORTB &= ~(1 << 6);
	
}

void melt(uint16_t time){
	sprintf(response, "sulatan: %d\n", time);
	usb_write(response);
	PORTB |= (1 << 6);
	for(uint16_t i = 0; i < time; i++){
		_delay_ms(1);
	}
	PORTB = ~(1 << 6);
	usb_write("done!\n");
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