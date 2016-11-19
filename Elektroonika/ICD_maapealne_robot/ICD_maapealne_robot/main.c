
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
#include "motorDriver.h"
#include "usb_serial.h"
#include "nrf24.h"

//variables used for NRF24L01+ module
uint8_t data_array[10];
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7}; //Main board
uint8_t rx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE1}; //Robot nr. 1
uint8_t rx_address_remote[5] = {0xE7,0xE7,0xE7,0xE7,0xE7}; //remote

//variables used for USB communication
char buf[64], response[64];

//Other variables
_Bool controlled_by_remote = 0;

int main(void) {
	//remove clkdiv8
	CLKPR = 0x80;
	CLKPR = 0x00;
	
	//NUPUTUVASTUS
	//D0 sisendiks ja pull-up sisse
	DDRD &= ~(1 << 0);
	PORTD |= (1 << 0);
	//D1 miinusesse
	DDRD |= (1 << 1);
	PORTD &= ~(1 << 1);
	//D2 ohutuse mõttes sisendiks
	DDRD &= ~(1 << 2);
	DDRE |= (1 << 6);
	
	if((PIND & (1 << 0))){
		//Kui D0 on kõrge (jumperit pole siin)
		controlled_by_remote = 0;
		PORTE |= (1 << 6);
	}else{
		//Kui D0 on madal (jumper ühendatud)
		controlled_by_remote = 1;
	}
	
	//init USB
	usb_init();
	//while (!usb_configured());
	_delay_ms(500);
	
	usb_write("init motors...");
	//motorDriverInit();
	usb_write("Done!\n");
	
	
	//initialize the NRF24L01+ module
	usb_write("init nrf...");
	nrf24_init();
	nrf24_config(2, 10);//RF_CH 127 => 2.400128GHz, 10 byte packets
	nrf24_tx_address(tx_address);
	nrf24_rx_address(controlled_by_remote? rx_address_remote : rx_address);
	
	usb_write("Done!\n");
	
	int16_t failsafe = 0;
    while (1) {
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
					sprintf(response, "pong (robot 1, NRF: 0xE1\n");
					usb_write(response);
				}else {
					sprintf(response, "got(%d): %s\n", available, buf);
					usb_write(response);
				}
			
			}
		}
		
		_delay_ms(10);
		
    }
}
