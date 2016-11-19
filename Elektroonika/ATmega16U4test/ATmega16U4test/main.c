/*
 * ATmega16U4test.c
 *
 * Created: 04-Nov-16 18:15:07
 * Author : Myrka
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

ISR(INT0_vect) {
	//decode();
	PORTE ^= (1 << 6);
}

ISR(INT1_vect) {
	//decode();
	PORTE ^= (1 << 6);
}


int main(void){
    
	DDRE |= (1 << 6);
	
	EICRA = 0b00000101; //both edges
	EIMSK = 0b00000011; //enable mask
	
	sei();
	
	while (1) {
		_delay_ms(1000);
		PORTE ^= (1 << 6);
    }
}

