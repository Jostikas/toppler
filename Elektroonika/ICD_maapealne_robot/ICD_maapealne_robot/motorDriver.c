//Motordriveri kood, mis ikkagi ei toimi. Mingi kala on PID kontrolliga. Isegi ideaalse simulatsiooniga l‰hevad v‰‰tused kaootiliseks

/*
 * motorDriver.c
 *
 * Created: 15-Oct-16 15:13:58
 *  Author: Myrka
 */ 

/** DEBUG STUFF */
#include <stdio.h>
char buf[64];
uint16_t dbg = 0;

#define POLE_COUNT 6
#define F_CPU 16000000UL

#include <avr/interrupt.h>
#include <util/delay.h>
#include "i2csoft.h"

//globaalne aeg
uint32_t global_time, lastGetSpeedTime1, lastGetSpeedTime2;

int32_t pid_P = 10;
int32_t pid_I = 0;
int32_t pid_D = 0;

//muutujad esimese mootori PID kontrolli jaoks
int32_t integral_m1, derivative_m1, error_m1, last_error_m1;
int32_t speed_to_set_m1;	//holds the motor speed, which should be sent to the motor (signed)
int16_t rpm_to_hold_m1 = 0;	//holds the RPM value, that should be kept
int32_t wanted_position_m1 = 100;	//holds the distance the motor should have moved
int32_t real_position_m1;	//holds the distance the motor has movedint32_t realRPM;		//holds the calculated RPM of the motor since last two motorDriverUpdate() calls
int8_t direction_m1 = 0;
uint16_t position_updater_counter_m1 = 0;


//muutujad teise mootori PID kontrolli jaoks
int32_t integral_m2, derivative_m2, error_m2, last_error_m2;
int16_t speed_to_set_m2;
int16_t rpm_to_hold_m2;
int32_t wanted_position_m2;
int32_t real_position_m2;	//holds the distance the motor has movedint32_t realRPM;		//holds the calculated RPM of the motor since last two motorDriverUpdate() calls
int8_t direction_m2 = 0;
uint16_t position_updater_counter_m2 = 0;

volatile int16_t interrupts1 = 0;
volatile int16_t interrupts2 = 0;

void set_speed_m1(int16_t);
void set_speed_m2(int16_t);
void usb_write(const char *str);

ISR(INT0_vect){
//realPosition += 1 * direction;
// 	if(PIND & (1 << 0)){
// 		usb_write("falling");
// 	}else{
// 		usb_write("rising");
// 	}
	interrupts1++;
	PORTE ^= (1 << 6);
}

ISR(INT1_vect){
	interrupts2++;
	PORTE ^= (1 << 6);
}

void motorDriverInit(){
	SoftI2CInit();
	/*
	TCCR1B |= 1 << WGM12;
	TCCR1B |= 1 << CS12;//clkdiv256
	OCR1A = 510;
	TIMSK1 |= 1 << OCIE1A;
	*/
	//enable pull-up resistors
	/*DDRD &= ~(1 << 0);
	DDRD &= ~(1 << 1);
	PORTD |= (1 << 0);
	PORTD |= (1 << 1);	
	EICRA = 0b00000101; //both edges
	EIMSK = 0b00000011; //enable mask
	*/
	sei();
	
	set_speed_m1(0);
	set_speed_m1(0);
	set_speed_m1(0);
	
	set_speed_m2(0);
	set_speed_m2(0);
	set_speed_m2(0);
	
}

void motorDriverUpdateM1(){
	direction_m1 = ( speed_to_set_m1 < 0? -1 : 1 ); //basically holds the sign of the motor turning speed
	
	//timing calculations
	int32_t dt = (global_time-lastGetSpeedTime1);
	lastGetSpeedTime1 = global_time;
	
	//Calculating the real RPM
	//realRPM = direction_m1 * motordata_m1   *60   *100   /dt/POLE_COUNT;
	
	//calculating the moved distance and the ideal distance
	real_position_m1 += interrupts1 * direction_m1;
	interrupts1 = 0;
	//realPosition += motordata * direction; //subtract distance if moving backwards
	//realPosition += speed_to_set;
	
	//wantedPosition += rpm_to_hold/10;
	int16_t threshold = rpm_to_hold_m1>0?rpm_to_hold_m1:(-1)*rpm_to_hold_m1;
	if(threshold > 500){
		threshold = 500;
	}
	if(position_updater_counter_m1++ > 500 - threshold){
		if(rpm_to_hold_m1 == 0){
			wanted_position_m1 += 0;
		}else if(rpm_to_hold_m1 > 0){
			wanted_position_m1 += 1;
		}else{
			wanted_position_m1 -= 1;
		}
		position_updater_counter_m1 = 0;
	}
	
	//PID calculation
	error_m1 = wanted_position_m1 - real_position_m1;
	int16_t derr = last_error_m1 - error_m1;
	if(dt != 0){
		derivative_m1 += 10*derr/dt;
	}
		integral_m1 += error_m1*dt;
	
	if(error_m1 > 1000) error_m1 = 1000;
	if(error_m1 < -1000) error_m1 = -1000;
	
	//int16_t enne = speed_to_set;
	int32_t changeP = error_m1*pid_P/100;
	int32_t changeI = integral_m1 * pid_I/10000;
	int32_t changeD = derivative_m1 * pid_D/100;
	
	speed_to_set_m1 = changeP + changeI + changeD;
	
	last_error_m1 = error_m1;
	
	int16_t limit = 32000;
	if(speed_to_set_m1 > limit)  speed_to_set_m1 = limit;
	if(speed_to_set_m1 < -limit) speed_to_set_m1 = -limit;
	
	
	if(dbg++%200 == 0){
		usb_write("\n\n-- MOTOR 1 --\n");
		sprintf(buf, "int: %d, rpm: %d\n", (int16_t)interrupts1, rpm_to_hold_m1);
		usb_write(buf);
		
		sprintf(buf, "PID change: %d # %d + %d + %d => %d\n", (int16_t)error_m1, (int16_t)changeP, (int16_t)changeI, (int16_t)changeD, (int16_t)speed_to_set_m1);
		usb_write(buf);
		
		sprintf(buf, "PID values: %d, %d, %d\n", (int16_t)pid_P, (int16_t)pid_I, (int16_t)pid_D);
		usb_write(buf);
		
		sprintf(buf, "r: %d , w: %d , e: %d ,", (int16_t)real_position_m1, (int16_t)wanted_position_m1, (int16_t)error_m1);//realPosition, wantedPosition, error
		usb_write(buf);
		
	}
	
	//try to make the speed a bit more linear
	/*if(speed_to_set < -100){
		setSpeed(-1);
		for(uint16_t t = 0; t < speed_to_set; t++){
			_delay_us(5);
		}
		setSpeed(0);
	}else if(speed_to_set > 100){
		setSpeed(1);
		for(uint16_t t = 0; t < speed_to_set; t++){
			_delay_us(5);
		}
		set_speed_m1(0);
	}else{
		setSpeed(speed_to_set_m1);
	}*/
	set_speed_m1(speed_to_set_m1);
}

void motorDriverUpdateM2(){
	direction_m2 = ( speed_to_set_m2 < 0? -1 : 1 ); //basically holds the sign of the motor turning speed
	
	//timing calculations
	int32_t dt = (global_time-lastGetSpeedTime2);
	lastGetSpeedTime2 = global_time;
	
	//Calculating the real RPM
	//realRPM = direction_m2 * motordata_m2   *60   *100   /dt/POLE_COUNT;
	
	//calculating the moved distance and the ideal distance
	real_position_m2 += interrupts2 * direction_m2;
	interrupts2 = 0;
	//realPosition += motordata * direction; //subtract distance if moving backwards
	//realPosition += speed_to_set;
	
	//wantedPosition += rpm_to_hold/10;
	int16_t threshold = rpm_to_hold_m2>0?rpm_to_hold_m2:(-1)*rpm_to_hold_m2;
	if(threshold > 500){
		threshold = 500;
	}
	if(position_updater_counter_m2++ > 500 - threshold){
		if(rpm_to_hold_m2 == 0){
			wanted_position_m2 += 0;
		}else if(rpm_to_hold_m2 > 0){
			wanted_position_m2 += 1;
		}else{
			wanted_position_m2 -= 1;
		}
		position_updater_counter_m2 = 0;
	}
	
	//PID calculation
	error_m2 = wanted_position_m2 - real_position_m2;
	int16_t derr = last_error_m2 - error_m2;
	if(dt != 0){
		derivative_m2 += 10*derr/dt;
	}
		integral_m2 += error_m2*dt;
	
	if(error_m2 > 1000) error_m2 = 1000;
	if(error_m2 < -1000) error_m2 = -1000;
	
	//int16_t enne = speed_to_set;
	int32_t changeP = error_m2*pid_P/100;
	int32_t changeI = integral_m2 * pid_I/10000;
	int32_t changeD = derivative_m2 * pid_D/100;
	
	speed_to_set_m2 = changeP + changeI + changeD;
	
	last_error_m2 = error_m2;
	
	int16_t limit = 32000;
	if(speed_to_set_m2 > limit)  speed_to_set_m2 = limit;
	if(speed_to_set_m2 < -limit) speed_to_set_m2 = -limit;
	
	
	if(dbg%200 == 0){
		usb_write("\n\n-- MOTOR 2 --\n");
		sprintf(buf, "int: %d, rpm: %d\n", (int16_t)interrupts2, rpm_to_hold_m2);
		usb_write(buf);
		
		sprintf(buf, "PID change: %d # %d + %d + %d => %d\n", (int16_t)error_m2, (int16_t)changeP, (int16_t)changeI, (int16_t)changeD, (int16_t)speed_to_set_m2);
		usb_write(buf);
		
		sprintf(buf, "PID values: %d, %d, %d\n", (int16_t)pid_P, (int16_t)pid_I, (int16_t)pid_D);
		usb_write(buf);
		
		sprintf(buf, "r: %d , w: %d , e: %d ,", (int16_t)real_position_m2, (int16_t)wanted_position_m2, (int16_t)error_m2);//realPosition, wantedPosition, error
		usb_write(buf);
		
	}
	
	//try to make the speed a bit more linear
	/*if(speed_to_set < -100){
		setSpeed(-1);
		for(uint16_t t = 0; t < speed_to_set; t++){
			_delay_us(5);
		}
		setSpeed(0);
	}else if(speed_to_set > 100){
		setSpeed(1);
		for(uint16_t t = 0; t < speed_to_set; t++){
			_delay_us(5);
		}
		setSpeed(0);
	}else{
		setSpeed(speed_to_set);
	}*/
	set_speed_m2(speed_to_set_m2);
}

void set_speed_m1(int16_t s){
	SoftI2CStart();
	SoftI2CWriteByte(0x29<<1);
	SoftI2CWriteByte(0);
	SoftI2CWriteByte(s>>8);
	SoftI2CWriteByte(s);
	SoftI2CStop();
}

void set_speed_m2(int16_t s){
	SoftI2CStart();
	SoftI2CWriteByte(0x2A<<1);
	SoftI2CWriteByte(0);
	SoftI2CWriteByte(s>>8);
	SoftI2CWriteByte(s);
	SoftI2CStop();
}

void motorDriverUpdate(){
	motorDriverUpdateM1();
	motorDriverUpdateM2();
}

void reset(){
	pid_P = 100;
	pid_I = 0;
	pid_D = 0;
	integral_m1 = 0;
	derivative_m1 = 0;
	speed_to_set_m1 = 0;	//holds the motor speed, which should be sent to the motor (signed)
	real_position_m1 = 0;	//holds the distance the motor has moved
	wanted_position_m1 = 0;	//holds the distance the motor should have moved
	rpm_to_hold_m1 = 0;	//holds the RPM value, that should be kept
	
	integral_m2 = 0;
	derivative_m2 = 0;
	speed_to_set_m2 = 0;	//holds the motor speed, which should be sent to the motor (signed)
	real_position_m2 = 0;	//holds the distance the motor has moved
	wanted_position_m2 = 0;	//holds the distance the motor should have moved
	rpm_to_hold_m2 = 0;	//holds the RPM value, that should be kept
	
}
/*
uint8_t tim = 0;
ISR(TIMER1_COMPA_vect){
	global_time++;
	if(tim++ >= 100){
		PORTE ^= (1 << 6);
		tim = 0;
	}
}*/

uint8_t first = 1;

void compass(){
	if(first == 1){
		first = 0;
		SoftI2CStart();
		SoftI2CWriteByte(0x1E<<1);
		SoftI2CWriteByte(0x02); //mode register
		SoftI2CWriteByte(0x00); //continous mode
		SoftI2CStop();
		
		SoftI2CStart();
		SoftI2CWriteByte(0x1E<<1);
		SoftI2CWriteByte(0x00); //mode register
		SoftI2CWriteByte(0b00011100); //continous mode
		SoftI2CStop();
		
		
		usb_write("first time config");
		_delay_ms(100);
	}
	/*SoftI2CStart();
	SoftI2CWriteByte(0x1E<<1);
	SoftI2CWriteByte(0x00); //mode register
	SoftI2CWriteByte(0b00011100); //continous mode
	SoftI2CStop();
	*/
	
	/*
	SoftI2CStart();
	SoftI2CWriteByte(0x1E<<1);
	SoftI2CWriteByte(0x02); //mode register
	SoftI2CWriteByte(0x00); //continous mode
	SoftI2CStop();
	*/
	
	
	
	uint8_t tmp[6];
	SoftI2CStart();
	SoftI2CWriteByte(0x1E<<1);
	SoftI2CWriteByte(0x00); //data start register
	SoftI2CStop();
	
	SoftI2CStart();
	SoftI2CWriteByte((0x1E<<1)|1);
	SoftI2CWriteByte(0x06);
	tmp[0] =  SoftI2CReadByte(0);
	SoftI2CWriteByte((0x1E<<1)|1);
	tmp[1] =  SoftI2CReadByte(0);
	SoftI2CStop();
	
	SoftI2CStart();
	SoftI2CWriteByte((0x1E<<1)|1);
	tmp[2] =  SoftI2CReadByte(0);
	SoftI2CStop();
	
	SoftI2CStart();
	SoftI2CWriteByte((0x1E<<1)|1);
	tmp[3] =  SoftI2CReadByte(0);
	SoftI2CStop();
	
	SoftI2CStart();
	SoftI2CWriteByte((0x1E<<1)|1);
	tmp[4] =  SoftI2CReadByte(0);
	SoftI2CStop();
	
	SoftI2CStart();
	SoftI2CWriteByte((0x1E<<1)|1);
	tmp[5] =  SoftI2CReadByte(0);
	SoftI2CStop();
	
	sprintf(buf, "compass: %d, %d, %d, %d, %d, %d\n", tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5]);
	usb_write(buf);
	
}