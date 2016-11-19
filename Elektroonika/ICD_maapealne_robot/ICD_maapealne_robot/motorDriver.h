/*
 * motorDriver.h
 *
 * Created: 15-Oct-16 15:14:13
 *  Author: Myrka
 */ 


#ifndef MOTORDRIVER_H_
#define MOTORDRIVER_H_

//ühised muutujad
int32_t pid_P;
int32_t pid_I;
int32_t pid_D;

//esimese mootori muutujad PID kontrolli jaoks
int16_t speed_to_set_m1;
int16_t rpm_to_hold_m1;
int32_t wanted_position_m1;
int32_t integral_m1, derivative_m1, error_m1, last_error_m1;

//teise mootori muutujad PID kontrolli jaoks
int16_t speed_to_set_m2;
int16_t rpm_to_hold_m2;
int32_t wanted_position_2;
int32_t integral_m2, derivative_m2, error_m2, last_error_m2;


void set_speed_m1(int16_t);
void set_speed_m2(int16_t);
void motorDriverUpdate();
void motorDriverInit();

void compass();

void reset();
#endif /* MOTORDRIVER_H_ */