#ifndef SERVOCONTROL_H_
#define SERVOCONTROL_H_

#include <avr/io.h>

void Servo_init(void);
void servo_writeA(float position);
void servo_writeB(float position);
float map(float x, float in_min, float in_max, float out_min, float out_max);


#endif /* SERVOCONTROL_H_ */