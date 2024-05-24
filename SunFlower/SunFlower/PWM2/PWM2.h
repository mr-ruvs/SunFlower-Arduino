#ifndef PWM2_H_
#define PWM2_H_

#include <avr/io.h>
#include <stdint.h>

#define invertido 1
#define no_invertido 0

float map1(float, float, float, float, float);

void initPWM2(void);

void updateDutyCA2(uint8_t duty);

void updateDutyCB2(uint8_t duty);

#endif /* PWM2_H_ */