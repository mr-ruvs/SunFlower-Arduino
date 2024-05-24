#include "PWM2.h"

float map1(float x, float in_min, float in_max, float out_min, float out_max){
	return ((x - in_min)*(out_max - out_min)/(in_max - in_min)) + out_min;
}

void initPWM2(void){
	// Configurar los pines PD3 y PB3 como salidas
	DDRD |= (1 << PD3);
	DDRB |= (1 << PB3);

	// Configurar el Timer/Counter2 para Fast PWM
	TCCR2A |= (1 << WGM21) | (1 << WGM20);  // Modo Fast PWM
	TCCR2A |= (1 << COM2A1);  // Clear OC2A on Compare Match, set OC2A at BOTTOM (non-inverting mode)
	TCCR2A |= (1 << COM2B1);  // Clear OC2B on Compare Match, set OC2B at BOTTOM (non-inverting mode)

	// Configurar el prescaler a 1024
	TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

	// Inicializar los registros de comparación de salida a 0
	OCR2A = 0;
	OCR2B = 0;
}

void updateDutyCA2(uint8_t duty){
	OCR2A = map1(duty,0,180,6,38);
}

void updateDutyCB2(uint8_t duty){
	OCR2B = map1(duty, 0, 100, 0, 255);
}