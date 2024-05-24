//***************************************************************************
// Universidad del Valle de Guatemala
// IE2023: Programación de Microcontroladores
// Autor: Ruben Granados
// Hardware: ATMEGA328P
// Created: 03/05/2024
//***************************************************************************
// Proyecto SunFlower
//***************************************************************************

#define F_CPU 16000000
#include <util/delay.h>
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include "ServoControl/ServoControl.h"
#include "PWM2/PWM2.h"

//__ Declaracion de Funciones __
void ADC_init(void);
uint16_t adcRead(uint8_t);
void initUART(void);
void writeUART(char caracter);
void writeTextUART(char* texto);
void PruebaLecturaLDR(void);
void PruebaServo(void);
void intensidadLuminica(void);
void initTimer0(void);
void setupPCINT(void);
void solarTracker(void);
void solarPanel(void);
int ChatIntConvert(char c);
int sumaTotal(int centena, int decena, int unidad);

//
#define BUTTON_PIN PB4
#define BUTTON_PP PD7
#define BUTTON_REC PB0
#define LGREEN PD4
#define LRED PD5
#define LBLUE PD6

//__ Declaracion de Variables __
// almacenar valor de LDR
int topL = 0;
int topR = 0;
int botL = 0;
int botR = 0;
// UART
char buffer[10];		// envio de dato
volatile char bufferRX[4];
volatile uint8_t bufferIndex = 0;
uint8_t IntensidadLuz = 0;
//
uint8_t DutyC2 = 0;
volatile uint8_t mainState = 2;
volatile uint32_t milliseconds = 0;
uint8_t recEeprom = 0;
uint8_t playEeprom = 0;
int contRecEeprom = 0;
int contRecEeprom2 = 0;
int contRecEeprom3 = 0;
// servos
int servoh = 0;
int servov = 0;
int servohMax = 180;
int servohMin = 0;
int servovMax = 15;
int servovMin = 90;
// solar tracker
uint16_t top = 0;
uint16_t bot = 0;
uint16_t left = 0;
uint16_t right = 0;
// eeprom
uint8_t eepromValue1 = 0;
uint8_t eepromValue2 = 0;
uint8_t eepromValue3 = 0;
// dc motor
int velocidadMotor = 0;
int stateCerrar = 0;
int stateAbrir = 1;
//***************************************************************************
// SETUP
//***************************************************************************
int main(void){
	cli();
	initUART();
	ADC_init();
	Servo_init();
	initPWM2();
	initTimer0();
	setupPCINT();
	// configuracion de pines
	DDRD |= (1 << LRED)|(1 << LGREEN)|(1 << LBLUE); //RGB
	DDRD |= (1 << PD2);
	sei();
	servoh = 90;
	servov = servovMax;
	servo_writeA(servov);
	servo_writeB(servoh);
	writeTextUART("\nSUNFLOWER SOLAR PANEL\n");
	
	//abrir
	updateDutyCB2(200);
	PORTD &= ~(1<<PD2);
	_delay_ms(1000);
	// desactivar
	updateDutyCB2(0);
	//PORTD |= (1<<PD2);
	//_delay_ms(1000);
	//PORTD &= ~(1<<PD2);
	//_delay_ms(1000);
	//***************************************************************************
	// LOOP
	//***************************************************************************
    while (1) {
		switch(mainState){
			// estado de funcionalidad normal
			case 0:
				PORTD |= (1 << LGREEN);
				PORTD &= ~(1<<LRED);
				PORTD &= ~(1<<LBLUE);
				intensidadLuminica();
				solarTracker();
				
				break;
			// estado de funcionalidad EEPROM
			case 1:
				// modo eeprom
				if(recEeprom == 0 && playEeprom == 0) {
					PORTD |= (1<<LGREEN);
					PORTD |= (1<<LBLUE);
					contRecEeprom = 0;
				}
				// reproducir eeprom
				 if (recEeprom == 0 && playEeprom == 1){
					PORTD |= (1 << LRED);
					PORTD &= ~(1<<LGREEN);
					PORTD &= ~(1<<LBLUE);
					if (milliseconds >= 1000){
						milliseconds = 0;
						if (contRecEeprom < 10){
							eepromValue1 = eeprom_read_byte((uint8_t*)contRecEeprom);
							updateDutyCA2(eepromValue1);
							contRecEeprom2 = contRecEeprom + 10;
							eepromValue2 = eeprom_read_byte((uint8_t*)contRecEeprom2);
							servo_writeA(eepromValue2);
							contRecEeprom3 = contRecEeprom + 20;
							eepromValue3 = eeprom_read_byte((uint8_t*)contRecEeprom3);
							servo_writeB(eepromValue3);
							contRecEeprom++;
						}
						if (contRecEeprom >= 10){
							playEeprom = 0;
						}
					}
				}
				// grabar eeprom
				if (recEeprom == 1 && playEeprom == 0){
					intensidadLuminica();
					solarTracker();
					PORTD &= ~(1<<LGREEN);
					PORTD &= ~(1<<LBLUE);
					if (milliseconds >= 1000) {
						PORTD ^= (1 << LRED);
						milliseconds = 0;  // Reiniciar el contador
						if (contRecEeprom < 10){
							eeprom_write_byte((uint8_t*)contRecEeprom, DutyC2);
							contRecEeprom2 = contRecEeprom + 10;
							eeprom_write_byte((uint8_t*)contRecEeprom2, servov);
							contRecEeprom3 = contRecEeprom + 20;
							eeprom_write_byte((uint8_t*)contRecEeprom3, servoh);
							contRecEeprom++;
						}
						if (contRecEeprom >= 10){
							recEeprom = 0;
						}
					}
				}
				break;
			// estado de funcionalidad remota UART
			case 2:
				PORTD |= (1 << LBLUE);
				PORTD &= ~(1<<LRED);
				PORTD &= ~(1<<LGREEN);
				int valorCen = ChatIntConvert(bufferRX[0]);
				int valorDec = ChatIntConvert(bufferRX[1]);
				int valorU = ChatIntConvert(bufferRX[2]);
				int valorSelect = sumaTotal(valorCen, valorDec, valorU);
				if (valorSelect >= 100 && valorSelect <= 199){
					valorSelect = map(valorSelect, 100, 199, 0, 180);
					updateDutyCA2(valorSelect);
				} else if (valorSelect >= 200 && valorSelect <= 299){
					valorSelect = map(valorSelect, 200, 299, 10, 90);
					servo_writeA(valorSelect);
				} else if (valorSelect >= 300 && valorSelect <= 399){
					valorSelect = map(valorSelect, 300, 399, 0, 180);
					servo_writeB(valorSelect);
				} else if(valorSelect >= 400 && valorSelect <= 499){
					velocidadMotor = map(valorSelect, 400, 499, 0, 100);
				}
				if (valorSelect == 501){	// abrir
					stateAbrir = 0;
					stateCerrar = 1;
					updateDutyCB2(velocidadMotor);
					PORTD &= ~(1<<PD2);
					_delay_ms(1000);
					updateDutyCB2(0);
					PORTD &= ~(1<<PD2);
				} else if (valorSelect == 510){	// cerrar
					stateCerrar = 0;
					stateAbrir = 1;
					updateDutyCB2(0);
					PORTD |= (1<<PD2);
					_delay_ms(1000);
					updateDutyCB2(0);
					PORTD &= ~(1<<PD2);
				}
				break;
		}
    }
}
//***************************************************************************
// FUNCIONES
//***************************************************************************
// ADC
void ADC_init(void){
	ADMUX |= (1<<REFS0);	// VCC REF
	ADMUX &= ~(1<<REFS1);
	ADMUX &= ~(1<<ADLAR);	// 10 bits
	// PRESCALER 128 > 16M/128 = 125KHz
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
	ADCSRA |= (1<<ADEN);	// ADC ON
}
uint16_t adcRead(uint8_t canal){
	ADMUX = (ADMUX & 0xF0)|canal;	// selección de canal
	ADCSRA |= (1<<ADSC);	// inicia conversión
	while((ADCSRA)&(1<<ADSC));	// hasta finalizar conversión
	return(ADC);
}
void PruebaLecturaLDR(void){
	topL = adcRead(0);
	// int a caracter
	sprintf(buffer, "%d", topL);
	// enviar valor de la ldr
	writeTextUART("TOP LEFT: ");
	writeTextUART(buffer);
	writeTextUART("\n");
	
	topR = adcRead(1);
	sprintf(buffer, "%d", topR);
	writeTextUART("TOP RIGHT: ");
	writeTextUART(buffer);
	writeTextUART("\n");
	
	botL = adcRead(3);
	sprintf(buffer, "%d", botL);
	writeTextUART("BOTTOM LEFT: ");
	writeTextUART(buffer);
	writeTextUART("\n");
	
	botR = adcRead(4);
	sprintf(buffer, "%d", botR);
	writeTextUART("BOTTON RIGHT: ");
	writeTextUART(buffer);
	writeTextUART("\n");
	writeTextUART("\n");
}
// COMUNICACION SERIAL
void initUART(void){
	// Rx y Tx
	DDRD &= ~(1<<DDD0);
	DDRD |= (1<<DDD1);
	// fast mode
	// CONFIGURAR A
	UCSR0A = 0;
	UCSR0A |= (1<<U2X0);
	// CONFIGURAR B /  ISR RX/ habilitar Rx y Tx
	UCSR0B = 0;
	UCSR0B |= (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
	// frame 8 bits, no paridad, 1 bit stop
	UCSR0C = 0;
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	// baudrate 9600
	UBRR0 = 207;
}
void writeUART(char caracter){
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0 = caracter;
}
void writeTextUART(char* texto){
	uint8_t i;
	for (i=0; texto[i] != '\0'; i++){
		while(!(UCSR0A&(1<<UDRE0)));
		UDR0 = texto[i];
	}
}
ISR(USART_RX_vect){
	char received = UDR0;
	if (received != '\n') {
		bufferRX[bufferIndex++] = received;
		// Verificar si se superó el límite del buffer
		if (bufferIndex >= 4 - 1) {
			bufferIndex = 4 - 1;
			bufferIndex = 0; // reiniciar
		}
	}
}
int ChatIntConvert(char c){
	return c - '0';
}
int sumaTotal(int centena, int decena, int unidad){
	return (centena*100 + decena*10 + unidad);
}
// FUNCIONES PRINCIPALES
void intensidadLuminica(void){
	topL = adcRead(1);
	topR = adcRead(2);
	botL = adcRead(0);
	botR = adcRead(3);
	int promIL = (topL + topR + botL + botR)/4;
	DutyC2 = map(promIL, 0, 1023, 180, 0);
	updateDutyCA2(DutyC2);
	_delay_ms(2);
	/*
	// int a caracter
	sprintf(buffer, "%d", topR);
	// enviar valor 
	writeTextUART("top R: ");
	writeTextUART(buffer);
	writeTextUART("\n");
	*/
}
void solarTracker(void){
	topL = adcRead(1);
	topR = adcRead(2);
	botL = adcRead(0);
	botR = adcRead(3);
	// calculo de promedio
	top = (topL + topR)/2;
	bot = (botL + botR)/2;
	left = (topL + botL)/2;
	right = (topR + botR)/2;
	// movimiento vertical
	if (top < bot){
		servov = servov - 1;
		servo_writeA(servov);
		if (servov <= servovMax){
			servov = servovMax;
		}
		_delay_ms(10);
	}else if (bot < top){
		servov = servov + 1;
		servo_writeA(servov);
		if (servov >= servovMin){
			servov = servovMin;
		}
		_delay_ms(10);
	} 
	// movimiento horizontal
	if (right < left){
		servoh = servoh + 1;
		servo_writeB(servoh);
		if (servoh >= servohMax){
			servoh = servohMax;
		}
		_delay_ms(20);
		}else if (left < right){
		servoh = servoh - 1;
		servo_writeB(servoh);
		if (servoh <= servohMin){
			servoh = servohMin;
		}
		_delay_ms(20);
	}
}
// solar panel
void solarPanel(void){
	topL = adcRead(0);
	topR = adcRead(1);
	botL = adcRead(3);
	botR = adcRead(4);
	int promIL = (topL + topR + botL + botR)/4;
	/*if(promIL <= 50 &&){

	}*/
	if(promIL >= 51 && stateAbrir == 1){
		updateDutyCB2(255);
		PORTD |= (1<<PD2);
	}
}
// TIMER 0 --> millis()
void initTimer0(void) {
	// Configurar Timer/Counter0 en modo CTC (Clear Timer on Compare Match)
	TCCR0A = (1 << WGM01);
	// Configurar preescala a 64 (para generar interrupción cada 1 ms con F_CPU = 16 MHz)
	TCCR0B = (1 << CS01) | (1 << CS00);
	// Configurar el valor de comparación (OCR0A) para generar una interrupción cada 1 ms
	OCR0A = 249;  // (F_CPU / (prescaler * desired frequency)) - 1
	// Habilitar la interrupción de comparación del Timer/Counter0
	TIMSK0 = (1 << OCIE0A);
}
// Timer/Counter0 Compare Match A
ISR(TIMER0_COMPA_vect) {
	milliseconds++;  // Incrementar el contador de milisegundos
}
// PULSADORES
void setupPCINT(void) {
	DDRB &= ~(1 << DDB4);  // entrada
	PORTB |= (1 << PORTB4);  // pull-up
	DDRB &= ~(1 << DDB0);  // PB0 
	PORTB |= (1 << PORTB0);  // pull-up
	PCICR |= (1 << PCIE0);  // PCINT0-7
	PCMSK0 |= (1 << PCINT4);  // PCINT4
	PCMSK0 |= (1 << PCINT0);  // PCINT0
	
	DDRD &= ~(1 << DDD7); // entrada
	PORTD |= (1 << PORTD7); // pull-up
	PCICR |= (1 << PCIE2); 
	PCMSK2 |= (1 << PCINT23); // PCINT23
}
// interrupcion
ISR(PCINT0_vect) {
	// Antirrebote simple 
	_delay_ms(10);  
	if (!(PINB & (1 << BUTTON_PIN))) { 
		// Incrementar el contador 
		mainState = (mainState + 1) % 3;
	}
	if (!(PINB & (1 << BUTTON_REC))) {  
		if (mainState == 1){
			if (recEeprom == 0){
				recEeprom = 1;
				} else {
				recEeprom = 0;
			}
		}
	}
}
ISR(PCINT2_vect) {
	_delay_ms(10);  
	if (!(PIND & (1 << BUTTON_PP))) {  
		if (mainState == 1){
			// Cambiar el estado
			if (playEeprom == 0){
				playEeprom = 1;
				} else {
				playEeprom = 0;
			}
		}
		
	}
}
