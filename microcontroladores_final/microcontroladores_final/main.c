/*
 * microcontroladores_final.c
 *
 * Created: 26/1/2024 06:59:29
 * Author : rafad
 */ 
#define F_CPU 16000000 //Frecuencia del microcontrolador
#define BAUDRATE 9600 //Velocidad de transmisi�n de baudios
//INCLUDES
#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

volatile uint16_t pulse_count = 0; // Contador de pulsos
volatile uint16_t prev_pulse_count = 0; // Contador de pulsos previo
volatile uint16_t timer_overflow_count = 0; // Contador de desbordamiento del temporizador
volatile uint16_t motor_speed = 0; // Velocidad del motor en revoluciones por minuto (RPM)
volatile uint8_t comando_ing = 1;
//CONSTANTES
#define TIMER_PRESCALER 64         // Preescalador del temporizador
#define POLO = -4000; //Tomado de parcial. Sujeto a cambio
#define LA 11.77; //Medida en mH. Tomado de parcial
#define RA 3.05; //Medida en ohms del motor
#define K 0.2462; //Tomado de parcial
#define R_prima 7.08; //Tomado de parcial
#define b_visc 0; //Asumido
#define PULSES_PER_REVOLUTION 1000 // N�mero de pulsos por revoluci�n del encoder. No es el correcto para nuestro
//encoder. Se necesita determinar la cantidad de pulsos del encoder por revoluci�n del eje.
//Nos podemos ayudar con �ste c�digo


//VARIABLES GLOBALES
volatile float t0 = 0;
volatile float t = 0; //Momento en el tiempo que se debe conseguir con timers medido en segundos
volatile float w = 0; //Se necesita determinar por medio del encoder e interrupciones
volatile float consigna_torque = 60; //Medida en mNm (mili newtons metro) y c�mo ingresar la consigna
volatile float torque_resistente = b_visc*w;
volatile float consigna_ia = ((consigna_torque/1000)+torque_resistente)/K; //Se pasa a Nm y luego se divide
volatile	float ia0 = 0;
volatile	float Et = K*w;
volatile	float Ut = 20; //Es la tensi�n aplicada al motor. En la primera iteraci�n es la tensi�n inicial
volatile float consigna_va = Ut; //Se trata de que sea ==Ut
volatile	float It = (Ut-Et)/RA;
volatile	float ia = (ia0-It)*exp((-RA/LA)*(t-t0)) + It; //Corriente

char comando[10];
uint8_t indcom;

//FUNCIONES INIT
void init_encoder() {
	// Configurar el PIN 4 (PCINT4) como entrada
	DDRD &= ~(1 << PCINT20);
	
	// Habilitar pull-up interno en el PIN 4
	PORTD |= (1 << PCINT20);
	
	// Habilitar interrupci�n por cambio de nivel en PCINT20 (PIN 4)
	PCICR |= (1 << PCIE2);
	PCMSK2 |= (1 << PCINT20);
	}

void init_timer() {
	//MEDICI�N DE LA VELOCIDAD	
	
	// Configurar el Timer1 en modo CTC. �ste se usar� para medir la velocidad del motor
	TCCR1B |= (1 << WGM12);
	
	// Configurar el preescalador del Timer1
	TCCR1B |= ((1 << CS10) | (1 << CS11)); // Preescalador 64
	
	// Configurar el valor de comparaci�n para que el temporizador se desborde cada 1 ms
	OCR1A = F_CPU / (TIMER_PRESCALER * 1000) - 1; //249
	
	// Habilitar la interrupci�n de comparaci�n del Timer1
	TIMSK1 |= (1 << OCIE1A);
	
	//PWM EN TIMER0
	DDRD |= (1<<PORTD6);
	TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1<<COM0A1);
} 

void init_UART(){
	UBRR0 = F_CPU/16/BAUDRATE -1;
	UCSR0B |= (1<<RXCIE0);
	UCSR0B |= (3<<UCSZ00);
}

//FUNCIONES DE INTERRUPCI�N

// Deberian ser más cortas, para no perder otras interrupciones
ISR(PCINT2_vect) {
	// Incrementar el contador de pulsos
	if (PIND & (1 << PCINT20)) {
		pulse_count++;
		} else {
		pulse_count--;
	}
}

ISR(TIMER1_COMPA_vect) {
	// Actualizar la velocidad del motor cada vez que el temporizador se desborda (cada 1 ms)
	// Calcular la diferencia en el contador de pulsos desde la �ltima interrupci�n del temporizador
	uint16_t pulse_diff = pulse_count - prev_pulse_count;
	prev_pulse_count = pulse_count;
	
	// Calcular la velocidad del motor en RPM
	motor_speed = ((pulse_diff * 60 * TIMER_PRESCALER) / PULSES_PER_REVOLUTION) * (1000 / (F_CPU / TIMER_PRESCALER));
	//Conversion a rad/s
	w = (2*3.14159265/60)*motor_speed;
	//Actualizamos tambi�n la variable de tiempo
	t += 0.001;
}

ISR(USART_RX_vect){
	char dato;
	dato = UART_Rec();
	switch(dato){
		case ':':
			indcom = 0;
			break;
		case '\r':
			comando[indcom] = 0;
			consigna_torque = interpretar_comando();
			torque_resistente = b_visc*w;
			consigna_ia = ((consigna_torque/1000)+torque_resistente)/K;
			comando_ing = 0;
			break;
		default:
			comando[indcom] = dato;
			indcom++;
	}
}
//OTRAS FUNCIONES

//RECEPCI�N. Pensada para recibir consignas de torque
uint8_t UART_Rec(){
while(!(UCSR0A & (1<<RXCO)));
return UDR0;
}

float interpretar_comando(){
	 return atof(&comando[1]);
}

int main(void)
{	
	indcom = 0;
	float duty_cycle;
	// Inicializar el encoder y el temporizador
	init_encoder();
	init_timer();
	init_UART();	
	// Habilitar las interrupciones globales
	sei();
    while (1) 
    {
		if((!comando_ing) && (abs(consigna_ia-ia)>0.01)){
			ia0 = ia;
			t0 = t;
			Et = K*w;
			consigna_va = (consigna_ia-ia)*R_prima + Et + RA*ia;
			duty_cycle  = (consigna_va/20)*100;
			OCR0A = duty_cycle*2.55;
			Ut = consigna_va;
			It = (Ut-Et)/RA;
			ia = (ia0-It)*exp((-RA/LA)*(t-t0)) + It; //Corriente
		} else {
			comando_ing = 1;
		}
    }
}

