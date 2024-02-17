/*
 * Proyecto Final Microcontroladores
 * Controlador en casacada de Torque Motor DC
 *
 * Autores:
 * - Salazar, Rafael
 * - Mercado, Tomás
 */

// Elegir el dispositivo a utilizar
#define DEVICE_ESP32
// #define DEVICE_ATMEGA328P

#ifdef DEVICE_ESP32

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "Main";

#define PIN_ENCODER GPIO_NUM_48
#define PIN_PWM GPIO_NUM_19
#define PIN_DIR GPIO_NUM_20

#define UART_NUMERO UART_NUM_0
#define BUF_SIZE 1024
#define TASK_MEMORY 2 * 1024
#endif

#ifdef DEVICE_ATMEGA328P

// INCLUDES
#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
// Defino el tipo de dato esp_err_t como un void para que sea compatible
typedef void esp_err_t;

#endif

// Definiciones
#define BAUDRATE 9600

esp_err_t init_encoder();
esp_err_t init_timer();
esp_err_t init_UART();
void encoder_isr_handler(void *arg);

// Variables Globales
uint16_t intervalo = 1000;                    // Intervalo de tiempo en ms
uint8_t ranuras = 48;                       // Ranuras del encoder
uint16_t pulse_count = 0;                   // Contador de pulsos
volatile uint16_t prev_pulse_count = 0;     // Contador de pulsos previo
volatile uint16_t timer_overflow_count = 0; // Contador de desbordamiento del temporizador
volatile uint16_t motor_speed = 0;          // Velocidad del motor en revoluciones por minuto (RPM)
volatile uint8_t comando_ing = 1;

float w = 0; // Velocidad angular del motor en rad/s

#ifdef DEVICE_ATMEGA328P
#define F_CPU 16000000 // Frecuencia del microcontrolador
#endif

#ifdef DEVICE_ESP32
TimerHandle_t xTimers;
static QueueHandle_t uart_queue;

static void uart_task(void *pvParameters);
static void vTimerCallback(TimerHandle_t xTimer);
#endif

void app_main(void)
{
    // Inicializar la UART
    // init_UART();
#ifdef DEVICE_ESP32
    // Habilito el servicio de interrupciones, cada funcion puede registrar su propia interrupcion de GPIO
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
#endif
#ifdef DEVICE_ATMEGA328P
    sei();
#endif
    // Inicializar el encoder
    init_encoder();
    // Inicializar el temporizador
    init_timer();
    while (1)
    {
        ESP_LOGI(TAG, "Velocidad: %d RPM", motor_speed);
        ESP_LOGI(TAG, "Pulsos contados: %d ", pulse_count);
        ESP_LOGI(TAG, "Pulsos anteriores: %d ", prev_pulse_count);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
}

#ifdef DEVICE_ESP32
static void uart_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE); // Se reservan 1024 bytes para el buffer
    while (1)
    {
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
        {
        }
    }
}
#endif

void encoder_isr_handler(void *arg)
{
    pulse_count++;
}

void vTimerCallback(TimerHandle_t xTimer)
{
    // Actualizar la velocidad del motor cada vez que el temporizador se desborda
    // Calcular la diferencia en el contador de pulsos desde la ultima interrupcion del temporizador
    uint16_t pulse_diff = pulse_count - prev_pulse_count; 

    //El calculo lo paso a otra task para que no se bloquee el timer.
    // Calcular la velocidad del motor en RPM. Se pasa de ms a min y hay que dividir por el numero de ranuras
    motor_speed = (pulse_diff / intervalo) * (60000 / ranuras);

    prev_pulse_count = pulse_count;
    // Conversion a rad/s con 6 cifras de pi_v: 3.1415
    w = (2 * 3.14159265 / 60) * motor_speed;
}

esp_err_t init_encoder()
{
#ifdef DEVICE_ATMEGA328P
    // Configurar el PIN 4 (PCINT4) como entrada
    DDRD &= ~(1 << PCINT20);

    // Habilitar pull-up interno en el PIN 4
    PORTD |= (1 << PCINT20);

    // Habilitar interrupci�n por cambio de nivel en PCINT20 (PIN 4)
    PCICR |= (1 << PCIE2);
    PCMSK2 |= (1 << PCINT20);
#endif
#ifdef DEVICE_ESP32
    // Configurar el PIN_ENCODER como entrada
    gpio_reset_pin(PIN_ENCODER);
    gpio_set_direction(PIN_ENCODER, GPIO_MODE_INPUT);
    // Habilito una interrupcion por flanco de bajada
    gpio_set_intr_type(PIN_ENCODER, GPIO_INTR_NEGEDGE);
    gpio_intr_enable(PIN_ENCODER);

    gpio_isr_handler_add(PIN_ENCODER, encoder_isr_handler, NULL);
    return ESP_OK;
#endif
}
esp_err_t init_timer()
{
#ifdef DEVICE_ATMEGA328P
    // Configurar el Timer1 en modo CTC. �ste se usar� para medir la velocidad del motor
    TCCR1B |= (1 << WGM12);

    // Configurar el preescalador del Timer1
    TCCR1B |= ((1 << CS10) | (1 << CS11)); // Preescalador 64

    // Configurar el valor de comparaci�n para que el temporizador se desborde cada 1 ms
    OCR1A = F_CPU / (TIMER_PRESCALER * 1000) - 1; // 249

    // Habilitar la interrupci�n de comparaci�n del Timer1
    TIMSK1 |= (1 << OCIE1A);

    // PWM EN TIMER0
    DDRD |= (1 << PORTD6);
    TCCR0A |= (1 << WGM00) | (1 << WGM01) | (1 << COM0A1);
#endif
#ifdef DEVICE_ESP32
    uint8_t TimerID = 1;
    xTimers = xTimerCreate("Timer Encoder",          // Just a text name, not used by the kernel.
                           pdMS_TO_TICKS(intervalo), // The timer period in ticks.
                           pdTRUE,                   // The timers will auto-reload themselves when they expire.
                           (void *)TimerID,          // Assign an unique id.
                           vTimerCallback            // Each timer calls the same callback when it expires.
    );

    if (xTimers == NULL)
    {
        // The timer was not created.
        ESP_LOGE(TAG, "No se pudo crear el timer");
    }
    else
    {
        // Start the timer.  No block time is specified, and even if one was
        // it would be ignored because the scheduler has not yet been
        // started.
        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            // The timer could not be set into the Active state.
            ESP_LOGE(TAG, "El timer no pudo ser iniciado");
        }
    }
    return ESP_OK;
#endif
}
esp_err_t init_UART()
{
#ifdef DEVICE_ATMEGA328P
    UBRR0 = F_CPU / 16 / BAUDRATE - 1;
    UCSR0B |= (1 << RXCIE0);
    UCSR0B |= (3 << UCSZ00);
#endif
#ifdef DEVICE_ESP32
    // 1º Configurar los parametros de la UART
    uart_config_t uart_config = {
        .baud_rate = BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB};

    ESP_ERROR_CHECK(uart_param_config(UART_NUMERO, &uart_config));
    // 2º Configurar los pines de la UART
    ESP_ERROR_CHECK(uart_set_pin(UART_NUMERO, -1, -1, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    // 3º Instalar el driver de la UART
    ESP_ERROR_CHECK(uart_driver_install(UART_NUMERO, 2 * BUF_SIZE, 2 * BUF_SIZE, 20, &uart_queue, 0));

    xTaskCreate(uart_task, "uart_task", TASK_MEMORY, NULL, 5, NULL);
    ESP_LOGI(TAG, "UART inicializada");
    return ESP_OK;
#endif
}