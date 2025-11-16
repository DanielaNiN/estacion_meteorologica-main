#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "fotoresistor.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "wifi_ap.h"
    //definicion de parametros y configuracion para el uso de uart
#define UART_PORT UART_NUM_1
#define buffer_size 1024
#define buffer_task 1024 * 2
#define UART_RX GPIO_NUM_16
#define UART_TX GPIO_NUM_17

uart_config_t uart_config = {
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
};

    //definicion de parametros para la lecura de fotoresistor
float porcentaje;
int lux;
    fotoresistor_handle_t fotorresistencia;
fotoresistor_config_t cfg = {
    .channel = ADC_CHANNEL_3,     // <-- GPIO3 (cámbialo si usas otro)
    .attenuation = ADC_ATTEN_DB_11,
    .unit = ADC_UNIT_1,
    .raw_min = 50,               
    .raw_max = 3200,            
    .num_samples = 32,
    .filter_size = 10
};
        //crear cola donde se guardan todas las variables de salida
typedef struct {
    float temperatura;
    float altura;
    float luz;
    float velocidad;
} sensor_data_t;
QueueHandle_t colaSensores;

static void comunicacion(void *pvParameters)
{
    sensor_data_t paquete;
    char json_buffer[128];

    while (1)
    {
        if (xQueueReceive(colaSensores, &paquete, portMAX_DELAY))
        {
            snprintf(json_buffer, sizeof(json_buffer),
                "{\"temperatura\": %.2f, \"altura\": %.2f, \"luz\": %.2f, \"velocidad\": %.2f}\n",
                paquete.temperatura, paquete.altura, paquete.luz, paquete.velocidad);

            uart_write_bytes(UART_PORT, json_buffer, strlen(json_buffer));
            wifi_enviar_udp(json_buffer);
        }

    }
}
//TAREA EXCLUSIVA PARA SIMULAR LOS DATOS, TEMPORAL
static void tarea_sensores(void *pvParameters)
{
    sensor_data_t paquete;

    while (1)
    {
        paquete.temperatura = fotoresistor_read_lux(fotorresistencia);
        paquete.altura =1;
        paquete.luz = fotoresistor_read_lux(fotorresistencia);
        paquete.velocidad = 1;

        xQueueSend(colaSensores, &paquete, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{   
        //creacion de cola para el uso en tareas
    colaSensores = xQueueCreate(5, sizeof(sensor_data_t));  // 5 elementos
    if (colaSensores == NULL) {
        printf("Error creando la cola\n");
    }
    fotorresistencia=fotoresistor_init(&cfg);
    //necesarios para la comunicacion por uart
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, buffer_size, buffer_size, 0, NULL, 0);
    wifi_init_softap();
        //tarea para la comunicacion
    xTaskCreatePinnedToCore(comunicacion, "udp", 4096, NULL, 5, NULL, 0);
    xTaskCreate(tarea_sensores, "tarea_sensores", 2048, NULL, 5, NULL); //SIMULACION DE DATOS

    while(1){
        porcentaje=fotoresistor_read_percentage(fotorresistencia);
        lux=fotoresistor_read_lux(fotorresistencia);
        printf("porcentaje: %.2f\n lux: %i\n",porcentaje,lux);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
