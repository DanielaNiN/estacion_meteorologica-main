#ifndef NTC_ADC_H
#define NTC_ADC_H

#include <stdio.h>
#include "esp_adc/adc_oneshot.h"
#include <math.h>

// Configuración del NTC
#define NTC_BETA 3950.0
#define NTC_R25 10000.0
#define SERIES_R 8900.0

typedef struct {
    adc_oneshot_unit_handle_t adc_handle; // Handle ADC one-shot
    adc_channel_t channel;                // Canal ADC
    float Vs;                             // Voltaje de alimentación
} ntc_adc_t;

// Inicializa ADC en modo one-shot
esp_err_t ntc_adc_init(ntc_adc_t *ntc, adc_oneshot_unit_handle_t adc, adc_channel_t channel, float supply_voltage);

// Lee temperatura
esp_err_t ntc_adc_read(ntc_adc_t *ntc, float *temperature);

// Cálculo de temperatura
float ntc_temperature(float Rntc);

#endif
