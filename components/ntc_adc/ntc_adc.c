#include "ntc_adc.h"
#include "esp_log.h"

static const char *TAG = "NTC_ADC";

// --- Cálculo temperatura ---
float ntc_temperature(float Rntc)
{
    float T0 = 298.15;               // 25°C en Kelvin
    float invT = (1/T0) + (1/NTC_BETA) * log(Rntc / NTC_R25);
    float T = 1 / invT;
    return T - 273.15;               // °C
}

// --- Inicialización ADC (one-shot) ---
esp_err_t ntc_adc_init(ntc_adc_t *ntc, adc_oneshot_unit_handle_t adc, adc_channel_t channel, float supply_voltage)
{
    if (!ntc || !adc) return ESP_ERR_INVALID_ARG;

    ntc->adc_handle = adc;      // ADC compartido (ADC1)
    ntc->channel = channel;     // Canal específico
    ntc->Vs = supply_voltage;

    ESP_LOGI(TAG, "NTC ADC inicializado correctamente (one-shot) en canal %d", channel);
    return ESP_OK;
}

// --- Lectura de temperatura (one-shot) ---
esp_err_t ntc_adc_read(ntc_adc_t *ntc, float *temperature)
{
    if (!ntc || !temperature) return ESP_ERR_INVALID_ARG;

    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(ntc->adc_handle, ntc->channel, &raw));

    float V = (raw / 4095.0f) * ntc->Vs;
    float Rntc = SERIES_R * V / (ntc->Vs - V);

    *temperature = ntc_temperature(Rntc);

    return ESP_OK;
}
