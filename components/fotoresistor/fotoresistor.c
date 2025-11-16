/**
 * @file fotoresistor.c
 * @brief Implementación de la librería de fotoresistor
 * Compatible con ESP-IDF v5.x
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "fotoresistor.h"
#include "esp_log.h"

// Constantes del modelo LDR (ajustables según calibración)
#define K          500000.0    // constante de proporcionalidad
#define ALPHA      0.7        // exponente
#define R_FIXED    11870.0     // resistencia fija en ohmios
#define VCC        3.33        // voltaje de alimentación
#define maxima_lux 5000.0
static const char* TAG = "FOTORESISTOR";

/**
 * @brief Estructura interna del fotoresistor
 */
typedef struct {
    fotoresistor_config_t config;
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    uint32_t *filter_buffer;
    int filter_index;
    bool filter_full;
    bool cali_enabled;
} fotoresistor_t;

/**
 * @brief Inicializa la calibración del ADC
 */
static bool adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibración Curve Fitting soportada, inicializando...");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
            ESP_LOGI(TAG, "Calibración Curve Fitting inicializada");
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "Calibración Line Fitting soportada, inicializando...");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
            ESP_LOGI(TAG, "Calibración Line Fitting inicializada");
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        return true;
    } else {
        ESP_LOGW(TAG, "Calibración no disponible, usando valores raw");
        return false;
    }
}

/**
 * @brief Desinicializa la calibración del ADC
 */
static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Desregistrando esquema de calibración Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "Desregistrando esquema de calibración Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

/**
 * @brief Filtro de media móvil
 */
static uint32_t moving_average_filter(fotoresistor_t *fr, uint32_t new_sample)
{
    fr->filter_buffer[fr->filter_index] = new_sample;
    fr->filter_index = (fr->filter_index + 1) % fr->config.filter_size;
    
    if (!fr->filter_full && fr->filter_index == 0) {
        fr->filter_full = true;
    }
    
    uint32_t sum = 0;
    int count = fr->filter_full ? fr->config.filter_size : fr->filter_index;
    
    for (int i = 0; i < count; i++) {
        sum += fr->filter_buffer[i];
    }
    
    return sum / count;
}

/**
 * @brief Filtro de outliers (mediana)
 */
static uint32_t outlier_filter(uint32_t *samples, int num_samples)
{
    uint32_t *temp_array = malloc(num_samples * sizeof(uint32_t));
    if (temp_array == NULL) {
        ESP_LOGE(TAG, "Error al asignar memoria para filtro");
        return samples[0];
    }
    
    memcpy(temp_array, samples, num_samples * sizeof(uint32_t));
    
    // Ordenamiento burbuja
    for (int i = 0; i < num_samples - 1; i++) {
        for (int j = 0; j < num_samples - i - 1; j++) {
            if (temp_array[j] > temp_array[j + 1]) {
                uint32_t temp = temp_array[j];
                temp_array[j] = temp_array[j + 1];
                temp_array[j + 1] = temp;
            }
        }
    }
    
    uint32_t median = temp_array[num_samples / 2];
    free(temp_array);
    
    return median;
}

fotoresistor_handle_t fotoresistor_init(const fotoresistor_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Configuración nula");
        return NULL;
    }
    
    // Asignar memoria para la estructura
    fotoresistor_t *fr = calloc(1, sizeof(fotoresistor_t));
    if (fr == NULL) {
        ESP_LOGE(TAG, "Error al asignar memoria para fotoresistor");
        return NULL;
    }
    
    // Copiar configuración
    memcpy(&fr->config, config, sizeof(fotoresistor_config_t));
    
    // Asignar buffer para filtro
    fr->filter_buffer = calloc(fr->config.filter_size, sizeof(uint32_t));
    if (fr->filter_buffer == NULL) {
        ESP_LOGE(TAG, "Error al asignar memoria para buffer de filtro");
        free(fr);
        return NULL;
    }
    
    fr->filter_index = 0;
    fr->filter_full = false;
    
    // Configurar ADC oneshot
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = fr->config.unit,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &fr->adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar unidad ADC: %s", esp_err_to_name(ret));
        free(fr->filter_buffer);
        free(fr);
        return NULL;
    }
    
    // Configurar canal ADC
    adc_oneshot_chan_cfg_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = fr->config.attenuation,
    };
    
    ret = adc_oneshot_config_channel(fr->adc_handle, fr->config.channel, &chan_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar canal ADC: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(fr->adc_handle);
        free(fr->filter_buffer);
        free(fr);
        return NULL;
    }
    
    // Inicializar calibración
    fr->cali_enabled = adc_calibration_init(fr->config.unit, fr->config.attenuation, &fr->cali_handle);
    
    ESP_LOGI(TAG, "Fotoresistor inicializado en canal %d", fr->config.channel);
    ESP_LOGI(TAG, "Rango: %d (0%%) - %d (100%%)", fr->config.raw_min, fr->config.raw_max);
    
    return (fotoresistor_handle_t)fr;
}

uint32_t fotoresistor_read_raw(fotoresistor_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle nulo");
        return 0;
    }
    
    fotoresistor_t *fr = (fotoresistor_t *)handle;
    
    // Asignar memoria para muestras
    uint32_t *samples = malloc(fr->config.num_samples * sizeof(uint32_t));
    if (samples == NULL) {
        ESP_LOGE(TAG, "Error al asignar memoria para muestras");
        return 0;
    }
    
    // Tomar muestras
    for (int i = 0; i < fr->config.num_samples; i++) {
        int raw_value = 0;
        esp_err_t ret = adc_oneshot_read(fr->adc_handle, fr->config.channel, &raw_value);
        if (ret == ESP_OK) {
            samples[i] = (uint32_t)raw_value;
        } else {
            samples[i] = 0;
        }
    }
    
    // Aplicar filtro de outliers (mediana)
    uint32_t filtered = outlier_filter(samples, fr->config.num_samples);
    free(samples);
    
    // Aplicar filtro de media móvil
    filtered = moving_average_filter(fr, filtered);
    
    return filtered;
}

uint32_t fotoresistor_read_voltage(fotoresistor_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle nulo");
        return 0;
    }
    
    fotoresistor_t *fr = (fotoresistor_t *)handle;
    uint32_t raw = fotoresistor_read_raw(handle);
    
    if (fr->cali_enabled) {
        int voltage = 0;
        esp_err_t ret = adc_cali_raw_to_voltage(fr->cali_handle, raw, &voltage);
        if (ret == ESP_OK) {
            return (uint32_t)voltage;
        }
    }
    
    // Si no hay calibración, estimar voltaje (aproximación)
    // Para ADC_ATTEN_DB_12: rango aproximado 0-3100mV
    return (raw * 3100) / 4095;
}

float fotoresistor_read_percentage(fotoresistor_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle nulo");
        return 0.0f;
    }
    
    fotoresistor_t *fr = (fotoresistor_t *)handle;
    
    // Obtener voltaje medido
    uint32_t voltage_mv = fotoresistor_read_voltage(handle);
    double v_out = voltage_mv / 1000.0; // Convertir mV a V
    
    // Calcular resistencia de la LDR desde el divisor resistivo
    // V_out = VCC * (R_FIXED / (R_FIXED + R_LDR))
    // Despejando R_LDR:
    // R_LDR = R_FIXED * ((VCC / V_out) - 1)
    
    if (v_out <= 0.001) { // Evitar división por cero
        ESP_LOGW(TAG, "Voltaje muy bajo, imposible calcular lux");
        return 0.0f;
    }
    
    double r_ldr = R_FIXED * ((VCC / v_out) - 1.0);
    
    // Calcular lux desde la resistencia
    // R_LDR = K * lux^(-ALPHA)
    // Despejando lux:
    // lux = (K / R_LDR)^(1/ALPHA)
    
    if (r_ldr <= 0) {
        ESP_LOGW(TAG, "Resistencia LDR negativa o cero");
        return 0.0f;
    }
    
    float lux = pow(K / r_ldr, 1.0 / ALPHA);
    
    // Limitar valores extremos
    if (lux < 0) lux = 0;
    if (lux > 100000) lux = 100000;
    
    // Convertir a porcentaje (0-100%)
    // Escala logarítmica: 1 lux = 0%, 10000 lux = 100%
    float percentage;
    if (lux <= 1.0) {
        percentage = 0.0f;
    } else if (lux >= 10000.0) {
        percentage = 100.0f;
    } else {
        // Escala logarítmica entre 1 y 10000 lux
        percentage = (lux*100)/maxima_lux;
    }
    
    //printf("V_out: %.3fV, R_LDR: %.1fΩ, Lux: %.1f, Porcentaje: %.2f%%\n", 
      //       v_out, r_ldr, lux, percentage);
    
    return percentage;
}

float fotoresistor_read_lux(fotoresistor_handle_t handle)
{
    if (handle == NULL) {
        ESP_LOGE(TAG, "Handle nulo");
        return 0.0f;
    }
    
    // Obtener voltaje medido
    uint32_t voltage_mv = fotoresistor_read_voltage(handle);
    double v_out = voltage_mv / 1000.0; // Convertir mV a V
    
    // Evitar división por cero
    if (v_out <= 0.001) {
        return 0.0f;
    }
    
    // Calcular resistencia de la LDR
    double r_ldr = R_FIXED * ((VCC / v_out) - 1.0);
    
    if (r_ldr <= 0) {
        return 0.0f;
    }
    //ESP_LOGI(TAG, "Vout=%.3fV  R_LDR=%.1fΩ", v_out, r_ldr);
    // Calcular lux
    double lux = pow(K / r_ldr, 1.0 / ALPHA);
    
    // Limitar valores extremos
    if (lux < 0) lux = 0;
    if (lux > 100000) lux = 100000;
    
    return (float)lux;
}

void fotoresistor_deinit(fotoresistor_handle_t handle)
{
    if (handle == NULL) {
        return;
    }
    
    fotoresistor_t *fr = (fotoresistor_t *)handle;
    
    // Desinicializar calibración
    if (fr->cali_enabled) {
        adc_calibration_deinit(fr->cali_handle);
    }
    
    // Eliminar unidad ADC
    if (fr->adc_handle != NULL) {
        adc_oneshot_del_unit(fr->adc_handle);
    }
    
    // Liberar buffer de filtro
    if (fr->filter_buffer != NULL) {
        free(fr->filter_buffer);
    }
    
    free(fr);
    
    ESP_LOGI(TAG, "Fotoresistor desinicializado");
}