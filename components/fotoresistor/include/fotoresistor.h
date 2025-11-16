/**
 * @file fotoresistor.h
 * @brief Librería para lectura de fotoresistor con filtrado de ruido
 * 
 * Esta librería proporciona funciones para leer un fotoresistor conectado
 * al ADC del ESP32, con filtros de ruido y conversión a porcentaje de luz.
 * Compatible con ESP-IDF v5.x
 */

#ifndef FOTORESISTOR_H
#define FOTORESISTOR_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuración del fotoresistor
 */
typedef struct {
    adc_channel_t channel;          ///< Canal ADC a usar
    adc_atten_t attenuation;        ///< Atenuación del ADC
    adc_unit_t unit;                ///< Unidad ADC (ADC_UNIT_1 o ADC_UNIT_2)
    uint16_t raw_min;               ///< Valor raw mínimo (0% de luz) - default: 700
    uint16_t raw_max;               ///< Valor raw máximo (100% de luz) - default: 4095
    uint8_t num_samples;            ///< Número de muestras para filtrado - default: 64
    uint8_t filter_size;            ///< Tamaño del filtro de media móvil - default: 10
} fotoresistor_config_t;

/**
 * @brief Handle del fotoresistor
 */
typedef void* fotoresistor_handle_t;

/**
 * @brief Configuración por defecto del fotoresistor
 */
#define FOTORESISTOR_CONFIG_DEFAULT() {             \
    .channel = ADC_CHANNEL_4,                       \
    .attenuation = ADC_ATTEN_DB_12,                 \
    .unit = ADC_UNIT_1,                             \
    .raw_min = 700,                                 \
    .raw_max = 4095,                                \
    .num_samples = 64,                              \
    .filter_size = 10                               \
}

/**
 * @brief Inicializa el fotoresistor
 * 
 * @param config Puntero a la configuración del fotoresistor
 * @return Handle del fotoresistor o NULL si hay error
 */
fotoresistor_handle_t fotoresistor_init(const fotoresistor_config_t *config);

/**
 * @brief Lee el valor raw filtrado del ADC
 * 
 * @param handle Handle del fotoresistor
 * @return Valor raw filtrado (0-4095)
 */
uint32_t fotoresistor_read_raw(fotoresistor_handle_t handle);

/**
 * @brief Lee el voltaje del fotoresistor en mV
 * 
 * @param handle Handle del fotoresistor
 * @return Voltaje en milivoltios
 */
uint32_t fotoresistor_read_voltage(fotoresistor_handle_t handle);

/**
 * @brief Lee el porcentaje de luz
 * 
 * Calcula los lux basándose en el modelo LDR y luego convierte a porcentaje
 * usando una escala logarítmica (1 lux = 0%, 10000 lux = 100%)
 * 
 * @param handle Handle del fotoresistor
 * @return Porcentaje de luz (0.0 a 100.0)
 */
float fotoresistor_read_percentage(fotoresistor_handle_t handle);

/**
 * @brief Lee el valor de iluminación en lux
 * 
 * Calcula los lux usando el modelo LDR:
 * R_LDR = K * lux^(-ALPHA)
 * 
 * @param handle Handle del fotoresistor
 * @return Iluminación en lux (0.0 a 100000.0)
 */
float fotoresistor_read_lux(fotoresistor_handle_t handle);

/**
 * @brief Desinicializa y libera recursos del fotoresistor
 * 
 * @param handle Handle del fotoresistor a liberar
 */
void fotoresistor_deinit(fotoresistor_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // FOTORESISTOR_H