/*#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "wifi_ap.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ===================== CONFIGURACIÓN UART ======================
#define UART_PORT UART_NUM_1
#define buffer_size 1024
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

// ===================== NTC Y FOTORESISTOR ======================
#define NTC_BETA 3950.0
#define NTC_R25 10000.0
#define SERIES_R 8900.0

#define FOTO_CHANNEL ADC_CHANNEL_3   // GPIO3
#define NTC_CHANNEL  ADC_CHANNEL_6   // GPIO34

#define RAW_MIN 50
#define RAW_MAX 3200
#define LUX_MAX 1000.0 // Ajusta según tu sensor

adc_cali_handle_t cali_handle = NULL;
adc_continuous_handle_t adc_handle = NULL;

float ntc_temperature(float Rntc) {
    float T0 = 298.15;
    float invT = (1.0 / T0) + (1.0 / NTC_BETA) * log(Rntc / NTC_R25);
    return (1.0 / invT) - 273.15;
}

// ===================== COLA DE SENSORES ======================
typedef struct {
    float temperatura;
    float altura;
    float luz;
    float velocidad;
} sensor_data_t;

QueueHandle_t colaSensores;

// ===================== TAREA COMUNICACIÓN ======================
static void comunicacion(void *pvParameters) {
    sensor_data_t paquete;
    char json_buffer[128];

    while (1) {
        if (xQueueReceive(colaSensores, &paquete, portMAX_DELAY)) {
            snprintf(json_buffer, sizeof(json_buffer),
                     "{\"temperatura\": %.2f, \"altura\": %.2f, \"luz\": %.2f, \"velocidad\": %.2f}\n",
                     paquete.temperatura, paquete.altura, paquete.luz, paquete.velocidad);

            uart_write_bytes(UART_PORT, json_buffer, strlen(json_buffer));
            wifi_enviar_udp(json_buffer);
        }
    }
}

// ===================== TAREA LECTURA SENSORES ======================
static void tarea_sensores(void *pvParameters) {
    sensor_data_t paquete;
    uint8_t buffer[256];

    while (1) {
        uint32_t length = 0;
        esp_err_t ret = adc_continuous_read(adc_handle, buffer, sizeof(buffer), &length, 10);

        float tempC = 0;
        float porcentaje = 0;
        float lux = 0;

        if (ret == ESP_OK && length >= sizeof(adc_digi_output_data_t)) {
            adc_digi_output_data_t *data = (adc_digi_output_data_t *)buffer;
            for (int i = 0; i < length / sizeof(adc_digi_output_data_t); i++) {
                int channel = data[i].type2.channel;
                int raw = data[i].type2.data;
                int voltage_mv = 0;
                adc_cali_raw_to_voltage(cali_handle, raw, &voltage_mv);
                float V = voltage_mv / 1000.0;
                float Vs = 3.3;

                if (channel == NTC_CHANNEL) {
                    float Rntc = SERIES_R * V / (Vs - V);
                    tempC = ntc_temperature(Rntc);
                } else if (channel == FOTO_CHANNEL) {
                    // Normalizar como antes
                    porcentaje = (raw - RAW_MIN) * 100.0 / (RAW_MAX - RAW_MIN);
                    if (porcentaje < 0) porcentaje = 0;
                    if (porcentaje > 100) porcentaje = 100;

                    lux = porcentaje * LUX_MAX / 100.0;
                }
            }
        }

        paquete.temperatura = tempC;
        paquete.luz = lux;
        paquete.altura = 1;
        paquete.velocidad = 1;

        xQueueSend(colaSensores, &paquete, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ===================== APP_MAIN ======================
void app_main(void) {
    // ---- COLA ----
    colaSensores = xQueueCreate(5, sizeof(sensor_data_t));

    // ---- UART ----
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, buffer_size, buffer_size, 0, NULL, 0);

    // ---- WIFI ----
    wifi_init_softap();

    // ---- CONFIG ADC CONTINUOUS ----
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 256,
    };
    adc_continuous_new_handle(&adc_config, &adc_handle);

    adc_continuous_config_t config = {
        .sample_freq_hz = 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2
    };

    static adc_digi_pattern_config_t patterns[2] = {
        {
            .atten = ADC_ATTEN_DB_11,
            .bit_width = ADC_BITWIDTH_12,
            .channel = FOTO_CHANNEL,
            .unit = ADC_UNIT_1
        },
        {
            .atten = ADC_ATTEN_DB_12,
            .bit_width = ADC_BITWIDTH_12,
            .channel = NTC_CHANNEL,
            .unit = ADC_UNIT_1
        }
    };
    config.pattern_num = 2;
    config.adc_pattern = patterns;

    adc_continuous_config(adc_handle, &config);

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle);
    adc_continuous_start(adc_handle);

    // ---- TAREAS ----
    xTaskCreatePinnedToCore(comunicacion, "udp", 4096, NULL, 5, NULL, 0);
    xTaskCreate(tarea_sensores, "tarea_sensores", 4096, NULL, 5, NULL);

    // ---- LOOP DEBUG ----
    while (1) {
        sensor_data_t paquete;
        if (xQueuePeek(colaSensores, &paquete, 0)) {
            printf("lux: %.2f %% | temperatura: %.2f °C\n", paquete.luz / LUX_MAX * 100.0, paquete.temperatura);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}*/
//main funcional fotocelda y ntc
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 10
#define I2C_MASTER_SDA_IO 9
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define BMP180_ADDR 0x77

// Registros del BMP180
#define REG_CAL_AC1 0xAA
#define REG_CONTROL 0xF4
#define REG_RESULT 0xF6
#define CMD_TEMP 0x2E
#define CMD_PRESS 0x34

// Variables de calibración
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;

// I2C write de un byte
esp_err_t i2c_write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// I2C lectura de bytes
esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (BMP180_ADDR << 1) | I2C_MASTER_READ, true);
    if(len > 1)
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Leer calibración
void bmp180_read_calibration() {
    uint8_t buf[22];
    i2c_read_bytes(REG_CAL_AC1, buf, 22);

    AC1 = (buf[0] << 8) | buf[1];
    AC2 = (buf[2] << 8) | buf[3];
    AC3 = (buf[4] << 8) | buf[5];
    AC4 = (buf[6] << 8) | buf[7];
    AC5 = (buf[8] << 8) | buf[9];
    AC6 = (buf[10] << 8) | buf[11];
    B1  = (buf[12] << 8) | buf[13];
    B2  = (buf[14] << 8) | buf[15];
    MB  = (buf[16] << 8) | buf[17];
    MC  = (buf[18] << 8) | buf[19];
    MD  = (buf[20] << 8) | buf[21];
}

// Leer temperatura cruda
int32_t bmp180_read_raw_temp() {
    i2c_write_byte(REG_CONTROL, CMD_TEMP);
    vTaskDelay(pdMS_TO_TICKS(5));
    uint8_t buf[2];
    i2c_read_bytes(REG_RESULT, buf, 2);
    return (buf[0] << 8) | buf[1];
}

// Leer presión cruda
int32_t bmp180_read_raw_pressure(uint8_t oss) {
    i2c_write_byte(REG_CONTROL, CMD_PRESS + (oss << 6));
    // Espera dependiendo de OSS
    if(oss == 0) vTaskDelay(pdMS_TO_TICKS(5));
    else if(oss == 1) vTaskDelay(pdMS_TO_TICKS(8));
    else if(oss == 2) vTaskDelay(pdMS_TO_TICKS(14));
    else vTaskDelay(pdMS_TO_TICKS(26));

    uint8_t buf[3];
    i2c_read_bytes(REG_RESULT, buf, 3);
    return ((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> (8 - oss);
}

// Calcular temperatura en °C
float bmp180_calc_temp(int32_t UT, int32_t *B5_out) {
    int32_t X1 = ((UT - AC6) * AC5) / 32768;
    int32_t X2 = (MC * 2048) / (X1 + MD);
    int32_t B5 = X1 + X2;
    *B5_out = B5;
    return ((B5 + 8) >> 4) / 10.0;
}

// Calcular presión en Pa
int32_t bmp180_calc_pressure(int32_t UP, int32_t B5, uint8_t oss) {
    int32_t B6 = B5 - 4000;
    int32_t X1 = (B2 * (B6 * B6 / 4096)) / 2048;
    int32_t X2 = AC2 * B6 / 2048;
    int32_t X3 = X1 + X2;
    int32_t B3 = (((AC1 * 4 + X3) << oss) + 2) / 4;

    X1 = AC3 * B6 / 8192;
    X2 = (B1 * (B6 * B6 / 4096)) / 65536;
    X3 = ((X1 + X2) + 2) / 4;
    int32_t B4 = AC4 * (uint32_t)(X3 + 32768) / 32768;
    int32_t B7 = ((uint32_t)UP - B3) * (50000 >> oss);

    int32_t p;
    if(B7 < 0x80000000) p = (B7 * 2) / B4;
    else p = (B7 / B4) * 2;

    X1 = (p / 256) * (p / 256);
    X1 = (X1 * 3038) / 65536;
    X2 = (-7357 * p) / 65536;
    p = p + (X1 + X2 + 3791) / 16;

    return p;
}

// Calcular altitud en metros
float bmp180_calc_altitude(float pressure) {
    return 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));
}

void app_main(void) {
    // Configurar I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    printf("Iniciando BMP180...\n");
    bmp180_read_calibration();

    while(1) {
        int32_t B5;
        int32_t UT = bmp180_read_raw_temp();
        int32_t UP = bmp180_read_raw_pressure(0); // OSS=0
        float temp = bmp180_calc_temp(UT, &B5);
        int32_t pres = bmp180_calc_pressure(UP, B5, 0);
        float alt = bmp180_calc_altitude((float)pres);

        printf("Temp: %.2f C | Pres: %ld Pa | Alt: %.2f m\n", temp, pres, alt);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
