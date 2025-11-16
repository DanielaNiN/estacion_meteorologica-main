#include "wifi_ap.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include <string.h>

static const char *TAG = "wifi softAP";

// Cliente MATLAB normalmente queda en 192.168.4.2
#define CLIENT_IP   "192.168.4.2"
#define CLIENT_PORT 5005

// -------------------------------------------------------------------
// MANEJADOR DE EVENTOS DEL AP
// -------------------------------------------------------------------
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_AP_START:
        ESP_LOGI(TAG, "AP Started");
        break;

    case WIFI_EVENT_AP_STOP:
        ESP_LOGI(TAG, "AP Stopped");
        break;

    case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) event_data;
        ESP_LOGI(TAG,
                 "Client connected: MAC %02x:%02x:%02x:%02x:%02x:%02x, AID=%d",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5],
                 event->aid);
        break;
    }

    case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) event_data;
        ESP_LOGI(TAG,
                 "Client disconnected: MAC %02x:%02x:%02x:%02x:%02x:%02x, AID=%d",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5],
                 event->aid);
        break;
    }

    default:
        break;
    }
}

// -------------------------------------------------------------------
// INICIALIZA EL SOFT-AP (SSID ESTACION)
// -------------------------------------------------------------------
void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Registra el handler de eventos
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESTACION",
            .ssid_len = 0,
            .channel = 1,
            .password = "12345678",
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };

    if (strlen((char *)wifi_config.ap.password) == 0)
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "AP listo: SSID=%s  PASS=%s",
             wifi_config.ap.ssid, wifi_config.ap.password);
}

// -------------------------------------------------------------------
// ENVÍA UN MENSAJE AL PUERTO UDP 5005 (HACIA MATLAB/PC)
// -------------------------------------------------------------------
void wifi_enviar_udp(const char *mensaje)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Error creando socket UDP");
        return;
    }

    struct sockaddr_in dest_addr = {0};
    dest_addr.sin_addr.s_addr = inet_addr(CLIENT_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(CLIENT_PORT);

    int sent = sendto(sock, mensaje, strlen(mensaje), 0,
                      (struct sockaddr *)&dest_addr, sizeof(dest_addr));

    if (sent < 0) {
        ESP_LOGE(TAG, "Error enviando UDP");
    } else {
        ESP_LOGI(TAG, "UDP enviado (%d bytes) -> %s:%d",
                 sent, CLIENT_IP, CLIENT_PORT);
    }

    close(sock);
}
