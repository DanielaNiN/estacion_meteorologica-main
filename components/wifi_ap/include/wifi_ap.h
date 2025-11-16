#ifndef WIFI_AP_H
#define WIFI_AP_H

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

// Inicializa el WiFi en modo Access Point
void wifi_init_softap(void);

// Envía un mensaje UDP al cliente conectado
void wifi_enviar_udp(const char *mensaje);

#endif
