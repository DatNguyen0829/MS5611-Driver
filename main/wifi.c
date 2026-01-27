// main.c
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"

#include "esp_system.h"

#define WIFI_SSID "iPhone"
#define WIFI_PASS "datdeptrai"

#define TAG "telemetry"

// Put target IP here if you want unicast; leave as "255.255.255.255" to broadcast
#define GROUND_STATION_IP "255.255.255.255"
#define GROUND_STATION_PORT 5005

static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

/* --- Wi-Fi event handler --- */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi started, connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Disconnected from AP, retrying...");
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* --- Wi-Fi initialization (station) --- */
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register handlers */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = { 0 };
    strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* --- UDP telemetry sender task --- */
static void telemetry_task(void *arg)
{
    // Wait for Wi-Fi
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    int sock = -1;
    struct sockaddr_in dest_addr;
    bool use_broadcast = false;

    // Setup destination
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(GROUND_STATION_PORT);

    if (strcmp(GROUND_STATION_IP, "255.255.255.255") == 0) {
        use_broadcast = true;
        dest_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    } else {
        if (inet_pton(AF_INET, GROUND_STATION_IP, &dest_addr.sin_addr) != 1) {
            ESP_LOGE(TAG, "Invalid GROUND_STATION_IP");
            vTaskDelete(NULL);
            return;
        }
    }

    while (1) {
        // Create socket
        if (sock < 0) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock < 0) {
                ESP_LOGE(TAG, "Failed to create socket");
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            // Allow broadcast if needed
            if (use_broadcast) {
                int broadcast_enable = 1;
                if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
                    ESP_LOGW(TAG, "Failed to set SO_BROADCAST");
                }
            }
            // Optional: set a send timeout
            struct timeval tv;
            tv.tv_sec = 2;
            tv.tv_usec = 0;
            setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        }

        // If Wi-Fi disconnected, wait and then continue (socket will be recreated if needed)
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
        if ((bits & WIFI_CONNECTED_BIT) == 0) {
            ESP_LOGW(TAG, "Wi-Fi not connected, waiting...");
            if (sock >= 0) {
                close(sock);
                sock = -1;
            }
            xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
            continue;
        }

        static uint32_t counter = 0;
        uint64_t uptime_ms = esp_timer_get_time() / 1000ULL;

        // Build JSON telemetry (small)
        char msg[128];
        int len = snprintf(msg, sizeof(msg),
                           "{\"counter\":%u,\"uptime_ms\":%llu}\n",
                           (unsigned)counter, (unsigned long long)uptime_ms);
        if (len <= 0) {
            ESP_LOGE(TAG, "Failed to format telemetry");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Send
        int sent = sendto(sock, msg, len, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (sent < 0) {
            ESP_LOGW(TAG, "sendto failed, closing socket and will retry");
            close(sock);
            sock = -1;
        } else {
            ESP_LOGI(TAG, "Sent telemetry: %.*s", sent, msg);
            counter++;
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // send every 1 second
    }

    // never reached
    if (sock >= 0) close(sock);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting telemetry example");

    wifi_init_sta();

    // Wait for Wi-Fi, then start telemetry task
    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
}
