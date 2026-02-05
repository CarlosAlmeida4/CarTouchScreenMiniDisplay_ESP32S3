#include "System.hpp"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define WIFI_SSID "Vodafone-8D12E4"
#define WIFI_PASS "KtcjA3dAgwRqFHCt"

static const char *TAG = "wifi";

#define OTA_URL "http://192.168.1.72:8000/CarTouchScreenMiniDisplay_ESP32S3.bin"

void ota_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Starting OTA...");

    esp_http_client_config_t config = {
        .url = OTA_URL,
        .timeout_ms = 10000,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .skip_cert_common_name_check = true,
    };

    esp_https_ota_config_t otaConfig = {
        .http_config = &config,
    };


    esp_err_t ret = esp_https_ota(&otaConfig);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA successful, rebooting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed");
    }

    vTaskDelete(NULL);
}

static void wifi_event_handler(void* arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void* event_data)
{
    if (event_base == WIFI_EVENT) {

        switch (event_id) {

        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi started, connecting...");
            esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected, retrying...");
            esp_wifi_connect();
            break;

        default:
            break;
        }
    }

    if (event_base == IP_EVENT &&
        event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;

        ESP_LOGI("NET", "IP: " IPSTR,
                 IP2STR(&event->ip_info.ip));

        // Start OTA only after IP is obtained
        xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, NULL);
    }
}

void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    // Register events BEFORE starting WiFi
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        NULL));


    esp_netif_ip_info_t ip;
    esp_netif_get_ip_info(
        esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),
        &ip);

    ESP_LOGI("NET", "IP: " IPSTR, IP2STR(&ip.ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi started");
}

extern "C" void app_main(void)
{
    static System system;
    system.start();

    wifi_init_sta();

    // wait for Wi-Fi to stabilize
    //vTaskDelay(pdMS_TO_TICKS(10000));

    //xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, NULL);
}