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

#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASS "YOUR_PASS"

static const char *TAG = "wifi";

#define OTA_URL "http://YOUR_SERVER/firmware.bin"

void ota_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Starting OTA...");

    esp_http_client_config_t config = {
        .url = OTA_URL,
        .timeout_ms = 10000,
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
    vTaskDelay(pdMS_TO_TICKS(5000));

    xTaskCreate(ota_task, "ota_task", 8192, NULL, 5, NULL);
}