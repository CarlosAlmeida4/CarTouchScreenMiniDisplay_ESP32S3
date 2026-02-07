#include "OTAUpdater.hpp"


void OTAUpdater::initWifi()
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
        &wifiEventHandlerEntry,
        this,
        NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifiEventHandlerEntry,
        this,
        NULL));

    esp_netif_get_ip_info(
        esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),
        &ip);

    ESP_LOGI("NET", "IP: " IPSTR, IP2STR(&ip.ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi started");

}

void OTAUpdater::wifiEventHandlerEntry(
    void* arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void* event_data)
{
    auto *self = static_cast<OTAUpdater*>(arg);
    if(self)
    {
        self->wifiEventHandler(event_base,event_id,event_data);
    }
}

void OTAUpdater::wifiEventHandler(
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
        xTaskCreate(task_entry, "ota_task", 8192, this, 5, NULL);
    }
}

void OTAUpdater::task_entry(void* arg)
{
    auto* self = static_cast<OTAUpdater*>(arg);
    self->OTAUpdaterTask();
}

void OTAUpdater::OTAUpdaterTask()
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

