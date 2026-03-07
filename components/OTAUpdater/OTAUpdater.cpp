#include "OTAUpdater.hpp"
#define OTA_URL "http://192.168.1.72:8000/CarTouchScreenMiniDisplay_ESP32S3.bin"

void OTAUpdater::triggerUpdate()
{
    ESP_LOGI(TAG, "OTA update requested from UI");
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
    {
        otaStatus = OTAUpdater::READY;
        xTaskCreate(task_entry, "ota_task", 8192, this, 5, NULL);
    }
    else
    {
        if(m_SWUpdateFeedbackCallback){m_SWUpdateFeedbackCallback("Error");}
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
    
    if(OTAUpdater::READY == otaStatus)
    {
        if(m_SWUpdateFeedbackCallback){m_SWUpdateFeedbackCallback("Started");}
    }
    else if(OTAUpdater::UPDATE_FAILED == otaStatus)
    {
        if(m_SWUpdateFeedbackCallback){m_SWUpdateFeedbackCallback("Retrying");}
    }

    otaStatus = OTAUpdater::UPDATING;

    esp_http_client_config_t config = {
        .url = OTA_URL,
        .timeout_ms = 10000,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,
        .skip_cert_common_name_check = true,
    };

    esp_https_ota_config_t otaConfig = {
        .http_config = &config,
    };

    if(m_SWUpdateFeedbackCallback){m_SWUpdateFeedbackCallback("updating");}
    esp_err_t ret = esp_https_ota(&otaConfig);

    if (ret == ESP_OK) {
        
        ESP_LOGI(TAG, "OTA successful, rebooting...");
        if(m_SWUpdateFeedbackCallback){m_SWUpdateFeedbackCallback("Rebooting");}
        otaStatus = OTAUpdater::UPDATE_FINISHED;
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA failed");
        if(m_SWUpdateFeedbackCallback){m_SWUpdateFeedbackCallback("Update Failed");}
        otaStatus = OTAUpdater::UPDATE_FAILED;
    }

    vTaskDelete(NULL);
}

void OTAUpdater::setSWUpdateFeedback(std::function<void(const std::string&)> callback)
{
    m_SWUpdateFeedbackCallback = std::move(callback);
}