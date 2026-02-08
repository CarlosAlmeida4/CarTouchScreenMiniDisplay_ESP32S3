#ifndef OTA_UPDATER_HPP
#define  OTA_UPDATER_HPP

#include "Secrets.hpp"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_system.h"
#include <bits/stdc++.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


class OTAUpdater {

public:

    OTAUpdater() = default;


    enum OTAStatus {
        INIT,
        READY,
        UPDATING,
        UPDATE_FAILED,
        UPDATE_FINISHED
    };

    void initWifi();
    
    /**
     * @brief Start OTA update process
     * Creates a task that handles the OTA download and update
     */
    void triggerUpdate();

    void setSWUpdateFeedback(std::function<void(const std::string&)> callback);
    

private:
    esp_netif_ip_info_t ip;

    OTAStatus otaStatus = OTAStatus::INIT;

    static constexpr auto *TAG = "OTAUpdater";

    void OTAUpdaterTask();

    static void task_entry(void* arg);

    static void wifiEventHandlerEntry(
        void* arg,
        esp_event_base_t event_base,
        int32_t event_id,
        void* event_data);

    void wifiEventHandler(
                        esp_event_base_t event_base,
                        int32_t event_id,
                        void* event_data);

    std::function<void(const std::string&)> m_SWUpdateFeedbackCallback;
};




#endif