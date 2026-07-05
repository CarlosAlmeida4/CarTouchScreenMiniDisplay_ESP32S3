#ifndef WIFI_MANAGER_HPP
#define WIFI_MANAGER_HPP

#include "Secrets.hpp"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include <bits/stdc++.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <functional>
#include <iostream>
#include <cstddef>
#include <atomic>

#include "network_provisioning\scheme_softap.h"

#include "PipelineTypes.hpp"

class WifiManager
{
    using List = std::vector<std::string>;

    public:


    WifiManager(QueueHandle_t q): Queue_(q) {}
    //WifiManager(const WifiManager&) = delete;
    //WifiManager& operator=(const WifiManager&) = delete;
    ~WifiManager() = default;
    void initWifi();
    void setWifiConnectionFeedback(std::function<void(const std::string&)> callback);
    void setConnectionStateHandler(std::function<void(bool)> callback);
    
    void WifiConnectRequest(std::string ssid, std::string passwrd);

    private:
    
    std::mutex networkListMutex_;
    std::mutex WifiStatusMutex_;
    static constexpr std::size_t DEFAULT_SCAN_LIST_SIZE = 5;
    static constexpr auto *TAG = "WifiManager";
    esp_netif_ip_info_t ip;
    WifiManagerStatus connectionStatus_ = WifiManagerStatus::INIT;
    List availableNetworks_;
    QueueHandle_t Queue_;


    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID, //Use secrets.hpp as default
            .password = WIFI_PASS, //Use secrets.hpp as default
        },
    };

    
    static void wifiEventHandlerEntry(
        void* arg,
        esp_event_base_t event_base,
        int32_t event_id,
        void* event_data);
        
    void wifiEventHandler(
        esp_event_base_t event_base,
        int32_t event_id,
        void* event_data);
            
    void WifiManagerTask();
    void storeAPPoints();
    void changeStatus(WifiManagerStatus status);
    void WifiConnect(const std::string ssid,const std::string pwd);
    void setPendingConnectionState(bool connected);
    static void task_entry(void* arg);
    static esp_err_t  custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                   uint8_t **outbuf, ssize_t *outlen, void *priv_data);


    std::function<void(const std::string&)> m_WifiConnectionCallback;
    std::function<void(bool)> m_ConnectionStateCallback;
    std::atomic<int> pendingConnectionState_ {-1};
    std::atomic<bool> pendingScanResults_{false};
};


#endif