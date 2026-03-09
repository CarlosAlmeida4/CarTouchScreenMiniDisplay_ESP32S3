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
    static void task_entry(void* arg);

    std::function<void(const std::string&)> m_WifiConnectionCallback;
    std::function<void(bool)> m_ConnectionStateCallback;
};


#endif