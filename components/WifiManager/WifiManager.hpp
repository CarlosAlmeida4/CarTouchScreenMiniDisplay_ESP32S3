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

#include "network_provisioning/scheme_softap.h"
#include <network_provisioning/manager.h>

#include "PipelineTypes.hpp"

#define PROV_DEVMODE
#define PROV_SEC2_USERNAME          "wifiprov"
#define PROV_SEC2_PWD               "abcd1234"

class WifiManager
{
    using List = std::vector<std::string>;

public:
    /// @brief Constructor - Initializes WifiManager with communication queue
    /// @param q FreeRTOS queue handle for sending WiFi status updates
    /// @return none
    WifiManager(QueueHandle_t q): Queue_(q) 
    {
        // Initialize the event handler with lambda that captures 'this'
        WifiProvEventHandler = {
        .event_cb = [](void *user_data, 
                       network_prov_cb_event_t event, 
                       void *event_data)
            {
                WifiManager* pThis = static_cast<WifiManager*>(user_data);
                if (pThis) {
                    pThis->WifiProvAppCallback(event, event_data);    
                }
            },
            .user_data = this,
        };
    }
    //WifiManager(const WifiManager&) = delete;
    //WifiManager& operator=(const WifiManager&) = delete;
    ~WifiManager() = default;
    
    /// @brief Initialize WiFi subsystem - sets up NVS, event loops, and starts provisioning/connection
    /// @param none
    /// @return none
    void initWifi();
    
    /// @brief Register callback for WiFi connection status messages (e.g., "Connected", "Disconnected")
    /// @param callback Function that receives human-readable WiFi status strings
    /// @return none
    void setWifiConnectionFeedback(std::function<void(const std::string&)> callback);
    
    /// @brief Register callback for binary connection state changes (connected/disconnected)
    /// @param callback Function receiving true=connected, false=disconnected
    /// @return none
    void setConnectionStateHandler(std::function<void(bool)> callback);
    
    /// @brief Request connection to WiFi network (validates SSID/password, then attempts connection)
    /// @param ssid Network SSID (max 32 characters)
    /// @param passwrd Network password (max 64 characters)
    /// @return none (result delivered via callback)
    void WifiConnectRequest(std::string ssid, std::string passwrd);

    /// @brief Handle WiFi provisioning events from esp-idf provisioning framework
    /// @param event Provisioning event type (NETWORK_PROV_START, NETWORK_PROV_WIFI_CRED_RECV, etc.)
    /// @param event_data Event-specific data (e.g., credentials for CRED_RECV)
    /// @return none
    void WifiProvAppCallback(network_prov_cb_event_t event, void *event_data);

private:
    // ========== MEMBERS ==========
    std::mutex networkListMutex_;           ///< Protects availableNetworks_ list
    std::mutex WifiStatusMutex_;            ///< Protects connectionStatus_ state
    static constexpr std::size_t DEFAULT_SCAN_LIST_SIZE = 5;  ///< Max WiFi APs to scan
    static constexpr auto *TAG = "WifiManager";                ///< ESP_LOG tag
    esp_netif_ip_info_t ip;                 ///< Current IP configuration
    WifiManagerStatus connectionStatus_ = WifiManagerStatus::INIT;  ///< Current WiFi state
    List availableNetworks_;                ///< Scanned WiFi networks
    QueueHandle_t Queue_;                   ///< Queue for WifiManagerPipeline updates
    EventGroupHandle_t WifiEventGroup;      ///< UNUSED - scheduled for removal
    bool isConnectedToProvisionedWifi=false; ///< True if connected to provisioned (vs temporary) network
    network_prov_event_handler_t WifiProvEventHandler;  ///< Provisioning callback handler

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,      ///< Default SSID from Secrets.hpp
            .password = WIFI_PASS,  ///< Default password from Secrets.hpp
        },
    };

    // ========== STATIC CALLBACKS (FreeRTOS/ESP-IDF) ==========
    
    /// @brief Static entry point for WiFi event handler (C callback wrapper)
    /// @param arg User context (WifiManager* this pointer)
    /// @param event_base Event category (WIFI_EVENT, IP_EVENT, etc.)
    /// @param event_id Specific event ID
    /// @param event_data Event-specific data structure
    /// @return none
    static void wifiEventHandlerEntry(
        void* arg,
        esp_event_base_t event_base,
        int32_t event_id,
        void* event_data);
    
    // ========== INSTANCE METHODS ==========
        
    /// @brief Handle WiFi and IP events (WIFI_EVENT_STA_START, WIFI_EVENT_STA_DISCONNECTED, IP_EVENT_STA_GOT_IP, etc.)
    /// @param event_base Event base (WIFI_EVENT or IP_EVENT)
    /// @param event_id Specific event ID within base
    /// @param event_data Event payload (wifi_event_sta_disconnected_t, ip_event_got_ip_t, etc.)
    /// @return none (updates state via changeStatus)
    void wifiEventHandler(
        esp_event_base_t event_base,
        int32_t event_id,
        void* event_data);

    /// @brief Register WiFi event handlers with esp_event subsystem
    /// @param none
    /// @return none (calls ESP_ERROR_CHECK for each registration)
    void registerWifiEvents();
    
    /// @brief Main WiFi manager task loop (runs every 1000ms, state machine)
    /// @param none
    /// @return none (runs forever until vTaskDelete)
    void WifiManagerTask();
    
    /// @brief WiFi provisioning task (SoftAP SSID + QR code provisioning)
    /// @param none
    /// @return none (deletes itself on completion)
    void WifiProvisioningTask();
    
    /// @brief Extract SSID list from ESP-IDF WiFi scan results
    /// @param none
    /// @return none (populates availableNetworks_ under networkListMutex_)
    void storeAPPoints();
    
    /// @brief Update WiFi connection status and notify via callback
    /// @param status New WiFiManagerStatus value
    /// @return none (thread-safe, protected by WifiStatusMutex_)
    void changeStatus(WifiManagerStatus status);
    
    /// @brief Initiate WiFi connection to SSID with password
    /// @param ssid Target SSID (max 32 chars)
    /// @param pwd Target password (max 64 chars)
    /// @return none (calls esp_wifi_set_config + esp_wifi_connect)
    void WifiConnect(const std::string ssid,const std::string pwd);
    
    /// @brief Signal pending connection state change for delivery via callback
    /// @param connected true=connected, false=disconnected (queued for safe delivery)
    /// @return none
    void setPendingConnectionState(bool connected);
    
    /// @brief Start WiFi provisioning (SoftAP + QR code authentication)
    /// @param none
    /// @return ESP_OK on success, ESP_ERR_* on failure
    esp_err_t WifiProvisioning() const;
    
    // ========== STATIC TASK ENTRY POINTS (FreeRTOS) ==========
    
    /// @brief FreeRTOS task entry point for WiFi manager task
    /// @param arg WifiManager* this pointer cast to void*
    /// @return none
    static void task_entry(void* arg);
    
    /// @brief FreeRTOS task entry point for provisioning task
    /// @param arg WifiManager* this pointer cast to void*
    /// @return none
    static void provisioning_task_entry(void* arg);
    
    /// @brief Custom provisioning data endpoint handler
    /// @param session_id Provisioning session ID
    /// @param inbuf Input buffer with custom provisioning data
    /// @param inlen Input buffer length
    /// @param outbuf Output buffer (allocated by handler, freed by framework)
    /// @param outlen Output buffer length
    /// @param priv_data Private context data
    /// @return ESP_OK on success, ESP_ERR_NO_MEM if allocation fails
    static esp_err_t  custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                   uint8_t **outbuf, ssize_t *outlen, void *priv_data);
    
    
    // ========== ASYNC STATE COMMUNICATION ==========
    std::function<void(const std::string&)> m_WifiConnectionCallback;  ///< Status message callback (e.g., "Connected", "Scanning")
    std::function<void(bool)> m_ConnectionStateCallback;  ///< Binary state callback (true=connected)
    std::atomic<int> pendingConnectionState_ {-1};  ///< Queued state for async callback (-1=none, 0=disconnected, 1=connected)
    std::atomic<bool> pendingScanResults_{false};  ///< Flag indicating WiFi scan results are ready to process
};


#endif