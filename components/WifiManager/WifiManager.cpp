#include "WifiManager.hpp"

inline void WifiManager::changeStatus(WifiManagerStatus status)
{
    if(status)
    {connectionStatus_ = status;}
}


void WifiManager::task_entry(void* arg)
{
    auto* self = static_cast<WifiManager*>(arg);
    self->WifiManagerTask(); 
}

void WifiManager::WifiManagerTask()
{
    
    assert(Queue_!=nullptr);
    
    while(1)
    {
        switch (connectionStatus_)
        {
            case WifiManagerStatus::CONNECTED:
                vTaskDelay(100 / portTICK_PERIOD_MS);
                break;

            case WifiManagerStatus::SCANNING_FINISHED:
                {            
                    std::lock_guard<std::mutex> lock(networkListMutex_);
                    if(!availableNetworks_.empty())
                    {
                        std::string dropdownDisplay;
                        dropdownDisplay.clear();
                        for (size_t i = 0; i < availableNetworks_.size(); ++i)
                        {
                            dropdownDisplay += availableNetworks_[i];
                            if (i + 1 < availableNetworks_.size())
                                dropdownDisplay += '\n';
                        }

                        WifiManagerPipeline WifiMgrPip{};
                        WifiMgrPip.WifiStatus = connectionStatus_;
                        
                        auto len = std::min<size_t>(dropdownDisplay.size(),(sizeof(WifiMgrPip.AvailableNetworks) - 1));
                        std::memcpy(WifiMgrPip.AvailableNetworks,dropdownDisplay.c_str(),len);

                        xQueueOverwrite(Queue_,&WifiMgrPip);
                    }

                } 
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;

            case WifiManagerStatus::SCANNING_READY: //TODO! : implement routine to jump to scanning ready
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                break;
                
            default:
                break;
        }


    }

}

void WifiManager::initWifi()
{
    changeStatus(WifiManagerStatus::INIT);
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

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
    esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),&ip);

    ESP_LOGI("NET", "IP: " IPSTR, IP2STR(&ip.ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    //Do initial wifi networks scan
    changeStatus(WifiManagerStatus::SCANNING);
    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL,true));

    //Initialize cyclic wifi task
    xTaskCreate(task_entry,"Wifi Manager",4096,this,2,NULL);
    //TODO: this needs to be called to set the wifi connection
    
}

void WifiManager::storeAPPoints()
{
    changeStatus(WifiManagerStatus::SCANNING_FINISHED);
    uint16_t ap_count = 0;
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    
    ESP_LOGI(TAG, "Max AP number ap_info can hold = %u", number);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    number = ap_count;

    std::vector<wifi_ap_record_t> ap_info(ap_count);
    
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info.data()));

    std::lock_guard<std::mutex> lock(networkListMutex_);
    availableNetworks_.clear();
    ESP_LOGI(TAG, "Total APs scanned = %u, actual AP number ap_info holds = %u", ap_count, number);
    for (auto it = ap_info.begin();it!=ap_info.end();++it) {
        ESP_LOGI(TAG, "SSID \t\t%s", it->ssid);
        ESP_LOGI(TAG, "RSSI \t\t%d", it->rssi);
        const char* CurrSSID = reinterpret_cast<const char*>(it->ssid);

        availableNetworks_.emplace_back(CurrSSID);
    }
   
}

void WifiManager::WifiConnect(std::string ssid, std::string passwrd)
{
    ESP_LOGI(TAG, "SSID \t\t%s", ssid.c_str());
    ESP_LOGI(TAG, "Password \t\t%s", passwrd.c_str());

    if(ssid.empty() || passwrd.empty())
    {
        //TODO! m_WifiConnectionCallback("Empty Field");
        return;
    }

    size_t ssid_len = std::min(ssid.size(), sizeof(wifi_config.sta.ssid) - 1);
    size_t pwd_len = std::min(passwrd.size(), sizeof(wifi_config.sta.password) - 1);
    
    std::memcpy(wifi_config.sta.ssid,ssid.data(),ssid_len);
    std::memcpy(wifi_config.sta.password,passwrd.data(),pwd_len);
    
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_connect();
}

void WifiManager::wifiEventHandlerEntry(
    void* arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void* event_data)
{
    auto *self = static_cast<WifiManager*>(arg);

    ESP_LOGI("STACK", "High water mark: %u",
         uxTaskGetStackHighWaterMark(NULL));
    if(self)
    {
        self->wifiEventHandler(event_base,event_id,event_data);
    }
}

void WifiManager::wifiEventHandler(
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void* event_data)
{
    //ESP_LOGI(TAG, "Event Base: %d", event_base);
    if (event_base == WIFI_EVENT) {

        switch (event_id) {

        case WIFI_EVENT_STA_START:
            ESP_LOGI(TAG, "WiFi started");
            
            //esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected, retrying...");
            //TODO! m_WifiConnectionCallback("Disconnected");
            esp_wifi_connect();
            break;

        case WIFI_EVENT_SCAN_DONE:
            ESP_LOGI(TAG, "Wifi Scan Finished");
            //TODO! m_WifiConnectionCallback("Scanning");
            storeAPPoints();
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

        connectionStatus_ = WifiManagerStatus::READY;
    }
}