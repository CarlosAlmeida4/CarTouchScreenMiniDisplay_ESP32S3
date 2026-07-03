#include "WifiManager.hpp"

inline void WifiManager::changeStatus(WifiManagerStatus status)
{   
    std::lock_guard<std::mutex> lock(WifiStatusMutex_);
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
        int pendingState = pendingConnectionState_.exchange(-1);
        if (pendingState != -1 && m_ConnectionStateCallback) {
            m_ConnectionStateCallback(pendingState == 1);
        }

        /*ESP_LOGI(TAG, "Wifi Status %d", connectionStatus_);*/
        switch (connectionStatus_)
        {
            case WifiManagerStatus::CONNECTED:
                if(m_WifiConnectionCallback)m_WifiConnectionCallback("Connected");
                break;

            case WifiManagerStatus::SCANNING_FINISHED:
                {            
                    std::lock_guard<std::mutex> lock(networkListMutex_);
                    // Safe to log here - this runs in WiFi Manager task with plenty of stack
                    ESP_LOGI(TAG, "Wifi Scan found %u networks", availableNetworks_.size());
                    
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
                //Check if any of the found networks is a known network  
                for(auto currSSID:availableNetworks_)
                {
                    if(WIFI_AP.contains(currSSID))
                    {
                        ESP_LOGI(TAG, "Known SSID %s", currSSID.c_str());
                        changeStatus(WifiManagerStatus::READY_TO_CONNECT);
                        //Known currSSID, connect to it
                        WifiConnect(currSSID,WIFI_AP[currSSID]);
                        break;
                    }
                }
    
                if(connectionStatus_!=WifiManagerStatus::READY_TO_CONNECT && connectionStatus_!=WifiManagerStatus::CONNECTING 
                    && connectionStatus_!=WifiManagerStatus::CONNECTED)
                {
                    //If no known wifi was found, restart searching
                    changeStatus(WifiManagerStatus::SCANNING_READY);
                    
                }
                
                break;

            case WifiManagerStatus::SCANNING_READY:
                {
                    changeStatus(WifiManagerStatus::SCANNING);
                    esp_err_t errScan = esp_wifi_scan_start(NULL, true);
                    if (errScan == ESP_ERR_WIFI_STATE)
                    {
                        // If auto-connect is already in progress, do not abort.
                        ESP_LOGW(TAG, "Initial scan skipped: STA is connecting");
                        //changeStatus(WifiManagerStatus::CONNECTING);
                    }
                    else
                    {
                        ESP_ERROR_CHECK(errScan);
                    }
                    }
                break;
            case WifiManagerStatus::DISCONNECTED:
                {
                    changeStatus(WifiManagerStatus::SCANNING_READY);
                }
                break;
            case INIT:
            case READY_TO_CONNECT:
            case CONNECTING:
            case SCANNING:
            case CONNECTION_FAILED:
            default:
                break;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }

}

static void get_device_service_name(char *service_name, size_t max)
{
    uint8_t eth_mac[6];
    const char *ssid_prefix = "PROV_";
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    snprintf(service_name, max, "%s%02X%02X%02X",
             ssid_prefix, eth_mac[3], eth_mac[4], eth_mac[5]);
}

/* Handler for the optional provisioning endpoint registered by the application.
 * The data format can be chosen by applications. Here, we are using plain ascii text.
 * Applications can choose to use other formats like protobuf, JSON, XML, etc.
 * Note that memory for the response buffer must be allocated using heap as this buffer
 * gets freed by the protocomm layer once it has been sent by the transport layer.
 */
 esp_err_t  WifiManager::custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                   uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    if (inbuf) {
        ESP_LOGI(TAG, "Received data: %.*s", inlen, (char *)inbuf);
    }
    char response[] = "SUCCESS";
    *outbuf = (uint8_t *)strdup(response);
    if (*outbuf == NULL) {
        ESP_LOGE(TAG, "System out of memory");
        return ESP_ERR_NO_MEM;
    }
    *outlen = strlen(response) + 1; /* +1 for NULL terminating byte */

    return ESP_OK;
}

void WifiManager::initWifi()
{
    changeStatus(WifiManagerStatus::INIT);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    //Initialize Wifi AP
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register events BEFORE starting WiFi
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        NETWORK_PROV_EVENT,
        ESP_EVENT_ANY_ID,
        &wifiEventHandlerEntry,
        this,
        NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        PROTOCOMM_SECURITY_SESSION_EVENT,
        ESP_EVENT_ANY_ID,
        &wifiEventHandlerEntry,
        this,
        NULL));


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

    network_prov_mgr_config_t NetProvMgr = 
    {
        .scheme = network_prov_scheme_softap,
        .scheme_event_handler = NETWORK_PROV_EVENT_HANDLER_NONE
    };

    ESP_ERROR_CHECK(network_prov_mgr_init(NetProvMgr));

    bool provisioned = false;

    /* Let's find out if the device is provisioned */
    ESP_ERROR_CHECK(network_prov_mgr_is_wifi_provisioned(&provisioned)); 

        /* If device is not yet provisioned start provisioning service */
    if (!provisioned) {
        ESP_LOGI(TAG, "Starting provisioning");

        /* What is the Device Service Name that we want
         * This translates to :
         *     - Wi-Fi SSID when scheme is network_prov_scheme_softap
         *     - device name when scheme is network_prov_scheme_ble
         */
        char service_name[12];
        get_device_service_name(service_name, sizeof(service_name));


        /* What is the security level that we want (0, 1, 2):
         *      - NETWORK_PROV_SECURITY_0 is simply plain text communication.
         *      - NETWORK_PROV_SECURITY_1 is secure communication which consists of secure handshake
         *          using X25519 key exchange and proof of possession (pop) and AES-CTR
         *          for encryption/decryption of messages.
         *      - NETWORK_PROV_SECURITY_2 SRP6a based authentication and key exchange
         *        + AES-GCM encryption/decryption of messages
         */
        network_prov_security_t security = NETWORK_PROV_SECURITY_0;


        /* What is the service key (could be NULL)
         * This translates to :
         *     - Wi-Fi password when scheme is network_prov_scheme_softap
         *          (Minimum expected length: 8, maximum 64 for WPA2-PSK)
         *     - simply ignored when scheme is network_prov_scheme_ble
         */
        const char *service_key = NULL;

        /* An optional endpoint that applications can create if they expect to
         * get some additional custom data during provisioning workflow.
         * The endpoint name can be anything of your choice.
         * This call must be made before starting the provisioning.
         */
        network_prov_mgr_endpoint_create("custom-data");

        /* Start provisioning service */
        ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(security, (const void *) NULL, service_name, service_key));

        /* The handler for the optional endpoint created above.
         * This call must be made after starting the provisioning, and only if the endpoint
         * has already been created above.
         */
        network_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL);
        // TODO! - you letf here, check example implementation

        /* Uncomment the following to wait for the provisioning to finish and then release
         * the resources of the manager. Since in this case de-initialization is triggered
         * by the default event loop handler, we don't need to call the following */
        // network_prov_mgr_wait();
        // network_prov_mgr_deinit();
        /* Print QR code for provisioning */
        //wifi_prov_print_qr(service_name, username, pop, PROV_TRANSPORT_SOFTAP);

    } 
    else {
        ESP_LOGI(TAG, "Already provisioned, starting Wi-Fi STA");

        /* We don't need the manager as device is already provisioned,
         * so let's release it's resources */
        ESP_ERROR_CHECK(network_prov_mgr_deinit());

        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandlerEntry, NULL));
        /* Start Wi-Fi station */
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    esp_netif_get_ip_info(
    esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),&ip);

    
    ESP_LOGI("NET", "IP: " IPSTR, IP2STR(&ip.ip));

    
    
    //Do initial wifi networks scan
    changeStatus(WifiManagerStatus::SCANNING);
    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL,true));

    //Initialize cyclic wifi task
    xTaskCreate(task_entry,"Wifi Manager",4096,this,2,NULL);
    
}

void WifiManager::storeAPPoints()
{
    uint16_t ap_count = 0;
    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    number = ap_count;
    std::vector<wifi_ap_record_t> ap_info(ap_count);
    
    if(ap_count <= 0) //Do not try to get the wifi if theres no APs
    {
        ap_info.resize(2);
    }

        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info.data()));
    
        std::lock_guard<std::mutex> lock(networkListMutex_);
        availableNetworks_.clear();
    
        // NO LOGGING IN EVENT HANDLER CONTEXT - causes stack overflow in sys_evt task
        // Log from main WiFi task instead
    
        for (auto it = ap_info.begin();it!=ap_info.end();++it) {
            const char* CurrSSID = reinterpret_cast<const char*>(it->ssid);
            availableNetworks_.emplace_back(CurrSSID);
        }

    
    
    changeStatus(WifiManagerStatus::SCANNING_FINISHED);
}

void WifiManager::WifiConnectRequest(std::string ssid, std::string passwrd)
{
    ESP_LOGI(TAG, "SSID \t\t%s", ssid.c_str());
    ESP_LOGI(TAG, "Password \t\t%s", passwrd.c_str());

    if(ssid.empty() || passwrd.empty())
    {
        if(m_WifiConnectionCallback)m_WifiConnectionCallback("Empty Field");
        return;
    }
    
    if(connectionStatus_ != WifiManagerStatus::CONNECTED) { WifiConnect(ssid,passwrd); }
    
    else
    {
        if(m_WifiConnectionCallback)m_WifiConnectionCallback("Connected");
    }
}

void WifiManager::WifiConnect(const std::string ssid,const std::string pwd)
{
    changeStatus(WifiManagerStatus::CONNECTING);
    memset(&wifi_config,0,sizeof(wifi_config));

    size_t ssid_len = std::min(ssid.size(), sizeof(wifi_config.sta.ssid) - 1);
    size_t pwd_len = std::min(pwd.size(), sizeof(wifi_config.sta.password) - 1);
    
    std::memcpy(wifi_config.sta.ssid,ssid.c_str(),ssid_len);
    std::memcpy(wifi_config.sta.password,pwd.c_str(),pwd_len);
    
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

void WifiManager::setWifiConnectionFeedback(std::function<void(const std::string&)> callback)
{
    m_WifiConnectionCallback = std::move(callback);
}

void WifiManager::setConnectionStateHandler(std::function<void(bool)> callback)
{
    m_ConnectionStateCallback = std::move(callback);
}

void WifiManager::setPendingConnectionState(bool connected)
{
    pendingConnectionState_.store(connected ? 1 : 0);
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
            //m_WifiConnectionCallback("Ready");
            //esp_wifi_connect();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            // Use lighter logging - avoid format strings in event context
            ESP_LOGW(TAG, "WiFi disconnected");
            changeStatus(WifiManagerStatus::DISCONNECTED);
            if(m_WifiConnectionCallback)m_WifiConnectionCallback("Disconnected");
            setPendingConnectionState(false);
            break;

        case WIFI_EVENT_SCAN_DONE:
            // Minimal logging before potentially heavy operation
            if(m_WifiConnectionCallback)m_WifiConnectionCallback("Scanning");
            if(connectionStatus_==WifiManagerStatus::SCANNING){storeAPPoints();};
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

        changeStatus(WifiManagerStatus::CONNECTED);
        setPendingConnectionState(true);
    }
}