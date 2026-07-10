#include "WifiManager.hpp"

/* This salt,verifier has been generated for username = "wifiprov" and password = "abcd1234"
 * IMPORTANT NOTE: For production cases, this must be unique to every device
 * and should come from device manufacturing partition.*/
static const char sec2_salt[] = {
    0x03, 0x6e, 0xe0, 0xc7, 0xbc, 0xb9, 0xed, 0xa8, 0x4c, 0x9e, 0xac, 0x97, 0xd9, 0x3d, 0xec, 0xf4
};

static const char sec2_verifier[] = {
    0x7c, 0x7c, 0x85, 0x47, 0x65, 0x08, 0x94, 0x6d, 0xd6, 0x36, 0xaf, 0x37, 0xd7, 0xe8, 0x91, 0x43,
    0x78, 0xcf, 0xfd, 0x61, 0x6c, 0x59, 0xd2, 0xf8, 0x39, 0x08, 0x12, 0x72, 0x38, 0xde, 0x9e, 0x24,
    0xa4, 0x70, 0x26, 0x1c, 0xdf, 0xa9, 0x03, 0xc2, 0xb2, 0x70, 0xe7, 0xb1, 0x32, 0x24, 0xda, 0x11,
    0x1d, 0x97, 0x18, 0xdc, 0x60, 0x72, 0x08, 0xcc, 0x9a, 0xc9, 0x0c, 0x48, 0x27, 0xe2, 0xae, 0x89,
    0xaa, 0x16, 0x25, 0xb8, 0x04, 0xd2, 0x1a, 0x9b, 0x3a, 0x8f, 0x37, 0xf6, 0xe4, 0x3a, 0x71, 0x2e,
    0xe1, 0x27, 0x86, 0x6e, 0xad, 0xce, 0x28, 0xff, 0x54, 0x46, 0x60, 0x1f, 0xb9, 0x96, 0x87, 0xdc,
    0x57, 0x40, 0xa7, 0xd4, 0x6c, 0xc9, 0x77, 0x54, 0xdc, 0x16, 0x82, 0xf0, 0xed, 0x35, 0x6a, 0xc4,
    0x70, 0xad, 0x3d, 0x90, 0xb5, 0x81, 0x94, 0x70, 0xd7, 0xbc, 0x65, 0xb2, 0xd5, 0x18, 0xe0, 0x2e,
    0xc3, 0xa5, 0xf9, 0x68, 0xdd, 0x64, 0x7b, 0xb8, 0xb7, 0x3c, 0x9c, 0xfc, 0x00, 0xd8, 0x71, 0x7e,
    0xb7, 0x9a, 0x7c, 0xb1, 0xb7, 0xc2, 0xc3, 0x18, 0x34, 0x29, 0x32, 0x43, 0x3e, 0x00, 0x99, 0xe9,
    0x82, 0x94, 0xe3, 0xd8, 0x2a, 0xb0, 0x96, 0x29, 0xb7, 0xdf, 0x0e, 0x5f, 0x08, 0x33, 0x40, 0x76,
    0x52, 0x91, 0x32, 0x00, 0x9f, 0x97, 0x2c, 0x89, 0x6c, 0x39, 0x1e, 0xc8, 0x28, 0x05, 0x44, 0x17,
    0x3f, 0x68, 0x02, 0x8a, 0x9f, 0x44, 0x61, 0xd1, 0xf5, 0xa1, 0x7e, 0x5a, 0x70, 0xd2, 0xc7, 0x23,
    0x81, 0xcb, 0x38, 0x68, 0xe4, 0x2c, 0x20, 0xbc, 0x40, 0x57, 0x76, 0x17, 0xbd, 0x08, 0xb8, 0x96,
    0xbc, 0x26, 0xeb, 0x32, 0x46, 0x69, 0x35, 0x05, 0x8c, 0x15, 0x70, 0xd9, 0x1b, 0xe9, 0xbe, 0xcc,
    0xa9, 0x38, 0xa6, 0x67, 0xf0, 0xad, 0x50, 0x13, 0x19, 0x72, 0x64, 0xbf, 0x52, 0xc2, 0x34, 0xe2,
    0x1b, 0x11, 0x79, 0x74, 0x72, 0xbd, 0x34, 0x5b, 0xb1, 0xe2, 0xfd, 0x66, 0x73, 0xfe, 0x71, 0x64,
    0x74, 0xd0, 0x4e, 0xbc, 0x51, 0x24, 0x19, 0x40, 0x87, 0x0e, 0x92, 0x40, 0xe6, 0x21, 0xe7, 0x2d,
    0x4e, 0x37, 0x76, 0x2f, 0x2e, 0xe2, 0x68, 0xc7, 0x89, 0xe8, 0x32, 0x13, 0x42, 0x06, 0x84, 0x84,
    0x53, 0x4a, 0xb3, 0x0c, 0x1b, 0x4c, 0x8d, 0x1c, 0x51, 0x97, 0x19, 0xab, 0xae, 0x77, 0xff, 0xdb,
    0xec, 0xf0, 0x10, 0x95, 0x34, 0x33, 0x6b, 0xcb, 0x3e, 0x84, 0x0f, 0xb9, 0xd8, 0x5f, 0xb8, 0xa0,
    0xb8, 0x55, 0x53, 0x3e, 0x70, 0xf7, 0x18, 0xf5, 0xce, 0x7b, 0x4e, 0xbf, 0x27, 0xce, 0xce, 0xa8,
    0xb3, 0xbe, 0x40, 0xc5, 0xc5, 0x32, 0x29, 0x3e, 0x71, 0x64, 0x9e, 0xde, 0x8c, 0xf6, 0x75, 0xa1,
    0xe6, 0xf6, 0x53, 0xc8, 0x31, 0xa8, 0x78, 0xde, 0x50, 0x40, 0xf7, 0x62, 0xde, 0x36, 0xb2, 0xba
};

static esp_err_t example_get_sec2_salt(const char **salt, uint16_t *salt_len)
{
#ifdef PROV_DEVMODE
    ESP_LOGI("WifiManagerInternal", "Development mode: using hard coded salt");
    *salt = sec2_salt;
    *salt_len = sizeof(sec2_salt);
    return ESP_OK;
#else
    ESP_LOGE(TAG, "Not implemented!");
    return ESP_FAIL;
#endif
}

static esp_err_t example_get_sec2_verifier(const char **verifier, uint16_t *verifier_len)
{
#ifdef PROV_DEVMODE
    ESP_LOGI("WifiManagerInternal", "Development mode: using hard coded verifier");
    *verifier = sec2_verifier;
    *verifier_len = sizeof(sec2_verifier);
    return ESP_OK;
#else
    /* This code needs to be updated with appropriate implementation to provide verifier */
    ESP_LOGE(TAG, "Not implemented!");
    return ESP_FAIL;
#endif
}

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
            case SCANNING:
                if(pendingScanResults_.load())
                {
                    storeAPPoints();
                    changeStatus(WifiManagerStatus::SCANNING_FINISHED);
                }
                break;
            case INIT:
            case READY_TO_CONNECT:
            case CONNECTING:
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

void WifiManager::WifiProvAppCallback( network_prov_cb_event_t event, void *event_data)
{
    /**
     * This is blocking callback, any configurations that needs to be set when a particular
     * provisioning event is triggered can be set here.
    */
    switch (event) {
    case NETWORK_PROV_SET_WIFI_STA_CONFIG: {
        /**
         * Wi-Fi configurations can be set here before the Wi-Fi is enabled in
         * STA mode.
        */
        wifi_config_t *wifi_config = (wifi_config_t *)event_data;
        (void) wifi_config;
        break;
    }
    default:
        break;
    }
}

/**
 * brief: register all events, to be called at initialization time
 * !Events shall be initialized before wifi
 */
void WifiManager::registerWifiEvents()
{
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

}

void WifiManager::initWifi()
{
    changeStatus(WifiManagerStatus::INIT);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    WifiEventGroup = xEventGroupCreate();

    registerWifiEvents();

    /* Initialize Wi-Fi including netif with default config */
    esp_netif_create_default_wifi_sta();

    //Initialize Wifi AP so that we can use SoftAP
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    

    network_prov_mgr_config_t NetProvMgr = 
    {
        
        .scheme = network_prov_scheme_softap,
        .scheme_event_handler = NETWORK_PROV_EVENT_HANDLER_NONE,
        .app_event_handler = WifiProvEventHandler
    };

    ESP_ERROR_CHECK(network_prov_mgr_init(NetProvMgr));

    bool provisioned = false;

    /* FIXME: Currently it doenst store the connection between connecitons, but if this is removed
    *   This will for some reason assume the device is already provisioned
    */ 
    //network_prov_mgr_reset_wifi_provisioning();

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
        network_prov_security_t security = NETWORK_PROV_SECURITY_2;

#ifdef PROV_DEVMODE
        /* This pop field represents the password that will be used to generate salt and verifier.
         * The field is present here in order to generate the QR code containing password.
         * In production this password field shall not be stored on the device */
        const char *username  = PROV_SEC2_USERNAME;
        const char *pop = PROV_SEC2_PWD;
#elif  
        /* The username and password shall not be embedded in the firmware,
         * they should be provided to the user by other means.
         * e.g. QR code sticker */
        const char *username  = NULL;
        const char *pop = NULL;
#endif        
        network_prov_security2_params_t sec2_params = {};

        ESP_ERROR_CHECK(example_get_sec2_salt(&sec2_params.salt, &sec2_params.salt_len));
        ESP_ERROR_CHECK(example_get_sec2_verifier(&sec2_params.verifier, &sec2_params.verifier_len));

        network_prov_security2_params_t *sec_params = &sec2_params;

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
        ESP_ERROR_CHECK(network_prov_mgr_start_provisioning(security, (const void *) sec_params, service_name, service_key));

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

    /* Wait for Wi-Fi connection */
    //xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_EVENT, true, true, portMAX_DELAY);


    //esp_netif_get_ip_info(
    //esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),&ip);

    
    //ESP_LOGI("NET", "IP: " IPSTR, IP2STR(&ip.ip));

    
    
    //Do initial wifi networks scan
    changeStatus(WifiManagerStatus::SCANNING);
    //ESP_ERROR_CHECK(esp_wifi_scan_start(NULL,true));

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
            if(connectionStatus_==WifiManagerStatus::SCANNING){pendingScanResults_.store(true);};
            break;
        case WIFI_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "SoftAP transport: Connected!");
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            ESP_LOGI(TAG, "SoftAP transport: Disconnected!");
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

    if(event_base == NETWORK_PROV_EVENT)
    {
        switch (event_id){

        case NETWORK_PROV_START:
            ESP_LOGI(TAG, "Provisioning started");
            break;
        case NETWORK_PROV_WIFI_CRED_RECV: {
            wifi_sta_config_t *wifi_sta_cfg = (wifi_sta_config_t *)event_data;
            ESP_LOGI(TAG, "Received Wi-Fi credentials"
                     "\n\tSSID     : %s\n\tPassword : %s",
                     (const char *) wifi_sta_cfg->ssid,
                     (const char *) wifi_sta_cfg->password);
            break;
        }
        case NETWORK_PROV_WIFI_CRED_FAIL: {
            network_prov_wifi_sta_fail_reason_t *reason = (network_prov_wifi_sta_fail_reason_t *)event_data;
            ESP_LOGE(TAG, "Provisioning failed!\n\tReason : %s"
                     "\n\tPlease reset to factory and retry provisioning",
                     (*reason == NETWORK_PROV_WIFI_STA_AUTH_ERROR) ?
                     "Wi-Fi station authentication failed" : "Wi-Fi access-point not found");
                        /* Reset the state machine on provisioning failure.
             * This is enabled by the CONFIG_EXAMPLE_RESET_PROV_MGR_ON_FAILURE configuration.
             * It allows the provisioning manager to retry the provisioning process
             * based on the number of attempts specified in wifi_conn_attempts. After attempting
             * the maximum number of retries, the provisioning manager will reset the state machine
             * and the provisioning process will be terminated.
             */
            network_prov_mgr_reset_wifi_sm_state_on_failure();
            break;
        }
        case NETWORK_PROV_WIFI_CRED_SUCCESS:
            ESP_LOGI(TAG, "Provisioning successful");
            break;
        case NETWORK_PROV_END:{
            /* De-initialize manager once provisioning is finished */
            esp_err_t err = network_prov_mgr_deinit();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to de-initialize provisioning manager: %s", esp_err_to_name(err));
            }
            break;
        }
        default:
            break;
        }
    }

    if (event_base == PROTOCOMM_SECURITY_SESSION_EVENT) {
        switch (event_id) {
        case PROTOCOMM_SECURITY_SESSION_SETUP_OK:
            ESP_LOGI(TAG, "Secured session established!");
            break;
        case PROTOCOMM_SECURITY_SESSION_INVALID_SECURITY_PARAMS:
            ESP_LOGE(TAG, "Received invalid security parameters for establishing secure session!");
            break;
        case PROTOCOMM_SECURITY_SESSION_CREDENTIALS_MISMATCH:
            ESP_LOGE(TAG, "Received incorrect username and/or PoP for establishing secure session!");
            break;
        default:
            break;
        }
    }
}