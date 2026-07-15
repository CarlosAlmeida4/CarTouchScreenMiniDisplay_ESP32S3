#ifndef SYSTEM_HPP
#define SYSTEM_HPP


#include <stdio.h>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "SensorLib.h"
#include "display.hpp"
#include "qmi8658cInterface.hpp"
#include "OTAUpdater.hpp"
#include "WifiManager.hpp"
#include "Diagnostics.hpp"

#include <string.h>


class System {
public:


    constexpr static std::string ReleaseTAG = "CarTSmD-v1.2.0";
    

    System(): RollPitchQueue(xQueueCreate(1,sizeof(RollPitch))),
    WifiMgrQueue(xQueueCreate(1,sizeof(WifiManagerPipeline))),
    display(RollPitchQueue,WifiMgrQueue),
    qmiItf(RollPitchQueue),
    WifiMgr(WifiMgrQueue) {}

    System(const System&) = delete;
    System& operator=(const System&) = delete;
    ~System() = default;
    
    void start()
    {
        //General inits needed by multiple components
        
        // Initialize NVS
        esp_err_t nvs_err = nvs_flash_init();
        if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) 
        {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            ESP_ERROR_CHECK(nvs_flash_erase());
            nvs_err = nvs_flash_init();
        }
        
        //Compoents Init
        diagnostics.init(500, 220);
        diagnostics.installLogSink();

        display.init();
        qmiItf.init();
        
        /**
         * Callback setting
         */
        display.setSoftwareUpdateHandler(
            [this](lv_event_t* e)
            {
                OTAUpd.triggerUpdate();
            });

        OTAUpd.setSWUpdateFeedback(
            [this](const std::string& msg)
            {
                display.SWUpdateFeedback(msg);
            }
        );

        display.setWifiConnectionHandler(
            [this](const std::string& ssid, const std::string& passwrd)
            {
                WifiMgr.WifiConnectRequest(ssid,passwrd);
            }
        );

        WifiMgr.setWifiConnectionFeedback(
            [this](const std::string& msg)
            {
                display.WifiConnectionFeedback(msg);
            }
        );

        WifiMgr.setConnectionStateHandler(
            [this](bool connected)
            {
                if (connected) {
                    diagnostics.onWifiConnected();
                } else {
                    diagnostics.onWifiDisconnected();
                }
            }
        );

        display.setInclinometerResetHandler(
            [this]()
            {
                qmiItf.setInclinometerOffset();
            }
        );

        WifiMgr.initWifi();


    } 

    

private:
      
    QueueHandle_t RollPitchQueue,WifiMgrQueue;
    Display display;
    qmi8658cInterface qmiItf;
    OTAUpdater OTAUpd;
    WifiManager WifiMgr;
    DiagnosticsService diagnostics;

};

#endif // SYSTEM_HPP