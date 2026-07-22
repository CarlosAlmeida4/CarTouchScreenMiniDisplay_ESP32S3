#ifndef SYSTEM_HPP
#define SYSTEM_HPP


#include <stdio.h>
#include <cstring>

#include "os_primitives.h"
#include "system_os.h"
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
    /// @brief Release version tag for the firmware
    constexpr static std::string ReleaseTAG = "CarTSmD-v1.2.0";
    
    /// @brief Constructor - Initializes all subsystem queues and component instances
    /// @param none
    /// @return none (creates 2 queues for inter-component communication)
    System() {
        // Initialize OS abstraction layer
        os::init();
        
        // Create inter-component communication queues using SystemOS factory
        auto result = SystemOS::createQueues(RollPitchQueue, WifiMgrQueue);
        if (result != os::Result::OK) {
            // Queue creation failed - this is fatal
            printf("ERROR: Failed to create system queues\n");
        }
    }

    System(const System&) = delete;
    System& operator=(const System&) = delete;
    ~System() = default;
    
    /// @brief Initialize all subsystems and connect callbacks (main entry point after boot)
    /// @param none
    /// @return none
    /// @details Performs the following initialization sequence:
    ///   1. NVS flash initialization (persistent storage for WiFi credentials, settings)
    ///   2. Component initialization (display hardware, WiFi, IMU sensor, OTA, diagnostics)
    ///   3. Cross-component callback wiring (connects all event handlers for propagation)
    ///   4. Starts WiFi provisioning or auto-connection based on stored credentials
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

    // ========== PRIVATE MEMBERS (Component instances) ==========
    
    QueueHandle_t RollPitchQueue,WifiMgrQueue;  ///< Inter-task communication queues
    Display display;                            ///< Display/LVGL/touch driver (reads queues, renders UI)
    qmi8658cInterface qmiItf;                   ///< IMU sensor interface (writes RollPitchQueue)
    OTAUpdater OTAUpd;                          ///< Over-the-air firmware update manager
    WifiManager WifiMgr;                        ///< WiFi provisioning and connection manager
    DiagnosticsService diagnostics;             ///< HTTP diagnostics server (logs + status)

};

#endif // SYSTEM_HPP