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


class System {
public:

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