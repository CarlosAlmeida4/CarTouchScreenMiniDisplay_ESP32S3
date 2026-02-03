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



class System {
public:

    System(): RollPitchQueue(xQueueCreate(1,sizeof(RollPitch))),
    display(RollPitchQueue),
    qmiItf(RollPitchQueue) {}

    System(const System&) = delete;
    System& operator=(const System&) = delete;
    ~System() = default;
    
    void start()
    {
        display.init();
        qmiItf.init();
    }

    void ota()
    {

    }
private:
      
    QueueHandle_t RollPitchQueue;
    Display display;
    qmi8658cInterface qmiItf;

};

#endif // SYSTEM_HPP