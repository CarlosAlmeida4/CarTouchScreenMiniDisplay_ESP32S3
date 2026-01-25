#ifndef SYSTEM_HPP
#define SYSTEM_HPP


#include <stdio.h>
#include <cstring>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "SensorLib.h"
#include "SensorQMI8658.hpp"
#include "display.hpp"
#include "qmi8658cInterface.hpp"
#include "rollsandpitch.hpp"

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

private:
      
    QueueHandle_t RollPitchQueue;
    Display display;
    qmi8658cInterface qmiItf;

};

#endif // SYSTEM_HPP