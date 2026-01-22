#ifndef QMI8652INTERFACE_HPP
#define QMI8652INTERFACE_HPP


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


struct qmi8652Pins 
{
    gpio_num_t pinMasterSDA;
    gpio_num_t pinMasterSCL;
};

struct RollPitch {
        float roll = 0.0f;
        float pitch = 0.0f;
};


class qmi8658cInterface {
public:

    qmi8658cInterface() = default;
    qmi8658cInterface(const qmi8658cInterface&) = delete;
    qmi8658cInterface& operator=(const qmi8658cInterface&) = delete;
    ~qmi8658cInterface();
    
    void init();
    static RollPitch getPitchAndRoll();


private:
      
    static constexpr auto *QMI8658C_TAG = "QMI8658C";
    static constexpr int QMI8658_ADDRESS = 0x6B;
    static constexpr int I2C_MASTER_FREQ_HZ = 100000;
    static constexpr i2c_port_t I2C_MASTER_NUM = I2C_NUM_1;
    static constexpr uint8_t I2C_MASTER_TX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
    static constexpr uint8_t I2C_MASTER_RX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
    static constexpr unsigned int I2C_MASTER_TIMEOUT_MS = 1000;

    static SensorQMI8658 qmi;
    static RollPitch RP;    


    static constexpr qmi8652Pins  Pins = {
        .pinMasterSDA = GPIO_NUM_15,
        .pinMasterSCL = GPIO_NUM_14
    };
    
    void setup_sensor();
    static void read_sensor_data(void* arg);

};

#endif // DISPLAY_HPP