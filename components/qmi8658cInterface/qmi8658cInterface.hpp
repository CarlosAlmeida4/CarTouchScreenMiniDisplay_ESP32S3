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
#include "rollsandpitch.hpp"


struct qmi8652Pins 
{
    gpio_num_t pinMasterSDA;
    gpio_num_t pinMasterSCL;
};



class qmi8658cInterface {
public:

    explicit qmi8658cInterface(QueueHandle_t q): Queue_(q) {}

    qmi8658cInterface(const qmi8658cInterface&) = delete;
    qmi8658cInterface& operator=(const qmi8658cInterface&) = delete;
    ~qmi8658cInterface();
    
    void init();
    static bool getPitchAndRoll(RollPitch& out);


private:
    QueueHandle_t  Queue_;  
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
    static void task_entry(void* arg);
    void read_sensor_data();

};

#endif // DISPLAY_HPP