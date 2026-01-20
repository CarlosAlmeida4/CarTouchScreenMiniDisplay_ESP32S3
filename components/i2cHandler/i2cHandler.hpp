#ifndef I2C_HANDLER_HPP
#define I2C_HANDLER_HPP

#include "lvgl.h"
#include "ui.h"
#include <stdio.h>
#include <cstring>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/queue.h"
#include "SensorLib.h"
#include "TouchDrvCST92xx.h"
#include "esp_lcd_sh8601.h"

struct i2cPins {
    gpio_num_t pinMasterSDA;
    gpio_num_t pinMasterSCL;
};

class i2cHandler {
public:
    i2cHandler() = default;
    i2cHandler(const i2cHandler&) = delete;
    i2cHandler& operator=(const i2cHandler&) = delete;
    ~i2cHandler();
    
    esp_err_t masterInit(void);

private:

    static bool isi2cInitialized = false;

    static SemaphoreHandle_t i2c_mux;
    
    static constexpr auto *I2C_TAG = "i2c";
    static constexpr int I2C_MASTER_FREQ_HZ = 100000;
    static constexpr i2c_port_t I2C_MASTER_NUM = I2C_NUM_1;
    static constexpr uint8_t I2C_MASTER_TX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
    static constexpr uint8_t I2C_MASTER_RX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
    static constexpr unsigned int I2C_MASTER_TIMEOUT_MS = 1000;
    

    static constexpr i2cPins Pins = {
        .pinMasterSDA = GPIO_NUM_15,
        .pinMasterSCL = GPIO_NUM_14
    }; 

    static void i2c_unlock(void);
    static bool i2c_lock(int timeout_ms);
};

#endif // DISPLAY_HPP