#include "i2cHandler.hpp"


esp_err_t i2cHandler::masterInit(void)
{
    if(isi2cInitialized) 
    {
        return ESP_OK;
    }
    else
    {
        isi2cInitialized = true;
        i2c_config_t i2c_conf;
        memset(&i2c_conf, 0, sizeof(i2c_conf));
        i2c_conf.mode = I2C_MODE_MASTER;
        i2c_conf.sda_io_num = Pins.pinMasterSDA;
        i2c_conf.scl_io_num = Pins.pinMasterSCL;
        i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        i2c_conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
        i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
        return i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    }
}

bool i2cHandler::i2c_lock(int timeout_ms)
{
    assert(i2c_mux && "i2c_init must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(i2c_mux, timeout_ticks) == pdTRUE;
}

void i2cHandler::i2c_unlock(void)
{
    assert(i2c_mux && "i2c_init must be called first");
    xSemaphoreGive(i2c_mux);
}

