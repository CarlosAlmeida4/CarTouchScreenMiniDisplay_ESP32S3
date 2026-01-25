#ifndef DISPLAY_HPP
#define DISPLAY_HPP

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
#include "rollsandpitch.hpp"
#include <bits/stdc++.h>

struct DisplayPins {
    gpio_num_t pinNumLcdCs;
    gpio_num_t pinNumLcdPclk;
    gpio_num_t pinNumLcdData0;
    gpio_num_t pinNumLcdData1;
    gpio_num_t pinNumLcdData2;
    gpio_num_t pinNumLcdData3;
    gpio_num_t pinNumLcdRst;
    gpio_num_t pinNumBkLight;
    gpio_num_t pinMasterSDA;
    gpio_num_t pinMasterSCL;
    gpio_num_t TouchINT;
    gpio_num_t TouchRST;
};

class Display {
public:
    explicit Display(QueueHandle_t q): Queue_(q) {}

    Display(const Display&) = delete;
    Display& operator=(const Display&) = delete;
    ~Display();
    
    void init();
    
    static bool notifyLvglFlushReady(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
    
private:
    
    QueueHandle_t Queue_;

    static TouchDrvCST92xx touch;

    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;
    static SemaphoreHandle_t lvgl_mux;
    
    static constexpr auto *DISPLAY_TAG = "Display";
    static constexpr auto BACKLIGHTPIN = -1;
    static constexpr uint16_t LCD_H_RES = 466;
    static constexpr uint16_t LCD_V_RES = 466;
    static constexpr spi_host_device_t LCD_HOST = SPI2_HOST;
    static constexpr int I2C_MASTER_FREQ_HZ = 100000;
    static constexpr i2c_port_t I2C_MASTER_NUM = I2C_NUM_1;
    static constexpr uint8_t I2C_MASTER_TX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
    static constexpr uint8_t I2C_MASTER_RX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
    static constexpr unsigned int I2C_MASTER_TIMEOUT_MS = 1000;
    static constexpr int LVGL_BUF_HEIGHT = LCD_V_RES/4; // in pixels
    static constexpr uint8_t LVGL_TICK_PERIOD_MS = 10; // in milliseconds
    static constexpr uint32_t LVGL_TASK_STACK_SIZE = 1024 * 4; // in bytes
    static constexpr uint8_t LVGL_TASK_PRIORITY = 2;

    static constexpr uint32_t LVGL_TASK_MAX_DELAY_MS = 500;
    static constexpr uint32_t LVGL_TASK_MIN_DELAY_MS = 1;

    static constexpr DisplayPins Pins = {
        .pinNumLcdCs = GPIO_NUM_12,
        .pinNumLcdPclk = GPIO_NUM_38,
        .pinNumLcdData0 = GPIO_NUM_4,
        .pinNumLcdData1 = GPIO_NUM_5,
        .pinNumLcdData2 = GPIO_NUM_6,
        .pinNumLcdData3 = GPIO_NUM_7,
        .pinNumLcdRst = GPIO_NUM_39,
        .pinNumBkLight = (gpio_num_t)BACKLIGHTPIN,
        .pinMasterSDA = GPIO_NUM_15,
        .pinMasterSCL = GPIO_NUM_14,
        .TouchINT = GPIO_NUM_11,
        .TouchRST = GPIO_NUM_40
    }; 

    void setup_sensor();
    static void lvglTouchCallBack(lv_indev_drv_t *drv, lv_indev_data_t *data);
    static void lvglFlushCallback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
    static void lvglUpdateCallback(lv_disp_drv_t *drv);
    static void lvglIncreaseTick(void *arg);
    static void lvgl_unlock(void);
    static bool lvgl_lock(int timeout_ms);
    static void task_entry(void *arg);

    void updateUI();
    void displayTask();
};

#endif // DISPLAY_HPP