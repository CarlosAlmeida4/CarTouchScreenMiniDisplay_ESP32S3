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
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_lcd_panel_commands.h"

#include "sdkconfig.h"
#include "freertos/queue.h"
#include "SensorLib.h"
#include "TouchDrvCST92xx.h"
#include "esp_lcd_sh8601.h"
#include "PipelineTypes.hpp"
#include <bits/stdc++.h>
#include <optional>

#include "network_provisioning/scheme_softap.h"
#include <network_provisioning/manager.h>

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
    /// @brief Constructor - Initializes Display with queue handles for sensor and WiFi data
    /// @param RPQueue FreeRTOS queue for roll/pitch/temperature sensor data
    /// @param WifiQueue FreeRTOS queue for WiFi status updates
    /// @return none
    explicit Display(QueueHandle_t  RPQueue,QueueHandle_t  WifiQueue):  RollPitchQueue_(RPQueue),
    WifiQueue_(WifiQueue) {m_activeInstance = this;}
    
    Display(const Display&) = delete;
    Display& operator=(const Display&) = delete;
    ~Display();
    
    /// @brief Initialize display hardware - SPI, LCD, touch, LVGL, and start rendering task
    /// @param none
    /// @return none
    void init();
    
    /// @brief Register callback handler for software update button events
    /// @param callback LVGL event handler function (receives lv_event_t* for update triggers)
    /// @return none
    void setSoftwareUpdateHandler(std::function<void(lv_event_t* )> callback);
    
    /// @brief Register callback for WiFi connection requests from UI
    /// @param callback Function receiving SSID and password strings entered by user
    /// @return none
    void setWifiConnectionHandler(std::function<void(const std::string&,const std::string&)>callback);
    
    /// @brief Register callback for inclinometer calibration/reset requests
    /// @param callback Function called when user requests zero-point reset
    /// @return none
    void setInclinometerResetHandler(std::function<void(void)>);
    
    /// @brief Handle brightness slider change events - adjusts LCD backlight (public for C callback)
    /// @param e LVGL event data containing brightness value
    /// @return none
    void setBrightnessHandler(lv_event_t* e); //Needs to be public because its called from a external C function
    
    void invokeSWUpdate(lv_event_t* e); 
    /// @brief Request WiFi connection to specified network (dispatches to WifiManager)
    /// @param ssid Network SSID
    /// @param passwrd Network password
    /// @return none
    void invokeWifiConnection(const std::string& ssid,const std::string& passwrd);
    
    /// @brief Update OTA update feedback text in UI (called from OTAUpdater)
    /// @param Feedback Status message string (e.g., "Downloading", "Installing", "Success")
    /// @return none
    void SWUpdateFeedback(const std::string& Feedback);
    
    /// @brief Update WiFi connection feedback text in UI (called from WifiManager)
    /// @param Feedback Status message string (e.g., "Connected", "Disconnected", "Scanning")
    /// @return none
    void WifiConnectionFeedback(const std::string& Feedback);

    esp_err_t setStoredBright() const;
    
    static Display* m_activeInstance;
    
private:
        
    QueueHandle_t RollPitchQueue_;
    QueueHandle_t WifiQueue_;

    esp_lcd_panel_io_handle_t m_IOhandle = NULL;
    int32_t brightSlideVal;

    static TouchDrvCST92xx touch;
    
    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;
    static SemaphoreHandle_t lvgl_mux;

    
    //----------Constants-------------//
    
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
    static constexpr uint8_t LVGL_TICK_PERIOD_MS = 5; // in milliseconds
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
    
    //========== PRIVATE METHODS ==========
    
    /// @brief Initialize touch sensor (I2C CST92xx capacitive touch controller)\n    /// @param none\n    /// @return none\n    void setup_sensor();\n    \n    // ========== LVGL CALLBACKS (Static - for LVGL C API compatibility) ==========\n    \n    /// @brief LVGL touch input callback - reads capacitive touch data from CST92xx\n    /// @param drv LVGL input device driver\n    /// @param data Output touch data (x, y, state)\n    /// @return none (populates lv_indev_data_t)\n    static void lvglTouchCallBack(lv_indev_drv_t *drv, lv_indev_data_t *data);\n    \n    /// @brief LVGL framebuffer flush callback - transfers pixels to display via SPI DMA\n    /// @param drv LVGL display driver\n    /// @param area Dirty region to update (x1, y1, x2, y2)\n    /// @param color_map Pixel buffer (may need color space conversion)\n    /// @return none (calls esp_lcd_panel_draw_bitmap + lv_disp_flush_ready)\n    static void lvglFlushCallback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);\n    \n    /// @brief LVGL display update callback - applies rotation/mirroring settings\n    /// @param drv LVGL display driver\n    /// @return none (calls esp_lcd_panel_swap_xy + esp_lcd_panel_mirror)\n    static void lvglUpdateCallback(lv_disp_drv_t *drv);\n    \n    /// @brief LVGL tick update callback (FreeRTOS timer interrupt context)\n    /// @param arg Unused (timer context)\n    /// @return none (calls lv_tick_inc)\n    static void lvglIncreaseTick(void *arg);\n    \n    /// @brief Release LVGL thread-safety semaphore\n    /// @param none\n    /// @return none (xSemaphoreGive)\n    static void lvgl_unlock(void);\n    \n    /// @brief Acquire LVGL thread-safety semaphore with timeout\n    /// @param timeout_ms Timeout milliseconds (-1 = wait forever, 0 = non-blocking)\n    /// @return true if lock acquired, false if timeout\n    static bool lvgl_lock(int timeout_ms);\n    \n    /// @brief FreeRTOS task entry point for display/LVGL task\n    /// @param arg Display* this pointer cast to void*\n    /// @return none (runs forever)\n    static void task_entry(void *arg);\n    \n    /// @brief Callback from SPI DMA completion - signals LVGL flush is complete\n    /// @param panel_io LCD panel IO handle\n    /// @param edata Event data (unused)\n    /// @param user_ctx Display* this pointer cast to void*\n    /// @return true to stop event propagation, false to continue\n    static bool notifyLvglFlushReady(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);\n\n    /// @brief Send brightness value to LCD backlight via PWM/GPIO\n    /// @param brightness_percent Brightness 0-100%\n    /// @return ESP_OK on success, ESP_ERR_* on failure\n    esp_err_t sendBrightnesstoScreen(const int32_t &brightness_percent) const;\n    \n    /// @brief Update UI state machine - dispatches to active screen render function\n    /// @param none\n    /// @return none (reads sensor/WiFi queues, updates LVGL widgets)\n    void updateUI();\n    \n    /// @brief Main LVGL task loop - calls lv_timer_handler and updateUI\n    /// @param none\n    /// @return none (runs forever, 5ms tick)\n    void displayTask();\n    \n    /// @brief Render inclinometer screen (roll/pitch/temperature sliders)\n    /// @param none\n    /// @return none (reads RollPitchQueue_, updates UI widgets)\n    void InclinometerUI();\n    \n    /// @brief Render alternative inclinometer screen design\n    /// @param none\n    /// @return none\n    void InclinometerNewUI();\n    \n    /// @brief Render WiFi provisioning/connection screen\n    /// @param none\n    /// @return none (reads WiFi status from queue)\n    void WifiUI();\n    \n    /// @brief Render manual WiFi SSID/password entry screen\n    /// @param none\n    /// @return none\n    void ManualWifiUI();\n\n    /// @brief Retrieve stored display brightness from NVS flash\n    /// @param none\n    /// @return Optional brightness percentage (0-100), nullopt if not stored\n    std::optional<int32_t> getStoredBright() const;

    /// @brief Registered callback for OTA software update trigger
    std::function<void(lv_event_t* )> m_SoftwareUpdateHandler;
    
    /// @brief Registered callback for WiFi connection requests (SSID, password)
    std::function<void(const std::string& ,const std::string& )> m_WifiConnectionHandler;
    
    /// @brief Registered callback for inclinometer zero-point calibration
    std::function<void(void)> m_ResetInclinometerHandler;

    // Pending feedback from non-LVGL tasks — written by any task, consumed by updateUI() under lvgl_mux
    std::mutex m_wifiFeedbackMutex;           ///< Protects WiFi feedback buffer from concurrent writes
    char       m_wifiFeedbackBuf[64]{};       ///< WiFi status message (updated by WifiManager)
    bool       m_wifiFeedbackPending{false};  ///< Flag indicating new WiFi message to display

    std::mutex m_swFeedbackMutex;             ///< Protects OTA feedback buffer from concurrent writes
    char       m_swFeedbackBuf[64]{};         ///< OTA update status message (updated by OTAUpdater)
    bool       m_swFeedbackPending{false};    ///< Flag indicating new OTA message to display
};

#endif // DISPLAY_HPP