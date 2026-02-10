#include "display.hpp"

#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL (16)
#endif

#include <atomic>

static std::atomic<bool> ui_ready{false};

// ---- static member definitions ----
lv_disp_drv_t Display::disp_drv{};
lv_disp_draw_buf_t Display::disp_buf{};
SemaphoreHandle_t Display::lvgl_mux = nullptr;
TouchDrvCST92xx Display::touch{};
Display* Display::m_activeInstance = nullptr;

inline float normalize(float input)
{
    return ((10/9)*input + 50);
}

inline std::string turnFloat2Char(float input)
{
    return std::format("{:.1f}", input);
    
}

static const sh8601_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFE, (uint8_t[]){0x00}, 1, 0},
    {0xC4, (uint8_t[]){0x80}, 1, 0},
    {0x3A, (uint8_t[]){0x55}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x53, (uint8_t[]){0x20}, 1, 0},
    {0x51, (uint8_t[]){0xFF}, 1, 0},
    {0x63, (uint8_t[]){0xFF}, 1, 0},
    {0x2A, (uint8_t[]){0x00, 0x06, 0x01, 0xD7}, 4, 0},
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0xD1}, 4, 600},
    {0x11, NULL, 0, 600}, // 命令后延时 600ms
    {0x29, NULL, 0, 0},   // 无延时
};

void lvglRounderCallback(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;

    uint16_t y1 = area->y1;
    uint16_t y2 = area->y2;

    // round the start of coordinate down to the nearest 2M number
    area->x1 = (x1 >> 1) << 1;
    area->y1 = (y1 >> 1) << 1;
    // round the end of coordinate up to the nearest 2N+1 number
    area->x2 = ((x2 >> 1) << 1) + 1;
    area->y2 = ((y2 >> 1) << 1) + 1;
}

int16_t x[5], y[5];

Display::~Display() {
}

void Display::setup_sensor() {
    uint8_t touchAddress = 0x5A;
    touch.setPins(Pins.TouchRST, Pins.TouchINT);
    touch.begin(I2C_MASTER_NUM, touchAddress, Pins.pinMasterSDA, Pins.pinMasterSCL);
    touch.reset();
    touch.setMaxCoordinates(466, 466);
    touch.setMirrorXY(true, true);
}

void Display::init() {
    
    if constexpr (BACKLIGHTPIN >=0)
    {
        ESP_LOGI(DISPLAY_TAG, "Turn off LCD backlight");
        gpio_config_t bk_gpio_config = {
            .pin_bit_mask = 1ULL << BACKLIGHTPIN,
            .mode = GPIO_MODE_OUTPUT};
            ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    }   

    
    lvgl_mux = NULL;

    ESP_LOGI(DISPLAY_TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = GPIO_NUM_38;
    buscfg.data0_io_num = GPIO_NUM_4;
    buscfg.data1_io_num = GPIO_NUM_5;
    buscfg.data2_io_num = GPIO_NUM_6;
    buscfg.data3_io_num = GPIO_NUM_7;
    buscfg.max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t);
    buscfg.flags = SPICOMMON_BUSFLAG_QUAD;
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(DISPLAY_TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = SH8601_PANEL_IO_QSPI_CONFIG(Pins.pinNumLcdCs,
                                                                                notifyLvglFlushReady,
                                                                                &disp_drv);
    sh8601_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = Pins.pinNumLcdRst,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI(DISPLAY_TAG, "Install SH8601 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    setup_sensor();


//#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
//    ESP_LOGI(DISPLAY_TAG, "Turn on LCD backlight");
//    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
//#endif

    ESP_LOGI(DISPLAY_TAG, "Initialize LVGL library");
    lv_init();

    ESP_LOGI("HEAP", "Free DMA heap: %d",
    heap_caps_get_free_size(MALLOC_CAP_DMA));
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = static_cast<lv_color_t *>(heap_caps_malloc(LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA));
    assert(buf1);
    //lv_color_t *buf2 = static_cast<lv_color_t *>(heap_caps_malloc(LCD_H_RES * LVGL_BUF_HEIGHT * sizeof(lv_color_t), MALLOC_CAP_DMA));
    //assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, NULL, LCD_H_RES * LVGL_BUF_HEIGHT);

    ESP_LOGI(DISPLAY_TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvglFlushCallback;
    disp_drv.rounder_cb = lvglRounderCallback;
    disp_drv.drv_update_cb = lvglUpdateCallback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(DISPLAY_TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvglIncreaseTick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = lvglTouchCallBack;
    indev_drv.user_data = &touch;
    lv_indev_drv_register(&indev_drv);

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);

    ui_init();
    lv_scr_load(ui_Inclinometer);   // VERY IMPORTANT
    ui_ready.store(true, std::memory_order_release);
    
    xTaskCreate(task_entry, "LVGL", LVGL_TASK_STACK_SIZE, this, LVGL_TASK_PRIORITY, NULL);
}

void Display::task_entry(void *arg)
{
    auto* self = static_cast<Display*>(arg);
    self->displayTask();
}

static inline bool lv_obj_ready(lv_obj_t * obj)
{
    return obj &&
           lv_obj_is_valid(obj) &&
           lv_obj_get_disp(obj) != NULL;
}

bool checkInclinometerFieldsVdl()
{
    return lv_obj_ready(uic_RollText)  &&
           lv_obj_ready(uic_PitchText) &&
           lv_obj_ready(uic_RollA)     &&
           lv_obj_ready(uic_RollB)     &&
           lv_obj_ready(uic_Pitch);
}

void Display::updateUI()
{
    RollPitch RP{0,0};

    if (!ui_ready.load(std::memory_order_acquire)) {
        return;
    }

    // Receive data WITHOUT LVGL lock

    if (!xQueueReceive(Queue_, &RP, 0)) {
        return;
    }

    // Now touch LVGL
    if (!checkInclinometerFieldsVdl()) {
        return;
    }

    char ip_str[16];

    esp_netif_ip_info_t ip_info;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    esp_netif_get_ip_info(netif, &ip_info);

    sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));


    std::string rollStr  = turnFloat2Char(RP.roll);
    std::string pitchStr = turnFloat2Char(RP.pitch);
    std::string TemperatureString = turnFloat2Char(RP.temperature);

    _ui_label_set_property(uic_IPString,_UI_LABEL_PROPERTY_TEXT,ip_str);
    _ui_label_set_property(uic_RollText,_UI_LABEL_PROPERTY_TEXT,rollStr.c_str());
    _ui_label_set_property(uic_PitchText,_UI_LABEL_PROPERTY_TEXT,pitchStr.c_str());
    _ui_label_set_property(uic_TemperatureReading,_UI_LABEL_PROPERTY_TEXT,TemperatureString.c_str());

    lv_slider_set_value(uic_RollA,(int32_t)(100-normalize(-RP.roll)), LV_ANIM_ON);
    lv_slider_set_value(uic_RollB,(int32_t)(100-normalize(RP.roll)), LV_ANIM_ON);
    lv_slider_set_value(uic_Pitch,(int32_t)normalize(RP.pitch), LV_ANIM_ON);

}

void Display::displayTask() 
{
    ESP_LOGI(DISPLAY_TAG, "Starting LVGL task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (lvgl_lock(-1))
        {   
            task_delay_ms = lv_timer_handler();
            updateUI();
            // Release the mutex
            lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

bool Display::notifyLvglFlushReady(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return true;
}

void Display::lvglTouchCallBack(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint8_t touched = touch.getPoint(x, y, 2);
    if (touched)
    {

        for (int i = 0; i < 1; ++i)
        {
            data->point.x = x[0];
            data->point.y = y[0];
            data->state = LV_INDEV_STATE_PRESSED;
            //ESP_LOGI(DISPLAY_TAG, "Touch[%d]: X=%d Y=%d", i, x[i], y[i]);
        }
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

void Display::lvglFlushCallback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    const int offsetx1 = area->x1; //+ 0x16;
    const int offsetx2 = area->x2; //+ 0x16;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

#if LCD_BIT_PER_PIXEL == 24
    uint8_t *to = (uint8_t *)color_map;
    uint8_t temp = 0;
    uint16_t pixel_num = (offsetx2 - offsetx1 + 1) * (offsety2 - offsety1 + 1);

    // Special dealing for first pixel
    temp = color_map[0].ch.blue;
    *to++ = color_map[0].ch.red;
    *to++ = color_map[0].ch.green;
    *to++ = temp;
    // Normal dealing for other pixels
    for (int i = 1; i < pixel_num; i++)
    {
        *to++ = color_map[i].ch.red;
        *to++ = color_map[i].ch.green;
        *to++ = color_map[i].ch.blue;
    }
#endif

    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

void Display::lvglUpdateCallback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;

    switch (drv->rotated)
    {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

void Display::lvglIncreaseTick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

bool Display::lvgl_lock(int timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

void Display::lvgl_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}

void Display::setSoftwareUpdateHandler(std::function<void(lv_event_t* )> callback)
{
    m_SoftwareUpdateHandler = std::move(callback);
}

void Display::invokeSWUpdate(lv_event_t* e)
{
    if(m_SoftwareUpdateHandler)
    {
        _ui_label_set_property(uic_SoftwareUpdateFeedback,_UI_LABEL_PROPERTY_TEXT,"In Progress");
        m_SoftwareUpdateHandler(e);
    }
    else
    {
        _ui_label_set_property(uic_SoftwareUpdateFeedback,_UI_LABEL_PROPERTY_TEXT,"Error in Update");
    }
}

void Display::SWUpdateFeedback(const std::string& Feedback)
{
    if(lvgl_lock(100))
    {
         if (lv_obj_ready(uic_SoftwareUpdateFeedback)) {
                _ui_label_set_property(uic_SoftwareUpdateFeedback, 
                    _UI_LABEL_PROPERTY_TEXT, Feedback.c_str());
            }
    }
    lvgl_unlock();  
}

extern "C" void UI_RequestSWUpdate(lv_event_t * e)
{
    if(Display::m_activeInstance)
    {
        Display::m_activeInstance->invokeSWUpdate(e);
    }
}