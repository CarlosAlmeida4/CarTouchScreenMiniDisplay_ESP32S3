idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES
                    display SensorLib qmi8658cInterface System 
                    esp_http_client app_update esp_https_ota   
                    esp_netif mbedtls nvs_flash esp_wifi esp_psram               
)

set_source_files_properties(
    ${LV_DEMOS_SOURCES}
    PROPERTIES COMPILE_OPTIONS
    -DLV_LVGL_H_INCLUDE_SIMPLE)