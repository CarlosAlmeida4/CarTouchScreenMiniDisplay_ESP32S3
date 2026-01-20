#include "qmi8658cInterface.hpp"

#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL (16)
#endif

SensorQMI8658 qmi8658cInterface::qmi{};

IMUdata acc;
IMUdata gyr;


qmi8658cInterface::~qmi8658cInterface() {

}



void qmi8658cInterface::setup_sensor()
{
    // Initialize QMI8658 sensor with 4 parameters (port number, address, SDA, SCL)
    if (!qmi.begin(I2C_MASTER_NUM, QMI8658_ADDRESS, Pins.pinMasterSDA, Pins.pinMasterSCL)) {
        ESP_LOGE(QMI8658C_TAG, "Failed to find QMI8658 - check your wiring!");
        vTaskDelete(NULL); // Handle error gracefully
    }

    // Get chip ID
    ESP_LOGI(QMI8658C_TAG, "Device ID: %x", qmi.getChipID());

    // Configure accelerometer
    qmi.configAccelerometer(
        SensorQMI8658::ACC_RANGE_4G,
        SensorQMI8658::ACC_ODR_1000Hz,
        SensorQMI8658::LPF_MODE_0,
        true
    );

    // Configure gyroscope
    qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_64DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_3,
        true
    );

    // Enable gyroscope and accelerometer
    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    ESP_LOGI(QMI8658C_TAG, "Ready to read data...");
}

void qmi8658cInterface::read_sensor_data(void* arg) {
    while (1) {
        if (qmi.getDataReady()) {
            if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                ESP_LOGI(QMI8658C_TAG, "ACCEL: %f, %f, %f", acc.x, acc.y, acc.z);
            } else {
                ESP_LOGE(QMI8658C_TAG, "Failed to read accelerometer data");
            }

            if (qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
                ESP_LOGI(QMI8658C_TAG, "GYRO: %f, %f, %f", gyr.x, gyr.y, gyr.z);
            } else {
                ESP_LOGE(QMI8658C_TAG, "Failed to read gyroscope data");
            }

            ESP_LOGI(QMI8658C_TAG, "Timestamp: %u, Temperature: %.2f *C", (unsigned int)qmi.getTimestamp(), qmi.getTemperature_C()); // Casting to unsigned int
        } else {
            ESP_LOGW(QMI8658C_TAG, "Data not ready yet");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}



void qmi8658cInterface::init() {
    setup_sensor();
    xTaskCreate(read_sensor_data, "sensor_read_task", 4096, NULL, 10, NULL);
}