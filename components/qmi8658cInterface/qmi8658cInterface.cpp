// TODO: Fix Z angle calculation

#include "qmi8658cInterface.hpp"

#if CONFIG_LV_COLOR_DEPTH == 32
#define LCD_BIT_PER_PIXEL (24)
#elif CONFIG_LV_COLOR_DEPTH == 16
#define LCD_BIT_PER_PIXEL (16)
#endif

SensorQMI8658 qmi8658cInterface::qmi{};
RollPitch qmi8658cInterface::RP{};

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
            RollPitch RP = getPitchAndRoll();
            //ESP_LOGI(QMI8658C_TAG, "Roll: %f, Pitch: %f", RP.roll, RP.pitch);
            
            /*if (qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
                ESP_LOGI(QMI8658C_TAG, "ACCEL: %f, %f, %f", acc.x, acc.y, acc.z);
            } else {
                ESP_LOGE(QMI8658C_TAG, "Failed to read accelerometer data");
            }*/

            /*if (qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
                ESP_LOGI(QMI8658C_TAG, "GYRO: %f, %f, %f", gyr.x, gyr.y, gyr.z);
            } else {
                ESP_LOGE(QMI8658C_TAG, "Failed to read gyroscope data");
            }*/

            //ESP_LOGI(QMI8658C_TAG, "Timestamp: %u, Temperature: %.2f *C", (unsigned int)qmi.getTimestamp(), qmi.getTemperature_C()); // Casting to unsigned int
        } else {
            ESP_LOGW(QMI8658C_TAG, "Data not ready yet");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

RollPitch  qmi8658cInterface::getPitchAndRoll()
{
    RollPitch RP;
    IMUdata acc;
    struct Angle
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    } angle;

    if(qmi.getAccelerometer(acc.x,acc.y,acc.z))
    {
        float mask = (float)acc.x / sqrt(((float)acc.y * (float)acc.y + (float)acc.z * (float)acc.z));
        angle.x = atan(mask) * 57.29578f; // 180/π=57.29578
        mask = (float)acc.y / sqrt(((float)acc.x * (float)acc.x + (float)acc.z * (float)acc.z));
        angle.y = atan(mask) * 57.29578f; // 180/π=57.29578
        mask = (float)acc.z / sqrt(((float)acc.x * (float)acc.x + (float)acc.y * (float)acc.y)); 
        angle.z = atan(mask) * 57.29578f; // 180/π=57.29578
        //angle z is not correctly calculated
        RP.roll = angle.y;
        RP.pitch = atan2(acc.z,acc.x) * 57.29578f;
    }
    else 
    {
        ESP_LOGE(QMI8658C_TAG, "Failed to read accelerometer data");
    }
    //ESP_LOGI(QMI8658C_TAG, "Angle X: %f, Angle Y: %f, Angle Z: %f", angle.x, angle.y,angle.z);
            
    
    return RP;

}


void qmi8658cInterface::init() {
    setup_sensor();
    xTaskCreate(read_sensor_data, "sensor_read_task", 4096, NULL, 10, NULL);
}