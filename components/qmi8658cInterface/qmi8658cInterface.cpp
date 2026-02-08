// TODO: Fix Z angle calculation

#include "qmi8658cInterface.hpp"

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
        SensorQMI8658::LPF_MODE_3,
        true
    );

    // Configure gyroscope
    /*qmi.configGyroscope(
        SensorQMI8658::GYR_RANGE_64DPS,
        SensorQMI8658::GYR_ODR_896_8Hz,
        SensorQMI8658::LPF_MODE_3,
        true
    );*/

    // Enable gyroscope and accelerometer
    //qmi.enableGyroscope();
    qmi.enableAccelerometer();

    ESP_LOGI(QMI8658C_TAG, "Ready to read data...");
}

void qmi8658cInterface::read_sensor_data() {
    
    assert(Queue_ != nullptr);
    
    while (1) {
        if (qmi.getDataReady()) {
            RollPitch rp{};
            if (getPitchAndRoll(rp)) 
            {
                xQueueOverwrite(Queue_, &rp);
            }
        } else {
            ESP_LOGW(QMI8658C_TAG, "Data not ready yet");
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

bool qmi8658cInterface::getPitchAndRoll(RollPitch& out)
{
    IMUdata acc{};

    if (!qmi.getAccelerometer(acc.x, acc.y, acc.z)) {
        ESP_LOGE(QMI8658C_TAG, "Failed to read accelerometer data");
        return false;
    }

    // Protect against invalid data
    const float eps = 1e-6f;
    if (fabs(acc.y) < eps && fabs(acc.z) < eps) {
        ESP_LOGW(QMI8658C_TAG, "Invalid accel data (near zero)");
        return false;
    }

    constexpr float RAD_TO_DEG = 57.2957795f; // 180/π=57.29578
    
    float mask = (float)acc.y / sqrt(((float)acc.x * (float)acc.x + (float)acc.z * (float)acc.z));
     
    out.roll  = atan(mask) * RAD_TO_DEG;
    out.pitch = atan2(acc.z,acc.x) * RAD_TO_DEG;
    out.temperature = qmi.getTemperature_C();
    /*ESP_LOGI(QMI8658C_TAG,
             "Roll: %.2f deg, Pitch: %.2f deg",
             out.roll, out.pitch);*/

    return true;
}

void qmi8658cInterface::task_entry(void* arg)
{
    auto* self = static_cast<qmi8658cInterface*>(arg);
    self->read_sensor_data();
}

void qmi8658cInterface::init() {
    setup_sensor();
    xTaskCreate(task_entry, "QMI Task", 4096, this, 10, NULL);
}