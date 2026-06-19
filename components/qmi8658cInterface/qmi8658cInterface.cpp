// TODO: Fix Z angle calculation

#include "qmi8658cInterface.hpp"

SensorQMI8658 qmi8658cInterface::qmi{};
RollPitch qmi8658cInterface::RP{};
RollPitch qmi8658cInterface::RPOffset
{
    .roll = 0,
    .pitch = 0,
    .temperature = 0
};

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
        SensorQMI8658::ACC_ODR_31_25Hz,
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
    {
        std::lock_guard<std::mutex> lock(RPOffsetMutex);
        RPOffset = getStoredOffset().value_or(RollPitch{0,0,0});
    }

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
            //ESP_LOGW(QMI8658C_TAG, "Data not ready yet");
        }
        vTaskDelay(QMI_TASK_TIME_MS / portTICK_PERIOD_MS);
    }
}

void qmi8658cInterface::setInclinometerOffset()
{
    
    //Reset RP offset to get real angle from getPitchAndRoll, if this isnt done it will increment the error
    RPOffset.roll = 0;
    RPOffset.pitch= 0;
    if (qmi.getDataReady()) {
            RollPitch rp{};
            if (getPitchAndRoll(rp)) 
            {
                std::lock_guard<std::mutex> lock(RPOffsetMutex);
                RPOffset = rp;//use current position as offset, it will zero out
                ESP_LOGI(QMI8658C_TAG, "Current Offset \n roll: %f \n pitch: %f",RPOffset.roll);
                //Store in flash
                (void)setStoredOffset(rp);
            }
            else
            {
                ESP_LOGE(QMI8658C_TAG, "Failed to get offset from data");
            }
    }
}

esp_err_t qmi8658cInterface::setStoredOffset(RollPitch &rp) const
{
    esp_err_t nvs_err;

    nvs_handle_t nvsHandle;

    nvs_err = nvs_open("QMIOffset",NVS_READWRITE,&nvsHandle);
    if (nvs_err != ESP_OK) {
        ESP_LOGE(QMI8658C_TAG, "Error (%s) opening NVS handle!", esp_err_to_name(nvs_err));
        nvs_close(nvsHandle);
        return nvs_err;
    }

    ESP_LOGI(QMI8658C_TAG,"Reading Roll");
    int32_t lRoll = static_cast<int32_t>(rp.roll);
    nvs_err = nvs_set_i32(nvsHandle,"QMIOffsetRoll",lRoll);
    switch (nvs_err) {
        case ESP_OK:
            ESP_LOGI(QMI8658C_TAG, "Read Roll = %" PRIu32, lRoll);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(QMI8658C_TAG, "The value is not initialized yet!");
            break;
        default:
            ESP_LOGE(QMI8658C_TAG, "Error (%s) reading!", esp_err_to_name(nvs_err));
    }

    ESP_LOGI(QMI8658C_TAG,"Reading Pitch");
    int32_t lPitch = static_cast<int32_t>(rp.pitch);;
    nvs_err = nvs_set_i32(nvsHandle,"QMIOffsetPitch",lPitch);
    switch (nvs_err) {
        case ESP_OK:
            ESP_LOGI(QMI8658C_TAG, "Read Pitch = %" PRIu32, lPitch);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(QMI8658C_TAG, "The value is not initialized yet!");
            break;
        default:
            ESP_LOGE(QMI8658C_TAG, "Error (%s) reading!", esp_err_to_name(nvs_err));
    }
    nvs_close(nvsHandle);
    return nvs_err;

}

std::optional<RollPitch> qmi8658cInterface::getStoredOffset() const
{
    RollPitch storedOffset;
    esp_err_t nvs_err;

    nvs_handle_t nvsHandle;

    nvs_err = nvs_open("QMIOffset",NVS_READONLY,&nvsHandle);
    if (nvs_err != ESP_OK) {
        ESP_LOGE(QMI8658C_TAG, "Error (%s) opening NVS handle!", esp_err_to_name(nvs_err));
        nvs_close(nvsHandle);
        return std::nullopt;
    }

    ESP_LOGI(QMI8658C_TAG,"Reading Roll");
    int32_t lRoll;
    nvs_err = nvs_get_i32(nvsHandle,"QMIOffsetRoll",&lRoll);
    switch (nvs_err) {
        case ESP_OK:
            ESP_LOGI(QMI8658C_TAG, "Read Roll = %" PRIu32, lRoll);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(QMI8658C_TAG, "The value is not initialized yet!");
            break;
        default:
            ESP_LOGE(QMI8658C_TAG, "Error (%s) reading!", esp_err_to_name(nvs_err));
    }

    ESP_LOGI(QMI8658C_TAG,"Reading Pitch");
    int32_t lPitch;
    nvs_err = nvs_get_i32(nvsHandle,"QMIOffsetPitch",&lPitch);
    switch (nvs_err) {
        case ESP_OK:
            ESP_LOGI(QMI8658C_TAG, "Read Pitch = %" PRIu32, lPitch);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(QMI8658C_TAG, "The value is not initialized yet!");
            break;
        default:
            ESP_LOGE(QMI8658C_TAG, "Error (%s) reading!", esp_err_to_name(nvs_err));
    }

    if(nvs_err!=ESP_OK)
    {
        nvs_close(nvsHandle);
        return std::nullopt;
    }
    else
    {
        storedOffset.pitch = static_cast<float>(lPitch);
        storedOffset.roll = static_cast<float>(lRoll);
        nvs_close(nvsHandle);
        return std::optional<RollPitch>(storedOffset);
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

    //Apply offset
    out.roll  -= RPOffset.roll;
    out.pitch -= RPOffset.pitch;
    
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