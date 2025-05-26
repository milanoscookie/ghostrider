#include "IMU.hpp"

IMU* IMU::instance = nullptr;

IMU* IMU::getInstance() {
    if (instance == nullptr) {
        instance = new IMU();
    }
    return instance;
}

void IMU::startTask() {
    xTaskCreate(
        IMU::readTask,
        "IMU_Task",
        Constants::IMU_TASK_STACK_SIZE,
        this,
        Constants::DEFAULT_TASK_PRIORITY,
        &taskHandle
    );
}

void IMU::readTask(void* parameter) {
    IMU* imu = static_cast<IMU*>(parameter);
    
    while (!imu->initialize()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    Data data;
    while (true) {
        if (imu->readData(data)) {
            xQueueSend(SyncObjects::imuQueue, &data, pdMS_TO_TICKS(100));
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool IMU::initialize(uint8_t addr) {
    bool success = false;

    if (xSemaphoreTake(SyncObjects::i2cMutex, portMAX_DELAY) == pdTRUE) {
        success = lsm6ds3.begin_I2C(addr);

        if (success) {
            initialized = true;
            configureIMU();
            SyncObjects::printMessage("LSM6DS3 initialized successfully");

            if (xSemaphoreTake(SyncObjects::madgwickMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                filter.begin(104, 1.0f);
                xSemaphoreGive(SyncObjects::madgwickMutex);
            }
        } else {
            SyncObjects::printMessage("Failed to find LSM6DS3 chip");
        }

        xSemaphoreGive(SyncObjects::i2cMutex);
    }

    return success;
}

void IMU::configureIMU() {
    lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds3.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds3.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lsm6ds3.setGyroDataRate(LSM6DS_RATE_104_HZ);
}

bool IMU::readData(Data& data) {
    if (!initialized) return false;

    sensors_event_t accel, gyro, temp;

    if (xSemaphoreTake(SyncObjects::i2cMutex, portMAX_DELAY) == pdTRUE) {
        lsm6ds3.getEvent(&accel, &gyro, &temp);
        xSemaphoreGive(SyncObjects::i2cMutex);
    }

    data.timestamp = millis();
    data.accel_x = accel.acceleration.x;
    data.accel_y = accel.acceleration.y;
    data.accel_z = accel.acceleration.z;
    data.gyro_x = gyro.gyro.x;
    data.gyro_y = gyro.gyro.y;
    data.gyro_z = gyro.gyro.z;
    data.data_valid = true;

    return true;
}

void IMU::updateFilter(const Data& data, Angles& angles) {
    if (xSemaphoreTake(SyncObjects::madgwickMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        filter.updateIMU(
            data.gyro_x * DEG_TO_RAD, data.gyro_y * DEG_TO_RAD, data.gyro_z * DEG_TO_RAD,
            data.accel_x / Constants::GRAVITY, data.accel_y / Constants::GRAVITY, data.accel_z / Constants::GRAVITY
        );

        angles.timestamp = data.timestamp;
        angles.roll = filter.getRoll();
        angles.pitch = filter.getPitch();
        angles.yaw = filter.getYaw();

        lastRoll = angles.roll;
        lastPitch = angles.pitch;
        lastYaw = angles.yaw;


        xSemaphoreGive(SyncObjects::madgwickMutex);
    }
}