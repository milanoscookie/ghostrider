#include "SideControl.hpp"

SideControl* SideControl::instance = nullptr;

SideControl::SideControl() : imu(IMU::getInstance()), taskHandle(nullptr) {}

SideControl* SideControl::getInstance() {
    if (instance == nullptr) {
        instance = new SideControl();
    }
    return instance;
}

void SideControl::startTask() {
    xTaskCreate(
        controlTask,                   // Task function
        "SideControl_Task",            // Task name
        Constants::SIDE_CONTROL_TASK_STACK_SIZE, // Stack size
        this,                          // Task parameter (this pointer)
        Constants::DEFAULT_TASK_PRIORITY,      // Task priority
        &taskHandle                    // Task handle
    );
}

void SideControl::controlTask(void* parameter) {
    SideControl* control = (SideControl*)parameter;
    IMU::Data imuData;
    IMU::Angles angles;

    while (true) {
        if (xQueueReceive(SyncObjects::imuQueue, &imuData, 0) == pdPASS) {
            SyncObjects::printValues("Accel: ", imuData.accel_x, imuData.accel_y, imuData.accel_z);

            control->imu->updateFilter(imuData, angles);
            SyncObjects::printValues("Angles: ", angles.roll, angles.pitch, angles.yaw);

            // Use roll angle for servo control
            double servoAngle = angles.roll;
            xQueueSend(SyncObjects::servoPositionQueue, &servoAngle, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
