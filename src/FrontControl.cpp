#include "FrontControl.hpp"

FrontControl* FrontControl::instance = nullptr;

FrontControl::FrontControl() : taskHandle(nullptr) {}

FrontControl* FrontControl::getInstance() {
    if (instance == nullptr) {
        instance = new FrontControl();
    }
    return instance;
}

void FrontControl::startTask() {
    xTaskCreate(
        controlTask,                   // Task function
        "FrontControl_Task",           // Task name
        Constants::SIDE_CONTROL_TASK_STACK_SIZE, // Stack size
        this,                          // Task parameter (this pointer)
        Constants::DEFAULT_TASK_PRIORITY,      // Task priority
        &taskHandle                    // Task handle
    );
}

void FrontControl::controlTask(void* parameter) {
    // Placeholder for front control implementation
    // This would be similar to SideControl but using pitch instead of roll
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
