#include "SyncObjects.hpp"

// Define static members
SemaphoreHandle_t SyncObjects::serialMutex = nullptr;
SemaphoreHandle_t SyncObjects::i2cMutex = nullptr;
SemaphoreHandle_t SyncObjects::madgwickMutex = nullptr;

QueueHandle_t SyncObjects::servoPositionQueue = nullptr;
QueueHandle_t SyncObjects::imuQueue = nullptr;
QueueHandle_t SyncObjects::encoderQueue = nullptr;

void SyncObjects::initialize()
{
    serialMutex = xSemaphoreCreateMutex();
    i2cMutex = xSemaphoreCreateMutex();
    madgwickMutex = xSemaphoreCreateMutex();

    imuQueue = xQueueCreate(50, sizeof(IMU::Data));         // Ensure IMU::Data is visible
    servoPositionQueue = xQueueCreate(10, sizeof(double));
    encoderQueue = xQueueCreate(4096, sizeof(Encoder::Event)); // Ensure Encoder::Event is visible

    // Optionally give mutexes initial tokens
    if (serialMutex) xSemaphoreGive(serialMutex);
    if (i2cMutex) xSemaphoreGive(i2cMutex);
    if (madgwickMutex) xSemaphoreGive(madgwickMutex);
}

void SyncObjects::printMessage(const String &message)
{
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        Serial.println(message);
        xSemaphoreGive(serialMutex);
    }
}

void SyncObjects::printValues(const String &label, float v1, float v2, float v3)
{
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(200)) == pdTRUE)
    {
        Serial.print(label);
        Serial.print(": ");
        Serial.print(v1, 3);
        Serial.print(", ");
        Serial.print(v2, 3);
        Serial.print(", ");
        Serial.println(v3, 3);
        xSemaphoreGive(serialMutex);
    }
}
