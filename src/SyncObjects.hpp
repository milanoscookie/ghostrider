#ifndef SYNC_OBJECTS_H
#define SYNC_OBJECTS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "IMU.hpp"
#include "Encoder.hpp"

class SyncObjects {
public:
    static SemaphoreHandle_t serialMutex;
    static SemaphoreHandle_t i2cMutex;
    static SemaphoreHandle_t madgwickMutex;
    
    static QueueHandle_t servoPositionQueue;
    static QueueHandle_t imuQueue;
    static QueueHandle_t encoderQueue;

    static void initialize();

    static void printMessage(const String &message);
    static void printValues(const String &label, float v1, float v2, float v3);
};

#endif // SYNC_OBJECTS_H
