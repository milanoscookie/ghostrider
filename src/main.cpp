#include <Arduino.h>
#include <Adafruit_LSM6DS3.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <MadgwickAHRS.h>
#include <cmath>
#include "esp_task_wdt.h"
#include "esp_int_wdt.h"

#include "SyncObjects.hpp"
#include "BikeMotor.hpp"
#include "Constants.hpp"
#include "Encoder.hpp"
#include "FrontControl.hpp"
#include "SideControl.hpp"
#include "ServoArm.hpp"
#include "IMU.hpp"


// Main setup function
void setup() {
    // Disable watchdogs
    esp_task_wdt_deinit();
    disableCore0WDT();
    disableCore1WDT();
    
    Serial.begin(115200);
    Wire.begin();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize synchronization objects
    SyncObjects::initialize();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize and start tasks
     IMU::getInstance()->startTask();
     //SideControl::getInstance()->startTask();
     ServoArm::getInstance()->startTask();
     Encoder::getInstance()->startTask();
     //FrontControl::getInstance()->startTask();
     //WebServer::getInstance()->startTask();
     BikeMotor::getInstance()->startTask();

    
}

void loop() {
    // Keep this blank
}