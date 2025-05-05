#ifndef IMU_HPP
#define IMU_HPP

#include <Arduino.h>
#include <Adafruit_LSM6DS3.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <stdint.h>
#include <WebServer.hpp>

#include "SyncObjects.hpp"
#include "Constants.hpp"

class IMU {
public:
    struct Data {
        uint32_t timestamp;
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        bool data_valid;
    };

    struct Angles {
        uint32_t timestamp;
        float roll, pitch, yaw;
    };

    static IMU* getInstance();

    void startTask(); 
    bool initialize(uint8_t addr=0x6AU);
    void configureIMU();

    bool readData(Data& data);
    void updateFilter(const Data& data, Angles& angles);

private:
    Adafruit_LSM6DS3 lsm6ds3;
    Madgwick filter;
    bool initialized = false;

    static IMU* instance;
    TaskHandle_t taskHandle = nullptr;

    static void readTask(void* parameter);

    IMU() = default;
};

#endif // IMU_H

