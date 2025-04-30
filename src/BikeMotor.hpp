#ifndef BIKE_MOTOR_HPP
#define BIKE_MOTOR_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "SyncObjects.hpp"
#include "Constants.hpp"

class BikeMotor {
public:

    BikeMotor();
    struct Config {
        static constexpr gpio_num_t MOTOR_CONTROLLER_PIN = GPIO_NUM_20;
        static constexpr double WHEEL_RADIUS = 16; // inches

        static constexpr double kP = 0.0;
        static constexpr double kI = 0.0;
        static constexpr double kD = 0.0;
    };

    static BikeMotor* getInstance();
    void startTask();
    void setSpeed(double speed); // inches/sec

private:
    static void bikeMotorTask(void* parameter);
    static BikeMotor* instance;
    TaskHandle_t taskHandle = nullptr;

    void setPWM(double dutyCycle);
    double wheelPID(double speed);
};

#endif // BIKE_MOTOR_HPP
