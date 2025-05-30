#ifndef BIKE_MOTOR_HPP
#define BIKE_MOTOR_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "SyncObjects.hpp"
#include "IMU.hpp"
#include "Constants.hpp"

class BikeMotor {
public:

    BikeMotor();
    struct Config {
        static constexpr gpio_num_t MOTOR_CONTROLLER_PIN = GPIO_NUM_23;
        static constexpr gpio_num_t CURRENT_SENSE_PIN = GPIO_NUM_15;
        static constexpr gpio_num_t CONTROL_A = GPIO_NUM_32;
        static constexpr gpio_num_t CONTROL_B = GPIO_NUM_33;

        static constexpr double MAX_CURRENT_AMPS = 5.0;
        static constexpr double MAX_DUTY = 0.5;
        static constexpr double WHEEL_RADIUS = 16; // inches

        static constexpr double kP_wheel = 0.5;
        static constexpr double kI_wheel = 0;//0.2;
        static constexpr double kD_wheel = 0.00;//0.01;

        static constexpr double kP_current = 0.1;
        static constexpr double kI_current = 0;//0.2;
        static constexpr double kD_current = 0;//0.01;

        static constexpr double TILT_SCALE = 10.0; // what pitch we want PID to saturate at

        static constexpr double ADC_RESOLUTION = 4096.0;
        static constexpr double VREF = 3.3;
        static constexpr double MV_PER_AMP = 140.0; // mV per amp

    };

    static BikeMotor* getInstance();
    void startTask();
    void setSpeed(double speed, double pitch); // inches/sec

private:
    static void bikeMotorTask(void* parameter);
    static BikeMotor* instance;
    TaskHandle_t taskHandle = nullptr;

    void setPWM(double targetCurrent);
    double wheelPID(double speed, double pitch);
    double currentPID(double targetCurrent, double actualCurrent);
    double readMotorCurrent();
};

#endif // BIKE_MOTOR_HPP
