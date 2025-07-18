#ifndef SERVOARM_HPP
#define SERVOARM_HPP

#include <ESP32Servo.h>
#include "SyncObjects.hpp"
#include "Constants.hpp"

// ServoArm class for controlling a servo motor
class ServoArm {
public:
    struct Config {
        static constexpr int MIN_PULSE_US = 500;
        static constexpr int MAX_PULSE_US = 2500;
        static constexpr int NEUTRAL_US = 1500;
        static constexpr int DEAD_BAND_US = 3;
        static constexpr int MAX_ANGLE = 270;

        static constexpr int SERVO_PIN = 2;

        static constexpr double kP_servo = 4.5;
        static constexpr double kI_servo = 0.0;
        static constexpr double kD_servo = 0.0;

    };
    
private:
    Servo servo;                    // Servo object for controlling the motor
    static ServoArm* instance;       // Singleton instance
    TaskHandle_t taskHandle = nullptr;

    // Static task function that runs in FreeRTOS
    static void servoTask(void* parameter);

public:
    // Constructor
    ServoArm();

    // Singleton pattern
    static ServoArm* getInstance();

    // Initialize the servo arm (attach servo to pin and set neutral position)
    void initialize();

    // Start the servo control task
    void startTask();

    // Set the position of the servo arm (angle in degrees)
    void setPosition(double position);

    double servoPID(double roll);
};

#endif // SERVOARM_HPP
