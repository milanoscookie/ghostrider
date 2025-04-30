#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP
class Constants {
public:
    // Physical constants
    static constexpr float GRAVITY = 9.8f;
    
    // Task priorities and stack sizes
    static constexpr int IMU_TASK_STACK_SIZE = 2048;
    static constexpr int CONTROL_TASK_STACK_SIZE = 4096;
    static constexpr int SERVO_TASK_STACK_SIZE = 4096;
    static constexpr int ENCODER_TASK_STACK_SIZE = 4096;
    static constexpr int BIKE_MOTOR_TASK_STACK_SIZE = 2048;

    static constexpr int DEFAULT_TASK_PRIORITY = 2;
};

#endif // CONSTANTS_H