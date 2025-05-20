#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP
class Constants {
public:
    // Physical constants
    static constexpr float GRAVITY = 9.8f;
    static constexpr float WHEEL_RADIUS = 8 * 0.0254; // in m
    
    // Task priorities and stack sizes
    static constexpr int IMU_TASK_STACK_SIZE = 2048;
    static constexpr int SIDE_CONTROL_TASK_STACK_SIZE = 4096;
    static constexpr int SERVO_TASK_STACK_SIZE = 4096;
    static constexpr int ENCODER_TASK_STACK_SIZE = 4096;
    static constexpr int BIKE_MOTOR_TASK_STACK_SIZE = 2048;
    static constexpr int WEB_SERVER_TASK_STACK_SIZE = 9128;

    static constexpr int DEFAULT_TASK_PRIORITY = 2;


    static constexpr float ENC_ROT_MATRIX[3][3]
    {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    
};

#endif // CONSTANTS_H