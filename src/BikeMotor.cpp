#include "BikeMotor.hpp"

BikeMotor* BikeMotor::instance = nullptr;

BikeMotor::BikeMotor() {
    // Initialize PWM
    ledcSetup(0, 1000, 8);  // Channel 0, 1kHz frequency, 8-bit resolution; TODO, look up specs for motor controller
    ledcAttachPin(Config::MOTOR_CONTROLLER_PIN, 0);  // Attach PWM to the specified GPIO pin
}

BikeMotor* BikeMotor::getInstance() {
    if (instance == nullptr) {
        instance = new BikeMotor();
    }
    return instance;
}

void BikeMotor::startTask() {
    // Create the task that will control the motor
    xTaskCreate(
        bikeMotorTask,                        // Task function
        "BikeMotor_Task",                     // Task name
        Constants::BIKE_MOTOR_TASK_STACK_SIZE, // Stack size
        this,                                 // Task parameter (this pointer)
        Constants::DEFAULT_TASK_PRIORITY,             // Task priority
        &taskHandle                           // Task handle
    );
}

void BikeMotor::bikeMotorTask(void* parameter) {
    BikeMotor* bikeMotor = static_cast<BikeMotor*>(parameter);

    // The task will loop indefinitely to control the motor speed
    while (true) {
        double speed = 0.0;
        // Retrieve the speed from a queue, sensor, or other source
        // Example: xQueueReceive(SyncObjects::motorSpeedQueue, &speed, pdMS_TO_TICKS(100));

        // Update PWM duty cycle based on the speed
        bikeMotor->setSpeed(speed);
        
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay for task cycle
    }
}

void BikeMotor::setSpeed(double speed) {
    double dutyCycle = wheelPID(speed);
    setPWM(dutyCycle);
}

void BikeMotor::setPWM(double dutyCycle) {
    // Set PWM duty cycle to control motor speed (0 to 255 for 8-bit resolution)
    int pwmValue = static_cast<int>(dutyCycle * 255);
    ledcWrite(0, pwmValue);  // Channel 0, set the PWM duty cycle
}

double BikeMotor::wheelPID(double speed) {
    // TODO: do PID

    double dutyCycle = 0.5;
    return dutyCycle;
}
