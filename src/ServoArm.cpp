#include "ServoArm.hpp"

ServoArm* ServoArm::instance = nullptr;

ServoArm::ServoArm() = default;

ServoArm* ServoArm::getInstance() {
    if (instance == nullptr) {
        instance = new ServoArm();
    }
    return instance;
}

void ServoArm::initialize() {
    // Attach the servo to the specified pin and pulse width range
    servo.attach(Config::PIN, Config::MIN_PULSE_US, Config::MAX_PULSE_US);
    servo.writeMicroseconds(Config::NEUTRAL_US);  // Start with the neutral position
}

void ServoArm::startTask() {
    initialize();  // Initialize the servo first

    // Create the task that will manage the servo's position
    xTaskCreate(
        servoTask,                        // Task function
        "ServoArm_Task",                  // Task name
        Constants::SERVO_TASK_STACK_SIZE, // Stack size
        this,                             // Task parameter (this pointer)
        Constants::DEFAULT_TASK_PRIORITY,         // Task priority
        &taskHandle                       // Task handle
    );
}

void ServoArm::servoTask(void* parameter) {
    ServoArm* servoArm = (ServoArm*)parameter;
    double position;

    while (true) {
        if (xQueueReceive(SyncObjects::servoPositionQueue, &position, pdMS_TO_TICKS(100)) == pdTRUE) {
            servoArm->setPosition(position);  // Set the servo position if a command is received
        } else {
            // Default position if no commands received
            servoArm->setPosition(130);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Delay for a while before checking again
    }
}

void ServoArm::setPosition(double position) {
    // Constrain position within allowable range
    position = constrain(position, 0, Config::MAX_ANGLE);

    // Map the position to a corresponding pulse width
    int pulseWidth = Config::MIN_PULSE_US + (position / Config::MAX_ANGLE) * 
                     (Config::MAX_PULSE_US - Config::MIN_PULSE_US);

    // Move the servo to the computed position
    servo.writeMicroseconds(pulseWidth);
}
