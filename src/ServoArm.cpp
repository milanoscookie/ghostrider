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
    //servo.setPeriodHertz(50);
    //servo.attach(Config::SERVO_PIN, Config::MIN_PULSE_US, Config::MAX_PULSE_US);
    //servo.writeMicroseconds(Config::NEUTRAL_US);  // Start with the neutral position

    ledcSetup(1,50,8);
    ledcAttachPin(Config::SERVO_PIN,1);
    ledcWrite(1,19);
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
    IMU::Data imuData;
    IMU::Angles imuAngles;
    double position;

    while (true) {
        if (xQueueReceive(SyncObjects::imuQueue, &imuData, pdMS_TO_TICKS(100)) == pdTRUE) {
            IMU::getInstance()->updateFilter(imuData, imuAngles);            // Default position if no commands received
            position = servoArm->servoPID(imuAngles.roll);
            //Serial.println(String(imuAngles.roll) + ", " + String(position));
            servoArm->setPosition(position);
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Delay for a while before checking again
    }
}

void ServoArm::setPosition(double position) {
    // Constrain position within allowable range
    //position = -1 * (position - Config::MAX_ANGLE);
    position = constrain(position, 0, Config::MAX_ANGLE);

    // Map the position to a corresponding pulse width
    double pulse = Config::MIN_PULSE_US + (position / Config::MAX_ANGLE) * 
                     (Config::MAX_PULSE_US - Config::MIN_PULSE_US);

    // Move the servo to the computed position
    int pulseWidth = ((pulse / 20000.0) * 255.0) + 5;
    Serial.println(String(position) + ", " + String(pulseWidth));
    
    ledcWrite(1, pulseWidth);
}

double ServoArm::servoPID(double roll) {
    static double integral = 0.0;
    static double prevError = 0.0;
    static uint32_t lastTime = millis();

    double error = 0.0 - roll;  // Target is zero roll (upright)

    uint32_t now = millis();
    double dt = (now - lastTime) / 1000.0;
    lastTime = now;

    integral += error * dt;
    double derivative = (error - prevError) / dt;
    prevError = error;

    double output = Config::kP_servo * error + Config::kI_servo * integral + Config::kD_servo * derivative;

    // Map output to servo angle (e.g., -30° to +30°)
    double angle = constrain(90 + -1.0f * output, 0, Config::MAX_ANGLE);
    //Serial.println(String(error) + ", " + String(integral) + ", " + String(derivative) + ", " + String(angle));
    return angle;
}