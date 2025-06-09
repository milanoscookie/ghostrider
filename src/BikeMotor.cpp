#include "BikeMotor.hpp"

BikeMotor* BikeMotor::instance = nullptr;

BikeMotor::BikeMotor() {
    // Setup PWM for EN pin (speed control)
    ledcSetup(0, 5000, 8);  // 5kHz PWM, 8-bit resolution
    ledcAttachPin(Config::PWM_PIN, 0);   // Enable pin for motor speed

    // Setup direction pins
    pinMode(Config::CONTROL_A, OUTPUT);  // IN1
    pinMode(Config::CONTROL_B, OUTPUT);  // IN2

    // Default motor state: stopped
    digitalWrite(Config::CONTROL_A, LOW);
    digitalWrite(Config::CONTROL_B, LOW);
    ledcWrite(0, 0);
}

BikeMotor* BikeMotor::getInstance() {
    if (instance == nullptr) {
        instance = new BikeMotor();
    }
    return instance;
}

void BikeMotor::startTask() {
    //BikeMotor();
    // Create the task that will control the motor
    xTaskCreate(
        bikeMotorTask,                        // Task function
        "BikeMotor_Task",                     // Task name
        Constants::BIKE_MOTOR_TASK_STACK_SIZE,// Stack size
        this,                                 // Task parameter (this pointer)
        Constants::DEFAULT_TASK_PRIORITY,     // Task priority
        &taskHandle                           // Task handle
    );
}

void BikeMotor::bikeMotorTask(void* parameter) {
    BikeMotor* bikeMotor = static_cast<BikeMotor*>(parameter);
    IMU::Data imuData;
    IMU::Angles imuAngles;

    // The task will loop indefinitely to control the motor speed
    while (true) {
        if (xQueueReceive(SyncObjects::imuQueue, &imuData, pdMS_TO_TICKS(20)) == pdTRUE) {
            IMU::getInstance()->updateFilter(imuData, imuAngles);

            // Read encoder data directly
            Encoder::State encState = Encoder::getInstance()->getState();

            // Use pitch and angular velocity for PID
            double pitch = imuAngles.pitch;
            double speed = encState.angularVelocity;
            double targetCurrent = bikeMotor->wheelPID(speed, pitch) * Config::MAX_CURRENT_AMPS;  // Outputs amps
            //Serial.println(String(pitch) + "    " + String(targetCurrent));
            bikeMotor->setPWM(targetCurrent);  // Uses current PID

            //bikeMotor->setSpeed(speed, pitch);

        }

        // Retrieve the speed from a queue, sensor, or other source
        // Example: xQueueReceive(SyncObjects::motorSpeedQueue, &speed, pdMS_TO_TICKS(100));

        // Update PWM duty cycle based on the speed
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Delay for task cycle
    }
}

void BikeMotor::setSpeed(double speed, double pitch) {
    double dutyCycle = wheelPID(speed, pitch);
    setPWM(dutyCycle);
}

void BikeMotor::setPWM(double targetCurrent) {
    float actualCurrent = readMotorCurrent();
    double pwmDuty = currentPID(targetCurrent, actualCurrent);

    pwmDuty *= Config::MAX_DUTY;
    pwmDuty = constrain(pwmDuty, -1.0, 1.0);

    int pwmValue = static_cast<int>(abs(pwmDuty) * 255);
    
    if (pwmValue < 4) {
        // Stop motor
        digitalWrite(Config::CONTROL_A, LOW);
        digitalWrite(Config::CONTROL_B, LOW);
        ledcWrite(0, 0);
        return;
    }

    if (pwmDuty > 0) {
        // Forward
        digitalWrite(Config::CONTROL_A, HIGH);
        digitalWrite(Config::CONTROL_B, LOW);
        ledcWrite(0, pwmValue);
    } else {
        // Reverse
        digitalWrite(Config::CONTROL_A, LOW);
        digitalWrite(Config::CONTROL_B, HIGH);
        ledcWrite(0, pwmValue);
    }
}

double BikeMotor::wheelPID(double speed, double pitch) {
    static double integral = 0.0f;
    static double prevError = 0.0f;
    static uint32_t lastTimeWheel = millis();
    uint32_t now = millis();
    double dt = (now - lastTimeWheel) / 1000.0f;
    lastTimeWheel = now;

    double error = pitch;  // Target pitch is 0
    integral += error * dt;
    double derivative = (error - prevError) / dt;
    prevError = error;
    
    double output = Config::kP_wheel * error + Config::kI_wheel * integral + Config::kD_wheel * derivative;

    //Serial.printf("%f %f %f %f\n", error, integral, derivative, output);
    // Dampen with wheel speed (optional)

    // Clamp
    output /= Config::TILT_SCALE;
    output = constrain(output, -1.0f, 1.0f);
    //Serial.printf("Error: %f, Output: %f\n", error, output);
    return output;
}

double BikeMotor::readMotorCurrent() {
    int raw = analogRead(Config::CURRENT_SENSE_PIN);
    double voltage = (raw / Config::ADC_RESOLUTION) * Config::VREF;      // in Volts
    double current = (voltage * 1000.0) / Config::MV_PER_AMP;     // convert V to mV, then to Amps
    return current;
    // return 0.0;
}

double BikeMotor::currentPID(double targetCurrent, double actualCurrent) {
    // Serial.printf("%f %f\n", targetCurrent,actualCurrent); WORKS
    static double integral = 0.0;
    static double prevError = 0.0;
    static uint32_t lastTimeCurrent = millis();

    uint32_t now = millis();
    double dt = (now - lastTimeCurrent) / 1000.0;
    lastTimeCurrent = now;
    if (dt < 1e-4) {
        return 0.0;
    }
    //Serial.printf("%f %f %f\n",actualCurrent, targetCurrent, dt);
    //Serial.printf("%f\n",targetCurrent, actualCurrent, dt);
    //Serial.println(String(actualCurrent) + ", " + String(targetCurrent) + ", " + String(dt));

    double error = targetCurrent - actualCurrent;
    integral += error * dt;
    double derivative = (error - prevError) / dt;
    prevError = error;

    double output = Config::kP_current * error + Config::kI_current * integral + Config::kD_current * derivative;
    //Serial.printf("%f %f %f\n",targetCurrent,actualCurrent,output);
    //Serial.println(String(error) + ", " + String(integral) + ", " + String(derivative) + ", " + String(dt) + ", " + String(prevError));
    return output;
    // return 0.0;
}