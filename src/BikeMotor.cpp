#include "BikeMotor.hpp"

BikeMotor* BikeMotor::instance = nullptr;

BikeMotor::BikeMotor() {
    // Initialize PWM
    ledcSetup(0, 50, 8);  // Channel 0, 1kHz frequency, 8-bit resolution; TODO, look up specs for motor controller
    ledcAttachPin(Config::MOTOR_CONTROLLER_PIN, 0);  // Attach PWM to the specified GPIO pin
    pinMode(Config::CONTROL_A, OUTPUT);
    pinMode(Config::CONTROL_B, OUTPUT);
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
        Constants::BIKE_MOTOR_TASK_STACK_SIZE, // Stack size
        this,                                 // Task parameter (this pointer)
        Constants::DEFAULT_TASK_PRIORITY,             // Task priority
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
            //Serial.printf("%f %f\n", pitch, targetCurrent);
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
        // Set PWM duty cycle to control motor speed (0 to 255 for 8-bit resolution)
        //int pwmValue = static_cast<int>(dutyCycle * 255);
        //ledcWrite(0, pwmValue);  // Channel 0, set the PWM duty cycle

    //SyncObjects::printValues("TEST: ",targetCurrent,0.0,0.0);
    float actualCurrent = readMotorCurrent();
    //Serial.printf("%f %f\n", targetCurrent,actualCurrent); //WORKS
    double pwmDuty = currentPID(targetCurrent, actualCurrent);  // Inner loop PID
    //pwmDuty = constrain(pwmDuty, 0.0, 1.0);
    pwmDuty *= Config::MAX_DUTY;
    int pwmValue = static_cast<int>(pwmDuty * 255);
    //SyncObjects::printValues("",targetCurrent,actualCurrent,pwmDuty);
    //Serial.println(String(targetCurrent) + ", " + String(actualCurrent) + ", " + String(pwmValue));
    if (abs(pwmValue) < 3 * Config::MAX_DUTY / 0.125) {
        ledcDetachPin(Config::MOTOR_CONTROLLER_PIN);
        pinMode(Config::MOTOR_CONTROLLER_PIN, OUTPUT);
        digitalWrite(Config::MOTOR_CONTROLLER_PIN, LOW);  // Ensure LOW
    } 
    else {
        if (pwmValue > 0) {
            digitalWrite(Config::CONTROL_A, 1);
            digitalWrite(Config::CONTROL_B, 0);
        }
        if (pwmValue < 0) {
            digitalWrite(Config::CONTROL_A, 0);
            digitalWrite(Config::CONTROL_B, 1);
        }
        ledcAttachPin(Config::MOTOR_CONTROLLER_PIN, 0);
        // ledcWrite(0, abs(pwmValue));
        ledcWrite(0, abs(242));
}
    //ledcWrite(0, 0);
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