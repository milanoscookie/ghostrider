#include "Encoder.hpp"
#include <math.h>

Encoder* Encoder::instance = nullptr;

Encoder* Encoder::getInstance() {
    if (instance == nullptr) {
        instance = new Encoder();
    }
    return instance;
}

void Encoder::resetState() {
    state.position = 0;
    state.angle = 0.0;
    state.angularVelocity = 0.0;
    state.angularAcceleration = 0.0;
    state.lastTime = 0;
    state.lastAngle = 0.0;
}

void Encoder::startTask() {
    xTaskCreate(
        encoderTask,
        "Encoder_Task",
        Constants::ENCODER_TASK_STACK_SIZE,
        this,
        Constants::DEFAULT_TASK_PRIORITY,
        &taskHandle
    );
}

void IRAM_ATTR Encoder::handleInterrupt() {
    Encoder* encoder = Encoder::getInstance();

    int MSB = digitalRead(Config::PIN_A);
    int LSB = digitalRead(Config::PIN_B);

    int encoded = (MSB << 1) | LSB;
    int sum = (encoder->state.lastEncoded << 2) | encoded;

    Event event;
    event.timestampMicros = micros();

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
        event.direction = Direction::CLOCKWISE;
        xQueueSendFromISR(SyncObjects::encoderQueue, &event, NULL);
    } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
        event.direction = Direction::COUNTER_CLOCKWISE;
        xQueueSendFromISR(SyncObjects::encoderQueue, &event, NULL);
    }

    encoder->state.lastEncoded = encoded;
}

void Encoder::encoderTask(void* parameter) {
    Encoder* encoder = static_cast<Encoder*>(parameter);

    pinMode(Config::PIN_A, INPUT_PULLUP);
    pinMode(Config::PIN_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(Config::PIN_A), handleInterrupt, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Config::PIN_B), handleInterrupt, CHANGE);

    encoder->resetState();

    int delta = 0;
    Event event;
    constexpr double degreesPerEdge = 360.0 / Config::EDGES_PER_REV;


    while (true) {
        // Serial.println("in loop");
        if (xQueueReceive(SyncObjects::encoderQueue, &event, pdMS_TO_TICKS(20))) {
            delta = (event.direction == Direction::CLOCKWISE) ? 1 : -1;
            encoder->state.position += delta;
            encoder->state.angle += degreesPerEdge * delta;
            encoder->state.angle = fmod(encoder->state.angle + 180.0, 360.0) - 180.0;

            double deltaTime = (event.timestampMicros - encoder->state.lastTime) / 1000000.0;

            if (deltaTime > 0) {
                encoder->state.angularVelocity =
                    (encoder->state.angle - encoder->state.lastAngle) / deltaTime;
                encoder->state.angularAcceleration =
                    (encoder->state.angularVelocity - 
                     ((encoder->state.lastAngle - encoder->state.lastAngle) / deltaTime)) / deltaTime;

                encoder->state.lastAngle = encoder->state.angle;
                encoder->state.lastTime = event.timestampMicros;
            }

            if (xSemaphoreTake(SyncObjects::serialMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                Serial.printf("Position = %d, Angle = %.2f, Angular Velocity = %.2f deg/s, Angular Acceleration = %.2f deg/s²\n",
                              encoder->state.position,
                              encoder->state.angle,
                              encoder->state.angularVelocity,
                              encoder->state.angularAcceleration);
                xSemaphoreGive(SyncObjects::serialMutex);
            }
        }
    }
}
