#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "SyncObjects.hpp"
#include "Constants.hpp"

class Encoder {
public:
    enum class Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    };

    struct Event {
        Direction direction;
        unsigned long timestampMicros;
    };

    struct Config {
        static constexpr gpio_num_t PIN_A = GPIO_NUM_18;
        static constexpr gpio_num_t PIN_B = GPIO_NUM_19;
        static constexpr int EDGES_PER_REV = 2400;
    };

    struct State {
        volatile int lastEncoded = 0;
        volatile int position = 0;
        double angle = 0.0;
        double angularVelocity = 0.0;
        double angularAcceleration = 0.0;
        unsigned long lastTime = 0;
        double lastAngle = 0.0;
    };

    static Encoder* getInstance();

    void resetState();
    void startTask();
    const State& getState() const;

private:
    Encoder() = default;

    static void IRAM_ATTR handleInterrupt();
    static void encoderTask(void* parameter);

    static Encoder* instance;
    State state;
    TaskHandle_t taskHandle = nullptr;
};

#endif // ENCODER_HPP
