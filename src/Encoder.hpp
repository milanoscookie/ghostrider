#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "SyncObjects.hpp"
#include "Constants.hpp"

class Encoder {
public:
    struct Config {
        static constexpr gpio_num_t PIN_A = GPIO_NUM_18;  // Quadrature A signal
        static constexpr gpio_num_t PIN_B = GPIO_NUM_19;  // Quadrature B signal
        static constexpr int EDGES_PER_REV = 2400;        // Encoder resolution
    };

    struct State {
        volatile int32_t position = 0;        // Current position in encoder ticks
        double angle = 0.0;                   // Current angle in degrees (-180 to +180)
        double angularVelocity = 0.0;         // Angular velocity in degrees/second
        double angularAcceleration = 0.0;     // Angular acceleration in degrees/second²
        uint64_t lastTime = 0;                // Last update timestamp in microseconds
        double lastAngle = 0.0;               // Last angle for velocity calculation
    };

    static Encoder* getInstance();
    void resetState();
    void startTask();
    const State& getState() const { return state; }

private:
    Encoder() = default;
    
    // Initialize the hardware PCNT module
    void setup();
    
    // PCNT interrupt handler for overflow/underflow events
    static void IRAM_ATTR pcntEventHandler(void* arg);
    
    // FreeRTOS task for monitoring and calculations
    static void encoderTask(void* parameter);
    
    static Encoder* instance;
    State state;
    volatile int32_t overflowCount = 0;  // Tracks counter overflows/underflows
    TaskHandle_t taskHandle = nullptr;
};

#endif // ENCODER_HPP