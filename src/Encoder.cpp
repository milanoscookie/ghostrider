#include "Encoder.hpp"
#include <math.h>
#include "driver/pcnt.h"

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
    
    // Reset hardware counter
    pcnt_counter_clear(PCNT_UNIT_0);
}

void Encoder::setup() {
    // Configure PCNT unit
    pcnt_config_t pcntConfig = {
        .pulse_gpio_num = Config::PIN_A,        // Pulse input GPIO (A signal)
        .ctrl_gpio_num = Config::PIN_B,         // Control input GPIO (B signal)
        .lctrl_mode = PCNT_MODE_REVERSE,        // Reverse count direction if B is high
        .hctrl_mode = PCNT_MODE_KEEP,           // Keep count direction if B is low
        .pos_mode = PCNT_COUNT_INC,             // Count up on rising edge of pulse input
        .neg_mode = PCNT_COUNT_DEC,             // Count down on falling edge of pulse input
        .counter_h_lim = 32767,                 // Max limit value
        .counter_l_lim = -32768,                // Min limit value
        .unit = PCNT_UNIT_0,                    // PCNT unit number
        .channel = PCNT_CHANNEL_0,              // PCNT channel number
    };
    
    // Initialize PCNT unit
    pcnt_unit_config(&pcntConfig);
    
    // Filter out glitches
    pcnt_set_filter_value(PCNT_UNIT_0, 100);  // Filter pulses shorter than 100 clock cycles
    pcnt_filter_enable(PCNT_UNIT_0);
    
    // Start counting
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    
    // Set up interrupt for position tracking
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_L_LIM);
    pcnt_isr_register(pcntEventHandler, NULL, 0, NULL);
    pcnt_intr_enable(PCNT_UNIT_0);
}

void IRAM_ATTR Encoder::pcntEventHandler(void* arg) {
    uint32_t status = 0;
    pcnt_get_event_status(PCNT_UNIT_0, &status);
    
    Encoder* encoder = Encoder::getInstance();
    
    // Handle counter overflow
    if (status & PCNT_EVT_H_LIM) {
        encoder->overflowCount++;
    }
    // Handle counter underflow
    else if (status & PCNT_EVT_L_LIM) {
        encoder->overflowCount--;
    }
    
    // Clear interrupt
    pcnt_counter_clear(PCNT_UNIT_0);
}

void Encoder::startTask() {
    // Initialize hardware counter
    setup();
    
    // Create monitoring task
    xTaskCreate(
        encoderTask,
        "Encoder_Task",
        Constants::ENCODER_TASK_STACK_SIZE,
        this,
        Constants::DEFAULT_TASK_PRIORITY,
        &taskHandle
    );
}

void Encoder::encoderTask(void* parameter) {
    Encoder* encoder = static_cast<Encoder*>(parameter);
    encoder->resetState();
    
    int16_t rawCount = 0;
    int32_t lastPosition = 0;
    constexpr double degreesPerEdge = 360.0 / Config::EDGES_PER_REV;
    
    while (true) {
        // Get counter value from hardware
        pcnt_get_counter_value(PCNT_UNIT_0, &rawCount);
        
        // Calculate absolute position using overflows and raw count
        int32_t currentPosition = (encoder->overflowCount * 65536) + rawCount;
        
        // Only process if the position changed
        if (currentPosition != lastPosition) {
            // Calculate delta
            int32_t delta = currentPosition - lastPosition;
            lastPosition = currentPosition;
            
            // Update position and angle
            encoder->state.position = currentPosition;
            encoder->state.angle = fmod((currentPosition * degreesPerEdge) + 180.0, 360.0) - 180.0;
            
            // Calculate timing and derivatives
            uint64_t currentTime = micros();
            double deltaTime = (currentTime - encoder->state.lastTime) / 1000000.0;
            
            if (deltaTime > 0) {
                // Calculate angular velocity (degrees per second)
                encoder->state.angularVelocity = (delta * degreesPerEdge) / deltaTime;
                
                // Calculate angular acceleration
                double prevVelocity = encoder->state.angularVelocity;
                encoder->state.angularVelocity = (delta * degreesPerEdge) / deltaTime;
                encoder->state.angularAcceleration = (encoder->state.angularVelocity - prevVelocity) / deltaTime;
                
                encoder->state.lastAngle = encoder->state.angle;
                encoder->state.lastTime = currentTime;
            }
            
            // Log data
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