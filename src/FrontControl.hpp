#ifndef FRONTCONTROL_HPP
#define FRONTCONTROL_HPP

#include <Arduino.h>
#include "Constants.hpp"

// Front control class (pitch control)
class FrontControl {
private:
    static FrontControl* instance;
    TaskHandle_t taskHandle;

    static void controlTask(void* parameter); // Static task function

public:
    // Constructor
    FrontControl();

    // Singleton pattern
    static FrontControl* getInstance();

    // Starts the control task
    void startTask();
};

#endif // FRONTCONTROL_HPP
