#ifndef SIDECONTROL_HPP
#define SIDECONTROL_HPP

#include "IMU.hpp"
#include "SyncObjects.hpp"
#include "Constants.hpp"

// Side control class (roll control)
class SideControl {
private:
    static SideControl* instance;
    TaskHandle_t taskHandle;
    IMU* imu;

    static void controlTask(void* parameter); // Static task function

public:
    // Constructor
    SideControl();
    struct Config {
        static constexpr double MAX_ANGLE = 180.0;  // Servo range in degrees
    };

    // Singleton pattern
    static SideControl* getInstance();

    // Starts the control task
    void startTask();
};

#endif // SIDECONTROL_HPP
