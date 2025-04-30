// #include <Arduino.h>
// #include <Wire.h>
// #include <Adafruit_LSM6DS3.h>
// #include <ESP32Servo.h>
// #include <MadgwickAHRS.h>

// #include <cmath>
// #include "esp_task_wdt.h"
// #include "esp_int_wdt.h"

// #define GRAVITY 9.8

// // Servo configuration
// #define SERVO_PIN 13
// #define SERVO_MIN_PULSE_US 500
// #define SERVO_MAX_PULSE_US 2500
// #define SERVO_NEUTRAL_US 1500
// #define SERVO_DEAD_BAND_US 3
// #define SERVO_MAX_ANGLE 270


// volatile int lastEncoded = 0;
// volatile int encoderPos = 0;
// #define ENC_A_PIN GPIO_NUM_18
// #define ENC_B_PIN GPIO_NUM_19

// // defined in degrees
// double angle;
// double angularVelocity;
// double angularAcceleration;

// unsigned long lastTime = 0;  // Last time in milliseconds
// double lastAngle = 0.0;  // Last angle (degrees)

// #define EDGES_PER_REV 2400
// // Semaphore handles
// SemaphoreHandle_t serialMutex;
// SemaphoreHandle_t i2cMutex;
// SemaphoreHandle_t servoMutex;
// SemaphoreHandle_t wifiMutex;
// SemaphoreHandle_t madgwickMutex;

// QueueHandle_t servoPositionQueue;
// QueueHandle_t imuQueue;
// QueueHandle_t encoderQueue;

// // IMU sensor instance
// Adafruit_LSM6DS3 lsm6ds3;

// // Madgwick filter
// Madgwick filter;

// // Data types
// enum EncoderDirection
// {
//   ENCODER_TURN_CW,
//   ENCODER_TURN_CCW
// };
// typedef struct {
//   EncoderDirection direction;
//   unsigned long timestampMicros;
// } EncoderEvent;
// typedef struct
// {
//   uint32_t timestamp;
//   float accel_x, accel_y, accel_z;
//   float gyro_x, gyro_y, gyro_z;
//   bool data_valid;
// } IMU_Data_t;

// typedef struct
// {
//   uint32_t timestamp;
//   float roll, pitch, yaw;
// } Angles_t;


// // Task prototypes
// void TaskReadIMU(void *pvParameters);
// void TaskFrontControl(void *pvParameters);
// void TaskSideControl(void *pvParameters);
// void TaskMoveServo(void *pvParameters);
// void TaskReadEncoder(void *pvParameters);
// void IRAM_ATTR handleEncoderInterrupt();

// void setup()
// {
//   // Disable task watchdog
//   esp_task_wdt_deinit();

//   // Disable interrupt watchdog
//   disableCore0WDT();
//   disableCore1WDT();

//   Serial.begin(115200);
//   // SDA, SCL
//   Wire.begin();
//   vTaskDelay(pdMS_TO_TICKS(1000));

//   serialMutex = xSemaphoreCreateMutex();
//   i2cMutex = xSemaphoreCreateMutex();
//   madgwickMutex = xSemaphoreCreateMutex();

//   imuQueue = xQueueCreate(50, sizeof(IMU_Data_t));
//   servoPositionQueue = xQueueCreate(10, sizeof(double));
//   encoderQueue = xQueueCreate(4096, sizeof(EncoderEvent));

//   if (serialMutex)
//     xSemaphoreGive(serialMutex);
//   if (i2cMutex)
//     xSemaphoreGive(i2cMutex);
//   if (madgwickMutex)
//     xSemaphoreGive(madgwickMutex);

//   xTaskCreate(TaskReadIMU, "ReadIMU", 2048, NULL, 2, NULL);
//   // xTaskCreate(TaskSideControl, "SideControl", 4096, NULL, 2, NULL);
//   // xTaskCreate(TaskMoveServo, "MoveServo", 4096, NULL, 2, NULL);
//   // xTaskCreate(TaskReadEncoder, "ReadEncoder", 4096, NULL, 2, NULL);
// }

// void loop()
// {
//   // Empty, all logic in tasks
// }

// void TaskReadIMU(void *pvParameters)
// {
//   IMU_Data_t imuData;
//   bool lsm6ds3_ok = false;

//   while (!lsm6ds3_ok)
//   {
//     if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
//     {
//       if (!lsm6ds3.begin_I2C())
//       {
//         if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(200)) == pdTRUE)
//         {
//           Serial.println("Failed to find LSM6DS3 chip");
//           xSemaphoreGive(serialMutex);
//         }
//       }
//       else
//       {
//         lsm6ds3_ok = true;
//         lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
//         lsm6ds3.setAccelDataRate(LSM6DS_RATE_104_HZ);
//         lsm6ds3.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
//         lsm6ds3.setGyroDataRate(LSM6DS_RATE_104_HZ);
//       }
//       xSemaphoreGive(i2cMutex);
//     }
//     vTaskDelay(pdMS_TO_TICKS(500));
//   }

//   if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(200)) == pdTRUE)
//   {
//     Serial.println("LSM6DS3 initialized successfully");
//     xSemaphoreGive(serialMutex);
//   }

//   if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(200)) == pdTRUE)
//   {
//     filter.begin(104, 1.0); // Madgwick filter sampling rate
//     xSemaphoreGive(madgwickMutex);
//   }

//   while (1)
//   {
//     if (lsm6ds3_ok)
//     {
//       sensors_event_t accel, gyro, temp;

//       if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE)
//       {
//         lsm6ds3.getEvent(&accel, &gyro, &temp);
//         xSemaphoreGive(i2cMutex);
//       }

//       imuData.timestamp = millis();
//       imuData.accel_x = accel.acceleration.x;
//       imuData.accel_y = accel.acceleration.y;
//       imuData.accel_z = accel.acceleration.z;
//       imuData.gyro_x = gyro.gyro.x;
//       imuData.gyro_y = gyro.gyro.y;
//       imuData.gyro_z = gyro.gyro.z;
//       imuData.data_valid = true;

//       xQueueSend(imuQueue, &imuData, pdMS_TO_TICKS(100));
//     }

//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

// void TaskFrontControl(void *pvParameters)
// {
//   // Placeholder
// }

// void TaskSideControl(void *pvParameters)
// {
//   IMU_Data_t imuData;
//   Angles_t angles;

//   while (1)
//   {
//     if (xQueueReceive(imuQueue, &imuData, pdMS_TO_TICKS(100)) == pdPASS)
//     {
//       if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(200)) == pdTRUE)
//       {
//         Serial.print(imuData.accel_x); Serial.print(", ");
//         Serial.print(imuData.accel_y); Serial.print(", ");
//         Serial.println(imuData.accel_z);
//         xSemaphoreGive(serialMutex);
//       }

//       angles.timestamp = imuData.timestamp;

//       if (xSemaphoreTake(madgwickMutex, pdMS_TO_TICKS(200)) == pdTRUE)
//       {
//         filter.updateIMU(
//             imuData.gyro_x * (PI / 180), imuData.gyro_y * (PI / 180), imuData.gyro_z * (PI / 180),
//             imuData.accel_x / GRAVITY, imuData.accel_y / GRAVITY, imuData.accel_z / GRAVITY);

//         angles.roll = filter.getRoll();
//         angles.pitch = filter.getPitch();
//         angles.yaw = filter.getYaw();

//         if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(200)) == pdTRUE)
//         {
//           Serial.print(angles.roll);
//           Serial.print(", ");
//           Serial.print(angles.pitch);
//           Serial.print(", ");
//           Serial.println(angles.yaw);
//           xSemaphoreGive(serialMutex);
//         }

//         xSemaphoreGive(madgwickMutex);
//       }

//       double servoAngle = angles.roll;
//       xQueueSend(servoPositionQueue, &servoAngle, pdMS_TO_TICKS(100));
//     }

//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }

// void TaskMoveServo(void *pvParameters)
// {
//   Servo myServo;
//   double position;

//   myServo.attach(SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
//   myServo.writeMicroseconds(SERVO_NEUTRAL_US);

//   while (1)
//   {
//     if (xQueueReceive(servoPositionQueue, &position, pdMS_TO_TICKS(100)) == pdTRUE)
//     {
//       position = constrain(position, 0, SERVO_MAX_ANGLE);
//     }
//     position = 130;
//     int pulseWidth = SERVO_MIN_PULSE_US + (position / SERVO_MAX_ANGLE) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
//     myServo.writeMicroseconds(pulseWidth);

//     vTaskDelay(pdMS_TO_TICKS(10));
//   }
// }


// void TaskReadEncoder(void *pvParameters)
// {
//   pinMode(ENC_A_PIN, INPUT_PULLUP);
//   pinMode(ENC_B_PIN, INPUT_PULLUP);

//   attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), handleEncoderInterrupt, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), handleEncoderInterrupt, CHANGE);

//   int position = 0;
//   angle = 0.0;
//   angularVelocity = 0.0;
//   angularAcceleration = 0.0;
  
//   int delta = 0;
//   EncoderEvent event;

//   double degreesPerEdge = 360.0 / EDGES_PER_REV;

//   while (1)
//   {
//     if (xQueueReceive(encoderQueue, &event, pdMS_TO_TICKS(20))) {
//       delta = (event.direction == ENCODER_TURN_CW) ? 1 : -1;
//       position += delta;
//       angle += degreesPerEdge * delta;
//       angle = fmod(angle + 180.0, 360.0) - 180.0;

//       double deltaTime = (event.timestampMicros - lastTime) / 1000000.0;

//       if (deltaTime > 0) {
//         angularVelocity = (angle - lastAngle) / deltaTime;
//         angularAcceleration = (angularVelocity - (lastAngle - lastAngle) / deltaTime) / deltaTime;
//         lastAngle = angle;
//         lastTime = event.timestampMicros;
//       }

//       if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(20)) == pdTRUE)
//       {
//         Serial.printf("Position = %d, Angle = %.2f, Angular Velocity = %.2f deg/s, Angular Acceleration = %.2f deg/s²\n",
//                       position, angle, angularVelocity, angularAcceleration);
//         xSemaphoreGive(serialMutex);
//       }
//     }
//   }
// }

// void IRAM_ATTR handleEncoderInterrupt()
// {
//   int MSB = digitalRead(ENC_A_PIN);
//   int LSB = digitalRead(ENC_B_PIN);

//   int encoded = (MSB << 1) | LSB;
//   int sum = (lastEncoded << 2) | encoded;

//   EncoderEvent event;
//   event.timestampMicros = micros();

//   if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
//   {
//     event.direction = ENCODER_TURN_CW;
//     xQueueSendFromISR(encoderQueue, &event, NULL);
//   }
//   else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
//   {
//     event.direction = ENCODER_TURN_CCW;
//     xQueueSendFromISR(encoderQueue, &event, NULL);
//   }

//   lastEncoded = encoded;
// }
