#ifndef WEBSERVER_HPP
#define WEBSERVER_HPP

#include <Arduino.h>
#include <Adafruit_LSM6DS3.h>
#include <Wire.h>
#include <MadgwickAHRS.h>
#include <ESPAsyncWebServer.h>
#include <Constants.hpp>
#include <IMU.hpp>
#include <SyncObjects.hpp>
#include <WiFi.h>

class WebServer {
public:
    volatile bool started = false;
    enum Command { FORWARD, BACKWARD, STEADY };

    // Config structure
    struct Config {
        static constexpr int port = 80;
        static constexpr const char* SSID = "Device-Northwestern";  // Changed String to const char*
    };
    static void handleImuRequest(AsyncWebServerRequest *request);

    // Singleton instance getter
    static WebServer* getInstance();

    // Public methods
    void startTask();
    bool setup();

    // Data structures to hold IMU data
    struct OutboundData {
        float roll;
        float pitch;
        float yaw;
    };

    struct InboundData {
        float tilt;
        float velocity;
    };

    // Singleton instance
    static WebServer* instance;

    // WiFi connection method
    bool connectToWifi();
    void updateImuData();

private:
    bool initialized = false;                    // Flag to check if the server is initialized
    TaskHandle_t taskHandle = nullptr;           // Handle for the task
    AsyncWebServer* server = nullptr;            // Pointer to the web server

    // Private methods
    static void serverTask(void* parameter);

    // Private constructor to enforce singleton pattern
    WebServer() = default;

    // Shared data between tasks
    OutboundData outboundData;
    InboundData inboundData;

    const char* WEBPAGE_STRING = R"rawliteral(<!DOCTYPE html>
<html>
<head>
  <title>ESP32 IMU Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
</head>
<body>
  <h1>ESP32 IMU Controller</h1>
  
  <div>
    <h2>IMU Data</h2>
    <p>Roll: <span id="roll">0.00</span></p>
    <p>Pitch: <span id="pitch">0.00</span></p>
    <p>Yaw: <span id="yaw">0.00</span></p>
    <button onclick="fetchIMUData()">Refresh IMU Data</button>
  </div>
  <!--
  TODO: uncomment
  <div>
    <h2>Controls</h2>
    <div>
      <label for="tiltRange">Tilt: <span id="tiltValue">0</span></label>
      <input type="range" id="tiltRange" min="-90" max="90" value="0" onchange="updateTiltValue(this.value)">
      <button onclick="sendTilt()">Set Tilt</button>
    </div>
    <div>
      <label for="velocityRange">Velocity: <span id="velocityValue">0</span></label>
      <input type="range" id="velocityRange" min="-100" max="100" value="0" onchange="updateVelocityValue(this.value)">
      <button onclick="sendVelocity()">Set Velocity</button>
    </div>
  </div>
  -->
  
  <div id="status"></div>

  <script>
    // Initialize variables
    let refreshInterval = null;
    // Set baseUrl directly (no user input needed)
    const baseUrl = '';
    
    // Update value displays when sliders change
    function updateTiltValue(value) {
      document.getElementById('tiltValue').innerText = value;
    }
    
    function updateVelocityValue(value) {
      document.getElementById('velocityValue').innerText = value;
    }
    
    // Fetch IMU data from the ESP32
    function fetchIMUData() {
      fetch(baseUrl + '/imu')
        .then(response => response.json())
        .then(data => {
          document.getElementById('roll').innerText = data.roll;
          document.getElementById('pitch').innerText = data.pitch;
          document.getElementById('yaw').innerText = data.yaw;
        })
        .catch(error => {
          showStatus('Error fetching IMU data: ' + error.message, true);
        });
    }
    
    // Send tilt value to ESP32
    function sendTilt() {
      const tilt = document.getElementById('tiltRange').value;
      sendPutRequest('/tilt', tilt);
    }
    
    // Send velocity value to ESP32
    function sendVelocity() {
      const velocity = document.getElementById('velocityRange').value;
      sendPutRequest('/velocity', velocity);
    }
    
    // Helper function for PUT requests
    function sendPutRequest(endpoint, value) {
      const formData = new FormData();
      formData.append('value', value);
      
      fetch(baseUrl + endpoint, {
        method: 'PUT',
        body: formData
      })
      .then(response => response.json())
      .then(data => {
        showStatus(data.status || data.error, data.error !== undefined);
      })
      .catch(error => {
        showStatus('Error: ' + error.message, true);
      });
    }
    
    // Display status messages
    function showStatus(message, isError) {
      const statusElement = document.getElementById('status');
      statusElement.innerText = message;
      statusElement.style.color = isError ? 'red' : 'green';
      
      // Clear status after 3 seconds
      setTimeout(() => {
        statusElement.innerText = '';
      }, 3000);
    }
    
    // Initialize when page loads
    window.onload = function() {
      // Start data refresh cycle immediately
      fetchIMUData();
      refreshInterval = setInterval(fetchIMUData, 1000);
    };
  </script>
</body>
</html>)rawliteral";
};

#endif // WEBSERVER_HPP
