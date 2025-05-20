#include "WebServer.hpp"

WebServer* WebServer::instance = nullptr;

WebServer* WebServer::getInstance() {
    if (instance == nullptr) {
        instance = new WebServer();
    }
    return instance;
}

bool WebServer::connectToWifi() {
    Serial.println("\nAttempting to connect to WiFi");
    WiFi.begin(Config::SSID);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        // Serial.print(".");
        Serial.printf("Attempt %d...\n", attempts + 1);
        vTaskDelay(pdMS_TO_TICKS(500));  // Wait 500 ms
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nConnected to WiFi!\nIP Address: %s\n", WiFi.localIP().toString().c_str());
        started = true;
        return true;
    } else {
        Serial.println("Failed to connect to WiFi");
        return false;
    }
}

void WebServer::startTask() {
    xTaskCreate(
        WebServer::serverTask,
        "WebServerTask",
        Constants::WEB_SERVER_TASK_STACK_SIZE,
        this,
        Constants::DEFAULT_TASK_PRIORITY,
        &taskHandle
    );
}

bool WebServer::setup() {
    started=true;
    if (!connectToWifi()) return false;
    if (initialized) return true;

    outboundData = {99, 99, 99};  // roll, pitch, yaw default

    server = new AsyncWebServer(Config::port);

    server->on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "text/html", WEBPAGE_STRING);
    });

    // Modified IMU endpoint to use current values on each request
    server->on("/imu", HTTP_GET, WebServer::handleImuRequest);
    // server->on("/imu", HTTP_GET, [this](AsyncWebServerRequest* request) {
    //     const String json = String("{\"roll\":") + String(this->outboundData.roll, 2) +
    //                       ",\"pitch\":" + String(this->outboundData.pitch, 2) +
    //                       ",\"yaw\":" + String(this->outboundData.yaw, 2) + "}";
    //     request->send(200, "application/json", json);
    // });

    server->on("/tilt", HTTP_PUT, [this](AsyncWebServerRequest* request) {
        if (request->hasParam("value", true)) {
            inboundData.tilt = request->getParam("value", true)->value().toFloat();
            request->send(200, "application/json", "{\"status\":\"tilt updated\"}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing value parameter\"}");
        }
    });

    server->on("/velocity", HTTP_PUT, [this](AsyncWebServerRequest* request) {
        if (request->hasParam("value", true)) {
            inboundData.velocity = request->getParam("value", true)->value().toFloat();
            request->send(200, "application/json", "{\"status\":\"velocity updated\"}");
        } else {
            request->send(400, "application/json", "{\"error\":\"Missing value parameter\"}");
        }
    });

    server->begin();
    initialized = true;
    return true;
}

void WebServer::serverTask(void* parameter) {
    WebServer* self = static_cast<WebServer*>(parameter);
    IMU::Angles angles;

    // Run setup only once at the start of the task
    if (!self->started) {
        if (!self->setup()) {
            Serial.println("Failed to setup WebServer");
            vTaskDelete(NULL);  // Delete the task if setup fails
            return;
        }
    }

    while (true) {
        // self->outboundData.roll = 0.0f;
        // self->outboundData.pitch = 0.0f;
        // self->outboundData.yaw = 0.0f;
        
        self->outboundData.roll =  IMU::getInstance()->lastRoll;
        self->outboundData.pitch = IMU::getInstance()->lastPitch;
        self->outboundData.yaw =   IMU::getInstance()->lastYaw;
        Serial.printf("%f, %f, %f\n", self->outboundData.roll, self->outboundData.pitch, self->outboundData.yaw);
        vTaskDelay(pdMS_TO_TICKS(10));  // Update every 10ms
    }
}

void WebServer::handleImuRequest(AsyncWebServerRequest *request) {
    // Get the singleton instance
    WebServer* ws = WebServer::getInstance();
    
    // Create JSON with current values
    const String json = String("{\"roll\":") + String(ws->outboundData.roll, 2) +
                    ",\"pitch\":" + String(ws->outboundData.pitch, 2) +
                    ",\"yaw\":" + String(ws->outboundData.yaw, 2) + "}";
    
    request->send(200, "application/json", json);
}