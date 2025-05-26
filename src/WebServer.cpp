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
    Serial.print("MAC Address: ");
    Serial.println(WiFi.macAddress());  // Print MAC address every attempt
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        Serial.printf("Attempt %d...\n", attempts + 1);
        Serial.print("MAC Address: ");
        Serial.println(WiFi.macAddress());  // Print MAC address every attempt
        vTaskDelay(pdMS_TO_TICKS(500));  // Wait 500 ms
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WiFi!");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
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
    if (!connectToWifi()) return false;
    started = true;

    if (initialized) return true;

    outboundData = {99, 99, 99};  // roll, pitch, yaw default

    server = new AsyncWebServer(Config::port);

    server->on("/", HTTP_GET, [this](AsyncWebServerRequest* request) {
        request->send(200, "text/html", WEBPAGE_STRING);
    });

    server->on("/imu", HTTP_GET, WebServer::handleImuRequest);

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


void WebServer::updateImuData() {
    outboundData.roll  = IMU::getInstance()->lastRoll;
    outboundData.pitch = IMU::getInstance()->lastPitch;
    outboundData.yaw   = IMU::getInstance()->lastYaw;
}

void WebServer::serverTask(void* parameter) {
    WebServer* self = static_cast<WebServer*>(parameter);

    if (!self->started) {
        if (!self->setup()) {
            Serial.println("Failed to setup WebServer");
            // vTaskDelete(NULL); // Optional: cleanly kill task
            return;
        }
    }

    while (true) {
        self->updateImuData();
        vTaskDelay(pdMS_TO_TICKS(10));  // Update every 10ms
    }
}

void WebServer::handleImuRequest(AsyncWebServerRequest* request) {
    WebServer* ws = WebServer::getInstance();

    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}",
             ws->outboundData.roll,
             ws->outboundData.pitch,
             ws->outboundData.yaw);

    request->send(200, "application/json", buffer);
}
