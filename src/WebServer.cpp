#include "WebServer.hpp"

WebServer *WebServer::instance = nullptr;

WebServer *WebServer::getInstance()
{
    if (instance == nullptr)
    {
        instance = new WebServer();
    }
    return instance;
}

bool WebServer::connectToWifi()
{
    WiFi.begin(Config::SSID);
    // Wait for connection
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30)
    { // 30 attempts (15 seconds max)
        SyncObjects::printMessage(".");
        delay(500); // Wait 500ms between attempts
        attempts++;
    }
    // Check if the connection was successful
    if (WiFi.status() == WL_CONNECTED)
    {
        // Use Arduino's String instead of std::string
        String wifiStatus = "\nConnected to WiFi!\nIP Address: " + WiFi.localIP().toString();
        SyncObjects::printMessage(wifiStatus);  // Log the connection status
        return true;
    }
    else
    {
        // Use Arduino's String instead of std::string
        String wifiStatus = "Failed to connect to WiFi\n";
        SyncObjects::printMessage(wifiStatus);  // Log the failure
        return false;
    }
}

void WebServer::startTask()
{
    setup();

    xTaskCreate(
        WebServer::serverTask,
        "WebServerTask",
        Constants::WEB_SERVER_TASK_STACK_SIZE,
        this,
        Constants::DEFAULT_TASK_PRIORITY,
        &taskHandle);
}

bool WebServer::setup()
{
    if (connectToWifi())
    {
        if (initialized)
            return true;

        outboundData.roll = 99;
        outboundData.pitch = 99;
        outboundData.yaw = 99;

        server = new AsyncWebServer(Config::port);
        // Serve the index.html page on root
        server->on("/", HTTP_GET, [this](AsyncWebServerRequest *request){
            request->send(200, "text/html", WEBPAGE_STRING );  // Send the HTML page directly
        });
        // GET /imu - returns roll, pitch, yaw
        server->on("/imu", HTTP_GET, [this](AsyncWebServerRequest *request)
                   {
            String json = "{";
            json += "\"roll\":" + String(outboundData.roll, 2) + ",";
            json += "\"pitch\":" + String(outboundData.pitch, 2) + ",";
            json += "\"yaw\":" + String(outboundData.yaw, 2);
            json += "}";
            request->send(200, "application/json", json); });
        // PUT /tilt - sets inbound tilt value
        server->on("/tilt", HTTP_PUT, [this](AsyncWebServerRequest *request)
                   {
            if (request->hasParam("value", true)) {
                inboundData.tilt = request->getParam("value", true)->value().toFloat();
                request->send(200, "application/json", "{\"status\":\"tilt updated\"}");
            } else {
                request->send(400, "application/json", "{\"error\":\"Missing value parameter\"}");
            } });
        // PUT /velocity - sets inbound velocity value
        server->on("/velocity", HTTP_PUT, [this](AsyncWebServerRequest *request)
                   {
            if (request->hasParam("value", true)) {
                inboundData.velocity = request->getParam("value", true)->value().toFloat();
                request->send(200, "application/json", "{\"status\":\"velocity updated\"}");
            } else {
                request->send(400, "application/json", "{\"error\":\"Missing value parameter\"}");
            } });
        server->begin();
        initialized = true;
        return true;
    }
    return false;
}

void WebServer::serverTask(void *parameter)
{

    WebServer *self = static_cast<WebServer *>(parameter);
    IMU::Angles angles;
    while (true)
    {
        self->outboundData.roll = 89;
        self->outboundData.pitch = 88;
        self->outboundData.yaw = 87;
        // if (xQueuePeek(SyncObjects::anglesQueue, &angles, pdMS_TO_TICKS(50)) == pdTRUE)
        // {
        //     // Save to outbound data
        //     self->outboundData.roll = angles.roll;
        //     self->outboundData.pitch = angles.pitch;
        //     self->outboundData.yaw = angles.yaw;
        // }
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}