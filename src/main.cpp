#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define LED_PIN 48

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "DHT20.h"

constexpr char WIFI_SSID[] = "Min";
constexpr char WIFI_PASSWORD[] = "123456789";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;

const char* ledStateControlKey = "ledState";
volatile bool ledState = false;
QueueHandle_t ledStateQueue;
DHT20 dht20;

#define MAX_DEVICES 10

void ledAttributeCallback(char* topic, byte* payload, unsigned int length);

struct Alldevice {
    const char* name;
    const char* token;
    WiFiClient* wifiClient;
    PubSubClient* mqttClient;
    bool connected;
    void (*callback)(char*, byte*, unsigned int);
};

Alldevice devices[MAX_DEVICES];
int deviceCount = 0;

int ledDeviceIndex = -1;
int dhtDeviceIndex = -1;

int addDevice(const char* name, const char* token, void (*callback)(char*, byte*, unsigned int) = nullptr) {
    if (deviceCount >= MAX_DEVICES) return -1;
    
    int deviceIndex = deviceCount++;
    
    devices[deviceIndex].name = name;
    devices[deviceIndex].token = token;
    devices[deviceIndex].wifiClient = new WiFiClient();
    devices[deviceIndex].mqttClient = new PubSubClient(*devices[deviceIndex].wifiClient);
    devices[deviceIndex].connected = false;
    devices[deviceIndex].callback = callback;
    
    // Configure MQTT client
    devices[deviceIndex].mqttClient->setServer(THINGSBOARD_SERVER, THINGSBOARD_PORT);
    if (callback != nullptr) {
        devices[deviceIndex].mqttClient->setCallback(callback);
    }
    
    return deviceIndex;
}

void connectWifi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");
}

bool connectDeviceToThingsBoard(int deviceIndex) {
    if (deviceIndex < 0 || deviceIndex >= deviceCount) return false;
    
    Alldevice& device = devices[deviceIndex];
    String clientId = String(device.name) + "_Client";
    
    if (!device.mqttClient->connected()) {
        Serial.printf("Connecting %s to ThingsBoard... ", device.name);
        
        if (device.mqttClient->connect(clientId.c_str(), device.token, nullptr)) {
            device.connected = true;
            Serial.printf("Connected!\n");
            
            // Device-specific setup after connection
            if (deviceIndex == ledDeviceIndex) {
                // Subscribe to attributes for LED control
                device.mqttClient->subscribe("v1/devices/me/attributes");
                
                // Request initial attribute values
                String payload = "{\"shared\":[\"" + String(ledStateControlKey) + "\"]}";
                device.mqttClient->publish("v1/devices/me/attributes", payload.c_str());
                Serial.println("Sent request for LED state attribute");
            }
            
            return true;
        } else {
            Serial.printf("Failed, rc=%d. Retrying...\n", device.mqttClient->state());
            return false;
        }
    }
    
    return true;
}

// LED attribute callback handler
void ledAttributeCallback(char* topic, byte* payload, unsigned int length) {
    Serial.println("LED callback called");
    Serial.print("Topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
    
    if (strstr(topic, "attributes")) {
        Serial.println("Processing attributes message");
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, payload, length);
        
        if (doc.containsKey(ledStateControlKey)) {
            String ledStateStr = doc[ledStateControlKey].as<String>();
            Serial.print("LED state value: ");
            Serial.println(ledStateStr);
            
            bool newLedState = (ledStateStr == "ON");
            
            // Send to LED control task via queue
            if (xQueueSend(ledStateQueue, &newLedState, 0) != pdTRUE) {
                Serial.println("Failed to send LED state to queue");
            } else {
                Serial.printf("Sent LED state to task: %s\n", newLedState ? "ON" : "OFF");
            }
        } else {
            Serial.println("LED state attribute not found in message");
        }
    }
}

// WiFi management task
void wifiTask(void *pvParameters) {
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            connectWifi();
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Device connection management task
void deviceManagerTask(void *pvParameters) {
    for (;;) {
        for (int i = 0; i < deviceCount; i++) {
            if (!connectDeviceToThingsBoard(i)) {
                vTaskDelay(pdMS_TO_TICKS(5000));
            }
            
            if (devices[i].connected) {
                devices[i].mqttClient->loop();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// LED control task
void ledControlTask(void *pvParameters) {
    bool currentLedState = false;
    for (;;) {
        if (xQueueReceive(ledStateQueue, &currentLedState, portMAX_DELAY) == pdTRUE) {
            digitalWrite(LED_PIN, currentLedState ? HIGH : LOW);
            Serial.printf("Setting LED to: %s\n", currentLedState ? "ON" : "OFF");
        }
    }
}

// DHT sensor reading
void sensorTask(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    for (;;) {
        dht20.read();
        float temperature = dht20.getTemperature();
        float humidity = dht20.getHumidity();
        
        if (!isnan(humidity) && !isnan(temperature)) {
            Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
            
            if (dhtDeviceIndex >= 0 && devices[dhtDeviceIndex].connected) {
                // Publish sensor data
                String payload = "{\"temperature\":" + String(temperature) + 
                                ",\"humidity\":" + String(humidity) + "}";
                devices[dhtDeviceIndex].mqttClient->publish("v1/devices/me/telemetry", payload.c_str());
            }
        } else {
            Serial.println("Failed to read from DHT20 sensor!");
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    dht20.begin();
    
    ledStateQueue = xQueueCreate(1, sizeof(bool));
    
    ledDeviceIndex = addDevice("LED", "fiu7c7huy80k83o74mw0", ledAttributeCallback);
    //dhtDeviceIndex = addDevice("DHT20", "mae15of5vf8oc2v3bdap");
    
    Serial.printf("Added %d devices\n", deviceCount);
    
    xTaskCreate(wifiTask, "WiFi Task", 4096, NULL, 1, NULL);
    xTaskCreate(deviceManagerTask, "Device Manager", 8192, NULL, 2, NULL);
    xTaskCreate(ledControlTask, "LED Control", 2048, NULL, 3, NULL);
    xTaskCreate(sensorTask, "Sensor Task", 4096, NULL, 2, NULL);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}