#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

#include <ArduinoHttpClient.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "DHT20.h"
#include <Update.h>
#include <vector>

// Thông tin kết nối
constexpr char WIFI_SSID[] = "TRUC ANH"; // Thay bằng SSID của bạn
constexpr char WIFI_PASSWORD[] = "23230903"; // Thay bằng mật khẩu WiFi của bạn
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

// Biến OTA toàn cục
bool otaInProgress = false;
bool waitingForChunk = false;
int fw_size = 0, chunk_size = 0, chunks_received = 0, offset = 0;
String fw_title = "", fw_version = "", fw_checksum = "", fw_algo = "sha256";

// Timeout cho OTA
unsigned long lastRequestTime = 0;
const unsigned long REQUEST_TIMEOUT = 5000; // 5 giây

DHT20 dht20;

#define MAX_DEVICES 10

// Cấu trúc thiết bị
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
int dhtDeviceIndex = -1;

// Hàm giải mã Base64
int b64decode(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

size_t decode_base64(const char *input, uint8_t *output, size_t output_len) {
    size_t i = 0;
    int buffer = 0, bits = 0;
    while (*input && output_len) {
        int val = b64decode(*input++);
        if (val < 0) continue;
        buffer = (buffer << 6) | val;
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            *output++ = (buffer >> bits) & 0xFF;
            output_len--;
            i++;
        }
    }
    return i;
}

// Hàm callback MQTT
void deviceCallback(char* topic, byte* payload, unsigned int length) {
    Serial.printf("MQTT -> %s\n", topic);

    DynamicJsonDocument doc(2048);
    DeserializationError error = deserializeJson(doc, payload, length);
    if (error) {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
    }

    String topicStr = String(topic);

    // Xử lý thông tin firmware
    if (topicStr.indexOf("attributes") >= 0) {
        if (doc.containsKey("fw_title")) {
            fw_title = doc["fw_title"].as<String>();
            fw_version = doc["fw_version"].as<String>();
            fw_checksum = doc["fw_checksum"].as<String>();
            fw_algo = doc["fw_checksum_algorithm"].as<String>();
            fw_size = doc["fw_size"];
            chunk_size = doc["fw_chunk_size"];
            offset = 0;
            chunks_received = 0;

            otaInProgress = true;
            devices[dhtDeviceIndex].mqttClient->publish("v1/devices/me/attributes", "{\"fw_state\":\"INITIATED\"}");
        }
    }

    // Xử lý dữ liệu chunk firmware
    if (topicStr.indexOf("firmware/response") >= 0 && otaInProgress && waitingForChunk) {
        const char* b64data = doc["data"];
        int len = strlen(b64data);
        std::vector<uint8_t> decoded(len / 4 * 3);

        size_t actualLen = decode_base64(b64data, decoded.data(), decoded.size());
        if (Update.write(decoded.data(), actualLen) != actualLen) {
            Serial.println("Update write failed!");
            devices[dhtDeviceIndex].mqttClient->publish("v1/devices/me/attributes", "{\"fw_state\":\"FAILED\"}");
            otaInProgress = false;
            Update.abort();
            return;
        }

        offset += actualLen;
        chunks_received++;
        Serial.printf("Chunk %d received. Total offset: %d/%d\n", chunks_received, offset, fw_size);

        waitingForChunk = false;
        lastRequestTime = 0;

        if (offset >= fw_size) {
            if (Update.end(true)) {
                Serial.println("OTA Update Success!");
                devices[dhtDeviceIndex].mqttClient->publish("v1/devices/me/attributes", "{\"fw_state\":\"UPDATED\"}");
                vTaskDelay(pdMS_TO_TICKS(2000));
                ESP.restart();
            } else {
                Serial.println("Update end failed.");
                devices[dhtDeviceIndex].mqttClient->publish("v1/devices/me/attributes", "{\"fw_state\":\"FAILED\"}");
                otaInProgress = false;
            }
        }
    }
}

// Kết nối WiFi
void connectWifi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");
}

// Thêm thiết bị
int addDevice(const char* name, const char* token, void (*callback)(char*, byte*, unsigned int) = nullptr) {
    if (deviceCount >= MAX_DEVICES) return -1;
    int deviceIndex = deviceCount++;
    devices[deviceIndex].name = name;
    devices[deviceIndex].token = token;
    devices[deviceIndex].wifiClient = new WiFiClient();
    devices[deviceIndex].mqttClient = new PubSubClient(*devices[deviceIndex].wifiClient);
    devices[deviceIndex].connected = false;
    devices[deviceIndex].callback = callback;
    devices[deviceIndex].mqttClient->setServer(THINGSBOARD_SERVER, THINGSBOARD_PORT);
    devices[deviceIndex].mqttClient->setBufferSize(16384);
    if (callback != nullptr) {
        devices[deviceIndex].mqttClient->setCallback(callback);
    }
    return deviceIndex;
}

// Kết nối thiết bị với ThingsBoard
bool connectDeviceToThingsBoard(int deviceIndex) {
    if (deviceIndex < 0 || deviceIndex >= deviceCount) return false;
    Alldevice& device = devices[deviceIndex];
    String clientId = String(device.name) + "_Client";
    if (!device.mqttClient->connected()) {
        if (device.mqttClient->connect(clientId.c_str(), device.token, nullptr)) {
            device.connected = true;
            Serial.printf("Connected!\n");
            if (deviceIndex == dhtDeviceIndex) {
                device.mqttClient->subscribe("v1/devices/me/attributes");
                device.mqttClient->subscribe("v1/devices/me/firmware/response");
            }
            return true;
        } else {
            Serial.printf("Failed, rc=%d\n", device.mqttClient->state());
            return false;
        }
    }
    return true;
}

// Task quản lý WiFi
void wifiTask(void *pvParameters) {
    for (;;) {
        if (WiFi.status() != WL_CONNECTED) {
            connectWifi();
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// Task quản lý thiết bị
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

// Task OTA
void otaTask(void *pvParameters) {
    for (;;) {
        if (otaInProgress && !waitingForChunk && offset < fw_size) {
            if (!Update.begin(fw_size)) {
                Serial.println("Failed to begin OTA");
                devices[dhtDeviceIndex].mqttClient->publish("v1/devices/me/attributes", "{\"fw_state\":\"FAILED\"}");
                otaInProgress = false;
                continue;
            }

            String req = "{\"title\":\"" + fw_title + "\",\"version\":\"" + fw_version + "\",\"chunkSize\":" + String(chunk_size) + ",\"chunk\":\"" + String(offset) + "\"}";
            devices[dhtDeviceIndex].mqttClient->publish("v1/devices/me/firmware/request", req.c_str());
            waitingForChunk = true;
            lastRequestTime = millis();
            Serial.printf("Requested chunk at offset %d\n", offset);
        }

        if (waitingForChunk && millis() - lastRequestTime > REQUEST_TIMEOUT) {
            Serial.println("Timeout waiting for chunk. Retrying...");
            waitingForChunk = false;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Task đọc cảm biến
void sensorTask(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    for (;;) {
        dht20.read();
        float temperature = dht20.getTemperature();
        float humidity = dht20.getHumidity();
        if (!isnan(humidity) && !isnan(temperature)) {
            Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n", temperature, humidity);
            if (dhtDeviceIndex >= 0 && devices[dhtDeviceIndex].connected) {
                String payload = "{\"temperature\":" + String(temperature) + ",\"humidity\":" + String(humidity) + "}";
                devices[dhtDeviceIndex].mqttClient->publish("v1/devices/me/telemetry", payload.c_str());
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Hello 1");
    Wire.begin(SDA_PIN, SCL_PIN);
    dht20.begin();
    dhtDeviceIndex = addDevice("DHT20", "s958tymnfdgw3xmiyeo8", deviceCallback);
    Serial.printf("Added %d devices\n", deviceCount);
    xTaskCreate(wifiTask, "WiFi Task", 4096, NULL, 1, NULL);
    xTaskCreate(deviceManagerTask, "Device Manager", 8192, NULL, 2, NULL);
    xTaskCreate(sensorTask, "Sensor Task", 4096, NULL, 2, NULL);
    xTaskCreate(otaTask, "OTA Task", 8192, NULL, 2, NULL);
}

void loop() {
    vTaskDelay(portMAX_DELAY); 
}