#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define LED_PIN 48  // Add LED pin definition

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

constexpr char TOKEN[] = "fiu7c7huy80k83o74mw0"; //LED
constexpr char TOKEN2[] = "mae15of5vf8oc2v3bdap"; //DHT20

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

const char* ledStateControlKey = "ledState"; // Sử dụng ledState làm key
volatile bool ledState = false;
QueueHandle_t ledStateQueue;

WiFiClient wifiClient;
PubSubClient client(wifiClient);
WiFiClient sensorWifiClient;
PubSubClient sensorClient(sensorWifiClient);

DHT20 dht20;

float temperature = NAN;
float humidity = NAN;

// Hàm để kết nối WiFi
void connectWifi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  Serial.println("Connected to WiFi");
}

// Hàm để kết nối ThingsBoard
void connectThingsBoard() {
  while (!client.connect("ESP32Client", TOKEN, nullptr)) {
      Serial.print("Failed to connect to ThingsBoard, rc=");
      Serial.println(client.state());
      delay(5000);
  }
  Serial.println("Connected to ThingsBoard");
  // Đăng ký nhận thông tin shared attributes
  client.subscribe("v1/devices/me/attributes");
  // Yêu cầu giá trị ban đầu của các shared attributes
  String payload = "{\"shared\":[\"" + String(ledStateControlKey) + "\"]}";
  client.publish("v1/devices/me/attributes", payload.c_str());
  Serial.println("Sent request for shared attributes.");
}

void connectSensorToThingsBoard() {
  while (!sensorClient.connect("ESP32SensorClient", TOKEN2, nullptr)) {
    Serial.print("Failed to connect DHT20 to ThingsBoard, rc=");
    Serial.println(sensorClient.state());
    delay(5000);
  }
  Serial.println("DHT20 Connected to ThingsBoard");
}

// Callback function khi nhận được tin nhắn MQTT
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Callback function called in MQTT Task.");
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
  }
  Serial.println();

  // Xử lý phản hồi shared attributes
  if (strstr(topic, "attributes")) {
      Serial.println("Processing shared attributes response...");
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, payload, length);

      if (doc.containsKey("ledState")) {
          String ledStateStr = doc["ledState"].as<String>();
          Serial.print("ledState value from TB: ");
          Serial.println(ledStateStr);

          bool newLedState = false;
          if (ledStateStr == "ON") {
              newLedState = true;
              Serial.println("ledState is now TRUE");
          } else {
              newLedState = false;
              Serial.println("ledState is now FALSE");
          }

          // Send the new LED state to the LED control task via the queue
          if (xQueueSend(ledStateQueue, &newLedState, 0) != pdTRUE) {
              Serial.println("Failed to send LED state to queue.");
          }
          Serial.print("Sent ledState to LED Control Task: ");
          Serial.println(newLedState ? "ON" : "OFF");
      } else {
          Serial.println("Attribute 'ledState' not found in response.");
      }
  }
}

// RPC callback to get current sensor data
void wifiTask(void *pvParameters) {
  for (;;) {
      if (WiFi.status() != WL_CONNECTED) {
          connectWifi();
      }
      vTaskDelay(pdMS_TO_TICKS(5000)); // Check WiFi status every 5 seconds
  }
}

// Task to handle MQTT connection and communication with ThingsBoard
void mqttTask(void *pvParameters) {
  // Set up both clients
  client.setServer(THINGSBOARD_SERVER, THINGSBOARD_PORT);
  client.setCallback(callback);
  sensorClient.setServer(THINGSBOARD_SERVER, THINGSBOARD_PORT);
  
  for (;;) {
    // Handle LED client
    if (!client.connected()) {
      connectThingsBoard();
    }
    client.loop();
    
    // Handle sensor client
    if (!sensorClient.connected()) {
      connectSensorToThingsBoard();
    }
    sensorClient.loop();
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task to control the LED based on the ledState variable
void ledControlTask(void *pvParameters) {
  bool currentLedState = false;
  for (;;) {
      if (xQueueReceive(ledStateQueue, &currentLedState, portMAX_DELAY) == pdTRUE) {
          digitalWrite(LED_PIN, currentLedState ? HIGH : LOW);
          Serial.print("Setting LED to: ");
          Serial.println(currentLedState ? "ON" : "OFF");
      }
  }
}

void sensorTask(void *pvParameters) {
  vTaskDelay(pdMS_TO_TICKS(5000));

  for (;;) {
    dht20.read();
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();
    if (!isnan(humidity) && !isnan(temperature)) {
      Serial.printf("Nhiệt độ: %.2f°C, Độ ẩm: %.2f%%\n", temperature, humidity);
      
      // Use the sensor client to publish data
      String payload = "{\"temperature\":" + String(temperature) + 
                       ",\"humidity\":" + String(humidity) + "}";
      sensorClient.publish("v1/devices/me/telemetry", payload.c_str());
    } else {
      Serial.println("Lỗi đọc cảm biến DHT20!");
    }
    vTaskDelay(pdMS_TO_TICKS(30000));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Đảm bảo ban đầu đèn tắt
  dht20.begin();
  
  // Create a queue to pass LED state updates between tasks
  ledStateQueue = xQueueCreate(1, sizeof(bool)); 

  // Create tasks
  xTaskCreate(wifiTask, "WiFi Task", 4096, NULL, 1, NULL); 
  xTaskCreate(mqttTask, "MQTT Task", 8192, NULL, 2, NULL); 
  xTaskCreate(ledControlTask, "LED Control Task", 2048, NULL, 3, NULL); 
  xTaskCreate(sensorTask, "Sensor Task", 4096, NULL, 2, NULL);

}

void loop() {
  // Loop function is empty in FreeRTOS as tasks are running independently
  vTaskDelay(portMAX_DELAY); // Prevent the main loop from exiting
}