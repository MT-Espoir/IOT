#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12
#define LED_PIN 48  // Add LED pin definition

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>

constexpr char WIFI_SSID[] = "Min";
constexpr char WIFI_PASSWORD[] = "123456789";

constexpr char TOKEN[] = "mae15of5vf8oc2v3bdap";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

// Shared attribute keys
constexpr char SCHEDULE_ATTR[] = "schedule";
constexpr char LED_STATE_ATTR[] = "ledState";

// Variables for scheduling
struct TimeSchedule {
  int onHour = 0;
  int onMinute = 0;
  int offHour = 0; 
  int offMinute = 0;
  bool enabled = false;
};

TimeSchedule deviceSchedule;
bool scheduleChanged = false;
bool ledState = false;

uint32_t previousStateChange;
uint32_t lastTimeCheck = 0;
constexpr uint32_t TIME_CHECK_INTERVAL = 60000; // Check time every minute

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;

float temperature = NAN;
float humidity = NAN;

// RPC callback to get current sensor data
RPC_Response requestSensorData(const RPC_Data &data) {
  Serial.println("Received request for current sensor data");
  
  dht20.read();
  float temp = dht20.getTemperature();
  float hum = dht20.getHumidity();
  
  // Create JSON response with current values
  StaticJsonDocument<200> respData;
  respData["temperature"] = temp;
  respData["humidity"] = hum;
  respData["rssi"] = WiFi.RSSI();
  
  String respStr;
  serializeJson(respData, respStr);
  
  return RPC_Response("sensorDataResponse", respStr);
}

// RPC callback to control LED
RPC_Response setLedState(const RPC_Data &data) {
  Serial.println("Received LED control command");
  
  // Get new LED state from RPC data
  ledState = data;
  
  // Apply the state to LED
  digitalWrite(LED_PIN, ledState);
  
  Serial.print("Set LED state to: ");
  Serial.println(ledState);
  
  // Send updated attribute to ThingsBoard
  tb.sendAttributeData(LED_STATE_ATTR, ledState);
  
  return RPC_Response("ledState", ledState);
}

// RPC callback to update schedule
RPC_Response updateSchedule(const RPC_Data &data) {
  Serial.println("Received schedule update");
  
  // Parse the schedule data from JSON
  JsonObject scheduleObj = data;
  
  deviceSchedule.onHour = scheduleObj["onHour"];
  deviceSchedule.onMinute = scheduleObj["onMinute"];
  deviceSchedule.offHour = scheduleObj["offHour"];
  deviceSchedule.offMinute = scheduleObj["offMinute"];
  deviceSchedule.enabled = scheduleObj["enabled"];
  
  scheduleChanged = true;
  
  Serial.print("Schedule updated - On: ");
  Serial.print(deviceSchedule.onHour);
  Serial.print(":");
  Serial.print(deviceSchedule.onMinute);
  Serial.print(", Off: ");
  Serial.print(deviceSchedule.offHour);
  Serial.print(":");
  Serial.print(deviceSchedule.offMinute);
  Serial.print(", Enabled: ");
  Serial.println(deviceSchedule.enabled ? "Yes" : "No");
  
  return RPC_Response("scheduleUpdated", true);
}

// Array of RPC callbacks
const std::array<RPC_Callback, 3U> callbacks = {
  RPC_Callback{ "requestSensorData", requestSensorData },
  RPC_Callback{ "setLedState", setLedState },
  RPC_Callback{ "updateSchedule", updateSchedule }
};

// Process shared attributes updates
void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), SCHEDULE_ATTR) == 0) {
      // Parse the schedule JSON
      JsonObject scheduleObj = it->value();
      
      deviceSchedule.onHour = scheduleObj["onHour"];
      deviceSchedule.onMinute = scheduleObj["onMinute"];
      deviceSchedule.offHour = scheduleObj["offHour"];
      deviceSchedule.offMinute = scheduleObj["offMinute"];
      deviceSchedule.enabled = scheduleObj["enabled"];
      
      Serial.print("Received schedule - On: ");
      Serial.print(deviceSchedule.onHour);
      Serial.print(":");
      Serial.print(deviceSchedule.onMinute);
      Serial.print(", Off: ");
      Serial.print(deviceSchedule.offHour);
      Serial.print(":");
      Serial.print(deviceSchedule.offMinute);
      Serial.print(", Enabled: ");
      Serial.println(deviceSchedule.enabled ? "Yes" : "No");
    } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);
      
      Serial.print("LED state updated to: ");
      Serial.println(ledState);
    }
  }
}

// Define shared attributes to subscribe to
const std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  SCHEDULE_ATTR,
  LED_STATE_ATTR
};

// Create callback for processing shared attributes
const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

// Create attribute request callback
const Attribute_Request_Callback attribute_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

const bool reconnect() {
  // Check to ensure we aren't connected yet
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }
  // If we aren't establish a new connection to the given WiFi network
  InitWiFi();
  return true;
}

// Function to check and apply schedule
void checkSchedule() {
  if (!deviceSchedule.enabled) {
    return; // Schedule is disabled
  }
  
  // Get current time from NTP (in real implementation)
  // For this demo, we'll just use millis to cycle through a day quickly for testing
  unsigned long currentMillis = millis();
  unsigned long dayProgress = (currentMillis / 1000) % 86400; // Seconds in a day
  
  int currentHour = dayProgress / 3600;
  int currentMinute = (dayProgress % 3600) / 60;
  
  // Check if we should turn ON
  if (currentHour == deviceSchedule.onHour && currentMinute == deviceSchedule.onMinute) {
    if (!ledState) {
      ledState = true;
      digitalWrite(LED_PIN, ledState);
      tb.sendAttributeData(LED_STATE_ATTR, ledState);
      Serial.println("Schedule: Turning LED ON");
    }
  }
  
  // Check if we should turn OFF
  if (currentHour == deviceSchedule.offHour && currentMinute == deviceSchedule.offMinute) {
    if (ledState) {
      ledState = false;
      digitalWrite(LED_PIN, ledState);
      tb.sendAttributeData(LED_STATE_ATTR, ledState);
      Serial.println("Schedule: Turning LED OFF");
    }
  }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(LED_PIN, OUTPUT);
  
  delay(1000);
  Serial.println("====== DEVICE STARTING ======");
  Serial.println("Initializing WiFi...");
  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
  
  Serial.println("Device initialization complete");
}

void loop() {
  delay(10);
  
  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    Serial.print("Connecting to ThingsBoard: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect to ThingsBoard");
      return;
    }
    
    // Subscribe to RPC commands
    if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }
    
    // Subscribe to shared attributes
    if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
      Serial.println("Failed to subscribe for shared attributes");
      return;
    }
    
    // Request shared attributes values
    if (!tb.Shared_Attributes_Request(attribute_request_callback)) {
      Serial.println("Failed to request shared attributes");
      return;
    }
    
    Serial.println("ThingsBoard subscriptions successful");
  }
  
  // Check schedule based on time
  if (millis() - lastTimeCheck > TIME_CHECK_INTERVAL) {
    lastTimeCheck = millis();
    checkSchedule();
  }

  // Send telemetry data periodically
  if (millis() - previousDataSend > telemetrySendInterval) {
    previousDataSend = millis();

    dht20.read();
    
    float temperature = dht20.getTemperature();
    float humidity = dht20.getHumidity();

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT20 sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");

      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    }

    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());
    
    // Also send current schedule status
    StaticJsonDocument<200> scheduleStatus;
    scheduleStatus["onHour"] = deviceSchedule.onHour;
    scheduleStatus["onMinute"] = deviceSchedule.onMinute;
    scheduleStatus["offHour"] = deviceSchedule.offHour;
    scheduleStatus["offMinute"] = deviceSchedule.offMinute;
    scheduleStatus["enabled"] = deviceSchedule.enabled;
    
    tb.sendAttributeData(SCHEDULE_ATTR, scheduleStatus);
  }

  tb.loop();
}
