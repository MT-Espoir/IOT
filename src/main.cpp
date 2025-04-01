#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

#include <ArduinoHttpClient.h>
#include <WiFi.h>
#include <HTTPUpdate.h>
#include <Update.h>
#include <HTTPClient.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>


constexpr char FIRMWARE_VERSION[] = "v1.0.0";
constexpr char FIRMWARE_UPDATE_URL[] = "http://your-update-server.com/firmware/";

constexpr char WIFI_SSID[] = "Min";
constexpr char WIFI_PASSWORD[] = "123456789";

constexpr char TOKEN[] = "mae15of5vf8oc2v3bdap";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

constexpr uint32_t CHECK_UPDATE_INTERVAL = 60000U; // Check for updates every 60 seconds
uint32_t lastUpdateCheck = 0;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;

float temperature = NAN;
float humidity = NAN;

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


// Add this function before setup()
void checkForUpdates() {
  Serial.println("Checking for firmware updates...");
  
  WiFiClient client;
  HTTPClient http;
  
  // Construct URL (firmware.bin will be the new firmware file)
  String url = String(FIRMWARE_UPDATE_URL) + "firmware.bin";
  Serial.println("Checking: " + url);
  
  http.begin(client, url);
  
  // Add version header so server can decide if update is needed
  http.addHeader("x-ESP32-version", FIRMWARE_VERSION);
  
  int httpCode = http.GET();
  if (httpCode == 200) {
    // If update is available, apply it
    Serial.println("Update found, starting update process...");
    
    // Start update process
    t_httpUpdate_return ret = httpUpdate.update(client, url);
    
    // Handle update result
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP update failed: (%d): %s\r\n", 
                      httpUpdate.getLastError(),
                      httpUpdate.getLastErrorString().c_str());
        break;
        
      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("No updates available");
        break;
        
      case HTTP_UPDATE_OK:
        Serial.println("Update successful, restarting...");
        // Device will restart automatically after successful update
        break;
    }
  } else {
    Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  Serial.println("Initializing WiFi...");
  InitWiFi();

  // Initialize OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Complete");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Authentication Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connection Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  Serial.println("OTA Ready");

  // Initialize I2C and sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
}

void loop() {
  ArduinoOTA.handle(); // Check for local network OTA updates
  
  delay(10);
  
  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
  }  
  
  // Check for HTTP updates periodically
  if (millis() - lastUpdateCheck > CHECK_UPDATE_INTERVAL) {
    lastUpdateCheck = millis();
    checkForUpdates();
  }
  
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
  }

  tb.loop();
}
  // TODO LIST:
  // 1.  Implement OTA Update:
  //    1.1 Configure the ESP32 to check for firmware updates over HTTP or MQTT.
  //    1.2 Deploy an update server and host a new firmware version.
  // 2. Test the Update Process:
  //    2.1 Trigger an OTA update and monitor the process.
  //    2.2 Verify that the new firmware is running correctly after the update
  // 3 Firmware Upload Test: Upload a new firmware version to the server and trigger an
  //    3.1 OTA update.
  //    3.2 Data Integrity Check: Verify that temperature and humidity data continue to be sent correctly after an update.
  //    3.3 Implement an OTA update mechanism using either HTTP or MQTT.
  //    3.4 Upload a new firmware version and trigger the update remotely