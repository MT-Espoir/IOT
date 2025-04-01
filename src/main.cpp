#define SDA_PIN GPIO_NUM_11
#define SCL_PIN GPIO_NUM_12

#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <Update.h>

#define HOSTNAME "ESP32-OTA"
#define OTA_PASSWORD "otapassword"

constexpr char WIFI_SSID[] = "Min";
constexpr char WIFI_PASSWORD[] = "123456789";

constexpr char TOKEN[] = "mae15of5vf8oc2v3bdap";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr char OTA_SERVER[] = "http://192.168.1.100:8000"; // Replace with your server IP
constexpr char FIRMWARE_VERSION[] = "1.0.0"; // Current firmware version
constexpr uint32_t OTA_CHECK_INTERVAL = 60000; // Check for updates every minute (for testing)
uint32_t lastOTACheck = 0;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;

float temperature = NAN;
float humidity = NAN;

void checkForUpdates() {
  Serial.println("Checking for firmware updates...");
  
  // Fix: Use HTTPClient properly
  HTTPClient http;
  String versionUrl = String(OTA_SERVER) + "/version.txt";
  http.begin(versionUrl);
  
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    String newVersion = http.getString();
    newVersion.trim();
    
    Serial.print("Current version: ");
    Serial.println(FIRMWARE_VERSION);
    Serial.print("Available version: ");
    Serial.println(newVersion);
    
    // If versions are different, update
    if (String(FIRMWARE_VERSION) != newVersion && newVersion.length() > 0) {
      Serial.println("New version available. Starting update...");
      
      // Report update status to ThingsBoard
      tb.sendAttributeData("firmwareUpdateStatus", "updating");
      tb.sendAttributeData("newFirmwareVersion", newVersion.c_str());
      
      // Start the update
      String firmwareUrl = String(OTA_SERVER) + "/firmware.bin";
      http.end();
      
      http.begin(firmwareUrl);
      httpCode = http.GET();
      
      if (httpCode == HTTP_CODE_OK) {
        // Get the update
        int contentLength = http.getSize();
        
        if (contentLength > 0) {
          Serial.println("Starting HTTP update...");
          WiFiClient *client = http.getStreamPtr();
          
          t_httpUpdate_return ret = httpUpdate.update(*client, firmwareUrl);
          
          switch (ret) {
            case HTTP_UPDATE_FAILED:
              Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", 
                httpUpdate.getLastError(), 
                httpUpdate.getLastErrorString().c_str());
              tb.sendAttributeData("firmwareUpdateStatus", "failed");
              break;
              
            case HTTP_UPDATE_NO_UPDATES:
              Serial.println("HTTP_UPDATE_NO_UPDATES");
              tb.sendAttributeData("firmwareUpdateStatus", "no_update");
              break;
              
            case HTTP_UPDATE_OK:
              Serial.println("HTTP_UPDATE_OK");
              tb.sendAttributeData("firmwareUpdateStatus", "success");
              delay(1000);
              ESP.restart(); // Restart after update
              break;
          }
        }
      } else {
        Serial.print("Failed to download firmware: ");
        Serial.println(httpCode);
        tb.sendAttributeData("firmwareUpdateStatus", "download_failed");
      }
    } else {
      Serial.println("Already on latest version");
      tb.sendAttributeData("firmwareUpdateStatus", "up_to_date");
    }
  } else {
    Serial.print("Failed to check version: ");
    Serial.println(httpCode);
    tb.sendAttributeData("firmwareUpdateStatus", "check_failed");
  }
  
  http.end();
}

// Process shared attributes from ThingsBoard
void processSharedAttributes(const Shared_Attribute_Data &data) {
  Serial.println("Received shared attribute update");
  
  // Fix: Use containsKey instead of contains
  if (data.containsKey("firmwareUpdate")) {
    bool shouldUpdate = data["firmwareUpdate"];
    if (shouldUpdate) {
      Serial.println("Firmware update triggered remotely");
      checkForUpdates();
    }
  }
}


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

void setupOTA() {
  // Set hostname for easier discovery
  ArduinoOTA.setHostname(HOSTNAME);
  
  // Set password for OTA updates
  ArduinoOTA.setPassword(OTA_PASSWORD);

  // OTA callbacks
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  // Enable mDNS discovery
  if (MDNS.begin(HOSTNAME)) {
    Serial.println("mDNS responder started");
    // Add service to mDNS
    MDNS.addService("arduino", "tcp", 3232);
  } else {
    Serial.println("Error setting up mDNS responder");
  }
  
  ArduinoOTA.begin();
  Serial.println("OTA initialized");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  Serial.println("Initializing WiFi...");
  InitWiFi();

  Wire.begin(SDA_PIN, SCL_PIN);
  dht20.begin();
  
  // Configure OTA
  setupOTA();
  
  // Send initial firmware version to ThingsBoard
  if (tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
    tb.sendAttributeData("firmwareVersion", FIRMWARE_VERSION);
    Serial.println("Connected to ThingsBoard and sent firmware version");
    
    // Fix: Use correct method name
    if (!tb.Shared_Attributes_Subscribe(processSharedAttributes)) {
      Serial.println("Failed to subscribe to attributes");
    } else {
      Serial.println("Subscribed to shared attributes");
    }
  }
}

void loop() {
  // Handle ArduinoOTA
  ArduinoOTA.handle();
  
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
    } else {
      // After connecting, subscribe to shared attributes
      Serial.println("Subscribing to shared attributes...");
      // Fix: Use correct method name
      if (!tb.Shared_Attributes_Subscribe(processSharedAttributes)) {
        Serial.println("Failed to subscribe to attributes");
      }
    }
  }
  
  // Check for updates periodically
  if (millis() - lastOTACheck > OTA_CHECK_INTERVAL) {
    lastOTACheck = millis();
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
    
    // Also regularly send firmware version
    tb.sendAttributeData("firmwareVersion", FIRMWARE_VERSION);
  }

  tb.loop();
}

  // TODO LIST update:
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