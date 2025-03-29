#define USE_SHT

#include <WiFi.h>
#include <MQTT.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#ifdef USE_SCD30
#include <SensirionI2cScd30.h>
SensirionI2cScd30 scd30;
#endif
#ifdef USE_DHT
#include <DHT.h>
// Define DHT sensor settings
#define DHTPIN 2  // Pin where the DHT sensor is connected
#define DHTTYPE DHT22
DHT sensor(DHTPIN, DHTTYPE);
#endif
#ifdef USE_BME
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
// Define BME280 sensor settings
#define BME280_I2C_ADDR 0x76  // Default I2C address for BME280
Adafruit_BME280 sensor;
#endif
#ifdef USE_SHT
#include <SensirionI2cSht4x.h>
SensirionI2cSht4x sht45;
#endif

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Define pins for relay modules
#define HEATER_RELAY_PIN 23
#define FAN_RELAY_PIN 25
#define HUMIDIFIER_RELAY_PIN 26

#define HUMIDITY_UP_PIN 32    // GPIO32 - unused pin
#define HUMIDITY_DOWN_PIN 33  // GPIO33 - unused pin
const float HUMIDITY_STEP = 1.0; // Change humidity by 1% increments
#define DEBOUNCE_DELAY 50

const uint8_t MAX_RETRIES = 10;
const char ssid[] = "eternity";
const char pass[] = "OpenSesame3";
const char token[] = "x6BJrHUyCEwZwKb0oeax";
const char broker[] = "dash.gugl.org";
const int port = 1883;

WiFiClient net;
MQTTClient client;

// Timing variables
unsigned long previousMillis = 0;
unsigned long previousFanMillis = 0;        // Tracks last time the fan was turned on
unsigned long fanStartMillis = 0;           // Tracks the time when fan is turned on
const unsigned long updateInterval = 2000;  // 2-second interval for main loop logic

// Create preferences object
Preferences preferences;

// Default values for settings
const float DEFAULT_DESIRED_HUMIDITY = 90.0f;
const float DEFAULT_HUMIDITY_THRESHOLD = 4.0f;
const float DEFAULT_DESIRED_TEMPERATURE = 23.0f;
const float DEFAULT_TEMPERATURE_THRESHOLD = 2.0f;
const unsigned long DEFAULT_FAN_DURATION = 30000;  // 30 seconds in milliseconds
const unsigned long DEFAULT_FAN_INTERVAL = 900000; // 15 minutes in milliseconds

// Settings variables
float desiredHumidity;     // Desired humidity percentage
float humidityThreshold;   // Tolerance range for humidity
float desiredTemperature;  // Target temperature
float temperatureThreshold;
unsigned long fanOnDuration;
unsigned long fanInterval;  // Add new setting variable

bool isFanOn = false;
bool isPumpOn = false;
bool isHeaterOn = false;

// Define larger icons (16x16 pixels)
const unsigned char PROGMEM fanIcon[] = {
    0x00, 0x00, 0x07, 0xE0, 0x0F, 0xF0, 0x0F, 0xF8,
    0x0F, 0xF0, 0x07, 0xE0, 0x00, 0x00, 0x71, 0xDC,
    0x7B, 0xDE, 0xFD, 0xDF, 0x7C, 0x3E, 0x7F, 0x7E,
    0x7E, 0x7E, 0x3C, 0x7C, 0x18, 0x38, 0x00, 0x00
};

const unsigned char PROGMEM heatIcon[] = {
    0x00, 0x00, 0x00, 0x00, 0x08, 0x90, 0x19, 0x10,
    0x1B, 0x30, 0x1B, 0x30, 0x1B, 0xB0, 0x1B, 0xB8,
    0x1D, 0xD8, 0x0D, 0xD8, 0x0C, 0xD8, 0x0C, 0xD8,
    0x08, 0x98, 0x09, 0x10, 0x00, 0x00, 0x00, 0x00
};

const unsigned char PROGMEM waterIcon[] = {
    0x00, 0x80, 0x01, 0x80, 0x03, 0xC0, 0x07, 0xE0,
    0x07, 0xE0, 0x0F, 0xF0, 0x0F, 0xF8, 0x1F, 0xF8,
    0x1F, 0xF8, 0x3F, 0xFC, 0x3F, 0xFC, 0x3F, 0xFC,
    0x1F, 0xF8, 0x1F, 0xF8, 0x0F, 0xF0, 0x03, 0xC0
};

#ifdef USE_SCD30
void initScd30 {
  Wire.begin();
  scd30.begin(Wire, SCD30_I2C_ADDR_61);
  scd30.stopPeriodicMeasurement();
  scd30.softReset();
  int16_t error = sensor.startPeriodicMeasurement(0);
  if (error != NO_ERROR) {
    char errorMessage[128];
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, sizeof(errorMessage));
    Serial.println(errorMessage);
  }
}
#endif

// Function to initialize the OLED display
void initDisplay() {
  // Initialize OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    return;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

// Function to draw status icons
void drawStatusIcons() {
  if (isFanOn) {
    display.drawBitmap(0, 48, fanIcon, 16, 16, SSD1306_WHITE);
  }
  if (isHeaterOn) {
    display.drawBitmap(56, 48, heatIcon, 16, 16, SSD1306_WHITE);
  }
  if (isPumpOn) {
    display.drawBitmap(112, 48, waterIcon, 16, 16, SSD1306_WHITE);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Serial started");

  // Load settings from NVS
  loadSettings();

#ifdef USE_DHT
  // Initialize the DHT sensor
  sensor.begin();
#endif

#ifdef USE_BME
  // Initialize the BME280 sensor
  if (!sensor.begin(BME280_I2C_ADDR)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
#endif

#ifdef USE_SHT
  Wire.begin();
  sht45.begin(Wire, SHT45_I2C_ADDR_44);
  sht45.softReset();
#endif

#ifdef USE_SCD30
  initScd30();
#endif

  // Initialize the OLED display
  initDisplay();

  // Set relay pins as output
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(HEATER_RELAY_PIN, OUTPUT);
  pinMode(HUMIDIFIER_RELAY_PIN, OUTPUT);

  pinMode(HUMIDITY_DOWN_PIN, INPUT_PULLUP);
  pinMode(HUMIDITY_UP_PIN, INPUT_PULLUP);

  // Turn off the relays initially
  digitalWrite(FAN_RELAY_PIN, HIGH);
  digitalWrite(HEATER_RELAY_PIN, HIGH);
  digitalWrite(HUMIDIFIER_RELAY_PIN, HIGH);

  // Configure the Task Watchdog Timer (TWDT)
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = 5000,            // Set the timeout to 5000 ms (5 seconds)
    .idle_core_mask = (1ULL << portNUM_PROCESSORS) - 1,  // Monitor all cores
    .trigger_panic = false         // Do not trigger a panic; reset the system instead
  };

  // Initialize the watchdog timer with the configuration
  if (esp_task_wdt_init(&wdtConfig) == ESP_OK) {
    esp_task_wdt_add(nullptr);
  } else {
    Serial.println("failed to initialize watchdog");
  }
}

// Helper method to publish telemetry data
void publishTelemetry(const char* payload) {
  if (client.connected()) {
    Serial.print("Sending payload: ");
    Serial.println(payload);
    client.publish("v1/devices/me/telemetry", payload, true, 1);
  }
}

// Helper method to publish device state
void publishDeviceState(const char* device, bool state) {
  char payload[32];
  snprintf(payload, sizeof(payload), "{\"%s_on\":%d}", device, state ? 1 : 0);
  publishTelemetry(payload);
}

void toggleFan(bool on) {
  digitalWrite(FAN_RELAY_PIN, on ? LOW : HIGH);
  isFanOn = on;
  Serial.print("Fan ");
  Serial.println(on ? "ON" : "OFF");
  publishDeviceState("fan", on);
}

void togglePump(bool on) {
  digitalWrite(HUMIDIFIER_RELAY_PIN, on ? LOW : HIGH);
  isPumpOn = on;
  Serial.print("Humidifier ");
  Serial.println(on ? "ON" : "OFF");
  publishDeviceState("humidifier", on);
}

void toggleHeater(bool on) {
  digitalWrite(HEATER_RELAY_PIN, on ? LOW : HIGH);
  isHeaterOn = on;
  Serial.print("Heater ");
  Serial.println(on ? "ON" : "OFF");
  publishDeviceState("heater", on);
}

void updateDisplay(float temperature, float humidity) {
  display.clearDisplay();

  // Display temperature with desired value
  display.setTextSize(1);  // Smaller font size
  display.setCursor(0, 0);
  display.print(temperature, 1);
  display.print("C");
  display.print(" (");
  display.print(desiredTemperature, 1);
  display.print("C)");

  // Display humidity with desired value
  display.setCursor(0, 12);  // Adjusted Y position for smaller font
  display.print(humidity, 1);
  display.print("%");
  display.print(" (");
  display.print(desiredHumidity, 1);
  display.print("%)");

  // Draw status icons at the bottom
  drawStatusIcons();

  display.display();

  // Also print to Serial for debugging
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%\tTemperature: ");
  Serial.print(temperature);
  Serial.println("C");
}

void handleButton() {
  static uint8_t lastUpState = HIGH, lastDownState = HIGH;
  static unsigned long lastUpDebounce = 0, lastDownDebounce = 0;
  unsigned long currentMillis = millis();

  // Read current button states
  int currentUpState = digitalRead(HUMIDITY_UP_PIN);
  int currentDownState = digitalRead(HUMIDITY_DOWN_PIN);

  // Handle UP button
  if (currentUpState != lastUpState) {
    lastUpDebounce = currentMillis;
  }

  if ((currentMillis - lastUpDebounce) > DEBOUNCE_DELAY) {
    if (currentUpState == LOW) {  // Button is pressed (remember it's active LOW)
      adjustHumidity(HUMIDITY_STEP);
      // Wait for button release to prevent multiple triggers
      while (digitalRead(HUMIDITY_UP_PIN) == LOW) {
        delay(10);
        yield();
      }
    }
  }

  // Handle DOWN button
  if (currentDownState != lastDownState) {
    lastDownDebounce = currentMillis;
  }

  if ((currentMillis - lastDownDebounce) > DEBOUNCE_DELAY) {
    if (currentDownState == LOW) {  // Button is pressed
      adjustHumidity(-HUMIDITY_STEP);
      // Wait for button release to prevent multiple triggers
      while (digitalRead(HUMIDITY_DOWN_PIN) == LOW) {
        delay(10);
        yield();
      }
    }
  }

  lastUpState = currentUpState;
  lastDownState = currentDownState;
}

void adjustHumidity(float adjustment) {
  Serial.println(adjustment > 0 ? "Up button pressed" : "Down button pressed");
  float newHumidity = desiredHumidity + adjustment;

  if (validateSettings(newHumidity, humidityThreshold,
                      desiredTemperature, temperatureThreshold,
                      fanOnDuration, fanInterval)) {
    desiredHumidity = newHumidity;
    saveSettings();
    Serial.print("Desired humidity ");
    Serial.print(adjustment > 0 ? "increased" : "decreased");
    Serial.print(" to: ");
    Serial.println(desiredHumidity);
  }
}

unsigned long getElapsedTime(unsigned long startTime) {
  unsigned long currentTime = millis();
  return (currentTime >= startTime) ? currentTime - startTime : (0xFFFFFFFF - startTime) + currentTime;
}

// Add at the top with other constants
const uint8_t SENSOR_READ_RETRIES = 3;
const unsigned long SENSOR_RETRY_DELAY = 1000;  // 1 second between retries

// Add with other constants
const unsigned long MQTT_RECONNECT_BASE_DELAY = 1000;  // Start with 1 second
const unsigned long MQTT_RECONNECT_MAX_DELAY = 60000;  // Max 1 minute
unsigned long mqttReconnectDelay = MQTT_RECONNECT_BASE_DELAY;
unsigned long lastMqttReconnectAttempt = 0;

// Add with other constants
const float MIN_HUMIDITY = 0.0f;
const float MAX_HUMIDITY = 100.0f;
const float MIN_TEMPERATURE = 0.0f;
const float MAX_TEMPERATURE = 40.0f;
const float MIN_THRESHOLD = 0.1f;
const float MAX_THRESHOLD = 10.0f;
const unsigned long MIN_FAN_DURATION = 10000;    // 10 seconds
const unsigned long MAX_FAN_DURATION = 300000;   // 5 minutes

// Add new validation function
bool validateSettings(float& humidity, float& humidityThresh,
                     float& temperature, float& temperatureThresh,
                     unsigned long& fanDuration, unsigned long& fanIntervalValue) {
  bool valid = true;

  // Validate and clamp humidity
  if (humidity < MIN_HUMIDITY || humidity > MAX_HUMIDITY) {
    humidity = constrain(humidity, MIN_HUMIDITY, MAX_HUMIDITY);
    valid = false;
  }

  // Validate and clamp humidity threshold
  if (humidityThresh < MIN_THRESHOLD || humidityThresh > MAX_THRESHOLD) {
    humidityThresh = constrain(humidityThresh, MIN_THRESHOLD, MAX_THRESHOLD);
    valid = false;
  }

  // Validate and clamp temperature
  if (temperature < MIN_TEMPERATURE || temperature > MAX_TEMPERATURE) {
    temperature = constrain(temperature, MIN_TEMPERATURE, MAX_TEMPERATURE);
    valid = false;
  }

  // Validate and clamp temperature threshold
  if (temperatureThresh < MIN_THRESHOLD || temperatureThresh > MAX_THRESHOLD) {
    temperatureThresh = constrain(temperatureThresh, MIN_THRESHOLD, MAX_THRESHOLD);
    valid = false;
  }

  // Validate and clamp fan duration
  if (fanDuration < MIN_FAN_DURATION || fanDuration > MAX_FAN_DURATION) {
    fanDuration = constrain(fanDuration, MIN_FAN_DURATION, MAX_FAN_DURATION);
    valid = false;
  }

  // Validate and clamp fan interval (minimum 1 minute, maximum 1 hour)
  const unsigned long MIN_FAN_INTERVAL = 60000;    // 1 minute
  const unsigned long MAX_FAN_INTERVAL = 3600000;  // 1 hour
  if (fanIntervalValue < MIN_FAN_INTERVAL || fanIntervalValue > MAX_FAN_INTERVAL) {
    fanIntervalValue = constrain(fanIntervalValue, MIN_FAN_INTERVAL, MAX_FAN_INTERVAL);
    valid = false;
  }

  return valid;
}

void loop() {
  esp_task_wdt_reset();

  if (!client.connected()) {
    connect();
  }

  client.loop();

  unsigned long currentMillis = millis();

  // Update cached relay states
  isFanOn = (digitalRead(FAN_RELAY_PIN) == LOW);
  isPumpOn = (digitalRead(HUMIDIFIER_RELAY_PIN) == LOW);
  isHeaterOn = (digitalRead(HEATER_RELAY_PIN) == LOW);

  handleButton();

  // Main loop tasks every 2 seconds
  if (getElapsedTime(previousMillis) >= updateInterval) {
    previousMillis = currentMillis;

    // Read humidity and temperature with retries
    float humidity = 0;
    float temperature = 0;
    bool validReading = false;

    for (uint8_t retry = 0; retry < SENSOR_READ_RETRIES && !validReading; retry++) {
      if (retry > 0) {
        delay(SENSOR_RETRY_DELAY);
        yield();
      }

#ifdef USE_BME
      humidity = sensor.readHumidity();
      temperature = sensor.readTemperature();
#endif

#ifdef USE_SHT
      uint16_t error = sht45.measureMediumPrecision(temperature, humidity);
      if (error) {
        Serial.print("Error reading SHT sensor: ");
        Serial.println(error);
        continue;
      }
#endif

      if (!isnan(humidity) && !isnan(temperature) && humidity != 0 && temperature != 0) {
        validReading = true;
        break;
      }

      Serial.print("Failed to read from sensor, attempt ");
      Serial.print(retry + 1);
      Serial.print(" of ");
      Serial.println(SENSOR_READ_RETRIES);
    }

    if (!validReading) {
      Serial.println("All sensor read attempts failed!");
      humidity = desiredHumidity;
      temperature = desiredTemperature;
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Sensor failed");
      display.setCursor(0, 16);
      display.print("Check wiring");
      display.display();
    } else {
      updateDisplay(temperature, humidity);
    }

    // Humidifier control based on desired humidity level
    if (!isPumpOn && humidity <= (desiredHumidity - humidityThreshold)) {
      togglePump(true);
    } else if (isPumpOn && humidity >= desiredHumidity) {
      togglePump(false);
    }
    // Heater control based on desired temperature
    if (!isHeaterOn && temperature <= (desiredTemperature - temperatureThreshold)) {
      toggleHeater(true);
    } else if (isHeaterOn && temperature >= (desiredTemperature + temperatureThreshold)) {
      toggleHeater(false);
    }

    float ppm = 0;
#ifdef USE_SCD30
    float scd30_temperature = 0;
    float scd30_humidity = 0;
    int16_t error = scd30.blockingReadMeasurementData(ppm, scd30_temperature, scd30_humidity);
    if (error == 0) {
      Serial.print("SCD30 Temperature: ");
      Serial.print(scd30_temperature);
      Serial.println(" degrees C");

      Serial.print("SCD30 Relative Humidity: ");
      Serial.print(scd30_humidity);
      Serial.println(" %");

      ppm = scd30.CO2;
      Serial.print("CO2: ");
      Serial.print(ppm, 3);
      Serial.println(" ppm");
      Serial.println("");
    } else {
      Serial.println("Error reading sensor data");
    }
#endif

    sendData(temperature, humidity, ppm);
  }

  // Fan control: Turn on for 1 minute every hour
  if (!isPumpOn && !isFanOn && (currentMillis - previousFanMillis >= fanInterval)) {
    toggleFan(true);
    fanStartMillis = currentMillis;
    previousFanMillis = currentMillis;
  }

  if (isFanOn && (currentMillis - fanStartMillis >= fanOnDuration)) {
    toggleFan(false);
  }
}

// MQTT message handler
void messageHandler(String &topic, String &payload) {
  Serial.println("Incoming: " + topic + " - " + payload);

  if (topic != "v1/devices/me/attributes") {
    return;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  bool settingsChanged = false;
  float tempDesiredHumidity = desiredHumidity;
  float tempHumidityThreshold = humidityThreshold;
  float tempDesiredTemperature = desiredTemperature;
  float tempTemperatureThreshold = temperatureThreshold;
  unsigned long tempFanOnDuration = fanOnDuration;
  unsigned long tempFanInterval = fanInterval;  // Add new setting

  if (doc["desiredHumidity"].is<float>()) {
    tempDesiredHumidity = doc["desiredHumidity"].as<float>();
    settingsChanged = true;
  }
  if (doc["humidityThreshold"].is<float>()) {
    tempHumidityThreshold = doc["humidityThreshold"].as<float>();
    settingsChanged = true;
  }
  if (doc["desiredTemperature"].is<float>()) {
    tempDesiredTemperature = doc["desiredTemperature"].as<float>();
    settingsChanged = true;
  }
  if (doc["temperatureThreshold"].is<float>()) {
    tempTemperatureThreshold = doc["temperatureThreshold"].as<float>();
    settingsChanged = true;
  }
  if (doc["fanOnDuration"].is<unsigned long>()) {
    tempFanOnDuration = doc["fanOnDuration"].as<unsigned long>();
    settingsChanged = true;
  }
  if (doc["fanInterval"].is<unsigned long>()) {
    tempFanInterval = doc["fanInterval"].as<unsigned long>();
    settingsChanged = true;
  }

  if (settingsChanged) {
    // Validate and potentially adjust new settings
    if (!validateSettings(tempDesiredHumidity, tempHumidityThreshold,
                        tempDesiredTemperature, tempTemperatureThreshold,
                        tempFanOnDuration, tempFanInterval)) {
      Serial.println("Warning: Some settings were out of range and have been adjusted");
    }

    // Apply validated settings
    desiredHumidity = tempDesiredHumidity;
    humidityThreshold = tempHumidityThreshold;
    desiredTemperature = tempDesiredTemperature;
    temperatureThreshold = tempTemperatureThreshold;
    fanOnDuration = tempFanOnDuration;
    fanInterval = tempFanInterval;  // Apply new setting

    saveSettings();
  }
}

void connect() {
  // Only attempt reconnection after delay has passed
  if (millis() - lastMqttReconnectAttempt < mqttReconnectDelay) {
    return;
  }

  lastMqttReconnectAttempt = millis();

  // Start or reconnect to Wi-Fi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("\nConnecting to Wi-Fi");
    WiFi.begin(ssid, pass);

    uint8_t retryCount = 0;
    while (WiFi.status() != WL_CONNECTED && retryCount < MAX_RETRIES) {
      Serial.print(".");
      delay(500);
      retryCount++;
      yield();
    }

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nFailed to connect to Wi-Fi");
      mqttReconnectDelay = min(mqttReconnectDelay * 2, MQTT_RECONNECT_MAX_DELAY);
      return;
    }

    Serial.println("\nConnected to Wi-Fi");
    // Reset delay on successful WiFi connection
    mqttReconnectDelay = MQTT_RECONNECT_BASE_DELAY;
  }

  Serial.print("\nConnecting to MQTT broker");
  client.begin(broker, port, net);
  client.onMessage(messageHandler);

  if (client.connect("ESP32Client", token, (const char *)nullptr)) {
    Serial.println("\nConnected to MQTT broker");
    client.subscribe("v1/devices/me/attributes");
    publishSettings();
    // Reset delay on successful connection
    mqttReconnectDelay = MQTT_RECONNECT_BASE_DELAY;
  } else {
    Serial.print("\nFailed to connect to MQTT broker (error: ");
    Serial.print(client.lastError());
    Serial.println("), retrying later...");
    mqttReconnectDelay = min(mqttReconnectDelay * 2, MQTT_RECONNECT_MAX_DELAY);
  }
}

void sendData(float temperature, float humidity, int ppm) {
  char payload[64];
  snprintf(payload, sizeof(payload), "{\"temperature\":%.1f,\"humidity\":%.1f,\"co2\":%d}", temperature, humidity, ppm);
  publishTelemetry(payload);
}

// Helper method to publish settings as attributes
void publishSettings() {
  char payload[160];  // Increased buffer size to accommodate new setting
  snprintf(payload, sizeof(payload),
           "{\"desiredHumidity\":%.1f,\"humidityThreshold\":%.1f,"
           "\"desiredTemperature\":%.1f,\"temperatureThreshold\":%.1f,"
           "\"fanOnDuration\":%lu,\"fanInterval\":%lu}",  // Add fanInterval
           desiredHumidity, humidityThreshold,
           desiredTemperature, temperatureThreshold,
           fanOnDuration, fanInterval);
  if (client.connected()) {
    client.publish("v1/devices/me/attributes", payload, true, 1);
  }
}

// Load settings from NVS
void loadSettings() {
  preferences.begin("tent-ctrl", false);

  float tempDesiredHumidity = preferences.getFloat("desiredHum", DEFAULT_DESIRED_HUMIDITY);
  float tempHumidityThreshold = preferences.getFloat("humidThresh", DEFAULT_HUMIDITY_THRESHOLD);
  float tempDesiredTemperature = preferences.getFloat("desiredTemp", DEFAULT_DESIRED_TEMPERATURE);
  float tempTemperatureThreshold = preferences.getFloat("tempThresh", DEFAULT_TEMPERATURE_THRESHOLD);
  unsigned long tempFanOnDuration = preferences.getULong("fanDuration", DEFAULT_FAN_DURATION);
  unsigned long tempFanInterval = preferences.getULong("fanInterval", DEFAULT_FAN_INTERVAL);  // Add new setting

  preferences.end();

  // Validate and potentially adjust settings
  if (!validateSettings(tempDesiredHumidity, tempHumidityThreshold,
                       tempDesiredTemperature, tempTemperatureThreshold,
                       tempFanOnDuration, tempFanInterval)) {  // Add fanInterval to validation
    Serial.println("Warning: Some settings were out of range and have been adjusted");
  }

  // Apply validated settings
  desiredHumidity = tempDesiredHumidity;
  humidityThreshold = tempHumidityThreshold;
  desiredTemperature = tempDesiredTemperature;
  temperatureThreshold = tempTemperatureThreshold;
  fanOnDuration = tempFanOnDuration;
  fanInterval = tempFanInterval;  // Apply new setting

  // Publish current settings
  publishSettings();
}

// Save settings to NVS
void saveSettings() {
  preferences.begin("tent-ctrl", false);

  preferences.putFloat("desiredHum", desiredHumidity);
  preferences.putFloat("humidThresh", humidityThreshold);
  preferences.putFloat("desiredTemp", desiredTemperature);
  preferences.putFloat("tempThresh", temperatureThreshold);
  preferences.putULong("fanDuration", fanOnDuration);
  preferences.putULong("fanInterval", fanInterval);  // Add new setting

  preferences.end();

  // Publish updated settings
  publishSettings();
}

