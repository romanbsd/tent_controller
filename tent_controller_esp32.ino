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

#define BUTTON_PIN A3
#define DEBOUNCE_DELAY 50

const uint8_t MAX_RETRIES = 10;
const char ssid[] = "eternity";
const char pass[] = "OpenSesame3";
const char token[] = "t0x1394lpdoaye1h6ac1";
const char broker[] = "dash.gugl.org";
const int port = 1883;

WiFiClient net;
MQTTClient client;

// Timing variables
unsigned long previousMillis = 0;
unsigned long previousFanMillis = 0;        // Tracks last time the fan was turned on
unsigned long fanStartMillis = 0;           // Tracks the time when fan is turned on
const unsigned long fanInterval = 3600000;  // 1 hour in milliseconds
const unsigned long updateInterval = 2000;  // 2-second interval for main loop logic

// Create preferences object
Preferences preferences;

// Default values for settings
const float DEFAULT_DESIRED_HUMIDITY = 90.0f;
const float DEFAULT_HUMIDITY_THRESHOLD = 4.0f;
const float DEFAULT_DESIRED_TEMPERATURE = 23.0f;
const float DEFAULT_TEMPERATURE_THRESHOLD = 2.0f;
const unsigned long DEFAULT_FAN_DURATION = 60000;  // 1 minute in milliseconds

// Settings variables
float desiredHumidity;     // Desired humidity percentage
float humidityThreshold;   // Tolerance range for humidity
float desiredTemperature;  // Target temperature
float temperatureThreshold;
unsigned long fanOnDuration;

bool isFanOn = false;
bool isPumpOn = false;
bool isHeaterOn = false;

// Define larger icons (16x16 pixels)
const unsigned char PROGMEM windIcon[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0,
    0x0F, 0xF0, 0x1F, 0xF8, 0x3F, 0xFC, 0x3F, 0xFC,
    0x3F, 0xFC, 0x3F, 0xFC, 0x1F, 0xF8, 0x0F, 0xF0,
    0x03, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char PROGMEM heatIcon[] = {
    0x01, 0x80, 0x03, 0xC0, 0x07, 0xE0, 0x0F, 0xF0,
    0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0, 0x07, 0xE0,
    0x07, 0xE0, 0x0F, 0xF0, 0x1F, 0xF8, 0x3F, 0xFC,
    0x3F, 0xFC, 0x1F, 0xF8, 0x0F, 0xF0, 0x07, 0xE0
};

const unsigned char PROGMEM waterIcon[] = {
    0x01, 0x80, 0x03, 0xC0, 0x07, 0xE0, 0x0F, 0xF0,
    0x1F, 0xF8, 0x3F, 0xFC, 0x3F, 0xFC, 0x7F, 0xFE,
    0x7F, 0xFE, 0x7F, 0xFE, 0x7F, 0xFE, 0x3F, 0xFC,
    0x3F, 0xFC, 0x1F, 0xF8, 0x0F, 0xF0, 0x07, 0xE0
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
    display.drawBitmap(0, 48, windIcon, 16, 16, SSD1306_WHITE);
  }
  if (isHeaterOn) {
    display.drawBitmap(56, 48, heatIcon, 16, 16, SSD1306_WHITE);
  }
  if (isPumpOn) {
    display.drawBitmap(112, 48, waterIcon, 16, 16, SSD1306_WHITE);
  }
}

void setup() {
  Serial.begin(9600);
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

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Turn off the relays initially
  digitalWrite(FAN_RELAY_PIN, LOW);
  digitalWrite(HEATER_RELAY_PIN, LOW);
  digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);

  // Configure the Task Watchdog Timer (TWDT)
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = 5000,            // Set the timeout to 5000 ms (5 seconds)
    .idle_core_mask = 0b01,        // Monitor idle task of core 0 only (ESP32 has two cores: core 0 and core 1)
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
  digitalWrite(FAN_RELAY_PIN, on ? HIGH : LOW);  // Turn on fan
  isFanOn = on;
  Serial.print("Fan ");
  Serial.println(on ? "ON" : "OFF");
  publishDeviceState("fan", on);
}

void togglePump(bool on) {
  digitalWrite(HUMIDIFIER_RELAY_PIN, on ? HIGH : LOW);  // Turn on humidifier
  isPumpOn = on;
  Serial.print("Humidifier ");
  Serial.println(on ? "ON" : "OFF");
  publishDeviceState("humidifier", on);
}

void toggleHeater(bool on) {
  digitalWrite(HEATER_RELAY_PIN, on ? HIGH : LOW);  // Turn on heater
  isHeaterOn = on;
  Serial.print("Heater ");
  Serial.println(on ? "ON" : "OFF");
  publishDeviceState("heater", on);
}

void updateDisplay(float temperature, float humidity) {
  display.clearDisplay();

  // Display temperature
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print(temperature, 1);
  display.print("C");

  // Display humidity
  display.setCursor(0, 20);
  display.print(humidity, 1);
  display.print("%");

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
  static uint8_t lastSteadyState = HIGH, lastFlickerableState = HIGH;
  static unsigned long lastDebounceTime = 0;

  int currentState = digitalRead(BUTTON_PIN);

  // Debounce logic: Update flickerable state and debounce timer
  if (currentState != lastFlickerableState) {
    lastDebounceTime = millis();
    lastFlickerableState = currentState;
  }

  // Check for a stable button state
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    if (lastSteadyState == HIGH && currentState == LOW) {
      Serial.println("Button pressed");
      toggleFan(!isFanOn);
      fanStartMillis = millis();
    }
    lastSteadyState = currentState;  // Update steady state
  }
}

unsigned long getElapsedTime(unsigned long startTime) {
  unsigned long currentTime = millis();
  return (currentTime >= startTime) ? currentTime - startTime : (0xFFFFFFFF - startTime) + currentTime;
}

void loop() {
  if (!client.connected()) {
    connect();
  }

  client.loop();

  unsigned long currentMillis = millis();

  handleButton();

  // Main loop tasks every 2 seconds
  if (getElapsedTime(previousMillis) >= updateInterval) {
    previousMillis = currentMillis;

    // Read humidity and temperature
    float humidity;
    float temperature;

#ifdef USE_BME
    humidity = sensor.readHumidity();
    temperature = sensor.readTemperature();
#endif

#ifdef USE_SHT
    sht45.measureMediumPrecision(temperature, humidity);
#endif

    // Check for valid readings
    if (isnan(humidity) || isnan(temperature) || humidity == 0 || temperature == 0) {
      Serial.println("Failed to read from sensor!");
      humidity = desiredHumidity;
      temperature = desiredTemperature;
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Sensor read failed");
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

  // Only handle shared attributes updates
  if (topic == "v1/devices/me/attributes") {
    // Parse JSON payload
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    bool settingsChanged = false;

    // Update settings if present in the payload
    if (doc["desiredHumidity"].is<float>()) {
      desiredHumidity = doc["desiredHumidity"].as<float>();
      settingsChanged = true;
    }
    if (doc["humidityThreshold"].is<float>()) {
      humidityThreshold = doc["humidityThreshold"].as<float>();
      settingsChanged = true;
    }
    if (doc["desiredTemperature"].is<float>()) {
      desiredTemperature = doc["desiredTemperature"].as<float>();
      settingsChanged = true;
    }
    if (doc["temperatureThreshold"].is<float>()) {
      temperatureThreshold = doc["temperatureThreshold"].as<float>();
      settingsChanged = true;
    }
    if (doc["fanOnDuration"].is<unsigned long>()) {
      fanOnDuration = doc["fanOnDuration"].as<unsigned long>();
      settingsChanged = true;
    }

    // If any setting was changed, save to NVS
    if (settingsChanged) {
      saveSettings();
    }
  }
}

void connect() {
  uint8_t retryCount = 0;

  // Start or reconnect to Wi-Fi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("\nConnecting to Wi-Fi");
    WiFi.begin(ssid, pass);  // Start the Wi-Fi connection

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED && retryCount < MAX_RETRIES) {  // Retry up to 10 times (5 seconds)
      Serial.print(".");
      delay(500);  // Wait 500 ms before retrying
      retryCount++;
      yield();
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected to Wi-Fi");
    } else {
      Serial.println("\nFailed to connect to Wi-Fi");
      return;
    }
  }

  Serial.print("\nConnecting to MQTT broker");
  client.begin(broker, port, net);

  // Set up message handler
  client.onMessage(messageHandler);

  retryCount = 0;
  while (!client.connected() && retryCount < MAX_RETRIES) {
    client.connect("ESP32Client", token, (const char *)nullptr);
    Serial.print(".");
    delay(500);
    retryCount++;
    yield();
  }

  if (client.connected()) {
    Serial.println("\nConnected to MQTT broker");
    // Subscribe to shared attributes topic
    client.subscribe("v1/devices/me/attributes");
    // Publish current settings after connection
    publishSettings();
  } else {
    Serial.println("\nFailed to connect to MQTT broker, retrying later...");
  }
}

void sendData(float temperature, float humidity, int ppm) {
  char payload[64];
  snprintf(payload, sizeof(payload), "{\"temperature\":%.1f,\"humidity\":%.1f,\"co2\":%d}", temperature, humidity, ppm);
  publishTelemetry(payload);
}

// Helper method to publish settings as attributes
void publishSettings() {
  char payload[128];
  snprintf(payload, sizeof(payload),
           "{\"desiredHumidity\":%.1f,\"humidityThreshold\":%.1f,"
           "\"desiredTemperature\":%.1f,\"temperatureThreshold\":%.1f,"
           "\"fanOnDuration\":%lu}",
           desiredHumidity, humidityThreshold,
           desiredTemperature, temperatureThreshold,
           fanOnDuration);
  if (client.connected()) {
    client.publish("v1/devices/me/attributes", payload, true, 1);
  }
}

// Load settings from NVS
void loadSettings() {
  preferences.begin("tent-ctrl", false);  // false = read/write mode

  // Load settings with defaults if not found
  desiredHumidity = preferences.getFloat("desiredHum", DEFAULT_DESIRED_HUMIDITY);
  humidityThreshold = preferences.getFloat("humidThresh", DEFAULT_HUMIDITY_THRESHOLD);
  desiredTemperature = preferences.getFloat("desiredTemp", DEFAULT_DESIRED_TEMPERATURE);
  temperatureThreshold = preferences.getFloat("tempThresh", DEFAULT_TEMPERATURE_THRESHOLD);
  fanOnDuration = preferences.getULong("fanDuration", DEFAULT_FAN_DURATION);

  preferences.end();

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

  preferences.end();

  // Publish updated settings
  publishSettings();
}

