#define USE_SHT

#include <WiFi.h>
#include <MQTT.h>
#include <esp_task_wdt.h>
#include <LiquidCrystal_I2C.h>
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

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

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
const unsigned long fanOnDuration = 60000;  // 1 minute in milliseconds
const unsigned long updateInterval = 2000;  // 2-second interval for main loop logic

// Humidity and temperature thresholds
const float desiredHumidity = 90.f;     // Desired humidity percentage
const float humidityThreshold = 4.f;    // Tolerance range for humidity
const float desiredTemperature = 23.f;  // 21-25
const float temperatureThreshold = 2.f;

bool isFanOn = false;
bool isPumpOn = false;
bool isHeaterOn = false;


// Define the custom characters
byte fanIcon[8] = {
  0b00000,
  0b00100,
  0b10101,
  0b01110,
  0b10101,
  0b00100,
  0b00000,
  0b00000
};

byte fireIcon[8] = {
  0b00000,
  0b00100,
  0b01010,
  0b00101,
  0b01010,
  0b10100,
  0b01000,
  0b11100
};

byte waterDropIcon[8] = {
  0b00100,
  0b00100,
  0b01110,
  0b01110,
  0b11111,
  0b11111,
  0b01110,
  0b00000
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


void setup() {
  Serial.begin(9600);
  Serial.println("Serial started");

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

  // Initialize the LCD
  lcd.init();
  lcd.backlight();  // Turn on the LCD backlight

  // Create the custom characters
  lcd.createChar(0, fanIcon);
  lcd.createChar(1, fireIcon);
  lcd.createChar(2, waterDropIcon);

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

void toggleFan(bool on) {
  digitalWrite(FAN_RELAY_PIN, on ? HIGH : LOW);  // Turn on fan
  isFanOn = on;
  Serial.print("Fan ");
  Serial.println(on ? "ON" : "OFF");
}

void togglePump(bool on) {
  digitalWrite(HUMIDIFIER_RELAY_PIN, on ? HIGH : LOW);  // Turn on humidifier
  isPumpOn = on;
  Serial.print("Humidifier ");
  Serial.println(on ? "ON" : "OFF");
}

void toggleHeater(bool on) {
  digitalWrite(HEATER_RELAY_PIN, on ? HIGH : LOW);  // Turn on heater
  isHeaterOn = on;
  Serial.print("Heater ");
  Serial.println(on ? "ON" : "OFF");
}

void updateDisplay(float temperature, float humidity) {
  // Display humidity and temperature on LCD and Serial
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print(temperature);
  lcd.print("C, ");
  lcd.print(humidity);
  lcd.print("%");

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%\tTemperature: ");
  Serial.print(temperature);
  Serial.println("C");

  if (isFanOn) {
    lcd.setCursor(3, 1);
    lcd.write((byte)0);
  }
  if (isHeaterOn) {
    lcd.setCursor(7, 1);
    lcd.write((byte)1);
  }
  if (isPumpOn) {
    lcd.setCursor(11, 1);
    lcd.write((byte)2);
  }
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
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Sensor read failed");
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
      return;  // Exit if Wi-Fi connection fails
    }
  }

  Serial.print("\nConnecting to MQTT broker");
  client.begin(broker, port, net);
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
  } else {
    Serial.println("\nFailed to connect to MQTT broker, retrying later...");
  }
}

void sendData(float temperature, float humidity, int ppm) {
  char payload[64];
  snprintf(payload, sizeof(payload), "{\"temperature\":%.1f,\"humidity\":%.1f,\"co2\":%d}", temperature, humidity, ppm);
  // Publish the data
  Serial.print("Sending payload: ");
  Serial.println(payload);
  client.publish("v1/devices/me/telemetry", payload, true, 1);
}

