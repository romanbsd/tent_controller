#define USE_BME

#include <avr/wdt.h>
#include <LiquidCrystal_I2C.h>
#ifdef USE_DHT
#include <DHT.h>
#endif
#ifdef USE_BME
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#endif

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

#ifdef USE_DHT
// Define DHT sensor settings
#define DHTPIN 2  // Pin where the DHT sensor is connected
#define DHTTYPE DHT22
DHT sensor(DHTPIN, DHTTYPE);
#endif

#ifdef USE_BME
// Define BME280 sensor settings
#define BME280_I2C_ADDR 0x76  // Default I2C address for BME280
Adafruit_BME280 sensor;
#endif

// Define pins for relay modules
#define FAN_RELAY_PIN 2
#define HEATER_RELAY_PIN 3
#define HUMIDIFIER_RELAY_PIN 4

#define BUTTON_PIN A3
#define DEBOUNCE_DELAY 50


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
  B00000,
  B00100,
  B10101,
  B01110,
  B10101,
  B00100,
  B00000,
  B00000
};

byte fireIcon[8] = {
  B00000,
  B00100,
  B01010,
  B00101,
  B01010,
  B10100,
  B01000,
  B11100
};

byte waterDropIcon[8] = {
  B00100,
  B00100,
  B01110,
  B01110,
  B11111,
  B11111,
  B01110,
  B00000
};

void setup() {
  wdt_disable();

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
  wdt_enable(WDTO_8S);
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
  wdt_reset();
  unsigned long currentMillis = millis();

  handleButton();

  // Main loop tasks every 2 seconds
  if (getElapsedTime(previousMillis) >= updateInterval) {
    previousMillis = currentMillis;

    // Read humidity and temperature
    float humidity = sensor.readHumidity();
    float temperature = sensor.readTemperature();

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
    } else if (isPumpOn && humidity >= (desiredHumidity + humidityThreshold)) {
      togglePump(false);
    }
    // Heater control based on desired temperature
    if (!isHeaterOn && temperature <= (desiredTemperature - temperatureThreshold)) {
      toggleHeater(true);
    } else if (isHeaterOn && temperature >= (desiredTemperature + temperatureThreshold)) {
      toggleHeater(false);
    }
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
