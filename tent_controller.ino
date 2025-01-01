#include <LowPower.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define BME280 sensor settings
#define BME280_I2C_ADDR 0x76  // Default I2C address for BME280
Adafruit_BME280 bme;

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
  Serial.begin(9600);
  Serial.println("Serial started");

  // Initialize the BME280 sensor
  if (!bme.begin(BME280_I2C_ADDR)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

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

void loop() {
  unsigned long currentMillis = millis();

  handleButton();

  // Main loop tasks every 2 seconds
  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;

    // Read humidity and temperature
    float humidity = bme.readHumidity();
    float temperature = bme.readTemperature();

    // Check for valid readings
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from BME280 sensor!");
      humidity = desiredHumidity;
      temperature = desiredTemperature;
    }

    updateDisplay(temperature, humidity);

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

  // Enter idle mode until the next interrupt (saves power)
  LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_ON, TWI_ON);
}
