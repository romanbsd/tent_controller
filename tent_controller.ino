#include <LowPower.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define DHT sensor settings
#define DHTPIN 2           // Pin where the DHT sensor is connected
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Define pins for relay modules
#define FAN_RELAY_PIN 0
#define HEATER_RELAY_PIN 1
#define HUMIDIFIER_RELAY_PIN 2

// Timing variables
unsigned long previousMillis = 0;
unsigned long previousFanMillis = 0;       // Tracks last time the fan was turned on
unsigned long fanStartMillis = 0;          // Tracks the time when fan is turned on
const unsigned long fanInterval = 3600000; // 1 hour in milliseconds
const unsigned long fanOnDuration = 60000; // 1 minute in milliseconds
const unsigned long updateInterval = 2000; // 2-second interval for main loop logic

// Humidity and temperature thresholds
const float desiredHumidity = 90.f; // Desired humidity percentage
const float humidityThreshold = 4.f; // Tolerance range for humidity
const float desiredTemperature = 23.f; // 22-24
const float temperatureThreshold = 2.f;

bool isFanOn = false;
bool isPumpOn = false;
bool isHeaterOn = false;


// Define the custom characters
byte fanIcon[8] = {
  B00100,
  B10101,
  B01110,
  B10101,
  B00100,
  B00000,
  B00000,
  B00000
};

byte fireIcon[8] = {
  B00100,
  B01010,
  B00101,
  B01010,
  B10100,
  B01000,
  B11100,
  B00000
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
  
  // Initialize the DHT sensor
  dht.begin();

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

  // Turn off the relays initially
  digitalWrite(FAN_RELAY_PIN, LOW);
  digitalWrite(HEATER_RELAY_PIN, LOW);
  digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);
}

void loop() {
  unsigned long currentMillis = millis();

  // Main loop tasks every 2 seconds
  if (currentMillis - previousMillis >= updateInterval) {
    previousMillis = currentMillis;

    // Read humidity and temperature
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    // Check for valid readings
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      humidity = desiredHumidity;
      temperature = 0;
    }

    // Display humidity and temperature on LCD and Serial
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(temperature);
    lcd.print("C, Humidity: ");
    lcd.print(humidity);
    lcd.print("%");

    Serial.print("\nHumidity: ");
    Serial.print(humidity);
    Serial.print("%\t");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("C");

    lcd.setCursor(1, 1);
    isFanOn ? lcd.write((byte)0) : lcd.print(' ');
    lcd.setCursor(3, 1);
    isHeaterOn ? lcd.write((byte)1) : lcd.print(' ');
    lcd.setCursor(5, 1);
    isPumpOn ? lcd.write((byte)2) : lcd.print(' ');

    // Humidifier control based on desired humidity level
    if (!isPumpOn && humidity <= (desiredHumidity - humidityThreshold)) {
      digitalWrite(HUMIDIFIER_RELAY_PIN, HIGH); // Turn on humidifier
      isPumpOn = true;
      Serial.println("Humidifier ON");
    } else if (isPumpOn && humidity >= (desiredHumidity + humidityThreshold)) {
      digitalWrite(HUMIDIFIER_RELAY_PIN, LOW); // Turn off humidifier
      isPumpOn = false;
      Serial.println("Humidifier OFF");
    }
    // Heater control based on desired temperature
    if (!isHeaterOn && temperature <= (desiredTemperature - temperatureThreshold)) {
      digitalWrite(HEATER_RELAY_PIN, HIGH); // Turn on heater
      isHeaterOn = true;
      Serial.println("Heater ON");
    } else if (isHeaterOn && temperature >= (desiredTemperature + temperatureThreshold)) {
      digitalWrite(HEATER_RELAY_PIN, LOW); // Turn off heater
      isHeaterOn = false;
      Serial.println("Heater OFF");
    }
  }

  // Fan control: Turn on for 1 minute every hour
  if (!isFanOn && (currentMillis - previousFanMillis >= fanInterval)) {
    digitalWrite(FAN_RELAY_PIN, HIGH); // Turn on fan
    isFanOn = true;
    fanStartMillis = currentMillis;
    previousFanMillis = currentMillis;
    Serial.println("Fan ON");
  }

  if (isFanOn && (currentMillis - fanStartMillis >= fanOnDuration)) {
    digitalWrite(FAN_RELAY_PIN, LOW); // Turn off fan
    isFanOn = false;
    Serial.println("Fan OFF");
  }

  // Enter idle mode until the next interrupt (saves power)
  LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_ON, TWI_ON);
}
