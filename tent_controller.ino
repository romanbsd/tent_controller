#include <LowPower.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <MHZ.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define DHT sensor settings
#define DHTPIN 2           // Pin where the DHT sensor is connected
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Define MH-Z19C sensor settings
#define MHZRXPIN 10
#define MHZTXPIN 11
MHZ mhz(MHZRXPIN, MHZTXPIN, MHZ19C);

// Define pins for relay modules
#define FAN_RELAY_PIN 3
#define HUMIDIFIER_RELAY_PIN 4

// Timing variables
unsigned long previousMillis = 0;
unsigned long previousFanMillis = 0;       // Tracks last time the fan was turned on
unsigned long fanStartMillis = 0;          // Tracks the time when fan is turned on
const unsigned long fanInterval = 3600000; // 1 hour in milliseconds
const unsigned long fanOnDuration = 60000; // 1 minute in milliseconds
const unsigned long updateInterval = 2000; // 2-second interval for main loop logic

// Humidity and temperature thresholds
float desiredHumidity = 90.0; // Desired humidity percentage
float humidityThreshold = 4.0; // Tolerance range for humidity

bool isFanOn = false;
bool isPumpOn = false;

void setup() {
  Serial.begin(9600);
  
  // Initialize the DHT sensor
  dht.begin();

  // Initialize the LCD
  lcd.init();
  lcd.backlight();  // Turn on the LCD backlight

  // Set relay pins as output
  pinMode(FAN_RELAY_PIN, OUTPUT);
  pinMode(HUMIDIFIER_RELAY_PIN, OUTPUT);

  // Turn off the relays initially
  digitalWrite(FAN_RELAY_PIN, LOW);
  digitalWrite(HUMIDIFIER_RELAY_PIN, LOW);

  mhz.setDebug(true);
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
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
    lcd.print("%");

    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print("%\t");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("C");

    // Additional MHZ CO2 sensor reading
    if (mhz.isReady()) {
      int ppm = mhz.readCO2UART();
      int temperature = mhz.getLastTemperature();

      lcd.print(" PPM: ");
      lcd.print(ppm);
      Serial.print(", PPM: ");
      Serial.print(ppm);

      if (temperature > 0) {
        Serial.print(", Temperature: ");
        Serial.println(temperature);
      } else {
        Serial.println(", Temperature: n/a");
      }
    }

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
  LowPower.idle(SLEEP_2S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_ON, SPI_OFF, USART0_OFF, TWI_OFF);
}
