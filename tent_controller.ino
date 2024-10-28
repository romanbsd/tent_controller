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

// Timing variables for fan control
unsigned long previousFanMillis = 0;       // Tracks last time the fan was turned on
unsigned long fanStartMillis = 0;          // Tracks the time when fan is turned on
const unsigned long fanInterval = 3600000; // 1 hour in milliseconds
const unsigned long fanOnDuration = 60000; // 1 minute in milliseconds

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
  // Read humidity and temperature from the sensor
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Check if reading was successful
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    humidity = desiredHumidity;
    temperature = 0;
  }

  // Display the readings on the Serial Monitor
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print("%\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("C");

  // **Display temperature and humidity on the LCD**
  lcd.clear();  // Clear the display before writing new data
  lcd.setCursor(0, 0);  // Set the cursor to the first row
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C");
  
  lcd.setCursor(0, 1);  // Set the cursor to the second row
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");

  if (mhz.isReady()) {
    int ppm = mhz.readCO2UART();
    lcd.print(" PPM: ");
    lcd.print(ppm);
    
    int temperature = mhz.getLastTemperature();
    Serial.print(", Temperature: ");

    if (temperature > 0) {
      Serial.println(temperature);
    } else {
      Serial.println("n/a");
    }
  }

  // Control the humidifier based on the desired humidity level
  if (!isPumpOn && humidity < (desiredHumidity - humidityThreshold)) {
    digitalWrite(HUMIDIFIER_RELAY_PIN, HIGH); // Turn on the humidifier
    Serial.println("Humidifier ON");
  } else if (isPumpOn && humidity > (desiredHumidity + humidityThreshold)) {
    digitalWrite(HUMIDIFIER_RELAY_PIN, LOW); // Turn off the humidifier
    Serial.println("Humidifier OFF");
  }

  unsigned long currentMillis = millis();

  // Check if it’s time to turn the fan on (every hour)
  if (!isFanOn && (currentMillis - previousFanMillis >= fanInterval)) {
    // Turn on the fan
    digitalWrite(FAN_RELAY_PIN, HIGH);
    Serial.println("Fan ON");

    // Record the time the fan was turned on and update the previous fan time
    fanStartMillis = currentMillis;
    previousFanMillis = currentMillis;

    // Set the fan state to on
    isFanOn = true;
  }

  // Check if it’s time to turn the fan off (after 1 minute of being on)
  if (isFanOn && (currentMillis - fanStartMillis >= fanOnDuration)) {
    // Turn off the fan
    digitalWrite(FAN_RELAY_PIN, LOW);
    Serial.println("Fan OFF");

    // Reset the fan state to off
    isFanOn = false;
  }

  // Small delay to avoid flooding the display and serial monitor
  delay(2000);  // Update every 2 seconds
}
