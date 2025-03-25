# ESP32 Grow Tent Controller

A smart controller for managing environmental conditions in a grow tent using an ESP32 microcontroller. The system monitors and controls temperature, humidity, and air circulation while providing real-time feedback through an OLED display and MQTT connectivity.

## Features

- **Temperature Control**
  - Configurable target temperature
  - Adjustable threshold for temperature control
  - Automatic heater control based on temperature readings

- **Humidity Control**
  - Configurable target humidity
  - Adjustable threshold for humidity control
  - Automatic humidifier control based on humidity readings
  - Manual adjustment via physical buttons

- **Air Circulation**
  - Configurable fan duration and interval
  - Automatic fan control to prevent stale air
  - Fan operation coordinated with humidifier to prevent interference

- **User Interface**
  - OLED display showing current and target values
  - Status icons for fan, heater, and humidifier
  - Physical buttons for manual humidity adjustment

- **Connectivity**
  - WiFi connection for remote monitoring
  - MQTT integration for remote control and monitoring
  - Automatic reconnection with exponential backoff
  - Real-time telemetry data transmission

## Hardware Components

- **Controller**
  - ESP32 development board
  - 128x64 OLED display (SSD1306)
  - 3x Relay modules for fan, heater, and humidifier control
  - 2x Push buttons for humidity adjustment
  - SHT45 temperature and humidity sensor

- **Optional Sensors**
  - SCD30 CO2 sensor (optional)
  - BME280 temperature and humidity sensor (optional)
  - DHT22 temperature and humidity sensor (optional)

## Pin Configuration

- `GPIO23`: Heater relay control
- `GPIO25`: Fan relay control
- `GPIO26`: Humidifier relay control
- `GPIO32`: Humidity up button
- `GPIO33`: Humidity down button
- `I2C`: OLED display and sensors

## Default Settings

- Desired Humidity: 90%
- Humidity Threshold: 4%
- Desired Temperature: 23°C
- Temperature Threshold: 2°C
- Fan Duration: 30 seconds
- Fan Interval: 15 minutes

## MQTT Integration

The device publishes and subscribes to the following topics:

- **Telemetry Topic**: `v1/devices/me/telemetry`
  - Publishes current temperature, humidity, and CO2 levels
  - Publishes device state changes (fan, heater, humidifier)

- **Attributes Topic**: `v1/devices/me/attributes`
  - Subscribes to configuration updates
  - Publishes current settings

### MQTT Message Format

```json
// Telemetry
{
  "temperature": 23.5,
  "humidity": 85.0,
  "co2": 800
}

// Device State
{
  "fan_on": 1,
  "heater_on": 0,
  "humidifier_on": 1
}

// Settings
{
  "desiredHumidity": 90.0,
  "humidityThreshold": 4.0,
  "desiredTemperature": 23.0,
  "temperatureThreshold": 2.0,
  "fanOnDuration": 30000,
  "fanInterval": 900000
}
```

## Dependencies

- WiFi.h
- MQTT.h
- Wire.h
- Adafruit_GFX.h
- Adafruit_SSD1306.h
- Preferences.h
- ArduinoJson.h
- SensirionI2cSht4x.h (optional)
- SensirionI2cScd30.h (optional)
- DHT.h (optional)
- Adafruit_BME280.h (optional)

## Configuration

1. Update WiFi credentials in the code:
   ```cpp
   const char ssid[] = "your_ssid";
   const char pass[] = "your_password";
   ```

2. Update MQTT credentials:
   ```cpp
   const char token[] = "your_token";
   const char broker[] = "your_broker";
   const int port = 1883;
   ```

3. Configure sensor type by uncommenting the appropriate define:
   ```cpp
   #define USE_SHT    // For SHT45
   //#define USE_BME   // For BME280
   //#define USE_DHT   // For DHT22
   //#define USE_SCD30 // For SCD30
   ```

## Installation

1. Install the required libraries through the Arduino Library Manager
2. Connect the hardware components according to the pin configuration
3. Upload the sketch to your ESP32
4. Monitor the Serial output (115200 baud) for connection status

## Usage

1. Power on the device
2. The OLED display will show current readings and target values
3. Use the physical buttons to adjust humidity
4. Monitor and control the device through MQTT
5. The system will automatically maintain the desired environmental conditions

## Error Handling

- Automatic retry for sensor readings
- Exponential backoff for WiFi and MQTT reconnection
- Watchdog timer to prevent system hangs
- Settings validation to ensure safe operation
- Fallback to default values if sensor readings fail

## License

This project is open source and available under the MIT License.