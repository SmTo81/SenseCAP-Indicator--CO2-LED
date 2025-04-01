// SenseCAP Indicator - Simplified Version
// This code reads sensor data from AHT20, SGP40, and SCD4x sensors,
// writes the data to an SD card, and updates a NeoPixel LED strip.
// The LED brightness is fixed at 5% of maximum (constant brightness).
// LED update is now performed together with sensor measurements.

#include <Arduino.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2cScd4x.h>
#include <VOCGasIndexAlgorithm.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <PacketSerial.h>
#include "AHT20.h"
#include <Adafruit_NeoPixel.h>

// Version string and banner
#define DEBUG 0
#define VERSION "v1.0.1"

// SENSECAP banner
#define SENSECAP "\n" \
"   _____                      _________    ____         \n" \
"  / ___/___  ____  _______  / ____/   |  / __ \\       \n" \
"  \\__ \\/ _ \\/ __ \\/ ___/ _ \\/ /   / /| | / /_/ /   \n" \
" ___/ /  __/ / / (__  )  __/ /___/ ___ |/ ____/         \n" \
"/____/\\___/_/ /_/____/\\___/\\____/_/  |_|/_/           \n" \
"--------------------------------------------------------\n" \
" Version: %s \n" \
"--------------------------------------------------------\n"

// Maximum CO2 value (ppm)
#define MAX_CO2 2000

// Constant LED brightness at 5% of 255 (approximately 13)
const uint8_t LED_BRIGHTNESS = (5 * 255) / 100; 

// Global sensor objects and communication objects
AHT20 AHT;
SensirionI2CSgp40 sgp40;
SensirionI2cScd4x scd4x;
VOCGasIndexAlgorithm voc_algorithm;
PacketSerial myPacketSerial;
String SDDataString = "";

// Global variables for sensor compensation values
float temperature = 0.0;
float humidity = 0.0;
uint16_t defaultCompenstaionRh = 0x8000;
uint16_t defaultCompenstaionT = 0x6666;
uint16_t compensationRh = defaultCompenstaionRh;
uint16_t compensationT = defaultCompenstaionT;

// Packet types for sensor data
#define PKT_TYPE_SENSOR_SCD41_CO2    0xB2
#define PKT_TYPE_SENSOR_SHT41_TEMP     0xB3
#define PKT_TYPE_SENSOR_SHT41_HUMIDITY 0xB4
#define PKT_TYPE_SENSOR_TVOC_INDEX     0xB5
#define PKT_TYPE_CMD_COLLECT_INTERVAL  0xA0
#define PKT_TYPE_CMD_BEEP_ON           0xA1
#define PKT_TYPE_CMD_SHUTDOWN          0xA3

// Global variable for last CO2 reading
uint16_t lastCO2Value = 0;

// LED update interval and sensor measurement interval
const uint32_t LED_UPDATE_INTERVAL = 1000;   // (Not used separately now)
const unsigned long MEASUREMENT_INTERVAL = 5000; // 5 seconds
unsigned long lastMeasurementMillis = 0;

// NeoPixel LED strip configuration: 30 LEDs on pin 27 (using NEO_GRB)
#define LED_COUNT 30
#define LED_PIN 27
Adafruit_NeoPixel ledStrip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// This variable is no longer used for separate LED update
uint16_t previousLEDCount = 0;

// Function to send sensor data via PacketSerial (only sensor data)
void sensor_data_send(uint8_t type, float data) {
  uint8_t data_buf[32] = {0};
  int index = 0;
  data_buf[0] = type;
  index++;
  memcpy(&data_buf[1], &data, sizeof(float));
  index += sizeof(float);
  myPacketSerial.send(data_buf, index);
#if DEBUG
  Serial.printf("---> send len:%d, data: ", index);
  for (int i = 0; i < index; i++) {
    Serial.printf("0x%x ", data_buf[i]);
  }
  Serial.println("");
#endif
}

// Sensor power control functions
void sensor_power_on() {
  pinMode(18, OUTPUT);
  digitalWrite(18, HIGH);
}
void sensor_power_off() {
  pinMode(18, OUTPUT);
  digitalWrite(18, LOW);
}

// AHT20 sensor initialization and data reading
void sensor_aht_init() {
  AHT.begin();
}
void sensor_aht_get() {
  float humi, temp;
  int ret = AHT.getSensor(&humi, &temp);
  if (ret) {
    Serial.print("AHT20 - Humidity: ");
    Serial.print(humi * 100);
    Serial.print("%, Temperature: ");
    Serial.println(temp);
    temperature = temp;
    humidity = humi * 100;
    compensationT = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
    compensationRh = static_cast<uint16_t>(humidity * 65535 / 100);
    SDDataString += "aht20,";
    SDDataString += String(temp) + ',' + String(humi * 100) + ',';
    sensor_data_send(PKT_TYPE_SENSOR_SHT41_TEMP, temp);
    sensor_data_send(PKT_TYPE_SENSOR_SHT41_HUMIDITY, humi * 100);
  } else {
    Serial.println("GET DATA FROM AHT20 FAIL");
    SDDataString += "aht20,-,-,";
  }
}

// SGP40 sensor initialization and reading
void sensor_sgp40_init() {
  uint16_t error;
  char errorMessage[256];
  sgp40.begin(Wire);
  uint16_t serialNumber[3];
  uint8_t serialNumberSize = 3;
  error = sgp40.getSerialNumber(serialNumber, serialNumberSize);
  if (error) {
    Serial.print("SGP40 - Error in getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("SGP40 - SerialNumber: 0x");
    for (size_t i = 0; i < serialNumberSize; i++) {
      uint16_t value = serialNumber[i];
      Serial.print(value < 4096 ? "0" : "");
      Serial.print(value < 256 ? "0" : "");
      Serial.print(value < 16 ? "0" : "");
      Serial.print(value, HEX);
    }
    Serial.println();
  }
  uint16_t testResult;
  error = sgp40.executeSelfTest(testResult);
  if (error) {
    Serial.print("SGP40 - Error in executeSelfTest(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (testResult != 0xD400) {
    Serial.print("SGP40 - executeSelfTest failed with error: ");
    Serial.println(testResult);
  }
}
void sensor_sgp40_get() {
  uint16_t error;
  char errorMessage[256];
  uint16_t srawVoc = 0;
  Serial.print("SGP40 - ");
  // Use global compensation values.
  error = sgp40.measureRawSignal(compensationRh, compensationT, srawVoc);
  if (error) {
    Serial.print("Error: ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("SRAW_VOC: ");
    Serial.println(srawVoc);
  }
  SDDataString += "sgp40,";
  if (error) {
    SDDataString += "-,";
  } else {
    SDDataString += String(srawVoc) + ',';
    int32_t voc_index = voc_algorithm.process(srawVoc);
    Serial.print("VOC Index: ");
    Serial.println(voc_index);
    sensor_data_send(PKT_TYPE_SENSOR_TVOC_INDEX, (float)voc_index);
  }
}

// SCD4x sensor initialization and reading
void sensor_scd4x_init() {
  uint16_t error;
  char errorMessage[256];
  scd4x.begin(Wire, 0x62);
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    Serial.print("SCD4x - Error in stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
  uint64_t serial;
  error = scd4x.getSerialNumber(serial);
  if (error) {
    Serial.print("SCD4x - Error in getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("SCD4x - Serial: 0x");
    Serial.println(serial, HEX);
  }
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.print("SCD4x - Error in startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  }
}
void sensor_scd4x_get() {
  uint16_t error;
  char errorMessage[256];
  Serial.print("SCD4x - ");
  uint16_t co2;
  float temp;
  float hum;
  error = scd4x.readMeasurement(co2, temp, hum);
  if (error) {
    Serial.print("Error in readMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else if (co2 == 0) {
    Serial.println("Invalid sample detected, skipping.");
  } else {
    lastCO2Value = co2;
    Serial.print("CO2: ");
    Serial.print(co2);
    Serial.print(" ppm, Temperature: ");
    Serial.print(temp);
    Serial.print(" °C, Humidity: ");
    Serial.println(hum);
  }
  SDDataString += "scd4x,";
  if (error) {
    SDDataString += "-,-,-,";
  } else {
    SDDataString += String(co2) + ',' + String(temp) + ',' + String(hum) + ',';
    sensor_data_send(PKT_TYPE_SENSOR_SCD41_CO2, (float)co2);
  }
}

// LED update function: update LED strip based on CO2 level.
// The LED brightness is fixed at 3%.
void updateLEDDisplay(uint16_t co2) {
  const uint16_t minCO2 = 400;
  const uint16_t maxCO2 = MAX_CO2;
  int litLEDs = 0;
  if (co2 < minCO2) {
    litLEDs = 0;
  } else {
    litLEDs = map(co2, minCO2, maxCO2, 0, LED_COUNT);
    litLEDs = constrain(litLEDs, 0, LED_COUNT);
  }
  float ratio = 0.0;
  if (co2 < minCO2) {
    ratio = 0.0;
  } else if (co2 > maxCO2) {
    ratio = 1.0;
  } else {
    ratio = (float)(co2 - minCO2) / (maxCO2 - minCO2);
  }
  uint8_t red = ratio * 255;
  uint8_t green = (1 - ratio) * 255;
  uint8_t blue = 0;
  
  // Set fixed brightness of 3%
  ledStrip.setBrightness(LED_BRIGHTNESS);
  
  Serial.print("LED Update -> CO2: ");
  Serial.print(co2);
  Serial.print(" | LitLEDs: ");
  Serial.print(litLEDs);
  Serial.print(" | Ratio: ");
  Serial.print(ratio);
  Serial.print(" | Colors (R,G,B): ");
  Serial.print(red);
  Serial.print(", ");
  Serial.print(green);
  Serial.print(", ");
  Serial.println(blue);
  
  for (int i = 0; i < LED_COUNT; i++) {
    if (i < litLEDs) {
      ledStrip.setPixelColor(i, ledStrip.Color(red, green, blue));
    } else {
      ledStrip.setPixelColor(i, ledStrip.Color(0, 0, 0));
    }
  }
  ledStrip.show();
  
  Serial.println("Status: LED brightness fixed at 3%.");
}

// Buzzer functions
#define Buzzer 19
void beep_init(void) {
  pinMode(Buzzer, OUTPUT);
}
void beep_off(void) {
  digitalWrite(Buzzer, LOW);
}
void beep_on(void) {
  analogWrite(Buzzer, 127);
  delay(50);
  analogWrite(Buzzer, 0);
}

// Grove ADC reading from analog pin 26
void grove_adc_get(void) {
  String dataString = "";
  int adc0 = analogRead(26);
  dataString += String(adc0);
  Serial.print("Grove ADC: ");
  Serial.println(dataString);
}

static bool shutdown_flag = false;
void onPacketReceived(const uint8_t *buffer, size_t size) {
#if DEBUG
  Serial.printf("<--- recv len:%d, data: ", size);
  for (int i = 0; i < size; i++) {
    Serial.printf("0x%x ", buffer[i]);
  }
  Serial.println("");
#endif
  if (size < 1) return;
  switch (buffer[0]) {
    case PKT_TYPE_CMD_SHUTDOWN:
      Serial.println("CMD shutdown");
      shutdown_flag = true;
      sensor_power_off();
      break;
    default:
      break;
  }
}

int cnt = 0;
bool sd_init_flag = 0;

void setup() {
  Serial.begin(115200);
  
  Serial1.setRX(17);
  Serial1.setTX(16);
  Serial1.begin(115200);
  myPacketSerial.setStream(&Serial1);
  myPacketSerial.setPacketHandler(&onPacketReceived);
  
  sensor_power_on();
  
  Wire.setSDA(20);
  Wire.setSCL(21);
  Wire.begin();
  
  const int chipSelect = 13;
  SPI1.setSCK(10);
  SPI1.setTX(11);
  SPI1.setRX(12);
  if (!SD.begin(chipSelect, 1000000, SPI1)) {
    Serial.println("SD Card failed or not present");
    sd_init_flag = 0;
  } else {
    Serial.println("SD Card initialized.");
    sd_init_flag = 1;
  }
  
  sensor_aht_init();
  sensor_sgp40_init();
  sensor_scd4x_init();
  
  int32_t index_offset, learning_time_offset_hours, learning_time_gain_hours;
  int32_t gating_max_duration_minutes, std_initial, gain_factor;
  voc_algorithm.get_tuning_parameters(
    index_offset, learning_time_offset_hours, learning_time_gain_hours,
    gating_max_duration_minutes, std_initial, gain_factor);
  
  Serial.println("\nVOC Gas Index Algorithm parameters");
  Serial.print("Index offset:\t"); Serial.println(index_offset);
  Serial.print("Learning time offset hours:\t"); Serial.println(learning_time_offset_hours);
  Serial.print("Learning time gain hours:\t"); Serial.println(learning_time_gain_hours);
  Serial.print("Gating max duration minutes:\t"); Serial.println(gating_max_duration_minutes);
  Serial.print("Std initial:\t"); Serial.println(std_initial);
  Serial.print("Gain factor:\t"); Serial.println(gain_factor);
  
  beep_init();
  delay(500);
  beep_on();
  
  ledStrip.begin();
  // Startup test: flash LED strip red
  for (int j = 0; j < LED_COUNT; j++) {
    ledStrip.setPixelColor(j, ledStrip.Color(255, 0, 0));
  }
  ledStrip.show();
  delay(500);
  for (int j = 0; j < LED_COUNT; j++) {
    ledStrip.setPixelColor(j, ledStrip.Color(0, 0, 0));
  }
  ledStrip.show();
  
  Serial.printf(SENSECAP, VERSION);
  
  lastMeasurementMillis = millis();
}

void loop() {
  // Perform sensor measurements every 5 seconds
  if (millis() - lastMeasurementMillis >= MEASUREMENT_INTERVAL) {
    lastMeasurementMillis = millis();
    SDDataString = "";
    Serial.printf("\r\n\r\n--------- start measure %d -------\r\n", cnt);
    SDDataString += String(cnt) + ',';
    cnt++;
    
    sensor_aht_get();
    sensor_sgp40_get();
    sensor_scd4x_get();
    grove_adc_get();
    
    // Update LED display immediately with the new CO2 reading
    if (lastCO2Value > 0) {
      updateLEDDisplay(lastCO2Value);
    }
    
    if (sd_init_flag) {
      File dataFile = SD.open("datalog.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println(SDDataString);
        dataFile.close();
        Serial.print("SD write: ");
        Serial.println(SDDataString);
      } else {
        Serial.println("Error opening datalog.csv");
      }
    }
  }
  
  myPacketSerial.update();
  if (myPacketSerial.overflow()) {
    // Handle overflow if necessary
  }
  delay(10);
  
  while (shutdown_flag) {
    delay(10);
    for (int j = 0; j < LED_COUNT; j++) {
      ledStrip.setPixelColor(j, ledStrip.Color(0, 0, 0));
    }
    ledStrip.show();
  }
}
