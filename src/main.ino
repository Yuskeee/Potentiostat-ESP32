/* Educational low-cost Potentiostat using the microcontroller ESP32 using arduino framework */

#include <Arduino.h>
#include <MCP4725.h>
#include <esp_adc_cal.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <string.h>

// BLE SECTION
BLEServer *pServer = NULL;

BLECharacteristic *message_characteristic = NULL;
BLECharacteristic *parameters_characteristic = NULL;

bool deviceConnected = false;

esp_adc_cal_characteristics_t chars;

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"

#define MESSAGE_CHARACTERISTIC_UUID "6d68efe5-04b6-4a85-abc4-c2670b7bf7fd"
#define parameters_CHARACTERISTIC_UUID "f27b53ad-c63d-49a0-8c0f-9f297e6cc520"

// #define SDA_1 21
// #define SCL_1 22

#define SDA_2 18
#define SCL_2 19

// Multiple I2C channels
// TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

#define DAC_REF_VOLTAGE 3.3 //DAC reference voltage
#define I2C_BUS_SPEED   100000 //i2c bus speed, 100 000Hz or 400 000Hz
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define BATTERY_MAX 12.6 // Maximum battery voltage
#define BATTERY_MIN 10.5 // Minimum battery voltage

const int adc_pin = 34;
const int battery_adc_pin = 35;

int batteryPercentage = 100;
const char* parametersString = "";
float startVoltage = 0.0;
float endVoltage = 0.0;
int stepsString = 0;
int delayString = 0;
int shouldRunString = 0;

enum state {
  WAIT_CONNECTION,
  CYCLIC_VOLTAMMETRY,
  SEND_DATA
};

state currentState = WAIT_CONNECTION;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("Connected");
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("Disconnected");
  }
};

class CharacteristicsCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    Serial.print("Value Written ");
    Serial.println(pCharacteristic->getValue().c_str());

    if (pCharacteristic == parameters_characteristic)
    {
      parametersString = pCharacteristic->getValue().c_str();
      // Parse parametersString into startVoltageString, endVoltageString, stepsString, delayString, and shouldRunString 
      startVoltage = strtof(strtok((char*)parametersString, " "), NULL);
      endVoltage = strtof(strtok((char*)parametersString, " "), NULL);
      stepsString = strtol(strtok((char*)parametersString, " "), NULL, 10);
      delayString = strtol(strtok((char*)parametersString, " "), NULL, 10);
      shouldRunString = stepsString = strtol(strtok((char*)parametersString, " "), NULL, 10);

      Serial.println("Parameters: ");
      Serial.println(startVoltage);
      Serial.println(endVoltage);
      Serial.println(stepsString);
      Serial.println(delayString);
      Serial.println(shouldRunString);
    }
  }
};

MCP4725 dac(MCP4725A0_ADDR_A00, DAC_REF_VOLTAGE);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Ctwo, -1);

void calculateBatteryPercentage() {
  // Calculate battery percentage with an ADC reading
  auto batteryVoltage = adc1_get_raw(ADC1_CHANNEL_7); // 35
  batteryPercentage = map(esp_adc_cal_raw_to_voltage(batteryVoltage, &chars), BATTERY_MIN * 1000, BATTERY_MAX * 1000, 0, 100);
  if (batteryPercentage < 0) {
    batteryPercentage = 0;
  }
  if (batteryPercentage > 100) {
    batteryPercentage = 100;
  }
  return;
}

void sendMessageBLE(BLECharacteristic *characteristic, String message) {
  characteristic->setValue(const_cast<char *>(message.c_str()));
  characteristic->notify();
}

// Prints header of screen and also fits the message parameter below
void printScreen(const char* message) {
  calculateBatteryPercentage();
  display.clearDisplay();
  // Header
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  // Current state
  switch(currentState) {
    case WAIT_CONNECTION:
      display.print("Connection...");
      break;
    case CYCLIC_VOLTAMMETRY:
      display.print("Cyclic Voltammetry");
      break;
    case SEND_DATA:
      display.print("Sending data...");
      break;
    default:
      display.print("Unknown state");
      break;
  }
  // Battery status in header
  display.setCursor(0,10);
  display.print("Battery:" + String(batteryPercentage) + "%");
  // Message
  display.setCursor(0,20);
  display.print(message);
  display.display();
}

// Read the signal from the ADC
String readSignal() {
  // Read the signal from the ADC
  auto v34 = adc1_get_raw(ADC1_CHANNEL_6); // 34
  Serial.print("Measured Signal (mV): ");
  Serial.println(esp_adc_cal_raw_to_voltage(v34, &chars));
  String signalMessage = "Measured Signal (mV): " + String(esp_adc_cal_raw_to_voltage(v34, &chars));
  return signalMessage;
}

// Voltage sweep fucntion for the MCP4725 DAC given start and end voltage
void voltageSweep(MCP4725 *dac, float startVoltage, float endVoltage, int steps, int delayTime) {
  float voltageStep = (endVoltage - startVoltage) / steps;
  float currentVoltage = startVoltage;
  for (int i = 0; i < steps; i++) {
    dac->setVoltage(currentVoltage);
    Serial.print("Voltage: ");
    Serial.println(currentVoltage, 3);
    String voltageMessage = "Voltage: " + String(currentVoltage, 3);
    currentVoltage += voltageStep;
    delay(delayTime);
    String readingMessage = readSignal();
    String output = voltageMessage + "\n" + readingMessage;

    printScreen(output.c_str());
    sendMessageBLE(message_characteristic, output.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(I2C_BUS_SPEED);
  // I2Cone.begin(SDA_1, SCL_1, I2C_BUS_SPEED); 
  I2Ctwo.begin(SDA_2, SCL_2, I2C_BUS_SPEED);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  while (dac.begin() != true)
  {
    Serial.println(F("MCP4725 is not connected")); //(F()) saves string to flash & keeps dynamic memory free
    delay(5000);
  }

  dac.begin();

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &chars);

  adc1_config_width(chars.bit_width);
  adc1_config_channel_atten(ADC1_CHANNEL_6, chars.atten); // 34
  adc1_config_channel_atten(ADC1_CHANNEL_7, chars.atten); // 35
  
  delay(1000);

  // Create the BLE Device
  BLEDevice::init("Edustat");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  delay(100);

  // Create a BLE Characteristic
  message_characteristic = pService->createCharacteristic(
      MESSAGE_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  parameters_characteristic = pService->createCharacteristic(
      parameters_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_NOTIFY |
          BLECharacteristic::PROPERTY_INDICATE);

  // Start the BLE service
  pService->start();

  display.clearDisplay();
  Serial.println("Finished Setup.");
}

void loop() {
  // Check the current state and run the appropriate functionality
  switch(currentState) {
    case WAIT_CONNECTION:
      // In this state, we could check if the device is connected or initialized
      printScreen("Waiting for connection...");
      // Start advertising
      pServer->getAdvertising()->start();
      delay(1000);
      if(deviceConnected) {
        currentState = CYCLIC_VOLTAMMETRY;  // Once connected, move to the next state
      }
      break;

    case CYCLIC_VOLTAMMETRY:
      // Perform the cyclic voltammetry (voltage sweep)
      printScreen("Cyclic Voltammetry");
      voltageSweep(&dac, 0.0, 3.0, 1000, 200);  // Sweep from 0 to 3V
      voltageSweep(&dac, 3.0, 0.0, 1000, 200);  // Sweep from 3V to 0

      currentState = SEND_DATA;  // After voltammetry, go to send data state
      break;

    case SEND_DATA:
      // Send data (e.g., save it, transmit it, or process it)
      printScreen("Sending data...");
      delay(1000);  // Simulate data sending
      
      // After sending data, return to wait connection or repeat the process
      currentState = WAIT_CONNECTION;  // Reset to waiting for next cycle
      break;

    default:
      currentState = WAIT_CONNECTION;  // Default to WAIT_CONNECTION in case of error
      break;
  }
}