/* Educational low-cost Potentiostat using the microcontroller ESP32 using arduino framework */

/* Include libs ---------------------------------------------------------------------------- */
#include <Arduino.h>
#include <MCP4725.h>
#include <esp_adc_cal.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <string.h>

/* Constants definitions -------------------------------------------------------------------- */
// BLE UUIDs
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define MESSAGE_CHARACTERISTIC_UUID "6d68efe5-04b6-4a85-abc4-c2670b7bf7fd"
#define parameters_CHARACTERISTIC_UUID "f27b53ad-c63d-49a0-8c0f-9f297e6cc520"

// PINs
// #define SDA_1 21
// #define SCL_1 22
#define SDA_2 19 // SDA pin for the OLED display
#define SCL_2 18 // SCL pin for the OLED display
#define ADC_PIN 34 // ADC pin
#define BATTERY_ADC_PIN 35 // ADC pin for the battery

// Other constants
#define DAC_REF_VOLTAGE 3.3 //DAC reference voltage
#define I2C_BUS_SPEED 100000 //i2c bus speed, 100 000Hz or 400 000Hz
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define BATTERY_MAX 12.6 // Maximum battery voltage
#define BATTERY_MIN 11.0 // Minimum battery voltage
#define BATTERY_VOLTAGE_DIVIDER 4.3 // Voltage divider for the battery
#define RESISTANCE_FOR_CURRENT 5080 // Resistance for the current calculation
#define voltage_divider_offset 1.071 // Voltage divider offset

/* Global variables declaration ---------------------------------------------------------- */

// BLE variables
BLEServer *pServer = NULL;
BLECharacteristic *message_characteristic = NULL; // For sending messages of the cyclic voltammetry
BLECharacteristic *parameters_characteristic = NULL; // For receiving parameters and start trigger of the cyclic voltammetry
esp_adc_cal_characteristics_t chars;
bool deviceConnected = false;
bool newParameters = false;

// I2C variables
// Multiple I2C channels
// TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1); // OLED display

// ADC variables
const int adc_pin = ADC_PIN;
const int battery_adc_pin = BATTERY_ADC_PIN;

// Run variables
int batteryPercentage = 100;
int batteryVoltageCumulative[20] = {0};
int batteryVoltageIndex = 0;
String parametersString = "";
float startVoltageSetting = 0.0;
float endVoltageSetting = 0.0;
int stepsSetting = 0;
int delaySetting = 0;
int shouldRunSetting = 0;
float referenceVoltageatBeginning = 0.0; // Reference voltage at the beginning of the cyclic voltammetry, to calculate the current with the measured voltage

// DAC initialization
MCP4725 dac(MCP4725A0_ADDR_A01, DAC_REF_VOLTAGE);

// OLED display initialization
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Ctwo, -1);

// State machine
enum state {
  WAIT_CONNECTION,
  WAIT_PARAMETERS,
  CYCLIC_VOLTAMMETRY
};
state currentState = WAIT_CONNECTION;

/* Functions declarations ------------------------------------------------------------------ */

// Since the DAC output is ruled by the formula V = 2 - V_desired
float convertVoltage(float voltage);

/* Classes definitions --------------------------------------------------------------------- */
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
      char* copy = strdup(parametersString.c_str());
      // Parse parametersString into startVoltageString, endVoltageString, stepsString, delayString, and shouldRunString 
      startVoltageSetting = strtof(strtok(copy, " "), NULL);
      endVoltageSetting = strtof(strtok(NULL, " "), NULL);
      stepsSetting = strtol(strtok(NULL, " "), NULL, 10);
      delaySetting = strtol(strtok(NULL, " "), NULL, 10);
      shouldRunSetting = strtol(strtok(NULL, " "), NULL, 10);

      // Convert the voltage to the DAC output
      startVoltageSetting = convertVoltage(startVoltageSetting);
      endVoltageSetting = convertVoltage(endVoltageSetting);

      free(copy);

      Serial.println("Parameters: ");
      Serial.println(startVoltageSetting);
      Serial.println(endVoltageSetting);
      Serial.println(stepsSetting);
      Serial.println(delaySetting);
      Serial.println(shouldRunSetting);
    }
  }
};

/* Functions definitions ------------------------------------------------------------------- */

// Since the DAC output is ruled by the formula V = 2 - V_desired
float convertVoltage(float voltage) {
  return 2 - voltage;
}

void calculateBatteryPercentage() {
  // Calculate battery percentage with an ADC reading
  auto batteryVoltage = adc1_get_raw(ADC1_CHANNEL_7); // 35
  // Update batteryVoltageCumulative
  batteryVoltageCumulative[batteryVoltageIndex++] = batteryVoltage;
  batteryVoltageIndex %= 20;
  // Calculate the average of the last 20 readings
  int sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += batteryVoltageCumulative[i];
  }
  batteryVoltage = sum / 20;
  batteryPercentage = map(esp_adc_cal_raw_to_voltage(batteryVoltage, &chars), BATTERY_MIN * 1000 / BATTERY_VOLTAGE_DIVIDER, BATTERY_MAX * 1000 / BATTERY_VOLTAGE_DIVIDER, 0, 100);
  if (batteryPercentage < 0) {
    batteryPercentage = 0;
  }
  if (batteryPercentage > 100) {
    batteryPercentage = 100;
  }

  // Round to the nearest 10
  batteryPercentage = (batteryPercentage + 5) / 10 * 10;
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
    case WAIT_PARAMETERS:
      display.print("Connected");
      break;
    case CYCLIC_VOLTAMMETRY:
      display.print("Cyclic Voltammetry");
      break;
    default:
      display.print("Unknown state");
      break;
  }
  // Battery status in header
  display.setCursor(0,30);
  display.print("Battery:" + String(batteryPercentage) + "%");
  // Message
  display.setCursor(0,40);
  display.print(message);
  display.display();
}

// Read the signal from the ADC
String readSignal() {
  // Read the signal from the ADC
  // auto v34 = adc1_get_raw(ADC1_CHANNEL_6); // 34
  auto reading = analogRead(ADC_PIN);
  auto v34 = -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
  if(reading < 1 || reading > 4095) v34 = 0;
  v34 *= 1000;
  Serial.print("Measured voltage (mV): ");
  Serial.println(v34);
  // Convert voltage to current
  float current = (v34 - referenceVoltageatBeginning) / RESISTANCE_FOR_CURRENT * 1000;
  Serial.print("Measured current(uA): ");
  Serial.println(current);
  String signalMessage = String(current, 3);
  return signalMessage;
}

// Voltage sweep function for the MCP4725 DAC given start and end voltage
void voltageSweep(MCP4725 *dac, float startVoltage, float endVoltage, int steps, int delayTime) {
  float voltageStep = (endVoltage - startVoltage) / steps;
  float currentVoltage = startVoltage;
  for (int i = 0; i < steps; i++) {
    dac->setVoltage(currentVoltage);
    Serial.print("Voltage: ");
    Serial.println(currentVoltage, 3);
    String voltageMessage = String(convertVoltage(currentVoltage), 3);
    currentVoltage += voltageStep;
    delay(delayTime);
    String readingMessage = readSignal();
    String output = voltageMessage + "\n" + readingMessage;

    printScreen(output.c_str());
    sendMessageBLE(message_characteristic, output.c_str());
  }
}

/* Main code ------------------------------------------------------------------------------- */
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
      parameters_characteristic->setCallbacks(new CharacteristicsCallbacks());
      if(deviceConnected) {
        currentState = WAIT_PARAMETERS;  // Once connected, move to the next state
      }
      break;

    case WAIT_PARAMETERS:
      printScreen("Set Parameters and run.");
      if(shouldRunSetting)
        currentState = CYCLIC_VOLTAMMETRY;
      break;

    case CYCLIC_VOLTAMMETRY:
      // Calculate the reference voltage at the beginning three times
      referenceVoltageatBeginning = 0.0;
      for (int i = 0; i < 3; i++) {
        auto reading = analogRead(ADC_PIN);
        auto v34 = -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
        if(reading < 1 || reading > 4095) v34 = 0;
        referenceVoltageatBeginning += v34;
      }
      referenceVoltageatBeginning /= 3;
      referenceVoltageatBeginning *= 1000;
      // Perform the cyclic voltammetry (voltage sweep)
      printScreen("Cyclic Voltammetry");
      // Make sure voltage sweep does not exceed max voltage, nor is negative
      if(startVoltageSetting < 0) {
        startVoltageSetting = 0;
      }
      if(endVoltageSetting < 0) {
        endVoltageSetting = 0;
      }
      if(startVoltageSetting > 4.0) {
        startVoltageSetting = 4.0;
      }
      if(endVoltageSetting > 4.0) {
        endVoltageSetting = 4.0;
      }
      if(stepsSetting < 0) {
        stepsSetting = 0;
      }
      if(delaySetting < 0) {
        delaySetting = 0;
      }
      voltageSweep(&dac, startVoltageSetting, endVoltageSetting, stepsSetting, delaySetting);  // Sweep from 0 to 3V
      voltageSweep(&dac, endVoltageSetting, startVoltageSetting, stepsSetting, delaySetting);  // Sweep from 3V to 0

      shouldRunSetting = 0;
      currentState = WAIT_CONNECTION;  // After voltammetry, go to send data state
      break;

    default:
      currentState = WAIT_CONNECTION;  // Default to WAIT_CONNECTION in case of error
      break;
  }
}