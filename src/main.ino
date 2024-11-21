/* Connection to the MCP4725 DAC code for ESP32 using arduino framerwork */

#include <Arduino.h>
#include <MCP4725.h>
#include <esp_adc_cal.h>

esp_adc_cal_characteristics_t chars;

#define DAC_REF_VOLTAGE 3.300 //DAC reference voltage
#define I2C_BUS_SPEED   100000 //i2c bus speed, 100 000Hz or 400 000Hz

const int adc_pin = 34; 
int entrySignal = 0;
const int meanNumber = 10;

MCP4725 dac(MCP4725A0_ADDR_A00, DAC_REF_VOLTAGE);

// Read the signal from the ADC
void readSignal() {
  // Read the signal from the ADC
  auto v34 = adc1_get_raw(ADC1_CHANNEL_6); // 34
  Serial.print("Measured Signal (mV): ");
  Serial.println(esp_adc_cal_raw_to_voltage(v34, &chars));
}

// Voltage sweep fucntion for the MCP4725 DAC given start and end voltage
void voltageSweep(MCP4725 *dac, float startVoltage, float endVoltage, int steps, int delayTime) {
  float voltageStep = (endVoltage - startVoltage) / steps;
  float currentVoltage = startVoltage;
  for (int i = 0; i < steps; i++) {
    dac->setVoltage(currentVoltage);
    Serial.print("Voltage: ");
    Serial.println(currentVoltage);
    currentVoltage += voltageStep;
    delay(delayTime);
    readSignal();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(I2C_BUS_SPEED);

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &chars);

  adc1_config_width(chars.bit_width);
  adc1_config_channel_atten(ADC1_CHANNEL_6, chars.atten); // 34
  delay(1000);
}

void loop() {
  dac.begin();
  dac.setVoltage(0.0);
  voltageSweep(&dac, 0.0, 3, 1000, 1000);
  voltageSweep(&dac, 3, 0.0, 1000, 1000);

  delay(1000);
}