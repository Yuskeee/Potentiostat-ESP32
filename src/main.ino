/* Connection to the MCP4725 DAC code for ESP32 using arduino framerwork */

#include <Arduino.h>
#include <MCP4725.h>
#include <esp_adc_cal.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

esp_adc_cal_characteristics_t chars;

#define SDA_1 21
#define SCL_1 22

#define SDA_2 18
#define SCL_2 19

// Multiple I2C channels
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

#define DAC_REF_VOLTAGE 3.300 //DAC reference voltage
#define I2C_BUS_SPEED   100000 //i2c bus speed, 100 000Hz or 400 000Hz
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

const int adc_pin = 34;

MCP4725 dac(0x60, &I2Cone);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2Ctwo, -1);

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
  I2Cone.begin(SDA_1, SCL_1, I2C_BUS_SPEED); 
  I2Ctwo.begin(SDA_2, SCL_2, I2C_BUS_SPEED);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  dac.begin();
  dac.setMaxVoltage(DAC_REF_VOLTAGE);

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &chars);

  adc1_config_width(chars.bit_width);
  adc1_config_channel_atten(ADC1_CHANNEL_6, chars.atten); // 34
  
  delay(1000);

  display.clearDisplay();
  Serial.println("Finished Setup.");
}

void loop() {
  dac.setVoltage(0.0);
  voltageSweep(&dac, 0.0, 3, 3000, 200);
  voltageSweep(&dac, 3, 0.0, 3000, 200);

  display.clearDisplay();
  // display temperature
  display.setTextColor(WHITE);
  display.setTextSize(3);
  display.setCursor(0,0);
  display.print("STATUS: ");
  display.setTextSize(2);
  display.setCursor(0,10);

  display.display();
  delay(1000);
}
