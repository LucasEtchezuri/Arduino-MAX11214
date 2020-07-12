#include <Arduino.h>
#include "MAX11214.h"

MAX11214 adc;

void setup()
{
  Serial.begin(115200);

  adc.begin(14, 12, 13, 5, 19);

  adc.setMode(CONVERSION_MODE_CONT);
  adc.setDataFormat(DATA_FORMAT_BINARY);
  adc.setMeasuremode(MEASURE_UNIPOLAR);
  adc.setClocktype(CLOCK_INTERNAL);
  adc.setPgaEnable(false);
  adc.setPgaGain(PGA_GAIN_1);
  adc.setDataMode(DATA_MODE_24BITS);
  adc.setAnalogInputBuffer(false);
  adc.setSyncMode(SYNC_CONTINUOUS);
  adc.setModulatorSync(false);
  delay(1);

  // Read configurated regs

  Serial.print("Reg 1 = ");
  Serial.println(adc.readRegister(REG_CTRL1), BIN);
  delay(1);
  Serial.print("Reg 2 = ");
  Serial.println(adc.readRegister(REG_CTRL2), BIN);
  delay(1);
  Serial.print("Reg 3 = ");
  Serial.println(adc.readRegister(REG_CTRL3), BIN);
  delay(1);
}

void loop()
{
  uint32_t res = 0;
  unsigned long timeAnt = millis();
  unsigned int count = 0;

  adc.startConversion(15); // set max sample rate.  32 KSPS

  while (1)
  {

    if (adc.isDataReadyHard())
    {
      res = adc.readADC();
      count++;
    }

    if ((millis() - timeAnt) >= 1000)
    {
      Serial.print("SPS = ");
      Serial.println(count);
      count = 0;
      timeAnt = millis();
    }
  }
}