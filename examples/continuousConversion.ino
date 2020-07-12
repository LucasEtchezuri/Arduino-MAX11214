#include <Arduino.h>                
#include "MAX11214.h"

MAX11214 adc;

uint32_t res = 0;

void setup()
{
  Serial.begin(115200);

  adc.begin(14, 12, 13, 5, 19); // CLK, MISO, MOSI, CS, DREADY

  adc.setMode(CONVERSION_MODE_CONT);                
  adc.setDataFormat(DATA_FORMAT_BINARY);           
  adc.setMeasuremode(MEASURE_UNIPOLAR);             
  adc.setClocktype(CLOCK_INTERNAL);
  adc.setPgaEnable(false);
  adc.setPgaGain(PGA_GAIN_1);
  adc.setDataMode(DATA_MODE_24BITS);
  adc.setAnalogInputBuffer(false);

  // read configured registers
  Serial.print("Register 1 = ");
  Serial.println(adc.readRegister(REG_CTRL1), BIN);
  delay(1);
  Serial.print("Register 2 = ");
  Serial.println(adc.readRegister(REG_CTRL2), BIN);
  delay(1);
  Serial.print("Register 3 = ");
  Serial.println(adc.readRegister(REG_CTRL3), BIN);
  delay(1);
}

void loop()
{
  adc.startConversion(10); // SPS 0 to 15.  View table in datasheet

  while (1)
  {
    if (!adc.isDataReadyHard())
    {
      while (!adc.isDataReadyHard())
      {
        ;
      }
      res = adc.readADC();
      Serial.print("Value = ");
      Serial.println(res);
      delay(100);
    }
  }
}