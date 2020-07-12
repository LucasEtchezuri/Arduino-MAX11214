#include <Arduino.h>
#include "MAX11214.h"

MAX11214 adc;

uint32_t res = 0;

void setup()
{
    Serial.begin(115200);

    adc.begin(14, 12, 13, 5, 19);

    adc.setMode(CONVERSION_MODE_SINGLE);
    adc.setDataFormat(DATA_FORMAT_BINARY);
    adc.setMeasuremode(MEASURE_UNIPOLAR);
    adc.setClocktype(CLOCK_INTERNAL);
    adc.setPgaEnable(false);
    adc.setPgaGain(PGA_GAIN_1);
    adc.setDataMode(DATA_MODE_24BITS);
    adc.setSyncMode(SYNC_CONTINUOUS);
    adc.setModulatorSync(false);

    // read configured registers
    Serial.print("Registro 1 = ");
    Serial.println(adc.readRegister(REG_CTRL1), BIN);
    delay(1);
    Serial.print("Registro 2 = ");
    Serial.println(adc.readRegister(REG_CTRL2), BIN);
    delay(1);
    Serial.print("Registro 3 = ");
    Serial.println(adc.readRegister(REG_CTRL3), BIN);
    delay(1);
}

void loop()
{
    while (1)
    {
        adc.startConversion(10);

        while (!adc.isDataReadyHard())
        {
            ;
        }
        res = adc.readADC();
        Serial.print("Value = ");
        Serial.println(res);
    }
}