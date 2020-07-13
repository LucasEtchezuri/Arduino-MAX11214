#include "Arduino.h"
#include "MAX11214.h"
#include "SPI.h"

#define settings SPISettings(5000000, MSBFIRST, SPI_MODE0)

MAX11214::MAX11214()
{
}

void MAX11214::writeRegister(uint8_t address, uint8_t value)
{
  digitalWrite(MAX11214_CS_PIN, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMD_WRITE_REGISTER | (address << 1));
  SPI.transfer(value);
  delayMicroseconds(1);
  digitalWrite(MAX11214_CS_PIN, HIGH);
}

uint8_t MAX11214::readRegister(uint8_t address)
{
  digitalWrite(MAX11214_CS_PIN, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMD_READ_REGISTER | (address << 1));
  uint8_t data = SPI.transfer(SPI_MASTER_DUMMY);
  delayMicroseconds(1);
  digitalWrite(MAX11214_CS_PIN, HIGH);
  return data;
}

uint32_t MAX11214::readRegister24(uint8_t address)
{
  digitalWrite(MAX11214_CS_PIN, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMD_READ_REGISTER | (address << 1));
  uint8_t data = SPI.transfer(SPI_MASTER_DUMMY);
  uint32_t val = data << 16;
  data = SPI.transfer(SPI_MASTER_DUMMY);
  uint32_t val2 = data << 8;
  uint32_t val3 = SPI.transfer(SPI_MASTER_DUMMY);
  delayMicroseconds(1);
  digitalWrite(MAX11214_CS_PIN, HIGH);
  return (val | val2 | val3);
  return 0;
}

uint32_t MAX11214::readRegister32(uint8_t address)
{
  digitalWrite(MAX11214_CS_PIN, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMD_READ_REGISTER | (address << 1));
  uint32_t val = SPI.transfer32(SPI_MASTER_DUMMY32);
  delayMicroseconds(1);
  digitalWrite(MAX11214_CS_PIN, HIGH);
  return val;
}

void MAX11214::begin(uint8_t clk_pin, uint8_t miso_pin, uint8_t mosi_pin, uint8_t cs_pin, uint8_t drdy_pin)
{
  // Set pins up
  MAX11214_CS_PIN = cs_pin;
  MAX11214_DRDY_PIN = drdy_pin;
  MAX11214_CLK_PIN = clk_pin;
  MAX11214_MISO_PIN = miso_pin;
  MAX11214_MOSI_PIN = mosi_pin;

  SPI.begin(MAX11214_CLK_PIN, MAX11214_MISO_PIN, MAX11214_MOSI_PIN);
  SPI.beginTransaction(settings);
  // Configure chip select as an output
  pinMode(MAX11214_CS_PIN, OUTPUT);
  // Configure DRDY as as input
  pinMode(MAX11214_DRDY_PIN, INPUT);
}

bool MAX11214::isDataReadyHard()
{
  if (digitalRead(MAX11214_DRDY_PIN) == HIGH)
  {
    return false;
  }
  return true;
}

void MAX11214::writeRegisterMasked(uint8_t address, uint8_t value, uint8_t mask)
{
  // Escribe el valor el registro, aplicando la mascara para tocar unicamente los bits necesarios.
  // No realiza el corrimiento de bits (shift), hay que pasarle ya el valor corrido a la posicion correcta

  // Leo el contenido actual del registro
  uint8_t register_contents = readRegister(address);

  // con el ~ Cambio bit a bit la mascara (queda 1 en los bits que no hay que tocar y 0 en los bits a modificar)
  // Se realiza un AND con el contenido actual del registro.  Quedan "0" en la parte a modificar
  register_contents = register_contents & ~mask;

  // se realiza un OR con el valor a cargar en el registro.  Ojo, valor debe estar en el posicion (shitf) correcta
  register_contents = register_contents | value;

  // Escribo nuevamente el registro
  writeRegister(address, register_contents);
}

bool MAX11214::isDataReadySoft()
{
  digitalWrite(MAX11214_CS_PIN, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMD_READ_REGISTER | (REG_STAT << 1));
  uint16_t data = SPI.transfer16(SPI_MASTER_DUMMY);
  delayMicroseconds(1);
  digitalWrite(MAX11214_CS_PIN, HIGH);

  uint16_t x = (0x0001 & data);
  if (x != 1)
  {
    return false;
  }
  return true;
}

bool MAX11214::conversionInProgress()
{
  digitalWrite(MAX11214_CS_PIN, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMD_READ_REGISTER | (REG_STAT << 1));
  uint16_t data = SPI.transfer16(SPI_MASTER_DUMMY);
  delayMicroseconds(1);
  digitalWrite(MAX11214_CS_PIN, HIGH);

  uint16_t x = (0x0002 & data);
  if (x > 0)
  {
    return true;
  }
  return false;
}

bool MAX11214::isPowerDown()
{
  digitalWrite(MAX11214_CS_PIN, LOW);
  delayMicroseconds(1);
  SPI.transfer(CMD_READ_REGISTER | (REG_STAT << 1));
  uint16_t data = SPI.transfer16(SPI_MASTER_DUMMY);
  delayMicroseconds(1);
  digitalWrite(MAX11214_CS_PIN, HIGH);

  uint16_t x = (0x0800 & data);
  if (x > 0)
  {
    return true;
  }
  return false;
}

bool MAX11214::setMode(uint8_t mode)
{
  if (mode == CONVERSION_MODE_CONT)
  {
    writeRegisterMasked(REG_CTRL1, B00000001, REG_MASK_MODE);
    return true;
  }
  else if (mode == CONVERSION_MODE_SINGLE)
  {
    writeRegisterMasked(REG_CTRL1, B00000010, REG_MASK_MODE);
    return true;
  }
  return false;
}

bool MAX11214::setDataFormat(uint8_t dataFormat)
{

  if (dataFormat == DATA_FORMAT_BINARY)
  {
    writeRegisterMasked(REG_CTRL1, B00000100, REG_MASK_DATA_FORMAT);
    return true;
  }
  else if (dataFormat == DATA_FORMAT_TWO_COMPLEMENT)
  {
    writeRegisterMasked(REG_CTRL1, B00000000, REG_MASK_DATA_FORMAT);
    return true;
  }
  return false;
}

bool MAX11214::setMeasuremode(uint8_t measureMode)
{

  if (measureMode == MEASURE_UNIPOLAR)
  {
    writeRegisterMasked(REG_CTRL1, B00001000, REG_MASK_MEASURE_MODE);
    return true;
  }
  else if (measureMode == MEASURE_BIPOLAR)
  {
    writeRegisterMasked(REG_CTRL1, B00000000, REG_MASK_MEASURE_MODE);
    return true;
  }
  return false;
}

bool MAX11214::setClocktype(uint8_t clockType)
{

  if (clockType == CLOCK_EXTERNAL)
  {
    writeRegisterMasked(REG_CTRL1, B10000000, REG_MASK_CLOCK_TYPE);
    return true;
  }
  else if (clockType == CLOCK_INTERNAL)
  {
    writeRegisterMasked(REG_CTRL1, B00000000, REG_MASK_CLOCK_TYPE);
    return true;
  }
  return false;
}

bool MAX11214::setPgaEnable(bool pgaEnable)
{
  if (pgaEnable)
  {
    writeRegisterMasked(REG_CTRL2, B00001000, REG_MASK_PGA_ENABLE);
  }
  else
  {
    writeRegisterMasked(REG_CTRL2, B00000000, REG_MASK_PGA_ENABLE);
  }
  return true;
}

bool MAX11214::setPgaGain(uint8_t pgaGain)
{
  if ((0 <= pgaGain) and (pgaGain < 8))
  {
    writeRegisterMasked(REG_CTRL2, pgaGain, REG_MASK_PGA_GAIN);
    return true;
  }
  return false;
}

bool MAX11214::setFilter(uint8_t filter)
{

  if (filter == FILTER_SINC)
  {
    writeRegisterMasked(REG_CTRL3, B00000000, REG_MASK_FILTER);
    return true;
  }
  else if (filter == FILTER_FIR)
  {
    writeRegisterMasked(REG_CTRL3, B00000010, REG_MASK_FILTER);
    return true;
  }
  else if (filter == FILTER_FIR_IIR)
  {
    writeRegisterMasked(REG_CTRL3, B00000011, REG_MASK_FILTER);
    return true;
  }
  return false;
}

bool MAX11214::setDataMode(uint8_t dataMode)
{

  if (dataMode == DATA_MODE_24BITS)
  {
    writeRegisterMasked(REG_CTRL3, B00000000, REG_MASK_DATA_MODE);
    return true;
  }
  else if (dataMode == DATA_MODE_32BITS)
  {
    writeRegisterMasked(REG_CTRL3, B00001000, REG_MASK_DATA_MODE);
    return true;
  }

  return false;
}

uint8_t MAX11214::getDataMode()
{
  uint8_t dataMode = readRegister(REG_CTRL3) & REG_MASK_DATA_MODE;

  if (dataMode == DATA_MODE_24BITS)
  {
    return DATA_MODE_24BITS;
  }
  return DATA_MODE_32BITS;
}

bool MAX11214::startConversion(uint8_t dataRate)
{
  if ((dataRate >= 0) and (dataRate < 16))
  {
    digitalWrite(MAX11214_CS_PIN, LOW);
    delayMicroseconds(1);
    SPI.transfer(CMD_CONVERSION | dataRate);
    delayMicroseconds(1);
    digitalWrite(MAX11214_CS_PIN, HIGH);
    return true;
  }
  return false;
}

uint32_t MAX11214::readADC(void)
{
  return (readRegister24(REG_DATA));
}

uint32_t MAX11214::readADC32(void)
{
  return (readRegister32(REG_DATA));
}

bool MAX11214::setAnalogInputBuffer(bool inputBuffer)
{
  if (inputBuffer)
  {
    writeRegisterMasked(REG_CTRL2, B00100000, REG_MASK_INPUT_BUFFER);
  }
  else
  {
    writeRegisterMasked(REG_CTRL2, B00000000, REG_MASK_INPUT_BUFFER);
  }
  return true;
}

bool MAX11214::setSyncMode(uint8_t syncMode)
{
  if (syncMode == SYNC_CONTINUOUS)
  {
    writeRegisterMasked(REG_CTRL1, B01000000, REG_MASK_SYNC_MODE);
  }
  else
  {
    writeRegisterMasked(REG_CTRL1, B00000000, REG_MASK_SYNC_MODE);
  }
  return true;
}

bool MAX11214::setModulatorSync(bool Modsync)
{
  if (Modsync)
  {
    writeRegisterMasked(REG_CTRL3, B00100000, REG_MASK_MOD_SYNC);
  }
  else
  {
    writeRegisterMasked(REG_CTRL3, B00000000, REG_MASK_MOD_SYNC);
  }
  return true;
}

bool MAX11214::setConfDIO(uint8_t dio, uint8_t mode)
{
  if (dio == 1)
  {
    if (mode == DIO_OUTPUT)
      writeRegisterMasked(REG_CTRL4, B00010000, REG_MASK_CONF_DIO1);
    else
      writeRegisterMasked(REG_CTRL4, B00000000, REG_MASK_CONF_DIO1);
  }

  if (dio == 2)
  {
    if (mode == DIO_OUTPUT)
      writeRegisterMasked(REG_CTRL4, B00100000, REG_MASK_CONF_DIO2);
    else
      writeRegisterMasked(REG_CTRL4, B00000000, REG_MASK_CONF_DIO2);
  }

  if (dio == 3)
  {
    if (mode == DIO_OUTPUT)
      writeRegisterMasked(REG_CTRL4, B01000000, REG_MASK_CONF_DIO3);
    else
      writeRegisterMasked(REG_CTRL4, B00000000, REG_MASK_CONF_DIO3);
  }
  return true;
}

bool MAX11214::writeDIO(uint8_t dio, bool data)
{
  if (dio == 1)
  {
    if (data == true)
      writeRegisterMasked(REG_CTRL4, B00000001, REG_MASK_DIO1);
    else
      writeRegisterMasked(REG_CTRL4, B00000000, REG_MASK_DIO1);
  }

  if (dio == 2)
  {
    if (data == true)
      writeRegisterMasked(REG_CTRL4, B00000010, REG_MASK_CONF_DIO2);
    else
      writeRegisterMasked(REG_CTRL4, B00000000, REG_MASK_CONF_DIO2);
  }

  if (dio == 3)
  {
    if (data == true)
      writeRegisterMasked(REG_CTRL4, B00000100, REG_MASK_CONF_DIO3);
    else
      writeRegisterMasked(REG_CTRL4, B00000000, REG_MASK_CONF_DIO3);
  }
  return true;
}

bool MAX11214::readDIO(uint8_t dio)
{
  uint8_t reg = readRegister(REG_CTRL4);
  uint8_t res;

  if (dio == 1)
    res = (reg & B00000001);
  if (dio == 2)
    res = (reg & B00000010);
  if (dio == 3)
    res = (reg & B00000100);
  if (res == 0)
    return 0;
  else
    return 1;
}
