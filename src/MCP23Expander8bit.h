#pragma once

// MCP23x08/MCP23x09 8-Bit GPIO Expander with I2C or SPI Interface
// Copyright (C) MattLabs and muman.ch, 2025.08.08
// All rights reversed
/*
For more details see the blog:
https://muman.ch/muman/index.htm?muman-mcp23017.htm

USAGE
-----
#include <Wire.h>

#define MCP23EXPANDER8BIT_I2C	// indicate it's the I2C version
#include "MCP23Expander8bit.h"
MCP23Expander8bitI2C mcp23008;

void setup()
{
	Wire.begin();
	Wire.setClock(1000000);
	Wire.setTimeout(100);

	mcp23008.begin(&Wire, 0x20);
	// configure all gpios as outputs
	mcp23008.configureGpios(0x00, 0x00, 0x00);

	mcp23008.dumpRegisters();
}
...

DIFFERENCES BETWEEN THE 23x08 and 23x09 CHIPS
---------------------------------------------
								 23008  23S08   23009  23S09
Open drain outputs                 N      N       Y      Y
I2C with ADDR pin addressing (1)   N      N       Y      N
I2C with A2..A0 addressing         Y      N       N      N
SPI with /CS pin addressing        N      Y       N      Y
SPI with A1..A0 addressing (2)     N      Y       N      N
Interrupt clearing control (3)     N      N       Y      Y
DISSLW slew rate control           Y      Y       N      N

(1)  I2C address bits A2..A0 are selected by a voltage divider.
(2)  The first byte of the SPI message contains the device address
	 bits A1..A0, allowing up to 4 x SPI devices with one /CS.
	 This is enabled by the HAEN bit, see enableHardwareAddressing().
	 The 23S009 has only one address, 0x20.
(3)  An interrupt is cleared by reading either the GPIO or the
	 INTCAP register. On the 23x08, reading either register clears
	 the interrupt.

DATA SHEETS
-----------
https://ww1.microchip.com/downloads/en/DeviceDoc/MCP23008-MCP23S08-Data-Sheet-20001919F.pdf
https://ww1.microchip.com/downloads/en/DeviceDoc/20002121C.pdf
*/

// Define one of these for the I2C or SPI version
#if defined(MCP23EXPANDER8BIT_SPI)
#include <SPI.h>
#elif defined(MCP23EXPANDER8BIT_I2C)
#include <Wire.h>
#else
#error MCP23EXPANDER8BIT_SPI or MCP23EXPANDER8BIT_I2C not defined before #include "MCP23Expander8bit.h"
#endif


// Base class for 8-bit expander SPI and I2C classes
class MCP23Expander8bitBase
{
protected:
	// For SPI 23S008 IOCON:HAEN bit, set by enableHardwareAddressing()
	static bool spiHardwareAddressingEnabled;

public:
	// Input mask for GPIO port, 1=the bit is an input, 0=output
	byte iodir = 0xff;

	// Register numbers
	enum REGS : byte
	{
		IODIR = 0x00,     // I/O direction: 1=input, 0=output
		IPOL = 0x01,     // Input polarity: 1=inverted, 0=not inverted
		GPINTEN = 0x02,     // Interrupt-on-change enable: 1=enable interrupt, 0=no interrupt
		DEFVAL = 0x03,     // Default compare register for interrupt, interrupt occurs if bit does NOT match
		INTCON = 0x04,     // Interrupt-on-change control: 1=pin state compared with DEFVAL bit, 0=not compared
		IOCON = 0x05,     // Chip configuration, see IOCON enum for bits, shared by both Ports
		GPPU = 0x06,     // Pull-up resistor configuration: 1=pull-up enabled, 0=pull-up disabled
		INTF = 0x07,     // Interrupt flag, which input caused the interrupt: 1=pin caused interrupt
		INTCAP = 0x08,     // Interrupt capture, port value at time of interrupt
		GPIO = 0x09,     // Port value, inputs and outputs
		OLAT = 0x0A      // Output latch, read/write output latches
	};

	// IOCON configuration register bits
	enum IOCON : byte
	{
		INTCC = 0x01,     // Interrupt clearing control, 23009 only
		INTPOL = 0x02,     // Polarity of INT pin: 0=active low, 1=active high
		ODR = 0x04,     // Open drain INT pin: 0=driven, 1=open drain (overrides INTPOL)
		HAEN = 0x08,     // Hardware address enable, 23S08 SPI devices only
		DISSLW = 0x10,     // SDA slew rate control, 23008 only: 0=enabled, 1=disabled
		SEQOP = 0x20      // Sequential address mode: 0=address increments, 1=address unchanged
	};

	bool setDefaults();
	bool configureInterruptPin(bool polarity, bool openDrain, bool intClear = false);
	bool configureInterrupt(byte gpinten, byte defval, byte intcon);
	bool configureGpios(byte iodir, byte ipol, byte gppu);
	bool readInterrupt(byte* intf, byte* intcap);
	void digitalWrite(byte pin, bool value);
	bool digitalRead(byte pin);
	bool readGpios(byte* value);
	bool readOutputs(byte* value);
	bool writeOutputs(byte value);
	bool readRegister(byte reg, byte* value);

	virtual bool isConnected() = 0;
	virtual bool readRegisters(byte reg, byte* value, int length) = 0;
	virtual bool writeRegister(byte reg, byte value) = 0;

	#ifdef DEBUG
	bool verify(byte reg, byte value);
	void dumpRegisters();
	void dumpRegister(byte reg, char* name, byte value);
	#endif
};

// for SPI's IOCON:HAEN bit, see SPI's enableHardwareAddressing()
// this is shared by all instances of this class
bool MCP23Expander8bitBase::spiHardwareAddressingEnabled = false;


// Set registers as they would be after a power-on reset (POR)
// Except for the SPI IOCON:HAEN bit which is controlled by SPI's
// enableHardwareAddressing().
// INT pin active low; INT pin driven; slew rate enabled; address increments;
// INTA/INTB separate; one-bank interleaved register addressing;
// HAEN according to enableHardwareAddressing.
// https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf#page=21
bool MCP23Expander8bitBase::setDefaults()
{
	// set IODIR to 0xff (all pins of both ports := inputs)
	if (!writeRegister(IODIR, 0xff))
		return false;
	iodir = 0xff;

	// IOCON register
	// for SPI 23S008, keep HAEN set if enableHardwareAddressing() was called
	if (!writeRegister(IOCON, spiHardwareAddressingEnabled ? HAEN : 0))
		return false;

	// set all other writable registers to 0
	for (byte reg = IPOL; reg <= OLAT; ++reg) {
		// skip read-only registers and IOCON
		switch (reg) {
		case INTF:
		case INTCAP:
		case IOCON:
			continue;
		}
		if (!writeRegister(reg, 0))
			return false;
	}
	return true;
}

// Configure the INT pin
// polarity = polarity of INT output pin: 0=active low, 1=active high
// openDrain = open drain INT pin: 0=driven, 1=open drain (overrides polarity)
// intClear = 23x09 only, interrupt clearing control INTCC: 
//            0=read GPIO register clears interrupt, 
//            1=read INTCAP register clears, 23x09 ONLY
bool MCP23Expander8bitBase::configureInterruptPin(bool polarity, bool openDrain,
	bool intClear/*=false*/)
{
	byte iocon = 0;

	// SPI 23S08 only, see enableHardwareAddressing()
	// HAEN=1 (SPI hardware address enable A0..A3)
	if (spiHardwareAddressingEnabled)
		iocon |= HAEN;

	if (polarity)
		iocon |= INTPOL;
	if (openDrain)
		iocon |= ODR;
	if (intClear)
		iocon |= INTCC;

	return writeRegister(IOCON, iocon);
}

// Configure interrupt
// gpinten = interrupt-on-change pin enable: 1=interrupt enabled, 0=interrupt disabled
// defval  = default value register: different bit causes an interrupt, enabled by intcon bit
// intcon  = interrupt control register: 1=bits compared with DEFVAL register, 0=not compared
bool MCP23Expander8bitBase::configureInterrupt(byte gpinten, byte defval, byte intcon)
{
	return writeRegister(GPINTEN, gpinten) &&
		writeRegister(DEFVAL, defval) &&
		writeRegister(INTCON, intcon);
}

// Configure a port's inputs/outputs
// iodir = direction: 1=input, 0=output
// ipol  = input polarity: 1=inverted, 0=not inverted
// gppu  = input pull-up: 1=pull-up enabled, 0=pull-up disabled
bool MCP23Expander8bitBase::configureGpios(byte iodir, byte ipol, byte gppu)
{
	// save input mask, 1=input
	this->iodir = iodir;

	return writeRegister(IODIR, iodir) &&
		writeRegister(IPOL, ipol) &&
		writeRegister(GPPU, gppu);
}

// Read interrupt details
// "The INTCAP register remains unchanged until the interrupt is cleared by 
// a read of INTCAP or GPIO."
bool MCP23Expander8bitBase::readInterrupt(byte* intf, byte* intcap)
{
	return readRegister(INTF, intf) &&
		readRegister(INTCAP, intcap);
}

// Read all bits of the GPIO register
// this clears the port's INTCAP register
bool MCP23Expander8bitBase::readGpios(byte* value)
{
	return readRegister(GPIO, value);
}

// Read/write the outputs (OLAT register)
bool MCP23Expander8bitBase::readOutputs(byte* value)
{
	return readRegister(OLAT, value);
}

bool MCP23Expander8bitBase::writeOutputs(byte value)
{
	// should not write to an input
	ASSERT((iodir & value) == 0);
	return writeRegister(OLAT, value);
}

// Write to a single output (OLAT register)
void MCP23Expander8bitBase::digitalWrite(byte pin, bool value)
{
	ASSERT(pin < 8);
	byte b;
	readOutputs(&b);
	byte mask = 1 << pin;
	if (value)
		b |= mask;
	else
		b &= ~mask;
	writeOutputs(b);
}

// Read a single input (or output) from the GPIO register
bool MCP23Expander8bitBase::digitalRead(byte pin)
{
	ASSERT(pin < 8);
	byte b;
	readGpios(&b);
	return (b & (1 << pin)) ? true : false;
}

// Read a single 8-bit register
bool MCP23Expander8bitBase::readRegister(byte reg, byte* value)
{
	return readRegisters(reg, value, 1);
}

#ifdef DEBUG

// Register read-after-write verification
bool MCP23Expander8bitBase::verify(byte reg, byte value)
{
	// read-after-write verification
	// skip read-only and GPIO registers
	switch (reg) {
	case INTF:
	case INTCAP:
	case GPIO:
		break;
	default:
		byte value1;
		if (!readRegister(reg, &value1))
			return false;
		if (value != value1) {
			LOGERROR("register compare error");
			return false;
		}
	}
	return true;
}

// Display all the register values
void MCP23Expander8bitBase::dumpRegister(byte reg, char* name, byte value)
{
	char buf[32];
	sprintf(buf, "%02X %-7s %02X", reg, name, value);
	Serial.println(buf);
	Serial.flush();
}
#define DUMP(reg, name) \
	readRegister(reg, &value); \
	dumpRegister(reg, name, value)

void MCP23Expander8bitBase::dumpRegisters()
{
	byte value;
	Serial.println("\nMCP23xxx REGISTERS");

	DUMP(IODIR, "IODIR");
	DUMP(IPOL, "IPOL");
	DUMP(GPINTEN, "GPINTEN");
	DUMP(DEFVAL, "DEFVAL");
	DUMP(INTCON, "INTCON");
	DUMP(IOCON, "IOCON");
	DUMP(GPPU, "GPPU");
	DUMP(INTF, "INTF");
	DUMP(INTCAP, "INTCAP");
	DUMP(GPIO, "GPIO");
	DUMP(OLAT, "OLAT");
}
#endif


#if defined(MCP23EXPANDER8BIT_SPI)

////////////////////////////////////////
// SPI VERSION

// 8-bit I/O expander with SPI interface
class MCP23Expander8bitSPI : public MCP23Expander8bitBase
{
protected:
	static SPISettings spiSettings;
	SPIClass* spi = NULL;
	byte csPin = 0xff;
	byte slaveAdds = 0xff;
	bool disableVerify = false;
	void enableHardwareAddressing();

public:
	bool begin(SPIClass* spiPort, byte chipSelectPin, byte slaveAddress = 0xff);
	bool isConnected();
	bool readRegisters(byte reg, byte* value, int length);
	bool writeRegister(byte reg, byte value);
};

// Shared SPI settings for all instances
// change the 10MHz clock frequency as desired
SPISettings MCP23Expander8bitSPI::spiSettings = SPISettings(10000000, MSBFIRST, SPI_MODE0);


// Don't forget to initialize SPI in setup()
//   SPI.begin();

// Call this for each expander chip, after calling SPI.begin().
// Use slaveAddress = 0..7 if you want to use SPI hardware addressing
// feature (or the default 0xff if not), see IOCON:HAEN bit
bool MCP23Expander8bitSPI::begin(SPIClass* spiPort, byte chipSelectPin,
	byte slaveAddress /*=0xff*/)
{
	spi = spiPort;
	csPin = chipSelectPin;
	pinMode(csPin, OUTPUT);
	::digitalWrite(csPin, 1);

	// the SPI device address feature allows up to 8 devices (0..7) to
	// use the same chip select
	// on first call, enable use of A3..A0 addresses with SPI if a 
	// slaveAddress has been supplied
	if (!spiHardwareAddressingEnabled && slaveAddress <= 7) {
		enableHardwareAddressing();
	}
	slaveAdds = (slaveAddress > 7) ? 0 : slaveAddress;

	// set the default configuration in case the chip did not get a hardware reset
	return setDefaults();
}

// MCP23S08 only
// Works only for the first message after a power-on-reset or /RESET signal.
// If using the A2..A0 address bit to allow up to 4 x SPI devices on the 
// same SPI chip select line, then this is called to set the HAEN bit in 
// all IOCON registers so the chips will use their addresses. 
// If this is not called then you won't be able to talk to these chips.
// The message is sent to all devices on the bus, immediately after reset.
void MCP23Expander8bitSPI::enableHardwareAddressing()
{
	// this cannot be called until the chip-select pin has been initialized
	ASSERT(csPin != 0xff);

	// only call it once
	if (spiHardwareAddressingEnabled)
		return;
	spiHardwareAddressingEnabled = true;
	disableVerify = true;

	// write command with address 0b000
	// this is seen by all devices, but only after a reset, when
	// the HAEN bit is clear
	slaveAdds = 0b000;
	writeRegister(IOCON, HAEN);

	disableVerify = false;
}

bool MCP23Expander8bitSPI::isConnected()
{
	//TODO what's the best way to detect this?
	return true;
}

// Read one or more consecutive 8-bit registers
// requires 'Sequential address mode' if reading more than one
bool MCP23Expander8bitSPI::readRegisters(byte reg, byte* values, int length)
{
	ASSERT(reg + length <= OLAT + 1);

	::digitalWrite(csPin, 0);
	spi->beginTransaction(spiSettings);
	spi->transfer(0b01000001 | (slaveAdds << 1));	// 0b0100aaa1
	spi->transfer(reg);
	for (int i = 0; i < length; ++i) {
		values[i] = spi->transfer(0);
	}
	spi->endTransaction();
	::digitalWrite(csPin, 1);
	return true;
}

// Write a single 8-bit register
bool MCP23Expander8bitSPI::writeRegister(byte reg, byte value)
{
	::digitalWrite(csPin, 0);
	spi->beginTransaction(spiSettings);
	spi->transfer(0b01000000 | (slaveAdds << 1));	// 0b0100aaa0
	spi->transfer(reg);
	spi->transfer(value);
	spi->endTransaction();
	::digitalWrite(csPin, 1);
	#ifdef DEBUG
		// read-after-write verification
	if (!disableVerify)
		return verify(reg, value);
	#endif
	return true;
}


#elif defined(MCP23EXPANDER8BIT_I2C)

////////////////////////////////////////
// I2C VERSION

// 8-bit I/O expander with I2C interface
class MCP23Expander8bitI2C : public MCP23Expander8bitBase
{
protected:
	TwoWire* wire;
	byte i2cAdds;

public:
	bool begin(TwoWire* twoWire, byte i2cAddress);
	bool isConnected();
	bool readRegisters(byte reg, byte* value, int length);
	bool writeRegister(byte reg, byte value);
};


// Initialize Wire before calling, for example...
//  Wire.begin();
//  Wire.setClock(1000000);
//  Wire.setTimeout(100);
//  mcp23.begin(&Wire, 0x20);
bool MCP23Expander8bitI2C::begin(TwoWire* twoWire, byte i2cAddress)
{
	wire = twoWire;
	i2cAdds = i2cAddress;

	// set the default configuration in case the chip did not get a hardware reset
	return setDefaults();
}

bool MCP23Expander8bitI2C::isConnected()
{
	wire->beginTransmission(i2cAdds);
	return wire->endTransmission() == 0;
}

// Read one or more consecutive 8-bit registers
// requires 'Sequential address mode' if reading more than one
bool MCP23Expander8bitI2C::readRegisters(byte reg, byte* values, int length)
{
	ASSERT(reg + length <= OLAT + 1);

	wire->beginTransmission(i2cAdds);
	if (wire->write(reg) != 1) {
		LOGERROR("write failed");
		return false;
	}
	if (wire->endTransmission() != 0) {
		LOGERROR("endtx failed");
		return false;
	}
	if (wire->requestFrom(i2cAdds, (size_t)length) != length) {
		LOGERROR("requestFrom failed");
		return false;
	}
	if (wire->readBytes(values, length) != length) {
		LOGERROR("readBytes failed");
		return false;
	}
	return true;
}

// Write a single 8-bit register
bool MCP23Expander8bitI2C::writeRegister(byte reg, byte value)
{
	wire->beginTransmission(i2cAdds);
	if (wire->write(reg) != 1) {
		LOGERROR("write failed");
		return false;
	}
	if (wire->write(value) != 1) {
		LOGERROR("write failed");
		return false;
	}
	if (wire->endTransmission() != 0) {
		LOGERROR("endtx failed");
		return false;
	}
	#ifdef DEBUG
		// read-after-write verification
	return verify(reg, value);
	#else
	return true;
	#endif
}

#endif

