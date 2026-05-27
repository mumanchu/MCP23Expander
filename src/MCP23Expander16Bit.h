#pragma once

// MCP23x17/MCP23x18 16-Bit GPIO Expander with I2C Interface
// Copyright (C) MattLabs and muman.ch, 2025.06.23
// All rights reversed
/*
DATA SHEETS
MCP23x17
https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf
MCP23x18
https://ww1.microchip.com/downloads/en/devicedoc/22103a.pdf

For details see the muman.ch blog post
https://muman.ch/muman/index.htm?muman-mcp23017.htm

"MCP23S17 Rev.A Silicon Errata" (document DS80311A)
https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/Errata/80311a.pdf
*/

#include <SPI.h>
#include <Wire.h>


// Base class for 16-bit expander SPI and I2C classes
class MCP23Expander16bitBase
{
protected:
	// For SPI IOCON:HAEN bit, set by enableHardwareAddressing()
	static bool spiHardwareAddressingEnabled;

public:
	// Input masks for each GPIO port, 1=the bit is an input, 0=output
	byte iodirA = 0xff;
	byte iodirB = 0xff;

	// For the 'port' parameter, handled as 2 x 8-bit ports
	enum PORT : byte
	{
		PORTA = 0,
		PORTB = 1
	};

	// Port A register numbers for BANK mode 0, add 1 for the Port B register
	enum REGS : byte
	{
		IODIR = 0x00,     // I/O direction: 1=input, 0=output
		IPOL = 0x02,     // Input polarity: 1=inverted, 0=not inverted
		GPINTEN = 0x04,     // Interrupt-on-change enable: 1=enable interrupt, 0=no interrupt
		DEFVAL = 0x06,     // Default compare register for interrupt, interrupt occurs if bit does NOT match
		INTCON = 0x08,     // Interrupt-on-change control: 1=pin state compared with DEFVAL bit, 0=not compared
		IOCON = 0x0A,     // Chip configuration, see IOCON enum for bits, shared by both Ports
		GPPU = 0x0C,     // Pull-up resistor configuration: 1=pull-up enabled, 0=pull-up disabled
		INTF = 0x0E,     // Interrupt flag, which input caused the interrupt: 1=pin caused interrupt
		INTCAP = 0x10,     // Interrupt capture, port value at time of interrupt
		GPIO = 0x12,     // Port value, inputs and outputs
		OLAT = 0x14      // Output latch, read/write output latches
	};

	// IOCON configuration register bits (the IOCON register is shared by both ports)
	enum IOCON : byte
	{
		INTPOL = 0x02,     // Polarity of INT pins: 0=active low, 1=active high
		ODR = 0x04,     // Open drain INT pins: 0=driven, 1=open drain (overrides INTPOL)
		HAEN = 0x08,     // Hardware address enable, SPI devices only
		DISSLW = 0x10,     // SDA slew rate control: 0=enabled, 1=disabled
		SEQOP = 0x20,     // Sequential address mode: 0=address increments, 1=address unchanged
		MIRROR = 0x40,     // Interrupt pins mirrored: 0=separate INTA and INTB, 1=connected
		BANK = 0x80      // How registers are addressed: 0=same bank (sequential), 1=two banks
	};

	bool setDefaults();
	bool configureInterruptPins(bool polarity, bool openDrain, bool mirrored);
	bool configureInterrupts(PORT port, byte gpinten, byte defval, byte intcon);
	bool configureGpios(PORT port, byte iodir, byte ipol, byte gppu);
	bool readBothInterrupts(byte* intfA, byte* intcapA, byte* intfB, byte* intcapB);
	bool readInterrupt(PORT port, byte* intf, byte* intcap);
	void digitalWrite(PORT port, byte pin, bool value);
	bool digitalRead(PORT port, byte pin);
	bool readBothGpios(byte* gpioA, byte* gpioB);
	bool readGpios(PORT port, byte* value);
	bool readOutputs(PORT port, byte* value);
	bool writeOutputs(PORT port, byte value);
	bool readRegister(byte reg, byte* value);

	virtual bool isConnected() = 0;
	virtual bool readRegisters(byte reg, byte* value, int length) = 0;
	virtual bool writeRegister(byte reg, byte value) = 0;

	#ifdef DEBUG
	bool verify(byte reg, byte value);
	void dumpRegisters(PORT port);
	void dumpRegister(byte reg, char* name, byte value);
	#endif
};

// for SPI's IOCON:HAEN bit, see SPI's enableHardwareAddressing()
// this is shared by all instances of this class
bool MCP23Expander16bitBase::spiHardwareAddressingEnabled = false;


// Set registers as they would be after a power-on reset (POR)
// Except for the SPI IOCON:HAEN bit which is controlled by SPI's
// enableHardwareAddressing().
// Note that both IOCON registers are the same, shared by both ports.
// INT pin active low; INT pin driven; slew rate enabled; address increments;
// INTA/INTB separate; one-bank interleaved register addressing;
// HAEN according to enableHardwareAddressing.
// https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf#page=21
bool MCP23Expander16bitBase::setDefaults()
{
	// set IODIR to 0xff (all pins of both ports := inputs)
	if (!writeRegister(IODIR, 0xff) || !writeRegister(IODIR + 1, 0xff))
		return false;
	iodirA = 0xff;
	iodirB = 0xff;

	// IOCON register
	// for SPI, keep HAEN set if enableHardwareAddressing() was called
	if (!writeRegister(IOCON, spiHardwareAddressingEnabled ? HAEN : 0))
		return false;

	// set all other writable registers to 0
	for (byte reg = IPOL; reg <= OLAT + 1; ++reg) {
		// skip read-only registers and IOCON
		switch (reg) {
		case INTF:
		case INTF + 1:
		case INTCAP:
		case INTCAP + 1:
		case IOCON:
		case IOCON + 1:
			continue;
		}
		if (!writeRegister(reg, 0))
			return false;
	}
	return true;
}

// Configure the INTA and INTB pins
// polarity  = polarity of INT output pins: 0=active low, 1=active high
// openDrain = open drain INT pins: 0=driven, 1=open drain (overrides polarity)
// mirrored  = interrupt pins mirrored: 0=separate INTA and INTB, 1=connected
bool MCP23Expander16bitBase::configureInterruptPins(bool polarity, bool openDrain, bool mirrored)
{
	// BANK=0 (one register bank)
	// SEQOP=0 (sequential operation enabled)
	// HAEN=1 (SPI hardware address enable A0..A3)
	byte iocon = 0;
	if (spiHardwareAddressingEnabled)    // SPI only, see enableHardwareAddressing()
		iocon |= HAEN;
	if (polarity)
		iocon |= INTPOL;
	if (openDrain)
		iocon |= ODR;
	if (mirrored)
		iocon |= MIRROR;
	return writeRegister(IOCON, iocon);
}

// Configure interrupts
// gpinten = interrupt-on-change pin enable: 1=interrupt enabled, 0=interrupt disabled
// defval  = default value register: different bit causes an interrupt, enabled by intcon bit
// intcon  = interrupt control register: 1=bit compared with DEFVAL register, 0=not compared
bool MCP23Expander16bitBase::configureInterrupts(PORT port, byte gpinten, byte defval, byte intcon)
{
	return writeRegister(GPINTEN + port, gpinten) &&
		writeRegister(DEFVAL + port, defval) &&
		writeRegister(INTCON + port, intcon);
}

// Configure a port's inputs/outputs
// iodir = direction: 1=input, 0=output
// ipol  = input polarity: 1=inverted, 0=not inverted
// gppu  = input pull-up: 1=pull-up enabled, 0=pull-up disabled
bool MCP23Expander16bitBase::configureGpios(PORT port, byte iodir, byte ipol, byte gppu)
{
	// save input mask, 1=input
	(port ? iodirB : iodirA) = iodir;

	return writeRegister(IODIR + port, iodir) &&
		writeRegister(IPOL + port, ipol) &&
		writeRegister(GPPU + port, gppu);
}

// Read interrupt details for both ports
// intf = interrupt flag register, 1=pin caused interrupt
// intcap = interrupt captured register, GPIO port value at time of interrupt
// "The INTCAP register remains unchanged until the interrupt is cleared by 
// a read of INTCAP or GPIO."
bool MCP23Expander16bitBase::readBothInterrupts(byte* intfA, byte* intcapA,
	byte* intfB, byte* intcapB)
{
	byte data[4];
	if (!readRegisters(INTF, data, 4))
		return false;
	*intfA = data[0];
	*intfB = data[1];
	*intcapA = data[2];
	*intcapB = data[3];
	return true;
}

// Read interrupt details for one port, Port A or B
// "The INTCAP register remains unchanged until the interrupt is cleared by 
// a read of INTCAP or GPIO."
bool MCP23Expander16bitBase::readInterrupt(PORT port, byte* intf, byte* intcap)
{
	return readRegister(INTF + port, intf) &&
		readRegister(INTCAP + port, intcap);
}

// Read both port's GPIO registers
// this clears both port's INTCAP registers
bool MCP23Expander16bitBase::readBothGpios(byte* gpiosA, byte* gpiosB)
{
	byte buf[2];
	if (!readRegisters(GPIO, buf, 2))
		return false;
	*gpiosA = buf[0];
	*gpiosB = buf[1];
	return true;
}

// Read all bits of GPIO register of Port A or B
// this clears the port's INTCAP register
bool MCP23Expander16bitBase::readGpios(PORT port, byte* value)
{
	return readRegister(GPIO + port, value);
}

// Read/write the outputs (OLAT register) of Port A or B
bool MCP23Expander16bitBase::readOutputs(PORT port, byte* value)
{
	return readRegister(OLAT + port, value);
}

bool MCP23Expander16bitBase::writeOutputs(PORT port, byte value)
{
	// should not write to an input
	ASSERT(((port ? iodirB : iodirA) & value) == 0);
	return writeRegister(OLAT + port, value);
}

// Write to a single output (OLAT register) of Port A or B
void MCP23Expander16bitBase::digitalWrite(PORT port, byte pin, bool value)
{
	ASSERT(pin < 8);
	byte b;
	readOutputs(port, &b);
	byte mask = 1 << pin;
	if (value)
		b |= mask;
	else
		b &= ~mask;
	writeOutputs(port, b);
}

// Read a single input (or output) from the GPIO register
bool MCP23Expander16bitBase::digitalRead(PORT port, byte pin)
{
	ASSERT(pin < 8);
	byte b;
	readGpios(port, &b);
	return (b & (1 << pin)) ? true : false;
}

// Read a single 8-bit register
bool MCP23Expander16bitBase::readRegister(byte reg, byte* value)
{
	return readRegisters(reg, value, 1);
}

#ifdef DEBUG

// Register read-after-write verification
bool MCP23Expander16bitBase::verify(byte reg, byte value)
{
	// read-after-write verification
	// skip read-only and GPIO registers
	switch (reg) {
	case INTF:
	case INTF + 1:
	case INTCAP:
	case INTCAP + 1:
	case GPIO:
	case GPIO + 1:
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

// Display all the register values of Port A or B
void MCP23Expander16bitBase::dumpRegister(byte reg, char* name, byte value)
{
	char buf[32];
	sprintf(buf, "%02X %-7s %02X", reg, name, value);
	Serial.println(buf);
	Serial.flush();
}
#define DUMP(reg, name) \
	readRegister(reg + port, &value); \
	dumpRegister(reg + port, name, value)

void MCP23Expander16bitBase::dumpRegisters(PORT port)
{
	byte value;
	Serial.println(port ? "\nPORT B" : "\nPORT A");

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


////////////////////////////////////////
// SPI VERSION

// 16-bit I/O expander with SPI interface
class MCP23Expander16bitSPI : public MCP23Expander16bitBase
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
SPISettings MCP23Expander16bitSPI::spiSettings = SPISettings(10000000, MSBFIRST, SPI_MODE0);


// Don't forget to initialize SPI in setup()
//   SPI.begin();

// Call this for each expander chip, after calling SPI.begin().
// Use slaveAddress = 0..7 if you want to use SPI hardware addressing
// feature (or the default 0xff if not), see IOCON:HAEN bit
bool MCP23Expander16bitSPI::begin(SPIClass* spiPort, byte chipSelectPin,
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

// This works only after a power-on-reset or /RESET signal.
// If using the A3..A0 address bit to allow up to 8 x SPI devices on the 
// same SPI chip select line, then this is called to set the HAEN bit in 
// all IOCON registers so the chips will use their addresses. 
// If this is not called then you won't be able to talk to these chips.
// The message is sent to all devices on the bus, immediately after reset.
void MCP23Expander16bitSPI::enableHardwareAddressing()
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

	// There is/was a silicon error in the MCP23S17 chip
	// see "MCP23S17 Rev.A Silicon Errata" (document DS80311A)
	// "When IOCON.HAEN = 0 (hardware addressing disabled) : 
	// If the A2 pin is high, then the device must be addressed as A2/A1/A0 = 
	// 1XX (i.e. OPCODE = 0b01001XXx).
	// Work around - None"
	// but they gave us the workaround, do the same thing with address 0b100...
	slaveAdds = 0b100;
	writeRegister(IOCON, HAEN);

	disableVerify = false;
}

bool MCP23Expander16bitSPI::isConnected()
{
	//TODO what's the best way to detect this?
	return true;
}

// Read one or more consecutive 8-bit registers
// requires 'Sequential address mode' if reading more than one
bool MCP23Expander16bitSPI::readRegisters(byte reg, byte* values, int length)
{
	ASSERT(reg + length <= OLAT + 2);

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
bool MCP23Expander16bitSPI::writeRegister(byte reg, byte value)
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


////////////////////////////////////////
// I2C VERSION

// 16-bit I/O expander with I2C interface
class MCP23Expander16bitI2C : public MCP23Expander16bitBase
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
bool MCP23Expander16bitI2C::begin(TwoWire* twoWire, byte i2cAddress)
{
	wire = twoWire;
	i2cAdds = i2cAddress;

	// set the default configuration in case the chip did not get a hardware reset
	return setDefaults();
}

bool MCP23Expander16bitI2C::isConnected()
{
	wire->beginTransmission(i2cAdds);
	return wire->endTransmission() == 0;
}

// Read one or more consecutive 8-bit registers
// requires 'Sequential address mode' if reading more than one
bool MCP23Expander16bitI2C::readRegisters(byte reg, byte* values, int length)
{
	ASSERT(reg + length <= OLAT + 2);

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
bool MCP23Expander16bitI2C::writeRegister(byte reg, byte value)
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

