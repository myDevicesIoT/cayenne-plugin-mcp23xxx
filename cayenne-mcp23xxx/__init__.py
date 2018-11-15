"""
This module provides classes for interfacing with MCP23XXX digital I/O extensions.
"""
import os
import time

from myDevices.devices.i2c import I2C
from myDevices.devices.spi import SPI
from myDevices.devices.digital import GPIOPort
from myDevices.plugins.digital import DigitalIO


class MCP23XXX(GPIOPort):
    """Base class for interacting with MCP23XXX extensions."""

    IODIR   = 0x00
    IPOL    = 0x01
    GPINTEN = 0x02
    DEFVAL  = 0x03
    INTCON  = 0x04
    IOCON   = 0x05
    GPPU    = 0x06
    INTF    = 0x07
    INTCAP  = 0x08
    GPIO    = 0x09
    OLAT    = 0x0A
    
    def __init__(self, channel_count):
        """Initializes MCP3XXX device.

        Arguments:
        channel_count: Number of channels on the device
        """
        GPIOPort.__init__(self, channel_count)
        self.banks = int(channel_count / 8)
        
    def getAddress(self, register, channel=0):
        """Returns the address for the specified register and channel."""
        return register * self.banks + int(channel / 8)

    def getChannel(self, register, channel):
        """Returns the address and channel mask for the specified register and channel."""
        self.checkDigitalChannel(channel)
        addr = self.getAddress(register, channel) 
        mask = 1 << (channel % 8)
        return (addr, mask)
    
    def __digitalRead__(self, channel):
        """Returns the value for the specified channel. Overrides GPIOPort.__digitalRead__."""
        (addr, mask) = self.getChannel(self.GPIO, channel) 
        d = self.readRegister(addr)
        return (d & mask) == mask

    def __digitalWrite__(self, channel, value):
        """Writes the value to the specified channel. Overrides GPIOPort.__digitalWrite__."""
        (addr, mask) = self.getChannel(self.GPIO, channel) 
        d = self.readRegister(addr)
        if value:
            d |= mask
        else:
            d &= ~mask
        self.writeRegister(addr, d)
        
    def __getFunction__(self, channel):
        """Returns the current IN/OUT function for the specified channel. Overrides GPIOPort.__getFunction__."""
        (addr, mask) = self.getChannel(self.IODIR, channel) 
        d = self.readRegister(addr)
        return self.IN if (d & mask) == mask else self.OUT
        
    def __setFunction__(self, channel, value):
        """Returns the IN/OUT function for the specified channel. Overrides GPIOPort.__setFunction__."""
        if not value in [self.IN, self.OUT]:
            raise ValueError("Requested function not supported")

        (addr, mask) = self.getChannel(self.IODIR, channel) 
        d = self.readRegister(addr)
        if value == self.IN:
            d |= mask
        else:
            d &= ~mask
        self.writeRegister(addr, d)

    def __portRead__(self):
        """Returns all values for the GPIO device as bits in an int. Overrides GPIOPort.__portRead__."""
        value = 0
        for i in range(self.banks):
            value |= self.readRegister(self.banks*self.GPIO+i) << 8*i
        return value
    
    def __portWrite__(self, value):
        """Writes all values for the GPIO device using the bits in the specified value. Overrides GPIOPort.__portWrite__."""
        for i in range(self.banks):
            self.writeRegister(self.banks*self.GPIO+i, (value >> 8*i) & 0xFF)


class MCP230XX(MCP23XXX, I2C):
    """Class for interacting with MCP230XX devices."""

    def __init__(self, slave, channel_count):
        """Initializes MCP230XX device.

        Arguments:
        slave: The I2C slave address
        channel_count: Number of channels on the device
        """
        I2C.__init__(self, int(slave))
        MCP23XXX.__init__(self, channel_count)
        
    def __str__(self):
        """Returns friendly name."""
        return "%s(slave=0x%02X)" % (self.__class__.__name__, self.slave)


class MCP23008(MCP230XX):
    """Class for interacting with a MCP23008 device."""

    def __init__(self, slave=0x20):
        """Initializes device with the specified slave address."""
        MCP230XX.__init__(self, slave, 8)


class MCP23009(MCP230XX):
    """Class for interacting with a MCP23009 device."""

    def __init__(self, slave=0x20):
        """Initializes device with the specified slave address."""
        MCP230XX.__init__(self, slave, 8)


class MCP23017(MCP230XX):
    """Class for interacting with a MCP23017 device."""

    def __init__(self, slave=0x20):
        """Initializes device with the specified slave address."""
        MCP230XX.__init__(self, slave, 16)


class MCP23018(MCP230XX):
    """Class for interacting with a MCP23018 device."""

    def __init__(self, slave=0x20):
        """Initializes device with the specified slave address."""
        MCP230XX.__init__(self, slave, 16)


MCP23S_SLAVES = [None for i in range(256)]

class MCP23SXX(MCP23XXX, SPI):
    """Class for interacting with MCP23SXX devices."""

    SLAVE = 0x20

    WRITE = 0x00
    READ  = 0x01
    
    def __init__(self, chip, slave, channel_count):
        """Initializes MCP23SXX device.

        Arguments:
        chip: The chip select
        slave: The slave address
        channel_count: Number of channels on the device
        """
        global MCP23S_SLAVES
        index = int(chip) << 7 | int(slave)
        print(MCP23S_SLAVES[index])

        if MCP23S_SLAVES[index] != None:
            raise Exception("SLAVE_ADDRESS_USED")
        
        SPI.__init__(self, int(chip), 0, 8, 10000000)
        MCP23XXX.__init__(self, channel_count)
        self.slave = self.SLAVE
        iocon_value = 0x08 # Hardware Address Enable
        iocon_addr  = self.getAddress(self.IOCON)
        self.writeRegister(iocon_addr, iocon_value)
        self.slave = int(slave)
        MCP23S_SLAVES[index] = self
        print(MCP23S_SLAVES[index])

    def __str__(self):
        """Returns friendly name."""
        return "%s(chip=%d, slave=0x%02X)" % (self.__class__.__name__, self.chip, self.slave)

    def readRegister(self, addr):
        """Returns the value in the specified register address."""
        d = self.xfer([(self.slave << 1) | self.READ, addr, 0x00])
        return d[2]

    def writeRegister(self, addr, value):
        """Writes value to the specified register address."""
        self.writeBytes([(self.slave << 1) | self.WRITE, addr, value])

    def close(self):
        """Closes the SPI bus."""
        SPI.close(self)

    
class MCP23S08(MCP23SXX):
    """Class for interacting with a MCP23S08 device."""

    def __init__(self, chip=0, slave=0x20):
        """Initializes device with the specified chip select and slave address."""
        MCP23SXX.__init__(self, chip, slave, 8)


class MCP23S09(MCP23SXX):
    """Class for interacting with a MCP23S09 device."""

    def __init__(self, chip=0, slave=0x20):
        """Initializes device with the specified chip select and slave address."""
        MCP23SXX.__init__(self, chip, slave, 8)


class MCP23S17(MCP23SXX):
    """Class for interacting with a MCP23S17 device."""

    def __init__(self, chip=0, slave=0x20):
        """Initializes device with the specified chip select and slave address."""
        MCP23SXX.__init__(self, chip, slave, 16)


class MCP23S18(MCP23SXX):
    """Class for interacting with a MCP23S18 device."""

    def __init__(self, chip=0, slave=0x20):
        """Initializes device with the specified chip select and slave address."""
        MCP23SXX.__init__(self, chip, slave, 16)


class PiFaceDigital(MCP23S17):
    """Class for interacting with a PiFace Digital device."""

    def __init__(self, board=0):
        """Initializes device with the specified board number."""
        self.board = int(board)
        MCP23S17.__init__(self, 0, 0x20 + self.board)
        self.writeRegister(self.getAddress(self.IODIR, 0), 0x00) # Port A as output
        self.writeRegister(self.getAddress(self.IODIR, 8), 0xFF) # Port B as input
        self.writeRegister(self.getAddress(self.GPPU,  0), 0x00) # Port A PU OFF
        self.writeRegister(self.getAddress(self.GPPU,  8), 0xFF) # Port B PU ON


class MCPTest(MCP230XX):
    """Class for simulating an MCP230XX device."""

    def __init__(self):
        """Initializes the test class."""
        MCP230XX.__init__(self, 0x20, 16)
        self.registers = {}

    def readRegister(self, addr):
        """Read value from a register."""
        if addr not in self.registers:
            self.registers[addr] = 0
        return self.registers[addr]

    def writeRegister(self, addr, value):
        """Write value to a register."""
        self.registers[addr] = value