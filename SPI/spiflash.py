# -*- coding:utf-8 -*-
""" SPI Flash driver for Micropython. """
# !/usr/bin/python
# Python:   3.5.2
# Platform: Cortex-M4 (STM32F4xx)
# Author:   Heyn (heyunhuan@gmail.com)
# Program:  SPI Flash driver for Micropython.
# History:  2018-01-11 V1.0.0 [Heyn]

import pyb
import ubinascii
from pyb import SPI
from micropython import const

# Command register definitions
__SFLASH_WRITE_STATUS_REGISTER = 0x01
__SFLASH_WRITE = 0x02
__SFLASH_READ = 0x03
__SFLASH_WRITE_DISABLE = 0x04         # WRDI
__SFLASH_READ_STATUS_REGISTER = 0x05  # RDSR
__SFLASH_WRITE_ENABLE = 0x06          # WREN
__SFLASH_FAST_READ = 0x0B             # FAST READ
__SFLASH_SECTOR_ERASE = 0x20          #
__SFLASH_READ_JEDEC_ID = 0x9F         # RDID

__SFLASH_CHIP_ERASE1 = 0x60
__SFLASH_CHIP_ERASE2 = 0xC7
__SFLASH_SECTOR_ERASE_MICRON = 0xD8   # Sector Erase (64KB) - Micron only


SFLASH_ID_SIZE = dict(ID_C22014=0x100000,  # MX25L8006E   SIZE_1MByte
                      ID_C22015=0x200000,  # MX25L1606E   SIZE_2MByte
                      ID_C22017=0x800000,  # MX25L6433F   SIZE_8MByte
                      ID_C22018=0x1000000, # MX25L12835F  SIZE_16MByte
                      ID_C22019=0x2000000, # MX25L25635F  SIZE_32MByte
                      ID_C22535=0x200000,  # MX25U1635F   SIZE_2MByte
                      ID_C2253A=0x4000000, # MX66U51235F  SIZE_64MByte
                      ID_BF258E=0x100000,  # SST25VF080B  SIZE_1MByte
                      ID_1C3015=0x200000,  # EN25QH16     SIZE_2MByte
                      ID_7F9D46=0x400000,  # ISSI25CQ032  SIZE_4MByte
                      ID_20BB20=0x4000000, # N25Q512A     SIZE_64MByte
                      ID_9D6017=0x800000,  # ISSI25LP064  SIZE_8MByte
                      ID_20BA17=0x800000,  # N25Q064A     SIZE_8MByte
                      ID_EF4017=0x800000,  # W25Q64FV     SIZE_8MByte
                      ID_7F7F7F=0x80000    # CY15B104Q    SIZE_512KByte
                     )

class SPIFlash:

    def __init__(self, nss='A4'):
        self.__nss = pyb.Pin(nss)
        self.__nss.init(pyb.Pin.OUT_PP)
        self.__nss.high()
        self.spi = SPI(1, SPI.MASTER, baudrate=42000000, polarity=0, phase=0)

    def __generic_command(self, cmd):
        self.__nss.low()
        self.spi.send(cmd)
        self.__nss.high()

    def __write_enable(self):
        self.__generic_command(__SFLASH_WRITE_ENABLE)
        self.__read_status_register()

    def __read_status_register(self):
        """ Status Register bit definitions

            STATUS_REGISTER_BUSY                             0x01
            STATUS_REGISTER_WRITE_ENABLED                    0x02
            STATUS_REGISTER_BLOCK_PROTECTED_0                0x04
            STATUS_REGISTER_BLOCK_PROTECTED_1                0x08
            STATUS_REGISTER_BLOCK_PROTECTED_2                0x10    [SST & Macronix Only]
            STATUS_REGISTER_BLOCK_PROTECTED_3                0x20    [SST & Macronix Only]
            STATUS_REGISTER_AUTO_ADDRESS_INCREMENT           0x40    [SST Only]
            STATUS_REGISTER_BLOCK_PROTECT_BITS_READ_ONLY     0x80    [SST Only]
            STATUS_REGISTER_QUAD_ENABLE                      0x40    [Macronix Only]
            STATUS_REGISTER_WRITE_PROTECT_PIN_ENABLE         0x80    [Macronix Only]
        """
        buf = bytearray(1)
        self.__nss.low()
        self.spi.send_recv(__SFLASH_READ_STATUS_REGISTER, buf)
        self.__nss.high()
        print('__read_status_register: %s'%ubinascii.hexlify(buf).decode('UTF-8').upper())
        return buf[0]

    def read_id(self):
        """ Read SPI Flash ID. """
        self.__nss.low()
        self.spi.send(__SFLASH_READ_JEDEC_ID)
        ret = self.spi.recv(3)
        self.__nss.high()

        return ubinascii.hexlify(ret).decode('UTF-8').upper()

    def wait(self):
        while True:
            if self.__read_status_register() == 0:
                return

    def chip_erase(self):
        self.__write_enable()
        self.__generic_command(__SFLASH_CHIP_ERASE2)
        return True

    def write(self, addr, buf):
        """ Write Data to Flash. """
        pos = 0
        write_size = len(buf)
        max_write_size = 128    # TODO

        while pos < write_size:
            size = min(write_size-pos, max_write_size)
            self.__write_enable()

            self.__nss.low()
            self.spi.send(__SFLASH_WRITE)
            self.spi.send((addr & 0x00FF0000) >> 16)
            self.spi.send((addr & 0x0000FF00) >>  8)
            self.spi.send((addr & 0x000000FF) >>  0)
            self.spi.send(buf[pos:pos+size])
            self.__nss.high()
            self.wait()
            addr += size
            pos += size
        return True

    def read(self, addr, size):
        self.__nss.low()
        self.spi.send(__SFLASH_READ)
        self.spi.send((addr & 0x00FF0000) >> 16)
        self.spi.send((addr & 0x0000FF00) >>  8)
        self.spi.send((addr & 0x000000FF) >>  0)
        ret = self.spi.recv(size)
        self.__nss.high()
        return ret

# import spiflash
# spi = spiflash.SPIFlash()
# spi.read_id()
# spi.chip_erase()
# spi.write(0, b'hello')
# spi.read(0, 10)
