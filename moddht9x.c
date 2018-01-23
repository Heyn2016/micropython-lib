/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


#include <stdio.h>

#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/machine_pulse.h"

#include "drivers/dht/dht9x.h"

#define DHT9X_NOACK           (0x00)
#define DHT9X_ACK             (0x01)

                                        //ADR   CMDs   R/W
#define DHT9X_STATUS_REG_W    (0x06)    //000   0011    0
#define DHT9X_STATUS_REG_R    (0x07)    //000   0011    1
#define DHT9X_MEASURE_TEMP    (0x03)    //000   0001    1
#define DHT9X_MEASURE_HUMI    (0x05)    //000   0010    1
#define DHT9X_RESET           (0x1E)    //000   1111    0


typedef struct _mp_dht9x_t {
    mp_hal_pin_obj_t sck;
    mp_hal_pin_obj_t sda;
} mp_dht9x_t;

enum {TEMP, HUMI};

typedef union {
    unsigned int i;
    float f;
}value;


// Generates a transmission start 
//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______

STATIC void dht9x_bus_transstart(mp_dht9x_t self)
{
    mp_hal_pin_write(self.sda, 1);  //Initial state
    mp_hal_pin_write(self.sck, 0);
    mp_hal_delay_us(1);

    mp_uint_t irq_state = mp_hal_quiet_timing_enter();

    mp_hal_pin_write(self.sck, 1);
    mp_hal_delay_us_fast(1);
    mp_hal_pin_write(self.sda, 0);
    mp_hal_delay_us_fast(1);
    mp_hal_pin_write(self.sck, 0);
    mp_hal_delay_us_fast(3);
    mp_hal_pin_write(self.sck, 1);
    mp_hal_delay_us_fast(1);
    mp_hal_pin_write(self.sda, 1);
    mp_hal_delay_us_fast(1);
    mp_hal_pin_write(self.sck, 0);

    mp_hal_quiet_timing_exit(irq_state);
}

STATIC void dht9x_bus_connectionreset(mp_dht9x_t self)
{
    mp_hal_pin_write(self.sda, 1);
    mp_hal_pin_write(self.sck, 0);

    mp_uint_t irq_state = mp_hal_quiet_timing_enter();
    for (int i=0; i<9; i++) {
        mp_hal_pin_write(self.sck, 1);
        mp_hal_pin_write(self.sck, 0);
    }
    mp_hal_quiet_timing_exit(irq_state);

    dht9x_bus_transstart(self);
}

STATIC uint8_t dht9x_bus_write_byte(mp_dht9x_t self, uint8_t value)
{
    uint8_t error = 0;

    mp_uint_t irq_state = mp_hal_quiet_timing_enter();

    for (int i=0x80; i>0; i/=2) {
        mp_hal_pin_write(self.sda, i & value);
        mp_hal_pin_write(self.sck, 1);
        mp_hal_delay_us_fast(3);
        mp_hal_pin_write(self.sck, 0);
    }

    mp_hal_quiet_timing_exit(irq_state);

    mp_hal_pin_write(self.sda, 1);
    mp_hal_pin_write(self.sck, 1);
    error = mp_hal_pin_read(self.sda);
    mp_hal_pin_write(self.sck, 0);

    return error;
}

STATIC int8_t dht9x_bus_read_byte(mp_dht9x_t self, uint8_t ack)
{
    int8_t value = 0;

    mp_hal_pin_write(self.sda, 1);
    mp_uint_t irq_state = mp_hal_quiet_timing_enter();

    for (int i=0x80; i>0; i/=2) {
        mp_hal_pin_write(self.sck, 1);
        if (mp_hal_pin_read(self.sda)) {
            value = value | i;
        }
        mp_hal_pin_write(self.sck, 0);
    }

    mp_hal_pin_write(self.sda, !ack);
    mp_hal_pin_write(self.sck, 1);
    mp_hal_delay_us(5);
    mp_hal_pin_write(self.sck, 0);
    mp_hal_pin_write(self.sda, 1);

    mp_hal_quiet_timing_exit(irq_state);

    return value;
}   

STATIC void _dht9x_calc_humi_temp(float *p_humidity ,float *p_temperature)
{
    const float C1 = -4.0;
    const float C2 = +0.0405;
    const float C3 = -0.0000028;
    const float T1 = +0.01;
    const float T2 = +0.00008;

    float rh = *p_humidity;
    float t = *p_temperature;
    float rh_lin;
    float rh_true;
    float t_C;

    t_C = t*0.01 - 40;
    rh_lin = C3*rh*rh + C2*rh + C1;
    rh_true = (t_C-25)*(T1+T2*rh) + rh_lin;
    if (rh_true > 100) rh_true = 100;
    if (rh_true < 0.1) rh_true = 0.1;

    *p_temperature = t_C;
    *p_humidity = rh_true;
}

uint8_t _dht9x_calc_crc8(uint8_t byte, uint8_t crc8)
{
    uint8_t temp;
    crc8 ^= byte;
    for (int i=0; i<8; i++) {
        temp = crc8;
        crc8 <<= 0x01;
        if ((temp & 0x80) != 0x00) {
            crc8 ^= 0x31;
        }
    }
    return crc8;
}

uint8_t _dht9x_swap_crc8(uint8_t crc8)
{
    uint8_t temp = 0x00;

    for (int i=0x80; i>0; i/=2) {
        if ((crc8 & i) != 0x00) {
            temp |= 0x80;
        }
        if (i == 0x01) {
            break;
        }
        temp >>= 0x01;
        temp  &= 0x7F;
    }
    return temp;
}

uint8_t _dht9x_check_crc8(uint8_t reg, uint8_t crc8, unsigned int value)
{
    uint8_t crc = 0x00;

    crc = _dht9x_calc_crc8(reg, crc);
    crc = _dht9x_calc_crc8(value/256, crc);
    crc = _dht9x_calc_crc8(value%256, crc);
    crc = _dht9x_swap_crc8(crc);
    if (crc != crc8) {
        return 0;
    }

    return 1;
}

// makes a measurement (humidity/temperature) with checksum
uint8_t dht9x_measure(mp_dht9x_t dht9x, uint8_t *p_value, uint8_t *p_checksum, uint8_t mode)
{ 
    uint8_t error=0;

    dht9x_bus_transstart(dht9x);

    switch(mode) {
        case TEMP	: error += dht9x_bus_write_byte(dht9x, DHT9X_MEASURE_TEMP); break;
        case HUMI	: error += dht9x_bus_write_byte(dht9x, DHT9X_MEASURE_HUMI); break;
        default     : break;
    }

    for (int i=0; i<65535; i++) {
        if(mp_hal_pin_read(dht9x.sda) == 0x00) {
            break;
        }
    }

    if(mp_hal_pin_read(dht9x.sda)) {
        error += 1;
    }

    *(p_value)   = dht9x_bus_read_byte(dht9x, DHT9X_ACK);    //read the first byte (MSB)
    *(p_value+1) = dht9x_bus_read_byte(dht9x, DHT9X_ACK);    //read the second byte (LSB)
    *p_checksum  = dht9x_bus_read_byte(dht9x, DHT9X_NOACK);  //read checksum

    return error;
}

/******************************************************************************/
// MicroPython bindings

// Resets the sensor by a softreset 
STATIC mp_obj_t dht9x_softreset(mp_obj_t pin_sck, mp_obj_t pin_sda)
{ 
    int8_t error = 0;
    mp_dht9x_t dht9x;

    dht9x.sck = mp_hal_get_pin_obj(pin_sck);
    dht9x.sda = mp_hal_get_pin_obj(pin_sda);
    
    mp_hal_pin_output(dht9x.sck);
    mp_hal_pin_open_drain(dht9x.sda);

    dht9x_bus_connectionreset(dht9x);
    error += dht9x_bus_write_byte(dht9x, DHT9X_RESET);

    return MP_OBJ_NEW_SMALL_INT(error);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(dht9x_softreset_obj, dht9x_softreset);

// Reads the status register with checksum (8-bit)
STATIC mp_obj_t dht9x_read_statusreg(mp_obj_t pin_sck, mp_obj_t pin_sda, mp_obj_t buf_in)
{ 
    uint8_t error = 0;
    mp_dht9x_t dht9x;

    dht9x.sck = mp_hal_get_pin_obj(pin_sck);
    dht9x.sda = mp_hal_get_pin_obj(pin_sda);

    dht9x_bus_transstart(dht9x);                              //transmission start

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len < 2) {
        mp_raise_ValueError("buffer too small");
    }

    uint8_t *pbuf = bufinfo.buf;

    error       = dht9x_bus_write_byte(dht9x, DHT9X_STATUS_REG_R);  //send command to sensor
    *(pbuf + 0) = dht9x_bus_read_byte(dht9x, DHT9X_ACK);      //read status register (8-bit)
    *(pbuf + 1) = dht9x_bus_read_byte(dht9x, DHT9X_NOACK);    //read checksum (8-bit)

    return MP_OBJ_NEW_SMALL_INT(error);   //error=1 in case of no response form the sensor
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(dht9x_read_statusreg_obj, dht9x_read_statusreg);

// Writes the status register with checksum (8-bit)
STATIC mp_obj_t dht9x_write_statusreg(mp_obj_t pin_sck, mp_obj_t pin_sda, mp_obj_t value_in)
{ 
    mp_dht9x_t dht9x;
    uint8_t error = 0;
    int value = mp_obj_get_int(value_in);

    dht9x.sck = mp_hal_get_pin_obj(pin_sck);
    dht9x.sda = mp_hal_get_pin_obj(pin_sda);

    dht9x_bus_transstart(dht9x);
    error += dht9x_bus_write_byte(dht9x, DHT9X_STATUS_REG_W);
    error += dht9x_bus_write_byte(dht9x, (uint8_t)(value & 0xFF));

    return MP_OBJ_NEW_SMALL_INT(error);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(dht9x_write_statusreg_obj, dht9x_write_statusreg);

// Makes a measurement temperature with checksum
STATIC mp_obj_t dht9x_measure_temp(mp_obj_t pin_sck, mp_obj_t pin_sda, mp_obj_t buf_in)
{
    value temp_val;
    uint8_t error = 0, checksum = 0;
    mp_dht9x_t dht9x;

    dht9x.sck = mp_hal_get_pin_obj(pin_sck);
    dht9x.sda = mp_hal_get_pin_obj(pin_sda);
    
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len < 3) {
        mp_raise_ValueError("buffer too small");
    }

    error = dht9x_measure(dht9x, (uint8_t*)&temp_val.i, &checksum, TEMP);
    temp_val.f = (((float)temp_val.i)*0.01 - 40)*100;

    uint8_t *pbuf = bufinfo.buf;
    *(pbuf + 0) = ((uint16_t)(temp_val.f * 100) >> 8) & 0xFF;
    *(pbuf + 1) = ((uint16_t)(temp_val.f * 100) >> 0) & 0xFF;
    *(pbuf + 2) = checksum;

    return MP_OBJ_NEW_SMALL_INT(error);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(dht9x_measure_temp_obj, dht9x_measure_temp);

// Makes a measurement humidity with checksum
STATIC mp_obj_t dht9x_measure_humi(mp_obj_t pin_sck, mp_obj_t pin_sda, mp_obj_t buf_in)
{
    value temp_val, humi_val;
    uint8_t error = 0, checksum = 0;
    mp_dht9x_t dht9x;
    
    dht9x.sck = mp_hal_get_pin_obj(pin_sck);
    dht9x.sda = mp_hal_get_pin_obj(pin_sda);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len < 3) {
        mp_raise_ValueError("buffer too small");
    }

    error += dht9x_measure(dht9x, (uint8_t*)&temp_val.i, &checksum, TEMP);
    error += dht9x_measure(dht9x, (uint8_t*)&humi_val.i, &checksum, HUMI);

    _dht9x_calc_humi_temp(&humi_val.f, &temp_val.f);
    uint8_t *pbuf = bufinfo.buf;
    *(pbuf + 0) = ((uint16_t)(humi_val.f * 100) >> 8) & 0xFF;
    *(pbuf + 1) = ((uint16_t)(humi_val.f * 100) >> 0) & 0xFF;
    *(pbuf + 2) = checksum;

    return MP_OBJ_NEW_SMALL_INT(error);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(dht9x_measure_humi_obj, dht9x_measure_humi);

// Makes a measurement temperature and humidity.
STATIC mp_obj_t dht9x_readinto(mp_obj_t pin_sck, mp_obj_t pin_sda, mp_obj_t buf_in)
{
    value temp_val, humi_val;
    uint8_t error = 0, checksum = 0;
    mp_dht9x_t dht9x;

    dht9x.sck = mp_hal_get_pin_obj(pin_sck);
    dht9x.sda = mp_hal_get_pin_obj(pin_sda);
    
    mp_hal_pin_output(dht9x.sck);
    mp_hal_pin_open_drain(dht9x.sda);

    dht9x_bus_connectionreset(dht9x);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len < 6) {
        mp_raise_ValueError("buffer too small");
    }

    uint8_t *pbuf = bufinfo.buf;

    error += dht9x_measure(dht9x, (uint8_t*)&temp_val.i, &checksum, TEMP);
    *(pbuf + 2) = _dht9x_check_crc8(DHT9X_MEASURE_TEMP, checksum, temp_val.i);

    checksum = 0;
    error += dht9x_measure(dht9x, (uint8_t*)&humi_val.i, &checksum, HUMI);
    *(pbuf + 5) = _dht9x_check_crc8(DHT9X_MEASURE_HUMI, checksum, humi_val.i);

    _dht9x_calc_humi_temp(&humi_val.f, &temp_val.f);
    
    *(pbuf + 0) = ((uint16_t)(temp_val.f * 100) >> 8) & 0xFF;
    *(pbuf + 1) = ((uint16_t)(temp_val.f * 100) >> 0) & 0xFF;

    *(pbuf + 3) = ((uint16_t)(humi_val.f * 100) >> 8) & 0xFF;
    *(pbuf + 4) = ((uint16_t)(humi_val.f * 100) >> 0) & 0xFF;

    return MP_OBJ_NEW_SMALL_INT(error);
}
MP_DEFINE_CONST_FUN_OBJ_3(dht9x_readinto_obj, dht9x_readinto);

STATIC const mp_rom_map_elem_t dht9x_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_dht9x) },

    { MP_ROM_QSTR(MP_QSTR_reset),    MP_ROM_PTR(&dht9x_softreset_obj) },
    { MP_ROM_QSTR(MP_QSTR_readreg),  MP_ROM_PTR(&dht9x_read_statusreg_obj) },
    { MP_ROM_QSTR(MP_QSTR_writereg), MP_ROM_PTR(&dht9x_write_statusreg_obj) },
    { MP_ROM_QSTR(MP_QSTR_temp),     MP_ROM_PTR(&dht9x_measure_temp_obj) },
    { MP_ROM_QSTR(MP_QSTR_humi),     MP_ROM_PTR(&dht9x_measure_humi_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&dht9x_readinto_obj) },
};

STATIC MP_DEFINE_CONST_DICT(dht9x_module_globals, dht9x_module_globals_table);

const mp_obj_module_t mp_module_dht9x = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&dht9x_module_globals,
};

/*
[Add]
../ports/stm32/mpconfigport.h
extern const struct _mp_obj_module_t mp_module_network;
extern const struct _mp_obj_module_t mp_module_onewire;
+++ extern const struct _mp_obj_module_t mp_module_dht9x;


#define MICROPY_PORT_BUILTIN_MODULES \
    { MP_ROM_QSTR(MP_QSTR__onewire), MP_ROM_PTR(&mp_module_onewire) }, \
+++ { MP_ROM_QSTR(MP_QSTR_dht9x), MP_ROM_PTR(&mp_module_dht9x) }, \

../ports/stm32/Makefile
EXTMOD_SRC_C = $(addprefix extmod/,\
	modonewire.c \
+++	moddht9x.c \
        )

[Used]
>>> import dht9x
>>> buf = bytearray(6)
>>> dht9x.readinto('A0', 'A1', buf)

*/
