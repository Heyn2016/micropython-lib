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
#include <math.h>

#include "py/runtime.h"
#include "py/mperrno.h"
#include "py/mphal.h"
#include "extmod/machine_pulse.h"

/// Example usage:
///     import sht1x
///     dht9x = sht1x.init(pin_sck, pin_sda)

#define SHT1X_NOACK                 (0x00)
#define SHT1X_ACK                   (0x01)

                                              //ADR   CMDs   R/W
#define SHT1X_STATUS_REG_W          (0x06)    //000   0011    0
#define SHT1X_STATUS_REG_R          (0x07)    //000   0011    1
#define SHT1X_MEASURE_REG_TEMP      (0x03)    //000   0001    1
#define SHT1X_MEASURE_REG_HUMI      (0x05)    //000   0010    1
#define SHT1X_RESET                 (0x1E)    //000   1111    0

#define TEMP_8BIT_HUMI_12BIT        (0x01)
#define TEMP_12BIT_HUMI_14BIT       (0x00)    //Default

typedef struct _mp_obj_sht1x_t {
    mp_obj_base_t base;
    uint32_t us_timeout;
    mp_hal_pin_obj_t pin_sda;
    mp_hal_pin_obj_t pin_sck;
} mp_obj_sht1x_t;

enum {TEMP, HUMI};


// STATIC void mp_hal_sht1x_delay(mp_obj_sht1x_t *self) {
//     // We need to use an accurate delay to get acceptable SHT1X
//     // speeds (eg 1us should be not much more than 1us).
//     mp_hal_delay_us_fast(self->us_delay);
// }

STATIC void mp_hal_sht1x_sck_low(mp_obj_sht1x_t *self) {
    mp_hal_pin_od_low(self->pin_sck);
}

STATIC void mp_hal_sht1x_sck_high(mp_obj_sht1x_t *self) {
    mp_hal_pin_od_high(self->pin_sck);
}

STATIC void mp_hal_sht1x_sda_low(mp_obj_sht1x_t *self) {
    mp_hal_pin_od_low(self->pin_sda);
}

STATIC void mp_hal_sht1x_sda_high(mp_obj_sht1x_t *self) {
    mp_hal_pin_od_high(self->pin_sda);
}

STATIC void mp_hal_sht1x_sda_release(mp_obj_sht1x_t *self) {
    mp_hal_pin_od_high(self->pin_sda);
}

STATIC int mp_hal_sht1x_sda_read(mp_obj_sht1x_t *self) {
    return mp_hal_pin_read(self->pin_sda);
}

// Generates a transmission start 
//       _____         ________
// SDA :      |_______|
//           ___     ___
// SCK : ___|   |___|   |______

STATIC void _sht1x_bus_transstart(mp_obj_sht1x_t *self)
{
    mp_hal_sht1x_sda_release(self);  //Initial state
    mp_hal_sht1x_sck_low(self);
    mp_hal_delay_us(1);

    mp_uint_t irq_state = mp_hal_quiet_timing_enter();

    mp_hal_sht1x_sck_high(self);
    mp_hal_delay_us_fast(1);
    mp_hal_sht1x_sda_low(self);
    mp_hal_delay_us_fast(1);
    mp_hal_sht1x_sck_low(self);
    mp_hal_delay_us_fast(5);
    mp_hal_sht1x_sck_high(self);
    mp_hal_delay_us_fast(1);
    mp_hal_sht1x_sda_high(self);
    mp_hal_delay_us_fast(1);
    mp_hal_sht1x_sck_low(self);

    mp_hal_quiet_timing_exit(irq_state);
}

// Communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       _____________________________________________________         ________
// SDA :                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
STATIC void _sht1x_bus_connectionreset(mp_obj_sht1x_t *self)
{
    mp_hal_sht1x_sda_release(self);
    mp_hal_sht1x_sck_low(self);

    mp_uint_t irq_state = mp_hal_quiet_timing_enter();
    for (uint8_t i=0; i<9; i++) {
        mp_hal_sht1x_sck_high(self);
        mp_hal_delay_us_fast(1);
        mp_hal_sht1x_sck_low(self);
    }
    mp_hal_quiet_timing_exit(irq_state);

    _sht1x_bus_transstart(self);
}

//Writes a byte on the Sensibus and checks the acknowledge 
STATIC uint8_t _sht1x_bus_write_byte(mp_obj_sht1x_t *self, uint8_t value)
{
    uint8_t error = 0;

    mp_uint_t irq_state = mp_hal_quiet_timing_enter();

    for (uint8_t i=0x80; i>0; i/=2) {
        if (i & value) {
            mp_hal_sht1x_sda_high(self);
        } else {
            mp_hal_sht1x_sda_low(self);
        }
        mp_hal_delay_us_fast(1);            // Observe setup time 
        mp_hal_sht1x_sck_high(self);
        mp_hal_delay_us_fast(5);
        mp_hal_sht1x_sck_low(self);
        mp_hal_delay_us_fast(1);            // Observe hold time
    }

    mp_hal_sht1x_sda_release(self);
    mp_hal_delay_us_fast(1);                // Observe setup time
    mp_hal_sht1x_sck_high(self);
    error = mp_hal_sht1x_sda_read(self);
    mp_hal_sht1x_sck_low(self);

    mp_hal_quiet_timing_exit(irq_state);

    return error;       //error=1 in case of no acknowledge
}

// Reads a byte form the Sensibus and gives an acknowledge in case of "ack=1"
STATIC uint8_t _sht1x_bus_read_byte(mp_obj_sht1x_t *self, uint8_t ack)
{
    uint8_t value = 0;

    mp_hal_sht1x_sda_release(self);
    mp_uint_t irq_state = mp_hal_quiet_timing_enter();

    for (uint8_t i=0x80; i>0; i/=2) {
        mp_hal_sht1x_sck_high(self);
        mp_hal_delay_us_fast(1);
        if (mp_hal_sht1x_sda_read(self)) {
            value = value | i;
        }
        mp_hal_sht1x_sck_low(self);
    }

     mp_hal_quiet_timing_exit(irq_state);
    if (!ack) {
        mp_hal_sht1x_sda_high(self);
    } else {
        mp_hal_sht1x_sda_low(self);
    }

    mp_hal_delay_us_fast(1);
    mp_hal_sht1x_sck_high(self);
    mp_hal_delay_us_fast(5);
    mp_hal_sht1x_sck_low(self);
    mp_hal_delay_us_fast(1);

    mp_hal_sht1x_sda_release(self);

    return value;
}

STATIC uint8_t _sht1x_calc_crc8(uint8_t byte, uint8_t crc8) {

    uint8_t temp = 0x00;
    crc8 ^= byte;
    for (uint8_t i=0; i<8; i++) {
        temp = crc8;
        crc8 <<= 0x01;
        if ((temp & 0x80) != 0x00) {
            crc8 ^= 0x31;
        }
    }
    return crc8;
}

STATIC uint8_t _sht1x_swap_crc8(const uint8_t crc8) {

    uint8_t temp = 0x00;
    for (uint8_t i=0x80; i>0; i/=2) {
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

// return value -> ERROR : 1
//              -> OK    : 0
uint8_t _sht1x_check_crc8(const uint8_t reg, const uint8_t crc8, const unsigned short value)
{
    uint8_t crc = 0x00;

    crc = _sht1x_calc_crc8(reg, crc);
    crc = _sht1x_calc_crc8(value/256, crc);
    crc = _sht1x_calc_crc8(value%256, crc);
    crc = _sht1x_swap_crc8(crc);
    if (crc != crc8) {
        return 1;
    }
    return 0;
}

// makes a measurement (humidity/temperature) with checksum
STATIC uint8_t _sht1x_measure(mp_obj_sht1x_t *self, uint8_t *p_value, uint8_t mode) {

    uint8_t error = 0, checksum = 0;

    _sht1x_bus_transstart(self);

    switch (mode) {
        case TEMP	: error += _sht1x_bus_write_byte(self, SHT1X_MEASURE_REG_TEMP); break;
        case HUMI	: error += _sht1x_bus_write_byte(self, SHT1X_MEASURE_REG_HUMI); break;
        default     : break;
    }

#if 0
    // for (size_t i=0; i<self->us_timeout; i++) {
    //     if (mp_hal_sht1x_sda_read(self) == 0x00) {
    //         break;
    //     } else {
    //         mp_hal_delay_us_fast(1);
    //     }
    // }
    // if (mp_hal_sht1x_sda_read(self)) {
    //     error += 1;
    // }
#else
    mp_uint_t ticks = mp_hal_ticks_us();
    while (mp_hal_sht1x_sda_read(self) != 0) {
        if ((mp_uint_t)(mp_hal_ticks_us() - ticks) > self->us_timeout) {
            error += 1;
            mp_raise_OSError(MP_ETIMEDOUT);
        }
    }
#endif


    *(p_value + 1) = _sht1x_bus_read_byte(self, SHT1X_ACK);    //read the first byte (MSB) (little endian)
    *(p_value + 0) = _sht1x_bus_read_byte(self, SHT1X_ACK);    //read the second byte (LSB)
    checksum       = _sht1x_bus_read_byte(self, SHT1X_NOACK);  //read checksum

    switch (mode) {
        case TEMP	: error += _sht1x_check_crc8(SHT1X_MEASURE_REG_TEMP, checksum, *(p_value + 1)*256 + *(p_value + 0)); break;
        case HUMI	: error += _sht1x_check_crc8(SHT1X_MEASURE_REG_HUMI, checksum, *(p_value + 1)*256 + *(p_value + 0)); break;
        default     : break;
    }

    return error;
}

/*
    VDD     d1(C)   d1(F)    SOt     d2(C)   d2(F)
    5V     -40.1   -40.2    14bit    0.01    0.018
    4V     -39.8   -39.6    12bit    0.04    0.072
    3.5V    -39.7   -39.5
    3V     -39.6   -39.3
    2.5V    -39.4   -38.9

    Temp = d1 + d2*SOt
*/
/*
    SOrh       C1      C2          C3           t1      t2
   12bit    -2.0468  0.0362   -0.0000015955    0.01   0.00008
    8bit    -2.0468  0.5872   -0.0000040845    0.01   0.00128

    RHlinear = C1 + C2*SOrh + C3*SOrh*SOrh
    Humi     = (Temp - 25)*(t1 + t2*SOrh) + RHlinear
*/
STATIC void _sht1x_calc_humi_temp(float *p_humidity ,float *p_temperature) {
    const float D1 = -40.1;
    const float D2 = +0.01;
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

    t_C = t*D2 - D1;
    rh_lin = C3*rh*rh + C2*rh + C1;
    rh_true = (t_C-25)*(T1+T2*rh) + rh_lin;
    if (rh_true > 100) rh_true = 100;
    if (rh_true < 0.1) rh_true = 0.1;

    *p_temperature = t_C;
    *p_humidity = rh_true;
}

/******************************************************************************/
// MicroPython bindings


STATIC void mp_sht1x_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    mp_obj_sht1x_t *self = self_in;
    (void)kind;

    mp_printf(print, "<SCK Port %d> \r\n", self->pin_sck->port);
    mp_printf(print, "<SCK Pin  %d> \r\n", self->pin_sck->pin);
    mp_printf(print, "<SCK Mode %d> \r\n", pin_get_mode(self->pin_sck));

    mp_printf(print, "<SDA Port %d> \r\n", self->pin_sda->port);
    mp_printf(print, "<SDA Pin  %d> \r\n", self->pin_sda->pin);
    mp_printf(print, "<SDA Mode %d> \r\n", pin_get_mode(self->pin_sda));

    mp_printf(print, "<Timeout(us) %d> \r\n", self->us_timeout);
}

STATIC void mp_sht1x_obj_init_helper(mp_obj_sht1x_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    enum { ARG_sck, ARG_sda, ARG_timeout };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_sck, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_sda, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 10000000} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    self->pin_sck = mp_hal_get_pin_obj(args[ARG_sck].u_obj);
    self->pin_sda = mp_hal_get_pin_obj(args[ARG_sda].u_obj);
    self->us_timeout = args[ARG_timeout].u_int;

    // init the pins to be push/pull outputs
    mp_hal_pin_open_drain(self->pin_sck);
    mp_hal_pin_open_drain(self->pin_sda);

    // mp_hal_pin_config(sht1x->pin_sck, MP_HAL_PIN_MODE_OUTPUT, MP_HAL_PIN_PULL_UP, 0);
    // mp_hal_pin_config(sht1x->pin_sda, MP_HAL_PIN_MODE_ALT_OPEN_DRAIN, MP_HAL_PIN_PULL_NONE, 0);

    mp_hal_delay_ms(5);
    _sht1x_bus_connectionreset(self);
}

STATIC mp_obj_t mp_sht1x_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 2, 3, false);

    // create sht1x object
    mp_obj_sht1x_t *self = m_new_obj(mp_obj_sht1x_t);
    self->base.type = type;

    mp_map_t kw_args;
    mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
    mp_sht1x_obj_init_helper(self, n_args, args, &kw_args);

    return (mp_obj_t)self;
}

// Resets the sensor by a softreset 
STATIC mp_obj_t sht1x_softreset(mp_obj_t self_in) {
 
    uint8_t error = 0;
    mp_obj_sht1x_t *self = MP_OBJ_TO_PTR(self_in);

    _sht1x_bus_connectionreset(self);
    error = _sht1x_bus_write_byte(self, SHT1X_RESET);

    return MP_OBJ_NEW_SMALL_INT(error);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sht1x_softreset_obj, sht1x_softreset);

// Reads the status register with checksum (8-bit)
// 
STATIC mp_obj_t sht1x_read_statusreg(mp_obj_t self_in, mp_obj_t buf_in) {
 
    mp_buffer_info_t bufinfo;
    uint8_t error = 0, checksum = 0;
    mp_obj_sht1x_t *self = MP_OBJ_TO_PTR(self_in);

    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len < 2) {
        mp_raise_ValueError("Buffer too small, must be >=2 bytearray");
    }

    _sht1x_bus_transstart(self);    //transmission start
    error += _sht1x_bus_write_byte(self, SHT1X_STATUS_REG_R);  //send command to sensor

    uint8_t *buf = bufinfo.buf;
    *(buf + 0) = _sht1x_bus_read_byte(self, SHT1X_ACK);       //read status register (8-bit)
    checksum   = _sht1x_bus_read_byte(self, SHT1X_NOACK);     //read checksum (8-bit)

    error += _sht1x_check_crc8(SHT1X_STATUS_REG_R, checksum, buf[0]);

    return MP_OBJ_NEW_SMALL_INT(error);   //error=1 in case of no response form the sensor
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sht1x_read_statusreg_obj, sht1x_read_statusreg);

// Writes the status register with checksum (8-bit)
//
STATIC mp_obj_t sht1x_write_statusreg(mp_obj_t self_in, mp_obj_t value_in) { 
    
    uint8_t error = 0;
    mp_int_t value = mp_obj_get_int(value_in);
    mp_obj_sht1x_t *self = MP_OBJ_TO_PTR(self_in);

    _sht1x_bus_transstart(self);    //transmission start
    error += _sht1x_bus_write_byte(self, SHT1X_STATUS_REG_W);
    error += _sht1x_bus_write_byte(self, (uint8_t)(value & 0xFF));

    return MP_OBJ_NEW_SMALL_INT(error);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sht1x_write_statusreg_obj, sht1x_write_statusreg);

// Makes a measurement temperature with checksum
STATIC mp_obj_t sht1x_measure_temp(mp_obj_t self_in, mp_obj_t buf_in) {
    
    uint8_t error = 0, checksum = 0;
    mp_obj_sht1x_t *self = MP_OBJ_TO_PTR(self_in);


    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len < 2) {
        mp_raise_ValueError("Buffer too small, must be >=2 bytearray");
    }
    uint8_t *pbuf = bufinfo.buf;

    _sht1x_bus_transstart(self);
    error = _sht1x_bus_write_byte(self, SHT1X_MEASURE_REG_TEMP);

    for (size_t i=0; i<self->us_timeout; i++) {
        if (mp_hal_sht1x_sda_read(self) == 0x00) {
            break;
        } else {
            mp_hal_delay_us_fast(1);
        }
    }

    if (mp_hal_sht1x_sda_read(self)) {
        error += 1;
    }

    pbuf[0] = _sht1x_bus_read_byte(self, SHT1X_ACK);    //read the first byte (MSB)
    pbuf[1] = _sht1x_bus_read_byte(self, SHT1X_ACK);    //read the second byte (LSB)

    checksum   = _sht1x_bus_read_byte(self, SHT1X_NOACK);

    error += _sht1x_check_crc8(SHT1X_MEASURE_REG_TEMP, checksum, pbuf[0]*256 + pbuf[1]);

    // return mp_obj_new_float((p_value[0]*256 + p_value[1])/100);
    return MP_OBJ_NEW_SMALL_INT(error);
}
MP_DEFINE_CONST_FUN_OBJ_2(sht1x_measure_temp_obj, sht1x_measure_temp);

// Makes a measurement humidity with checksum
STATIC mp_obj_t sht1x_measure_humi(mp_obj_t self_in, mp_obj_t buf_in) {
    
    uint8_t error = 0, checksum = 0;
    mp_obj_sht1x_t *self = MP_OBJ_TO_PTR(self_in);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len < 2) {
        mp_raise_ValueError("Buffer too small, must be >=2 bytearray");
    }
    uint8_t *pbuf = bufinfo.buf;

    _sht1x_bus_transstart(self);
    error = _sht1x_bus_write_byte(self, SHT1X_MEASURE_REG_HUMI);

    for (size_t i=0; i<self->us_timeout; i++) {
        if (mp_hal_sht1x_sda_read(self) == 0x00) {
            break;
        } else {
            mp_hal_delay_us_fast(1);
        }
    }

    if (mp_hal_sht1x_sda_read(self)) {
        error += 1;
    }

    pbuf[0] = _sht1x_bus_read_byte(self, SHT1X_ACK);    //read the first byte (MSB)
    pbuf[1] = _sht1x_bus_read_byte(self, SHT1X_ACK);    //read the second byte (LSB)

    pbuf[0] = 0x01;
    pbuf[1] = 0x02;

    checksum   = _sht1x_bus_read_byte(self, SHT1X_NOACK);

    error += _sht1x_check_crc8(SHT1X_MEASURE_REG_TEMP, checksum, pbuf[0]*256 + pbuf[1]);

    // return mp_obj_new_float((p_value[0]*256 + p_value[1])/100);
    return MP_OBJ_NEW_SMALL_INT(error);
}
MP_DEFINE_CONST_FUN_OBJ_2(sht1x_measure_humi_obj, sht1x_measure_humi);

// Makes a measurement temperature and humidity.
STATIC mp_obj_t sht1x_readinto(mp_obj_t self_in, mp_obj_t buf_in) {
    uint8_t error = 0;
    mp_obj_sht1x_t *self = MP_OBJ_TO_PTR(self_in);

    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf_in, &bufinfo, MP_BUFFER_WRITE);
    if (bufinfo.len < 4) {
        mp_raise_ValueError("Buffer too small, must be >=4 bytearray");
    }
    
    unsigned short temp_val = 0, humi_val = 0;

    error += _sht1x_measure(self, (uint8_t *)&temp_val, TEMP);
    error += _sht1x_measure(self, (uint8_t *)&humi_val, HUMI);

    float temperature = 0.0, humidity = 0.0;
    _sht1x_calc_humi_temp(&humidity, &temperature);

    printf("Temp = %f\r\n", (double)temperature);
    printf("Humi = %f\r\n", (double)humidity);


    ((uint8_t *)bufinfo.buf)[0] = (unsigned short)(temperature*100) & 0xFF;
    ((uint8_t *)bufinfo.buf)[1] = (unsigned short)(temperature*100) / 256;
    ((uint8_t *)bufinfo.buf)[2] = (unsigned short)( humidity*100  ) & 0xFF;
    ((uint8_t *)bufinfo.buf)[3] = (unsigned short)( humidity*100  ) / 256;

    return MP_OBJ_NEW_SMALL_INT(error);
}

MP_DEFINE_CONST_FUN_OBJ_2(sht1x_readinto_obj, sht1x_readinto);

// Calculates dew point
// humidity [%RH], temperature [C]
STATIC mp_obj_t sht1x_dewpoint(mp_obj_t self_in, mp_obj_t temp_in, mp_obj_t humi_in) {
    
    // static const double _M_LN10 = 2.302585092994046;
    // float temp = mp_obj_get_float(temp_in);
    // float humi = mp_obj_get_float(humi_in);
    float dew_point = 0.0;

    // logex = 0.66077 + 7.5*temp/(237.3 + temp) + (float)(log(humi) / (double)_M_LN10 - 2);
    // dew_point = (logex - 0.66077)*237.3/(0.66077+7.5 - logex);

    return mp_obj_new_float(dew_point);
}
MP_DEFINE_CONST_FUN_OBJ_3(sht1x_dewpoint_obj, sht1x_dewpoint);

///////////////////////////////////////////////////////////////////////////////////////////////////
// Test
STATIC mp_obj_t sht1x_test_sda(mp_obj_t self_in, mp_obj_t value_in, mp_obj_t time_in) { 
    
    uint8_t error = 0;
    mp_int_t value = mp_obj_get_int(value_in);
    mp_int_t delay = mp_obj_get_int(time_in);
    mp_obj_sht1x_t *self = MP_OBJ_TO_PTR(self_in);

    if (value > 0) {
        mp_hal_sht1x_sda_high(self);
    } else {
        mp_hal_sht1x_sda_low(self);
    }

    mp_hal_delay_us_fast(delay);
    error = mp_hal_sht1x_sda_read(self);

    return MP_OBJ_NEW_SMALL_INT(error);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(sht1x_test_sda_obj, sht1x_test_sda);
///////////////////////////////////////////////////////////////////////////////////////////////////


STATIC const mp_rom_map_elem_t sht1x_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&sht1x_softreset_obj) },
    { MP_ROM_QSTR(MP_QSTR_readreg),  MP_ROM_PTR(&sht1x_read_statusreg_obj) },
    { MP_ROM_QSTR(MP_QSTR_writereg), MP_ROM_PTR(&sht1x_write_statusreg_obj) },

    { MP_ROM_QSTR(MP_QSTR_temp), MP_ROM_PTR(&sht1x_measure_temp_obj) },
    { MP_ROM_QSTR(MP_QSTR_humi), MP_ROM_PTR(&sht1x_measure_humi_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&sht1x_readinto_obj) },

    { MP_ROM_QSTR(MP_QSTR_dewpoint), MP_ROM_PTR(&sht1x_dewpoint_obj) },

    { MP_ROM_QSTR(MP_QSTR_sda), MP_ROM_PTR(&sht1x_test_sda_obj) },
};

STATIC MP_DEFINE_CONST_DICT(sht1x_locals_dict, sht1x_locals_dict_table);

STATIC const mp_obj_type_t mp_type_sht1x = {
    { &mp_type_type },
    .name = MP_QSTR_init,
    .print = mp_sht1x_print,
    .make_new = mp_sht1x_make_new,
    .locals_dict = (mp_obj_dict_t*)&sht1x_locals_dict,
};


STATIC const mp_rom_map_elem_t sht1x_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_sht1x) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mp_type_sht1x) },
    
    { MP_ROM_QSTR(MP_QSTR_T08H12BIT), MP_ROM_INT(TEMP_8BIT_HUMI_12BIT) },
    { MP_ROM_QSTR(MP_QSTR_T12H14BIT), MP_ROM_INT(TEMP_12BIT_HUMI_14BIT) },
};

STATIC MP_DEFINE_CONST_DICT(sht1x_module_globals, sht1x_module_globals_table);

const mp_obj_module_t mp_module_sht1x = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&sht1x_module_globals,
};


/*
[Add]
../ports/stm32/mpconfigport.h
extern const struct _mp_obj_module_t mp_module_network;
extern const struct _mp_obj_module_t mp_module_onewire;
+++ extern const struct _mp_obj_module_t mp_module_sht1x;


#define MICROPY_PORT_BUILTIN_MODULES \
    { MP_ROM_QSTR(MP_QSTR__onewire), MP_ROM_PTR(&mp_module_onewire) }, \
+++ { MP_ROM_QSTR(MP_QSTR_sht1x), MP_ROM_PTR(&mp_module_sht1x) }, \

../ports/stm32/Makefile
EXTMOD_SRC_C = $(addprefix extmod/,\
	modonewire.c \
+++	modsht1x.c \
        )

[Used]
>>> import sht1x
>>> buf = bytearray(6)
>>> sht1x.readinto('C0', 'C1', buf)
>>> sht1x.temp('C0', 'C1', buf)

>>> ss = sht1x.init('E3', 'E1')
>>> ss.readinto(buf, 0)
*/
