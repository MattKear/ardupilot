/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_ToughSonic.h"

#if AP_RANGEFINDER_TOUGHSONIC_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/AP_Math.h>
#include <ctype.h>
#include <AP_Math/crc.h>

extern const AP_HAL::HAL& hal;

void AP_RangeFinder_ToughSonic::init_serial(uint8_t serial_instance)
{
    // Run base class config 
    AP_RangeFinder_Backend_Serial::init_serial(serial_instance);

    // Set flow control and inversion
    if (uart != nullptr) {
        uart->set_unbuffered_writes(true);
        uart->set_flow_control(AP_HAL::UARTDriver::flow_control::FLOW_CONTROL_RTS_DE);
    }
}


// distance returned in reading_m, set to true if sensor reports a good reading
bool AP_RangeFinder_ToughSonic::get_reading(float &reading_m)
{
    if (uart == nullptr) {
        return false;
    }

    // Send command to read
    send_read_holding_registers(sensor_ID, 0x0208, 1);

    // Try and read data a few times, this could be a while loop, using a for loop gives a upper bound to run time
    bool ret = false;
    for (uint8_t attempts = 0; attempts < sizeof(buffer) * 4; attempts++) {

        uint32_t n = uart->available();
        if (n == 0) {
            // No data available
            break;
        }
        if (buffer_offset < sizeof(buffer)) {
            // Read enough bytes to fill buffer
            ssize_t nread = uart->read(&buffer[buffer_offset], MIN(n, unsigned(sizeof(buffer)-buffer_offset)));
            if (nread <= 0) {
                // Read failed
                break;
            }
            buffer_offset += nread;
        }

        // Reply should start with the address of the sensor
        if (buffer[0] != sensor_ID) {

            // Search through the buffer and find a valid start byte
            uint8_t *p = (uint8_t *)memchr(&buffer[1], sensor_ID, buffer_offset-1);
            if (p) {
                uint8_t newlen = buffer_offset - (p - buffer);
                memmove(&buffer[0], p, newlen);
                buffer_offset = newlen;
            } else {
                buffer_offset = 0;
            }

            continue;
        }

        if (buffer_offset < sizeof(buffer)) {
            // Not enough data to make up packet
            continue;
        }

        // Expecting a reply to the function we requested
        if ((buffer[1] == 0x03) && (buffer[2] == 0x02)) {
            // read_holding_registers response with two bytes
            // check crc
            const uint16_t crc = calc_crc_modbus(buffer, 5);
            if ((buffer[5] == LOWBYTE(crc)) && (buffer[6] == HIGHBYTE(crc))) {
                // CRC matches, valid response
                uint16_t count = UINT16_VALUE(buffer[3], buffer[4]);
                reading_m = count * 0.085954 * 0.001;
                ret = true;

                // Zero offset and see if there is any more data
                buffer_offset = 0;
                continue;
            }
        }

        // Probably lost sync shift by one and try again
        memmove(&buffer[0], &buffer[1], buffer_offset - 1);
        buffer_offset -= 1;
    }

    return ret;
}

void AP_RangeFinder_ToughSonic::send_read_holding_registers(const uint8_t ID, const uint16_t start_address, const uint16_t count)
{
    uint8_t data[] {
        ID,
        0x03,
        HIGHBYTE(start_address),
        LOWBYTE(start_address),
        HIGHBYTE(count),
        LOWBYTE(count),
        0, // crc low
        0  // crc high
    };

    const uint16_t crc = calc_crc_modbus(data, 6);
    data[6] = LOWBYTE(crc);
    data[7] = HIGHBYTE(crc);

    uart->write(data, sizeof(data));
}

#endif // AP_RANGEFINDER_TOUGHSONIC_ENABLED
