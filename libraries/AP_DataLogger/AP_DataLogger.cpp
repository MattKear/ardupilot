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

// Data Logger - support for logging none flight related sensores to dataflash

#include "AP_DataLogger.h"

const AP_Param::GroupInfo AP_DataLogger::var_info[] = {

    // @Param: TYPE
    // @DisplayName: Data Logger type
    // @Description: Data Logger type
    // @Values: 0:None,1:Serial ascii
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_DataLogger, _type, 0, AP_PARAM_FLAG_ENABLE),
 
    AP_GROUPEND
};

// constructor
AP_DataLogger::AP_DataLogger()
{
    AP_Param::setup_object_defaults(this, var_info);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton) {
        AP_HAL::panic("Too many DataLoggers");
    }
#endif
    _singleton = this;
}

/*
 * Get the AP_DataLogger singleton
 */
AP_DataLogger *AP_DataLogger::get_singleton()
{
    return _singleton;
}

// return true if wind vane is enabled
bool AP_DataLogger::enabled() const
{
    return _type != _DATALOGGER_NONE;
}

// Initialize the datalogger object and prepare it for use
void AP_DataLogger::init()
{
    if (_type == _DATALOGGER_NONE) {
        return;
    }

    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Data_log, 0);

    if (_uart != nullptr) {
        _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_Data_log, 0));
    }

    _logger = AP::logger().get_singleton();
}

// update datalogger, expected to be called at 10hz
void AP_DataLogger::update()
{
    if (_type == _DATALOGGER_NONE) {
        return;
    }
    if (_logger == nullptr) {
        return;
    }
    if (!_logger->should_log(1)) { // need to pick a appropriate mask value here
        return;
    }

    switch (_type) {
        case Datalogger_Type::_DATALOGGER_ASCII:
            update_ascii(); 
            break;
    }
}

// Write ascii data to data flash logs, delimited by \n and/or \r
void AP_DataLogger::update_ascii()
{
    if (_uart == nullptr) {
        return;
    }


   // read any available data
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        char c = _uart->read();
        if (decode(c)) {
            // log latest line
            _logger->Write("DATA", "TimeUS,CSV",
                        "s-", "F?", "Qz",
                        AP_HAL::micros64(),
                        _term);

            for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
                _term[i] = ' ';
            }
            _num_chars = 0;
        }
    }
}

// add a single character to the buffer and attempt to decode
// returns true if a complete sentence was successfully decoded or if the buffer is full
bool AP_DataLogger::decode(char c)
{
    if (c == '\r' || c == '\n') {
        // we have got to the end of this string
        if (_num_chars > 0) {
            return true;
        } else {
            return false;
        }
    }
    // otherwise we should add the character to the string to be logged

    if (_num_chars == 0) {
        // record the timestamp of the first char in the string
        _time_stamp = AP_HAL::millis();
    }
    
    _num_chars++;
    _term[_num_chars] = c;

    // if were going to overflow the buffer then start a new line
    if (_num_chars == ASCII_BUFFER) {
        return true;
    }

    return false;
}

AP_DataLogger *AP_DataLogger::_singleton = nullptr;

namespace AP {
    AP_DataLogger *datalogger()
    {
        return AP_DataLogger::get_singleton();
    }
};
