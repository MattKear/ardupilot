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
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>

#define ASCII_BUFFER 64 // max length of string we suport on a single line, will auto overflow if we execeed

class AP_DataLogger
{

public:
    AP_DataLogger();

    /* Do not allow copies */
    AP_DataLogger(const AP_DataLogger &other) = delete;
    AP_DataLogger &operator=(const AP_DataLogger&) = delete;

    static AP_DataLogger *get_singleton();

    // Return true if datalogger is enabled
    bool enabled() const;

    // Initialize the datalogger object and prepare it for use
    void init();

    // Update datalogging
    void update();

    // Parameter block
    static const struct AP_Param::GroupInfo var_info[];

private:

    // Parameters
    AP_Int8 _type; // type of data logger

    uint8_t _num_chars;        // number of chars we have got so far
    char _term[ASCII_BUFFER];  // buffer for the current term within the current sentence
    uint32_t _time_stamp;      // time stamp at the start of each new log item

    // Pointer to serial uart
    AP_HAL::UARTDriver *_uart = nullptr; 

    // Pointer to AP_logger
    AP_Logger *_logger = nullptr;
    
    enum Datalogger_Type {
        _DATALOGGER_NONE  = 0,
        _DATALOGGER_ASCII = 1
    };

    // Functions
    void update_ascii();
    bool decode(char c);

    static AP_DataLogger *_singleton;
};

namespace AP {
    AP_DataLogger *datalogger();
};
