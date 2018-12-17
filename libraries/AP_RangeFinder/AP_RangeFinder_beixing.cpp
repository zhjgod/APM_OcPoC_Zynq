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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_beixing.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include "../ArduCopter/utility.h"

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_beixing::AP_RangeFinder_beixing(RangeFinder &_ranger, uint8_t instance,
		RangeFinder::RangeFinder_State &_state,
		AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_beixing, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Aerotenna_beixing, 0));
    }
}

/*
   detect if a uLanding rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_beixing::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_beixing, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_beixing::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the uLanding
    float sum = 0;
    uint16_t count = 0;
    uint8_t  index = 0;

    uint8_t hdr = 85;

    int16_t nbytes = uart->available();
    hal.console->printf("\nbeixing: read%dbytes    \n", nbytes);
    while (nbytes-- > 0) {
        uint8_t c = uart->read();
        // ok, we have located start byte
        if (c == hdr && index == 0) {
            linebuf_len = 0;
            index       = 1;
        }
        // now it is ready to decode index information
        if (index == 1) {
            linebuf[linebuf_len] = c;
            linebuf_len ++;
            if (linebuf_len == 8) {
            // we have received six bytes data 
            // checksum
                if (((linebuf[1] + linebuf[2] + linebuf[3] + linebuf[4] + linebuf[5] + linebuf[6]) & 0xFF) == linebuf[7]) {
                    sum += linebuf[4]*256 + linebuf[3];
                    count ++;
                }
                index = 0;
                linebuf_len = 0;
            }
        }
    }

    if (count == 0) {
        return false;
    }

    reading_cm = sum / count;
    Utility::my_lidar_alt = reading_cm;
    hal.console->printf("\nbeixing_alt:%d  count:%d  \n", reading_cm, count);
    return true;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_beixing::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
