/*
   Generic RGBLed driver
*/

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

*/


#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include "MyLed.h"
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

// update - updates led according to timed_updated.  Should be called
// at 50Hz
void RGBLed::test_led()
{
    hal.scheduler->delay(pNotify->_led_deatime);
    const uint8_t brightness = _led_bright;
    if(l_j==0){
    hal.uartE->printf("0\r\n");
    _red_des = brightness;
    _blue_des = _led_off;
    _green_des = _led_off;
    l_j = 1;
    }else if(l_j ==1){
        _red_des = _led_off;
        _blue_des = brightness;
        _green_des = _led_off;
        l_j = 2;

        }else if(l_j == 2){
            _red_des = _led_off;
        _blue_des = _led_off;
        _green_des = brightness;
        l_j = 0;
}
}

