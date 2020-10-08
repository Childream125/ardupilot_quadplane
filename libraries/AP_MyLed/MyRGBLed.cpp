/*
   Generic MyRGBLed driver
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
#include "AP_MyLed.h"
#include "MyRGBLed.h"

extern const AP_HAL::HAL& hal;

MyRGBLed::MyRGBLed(uint8_t led_off, uint8_t led_bright, uint8_t led_medium, uint8_t led_dim):
    _led_off(led_off),
    _led_bright(led_bright),
    _led_medium(led_medium),
    _led_dim(led_dim)
{

}    

bool MyRGBLed::init()
{
    return hw_init();
}

// set_rgb - set color as a combination of red, green and blue values
void MyRGBLed::_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (red != _red_curr ||
        green != _green_curr ||
        blue != _blue_curr) {
        // call the hardware update routine
        if (hw_set_rgb(red, green, blue)) {
            _red_curr = red;
            _green_curr = green;
            _blue_curr = blue;
        }
    }
}


// set_rgb - set color as a combination of red, green and blue values
void MyRGBLed::set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    _set_rgb(red, green, blue);
}


// update - updates led according to timed_updated.  Should be called
// at 50Hz
void MyRGBLed::update()
{
    static uint8_t num= 0;
    num++;
    if(num>=100){
    const uint8_t brightness = _led_bright;
        if(l_j==0){
            //hal.uartE->printf("0\r\n");
            _red_des = brightness;
            _blue_des = _led_off;
            _green_des = _led_off;
            l_j = 1;
        }else if(l_j ==1)
        {
            //hal.uartE->printf("1\r\n");
            _red_des = _led_off;
            _blue_des = brightness;
            _green_des = _led_off;
            l_j = 2;
        }else if(l_j == 2)
        {
            //hal.uartE->printf("2\r\n");
            _red_des = _led_off;
            _blue_des = _led_off;
            _green_des = brightness;
            l_j = 0;
        }
        num = 0;
    }
    set_rgb(_red_des, _green_des, _blue_des);
}
