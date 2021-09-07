/*  06.010 - LCD screen with the I2C backpack demo sketch
   This sketch shows you how to use the 16x2 LCD display
   using the I2C backpack adaptor. This way, we can save a
   lot of digital pins on the Arduino.
   It is provided in this course to support projects that
   require an LCD screen for text output
   This I2C LCD backpack contains the PCF8574 port-expander
   IC. Beware that this sketch can work with backpacks that
   contains this IC, but may not work with variations.
   Components
   ----------
    - ESP32
    - An I2C to LCD backpack adaptor
    - Jumper wires
    - Breadboard
    Libraries
    ---------
    - LiquidCrystal_I2C
   Connections
   -----------
    ESP32         |    I2C backpack
    -----------------------------
        GND       |      GND
        5V        |      5V
        GPIO21    |      SDA
        GPIO22    |      SCL
   Other information
   -----------------
    1. ESP32 Datasheet: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
    2. For information on the LiquidCrystal library: https://github.com/johnrickman/LiquidCrystal_I2C
    3. If you need to run the port scanner, look for sketch 06-012 in this course repository
    Created on April 1 2019 by Peter Dalmaris
*/

#ifndef _MAIN_H
#define _MAIN_H

#include "secrets.h"

/***
 * Global State define
***/
#define STATE_NONE  0x00
#define STATE_CO2   0x01
#define STATE_LCD   0x02
#define STATE_WIFI  0x03

void changeState(int retState);

void skLCDsetup(void);
void skLCDprint(uint16_t co2concentration);
void skLCDprintNull(void);
void skLCDprintAll(uint16_t co2concentration, float temp, float humid);

void co2setup(void);
void co2GetData(void);
uint8_t co2GetCheckSum(uint8_t *p_packet);

void getTime(void);

#endif // _MAIN_H

