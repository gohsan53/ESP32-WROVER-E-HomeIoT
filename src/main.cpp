#include <Arduino.h>
// #include <wire.h>
// #include <WiFi.h>

#include <LiquidCrystal_I2C.h>
#include <MHZ.h>
#include "main.h"
// #include "skWIFI.h"


void skLCDprint(uint16_t co2concentration);
void skLCDsetup(void);

/***
 * Global State relations
 ***/
int gState = STATE_NONE;
uint32_t gStateStartMillis = 0;  // millis() of the state changeover
uint32_t gCo2Millis = 0;
uint16_t co2Concentration = 0;
int gHwState = 3;  // 0010: CO2 device run, 0001: LCD device run

uint64_t chipid;  
int retState = STATE_NONE;

HardwareSerial Serial2(2);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  // initialize serial ports
  Serial.begin(115200);
  while(!Serial);
  Serial2.begin(9600);
  while(!Serial2);

  Serial.println("Serial begin");

  // chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
	// Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
	// Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

  /*
  With external devices 
  */
  if (gHwState == 3) {
    skLCDsetup();
    co2setup();
    gState = STATE_CO2;
    gCo2Millis = 0;
  }

  // skWIFIsetup();
  delay(1000);

}

void loop()
{



}

/**
 * @brief chage state
 * @param[in] int state
 * @param[out] None
 * @param[inout] None
 * @retval int state
 *  STATE_*** : 0x**
 */
void changeState(int state) {
  gState = state;
  gStateStartMillis = millis();
}

void skLCDsetup(void)
{
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Hello world!");
  delay(1000);
}

void skLCDprint(uint16_t co2concentration)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CO2: ");
  lcd.print(co2concentration);
  lcd.print("ppm");
}