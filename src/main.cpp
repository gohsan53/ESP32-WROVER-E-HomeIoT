#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BME280.h>
#include "main.h"

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

HardwareSerial SerialDevice(2);
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_BME280 bme;

uint8_t cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};  // 0x86:Gas concentration
uint8_t reset[9] = {0xFF,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78}; // 0x87:Calibrate zero point
uint8_t calOff[9] = {0xFF,0x01,0x79,0x00,0x00,0x00,0x00,0x00,0x86}; // auto cal off

uint8_t res[9] = {};
uint8_t idx = 0;
bool flag = false;
uint16_t co2=0;

float temp;
float humid;
float pressure;

void setup()
{
  // initialize serial ports
  Serial.begin(115200);
  while(!Serial);
  SerialDevice.begin(9600);
  while(!SerialDevice);

  Serial.println("Serial begin");

  // chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
	// Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
	// Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

  /*
  With external devices 
  */
  if (gHwState == 3) {
    skLCDsetup();
    while(!bme.begin(0x76));
    co2setup();
    gState = STATE_CO2;
    gCo2Millis = 0;
  }

  // skWIFIsetup();

}

void loop()
{
  co2GetData();
  delay(1000*60*1);

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

void skLCDprintNull(void)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CO2: ");
  lcd.print("----");
  lcd.print("ppm");
  lcd.setCursor(0, 1);
  lcd.print("--.--");
  lcd.print("C ");
  lcd.print("--.--");
  lcd.print("%");
}

void skLCDprintAll(uint16_t co2concentration, float temp, float humid)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CO2: ");
  lcd.print(co2concentration);
  lcd.print("ppm");
  lcd.setCursor(0, 1);
  lcd.print(temp);
  lcd.print("C ");
  lcd.print(humid);
  lcd.print("%");
}

void co2setup(void)
{
  delay(1000);
  SerialDevice.write(calOff,9);
  Serial.print("[Info] checksum: ");
  Serial.printf("%02X\n",co2GetCheckSum(calOff));
  while(SerialDevice.available()>0)
  {
    res[idx++]=SerialDevice.read();
    flag=true;
  }
  idx = 0;
  if(flag)
  {
    flag=false;
    co2 = 0;
    co2 += (uint16_t)res[2] <<8;
    co2 += res[3];
    Serial.print("[Info] checksum: ");
    Serial.printf("%02X\n",co2GetCheckSum(res));
  }
  Serial.println(co2);
  // skLCDprint(co2);
  skLCDprintNull();

  delay(1000);
  SerialDevice.write(cmd,9);
  Serial.print("[Info] checksum: ");
  Serial.printf("%02X\n",co2GetCheckSum(cmd));
  while(SerialDevice.available()>0)
  {
    res[idx++]=SerialDevice.read();
    flag=true;
  }
  idx = 0;
  if(flag)
  {
    flag=false;
    co2 = 0;
    co2 += (uint16_t)res[2] <<8;
    co2 += res[3];
    Serial.print("[Info] checksum: ");
    Serial.printf("%02X\n",co2GetCheckSum(res));
  }
  Serial.println(co2);
  // skLCDprint(co2);

  delay(1000*60*1);
}

void co2GetData(void)
{
  SerialDevice.write(cmd,9);
  while(SerialDevice.available()>0)
  {
    res[idx++]=SerialDevice.read();
    flag=true;
  }
  idx = 0;
  if(flag)
  {
    flag=false;
    co2 = 0;
    co2 += (uint16_t)res[2] <<8;
    co2 += res[3];
    // Serial.print("[Info] checksum: ");
    // Serial.printf("%02X\n",co2GetCheckSum(res));
  }
  Serial.print("CO2  :");
  Serial.println(co2);

  temp=bme.readTemperature();
  pressure=bme.readPressure() / 100.0F;
  humid=bme.readHumidity();
  Serial.print("温度 :");
  Serial.print(temp);
  Serial.println(" °C");
  Serial.print("気圧 :");
  Serial.print(pressure);
  Serial.println(" hPa");
  Serial.print("湿度 :");
  Serial.print(humid);
  Serial.println(" %");
  Serial.println();
  delay(1000);
  skLCDprintAll(co2, temp, humid);
}

uint8_t co2GetCheckSum(uint8_t *p_packet)
{
  uint8_t i, checksum = 0;
  for(i=1; i<8; i++)
  {
    checksum += p_packet[i];
  }
  checksum = 0xff - checksum;
  checksum += 1;
  return checksum;
}