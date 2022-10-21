#include <Arduino.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BME280.h>
#include <Ambient.h>
#include <time.h>
#include "main.h"

#define JST 60*60*9

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
WiFiClient client;
Ambient ambient;

uint8_t cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};  // 0x86:Gas concentration
uint8_t reset[9] = {0xFF,0x01,0x87,0x00,0x00,0x00,0x00,0x00,0x78}; // 0x87:Calibrate zero point
uint8_t calOff[9] = {0xFF,0x01,0x79,0x00,0x00,0x00,0x00,0x00,0x86}; // auto cal off

uint8_t res[9] = {};
uint8_t idx = 0;
bool flag = false;
uint16_t co2=0;

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
    delay(500);
    skLCDsetup();
    while(!bme.begin(0x76));
    co2setup();
    gState = STATE_CO2;
    gCo2Millis = 0;
  }

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }
  Serial.println();
  Serial.print("[INFO] WiFi connected.\r\n[INFO] IP address: ");
  Serial.println(WiFi.localIP());
  configTime(JST, 0, "ntp.nict.jp", "ntp.jst.mfeed.ad.jp");
  ambient.begin(CHANNEL_ID, WRITE_KEY, &client);
  getTime();
}

void loop()
{
  static uint8_t cnt = 0;
  if (cnt >= 60) {
    cnt = 0;
    getTime();
  }
  co2GetData(cnt);
  delay(1000*60*1);
  cnt++;
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
}

// void skLCDprint(uint16_t co2concentration)
// {
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   // lcd.print("CO2: ");
//   lcd.print(co2concentration);
//   lcd.print("ppm");
// }

void skLCDprintNull(void)
{
  lcd.clear();
  lcd.setCursor(2, 0);
  // lcd.print("CO2: ");
  lcd.print("----");
  lcd.print("ppm ");
  lcd.print("--.--");
  lcd.print("G ");
  lcd.setCursor(1, 1);
  lcd.print("--.--");
  lcd.print("C   ");
  lcd.print("--.--");
  lcd.print("%");
}

void skLCDprintAll(uint16_t co2concentration, float temp, float humid, float aHumid)
{
  lcd.clear();
  lcd.setCursor(2, 0);
  // lcd.print("CO2: ");
  if(co2concentration < 1000) lcd.print(" ");
  lcd.print(co2concentration);
  lcd.print("ppm ");
  if(aHumid < 10) lcd.print(" ");
  lcd.print(aHumid);
  lcd.print("G ");
  lcd.setCursor(1, 1);
  lcd.print(temp);
  lcd.print("C   ");
  lcd.print(humid);
  lcd.print("%");
}

void co2setup(void)
{
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

  delay(500);

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
}

void co2GetData(int cnt)
{
  float temp = 0.0;
  float humid = 0.0;
  float pressure = 0.0;

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
  Serial.print("　　 CO2 :");
  Serial.print(co2);
  Serial.println(" ppm");

  temp=bme.readTemperature();
  humid=bme.readHumidity();
  pressure=bme.readPressure() / 100.0F;

  if (cnt != 0 && cnt%5 == 0) {
    ambient.set(1, co2);
    ambient.set(2, temp);
    ambient.set(3, humid);
    ambient.set(4, pressure);
    ambient.send();
    Serial.print("\r\n[ambient] data is sent\r\n\r\n");
  }

  float aHumid = calcAbsoluteHumidity(temp, humid);

  Serial.print("　　温度 :");
  Serial.print(temp);
  Serial.println(" °C");
  Serial.print("　　気圧 :");
  Serial.print(pressure);
  Serial.println(" hPa");
  Serial.print("　　湿度 :");
  Serial.print(humid);
  Serial.println(" %");
  Serial.print("絶対湿度 :");
  Serial.print(aHumid);
  Serial.print(" g");
  Serial.println();
  skLCDprintAll(co2, temp, humid, aHumid);
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

void getTime(void)
{
  time_t t;
  struct tm *tm;
  static const char *wd[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

  t = time(NULL);
  tm = localtime(&t);
  Serial.printf(" %04d/%02d/%02d(%s) %02d:%02d:%02d\n",
    tm->tm_year+1900,
    tm->tm_mon+1,
    tm->tm_mday,
    wd[tm->tm_wday],
    tm->tm_hour,
    tm->tm_min,
    tm->tm_sec);
}

float calcAbsoluteHumidity(float temp, float humid) {
  
  // saturation water vapor pressure
  float swvp = 6.1078 * exp(log(10)*((7.5*temp)/(237.3+temp)));

  // saturation water vapor density
  float swvd = (217*swvp)/(temp+273.15);
  
  // absolute humidity
  float ah = swvd * humid/100;
  
  return ah;
}
