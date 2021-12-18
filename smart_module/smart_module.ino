#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include <Esp.h>
#include <mDNS.h>
#include <ESP32Ping.h>
#include <SPI.h>
#include <EEPROM.h>
#include <simpleDSTadjust.h>
#include <Nextion.h> 
#include <BlynkSimpleEsp32.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <RF24.h> 
#include <RF24_config.h> 
#include <nRF24L01.h> 
#include "OpenWeatherMapCurrent.h"
#include "OpenWeatherMapForecast.h"
#include "Astronomy.h"

/** Communication definitions */
const char auth[] = "XXXXXXX";     			    /* Your Blynk auth */
const char ssid[] = "XXXXXXX";                              /* Your WiFi ssid */
const char pass[] = "XXXXXXX";                        	    /* Your WiFi pass */
const String hostname = "ESP32 Smart Home";                 /* WiFi hostname */
bool pingTest = 0;                                          /* Internet access control test */
bool blynkCSFlag = 0;                                       /* Blynk connection status flag */
bool WiFiSttFlag = 1;                                       /* WiFi State flag */  
bool BlynkSttFlag = 1;                                      /* Blynk state flag */

#define BLYNK_PRINT Serial
//BlynkTimer timer;

/** Timer 0 setup */
hw_timer_t * timer = NULL;

/** Adjust according to your language */                                                                               
const String WDAY_NAMES[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};                                      
const String MONTH_NAMES[] = {"JAN", "FEB", "MAR", "APR", "MAY", "JUN", "JUL", "AUG", "SEP", "OCT", "NOV", "DEC"};  
const String SUN_MOON_TEXT[] = {"Sonne", "Auf", "Unter", "Mond", "Age", "Illum"};                                   
const String MOON_PHASES[] = {"New Moon", "Waxing Crescent", "First Quarter", "Waxing Gibbous",                     
                              "Full Moon", "Waning Gibbous", "Third quarter", "Waning Crescent"};                   
                                                                                               
#define UTC_OFFSET +3                                                                                               
struct dstRule StartRule = {"CEST", Last, Sun, Mar, 2, 3600}; /* Central European Summer Time = UTC/GMT +2 hours */   
struct dstRule EndRule = {"CET", Last, Sun, Oct, 2, 0};       /* Central European Time = UTC/GMT +1 hour */           
                                                                                                                    
// Settings for Boston                                                                                              
// #define UTC_OFFSET -5                                                                                            
// struct dstRule StartRule = {"EDT", Second, Sun, Mar, 2, 3600}; // Eastern Daylight time = UTC/GMT -4 hours       
// struct dstRule EndRule = {"EST", First, Sun, Nov, 1, 0};       // Eastern Standard time = UTC/GMT -5 hour        
                                                                                                                    
/** Values in metric or imperial system? */                                                                            
const boolean IS_METRIC = true; 

/** Timezone selection variable */
int8_t timeZone = 3; /* Default time zone */
                                                                                                                    
/** Change for 12 Hour/ 24 hour style clock */                                                                         
bool IS_STYLE_12HR = false;                                                                                         
                                                                                                                    
/** Change for different ntp (time servers) */                                                                          
#define NTP_SERVERS "0.ch.pool.ntp.org", "1.ch.pool.ntp.org", "2.ch.pool.ntp.org"        

/** OpenWeatherMap Settings                                                                                                                 
 *  Sign up here to get an API key: https://docs.thingpulse.com/how-tos/openweathermap-key/            
 */
String OPEN_WEATHER_MAP_APP_ID = "2b998ea2f1b81d537adb8b5b270d13bd";                                  
/*                                                                                                    
    Go to https://openweathermap.org/find?q= and search for a location. Go through the                
    result set and select the entry closest to the actual location you want to display                
    data for. It'll be a URL like https://openweathermap.org/city/2657896. The number                 
    at the end is what you assign to the constant below.                                             
*/  
                                                                                                 
String OPEN_WEATHER_MAP_LOCATION_ID = "749778"; /* OpenWeatherMap location ID */          
String DISPLAYED_CITY_NAME = "Canakkale";                                                             
                                                                                                      
/*                                                                                                    
    Arabic -> ar, Bulgarian -> bg, Catalan -> ca, Czech -> cz, German -> de, Greek -> el,             
    English -> en, Persian (Farsi) -> fa, Finnish -> fi, French -> fr, Galician -> gl,                
    Croatian -> hr, Hungarian -> hu, Italian -> it, Japanese -> ja, Korean -> kr,                     
    Latvian -> la, Lithuanian -> lt, Macedonian -> mk, Dutch -> nl, Polish -> pl,                       
    Portuguese -> pt, Romanian -> ro, Russian -> ru, Swedish -> se, Slovak -> sk,                     
    Slovenian -> sl, Spanish -> es, Turkish -> tr, Ukrainian -> ua, Vietnamese -> vi,                 
    Chinese Simplified -> zh_cn, Chinese Traditional -> zh_tw.                                        
*/                                                                                                    
                                                                                                      
String OPEN_WEATHER_MAP_LANGUAGE = "en";   

const uint8_t MAX_FORECASTS = 9; /* Must be >= 9 */

String getTime(time_t *timestamp);
time_t dstOffset = 0;
simpleDSTadjust dstAdjusted(StartRule, EndRule);
OpenWeatherMapCurrentData currentWeather;             
OpenWeatherMapForecastData forecasts[MAX_FORECASTS];
Astronomy::MoonData moonData;                         
String moonAgeImage = "";
uint8_t moonAge = 0;
uint8_t forecastUpdateCount = 0;

/** ESP32 pin definitions */
#define buzzer            25    /* GPIO25 Alarm buzzer pin */
#define relay             12    /* GPIO12 Combi relay pin */
#define intMSensor        36    /* GPIO36 Internal motion sensor pin */
#define blynkLed          22    /* GPIO22 Blynk status led pin */
#define heatingLed         4    /* GPIO4 Combi heating led pin */

/** DHT11 initial designations */
#define DHTPIN 2          /* Digital pin connected to the DHT sensor */
#define DHTTYPE DHT11     /* Model type DHT11 */
DHT dht(DHTPIN, DHTTYPE);

/** ESP32 pin connection with RF24 module */
RF24 radio(21, 5);  /* CE-->GPIO17, CSN-->GPIO05 */

/** Channel selection to communicate with external RF24 modules 
*   { RF sensor channel array variables -->> 0th: Motion 1st: Gas, 2nd: Smoke, 3rd: Door, 4th: Temp & Hum }
*/
const uint8_t CH[5] = {0, 1, 2, 3, 4};

/** Address selection to communicate with external RF24 modules 
 *  { RF sensor address array variables -->> 0th: Motion, 1st: Gas, 2nd: Somoke, 3rd: Door, 4th: Temp & Hum }
 */
const uint8_t ADD[][6] = {"00001", "00002", "00003", "00004", "00001"};


/** Alarm data array for external RF24 modules 
 *  { RF sensor alarm array variables -->> 0th: Motion, 1st: Gas, 2nd: Smoke, 3rd: Door } 
*/
int RFA[4] = {10, 20, 30, 40};

/** RF module TH package data */
typedef struct _data{              
  float t;  /* Outdoor temperature (x10), 12 bits */                                
  float h;  /* Outdoor humidity */                                   
};
_data data;

/** DS18B20 initial designations */
OneWire oneWire(32);  /* Onewire communication pin selected */
DallasTemperature DS18B20(&oneWire);                  
DeviceAddress DS18B20address;


/** EEPROM data address length 256 bytes */
#define EEPROM_SIZE 255

float indoorTemp, outdoorTemp;
int indoorHumidity, outdoorHumidity;

/** Alarm flags 
 * { lRLSttFlg ->  Livingroom light state flag }
 * { kLSttFlag ->  Kitchen light state flag }
 * { bRLSttFlag -> Bedroom light state flag }
 * { kRLSttFlag -> Kidsroom light state flag }
 * { gLSttFlag ->  Garage light state flag }
 */
bool lRLSttFlag = 0, kLSttFlag = 0, bRLSttFlag = 0, kRLSttFlag = 0, gLSttFlag = 0;

/** Alarm flags 
 * { mASttFlag -> Motion alarm state flag }
 * { gASttFlag -> Gas alarm state flag }
 * { sASttFlag -> Smoke alarm state flag }
 * { dASttFlag -> Door alarm state flag }
 */
bool mASttFlag, gASttFlag, sASttFlag, dASttFlag;

/** RF module alarm information structure 
 * { Motioan sensor (MAlarm):  10 -> No alarm, 11 -> Alarm, 12 -> Lock alarm }
 * { Gas sensor (GAlarm):      20 -> No alarm, 21 -> Alarm, 22 -> Lock alarm }
 * { Smoke sensor (SAlarm):    30 -> No alarm, 31 -> Alarm, 32 -> Lock alarm }
 * { Door sensor (DAlarm):     40 -> No alarm, 41 -> Alarm, 42 -> Lock alarm }
 */
typedef struct _rf24A {

  int16_t MAlarm = 10; /* RF24 motion alarm data */
  int16_t GAlarm = 20; /* RF24 gas alarm data */
  int16_t SAlarm = 30; /* RF24 smoke alarm data */
  int16_t DAlarm = 40; /* RF24 door alarm data */    
};
_rf24A rf;

/** Sensor temperature and humidity data structure */
typedef struct {
  float inT;      /* Indoor temperature, 12 bits */
  float inH;      /* Indoor humidity */
  float outT;     /* Outdoor temperature,12 bits */
  float outH;     /* Outdoor humidity */
}TH;
TH th;

/** Sensor errors structure */
typedef struct {
  /* { 0: No error, 1: Error } */
  bool inTS = 0;       /* Indoor temperature sensor error variable */
  bool inHS = 0;       /* Indoor humidity sensor error variable */
  bool outTS = 0;      /* Outdoor temperature sensor error variable */
  bool outHS = 0;      /* Outdoor humidity sensor error variable */
}THS_ERR;
THS_ERR thsERR;

/** CPU task definitions */
TaskHandle_t Task0;
TaskHandle_t Task1;

/** Combi variable definitions structure */
typedef struct {
  bool SttFlag = 0;     /* Combi On/Off state flag  { 0: off, 1: on } */
  bool HFlag = 0;       /* Combi heating status flag { 0: passive, 1: active } */
  bool QHFlag = 1;      /* Combi quick heating status flag { 0: passive, 1: active } */
  bool ActFlag = 0;     /* Combi activity status flag { 0: passive, 1: active } */
  uint8_t WMsgFlag = 0; /* Combi warning message flag { 0: No warning, 1: Set temp, 2: Turn on combi } */
  float T = 18.00f;     /* Combi setting temperature */
  float TCV = 0.5f;     /* Combi temperature change value */
  float ULT = 1.0f;     /* Combi upper limit temperature */
  float LLT = 1.0f;     /* Combi lower limit temperature */
  uint8_t tc = 2;       /* Tempereture change value selection */
  uint8_t ul = 2;       /* Upper limit value selection */
  uint8_t ll = 2;       /* Lower limit value selection */
  unsigned long RT = 0; /* Combi runtime */
}Combi;
Combi combi;

/** Blynk definitions */
WidgetLED blynkMotionSensorLed(V8); 
WidgetLED blynkGasSensorLed(V9); 
WidgetLED blynkSmokeSensorLed(V10); 
WidgetLED blynkDoorSensorLed(V11); 
WidgetLED blynkMotionSensorALARMLed(V12); 
WidgetLED blynkGasSensorALARMLed(V13); 
WidgetLED blynkSmokeSensorALARMLed(V14);
WidgetLED blynkDoorSensorALARMLed(V15); 
WidgetLED blynkCombiLed(V20); 
WidgetLED blynkHeatingLed(V21); 
WidgetLED blynkLivingRooomLed(V27); 
WidgetLED blynkKitchenLed(V28); 
WidgetLED blynkBedRoomLed(V29); 
WidgetLED blynkKidsRoomLed(V30);
WidgetLED blynkBuzzerLed(V32);

/** Last updated time variables */ 
unsigned long lastNST = millis(); /* Last Nextion screen time */          
unsigned long lastUT1 = millis();
unsigned long lastUT2 = millis();                 
unsigned long lastUT3 = millis();                   
unsigned long lastUT4 = millis();
unsigned long lastUT5 = millis();
unsigned long lastUT6 = millis();                 
unsigned long lastUT7 = millis();                   
unsigned long lastUT8 = millis();
unsigned long lastUT9 = millis();                 
unsigned long lastUT10 = millis();                   
unsigned long lastUT11 = millis();
unsigned long lastUT12 = millis();                   
unsigned long lastUT13 = millis();
unsigned long lastUT14 = millis();                 
unsigned long lastUT15 = millis();                   
unsigned long lastUT16 = millis();
unsigned long lastUT17 = millis();
unsigned long millisTime = millis();

/** Other variables */
char updt_str[15];
uint8_t language = 0;           /* Default language {TR} */
uint8_t currentPage = 0;        /* Nextion HMI default page */
const char degreeSymbol = 176;  /* ° sign ASCII code */
const char percentSymbol = 37;  /* % sign ASCII code */
long dpOn = 0;                  /* Display power on sate*/
long dpOff = 0;                 /* Display power off state*/
unsigned long dailyCRT = 0;     /* Combi daily runtime */ 
unsigned long weeklyCRT = 0;    /* Combi weekly runtime */ 
unsigned long monthlyCRT = 0;   /* Combi monthly runtime */ 
float dailyPC = 0.00f;          /* Daily power consumption */
float weeklyPC = 0.00f;         /* Weekly power consumption */
float monthlyPC = 0.00f;        /* Monthly power consumption */
float totalPC = 0.00f;          /* Total power consumption */
uint8_t RebootTime = 0;         /* Reboot time */    
uint8_t checkRebootTime = 0;    /* Check reboot time */
bool buzzerSttFlag = 0;         /* Buzzer state flag */
uint64_t wt1 = 0;               /* Worktime 1 */
uint64_t wt2 = 0;               /* Worktime 2 */
uint64_t totalWT = 0;           /* Total work time */ 




/** Nextion serial interface definition for ESP32 serial communication */                                                       
#define nexSerial Serial2 

/** Nextion HMI screen update variables */
String nexDate, nexCWIcon, nexWDsc, nexWDsc1, nexWDsc2, nexWCity, nexWPress, nexWHum, nexCHIcon,
      nexMAlarm = "0", nexGAlarm = "0", nexSAlarm = "0", nexDAlarm = "0", nexErrMsg = "0", nexMIcon, nexBIcon, 
      nexn100 = "0", nexn101 = "0", nexn102 = "0", nexFreeHeap, nexFlashSize, nexWiFiRSSI, nexCPUFreq, nexIP,
      nexComm1[4], nexComm2[4], nexComm3[4], nexFDay[4], nexFIcon[4], nexFTemp[4], nexWiFiQa, nexComm4[5], nexWiFiQaBar[5];
char  nexDPC[10], nexWPC[10], nexMPC[10], nexTPC[10], nexCDRT[10], nexCWRT[10], nexCMRT[10], nexCTemp[8], nexCTempVal[8], nexMillis[20],
      nexInT[8], nexInH[6], nexOutT[8], nexOutH[6], nexTime[8], nexUpdTime[14], nexUpdTime2[15], nexTZone[4], nexChipID[64];
bool nexScrFlag, nexWiFiBt = 1, nexBlynkBt = 1;

/** Nextion NX8048P070-011C HMI Command Definations */
String nCommand; 
String nData;
uint8_t txt = 1;  // For text
uint8_t pic = 2;  // For images
uint8_t val = 3;  // For value
uint8_t pbar = 4; // For progress bar
uint8_t page = 5; // For page

// Nextion items
/*Page 0*/
NexButton b000 = NexButton(0, 1, "b000");         /* Setting page button */
NexButton b001 = NexButton(0, 2, "b001");         /* Info page button */
NexButton b002 = NexButton(0, 13, "b002");        /* Decrease the temperature of the combi */
NexButton b003 = NexButton(0, 14, "b003");        /* Increase the temperature of the combi */
NexButton b004 = NexButton(0, 70, "b004");        /* Combi quick heating button */
NexDSButton bt000 = NexDSButton(0, 3, "bt000");   /* Motion sonsor on-off */
NexDSButton bt001 = NexDSButton(0, 4, "bt001");   /* Gas sensor on-off */
NexDSButton bt002 = NexDSButton(0, 5, "bt002");   /* Smoke sensor on-off */
NexDSButton bt003 = NexDSButton(0, 6, "bt003");   /* Door sensor on-off */
NexDSButton bt004 = NexDSButton(0, 7, "bt004");   /* Living room light on-off */
NexDSButton bt005 = NexDSButton(0, 8, "bt005");   /* Kitchen light on-off */
NexDSButton bt006 = NexDSButton(0, 9, "bt006");   /* Bedroom light on-off */
NexDSButton bt007 = NexDSButton(0, 10, "bt007");  /* Kidsroom light on-off */
NexDSButton bt008 = NexDSButton(0, 11, "bt008");  /* Garage light on-off */
NexDSButton bt009 = NexDSButton(0, 12, "bt009");  /* Combi on-off Page 0 button */
/*Page 1*/
NexButton b100 = NexButton(1, 11, "b100");        /* Back to home page button */      
NexButton b101 = NexButton(1, 15, "b101");        /* Combi lower limit temperature 0.5°-1.5° */
NexButton b102 = NexButton(1, 16, "b102");        /* Combi upper limit temperature 0.5°-1.5° */
NexButton b103 = NexButton(1, 17, "b103");        /* Combi temperature sensitivity 0.1°-1.0° */
NexButton b104 = NexButton(1, 22, "b104");        /* Time zone button */
NexDSButton bt100 = NexDSButton(1, 18, "bt100");  /* WiFi On/Off button */ 
NexDSButton bt101 = NexDSButton(1, 19, "bt101");  /* Blynk Server On/Off button */
NexDSButton bt102 = NexDSButton(1, 23, "bt102");  /* Language selection */
NexDSButton bt103 = NexDSButton(1, 24, "bt103");  /* Buzzer On/Off button */
/*Page 2*/
NexButton b200 = NexButton(2, 1, "b200");         /* Back to home page button */ 

NexTouch *nex_listen_list[] = 
{
  &b000, &b001, &b002, &b003, &b004, /* Page 0 Buttons */
  &bt000, &bt001, &bt002, &bt003, &bt004, &bt005, &bt006, &bt007, &bt008, &bt009, /* Page 0 Dual State Buttons */
  &b100, &b101, &b102, &b103, &b104, // Page 1 Buttons
  &bt100, &bt101, &bt102, &bt103, /* Page 1 Dual State Buttons */
  &b200, /* Page 2 Buttons */
  NULL
};


/**
 * Functions
 */
void setupNex(void);                        /* Initialize Nextion HMI */
void setupUnits(void);                      /* Initialize all units */
void connectWiFi(void);                     /* Connect to WiFi */
void connectBlynk(void);                    /* Connect to Blynk Server */
void checkWiFi(void);                       /* Check WiFi connection */
void checkBlynk(void);                      /* Check Blynk connection */     
void getTH(void);                           /* Get temperature and humidity data from all sensors */
void getInTH(void);                         /* Indoor temperature and humidity data */    
void getOutTH(void);                        /* Outdoor temperature and humidity data */  
void checkSErr(void);                       /* Check all sensor errors */
void checkAlarms(void);                     /* Check all alarm data */
void checkCombi(void);                      /* Check combi status */
void setCombi(void);                        /* Set combi temperature and operation */
void quickHeat(void);                       /* Combi quick heating */
void combiOn(void);                         /* Combi On */
void combiOff(void);                        /* Combi Off */
void rf24gTH(void);                         /* Receive outdoor temperature and humidity data with RF24 */
void rf24cA(void);                          /* Receive data from external alarm modules */
void rf24sCH(uint8_t ch);                   /* Channel selection for RF24 module */
void readEPP(void);                         /* Read data from EEPROM */
void updateData(void);                      /* Update Time, weather forecast and sstronomy data */
void updateBlynkData(void);                 /* Upfate Blynk data */
void updatePage0(void);                     /* Update Nextion HMI page 0 data */
void showTime(void);                        /* Get time for Nextion HMI display */ 
void showCurrentWeather(void);              /* Show current weather on Nextion HMI */ 
void showCurrentWeatherDetail(void);        /* Show current weather details on Nextion HMI */
void showForecast(void);                    /* Show forecast on Nextion HMI screen */
void showForecastDetail(uint8_t start);     /* Show forecast details on Nextion HMI screen */
void showAstronomy(void);                   /* Show astronomy on Nextion HMI */
void showSystemInfo(void);                  /* Show system info on Nextion HMI */
void showWiFiQuality(void);                 /* Show WiFi quality on Nextion HMI */
int8_t getWifiQuality(void);                /* Get WiFi quality */
String getTime(time_t *timestamp);          /* Get time info */
void showTimeZone(void);                    /* Show time zone */
void beepTone(uint8_t i);                   /* Buzzer beep tone */
void checkNexScrStt(void);                  /* Check Nextion HMI display status */
void nextionCommand(uint8_t type, String cmd, String dt);   /* Nextion HMI sending data and commands */
void EEPROMWritelong(int address, long value);              /* Write long data to EEPROM */
long EEPROMReadlong(int address);                           /* Read long data from EEPROM */
void getCombiRuntime(void);                 /* Get combi runtime */ 
void showCombiRuntime(void);                /* Show combi runtime */
void getPower(void);                        /* Calculate power consumption */
void showPower(void);                       /* Show power consumption on Nextion screen */
int setWeatherIconBig(String icon_info);    /* Set big weather icon for Nextion HMI */
int setWeatherIconSmall(String icon_info);  /* Set small weather icon for Nextion HMI */

/**
 * Timer 0 task 
 */
void IRAM_ATTR onTimer(){
  /* Screen power calculate */
  if(nexScrFlag == 0) dpOn++;
  else dpOff++;
  /* Combi runtime */
  if(combi.ActFlag == 1) combi.RT += 1;
  /* Total work time */
  totalWT += 1; 
}

void setup() {

  //Serial.begin(9600);
  /* initialize Nextion HMI Items and communication
   *  { If Serial.begin(115200) is defined as 115200 bps first, nexSerial.begin(115200) will cause an error.
   *    Therefore, the settings of the Nextion HMI display must be made first. If both expressions are defined as 9600 bps, 
   *    there is no need for priority order. }
   */ 
  setupNex(); 
   
  /* Serial communication speed 115200 bps */
  Serial.begin(115200);
  
  /* Prepare units */
  setupUnits();

  /* Read initial data from EEPROM */
  readEPP();

  /* Get temperature and humidity data*/
  getTH();
  
  /* Connect to WiFi */
  connectWiFi();

  /* Connect to Blynk Network */
  connectBlynk();

  /* Update internet data */
  updateData();

  /* Get power consumption */
  getPower();                        
  showPower();

  /* Get combi runtime */
  getCombiRuntime();
  showCombiRuntime();

  /* System info */
  showSystemInfo();

  /* Timer 0 setup */
  Serial.println("start timer ");
  timer = timerBegin(0, 80, true);  /* timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp */
  timerAttachInterrupt(timer, &onTimer, true); /* edge (not level) triggered */
  timerAlarmWrite(timer, 1000000, true); /* 1000000 * 1 us = 1 s, autoreload true */
  timerAlarmEnable(timer); /* enable */

  /* Task 0 setup */
  xTaskCreatePinnedToCore(
                    Task0code,   /* Task function. */
                    "Task0",     /* name of task. */ 
                    48000,       /* Stack size of task */ 
                    NULL,        /* parameter of the task */ 
                    1,           /* priority of the task */
                    &Task0,      /* Task handle to keep track of created task */ 
                    0);          /* pin task to core 0 */                                                                      

  /* Task 1 setup */
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */ 
                    50000,       /* Stack size of task */ 
                    NULL,        /* parameter of the task */ 
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */ 
                    1);          /* pin task to core 1 */                                                                      
}

/**
 * Task 0 
 */
void Task0code( void * pvParameters ){
  Serial.print("Task0 running on core ");
  Serial.println(xPortGetCoreID());
  delay(10000);
  for(;;){ 
    nexLoop(nex_listen_list);
    delay(1); 
  } 
}

/**
 * Task 1 
 */
void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  delay(10000);
  for(;;){ 
    Blynk.run();
    delay(1); 
  } 
}

void loop() {
  if(millis() - lastUT1 > 900) {lastUT1 = millis(); showTime(); showTimeZone();} 
  if(millis() - lastUT2 > 1000) {lastUT2 = millis(); checkNexScrStt();}
  if(millis() - lastUT3 > 1100) {lastUT3 = millis(); checkCombi();}
  if(millis() - lastUT4 > 10000) {lastUT4 = millis(); showCurrentWeather();} 
  if(millis() - lastUT5 > 11000) {lastUT5 = millis(); showForecast();} 
  if(millis() - lastUT6 > 12000) {lastUT6 = millis(); showAstronomy();} 
  if(millis() - lastUT7 > 12500) {lastUT7 = millis(); checkWiFi();}
  if(millis() - lastUT8 > 13000) {lastUT8 = millis(); checkBlynk();}
  if(millis() - lastUT9 > 14000) {lastUT9 = millis(); showWiFiQuality();}
  if(millis() - lastUT10 > 15000) {lastUT10 = millis(); showSystemInfo();}
  if(millis() - lastUT11 > 30000) {lastUT11 = millis(); getTH();}    
  if(millis() - lastUT12 > 31000) {lastUT12 = millis(); checkSErr();}
  if(millis() - lastUT13 > 180000) {lastUT13 = millis(); updateBlynkData();}
  if(millis() - lastUT14 > 600000) {lastUT14 = millis(); getCombiRuntime(); getPower();}
  if(millis() - lastUT15 > 601000) {lastUT15 = millis(); showCombiRuntime(); showPower;}    
  if(millis() - lastUT16 > 3600000) {lastUT16 = millis(); updateData();}          
  
  if(millis() - lastUT17 > 1000){ 
    lastUT17 = millis(); 
    if(currentPage == 0) updateNexPage0();
    else if(currentPage == 1) updateNexPage1();
    else if(currentPage == 2) updateNexPage2();
    else  updateNexPage0();
  } 
}

/**
 * Initialize Nextion HMI
 */
void setupNex(void){
  /* Initialize Nextion HMI 
   * Default speed for nextion is 9600 bps
   * New speed set to 115200 bps
   */   
  nexSerial.begin(9600); 
  nexSerial.print("baud=115200"); 
  nexSerial.write(0xff); 
  nexSerial.write(0xff);
  nexSerial.write(0xff);
  nexSerial.begin(115200);

  nexInit();

    /*Page 0 buttons*/
  b000.attachPop(b000PopCallback, &b000); 
  b001.attachPop(b001PopCallback, &b001); 
  b002.attachPop(b002PopCallback, &b002); 
  b003.attachPop(b003PopCallback, &b003); 
  b004.attachPop(b004PopCallback, &b004);  
  bt000.attachPop(bt000PopCallback, &bt000); 
  bt001.attachPop(bt001PopCallback, &bt001); 
  bt002.attachPop(bt002PopCallback, &bt002);
  bt003.attachPop(bt003PopCallback, &bt003); 
  bt004.attachPop(bt004PopCallback, &bt004); 
  bt005.attachPop(bt005PopCallback, &bt005); 
  bt006.attachPop(bt006PopCallback, &bt006); 
  bt007.attachPop(bt008PopCallback, &bt008); 
  bt008.attachPop(bt007PopCallback, &bt007); 
  bt009.attachPop(bt009PopCallback, &bt009); 
  //m000.attachPop(m000PopCallback, &m000); 
  /*Page 1 buttons*/
  b100.attachPop(b100PopCallback, &b100); 
  b101.attachPop(b101PopCallback, &b101); 
  b102.attachPop(b102PopCallback, &b102);
  b103.attachPop(b103PopCallback, &b103); 
  b104.attachPop(b104PopCallback, &b104);  
  bt100.attachPop(bt100PopCallback, &bt100);  
  bt101.attachPop(bt101PopCallback, &bt101);
  bt102.attachPop(bt102PopCallback, &bt102);
  bt103.attachPop(bt103PopCallback, &bt103); 
  /*Page 2 buttons*/
  b200.attachPop(b200PopCallback, &b200); 
}

/**
 * Initialize units
 */
void setupUnits(void){

  Serial.println("Prepare all units");

  /* ESP32 pin settings */
  pinMode(blynkLed, OUTPUT);    
  pinMode(heatingLed, OUTPUT); 
  pinMode(buzzer, OUTPUT); 
  pinMode(relay, OUTPUT); 
  pinMode(intMSensor, INPUT);
  
  /* Start EEPROM */
  EEPROM.begin(EEPROM_SIZE);
  
  /* Initialize DHT11 */
  dht.begin();

  /* Initialize DS18B20 
   * Set for 12 bits resolution for DS18B20 (eg 27.3 ° display) 
   */
  DS18B20.begin(); 
  DS18B20.getAddress(DS18B20address, 0); 
  DS18B20.setResolution(DS18B20address, 12); 

  rf24sCH(CH[4]);
}
 
/**
 * Read initial data from eeprom
 */
void readEPP(void){
  
  /* EEPROM data must be reset when first programming the ESP32. 
   * Then this function is disabled.
   */ 
/*   
  for(int i = 0; i < 512; i++){
    EEPROM.write(i, 0x00);
    EEPROM.commit(); //    
  } 
*/
  Serial.println("Read initial data from EEPROM");
  delay(100);
  
  /* Read alarm flags */
  mASttFlag = 0x01 & EEPROM.read(0);   /* Motion alarm On/Off flag */
  gASttFlag = 0x01 & EEPROM.read(1);   /* Gas alarm On/Off flag */
  sASttFlag = 0x01 & EEPROM.read(2);   /* Smoke alarm On/Off flag */
  dASttFlag = 0x01 & EEPROM.read(3);   /* Door alarm On/Off flag */

  /* Read combi initial values */
  combi.SttFlag = 0x01 & EEPROM.read(5);  /* Read combi On/Off state flag */
  combi.tc = EEPROM.read(20);             /* Read tempereture change value selection */
  combi.ul = EEPROM.read(21);             /* Read upper limit value selection */
  combi.ll = EEPROM.read(22);             /* Read lower limit value selection */
  EEPROM.get(10, combi.T);                /* Read combi setting temperature */

  if(combi.tc == 1) combi.TCV = 0.1;
  else if(combi.tc  == 2) combi.TCV = 0.5;
  else if(combi.tc  == 3) combi.TCV = 1.0;
  else combi.TCV = 0.5;

  if(combi.ul == 1) combi.ULT = 0.5;
  else if(combi.ul == 2) combi.ULT = 1.0;
  else if(combi.ul == 3) combi.ULT = 1.5; 
  else combi.ULT = 1.0;

  if(combi.ll == 1) combi.LLT = 0.5;
  else if(combi.ll == 2) combi.LLT = 1.0;
  else if(combi.ll == 3) combi.LLT = 1.5; 
  else combi.LLT = 1.0;

  /* Other variables */
  timeZone = EEPROM.read(4);              /* Read timezone  */
  RebootTime = EEPROM.read(24);           /* Read reebot time */
  language = EEPROM.read(25);             /* Read system language */
  buzzerSttFlag = 0x01 & EEPROM.read(4);  /* Read buzzer On/Off flag */
  totalWT = EEPROMReadlong(120);          /* Read total work time */
  wt1 = totalWT;
  wt2 = totalWT;
}     

/**
 * Connect to Wifi
 */
void connectWiFi(void){

  if(!WiFiSttFlag)  WiFi.disconnect();
  
  else{
    if (WiFi.status() == WL_CONNECTED) return;
  
    else  {     
      Serial.print(ssid);
      Serial.print("/");
      Serial.println(pass);
            
      WiFi.mode(WIFI_STA);
      WiFi.setHostname(hostname.c_str());
      WiFi.begin(ssid, pass);
      Serial.print("Connecting to WiFi");
   
      for (int i = 0; i < 10; i ++) {
        Serial.print(".");
        if (WiFi.status() == WL_CONNECTED) {
          break;
        }
        delay(1000);
      }
    }
         
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected...");
      Serial.printf("Hostname: %s\n", WiFi.getHostname());
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("RSSI: ");
      Serial.println(WiFi.RSSI()); 
    }
    else {
      Serial.println("No WIFI found...");
      WiFi.disconnect();
    }
  }
}  

/**
 * Connect to Blynk Server
 */
void connectBlynk(void){ 
  if(!BlynkSttFlag) {
    blynkCSFlag = 0;
    digitalWrite(blynkLed, LOW);
    Blynk.disconnect();
    nexBIcon = "8";  
  }

  else{
    pingTest = Ping.ping("www.google.com", 3);
    if((WiFi.status() == WL_CONNECTED) && pingTest == 1){
      if(!Blynk.connected()){
        blynkCSFlag = 0;
        Serial.println("Connecting to Blynk Server");
        Blynk.disconnect();
        Blynk.config(auth);
        Blynk.connect();
        delay(1000);
        //Blynk.begin(auth, ssid, pass); 
        if(Blynk.connected()){  
          blynkCSFlag = 1; 
          digitalWrite(blynkLed, HIGH);
          Serial.println("Connected to Blynk Server");
          nexBIcon = "6";
        }
      }
    
      else { blynkCSFlag = 1; digitalWrite(blynkLed, HIGH); nexBIcon = "6";}
    }
      
    else{
      blynkCSFlag = 0;
      digitalWrite(blynkLed, LOW);
      Blynk.disconnect(); 
      Serial.println("No Blynk Server");
      nexBIcon = "8"; 
    }        
  }
  nextionCommand(pic , "p1", nexBIcon);
}

/** Sync blynk data */
BLYNK_CONNECTED() { 
 //rtc.begin();
Blynk.syncAll(); 
}

/**
 * Check WiFi Connection
 */
void checkWiFi(void){
  connectWiFi();  
}

/**
 * Check Blynk Connection
 */
void checkBlynk(void){
  connectBlynk();  
}

/**
 * Get temperature and humidity data from all sensors
 */
void getTH(void){
  /* Indoor */
  getInTH();
  /* Outdoor */
  getOutTH();

  /* For nextion screen data update */
  sprintf(nexInT, "%.1f%c", th.inT, degreeSymbol);
  sprintf(nexInH, "%.1f%c", th.inH, percentSymbol);
  sprintf(nexOutT, "%.1f%c", th.outT, degreeSymbol);
  sprintf(nexOutH, "%.1f%c", th.outH, percentSymbol);

  if(thsERR.inTS != 1)  Serial.printf("Indoor Temperature= %.1f\n", th.inT);
  if(thsERR.inHS != 1)  Serial.printf("Indoor Humidity= %.1f\n", th.inH);
  if(thsERR.outTS != 1) Serial.printf("Outdoor Temperature= %.1f\n", th.outT);
  if(thsERR.outHS != 1) Serial.printf("Outdoor Humidity= %.1f\n", th.outH);
}

/**
 * Indoor temperature and humidity data 
 */
void getInTH(void){ 
  
  DS18B20.requestTemperatures(); 
  th.inT = DS18B20.getTempC(DS18B20address);  /* Indoor temperature */
  th.inH = dht.readHumidity();                /* Indoor humidity */  
}

/**
 * Outdoor temperature and humidity data 
 */
void getOutTH(void){ 
  rf24gTH();
}

/**
 * Get outdoor temperature and humidity data 
 */
void rf24gTH(void){
  
  if (radio.available()) { 
    radio.read( &data, sizeof(data) );
    th.outT = data.t; 
    th.outH = data.h; 
  }
  rf24sCH(CH[4]);
}

/**
 * Set RF24 channel selection
 */
void rf24sCH(uint8_t ch){
  int _data = 0;
  if( ch == 4){
    radio.begin(); 
    radio.setChannel(111 + ch); 
    radio.setPALevel(RF24_PA_MAX); 
    radio.setDataRate(RF24_250KBPS); 
    radio.openReadingPipe(1, ADD[ch]);
    radio.startListening();
  }
 
  else{
    radio.begin(); 
    radio.setChannel(110 + ch); 
    radio.setPALevel(RF24_PA_MAX); 
    radio.setDataRate(RF24_250KBPS); 
    radio.openReadingPipe(1, ADD[ch]);
    radio.startListening();
    delay(100);
    
    if(radio.available()) {
      radio.read(&_data, sizeof(_data));
    } 
    RFA[ch] = _data;
    radio.stopListening();
  }  
}

/**
 * Check sensor errors
 */
void checkSErr(void){
  /* Check indoor DS18B20 sensor error */
  if(th.inT == -127) {thsERR.inTS = 1; Serial.println("Indoor Temp Sensor Error");}
  else thsERR.inTS = 0;
  /* Check indoor DHT11 sensor error */
  if (isnan(th.inH)) {thsERR.inHS = 1; Serial.println("Indoor Humidity Sensor Error");}
  else thsERR.inHS  = 0;
  /* Check outdoor DS18B20 sensor error */
  /* --> optional */
  /* Check outdoor DHT11 sensor error */
  /* --> optional */
}

/**
 * Update Blynk data
 */
void updateBlynkData(){ 
  Blynk.virtualWrite(V0, th.outT); 
  Blynk.virtualWrite(V1, th.inT); 
  Blynk.virtualWrite(V2, th.outH); 
  Blynk.virtualWrite(V3, combi.T); 
}

/**
 * Check combi status
 * { If there is a sensor error and the combi is off, the heating is passive,
 * if not, it is active }
 */  
void checkCombi(void){
   
  if(thsERR.inTS == 1){
    combi.SttFlag = 0; 
    combi.HFlag = 0; 
    nexErrMsg = "3"; /* 3 --> Temperature sensor error code */
    Blynk.virtualWrite(V17, 0);
  } 
  else{
    nexErrMsg = "0"; /* No temperature eror */
    if(combi.SttFlag == 0) combi.HFlag = 0; 
    else combi.HFlag = 1;
  }   
  setCombi();      
}

/**
 * Set combi temperature and operation 
 * { Turn the combi on or off automatically for the specified conditions }
 */ 
void setCombi(void){

  /* Start the combi automatically if the specified conditions are valid */
  if(combi.HFlag == 1 && (th.inT <= combi.T)){
    if(combi.LLT <= (combi.T - th.inT)) combiOn();  
  }
   /* If the conditions are valid, turn off the combi automatically */
  else if(combi.HFlag == 1 && (th.inT > combi.T)){
    if(combi.ULT <= (th.inT - combi.T)) combiOff();
  }
   /* Turn off the combi if the heating is canceled */
  else  combiOff();

  /* Send combi data to Nextion HMI */
  int t;
  sprintf(nexCTemp, "%.1f%c", combi.T, degreeSymbol);
  if(combi.SttFlag == 0){
    t = 0; 
    sprintf(nexCTempVal, "%d", t);  
    nexn100 = "0"; nexn101 = "0"; nexn102 = "0";   
  }
  if(combi.SttFlag == 1){
    t = int(combi.T);
    sprintf(nexCTempVal, "%d", t); 
    
    if(combi.tc == 1) nexn102 = "1";
    if(combi.tc == 2) nexn102 = "2";
    if(combi.tc == 3) nexn102 = "3";

    if(combi.ul == 1) nexn101 = "1";
    if(combi.ul == 2) nexn101 = "2";
    if(combi.ul == 3) nexn101 = "3";

    if(combi.ll == 1) nexn100 = "1";
    if(combi.ll == 2) nexn100 = "2";
    if(combi.ll == 3) nexn100 = "3";
  }
}

/**
 * Combi quick heating 
 * { Manually start the combi quickly within the specified limit values }
 */ 
void quickHeat(void){
  
  /* Change the fast heating status according to the combi's activity */
  if(combi.ActFlag == 1) combi.QHFlag = 0; 
  else  combi.QHFlag = 1;

  /* If the specified conditions are valid, start the combi, otherwise activate the warning message flag */
  if(combi.HFlag == 1 && combi.QHFlag == 1){
    if(((combi.T - combi.LLT) <= th.inT) && (th.inT < (combi.T + combi.ULT))) {combiOn();}
    else{
      //combi.WMsgFlag = 1;
      if(currentPage == 0){
        nexErrMsg = "1"; /* Set temperature message code */
        nextionCommand(val, "n006", nexErrMsg);       
        beepTone(1); 
        delay(1500);
        nexErrMsg = "0"; /* Cancel warning message */
        nextionCommand(val, "n006", nexErrMsg); /* Warning end */ 
      }
    }  
  }

  /* Turn off the combi if fast heating is canceled */
  if(combi.HFlag == 1 && combi.QHFlag == 0){
    if(th.inT > (combi.T - combi.LLT)) {combiOff();}
  }

  /* Activate the warning message flag if fast heating is started while the combi is off. */
  if(combi.HFlag == 0 && thsERR.inTS != 1){
    //combi.WMsgFlag = 2;
    if(currentPage == 0){
        nexErrMsg = "2"; /* Start combi warning message */
        nextionCommand(val, "n006", nexErrMsg);       
        beepTone(1); 
        delay(1500);
        nexErrMsg = "0"; /* Cancel warning message */
        nextionCommand(val, "n006", nexErrMsg); /* Warning end */ 
    } 
  }
}

/**
 * Combi Turn On Function 
 */ 
void combiOn(void){
  /* Start the combi */
  combi.ActFlag = true;
  nexCHIcon = "119";
  digitalWrite(relay, HIGH);
  digitalWrite(heatingLed, HIGH);
  //beepTone(0);
  blynkHeatingLed.on();  
}

/**
 * Combi Turn Off Function 
 */ 
void combiOff(void){
  /* Stop the combi */
  combi.ActFlag = false;
  nexCHIcon = "120";
  digitalWrite(relay, LOW);
  digitalWrite(heatingLed, LOW); 
  blynkHeatingLed.off();  
}

/**
 * Buzzer beep tone 
 */
void beepTone(uint8_t i){
  if(i == 0) { // One beep
    digitalWrite(buzzer, HIGH);
    delay(40);
    digitalWrite(buzzer, LOW);   
  }
  else { // Two beep
    digitalWrite(buzzer, HIGH);
    delay(40);
    digitalWrite(buzzer, LOW);
    delay(200);
    digitalWrite(buzzer, HIGH);
    delay(40);
    digitalWrite(buzzer, LOW);
  }
}

/** 
 *  Update Time, Weather Forecast and Astronomy data  
 */
void updateData(){
  Serial.println("Updating time...");
  //configTime(UTC_OFFSET * 3600, 0, NTP_SERVERS); 
  configTime(timeZone * 3600, 0, NTP_SERVERS); 
  while(!time(nullptr)) {
    Serial.print("#");
    delay(100);
  }
  // calculate for time calculation how much the dst class adds.
  //dstOffset = UTC_OFFSET * 3600 + dstAdjusted.time(nullptr) - time(nullptr);  
  dstOffset = timeZone * 3600 + dstAdjusted.time(nullptr) - time(nullptr);   
  Serial.printf("Time difference for DST: %d", dstOffset);
  
  Serial.println("Updating forecasts...");
  OpenWeatherMapCurrent *currentWeatherClient = new OpenWeatherMapCurrent();          
  currentWeatherClient->setMetric(IS_METRIC);
  currentWeatherClient->setLanguage(OPEN_WEATHER_MAP_LANGUAGE);
  currentWeatherClient->updateCurrentById(&currentWeather, OPEN_WEATHER_MAP_APP_ID, OPEN_WEATHER_MAP_LOCATION_ID);
  //currentWeatherClient->updateCurrent(&currentWeather, OPEN_WEATHER_MAP_APP_ID, OPEN_WEATHER_MAP_LOCATION);
  delete currentWeatherClient;
  currentWeatherClient = nullptr;

  Serial.println("Updating astronomy..."); 
  OpenWeatherMapForecast *forecastClient = new OpenWeatherMapForecast();
  forecastClient->setMetric(IS_METRIC);
  forecastClient->setLanguage(OPEN_WEATHER_MAP_LANGUAGE);
  uint8_t allowedHours[] = {12, 0};
  forecastClient->setAllowedHours(allowedHours, sizeof(allowedHours));
  forecastClient->updateForecastsById(forecasts, OPEN_WEATHER_MAP_APP_ID, OPEN_WEATHER_MAP_LOCATION_ID, MAX_FORECASTS);
  //forecastClient->updateForecasts(forecasts, OPEN_WEATHER_MAP_APP_ID, OPEN_WEATHER_MAP_LOCATION, MAX_FORECASTS);
  delete forecastClient;
  forecastClient = nullptr;

  Serial.println("Updating astronomy..."); 
  Astronomy *astronomy = new Astronomy();
  moonData = astronomy->calculateMoonData(time(nullptr));
  float lunarMonth = 29.53;
  moonAge = moonData.phase <= 4 ? lunarMonth * moonData.illumination / 2 : lunarMonth - moonData.illumination * lunarMonth / 2;
  moonAgeImage = String((char) (65 + ((uint8_t) ((26 * moonAge / 30) % 26))));
  delete astronomy;
  astronomy = nullptr; 
  
  char *dstAbbrev;
  time_t now = dstAdjusted.time(&dstAbbrev);
  struct tm * timeinfo = localtime(&now);
  //int hour = (timeinfo->tm_hour+11)%12+1;
  sprintf(nexUpdTime, "Update time:%02d:%02d", timeinfo->tm_hour, timeinfo->tm_min);
}

/** 
 *  Get time info to show on Nextion HMI screen 
 */
void showTime(){
  
  char *dstAbbrev;
  time_t now = dstAdjusted.time(&dstAbbrev);
  struct tm * timeinfo = localtime(&now);
  nexDate = ctime(&now); 
  nexDate = nexDate.substring(0,11) + String(1900 + timeinfo->tm_year);
  //String date = WDAY_NAMES[timeinfo->tm_wday] + " " + MONTH_NAMES[timeinfo->tm_mon] + " " + String(timeinfo->tm_mday) + " " + String(1900 + timeinfo->tm_year);
  //date.toCharArray(data_str,16);                                        

  if(IS_STYLE_12HR){
    int hour = (timeinfo->tm_hour+11)%12+1;
    //sprintf(nexTime, "%2d:%02d:%02d", hour, timeinfo->tm_min, timeinfo->tm_sec);
    sprintf(nexTime, "%2d:%02d", hour, timeinfo->tm_min);
    } 
  else{
    //sprintf(nexTime, "%02d:%02d:%02d",timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    sprintf(nexTime, "%02d:%02d",timeinfo->tm_hour, timeinfo->tm_min);  
  }
}

/** 
 *  For Time
 */
String getTime(time_t *timestamp) {
  struct tm *timeInfo = gmtime(timestamp);
  
  char buf[6];
  sprintf(buf, "%02d:%02d", timeInfo->tm_hour, timeInfo->tm_min);
  return String(buf);
}

/** 
 *  Set the time zone
 */
void showTimeZone(void){
  char sgn;
  char tz[5];

  if(timeZone > 12) timeZone = -12;                   
  if(timeZone < -12) timeZone = 12;                  
  if(timeZone > 0 && timeZone <= 12) {
    sgn = '+';  
    sprintf(nexTZone, "%c%i", sgn, timeZone);
  }
  else sprintf(nexTZone, "%i", timeZone);
}

/** 
 *  Show current weather on Nextion HMI
 */
void showCurrentWeather(void){

  String info;
  int i=0;

  //i = setWeatherIcon(String(currentWeather.icon)); 
  Serial.println(currentWeather.icon);
  nexCWIcon = setWeatherIconBig(String(currentWeather.icon));
  
  if(String(currentWeather.icon) == "01d" || String(currentWeather.icon) == "01n"){ // Clear Sky
    nexWDsc1 = "The weather is clear today.";
    nexWDsc2 = "Enjoy the day.";     
  }
  else if(String(currentWeather.icon) == "02d" || String(currentWeather.icon) == "02n"){ // Few clouds
    nexWDsc1 = "The weather is partly clouds today.";
    nexWDsc2 = "Enjoy the day.";     
  }
  else if(String(currentWeather.icon) == "03d" || String(currentWeather.icon) == "03n"){ // Scattered clouds
    nexWDsc1 = "The weather is cloudy today.";
    nexWDsc2 = "The sun will come out soon.";     
  }
  else if(String(currentWeather.icon) == "04d" || String(currentWeather.icon) == "04n"){ // Broken clouds
    nexWDsc1 = "The weather is cloudy today.";
    nexWDsc2 = "Do not wear sunglasses :))";     
  }
  else if(String(currentWeather.icon) == "09d" || String(currentWeather.icon) == "09n"){ // Shower rain
    nexWDsc1 = "The weather is rainy today."; 
    nexWDsc2 = "Keep your umbrella with you.";     
  }
  else if(String(currentWeather.icon) == "10d" || String(currentWeather.icon) == "10n"){ // Rain
    nexWDsc1 = "The weather is rainy today.";
    nexWDsc2 = "Don't forget to take your umbrella!";     
  }
  else if(String(currentWeather.icon) == "11d" || String(currentWeather.icon) == "11n"){ // Thunderstorm
    nexWDsc1 = "The weather is thunderstorm today.";
    nexWDsc2 = "Protect yourself from lightning.";     
  }
  else if(String(currentWeather.icon) == "13d" || String(currentWeather.icon) == "33n"){ // Snow
    nexWDsc1 = "The weather is snowy today.";
    nexWDsc2 = "Do not forget to wear thick!";     
  }
  else if(String(currentWeather.icon) == "50d" || String(currentWeather.icon) == "50n"){ // Mist
    nexWDsc1 = "The weather is foggy today.";
    nexWDsc2 = "Drive carefully!";     
  } 
  else{
    nexWDsc1 = "N/a"; // Not available
    nexWDsc2 = "N/a";     
  }         
  
  nexWCity = String(currentWeather.cityName); 
  
  nexWDsc = String(currentWeather.description); 

  
  //nData = String(currentWeather.temp,1) + degree_symbol; 
  //nextionCommand(txt, "t0", nData);

  //info = String(currentWeather.tempMin,0) + degree_symbol;
  //info.toCharArray(data_str, 25);
  
  //info = String(currentWeather.tempMax,0) + degree_symbol; 
  //info.toCharArray(data_str, 25);
  
  nexWPress = String(currentWeather.pressure)+"hPa"; 
 
  //nData = String(currentWeather.windSpeed, 1) + (IS_METRIC ? "m/s" : "mph") ; 
  //nextionCommand(txt, "t15", nData);

  nexWHum = String(currentWeather.humidity)+"%";                 
}

/** 
 *  Show current weather details on Nextion HMI
 */
void showCurrentWeatherDetail(void){
String degreeSign = "°F";
  if (IS_METRIC) {
    degreeSign = "°C";
  }
  /*
  // 
  nData = String(currentWeather.temp) + degree_symbol; 
  nextionCommand(txt, "t100", nData);
  nData = String(currentWeather.windSpeed, 1) + (IS_METRIC ? "m/s" : "mph"); 
  nextionCommand(txt, "t101", nData);
  nData = String(currentWeather.windDeg, 1) + degree_symbol;
  nextionCommand(txt, "t102", nData);
  nData = String(currentWeather.humidity) + "%"; 
  nextionCommand(txt, "t103", nData);
  nData = String(currentWeather.pressure) + "hPa";
  nextionCommand(txt, "t104", nData);
  nData = String(currentWeather.clouds) + "%"; 
  nextionCommand(txt, "t105", nData);
  nData = String(currentWeather.visibility) + "m"; 
  nextionCommand(txt, "t106", nData);
  */  
}

/** 
 *  How forecast on Nextion HMI screen
 */
void showForecast(void){
  if(forecastUpdateCount == 0) showForecastDetail(0); // 1st group days
  else  showForecastDetail(4);                        // 2nd group days
  if(forecastUpdateCount == 0) forecastUpdateCount = 1; // Select group 1 or group 2
  else  forecastUpdateCount = 0;   
}

/** 
 *  Show forecast details on Nextion HMI screen
 */
void showForecastDetail(uint8_t start){
  char data_str[20];
  uint8_t vl = 0;
    
  String degreeSign = "°F";
  if (IS_METRIC) {
    degreeSign = "°C";
  }

  for (uint8_t i = start; i < start + 4; i++){
     
    time_t time = forecasts[i].observationTime + dstOffset;
    struct tm * timeinfo = localtime (&time);
    
    nexComm1[vl] ="t" + String(vl+3);  
    nexFDay[vl] = WDAY_NAMES[timeinfo->tm_wday] + " " + String(timeinfo->tm_hour) + ":00";
    //sprintf(data_str, "%s", nData);
    //t3.setText(data_str); 
     
    nexComm2[vl] = "p" + String(vl+3);                          
    nexFIcon[vl] = setWeatherIconSmall(String(forecasts[i].icon));               
    
    nexComm3[vl] = "t" + String(vl+7);
    nexFTemp[vl] = String(forecasts[i].temp, 0) + degreeSymbol + "C";        
                          
    vl += 1;
  }  
}

/** 
 *  Show astronomy on Nextion HMI
 */
void showAstronomy(void){
  String info;
  char data_str[5];
  //nData = moonAgeImage;
  //nData = MOON_PHASES[moonData.phase];
  //nData = "Sonne";
  
  time_t time = currentWeather.sunrise + dstOffset;
//"Rise:"
// nData = getTime(&time);
//  nextionCommand(txt, "t12", nData);
  
  time = currentWeather.sunset + dstOffset;
//nData = "Set:"
//  nData = getTime(&time);
//  nextionCommand(txt, "t13", nData);
  
  //nData = "Mond";
  //nData = String(moonAge) + "d";
  //nData = String(moonData.illumination * 100, 0) + "%";
  //nData = "Age:";
  //nData = "Illum:";  
}

/** 
 *  Show system info on Nextion HMI
 */
void showSystemInfo(void){

  nexFreeHeap = String(ESP.getFreeHeap() / 1024)+"kb"; /* Heap memory */  
                   
  nexFlashSize = String(ESP.getFlashChipSize() / 1024 / 1024) + "MB"; /* Flash memory */
  
  nexWiFiRSSI = String(WiFi.RSSI()) + "dB"; /* WiFi RSSI */  
       
  uint64_t macAddress = ESP.getEfuseMac(); /* ESP chip ID */
  uint64_t macAddressTrunc = macAddress << 40;
  uint64_t chipid = macAddressTrunc >> 40;
  sprintf(nexChipID, "%u", chipid);
  
  //si = String(ESP.getVcc() / 1024.0) +"V"; /* CPU voltage */
  
  millisTime = millis(); /* millis() time */                                        
  sprintf(nexMillis, "%d", millisTime);
  
  nexCPUFreq = String(ESP.getCpuFreqMHz()) + "MHz"; /* CPU frequency */
  
  const uint32_t millis_in_day = 1000 * 60 * 60 * 24; /* Operation time */
  const uint32_t millis_in_hour = 1000 * 60 * 60;
  const uint32_t millis_in_minute = 1000 * 60;
  uint8_t days = millis() / (millis_in_day);
  uint8_t hours = (millis() - (days * millis_in_day)) / millis_in_hour;
  uint8_t minutes = (millis() - (days * millis_in_day) - (hours * millis_in_hour)) / millis_in_minute;
  sprintf(nexUpdTime2, "%2dd%2dh%2dm", days, hours, minutes); 
  
  //si = String(ESP.getResetInfo()); /* Reset info */
}

/** 
 *  Show WiFi quality on Nextion HMI
 */
void showWiFiQuality(void){
  int8_t quality = getWifiQuality();
  nexWiFiQa = String(quality) + "%";

  for (int8_t i = 0; i < 5; i++) { 
    if(quality > i * 20) { 
      nexComm4[i] = "j" + String(i);
      nexWiFiQaBar[i] = "100";   
    }
    else {
      nexComm4[i] = "j" + String(i);
      nexWiFiQaBar[i] = "0";  
    }
  } 
}

/** 
 *  Get WiFi quality
 */
int8_t getWifiQuality(){
  int32_t dbm = WiFi.RSSI();
  if (dbm <= -100) {
    return 0;
  } else if (dbm >= -50) {
    return 100;
    } else {
    return 2 * (dbm + 100);
  }
}

/** 
 *  Check Nextion HMI screen status
 *  { Turn on the screen if there is movement in the environment, 
 *  otherwise turn it off after the specified time }
 */
 /*
void checkNexScrStt() {

  if(digitalRead(intMSensor) == 1){
    if(nexScrFlag == 1) {nexSerial.print("sleep=1"); nexSerial.write(0xff); nexSerial.write(0xff); nexSerial.write(0xff);} 
    nexScrFlag = 0;
    lastNST = millis();   
    nexMIcon = "7";                            
  }   
  else  nexMIcon = "8";                                              
  
  if (millis() - lastNST > 1000 *  100){ 
    
    if(nexScrFlag == 0) {nexSerial.print("sleep=0"); nexSerial.write(0xff); nexSerial.write(0xff); nexSerial.write(0xff);}
    nexScrFlag = 1;
    //lastNST = millis();    
  }
  nextionCommand(pic, "p0", nexMIcon);
  //if(nexScrFlag == 0) dpOn++;
  //else dpOff++;
}
*/

/** 
 *  Nextion HMI running time
 */
void checkScreenRunningTime() { 

  if (millis() - lastNST > 1000 *  60) { 
    //nexSerial.print("dim=30");
    //Serial2.print("sleep=1");
    //Serial2.write(0xff); 
    //Serial2.write(0xff);
    //Serial2.write(0xff);
    nexScrFlag = 1;
    lastNST = millis();    
  }
}

/** 
 *  Check Nextion HMI status
 */
void checkNexScrStt() {

  if(digitalRead(intMSensor) == 1)
  {
    lastNST = millis(); 
    nexScrFlag = 0;
    Serial2.print("sleep=0"); 
    Serial2.write(0xff); 
    Serial2.write(0xff);
    Serial2.write(0xff);
    //nexSerial.print("dim=100"); 
    //Serial2.write(0xff); 
    //Serial2.write(0xff);
    //Serial2.write(0xff);
    nextionCommand(pic, "p0", "7");                               
  }                                                 
  
  else
  { 
    nextionCommand(pic, "p0", "8");           
    if(nexScrFlag == 1){ 
      Serial2.print("sleep=1"); 
      Serial2.write(0xff); 
      Serial2.write(0xff);
      Serial2.write(0xff);
    }
  }
  if(nexScrFlag == 0) dpOn++;
  else dpOff++;
  
  checkScreenRunningTime();
}

/**
 * Reading and writing long type variables to eeprom
 */
void EEPROMWritelong(int address, long value){

  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  EEPROM.write(address, four); EEPROM.commit();
  EEPROM.write(address + 1, three); EEPROM.commit();
  EEPROM.write(address + 2, two); EEPROM.commit();
  EEPROM.write(address + 3, one); EEPROM.commit();
}

long EEPROMReadlong(int address){

    long four = EEPROM.read(address); 
    long three = EEPROM.read(address + 1); 
    long two = EEPROM.read(address + 2); 
    long one = EEPROM.read(address + 3);
    return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

/** 
 *  Calculate combi runtime
 */
void getCombiRuntime(){
  unsigned long rt;
  char ts[32];

  rt = EEPROMReadlong(90);  EEPROMWritelong(90, rt + combi.RT);
  dailyCRT = rt + combi.RT; /* Daily data */
  if(86400 < totalWT - wt1){  /* A day is 86400 seconds */
    wt1 = totalWT;  /* Reset the value after one day */ 
    rt = 0;
    EEPROMWritelong(90, rt);
  }

  rt = EEPROMReadlong(100);  EEPROMWritelong(100, rt + combi.RT);
  weeklyCRT = rt + combi.RT; /* Weekly data */
  if(604800 < totalWT - wt2){ /* A week is 604800 seconds */
    wt2 = totalWT;  /* Reset the value after one week */
    rt = 0;
    EEPROMWritelong(100, rt);
  }

  rt = EEPROMReadlong(110);  EEPROMWritelong(110, rt + combi.RT);
  monthlyCRT = rt + combi.RT; /* Monthly data */
  if(2419200 < totalWT){  /* A month is 2419200 seconds */
    wt1 = 0; wt2 = 0; totalWT = 0;  /* Reset all values after one month */
    rt = 0;
    EEPROMWritelong(90, rt); EEPROMWritelong(100, rt); EEPROMWritelong(110, rt);
  }
  
  EEPROMWritelong(120, totalWT); /* Total work time */
  combi.RT = 0;
}

/** 
 *  Show combi runtime 
 */
void showCombiRuntime(){
  sprintf(nexCDRT, "%d sec.", dailyCRT);    /* Daily */
  sprintf(nexCWRT, "%d sec.", weeklyCRT);   /* Weekly */
  sprintf(nexCMRT, "%d sec.", monthlyCRT);  /* Monthly */  
}

/** 
 *  Power consumption measurement
 */
void getPower(){

unsigned long rt;
unsigned long po;
unsigned long pf;
unsigned long x;
char ts[32];
  
  po = EEPROMReadlong(50);  EEPROMWritelong(50,dpOn+po); /* Daily logging to EEPROM */
  pf = EEPROMReadlong(54);  EEPROMWritelong(54,dpOff+pf); 

  //x = po + dpOn + dpOff + pf;
  //sprintf(ts, "%d", x);
  //nextionCommand(txt, "t028", ts);
  /* In theory, the sum of dpOn + dpOff for every second is 3600 in 1 hour */
  dailyPC = (float(dpOn+po)/3600)*5*0.55 + (float(dpOff+pf)/3600)*5*0.25; 
  if(86400 < po+dpOn + pf+dpOff){ /* Maximum number of ScreenOn + ScreenOff to be measured every second in a day 86400 */                                 
    po = 0; pf =0; /* Reset daily power consumption when exceeding limit value */
    EEPROMWritelong(50,po);  EEPROMWritelong(54,pf); 
  }  

  po = EEPROMReadlong(60);  EEPROMWritelong(60,dpOn+po); /* Weekly logging to EEPROM */
  pf = EEPROMReadlong(64);  EEPROMWritelong(64,dpOff+pf);
  weeklyPC = (float(dpOn+po)/3600)*5*0.55 + (float(dpOff+pf)/3600)*5*0.25; 
  if(604800 <  (po+dpOn + pf+dpOff)){ /* Maximum number of ScreenOn + ScreenOff to be measured every second in a week 604800 */                               
    po = 0; pf =0; /* Reset weekly power consumption when exceeding limit value */
    EEPROMWritelong(60,po);  EEPROMWritelong(64,pf); 
  }

  po = EEPROMReadlong(70);  EEPROMWritelong(70,dpOn+po); /* Mounthly logging to EEPROM */
  pf = EEPROMReadlong(74);  EEPROMWritelong(74,(dpOff+pf));
  monthlyPC = ((float(dpOn+po)/3600)*5*0.55 + (float(dpOff+pf)/3600)*5*0.25)/1000; 
  if(2419200 < po+dpOn + pf+dpOff){ /* Maximum number of ScreenOn + ScreenOff to be measured every second in a month 2419200 */                                   
    po = 0; pf =0; /* Reset mountly power consumption when exceeding limit value */
    /* Reset when the one month measurement limit is reached, and reset all power data */
    EEPROMWritelong(50,po);  EEPROMWritelong(54,pf); 
    EEPROMWritelong(60,po);  EEPROMWritelong(64,pf); 
    EEPROMWritelong(70,po);  EEPROMWritelong(74,pf); 
  }

  po = EEPROMReadlong(80);  EEPROMWritelong(80,dpOn+po); /* Total logging to EEPROM */
  pf = EEPROMReadlong(84);  EEPROMWritelong(84,(dpOff+pf));
  totalPC = ((float(dpOn+po)/3600)*5*0.55 + (float(dpOff+pf)/3600)*5*0.25)/1000; 
  
  dpOn = 0;
  dpOff = 0;

  //if(2422200 < totalWT) ESP.restart();
} 

/** 
 *  Show power consumption
 */
void showPower(){
  sprintf(nexDPC, "%.1fW", dailyPC);    /* Daily */
  sprintf(nexWPC, "%.1fW", weeklyPC);   /* Weekly */
  sprintf(nexMPC, "%.2fKW", monthlyPC); /* Monthly */
  sprintf(nexTPC, "%.1f KW", totalPC);  /* Total */
}

/** 
 *  Check alarm states
 */
void checkAlarms(void){
  /* Check internal motion sensor */
  if(mASttFlag == 1 && rf.MAlarm == 10){
    if(digitalRead(intMSensor) == 1) rf.MAlarm == 11;
  }
  /* Check motion alarm */
  if(mASttFlag == 1 && rf.MAlarm == 11){ 
    Blynk.notify(String("Motion detected at home!"));
    blynkMotionSensorALARMLed.on();
    rf.MAlarm = 12; /* Lock alarm if alarm data */
    nexMAlarm = "1";
    if(currentPage == 0) nextionCommand(val, "n002", nexMAlarm);
  }
  /* Check gas alarm */
  if(gASttFlag == 1 && rf.GAlarm == 21){ 
    Blynk.notify(String("Gas detected at home!"));
    blynkGasSensorALARMLed.on();
    rf.GAlarm = 22; /* Lock alarm if alarm data */
    nexGAlarm = "1";
    if(currentPage == 0) nextionCommand(val, "n003", nexGAlarm);
  }
  /* Check smoke alarm */
  if(sASttFlag == 1 && rf.SAlarm == 31){ 
    Blynk.notify(String("Smoke detected at home!"));
    blynkSmokeSensorALARMLed.on();
    rf.SAlarm = 32; /* Lock alarm if alarm data */
    nexSAlarm = "1";
    if(currentPage == 0) nextionCommand(val, "n004", nexSAlarm);
  }
  /* Check door alarm */
  if(dASttFlag == 1 && rf.DAlarm == 41){ 
    Blynk.notify(String("The door of the house opened."));
    blynkDoorSensorALARMLed.on();
    rf.DAlarm = 42; /* Lock alarm if alarm data */
    nexDAlarm = "1";
    if(currentPage == 0) nextionCommand(val, "n005", nexDAlarm);
  }
  /* Sound the buzzer if there is an alarm */
  if(buzzerSttFlag == 1 && (rf.MAlarm == 12 || rf.GAlarm == 22 || rf.SAlarm == 32 || rf.DAlarm == 42))
    digitalWrite(buzzer, HIGH);
  else
    digitalWrite(buzzer, LOW);  
}

/* 
 * Functions of sending data to the Nextion HMI display  
 */
void nextionCommand(uint8_t type, String cmd, String dt){
  if(type==1){ /* Send text */
    nexSerial.print(cmd);     /* These commands do the same as t3.setText(data_str) */
    nexSerial.print(".txt="); /* e.g t3.txt=\"Hello world\"  */
    nexSerial.print("\"");    
    nexSerial.print(dt);
    nexSerial.print("\"");   
  } 
  
  else if(type==2) { /* Send pic */
    nexSerial.print(cmd); /* Similar to text */    
    nexSerial.print(".pic="); 
    nexSerial.print(dt);
  }  
  
  else if(type==3) { /* Send value */
    nexSerial.print(cmd); /* Similar to text */ 
    nexSerial.print(".val=");
    nexSerial.print(dt);
  }

  else if(type==4) { /* Send progress bar data */
    nexSerial.print(cmd); /* Similar to text */ 
    nexSerial.print(".val=");
    nexSerial.print(dt);
  }
  
  else if(type==5) { /* Send page data */
    nexSerial.print("page "); /* Similar to text */ 
    nexSerial.print(dt);
  }

  else return;

  /* Complete sending data for Nextion HMI */  
  Serial2.write(0xff); 
  Serial2.write(0xff);
  Serial2.write(0xff);
}

/**
 *  Update Nextion HMI Page 0 data 
 */
void updateNexPage0(void){
  int i;
  nextionCommand(txt, "t0", nexTime);             /* Time */
  nextionCommand(txt, "t1", nexDate);             /* Date */
  nextionCommand(txt, "t2", nexWiFiQa);           /* WiFi quality */
  for(i = 0; i < 5; i++){
    nextionCommand(pbar , nexComm4[i], nexWiFiQaBar[i]);  /* WiFi quality bar */
  }
  nextionCommand(txt, "t009", nexUpdTime);        /* Update time */
  nextionCommand(txt, "t022", nexInT);            /* Indoor temperature */ 
  //nextionCommand(txt, "t31", nexInH);           /* Indoor humidity */
  nextionCommand(txt, "t004", nexOutT);           /* Outdoor temperature */
  nextionCommand(pic, "p002", nexCWIcon);         /* Current weather icon */
  nextionCommand(txt, "t005", nexWDsc);           /* Current weather description 1 */
  nextionCommand(txt, "t003", nexWDsc1);          /* Weather description 1 */
  nextionCommand(txt, "t026", nexWDsc2);          /* Weather description 2 */ 
  nextionCommand(txt, "t006", nexWCity);          /* Current city name */ 
  nextionCommand(txt, "t008", nexWPress);         /* Current weather pressure */
  nextionCommand(txt, "t007", nexWHum);           /* Current weather humidity */
  for(i = 0; i < 4; i++){                                 /* 4 days weather forecast */
    nextionCommand(txt , nexComm1[i], nexFDay[i]);        /* Forecast day & time */
    nextionCommand(pic , nexComm2[i], nexFIcon[i]);       /* Forecast icon */
    nextionCommand(txt , nexComm3[i], nexFTemp[i]);       /* Forecast temperature */
  }
  bt000.setValue(0x0001 & mASttFlag);             /* Set motion sensor On/Off buton */
  bt001.setValue(0x0001 & gASttFlag);             /* Set gas sensor On/Off buton */
  bt002.setValue(0x0001 & sASttFlag);             /* Set smoke sensor On/Off buton */
  bt003.setValue(0x0001 & dASttFlag);             /* Set door sensor On/Off buton */
  bt004.setValue(0x0001 & lRLSttFlag);            /* Livingroom light */
  bt005.setValue(0x0001 & kLSttFlag);             /* Kitchen light */
  bt006.setValue(0x0001 & bRLSttFlag);            /* Bedroom light */
  bt007.setValue(0x0001 & kRLSttFlag);            /* Kids room light */
  bt008.setValue(0x0001 & gLSttFlag);             /* Garage light */
  bt009.setValue(0x0001 & combi.SttFlag );        /* Set combi On/Off buton */
  nextionCommand(txt, "t023", nexCTemp);          /* Combi temperature */
  nextionCommand(val, "n001", nexCTempVal);       /* Combi temperature for icon display */
  nextionCommand(pic, "p008", nexCHIcon);         /* Combi heating icon */
  nextionCommand(val, "n006", nexErrMsg);         /* Nextion Error message */
  nextionCommand(txt, "t029", nexDPC);            /* Daily power consumption */
  nextionCommand(txt, "t028", nexWPC);            /* Weekly power consumption */
  nextionCommand(txt, "t027", nexMPC);            /* Monthly power consumption */
}

/**
 *  Update Nextion HMI Page 1 data 
 */
void updateNexPage1(void){
  int i;
  nextionCommand(txt, "t0", nexTime);             /* Time */
  nextionCommand(txt, "t1", nexDate);             /* Date */
  nextionCommand(txt, "t2", nexWiFiQa);           /* WiFi quality */
  for(i = 0; i < 5; i++){
    nextionCommand(pbar , nexComm4[i], nexWiFiQaBar[i]);  /* WiFi quality bar */
  }
  nextionCommand(val, "n102", nexn102);           /* Combi temperature change value */ 
  nextionCommand(val, "n101", nexn101);           /* Combi upper temp limit value */
  nextionCommand(val, "n100", nexn100);           /* Combi lower temp limit value */
  nextionCommand(txt, "t100", nexTZone);          /* Time zone */
  bt100.setValue(0x0001 & nexWiFiBt);             /* WiFi On/Off */
  bt101.setValue(0x0001 & nexBlynkBt);            /* Blynk On/Off */
  bt103.setValue(0x0001 & buzzerSttFlag);         /* Buzzer On/Off */ 
}

/**
 *  Update Nextion HMI Page 2 data 
 */
void updateNexPage2(void){
  int i;  
  nextionCommand(txt, "t0", nexTime);             /* Time */
  nextionCommand(txt, "t1", nexDate);             /* Date */
  nextionCommand(txt, "t2", nexWiFiQa);           /* WiFi quality */
  for(i = 0; i < 5; i++){
    nextionCommand(pbar , nexComm4[i], nexWiFiQaBar[i]);  /* WiFi quality bar */
  }
 
  nextionCommand(txt, "t208", nexFreeHeap);      /* Free heap Size */
  nextionCommand(txt, "t209", nexFlashSize);      /* Flash memory size */
  nextionCommand(txt, "t210", nexWiFiRSSI);       /* WiFi RSSI */
  nextionCommand(txt, "t211", nexChipID);         /* Chip ID */
  nextionCommand(txt, "t212", nexMillis);         /* millis() */
  nextionCommand(txt, "t213", nexCPUFreq);        /* CPU frequency */
  nextionCommand(txt, "t214", nexUpdTime2);       /* Total working time */
  //nextionCommand(txt, "t215", nexIP);             /* System IP */
  nextionCommand(txt, "t220", nexCDRT);           /* Combi daily total working time */
  nextionCommand(txt, "t221", nexCWRT);           /* Combi weekly total working time */
  nextionCommand(txt, "t222", nexCMRT);           /* Combi monthly total working time */
  nextionCommand(txt, "t223", nexTPC);            /* Total power consumption */  
}


/********************** Blynk button control functions ************************/
   
/** Function definitions **/

/** Has the motion sensor On/Off button been pressed in the Blynk program? */
BLYNK_WRITE(V4){
  int btValue = param.asInt(); 
  if(btValue == 1) {
    mASttFlag = 1; 
    blynkMotionSensorLed.on();  
  }
  else { 
    mASttFlag = 0; 
    rf.MAlarm = 10;
    blynkMotionSensorALARMLed.off(); 
    blynkMotionSensorLed.off(); 
    nexMAlarm = "0";
    digitalWrite(buzzer, LOW);               
  }
  EEPROM.write(0, 0x01 & mASttFlag);
  EEPROM.commit(); 
  if(currentPage == 0){
    bt000.setValue(0x0001 & mASttFlag);
    nextionCommand(val, "n002", nexMAlarm); 
  }
}

/** Has the gas sensor ON / OFF button been pressed in the Blynk program? */
BLYNK_WRITE(V5){
  int btValue = param.asInt(); 
  if(btValue == 1) {
    gASttFlag = 1; 
    blynkGasSensorLed.on();  
  }
  else { 
    gASttFlag = 0; 
    rf.GAlarm = 20;
    blynkGasSensorALARMLed.off(); 
    blynkGasSensorLed.off(); 
    nexGAlarm = "0";
    digitalWrite(buzzer, LOW);               
  }
  EEPROM.write(1, 0x01 & gASttFlag);
  EEPROM.commit(); 
  if(currentPage == 0){
    bt001.setValue(0x0001 & gASttFlag);
    nextionCommand(val, "n003", nexGAlarm); 
  }  
}

/** Has the smoke sensor ON / OFF button been pressed in the Blynk program? */
BLYNK_WRITE(V6){
  int btValue = param.asInt(); 
  if(btValue == 1) {
    sASttFlag = 1; 
    blynkSmokeSensorLed.on();  
  }
  else { 
    sASttFlag = 0; 
    rf.SAlarm = 30;
    blynkSmokeSensorALARMLed.off(); 
    blynkSmokeSensorLed.off(); 
    nexSAlarm = "0";
    digitalWrite(buzzer, LOW);               
  }
  EEPROM.write(2, 0x01 & sASttFlag);
  EEPROM.commit(); 
  if(currentPage == 0){
    bt002.setValue(0x0001 & sASttFlag);
    nextionCommand(val, "n004", nexSAlarm); 
  }  
}

/** Has the door sensor ON / OFF button been pressed in the Blynk program? */
BLYNK_WRITE(V7){
  int btValue = param.asInt(); 
  if(btValue == 1) {
    dASttFlag = 1; 
    blynkDoorSensorLed.on();  
  }
  else { 
    dASttFlag = 0; 
    rf.DAlarm = 40;
    blynkDoorSensorALARMLed.off(); 
    blynkDoorSensorLed.off(); 
    nexDAlarm = "0";
    digitalWrite(buzzer, LOW);               
  }
  EEPROM.write(3, 0x01 & dASttFlag);
  EEPROM.commit(); 
  if(currentPage == 0){
    bt003.setValue(0x0001 & dASttFlag);
    nextionCommand(val, "n005", nexDAlarm); 
  }  
}

/** Has the combi On/Off button been pressed in the Blynk program? */
BLYNK_WRITE(V17){
  int btValue = param.asInt(); 
  if(btValue == 1 && thsERR.inTS == 0) {
    combi.SttFlag = 1; 
    blynkCombiLed.on(); 
  }
  else { 
    combi.SttFlag = 0; 
    blynkCombiLed.off(); 
    blynkHeatingLed.off(); 
  }
  Blynk.virtualWrite(V3, combi.T); 
  EEPROM.write(5, combi.SttFlag); 
  EEPROM.commit(); 
  if(currentPage == 0) bt009.setValue(0x0001 & combi.SttFlag); 
}

/** Has the combi temperature reduction button been pressed in the Blynk program? */
BLYNK_WRITE(V18){
  if(param.asInt() == 1){ 
    if(combi.SttFlag == 1){ 
      combi.T = combi.T - combi.TCV; 
      if(combi.T < 10.0) combi.T = 10.0; 
      EEPROM.put(10,combi.T); 
      EEPROM.commit(); 
      Blynk.virtualWrite(V3, combi.T);                                                                
      sprintf(nexCTemp, "%.1f%c", combi.T, degreeSymbol);       
      if(currentPage == 0)  nextionCommand(txt , "t023", nexCTemp);
    }
  }
}

/** Has the combi temperature increase button been pressed in the Blynk program? */
BLYNK_WRITE(V19){
  if(param.asInt() == 1){ 
    if(combi.SttFlag == 1){
      combi.T = combi.T + combi.TCV;
      if(combi.T > 30.0) combi.T = 30.0;  
      EEPROM.put(10,combi.T); 
      EEPROM.commit(); 
      Blynk.virtualWrite(V3, combi.T);                                                   
      sprintf(nexCTemp, "%.1f%c", combi.T, degreeSymbol);                    
      if(currentPage == 0) nextionCommand(txt , "t023", nexCTemp);  
    }                                       
  }
}

/** Has the living room lamp ON / OFF button been pressed in the Blynk program? */
BLYNK_WRITE(V23){
  int btValue = param.asInt(); 
  if(btValue == 1) { 
    lRLSttFlag = 1;                                
    blynkLivingRooomLed.on();  
    if(currentPage == 0) bt004.setValue(0x0001 & lRLSttFlag);                    
  } 
  else {
    lRLSttFlag = 0; 
    blynkLivingRooomLed.off(); 
    if(currentPage == 0) bt004.setValue(0x0001 & lRLSttFlag);      
  } 
}

/** Has the kitchen lamp ON / OFF button been pressed in the Blynk program? */
BLYNK_WRITE(V24){
  int btValue = param.asInt(); 
  if(btValue == 1) {
    kLSttFlag = 1;                                
    blynkKitchenLed.on(); 
    if(currentPage == 0) bt005.setValue(0x0001 & kLSttFlag);                    
  } 
  else { 
    kLSttFlag = 0; 
    blynkKitchenLed.off(); 
    if(currentPage == 0) bt005.setValue(0x0001 & kLSttFlag); 
  } 
}

/** Has the bedroom lamp ON / OFF button been pressed in the Blynk program? */
BLYNK_WRITE(V25) {
  int btValue = param.asInt(); 
  if(btValue == 1) { 
    bRLSttFlag = 1;                                
    blynkBedRoomLed.on(); 
    if(currentPage == 0) bt006.setValue(0x0001 & bRLSttFlag);                    
  } 
  else { 
    bRLSttFlag = 0; 
    blynkBedRoomLed.off(); 
    if(currentPage == 0) bt006.setValue(0x0001 & bRLSttFlag);   
  } 
}

/** Has the kids room lamp ON / OFF button been pressed in the Blynk program? */
BLYNK_WRITE(V26) {
  int btValue = param.asInt(); 
  if(btValue == 1) { 
    kRLSttFlag = 1;                                
    blynkKidsRoomLed.on(); 
    if(currentPage == 0) bt007.setValue(0x0001 & kRLSttFlag);   
    }                    
  else { 
    kRLSttFlag = 0; 
    blynkKidsRoomLed.off(); 
    if(currentPage == 0) bt007.setValue(0x0001 & kRLSttFlag);     
  }
}

/** Has the alarm buzzer On/Off button been pressed in the Blynk program? */
BLYNK_WRITE(V31) {
  int btValue = param.asInt(); 
  if(btValue == 1) {                                         
    buzzerSttFlag = 1;                               
    blynkBuzzerLed.on();                   
  } 
  else { 
    buzzerSttFlag = 0; 
    blynkBuzzerLed.off();     
  }
  EEPROM.write(6, 0x01 & buzzerSttFlag); 
  EEPROM.commit();
  if(currentPage == 1)  bt103.setValue(0x0001 & buzzerSttFlag); 
}

/** Has the fast update data button been pressed in the Blynk program? */
BLYNK_WRITE(V22) {
  
  updateBlynkData(); 
  updateData();  
}


/********************** Nextion button control functions **********************/

/**
 * Page 0 Buttons  
 */

/** Go to Page 1 */
void b000PopCallback(void *ptr){ /* Page 1 Button - Settings Page */

  currentPage = 1;
}

/** Go to Page 2 */
void b001PopCallback(void *ptr){ /* Page 2 Button - Info Page */
  
  currentPage = 2;
}

/** Lower the combi temperature */
void b002PopCallback(void *ptr){ 
  if(combi.SttFlag == 1) { 
    combi.T = combi.T - combi.TCV; 
    if(combi.T < 10.0) combi.T = 10.0; 
    EEPROM.put(10,combi.T); 
    EEPROM.commit(); 
    Blynk.virtualWrite(V3, combi.T);                                                                
    sprintf(nexCTemp, "%.1f%c", combi.T, degreeSymbol);       
    nextionCommand(txt , "t023", nexCTemp);
  }                                      
}

/** Raise the combi temperature */
void b003PopCallback(void *ptr){ 
  if(combi.SttFlag == 1) { 
    combi.T = combi.T + combi.TCV;
    if(combi.T > 30.0) combi.T = 30.0;  
    EEPROM.put(10,combi.T); 
    EEPROM.commit(); 
    Blynk.virtualWrite(V3, combi.T);                                                   
    sprintf(nexCTemp, "%.1f%c", combi.T, degreeSymbol);                  
    nextionCommand(txt , "t023", nexCTemp); 
  }                                    
}

/** Run quick heating */
void b004PopCallback(void *ptr){ 

  quickHeat(); 
}                                                                    

/** Set motion sensor On/Off State */
void bt000PopCallback(void *ptr){ 
  mASttFlag = (0x01 & ~mASttFlag);
  bt000.setValue(0x0001 & mASttFlag); 
  EEPROM.write(0, 0x01 & mASttFlag);
  EEPROM.commit(); 
  if(mASttFlag == 1) {
    Blynk.virtualWrite(V4, 1);
    blynkMotionSensorLed.on();  
  }
  else { 
    //mASttFlag = 0; 
    rf.MAlarm = 10;
    Blynk.virtualWrite(V4, 0);
    blynkMotionSensorLed.off(); 
    blynkMotionSensorALARMLed.off(); 
    nexMAlarm = "0";
    digitalWrite(buzzer, LOW);               
  }
  nextionCommand(val, "n002", nexMAlarm);                            
}

/** Set gas sensor On/Off State */
void bt001PopCallback(void *ptr){ 
  gASttFlag = (0x01 & ~gASttFlag);
  bt001.setValue(0x0001 & gASttFlag); 
  EEPROM.write(1, 0x01 & gASttFlag);
  EEPROM.commit(); 
  if(gASttFlag == 1) {
    Blynk.virtualWrite(V5, 1);
    blynkGasSensorLed.on();  
  }
  else { 
    //gASttFlag = 0; 
    rf.GAlarm = 20;
    Blynk.virtualWrite(V5, 0);
    blynkGasSensorLed.off(); 
    blynkGasSensorALARMLed.off(); 
    nexGAlarm = "0";
    digitalWrite(buzzer, LOW);               
  }
  nextionCommand(val, "n003", nexGAlarm);                                 
}

/** Set smoke sensor On/Off State */
void bt002PopCallback(void *ptr){ 
  sASttFlag = (0x01 & ~sASttFlag);
  bt002.setValue(0x0001 & sASttFlag); 
  EEPROM.write(2, 0x01 & sASttFlag);
  EEPROM.commit(); 
  if(sASttFlag == 1) {
    Blynk.virtualWrite(V6, 1);
    blynkSmokeSensorLed.on();  
  }
  else { 
    //sASttFlag = 0; 
    rf.SAlarm = 30;
    Blynk.virtualWrite(V6, 0);
    blynkSmokeSensorLed.off(); 
    blynkSmokeSensorALARMLed.off(); 
    nexSAlarm = "0";
    digitalWrite(buzzer, LOW);               
  }
  nextionCommand(val, "n004", nexSAlarm);                                   
}

/** Set door sensor On/Off State */
void bt003PopCallback(void *ptr){                                 
  dASttFlag = (0x01 & ~dASttFlag);
  bt003.setValue(0x0001 & dASttFlag); 
  EEPROM.write(3, 0x01 & dASttFlag);
  EEPROM.commit(); 
  if(dASttFlag == 1) {
    Blynk.virtualWrite(V7, 1);
    blynkDoorSensorLed.on();  
  }
  else { 
    //dASttFlag = 0; 
    rf.DAlarm = 40;
    Blynk.virtualWrite(V7, 0);
    blynkDoorSensorLed.off(); 
    blynkDoorSensorALARMLed.off(); 
    nexDAlarm = "0";
    digitalWrite(buzzer, LOW);               
  }
  nextionCommand(val, "n005", nexDAlarm);
}

/** Livingroom light */
void bt004PopCallback(void *ptr){                                 
  uint32_t dual_state;
  bt004.getValue(&dual_state);   
  lRLSttFlag = uint8_t(0x0001 & dual_state); 
  if(lRLSttFlag == 1) { 
    Blynk.virtualWrite(V23, 1); 
    blynkLivingRooomLed.on(); 
  } 
  else { 
    Blynk.virtualWrite(V23, 0); 
    blynkLivingRooomLed.off(); 
  }                            
}

/** Kitchen light */
void bt005PopCallback(void *ptr){                                 
  uint32_t dual_state;
  delay(200);
  bt005.getValue(&dual_state); 
  kLSttFlag = uint8_t(0x0001 & dual_state); 
  if(kLSttFlag == 1) { 
    Blynk.virtualWrite(V24, 1); 
    blynkKitchenLed.on(); 
  } 
  else { 
    Blynk.virtualWrite(V24, 0); 
    blynkKitchenLed.off(); 
  } 
}

/** Bedroom light */
void bt006PopCallback(void *ptr){                                 
  uint32_t dual_state;
  delay(200);
  bt006.getValue(&dual_state); 
  bRLSttFlag = uint8_t(0x0001 & dual_state); 
  if(bRLSttFlag == 1) { 
    Blynk.virtualWrite(V25, 1); 
    blynkBedRoomLed.on();
  } 
  else { 
    Blynk.virtualWrite(V25, 0);
    blynkBedRoomLed.off();
  } 
}
  
/** Kidsroom light */
void bt007PopCallback(void *ptr){                                 
  uint32_t dual_state;
  delay(200);
  bt007.getValue(&dual_state);   
  kRLSttFlag = uint8_t(0x0001 & dual_state); 
  if(kRLSttFlag == 1) { 
    Blynk.virtualWrite(V26, 1); 
    blynkKidsRoomLed.on(); 
  } 
  else { 
    Blynk.virtualWrite(V26, 0); 
    blynkKidsRoomLed.off(); 
  } 
}

/** Garage light */
void bt008PopCallback(void *ptr) 
{                                 
  delay(1);
}

/** Set combi On/Off state */
void bt009PopCallback(void *ptr){ /* Has the combi On/Off button been pressed?
                                     If it is ON, enable parameters, otherwise disable it. */ 
  if(thsERR.inTS == 1) combi.SttFlag = 0;         
  else combi.SttFlag = (0x01 & ~combi.SttFlag);
  bt009.setValue(0x0001 & combi.SttFlag);                                         
  EEPROM.write(5, 0x01 & combi.SttFlag); 
  EEPROM.commit();
  sprintf(nexCTemp, "%.1f%c", combi.T, degreeSymbol);        
  nextionCommand(txt , "t023", nexCTemp); 
  if(combi.SttFlag == 1){
    Blynk.virtualWrite(V17, 1); 
    blynkCombiLed.on(); 
  }
  else   {
    Blynk.virtualWrite(V17, 0); 
    blynkCombiLed.off(); 
    blynkHeatingLed.off(); 
  }  
} 

/** 
 * Page 1 Buttons 
 */
 
/** Back to Page 0 */
void b100PopCallback(void *ptr){ 
  currentPage = 0; 
}

/** Set combi lower limit temperature */
void b101PopCallback(void *ptr){ 
  combi.ll++;
  if(combi.ll > 3) combi.ll = 1;
  if(combi.ll == 1) {combi.LLT = 0.5; nexn100 = "1";}
  if(combi.ll == 2) {combi.LLT = 1.0; nexn100 = "2";}
  if(combi.ll == 3) {combi.LLT = 1.5; nexn100 = "3";}
  EEPROM.write(22, combi.ll); 
  EEPROM.commit();
  combi.T = int(combi.T);
  EEPROM.put(10, combi.T); 
  EEPROM.commit();
  nextionCommand(val, "n100", nexn100);  
}

/** Set combi upper limit temperature */
void b102PopCallback(void *ptr){
  combi.ul++;
  if(combi.ul > 3) combi.ul = 1;
  if(combi.ul == 1) {combi.ULT = 0.5; nexn101 = "1";}
  if(combi.ul == 2) {combi.ULT = 1.0; nexn101 = "2";}
  if(combi.ul == 3) {combi.ULT = 1.5; nexn101 = "3";}
  EEPROM.write(21, combi.ul); 
  EEPROM.commit();
  combi.T = int(combi.T);
  EEPROM.put(10, combi.T); 
  EEPROM.commit();
  nextionCommand(val, "n101", nexn101);  
}

/** Set combi temperature sensitivity */
void b103PopCallback(void *ptr){ 
  combi.tc++;
  if(combi.tc > 3) combi.tc = 1;
  if(combi.tc == 1) {combi.TCV = 0.1; nexn102 = "1";}
  if(combi.tc == 2) {combi.TCV = 0.5; nexn102 = "2";}
  if(combi.tc == 3) {combi.TCV = 1.0; nexn102 = "3";}
  EEPROM.write(20, combi.tc); 
  EEPROM.commit();
  combi.T = int(combi.T); 
  EEPROM.put(10, combi.T); 
  EEPROM.commit();
  nextionCommand(val, "n102", nexn102);
}

/** Set the time zone */
void b104PopCallback(void *ptr){ 
  timeZone++;
  showTimeZone();

  EEPROM.write(4, timeZone); 
  EEPROM.commit();
}

/** Set WiFi On/Off */
void bt100PopCallback(void *ptr){
  uint32_t dual_state;
  bt100.getValue(&dual_state); 
  WiFiSttFlag = bool(0x0001 & dual_state);
  if(WiFiSttFlag == 1) nexWiFiBt = 1;
  else nexWiFiBt = 0;  
  checkWiFi(); 
}

/** Set Blynk On/Off */
void bt101PopCallback(void *ptr){
  uint32_t dual_state;
  bt101.getValue(&dual_state); 
  BlynkSttFlag = bool(0x0001 & dual_state);
  if(BlynkSttFlag == 1) nexBlynkBt = 1;
  else nexBlynkBt = 0;
  checkBlynk(); 
}

/** Set system language */
void bt102PopCallback(void *ptr){
  uint32_t dual_state;
  bt102.getValue(&dual_state); 
  language = uint8_t(0x0001 & dual_state); 
  EEPROM.write(25, language); 
  EEPROM.commit();
}

/** Set buzzer On/Off */
void bt103PopCallback(void *ptr){
  buzzerSttFlag = (0x01 & ~buzzerSttFlag);
  bt103.setValue(0x0001 & buzzerSttFlag);              
  EEPROM.write(6, 0x01 & buzzerSttFlag); 
  EEPROM.commit(); 
  if(buzzerSttFlag == 1) {                   
    Blynk.virtualWrite(V31, 1); 
    blynkBuzzerLed.on(); 
  } 
  else { 
    Blynk.virtualWrite(V31, 0); 
    blynkBuzzerLed.off(); 
  }
}

/** 
 * Page 2 Buttons 
 */
 
/** Back to Page 0 */
void b200PopCallback(void *ptr){
  currentPage = 0; 
}

//****************************** Openweather icons **************************/

/* BIG */
int setWeatherIconBig(String icon_info){

  if(icon_info == "01d")  return 26;        // clear sky 
  if(icon_info == "01n")  return 26;        // clear sky
  if(icon_info == "02d")  return 20;        // few clouds 
  if(icon_info == "02n")  return 20;        // few clouds 
  if(icon_info == "03d")  return 19;        // scattered clouds 
  if(icon_info == "03n")  return 19;        // scattered clouds 
  if(icon_info == "04d")  return 15;        // broken clouds
  if(icon_info == "04n")  return 15;        // broken clouds
  if(icon_info == "09d")  return 23;        // shower rain 
  if(icon_info == "09n")  return 23;        // shower rain 
  if(icon_info == "10d")  return 16;        // rain 
  if(icon_info == "10n")  return 16;        // rain 
  if(icon_info == "11d")  return 27;        // thunderstorm 
  if(icon_info == "11n")  return 27;        // thunderstorm 
  if(icon_info == "13d")  return 25;        // snow
  if(icon_info == "13n")  return 25;        // snow 
  if(icon_info == "50d")  return 18;        // mist 
  if(icon_info == "50n")  return 18;        // mist  
  return 28;                                // Nothing matched: N/A 
}

/* SMALL */
int setWeatherIconSmall(String icon_info){

  if(icon_info == "01d")  return 46;        // clear sky
  if(icon_info == "01n")  return 46;        // clear sky
  if(icon_info == "02d")  return 40;        // few clouds
  if(icon_info == "02n")  return 40;        // few clouds 
  if(icon_info == "03d")  return 41;        // scattered clouds 
  if(icon_info == "03n")  return 41;        // scattered clouds 
  if(icon_info == "04d")  return 35;        // broken clouds
  if(icon_info == "04n")  return 35;        // broken clouds
  if(icon_info == "09d")  return 43;        // shower rain
  if(icon_info == "09n")  return 43;        // shower rain 
  if(icon_info == "10d")  return 36;        // rain
  if(icon_info == "10n")  return 36;        // rain 
  if(icon_info == "11d")  return 47;        // thunderstorm
  if(icon_info == "11n")  return 47;        // thunderstorm 
  if(icon_info == "13d")  return 45;        // snow
  if(icon_info == "13n")  return 45;        // snow 
  if(icon_info == "50d")  return 38;        // mist
  if(icon_info == "50n")  return 38;        // mist  
  return 48;                                // Nothing matched: N/A 
}
