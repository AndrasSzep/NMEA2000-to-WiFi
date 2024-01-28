//! config.h for N2K_DataDisplay_WiFi.ino
/* 
by Dr.András Szép under GNU General Public License (GPL).
*/

//#define M5AtomLite
//#define M5AtomU
//#define Lolin32
#ifdef M5AtomLite
#define ESP32_Type "M5AtomLite"
#define ESP32_CAN_TX_PIN GPIO_NUM_22 //  22 for M5Stack Atom Lite // 32 for AtomU white
#define ESP32_CAN_RX_PIN GPIO_NUM_19 // 19  for M5Stack Atom Lite // 26 for AtomU yellow
#endif
#ifdef M5AtomU
#define ESP32_Type "M5AtomU"
#define ESP32_CAN_TX_PIN GPIO_NUM_32 // for AtomU white
#define ESP32_CAN_RX_PIN GPIO_NUM_26 //for AtomU yellow
#endif
#ifdef  Lolin32   //  consider Wemos Lolin32Lite
#define ESP32_Type "Lolin32Lite"
#define ESP32_CAN_TX_PIN GPIO_NUM_5  // Set CAN TX port to 5  for Wemos Lolin32 Lite/home/andras/Arduino/libraries/NMEA2000_esp32xx/NMEA2000_esp32xx.h:48: note: this is the location of the previous definition #define ESP32_CAN_TX_PIN GPIO_NUM_5
#define ESP32_CAN_RX_PIN GPIO_NUM_4  // Set CAN RX port to 4  for Wemos Lolin32 Lite/home/andras/Arduino/libraries/NMEA2000_esp32xx/NMEA2000_esp32xx.h:51: note: this is the location of the previous definition #define ESP32_CAN_RX_PIN GPIO_NUM_4
#endif

//#define WINDAPPARENT
#ifndef WINDAPPARENT
#define WINDTRUE
#endif

//#define NMEA2000_ARDUINO_DUE_CAN_BUS tNMEA2000_due::CANDevice1    // Uncomment this, if you want to use CAN bus 1 instead of 0 for Arduino DUE

#define DEBUG       //additional print of all data on serial
#define PRINTNMEA
//#define STOREWIFI   // store wifi credentials on the SPIFFS

#define READWIFI    // get Wifi credentials from SPIFFS

#define ENVSENSOR       //environmental sensors connected

#ifdef ENVSENSOR
#define SDA_PIN 26    //be careful - it collides with the CANBUS TX/RX pins on M5AtomU
#define SCL_PIN 32
#endif

//OTA port  - if defined it means we can access the OverTheAir interface to update files on the SPIFFs and the programm itself
#define OTAPORT 8080    
const char*   servername  = "nmea";     //nDNS servername - http://servername.local

#define UDPPort 10110 // 10110 is the default NMEA0183 port# if defined it means we listen to the NMEA0183 messages broadcasted on this port
const char* ntpServer = "europe.pool.ntp.org";
const int timeZone = 0;  // Your time zone in hours
//String  UTC ="2023-07-11 20:30:00";
#define TIMEPERIOD 60.0

#define MAX_NMEA0183_MSG_BUF_LEN 4096
#define MAX_NMEA_FIELDS  64

const double radToDeg = 180.0 / 3.14159265358979323846;
const double msToKn = 3600.0 / 1852.0;
const double kpaTommHg = 133.322387415;
const double KToC = 273.15;
const double mpsToKn = 1.943844;

#ifdef ENVSENSOR
#include <M5StickC.h>
#include "M5_ENV.h"

SHT3X sht30;
QMP6988 qmp6988;
float tmp = 20.0;
float hum = 50.0;
float pres = 760.0;
#define ENVINTERVAL 10000     // Interval in milliseconds
#define STOREINTERVAL 60000 // store env data in SPIFFS ones/hour
#endif

#define UDPINTERVAL 1000 // ignore UDP received within 1 second
