/*
by Dr.András Szép v1.4 13.7.2023 GNU General Public License (GPL).
*/

/*
This is an AI (chatGPT) assisted development for
Arduino ESP32 code to display data from NMEA2000 data bus.
https://shop.m5stack.com/products/atom-canbus-kit-ca-is3050g

through a webserver to be seen on any mobile device for free.
I left the test input over UDP ports to for NMEA0183 data from simuators incroporated, just in case.
https://github.com/panaaj/nmeasimulator

Websockets used to autoupdate the data.
Environmental sensors incorporated and data for the last 24hours stored respectively
in the SPIFFS files /pressure.txt, /temperature.txt, /humidity.txt.
The historical environmental data displayed in the background as charts using https://www.chartjs.org

Local WiFi attributes are stored at SPIFFS in files named /ssid.txt and /password.txt = see initWiFi()
WPS never been tested but assume working.

Implemented OverTheAir update of the data files as well as the code itself on port 8080
(i.e. http://nmea2000.local:8080 ) see config.h .
*** Arduino IDE 2.0 does not support file upload, this makes much simplier uploading updates
especially in the client and stored data files.

ToDo:
      LED lights on M5Atom. Still need some ideas of colors and blinking signals


*/

#define LITTLE_FS 0
#define SPIFFS_FS 1
#define FILESYSTYPE SPIFFS_FS
// const char *otahost = "OTASPIFFS";
#define FILEBUFSIZ 4096

#ifndef WM_PORTALTIMEOUT
#define WM_PORTALTIMEOUT 180
#endif

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <WiFiServer.h>
#include <NTPClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <ArduinoJson.h>
#include "config.h"
#include "mywifi.h"
#include "BoatData.h"
#include "aux_functions.h"
#include <math.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <N2kMessagesEnumToStr.h>

unsigned long previousMillis = 0;       // Variable to store the previous time
unsigned long storedMillis = 0;         // time of last stored env.data
unsigned long previousPacketMillis = 0; // Variable to store the previous packet reception time

char nmeaLine[MAX_NMEA0183_MSG_BUF_LEN]; // NMEA0183 message buffer
size_t i = 0, j = 1;                     // indexers
uint8_t *pointer_to_int;                 // pointer to void *data (!)
int noOfFields = MAX_NMEA_FIELDS;        // max number of NMEA0183 fields
String nmeaStringData[MAX_NMEA_FIELDS + 1];

WiFiUDP udp;
NTPClient timeClient(udp, ntpServer, timeZone);
time_t utcTime = 0;
sBoatData stringBD; // Strings with BoatData
tBoatData BoatData; // BoatData

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void SystemTime(const tN2kMsg &N2kMsg);
void Rudder(const tN2kMsg &N2kMsg);
void EngineRapid(const tN2kMsg &N2kMsg);
void EngineDynamicParameters(const tN2kMsg &N2kMsg);
void TransmissionParameters(const tN2kMsg &N2kMsg);
void TripFuelConsumption(const tN2kMsg &N2kMsg);
void Speed(const tN2kMsg &N2kMsg);
void WaterDepth(const tN2kMsg &N2kMsg);
void BinaryStatus(const tN2kMsg &N2kMsg);
void FluidLevel(const tN2kMsg &N2kMsg);
void OutsideEnvironmental(const tN2kMsg &N2kMsg);
void Temperature(const tN2kMsg &N2kMsg);
void TemperatureExt(const tN2kMsg &N2kMsg);
void DCStatus(const tN2kMsg &N2kMsg);
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg);
void COGSOG(const tN2kMsg &N2kMsg);
void GNSS(const tN2kMsg &N2kMsg);
void LocalOffset(const tN2kMsg &N2kMsg);
void Attitude(const tN2kMsg &N2kMsg);
void Heading(const tN2kMsg &N2kMsg);
void Humidity(const tN2kMsg &N2kMsg);
void Pressure(const tN2kMsg &N2kMsg);
void UserDatumSettings(const tN2kMsg &N2kMsg);
void GNSSSatsInView(const tN2kMsg &N2kMsg);

tNMEA2000Handler NMEA2000Handlers[]={
  {126992L,&SystemTime},
  {127245L,&Rudder },
  {127250L,&Heading},
  {127257L,&Attitude},
  {127488L,&EngineRapid},
  {127489L,&EngineDynamicParameters},
  {127493L,&TransmissionParameters},
  {127497L,&TripFuelConsumption},
  {127501L,&BinaryStatus},
  {127505L,&FluidLevel},
  {127506L,&DCStatus},
  {127513L,&BatteryConfigurationStatus},
  {128259L,&Speed},
  {128267L,&WaterDepth},
  {129026L,&COGSOG},
  {129029L,&GNSS},
  {129033L,&LocalOffset},
  {129045L,&UserDatumSettings},
  {129540L,&GNSSSatsInView},
  {130310L,&OutsideEnvironmental},
  {130312L,&Temperature},
  {130313L,&Humidity},
  {130314L,&Pressure},
  {130316L,&TemperatureExt},
  {0,0}
};

Stream *OutputStream;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg);

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
// Create a WebSocket object
AsyncWebSocket ws("/ws");
char sn[30];                // to store mDNS name
WebServer servOTA(OTAPORT); // webserver for OnTheAir update on port 8080
bool fsFound = false;
#include "webpages.h"
#include "filecode.h"

double lastTime = 0.0;
String message = "";
String timedate = "1957-04-28 10:01:02"; // guess when the new era was born
String rpm = "0";
String depth = "0";
String speed = "0";
String heading = "90";
String cog = "0";
String sog = "0";
String windspeed = "0";
String winddir = "0";
String longitude = "45º10'20\"N";
String latitude = "05º10'20\"E";
String watertemp = "18";
String humidity = "50";
String pressure = "760";
String airtemp = "21";
String pressurearray = "760,760,760,760,760,760,760,760,760,760,760,760,760,760,760,760,760,760,760,760,760,760,760,760,760";

DynamicJsonDocument jsonDoc(1024);


void notifyClients()
{
  String jsonString;
  serializeJson(jsonDoc, jsonString);
  // Send the JSON string as a response
  //        request->send(200, "application/json", jsonString);
  //        client->text(jsonString);
  if (jsonString.length() != 0 && jsonString != "null")
  {
#ifdef DEBUG
    Serial.print("notifyClients: ");
    Serial.println(jsonString);
#endif
    ws.textAll(jsonString);
    jsonDoc.clear();
  }
}

void handleRequest(AsyncWebServerRequest *request)
{
  if (request->url() == "/historicdata")
  {
    // Create a JSON document
    DynamicJsonDocument jsonDoc(1024);
    // read data
    jsonDoc["histtemp"] = readStoredData("/temperature");
    jsonDoc["histhum"] = readStoredData("/humidity");
    jsonDoc["histpres"] = readStoredData("/pressure");
    jsonDoc["histwater"] = readStoredData("/water");
    notifyClients();
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_CONNECT)
  {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  }
  else if (type == WS_EVT_DATA)
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
      // Handle received text message
      data[len] = '\0'; // Null-terminate the received data
      Serial.print("Received message: ");
      Serial.println((char *)data);
      // Check if the request is '/historicdata'
      if (strcmp((char *)data, "/historicdata") == 0)
      {
        // Prepare JSON data
        DynamicJsonDocument jsonDoc(1024);
        // read data
        jsonDoc["histtemp"] = readStoredData("/temperature.txt");
        jsonDoc["histhum"] = readStoredData("/humidity.txt");
        jsonDoc["histpres"] = readStoredData("/pressure.txt");
        jsonDoc["histwater"] = readStoredData("/water.txt");
        notifyClients();
      }
      else
      {
        // Handle other requests here
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n**** NMEA2000 ->-> WEB ****\n");
  delay(100);
#ifdef ENVSENSOR
//    M5.begin();             // Init M5StickC.  初始化M5StickC
    Wire.begin();  // Wire init, adding the I2C bus.  Wire初始化, 加入i2c总线
    qmp6988.init();
  Serial.println(F("ENVIII Hat(SHT30 and QMP6988) has initialized "));
#endif

  initFS(false, false); // initialalize file system SPIFFS
  initWiFi();           // init WifI from SPIFFS or WPS
  // Initialize the NTP client
  timeClient.begin();
  stringBD.UTC = getDT(); // store UTC
  
#ifdef UDPPort
  // Initialize UDP
  udp.begin(UDPPort);
  Serial.print("UDP listening on: ");
  Serial.println(UDPPort);
#endif

  OutputStream=&Serial;

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(SPIFFS, "/index.html", "text/html"); });

  // Register the request handler
  server.on("/historicdata", HTTP_GET, handleRequest);

  server.serveStatic("/", SPIFFS, "/");

  // Start server
  server.begin();
  Serial.println("WebServer started");
  delay(100);
  /*use mdns for host name resolution*/
  uint64_t chipID = ESP.getEfuseMac();

  sprintf(sn, "%s%llu", servername, chipID);

  if (!MDNS.begin(sn)) // http://nmeaXXXXXXX.local
    Serial.println("Error setting up MDNS responder!");
  else
    Serial.print("mDNS responder started, OTA: http://");
  Serial.print(sn);

#ifdef OTAPORT
  Serial.printf(".local:%d\n", OTAPORT);
  servOTA.on("/", HTTP_GET, handleMain);

  // upload file to FS. Three callbacks
  servOTA.on(
      "/update", HTTP_POST, []()
      {
    servOTA.sendHeader("Connection", "close");
    servOTA.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); },
      []()
      {
        HTTPUpload &upload = servOTA.upload();
        if (upload.status == UPLOAD_FILE_START)
        {
          Serial.printf("Update: %s\n", upload.filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN))
          { // start with max available size
            Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_WRITE)
        {
          /* flashing firmware to ESP*/
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
          {
            Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_END)
        {
          if (Update.end(true))
          { // true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
          }
          else
          {
            Update.printError(Serial);
          }
        }
      });

  servOTA.on("/delete", HTTP_GET, handleFileDelete);
  servOTA.on("/main", HTTP_GET, handleMain); // JSON format used by /edit
  // second callback handles file uploads at that location
  servOTA.on(
      "/edit", HTTP_POST, []()
      { servOTA.send(200, "text/html", "<meta http-equiv='refresh' content='1;url=/main'>File uploaded. <a href=/main>Back to list</a>"); },
      handleFileUpload);
  servOTA.onNotFound([]()
                     {if(!handleFileRead(servOTA.uri())) servOTA.send(404, "text/plain", "404 FileNotFound"); });

  servOTA.begin();
#endif
  //  NMEA2000.SetN2kCANReceiveFrameBufSize(50);
  // Do not forward bus messages at all
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  NMEA2000.SetForwardStream(OutputStream);
  // Set false below, if you do not want to see messages parsed to HEX withing library
  NMEA2000.EnableForward(false);
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
//  NMEA2000.SetN2kCANMsgBufSize(2);
  NMEA2000.Open();
  OutputStream->print("Running...");
}

//*****************************************************************************
template<typename T> void PrintLabelValWithConversionCheckUnDef(const char* label, T val, double (*ConvFunc)(double val)=0, bool AddLf=false, int8_t Desim=-1 ) {
#ifdef PRINTNMEA
  OutputStream->print(label);
  if (!N2kIsNA(val)) {
    if ( Desim<0 ) {
      if (ConvFunc) { OutputStream->print(ConvFunc(val)); } else { OutputStream->print(val); }
    } else {
      if (ConvFunc) { OutputStream->print(ConvFunc(val),Desim); } else { OutputStream->print(val,Desim); }
    }
  } else OutputStream->print("not available");
  if (AddLf) OutputStream->println();
#endif
}

//**#timedate***************************************************************************
void SystemTime(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    uint16_t SystemDate;
    double SystemTime;
    tN2kTimeSource TimeSource;
    
    if (ParseN2kSystemTime(N2kMsg,SID,SystemDate,SystemTime,TimeSource) ) 
    {
    #ifdef PRINTNMEA
      OutputStream->println("System time:");
      PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
      PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ",SystemDate,0,true);
      PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ",SystemTime,0,true);
      OutputStream->print("  time source: "); 
      PrintN2kEnumType(TimeSource,OutputStream);
    #endif
      jsonDoc["timedate"] = convertDaysToDate(SystemDate) + "___" + secondsToTimeString((int)SystemTime) +"___";
      notifyClients();
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void Rudder(const tN2kMsg &N2kMsg) {
    unsigned char Instance;
    tN2kRudderDirectionOrder RudderDirectionOrder;
    double RudderPosition;
    double AngleOrder;
    
    if (ParseN2kRudder(N2kMsg,RudderPosition,Instance,RudderDirectionOrder,AngleOrder) ) {
      PrintLabelValWithConversionCheckUnDef("Rudder: ",Instance,0,true);
      PrintLabelValWithConversionCheckUnDef("  position (deg): ",RudderPosition,&RadToDeg,true);
                        OutputStream->print("  direction order: "); PrintN2kEnumType(RudderDirectionOrder,OutputStream);
      PrintLabelValWithConversionCheckUnDef("  angle order (deg): ",AngleOrder,&RadToDeg,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//**#rpm**************************************************************************
void EngineRapid(const tN2kMsg &N2kMsg) {
    unsigned char EngineInstance;
    double EngineSpeed;
    double EngineBoostPressure;
    int8_t EngineTiltTrim;
    
    if (ParseN2kEngineParamRapid(N2kMsg,EngineInstance,EngineSpeed,EngineBoostPressure,EngineTiltTrim) ) {
      PrintLabelValWithConversionCheckUnDef("Engine rapid params: ",EngineInstance,0,true);
      PrintLabelValWithConversionCheckUnDef("  RPM: ",EngineSpeed,0,true);
      PrintLabelValWithConversionCheckUnDef("  boost pressure (Pa): ",EngineBoostPressure,0,true);
      PrintLabelValWithConversionCheckUnDef("  tilt trim: ",EngineTiltTrim,0,true);
      rpm = String((int)EngineSpeed);
      jsonDoc["rpm"] = rpm;
      notifyClients();
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void EngineDynamicParameters(const tN2kMsg &N2kMsg) {
    unsigned char EngineInstance;
    double EngineOilPress;
    double EngineOilTemp;
    double EngineCoolantTemp;
    double AltenatorVoltage;
    double FuelRate;
    double EngineHours;
    double EngineCoolantPress;
    double EngineFuelPress; 
    int8_t EngineLoad;
    int8_t EngineTorque;
    tN2kEngineDiscreteStatus1 Status1;
    tN2kEngineDiscreteStatus2 Status2;
    
    if (ParseN2kEngineDynamicParam(N2kMsg,EngineInstance,EngineOilPress,EngineOilTemp,EngineCoolantTemp,
                                 AltenatorVoltage,FuelRate,EngineHours,
                                 EngineCoolantPress,EngineFuelPress,
                                 EngineLoad,EngineTorque,Status1,Status2) ) {
      PrintLabelValWithConversionCheckUnDef("Engine dynamic params: ",EngineInstance,0,true);
      PrintLabelValWithConversionCheckUnDef("  oil pressure (Pa): ",EngineOilPress,0,true);
      PrintLabelValWithConversionCheckUnDef("  oil temp (C): ",EngineOilTemp,&KelvinToC,true);
      PrintLabelValWithConversionCheckUnDef("  coolant temp (C): ",EngineCoolantTemp,&KelvinToC,true);
      PrintLabelValWithConversionCheckUnDef("  altenator voltage (V): ",AltenatorVoltage,0,true);
      PrintLabelValWithConversionCheckUnDef("  fuel rate (l/h): ",FuelRate,0,true);
      PrintLabelValWithConversionCheckUnDef("  engine hours (h): ",EngineHours,&SecondsToh,true);
      PrintLabelValWithConversionCheckUnDef("  coolant pressure (Pa): ",EngineCoolantPress,0,true);
      PrintLabelValWithConversionCheckUnDef("  fuel pressure (Pa): ",EngineFuelPress,0,true);
      PrintLabelValWithConversionCheckUnDef("  engine load (%): ",EngineLoad,0,true);
      PrintLabelValWithConversionCheckUnDef("  engine torque (%): ",EngineTorque,0,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void TransmissionParameters(const tN2kMsg &N2kMsg) {
    unsigned char EngineInstance;
    tN2kTransmissionGear TransmissionGear;
    double OilPressure;
    double OilTemperature;
    unsigned char DiscreteStatus1;
    
    if (ParseN2kTransmissionParameters(N2kMsg,EngineInstance, TransmissionGear, OilPressure, OilTemperature, DiscreteStatus1) ) {
      PrintLabelValWithConversionCheckUnDef("Transmission params: ",EngineInstance,0,true);
                        OutputStream->print("  gear: "); PrintN2kEnumType(TransmissionGear,OutputStream);
      PrintLabelValWithConversionCheckUnDef("  oil pressure (Pa): ",OilPressure,0,true);
      PrintLabelValWithConversionCheckUnDef("  oil temperature (C): ",OilTemperature,&KelvinToC,true);
      PrintLabelValWithConversionCheckUnDef("  discrete status: ",DiscreteStatus1,0,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void TripFuelConsumption(const tN2kMsg &N2kMsg) {
    unsigned char EngineInstance;
    double TripFuelUsed;
    double FuelRateAverage; 
    double FuelRateEconomy; 
    double InstantaneousFuelEconomy; 
    
    if (ParseN2kEngineTripParameters(N2kMsg,EngineInstance, TripFuelUsed, FuelRateAverage, FuelRateEconomy, InstantaneousFuelEconomy) ) {
      PrintLabelValWithConversionCheckUnDef("Trip fuel consumption: ",EngineInstance,0,true);
      PrintLabelValWithConversionCheckUnDef("  fuel used (l): ",TripFuelUsed,0,true);
      PrintLabelValWithConversionCheckUnDef("  average fuel rate (l/h): ",FuelRateAverage,0,true);
      PrintLabelValWithConversionCheckUnDef("  economy fuel rate (l/h): ",FuelRateEconomy,0,true);
      PrintLabelValWithConversionCheckUnDef("  instantaneous fuel economy (l/h): ",InstantaneousFuelEconomy,0,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//**#heading***************************************************************************
void Heading(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    tN2kHeadingReference HeadingReference;
    double Heading;
    double Deviation;
    double Variation;
    
    if (ParseN2kHeading(N2kMsg,SID,Heading,Deviation,Variation,HeadingReference) ) {
      #ifdef PRINTNMEA
                      OutputStream->println("Heading:");
      PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
                        OutputStream->print("  reference: "); PrintN2kEnumType(HeadingReference,OutputStream);
      PrintLabelValWithConversionCheckUnDef("  Heading (deg): ",Heading,&RadToDeg,true);
      PrintLabelValWithConversionCheckUnDef("  Deviation (deg): ",Deviation,&RadToDeg,true);
      PrintLabelValWithConversionCheckUnDef("  Variation (deg): ",Variation,&RadToDeg,true);
      #endif
      heading = String((int)(Heading * radToDeg));
      jsonDoc["heading"] = heading;
      notifyClients();
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void COGSOG(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    tN2kHeadingReference HeadingReference;
    double COG;
    double SOG;
    
    if (ParseN2kCOGSOGRapid(N2kMsg,SID,HeadingReference,COG,SOG) ) {
                      OutputStream->println("COG/SOG:");
      PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
                        OutputStream->print("  reference: "); PrintN2kEnumType(HeadingReference,OutputStream);
      PrintLabelValWithConversionCheckUnDef("  COG (deg): ",COG,&RadToDeg,true);
      PrintLabelValWithConversionCheckUnDef("  SOG (m/s): ",SOG,0,true);
      cog = String((int)(COG * radToDeg));
      jsonDoc["cog"] = cog;
      sog = String((int)(SOG * mpsToKn));
      jsonDoc["sog"] = sog;
      notifyClients();
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*#latlong*#timedate***************************************************************************
void GNSS(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  uint16_t DaysSince1970;
  double SecondsSinceMidnight; 
  double Latitude;
  double Longitude;
  double Altitude; 
  tN2kGNSStype GNSStype;
  tN2kGNSSmethod GNSSmethod;
  unsigned char nSatellites;
  double HDOP;
  double PDOP;
  double GeoidalSeparation;
  unsigned char nReferenceStations;
  tN2kGNSStype ReferenceStationType;
  uint16_t ReferenceSationID;
  double AgeOfCorrection;

  if (ParseN2kGNSS(N2kMsg,SID,DaysSince1970,SecondsSinceMidnight,
                Latitude,Longitude,Altitude,
                GNSStype,GNSSmethod,
                nSatellites,HDOP,PDOP,GeoidalSeparation,
                nReferenceStations,ReferenceStationType,ReferenceSationID,
                AgeOfCorrection) ) {
    #ifdef PRINTNMEA
                    OutputStream->println("GNSS info:");
    PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
    PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ",DaysSince1970,0,true);
    PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ",SecondsSinceMidnight,0,true);
    PrintLabelValWithConversionCheckUnDef("  latitude: ",Latitude,0,true,9);
    PrintLabelValWithConversionCheckUnDef("  longitude: ",Longitude,0,true,9);
    PrintLabelValWithConversionCheckUnDef("  altitude: (m): ",Altitude,0,true);
                      OutputStream->print("  GNSS type: "); PrintN2kEnumType(GNSStype,OutputStream);
                      OutputStream->print("  GNSS method: "); PrintN2kEnumType(GNSSmethod,OutputStream);
    PrintLabelValWithConversionCheckUnDef("  satellite count: ",nSatellites,0,true);
    PrintLabelValWithConversionCheckUnDef("  HDOP: ",HDOP,0,true);
    PrintLabelValWithConversionCheckUnDef("  PDOP: ",PDOP,0,true);
    PrintLabelValWithConversionCheckUnDef("  geoidal separation: ",GeoidalSeparation,0,true);
    PrintLabelValWithConversionCheckUnDef("  reference stations: ",nReferenceStations,0,true);
    #endif
    jsonDoc["timedate"] = convertDaysToDate(DaysSince1970) + " " + secondsToTimeString((int)SecondsSinceMidnight) + "___";
    BoatData.Latitude = Latitude;
    BoatData.Longitude = Longitude;
    jsonDoc["latitude"] = GPStoString(BoatData.Latitude);
    jsonDoc["longitude"] = GPStoString(BoatData.Longitude);
    notifyClients();
  } else {
    OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
  }
}

//*****************************************************************************
void UserDatumSettings(const tN2kMsg &N2kMsg) {
  if (N2kMsg.PGN!=129045L) return;
  int Index=0;
  double val;

  OutputStream->println("User Datum Settings: ");
  val=N2kMsg.Get4ByteDouble(1e-2,Index);
  PrintLabelValWithConversionCheckUnDef("  delta x (m): ",val,0,true);
  val=N2kMsg.Get4ByteDouble(1e-2,Index);
  PrintLabelValWithConversionCheckUnDef("  delta y (m): ",val,0,true);
  val=N2kMsg.Get4ByteDouble(1e-2,Index);
  PrintLabelValWithConversionCheckUnDef("  delta z (m): ",val,0,true);
  val=N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  rotation in x (deg): ",val,&RadToDeg,true,5);
  val=N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  rotation in y (deg): ",val,&RadToDeg,true,5);
  val=N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  rotation in z (deg): ",val,&RadToDeg,true,5);
  val=N2kMsg.GetFloat(Index);
  PrintLabelValWithConversionCheckUnDef("  scale: ",val,0,true,3);
}

//*****************************************************************************
void GNSSSatsInView(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  tN2kRangeResidualMode Mode;
  uint8_t NumberOfSVs;
  tSatelliteInfo SatelliteInfo;

  if (ParseN2kPGNSatellitesInView(N2kMsg,SID,Mode,NumberOfSVs) ) {
    OutputStream->println("Satellites in view: ");
                      OutputStream->print("  mode: "); OutputStream->println(Mode);
                      OutputStream->print("  number of satellites: ");  OutputStream->println(NumberOfSVs);
    for ( uint8_t i=0; i<NumberOfSVs && ParseN2kPGNSatellitesInView(N2kMsg,i,SatelliteInfo); i++) {
                        OutputStream->print("  Satellite PRN: ");  OutputStream->println(SatelliteInfo.PRN);
      PrintLabelValWithConversionCheckUnDef("    elevation: ",SatelliteInfo.Elevation,&RadToDeg,true,1);
      PrintLabelValWithConversionCheckUnDef("    azimuth:   ",SatelliteInfo.Azimuth,&RadToDeg,true,1);
      PrintLabelValWithConversionCheckUnDef("    SNR:       ",SatelliteInfo.SNR,0,true,1);
      PrintLabelValWithConversionCheckUnDef("    residuals: ",SatelliteInfo.RangeResiduals,0,true,1);
                        OutputStream->print("    status: "); OutputStream->println(SatelliteInfo.UsageStatus);
    }
  }
}

//*****************************************************************************
void LocalOffset(const tN2kMsg &N2kMsg) {
    uint16_t SystemDate;
    double SystemTime;
    int16_t Offset;
    
    if (ParseN2kLocalOffset(N2kMsg,SystemDate,SystemTime,Offset) ) {
                      OutputStream->println("Date,time and local offset: ");
      PrintLabelValWithConversionCheckUnDef("  days since 1.1.1970: ",SystemDate,0,true);
      PrintLabelValWithConversionCheckUnDef("  seconds since midnight: ",SystemTime,0,true);
      PrintLabelValWithConversionCheckUnDef("  local offset (min): ",Offset,0,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//**#watertemp*#airtemp*#pressure*************************************************************************
void OutsideEnvironmental(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double WaterTemperature;
    double OutsideAmbientAirTemperature;
    double AtmosphericPressure;
    
    if (ParseN2kOutsideEnvironmentalParameters(N2kMsg,SID,WaterTemperature,OutsideAmbientAirTemperature,AtmosphericPressure) ) {
      PrintLabelValWithConversionCheckUnDef("Water temp: ",WaterTemperature,&KelvinToC);
      PrintLabelValWithConversionCheckUnDef(", outside ambient temp: ",OutsideAmbientAirTemperature,&KelvinToC);
      PrintLabelValWithConversionCheckUnDef(", pressure: ",AtmosphericPressure,0,true);
      airtemp = String(KelvinToC(OutsideAmbientAirTemperature), 1);
      jsonDoc["airtemp"] = airtemp;
      watertemp = String(KelvinToC(WaterTemperature), 1);
      jsonDoc["watertemp"] = watertemp;
      pressure = String((int)(AtmosphericPressure/kpaTommHg));
      jsonDoc["pressure"] = pressure;
      notifyClients();
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//**#airtemp***************************************************************************
void Temperature(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    unsigned char TempInstance;
    tN2kTempSource TempSource;
    double ActualTemperature;
    double SetTemperature;
    
    if (ParseN2kTemperature(N2kMsg,SID,TempInstance,TempSource,ActualTemperature,SetTemperature) ) {
    #ifdef PRINTNMEA
                        OutputStream->print("Temperature source: "); PrintN2kEnumType(TempSource,OutputStream,false);
      PrintLabelValWithConversionCheckUnDef(", actual temperature: ",ActualTemperature,&KelvinToC);
      PrintLabelValWithConversionCheckUnDef(", set temperature: ",SetTemperature,&KelvinToC,true);
    #endif
      airtemp = String(KelvinToC(ActualTemperature),1);
      jsonDoc["airtemp"] = airtemp;
      notifyClients();
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//**#humidity***************************************************************************
void Humidity(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    unsigned char Instance;
    tN2kHumiditySource HumiditySource;
    double ActualHumidity,SetHumidity;
    
    if ( ParseN2kHumidity(N2kMsg,SID,Instance,HumiditySource,ActualHumidity,SetHumidity) ) 
    {
      #ifdef PRINTNMEA
      OutputStream->print("Humidity source: "); 
      PrintN2kEnumType(HumiditySource,OutputStream,false);
      PrintLabelValWithConversionCheckUnDef(", humidity: ",ActualHumidity,0,false);
      PrintLabelValWithConversionCheckUnDef(", set humidity: ",SetHumidity,0,true);
      #endif
      humidity = String((int)ActualHumidity);
      jsonDoc["humidity"] = humidity;
      notifyClients();
    } else 
    {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//**#pressure***************************************************************************
void Pressure(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    unsigned char Instance;
    tN2kPressureSource PressureSource;
    double ActualPressure;
    
    if ( ParseN2kPressure(N2kMsg,SID,Instance,PressureSource,ActualPressure) ) {
      #ifdef PRINTNMEA
                        OutputStream->print("Pressure source: "); PrintN2kEnumType(PressureSource,OutputStream,false);
      PrintLabelValWithConversionCheckUnDef(", pressure: ",ActualPressure,&PascalTomBar,true);
      #endif
      pressure = String((int)(ActualPressure/kpaTommHg));
      jsonDoc["pressure"] = pressure;
      notifyClients();
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void TemperatureExt(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    unsigned char TempInstance;
    tN2kTempSource TempSource;
    double ActualTemperature;
    double SetTemperature;
    
    if (ParseN2kTemperatureExt(N2kMsg,SID,TempInstance,TempSource,ActualTemperature,SetTemperature) ) {
                        OutputStream->print("Temperature source: "); PrintN2kEnumType(TempSource,OutputStream,false);
      PrintLabelValWithConversionCheckUnDef(", actual temperature: ",ActualTemperature,&KelvinToC);
      PrintLabelValWithConversionCheckUnDef(", set temperature: ",SetTemperature,&KelvinToC,true);
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void BatteryConfigurationStatus(const tN2kMsg &N2kMsg) {
    unsigned char BatInstance;
    tN2kBatType BatType;
    tN2kBatEqSupport SupportsEqual;
    tN2kBatNomVolt BatNominalVoltage;
    tN2kBatChem BatChemistry;
    double BatCapacity;
    int8_t BatTemperatureCoefficient;
    double PeukertExponent; 
    int8_t ChargeEfficiencyFactor;

    if (ParseN2kBatConf(N2kMsg,BatInstance,BatType,SupportsEqual,BatNominalVoltage,BatChemistry,BatCapacity,BatTemperatureCoefficient,PeukertExponent,ChargeEfficiencyFactor) ) {
      PrintLabelValWithConversionCheckUnDef("Battery instance: ",BatInstance,0,true);
                        OutputStream->print("  - type: "); PrintN2kEnumType(BatType,OutputStream);
                        OutputStream->print("  - support equal.: "); PrintN2kEnumType(SupportsEqual,OutputStream);
                        OutputStream->print("  - nominal voltage: "); PrintN2kEnumType(BatNominalVoltage,OutputStream);
                        OutputStream->print("  - chemistry: "); PrintN2kEnumType(BatChemistry,OutputStream);
      PrintLabelValWithConversionCheckUnDef("  - capacity (Ah): ",BatCapacity,&CoulombToAh,true);
      PrintLabelValWithConversionCheckUnDef("  - temperature coefficient (%): ",BatTemperatureCoefficient,0,true);
      PrintLabelValWithConversionCheckUnDef("  - peukert exponent: ",PeukertExponent,0,true);
      PrintLabelValWithConversionCheckUnDef("  - charge efficiency factor (%): ",ChargeEfficiencyFactor,0,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
void DCStatus(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    unsigned char DCInstance;
    tN2kDCType DCType;
    unsigned char StateOfCharge;
    unsigned char StateOfHealth;
    double TimeRemaining;
    double RippleVoltage;
    double Capacity;

    if (ParseN2kDCStatus(N2kMsg,SID,DCInstance,DCType,StateOfCharge,StateOfHealth,TimeRemaining,RippleVoltage,Capacity) ) {
      OutputStream->print("DC instance: ");
      OutputStream->println(DCInstance);
      OutputStream->print("  - type: "); PrintN2kEnumType(DCType,OutputStream);
      OutputStream->print("  - state of charge (%): "); OutputStream->println(StateOfCharge);
      OutputStream->print("  - state of health (%): "); OutputStream->println(StateOfHealth);
      OutputStream->print("  - time remaining (h): "); OutputStream->println(TimeRemaining/60);
      OutputStream->print("  - ripple voltage: "); OutputStream->println(RippleVoltage);
      OutputStream->print("  - capacity: "); OutputStream->println(Capacity);
    } else {
      OutputStream->print("Failed to parse PGN: ");  OutputStream->println(N2kMsg.PGN);
    }
}

//**#speed***************************************************************************
void Speed(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double SOW;
    double SOG;
    tN2kSpeedWaterReferenceType SWRT;

    if (ParseN2kBoatSpeed(N2kMsg,SID,SOW,SOG,SWRT) ) {
    #ifdef PRINTNMEA
      OutputStream->print("Boat speed:");
      PrintLabelValWithConversionCheckUnDef(" SOW:",N2kIsNA(SOW)?SOW:msToKnots(SOW));
      PrintLabelValWithConversionCheckUnDef(", SOG:",N2kIsNA(SOG)?SOG:msToKnots(SOG));
      OutputStream->print(", ");
      PrintN2kEnumType(SWRT,OutputStream,true);
    #endif
      speed = String( SOW, 1);
      jsonDoc["speed"] = speed;
      notifyClients();
    }
}

//**#depth***************************************************************************
void WaterDepth(const tN2kMsg &N2kMsg) {
  unsigned char SID;
  double DepthBelowTransducer;
  double Offset;

  if (ParseN2kWaterDepth(N2kMsg,SID,DepthBelowTransducer,Offset) ) {
    #ifdef PRINTNMEA
    if ( N2kIsNA(Offset) || Offset == 0 ) {
      PrintLabelValWithConversionCheckUnDef("Depth below transducer",DepthBelowTransducer);
      if ( N2kIsNA(Offset) ) {
        OutputStream->println(", offset not available");
      } else {
        OutputStream->println(", offset=0");
      }
    } else {
      if (Offset>0) {
        OutputStream->print("Water depth:");
      } else {
        OutputStream->print("Depth below keel:");
      }
      if ( !N2kIsNA(DepthBelowTransducer) ) { 
        OutputStream->println(DepthBelowTransducer+Offset); 
      } else {  OutputStream->println(" not available"); }
    }
    #endif
    depth = String(DepthBelowTransducer, 1);
    jsonDoc["depth"] = depth;
    notifyClients();
  }
}

//*****************************************************************************
void printLLNumber(Stream *OutputStream, unsigned long long n, uint8_t base=10)
{
  unsigned char buf[16 * sizeof(long)]; // Assumes 8-bit chars.
  unsigned long long i = 0;

  if (n == 0) {
    OutputStream->print('0');
    return;
  }

  while (n > 0) {
    buf[i++] = n % base;
    n /= base;
  }

  for (; i > 0; i--)
    OutputStream->print((char) (buf[i - 1] < 10 ?
      '0' + buf[i - 1] :
      'A' + buf[i - 1] - 10));
}

//*****************************************************************************
void BinaryStatusFull(const tN2kMsg &N2kMsg) {
    unsigned char BankInstance;
    tN2kBinaryStatus BankStatus;

    if (ParseN2kBinaryStatus(N2kMsg,BankInstance,BankStatus) ) {
      OutputStream->print("Binary status for bank "); OutputStream->print(BankInstance); OutputStream->println(":");
      OutputStream->print("  "); //printLLNumber(OutputStream,BankStatus,16);
      for (uint8_t i=1; i<=28; i++) {
        if (i>1) OutputStream->print(",");
        PrintN2kEnumType(N2kGetStatusOnBinaryStatus(BankStatus,i),OutputStream,false);
      }
      OutputStream->println();
    }
}

//*****************************************************************************
void BinaryStatus(const tN2kMsg &N2kMsg) {
    unsigned char BankInstance;
    tN2kOnOff Status1,Status2,Status3,Status4;

    if (ParseN2kBinaryStatus(N2kMsg,BankInstance,Status1,Status2,Status3,Status4) ) {
      if (BankInstance>2) { // note that this is only for testing different methods. MessageSender.ini sends 4 status for instace 2
        BinaryStatusFull(N2kMsg);
      } else {
        OutputStream->print("Binary status for bank "); OutputStream->print(BankInstance); OutputStream->println(":");
        OutputStream->print("  Status1=");PrintN2kEnumType(Status1,OutputStream,false);
        OutputStream->print(", Status2=");PrintN2kEnumType(Status2,OutputStream,false);
        OutputStream->print(", Status3=");PrintN2kEnumType(Status3,OutputStream,false);
        OutputStream->print(", Status4=");PrintN2kEnumType(Status4,OutputStream,false);
        OutputStream->println();
      }
    }
}

//*****************************************************************************
void FluidLevel(const tN2kMsg &N2kMsg) {
    unsigned char Instance;
    tN2kFluidType FluidType;
    double Level=0;
    double Capacity=0;

    if (ParseN2kFluidLevel(N2kMsg,Instance,FluidType,Level,Capacity) ) {
      switch (FluidType) {
        case N2kft_Fuel:
          OutputStream->print("Fuel level :");
          break;
        case N2kft_Water:
          OutputStream->print("Water level :");
          break;
        case N2kft_GrayWater:
          OutputStream->print("Gray water level :");
          break;
        case N2kft_LiveWell:
          OutputStream->print("Live well level :");
          break;
        case N2kft_Oil:
          OutputStream->print("Oil level :");
          break;
        case N2kft_BlackWater:
          OutputStream->print("Black water level :");
          break;
        case N2kft_FuelGasoline:
          OutputStream->print("Gasoline level :");
          break;
        case N2kft_Error:
          OutputStream->print("Error level :");
          break;
        case N2kft_Unavailable:
          OutputStream->print("Unknown level :");
          break;
      }
      OutputStream->print(Level); OutputStream->print("%"); 
      OutputStream->print(" ("); OutputStream->print(Capacity*Level/100); OutputStream->print("l)");
      OutputStream->print(" capacity :"); OutputStream->println(Capacity);
    }
}

//*****************************************************************************
void Attitude(const tN2kMsg &N2kMsg) {
    unsigned char SID;
    double Yaw;
    double Pitch;
    double Roll;
    
    if (ParseN2kAttitude(N2kMsg,SID,Yaw,Pitch,Roll) ) {
                      OutputStream->println("Attitude:");
      PrintLabelValWithConversionCheckUnDef("  SID: ",SID,0,true);
      PrintLabelValWithConversionCheckUnDef("  Yaw (deg): ",Yaw,&RadToDeg,true);
      PrintLabelValWithConversionCheckUnDef("  Pitch (deg): ",Pitch,&RadToDeg,true);
      PrintLabelValWithConversionCheckUnDef("  Roll (deg): ",Roll,&RadToDeg,true);
    } else {
      OutputStream->print("Failed to parse PGN: "); OutputStream->println(N2kMsg.PGN);
    }
}

//*****************************************************************************
//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  
  // Find handler
//  OutputStream->print("In Main Handler: "); OutputStream->println(N2kMsg.PGN);
  for (iHandler=0; NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); iHandler++);
  
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  }
}

//*****************************************************************************
void loop() 
{
  jsonDoc.clear();
  jsonDoc["chipid"] = String(sn);
#ifdef ENVSENSOR
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= ENVINTERVAL)
  {
    pres = qmp6988.calcPressure();
    if (sht30.get() == 0) {  // Obtain the data of shT30.  获取sht30的数据
        tmp = sht30.cTemp;   // Store the temperature obtained from shT30.
        hum = sht30.humidity;  // Store the humidity obtained from the SHT30.
    } else {
        tmp = 20, hum = 50;
    }
    airtemp = String(tmp, 1) + "°C";
    humidity = String(hum, 1) + "%";
    pressure = String((pres/kpaTommHg), 0) + "mm";
    jsonDoc["airtemp"] = airtemp;
    jsonDoc["humidity"] = humidity;
    jsonDoc["pressure"] = pressure;
    if (currentMillis - storedMillis >= STOREINTERVAL)
    { // update stored value once/hour
          pressurearray = updateStoredData("/temperature.txt", airtemp.toInt());
          pressurearray = updateStoredData("/humidity.txt", humidity.toInt());
          pressurearray = updateStoredData("/pressure.txt", pressure.toInt());
          pressurearray = updateStoredData("/time.txt", utcTime);
    }
    notifyClients();
    previousMillis = currentMillis;
  }
#endif
#ifdef UDPPort
  int packetSize = udp.parsePacket();
  if (packetSize) {
    processPacket(packetSize);
  }
#endif
  NMEA2000.ParseMessages();
#ifdef OTAPORT
  servOTA.handleClient();
#endif
  ws.cleanupClients();
}
#ifdef UDPPort
void processPacket(int packetSize)
{
  char packetBuffer[4096];
  udp.read(packetBuffer, packetSize);
  packetBuffer[packetSize] = '\0';
#ifdef DEBUG
  Serial.println(packetBuffer);
#endif
  uint8_t *pointer_to_int;
  pointer_to_int = (uint8_t *)packetBuffer;
  if (packetSize == 1)
  { // in case just one byte was received for whatever reason
    nmeaLine[0] = pointer_to_int[0];
    j = 1;
  }
  else
  {
    for (i = 0; i < packetSize; i++)
    {
          nmeaLine[j++] = pointer_to_int[i];
          if (pointer_to_int[i - 1] == 13 && pointer_to_int[i] == 10)
          {
          nmeaLine[j] = 0;
          noOfFields = parseNMEA0183(nmeaLine, nmeaStringData);
          // act accodring to command
          String command = nmeaStringData[0];
          if (command == "APB")
          {
            //          Serial.print("APB");    //autopilot
          }
          else if (command == "DBK")
          {
            //          Serial.print("DBK");
          }
          else if (command == "DBT")
          {
            stringBD.WaterDepth = nmeaStringData[3];
            depth = stringBD.WaterDepth;
            jsonDoc["depth"] = depth;
            notifyClients();
          }
          else if (command == "DPT")
          {
            /*
                      stringBD.WaterDepth = nmeaStringData[1];
                      depth = stringBD.WaterDepth;
                      jsonDoc["depth"] = depth;
                      notifyClients();
            */
          }
          else if (command == "GGA")
          {
            stringBD.UTC = nmeaStringData[1];
            int hours = stringBD.UTC.substring(0, 2).toInt();
            int minutes = stringBD.UTC.substring(2, 4).toInt();
            int seconds = stringBD.UTC.substring(4, 6).toInt();
            BoatData.GPSTime = stringBD.UTC.toDouble();
            stringBD.UTC = int2string(hours) + ":" + int2string(minutes) + ":" + int2string(seconds);
            timedate = stringBD.UTC;
            jsonDoc["timedate"] = timedate;
            //          Serial.printf("GGA %s", timedate);
            stringBD.Latitude = convertGPString(nmeaStringData[2]) + nmeaStringData[3];
            latitude = stringBD.Latitude;
            jsonDoc["latitude"] = latitude;
            stringBD.Longitude = convertGPString(nmeaStringData[4]) + nmeaStringData[5];
            longitude = stringBD.Longitude;
            jsonDoc["longitude"] = longitude;
            notifyClients();
          }
          else if (command == "GLL")
          {
            stringBD.Latitude = convertGPString(nmeaStringData[1]) + nmeaStringData[2];
            latitude = stringBD.Latitude;
            jsonDoc["latitude"] = latitude;
            stringBD.Longitude = convertGPString(nmeaStringData[3]) + nmeaStringData[4];
            longitude = stringBD.Longitude;
            jsonDoc["longitude"] = longitude;
            stringBD.UTC = nmeaStringData[5];
            int hours = stringBD.UTC.substring(0, 2).toInt();
            int minutes = stringBD.UTC.substring(2, 4).toInt();
            int seconds = stringBD.UTC.substring(4, 6).toInt();
            stringBD.UTC = int2string(hours) + ":" + int2string(minutes) + ":" + int2string(seconds);
            timedate = stringBD.UTC;
            jsonDoc["timedate"] = timedate;
            notifyClients();
            //          Serial.printf("GLL %s", timedate);
          }
          else if (command == "GSA")
          { // GPS Sat
            //
          }
          else if (command == "HDG")
          {
            stringBD.HeadingM = String(int(nmeaStringData[1].toDouble()));
            heading = stringBD.HeadingM + "°";
            jsonDoc["heading"] = heading;
            notifyClients();
          }
          else if (command == "HDM")
          {
            stringBD.HeadingM = String(int(nmeaStringData[1].toDouble()));
            heading = stringBD.HeadingM + "°";
            jsonDoc["heading"] = heading;
            notifyClients();
          }
          else if (command == "HDT")
          {
            stringBD.HeadingT = String(int(nmeaStringData[1].toDouble()));
            heading = stringBD.HeadingT + "°";
            jsonDoc["heading"] = heading;
            notifyClients();
          }
          else if (command == "MTW")
          {
            stringBD.WaterTemperature = nmeaStringData[1];
            watertemp = stringBD.WaterTemperature + "°C";
            jsonDoc["watertemp"] = watertemp;
            notifyClients();
          }
          else if (command == "MWD")
          {
            stringBD.WindDirectionT = String(int(nmeaStringData[1].toDouble()));
            winddir = stringBD.WindDirectionT + "°true";
            jsonDoc["winddir"] = winddir;
            stringBD.WindDirectionM = String(int(nmeaStringData[3].toDouble()));
            //          winddir  = stringBD.WindDirectionM + "m";
            //          jsonDoc["winddir"] = winddir;
            stringBD.WindSpeedK = String(int(nmeaStringData[5].toDouble()));
            windspeed = stringBD.WindSpeedK;
            jsonDoc["windspeed"] = windspeed;
            notifyClients();
          }
          else if (command == "MWV")
          { // wind speed and angle
            stringBD.WindDirectionT = String(int(nmeaStringData[1].toDouble()));
            winddir = stringBD.WindDirectionT + "°app";
            jsonDoc["winddir"] = winddir;
            stringBD.WindSpeedK = nmeaStringData[3];
            windspeed = stringBD.WindSpeedK; // + nmeaStringData[4];
            jsonDoc["windspeed"] = windspeed + " app";
            notifyClients();
          }
          else if (command == "RMB")
          { // nav info
            //          Serial.print("RMB");        //waypoint info
          }
          else if (command == "RMC")
          { // nav info
            stringBD.SOG = nmeaStringData[7];
            speed = stringBD.SOG;
            jsonDoc["speed"] = speed;
            stringBD.Latitude = convertGPString(nmeaStringData[3]) + nmeaStringData[4];
            latitude = stringBD.Latitude;
            jsonDoc["latitude"] = latitude;
            stringBD.Longitude = convertGPString(nmeaStringData[5]) + nmeaStringData[6];
            longitude = stringBD.Longitude;
            jsonDoc["longitude"] = longitude;
            stringBD.UTC = nmeaStringData[1];
            int hours = stringBD.UTC.substring(0, 2).toInt();
            int minutes = stringBD.UTC.substring(2, 4).toInt();
            int seconds = stringBD.UTC.substring(4, 6).toInt();
            stringBD.UTC = int2string(hours) + ":" + int2string(minutes) + ":" + int2string(seconds);
            stringBD.Date = "20" + nmeaStringData[9].substring(4, 6) + "-" + nmeaStringData[9].substring(2, 4) + "-" + nmeaStringData[9].substring(0, 2);
            timedate = stringBD.Date + " " + stringBD.UTC;
            jsonDoc["timedate"] = timedate;
            notifyClients();
            //          Serial.printf("RMC %s", timedate);
          }
          else if (command == "RPM")
          { // engine RPM
            if (nmeaStringData[2] == "1")
            { // engine no.1
              stringBD.RPM = String(int(nmeaStringData[3].toDouble()));
              rpm = stringBD.RPM;
              jsonDoc["rpm"] = rpm;
              notifyClients();
            }
          }
          else if (command == "VBW")
          { // dual ground/water speed longitudal/transverse
            //
          }
          else if (command == "VDO")
          {
            //
          }
          else if (command == "VDM")
          {
            //
          }
          else if (command == "APB")
          {
            //
          }
          else if (command == "VHW")
          { // speed and Heading over water
            stringBD.HeadingT = String(int(nmeaStringData[1].toDouble()));
            heading = stringBD.HeadingT + "t";
            jsonDoc["heading"] = heading;
            stringBD.HeadingM = String(int(nmeaStringData[3].toDouble()));
            heading = stringBD.HeadingM + "m";
            jsonDoc["heading"] = heading;
            stringBD.Speed = nmeaStringData[5];
            speed = stringBD.Speed;
            jsonDoc["speed"] = speed;
            notifyClients();
            //
          }
          else if (command == "VTG")
          { // Track Made Good and Ground Speed
            //
          }
          else if (command == "ZDA")
          { // Date&Time
            stringBD.Date = nmeaStringData[4] + "-" + nmeaStringData[3] + "-" + nmeaStringData[2];
            stringBD.UTC = nmeaStringData[1];
            int hours = stringBD.UTC.substring(0, 2).toInt();
            int minutes = stringBD.UTC.substring(2, 4).toInt();
            int seconds = stringBD.UTC.substring(4, 6).toInt();
            stringBD.UTC = int2string(hours) + ":" + int2string(minutes) + ":" + int2string(seconds);
            timedate = stringBD.Date + " " + stringBD.UTC;
            jsonDoc["timedate"] = timedate;
            notifyClients();
            //          Serial.printf("ZDA %s", timedate);
          }
          else
          {
            Serial.println("unsupported NMEA0183 sentence");
          }
          jsonDoc.clear();
          j = 0;
          }
    }
  }
}
#endif
