/* 
by Dr.András Szép under GNU General Public License (GPL).
*/

int parseNMEA0183( String sentence, String data[]);
//void initFS();
void storeString(String path, String content);
String retrieveString(String path);
String GPStoString( double coordinate);
String convertGPString(String input) ;
String int2string(int number);
String readStoredData( const char* filename);
String updateStoredData(const char* filename, int newValue);
String getDT();
String secondsToTimeString(int seconds);
String convertDaysToDate(uint16_t daysSince1970);
void initWiFi();
