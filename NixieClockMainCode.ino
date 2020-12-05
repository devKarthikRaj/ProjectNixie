//Headers
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

// Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

void setup() {
  Serial.begin(115200);
  
  connectToWiFiNetwork();
  initializeNTP();
}

void loop() {
  fetchNTPTime();
}

void connectToWiFiNetwork() {
  #ifndef STASSID
  #define STASSID "wifi_name" //STASSID = Station SSID
  #define STAPSK "wifi_password" //STA PSK = Station Passkey
  #endif
  
  const char* ssid = STASSID;
  const char* password = STAPSK;

  //Connceting to specified Wifi network
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  //Setting the ESP8266 to act as a WiFi client
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting...");
  }

  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void initializeNTP() {
  // Initialize a NTPClient to get time
  timeClient.begin();
  //timeClient.setTimeOffset();
}

void fetchNTPTime() {
  //Use timer interrupt to run this function once every hour
  timeClient.update();

  unsigned long epochTime = timeClient.getEpochTime();
  String formattedTime = timeClient.getFormattedTime(); 
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();
  int currentSecond = timeClient.getSeconds();

  //Serial print for debugging only
  if(currentHour<10){Serial.print(0);}
  Serial.print(currentHour-4);  
  Serial.print(":");
  if(currentMinute<10){Serial.print(0);}
  Serial.print(currentMinute); 
  Serial.print(":");
  if(currentSecond<10){Serial.print(0);} 
  Serial.print(currentSecond);  
  Serial.println("");
  delay(1000); //Remove the delay later... use interrupt to fetch the time every hour from ntp
}
