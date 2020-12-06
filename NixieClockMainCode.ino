//Headers
#include <WiFi.h> //<Wifi.h> for esp32 and <ESP8266WiFi.h> for esp8266 
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <Wire.h>

//Pin Definitions
#define bootControl     0 //ref to ds
#define usbSerialTx     1 //Used for code upload from programmer
#define rtcInt          2 //Unused for now
#define usbSerialRx     3 //Used for code upload from programmer
#define en170V          4 //170V enable - Pull to low to enable
#define errata1nc       5 //NC
#define en5V           16 //5V enable - Pull to high to enable
#define supervisor5V   17 //5V rail blackout indicator - If low, 5V blackout!!!
#define twimIntSCL     18 //I2C SCK 
#define twimIntSDA     19 //I2C SDA
#define ledBus         21 //WS2812 RGBLED Control Bus
#define sysLed         22 //System LED
#define oprLed         23 //Operation LED
#define devLed         25 //Dev LED
#define pwrLed         26 //Power LED
#define comLed         27 //Comms LED

// Define NTP Client to connect to time server
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

//Define cross-function variables
int currentHour;  
int currentMinute; 
int currentSecond;
int currentHourBCD;
int currentMinuteBCD;
int currentSecondBCD;
int rtcHour;
int rtcMinute;
int rtcSecond;

void setup() {
  Serial.begin(115200);

  //Input/Output Definitions
  pinMode(en170V,OUTPUT);
  pinMode(en5V,OUTPUT);
  pinMode(supervisor5V,INPUT);
  pinMode(ledBus,OUTPUT);
  pinMode(sysLed,OUTPUT);
  pinMode(oprLed,OUTPUT);
  pinMode(devLed,OUTPUT);
  pinMode(pwrLed,OUTPUT);
  pinMode(comLed,OUTPUT);
  
  connectToWiFiNetwork();
  initializeNTP();

  //Initialize I2C - initiate wire library and join I2C bus as master
  Wire.begin(twimIntSDA, twimIntSCL);

  //Configure RTC control registers 1,2 & 3 
  rtcInitialConfig();

  //Fetch current time from online server
  fetchNTPTime();

  //Format server time to send to RTC
  currentHourBCD = formatCurrentHourToBCD();
  currentMinuteBCD = formatCurrentMinuteToBCD();
  currentSecondBCD = formatCurrentSecondToBCD();

  //Update RTC with current time
  updateCurrentTimeToRTC();
}

void loop() {
  //Obtain current time from RTC
  readCurrentTimeFromRTC();

  //Display current time on nixies
}

void connectToWiFiNetwork() {
  #ifndef STASSID
  #define STASSID "dlink-193C" //STASSID = Station SSID
  #define STAPSK "mczep88481" //STA PSK = Station Passkey
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
  currentHour = (timeClient.getHours()+8);
  currentMinute = timeClient.getMinutes();
  currentSecond = timeClient.getSeconds();
}

void rtcInitialConfig() {
  //RTC initial config 
  //Control reg 1 config
  Wire.beginTransmission(0x51); //Start + write slave address
  Wire.write(0x00); //Write reg address
  Wire.write(0x00); //Write data
  Wire.endTransmission(); 
  
  //Control reg 2 config
  Wire.beginTransmission(0xA2); 
  Wire.write(0x01); 
  Wire.write(0x00); 
  Wire.endTransmission();

  //Control reg 3 config
  Wire.beginTransmission(0xA2);
  Wire.write(0x02); 
  Wire.write(0xE0); 
  Wire.endTransmission();
}

int formatCurrentSecondToBCD() {
  int currentSecondBCD1 = currentSecond/10;
  int currentSecondBCD2 = currentSecond%10;
  int currentSecondBCD12;

  bitWrite(currentSecondBCD12, 7, 0);
  bitWrite(currentSecondBCD12, 6, bitRead(currentSecondBCD1,2));
  bitWrite(currentSecondBCD12, 5, bitRead(currentSecondBCD1,1));
  bitWrite(currentSecondBCD12, 4, bitRead(currentSecondBCD1,0));
  bitWrite(currentSecondBCD12, 3, bitRead(currentSecondBCD2,3));
  bitWrite(currentSecondBCD12, 2, bitRead(currentSecondBCD2,2));
  bitWrite(currentSecondBCD12, 1, bitRead(currentSecondBCD2,1));
  bitWrite(currentSecondBCD12, 0, bitRead(currentSecondBCD2,0));

  return currentSecondBCD12;
}

int formatCurrentMinuteToBCD() {
  int currentMinuteBCD1 = currentMinute/10;
  int currentMinuteBCD2 = currentMinute%10;
  int currentMinuteBCD12;
  
  bitWrite(currentMinuteBCD12, 7, 0);
  bitWrite(currentMinuteBCD12, 6, bitRead(currentMinuteBCD1,2));
  bitWrite(currentMinuteBCD12, 5, bitRead(currentMinuteBCD1,1));
  bitWrite(currentMinuteBCD12, 4, bitRead(currentMinuteBCD1,0));
  bitWrite(currentMinuteBCD12, 3, bitRead(currentMinuteBCD2,3));
  bitWrite(currentMinuteBCD12, 2, bitRead(currentMinuteBCD2,2));
  bitWrite(currentMinuteBCD12, 1, bitRead(currentMinuteBCD2,1));
  bitWrite(currentMinuteBCD12, 0, bitRead(currentMinuteBCD2,0));

  return currentMinuteBCD12;
}

int formatCurrentHourToBCD() {
  int currentHourBCD1 = currentHour/10;
  int currentHourBCD2 = currentHour%10;
  int currentHourBCD12;
  
  bitWrite(currentHourBCD12, 7, 0);
  bitWrite(currentHourBCD12, 6, bitRead(currentHourBCD1,2));
  bitWrite(currentHourBCD12, 5, bitRead(currentHourBCD1,1));
  bitWrite(currentHourBCD12, 4, bitRead(currentHourBCD1,0));
  bitWrite(currentHourBCD12, 3, bitRead(currentHourBCD2,3));
  bitWrite(currentHourBCD12, 2, bitRead(currentHourBCD2,2));
  bitWrite(currentHourBCD12, 1, bitRead(currentHourBCD2,1));
  bitWrite(currentHourBCD12, 0, bitRead(currentHourBCD2,0));

  return currentHourBCD12;
}

void updateCurrentTimeToRTC() {
  //Update seconds register in RTC
  Wire.beginTransmission(0x51);
  Wire.write(0x03);
  Wire.write(currentSecondBCD);
  Wire.endTransmission();

  //Update minutes register in RTC
  Wire.beginTransmission(0x51);
  Wire.write(0x04);
  Wire.write(currentMinuteBCD);
  Wire.endTransmission();

  //Update hours register in RTC
  Wire.beginTransmission(0x51);
  Wire.write(0x05);
  Wire.write(currentHourBCD);
  Wire.endTransmission();
}

void readCurrentTimeFromRTC() {
  //Read hour from RTC
  Wire.beginTransmission(0x51);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.requestFrom(0x51,1);
  if(Wire.available()) {
    rtcHour = (Wire.read());
  }

  //Read minute from RTC
  Wire.beginTransmission(0x51);
  Wire.write(0x04);
  Wire.endTransmission();
  Wire.requestFrom(0x51,1);
  if(Wire.available()) {
    rtcMinute = (Wire.read());
  }

  //Read second from RTC
  Wire.beginTransmission(0x51);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(0x51,1);
  if(Wire.available()) {
    rtcSecond = (Wire.read());
  }

  //Display RTC time on serial monitor (for debugging only)
  Serial.println();
  Serial.print("RTC: ");
  Serial.print(rtcHour);
  Serial.print(":");
  Serial.print(rtcMinute);
  Serial.print(":");
  Serial.print(rtcSecond);
}

//up next
//led driver function --- to check sub system statuses and set leds in void setup
//use interrupt to toggle leds if subsystems fail!!!
