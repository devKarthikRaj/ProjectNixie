//Use NodeMCU-32S for Nixie Hardware
//Use ESP32 DEV MODULE for esp32 dev board 

//Headers
//#include <Wire.h> //I2C lib
#include <pcf2129rtc.h> //RTC lib

//Pin Definitions 
#define bootControl     0 //ref to ds - not req atm                                 (NOT USED IN THIS CODE)
#define usbSerialTx     1 //Used for code upload from programmer                    (NOT USED IN THIS CODE)
#define rtcInt          2 //Unused for now                                          (NOT USED IN THIS CODE)
#define usbSerialRx     3 //Used for code upload from programmer                    (NOT USED IN THIS CODE)
#define en170V          4 //170V enable - Pull to low to enable               
#define errata1nc       5 //NC                                                      (NOT USED IN THIS CODE)
#define en5V           16 //5V enable - Pull to high to enable
#define supervisor5V   17 //5V rail blackout indicator - If low, 5V blackout!!!
#define twimIntSCL     18 //I2C SCK 
#define twimIntSDA     19 //I2C SDA
#define ledBus         21 //WS2812 RGBLED Control Bus
#define sysLed         22 //System LED - UNUSED FOR NOW 
#define oprLed         23 //Operation LED for RTC status
#define devLed         25 //Dev LED - UNUSED FOR NOW
#define pwrLed         26 //Power LED for power system (5V & 170V) status
#define comLed         27 //Comms LED for WiFi connection

//Defining Cross-Funtion Vars
int rtcHour;
int rtcMinute;
int rtcSecond;

//Creating an instance of the PCF2129 RTC Lib
//pcf2129rtc name_of_instance(sda_pin, scl_pin);
pcf2129rtc pcf2129rtcinstance(twimIntSDA, twimIntSCL);

void setup() {
  Serial.begin(115200);

  //IO Definitions
  pinMode(en170V,OUTPUT);
  pinMode(en5V,OUTPUT);
  pinMode(supervisor5V,INPUT);
  pinMode(ledBus,OUTPUT);
  pinMode(sysLed,OUTPUT);
  pinMode(oprLed,OUTPUT);
  pinMode(devLed,OUTPUT);
  pinMode(pwrLed,OUTPUT);
  pinMode(comLed,OUTPUT);

  enablePowerSupplies(); //Turn on the 5V and 170V supplies (3.3V is auto turned on cuz ESP32 run off it)

  //RTC Initial Config
  pcf2129rtcinstance.rtcInitialConfig(); 

  //Update RTC with current time
  //Manually write the hour,min,sec (for debugging only)
  int manuallyWrittenHour = 1;
  int manuallyWrittenMin = 1;
  int manuallyWrittenSec = 1;
  pcf2129rtcinstance.updateCurrentTimeToRTC(manuallyWrittenHour, manuallyWrittenMin, manuallyWrittenSec);

  //Interrupt Definitions
  //attachInterrupt(digitalPinToInterrupt(PIN_NUM), ISR, mode)
  //Mode: LOW/CHANGE/RISING/FALLING/*HIGH(Only for Due,Zero,MKR1000 boards)* 
  attachInterrupt(digitalPinToInterrupt(supervisor5V), statusLedsController, CHANGE); 
}

void loop() {
  //Obtain current time from RTC
  //readCurrentTimeFromRTC();
  int rtcHr = pcf2129rtcinstance.readRtcHour();
  int rtcMin = pcf2129rtcinstance.readRtcMin();
  int rtcSec = pcf2129rtcinstance.readRtcSec();

  //For debugging purposes only
  //Serial.println(rtcHr);
  //Serial.print(rtcMin);
  Serial.print(rtcSec);

  //Display current time on nixies
  //TBD...
}

//Function Definitions
//--------------------
void enablePowerSupplies() {
  digitalWrite(en5V, HIGH);
  digitalWrite(en170V, LOW); 
}

//THIS FUNCTION IS UNTESTED
ICACHE_RAM_ATTR void statusLedsController() {
  //Check sub system statuses and set LEDs in void setup
  //Use interrupt to toggle LEDs if subsystems fail!!!
  
  //Power Sub-System LED - LED on if 5V supply is present
  if(digitalRead(supervisor5V) == HIGH) {
    digitalWrite(pwrLed, HIGH);
    //Serial.println("5V RAIL ACTIVE");
  }
  else {
    digitalWrite(pwrLed, LOW);
    //Serial.println("5V RAIL INACTIVE!!!");
  }
}

//THIS FUNCTION IS INCOMPLETE AND UNTESTED
void disableSubsystems() {
  //This fucntion will disable all subsystems when there's an interrupt indicating a sub system fault

  //Disable power sub systems
  digitalWrite(en170V, HIGH);
  digitalWrite(en5V, LOW);

  //Disable RGB LEDs
}
