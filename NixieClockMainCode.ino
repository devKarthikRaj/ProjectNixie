//Headers
#include <Wire.h> //I2C
#include <WS2812FX.h> //WS2812

//Pin Definitions
#define bootControl     0 //ref to ds - not req atm
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
#define sysLed         22 //System LED - UNUSED FOR NOW 
#define oprLed         23 //Operation LED for RTC status
#define devLed         25 //Dev LED - UNUSED FOR NOW
#define pwrLed         26 //Power LED for power system (5V & 170V) status
#define comLed         27 //Comms LED for WiFi connection

//Define cross-function variables
int rtcHour;
int rtcMinute;
int rtcSecond;

//Define WS2812 RGB LED lib instance 
WS2812FX ws2812fx = WS2812FX(6, ledBus, NEO_GRB + NEO_KHZ800);

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

  enablePowerSupplies(); //Turn on the 5V and 170V supplies

  //Initialize I2C - initiate wire library and join I2C bus as master
  Wire.begin(twimIntSDA, twimIntSCL);

  //Configure RTC control registers 1,2 & 3 
  rtcInitialConfig();

  //Update RTC with current time
  updateCurrentTimeToRTC();

  //Initiate WS2812 RGB LEDs
  initiateWS2812();

  //Interrupt Definitions
  attachInterrupt(digitalPinToInterrupt(supervisor5V), statusLedsController, CHANGE); 
}

void loop() {
  //Obtain current time from RTC
  readCurrentTimeFromRTC();

  //Display current time on nixies

  //WS2812 RGB LED instance
  ws2812fx.service();
}

// THIS FUNCTION IS YET TO BE TESTED
void enablePowerSupplies() {
  digitalWrite(en5V, HIGH);
  digitalWrite(en170V, LOW);
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

void updateCurrentTimeToRTC() {
  //Update seconds register in RTC
  Wire.beginTransmission(0x51);
  Wire.write(0x03);
  Wire.write(); //Write seconds in the parenthesis in BCD
  Wire.endTransmission();

  //Update minutes register in RTC
  Wire.beginTransmission(0x51);
  Wire.write(0x04);
  Wire.write(); //Write minutes in the parenthesis in BCD
  Wire.endTransmission();

  //Update hours register in RTC
  Wire.beginTransmission(0x51);
  Wire.write(0x05);
  Wire.write(); //Write hours in the parenthesis in BCD
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

  /*
  //Display RTC time on serial monitor (for debugging only)
  Serial.println();
  Serial.print("RTC: ");
  Serial.print(rtcHour);
  Serial.print(":");
  Serial.print(rtcMinute);
  Serial.print(":");
  Serial.print(rtcSecond);
  */
  
  if(rtcHour && rtcMinute && rtcSecond != 0) {
    digitalWrite(oprLed,HIGH);
    //Serial.println("opr high"); // For Debugging Only
  }
  else {
    digitalWrite(oprLed,LOW);
    //Serial.println("opr low"); // For Debugging Only
  }
}

//THIS FUNCTION IS UNTESTED
ICACHE_RAM_ATTR void statusLedsController() {
  //Check sub system statuses and set LEDs in void setup
  //Use interrupt to toggle leds if subsystems fail!!!
  
  //Power Sub-System LED - LED on if 5V supply is present
  if(digitalRead(supervisor5V) == HIGH) {
    digitalWrite(pwrLed, HIGH);
  }
}

//THIS FUNCTION IS UNTESTED
void initiateWS2812() {
  //Initiate WS2812 RGB LEDs
  ws2812fx.init();
  ws2812fx.setBrightness(100);
  ws2812fx.setSpeed(200);
  ws2812fx.setMode(FX_MODE_RAINBOW_CYCLE);
  ws2812fx.start();
}

//THIS FUNCTION IS INCOMPLETE AND UNTESTED
void disableSubsystems() {
  //This fucntion will disable all subsystems when there's an interrupt indicating a sub system fault

  //Disable power sub systems
  digitalWrite(en170V, HIGH);
  digitalWrite(en5V, LOW);

  //Disable RGB LEDs
}

//THIS FUNCTION IS INCOMPLETE AND UNTESTED
void tempMonitor() {s temperature hourly and turns 
  //Monitoroff nixies and RGB LEDs if temp too high   
}
