//Use NodeMCU-32S for Nixie Hardware
//Use ESP32 DEV MODULE for esp32 dev board 

//Taskhandlers to handle the tasks for each core
TaskHandle_t Task0; //Runs on core 0
TaskHandle_t Task1; //Runs on core 1

//Headers
//#include <Wire.h> //I2C lib
#include <pcf2129rtc.h> //RTC lib
#include "NTC_PCA9698.h" //Port Expander lib
#include <getNixieExpanderPin.h> //Port Expander to Nixie Digit mapping lib
#include "BluetoothSerial.h" //Bluetooth lib
#include <WS2812FX.h> //RGB LED lib

//Pin Definitions 
#define bootControl     0 //ref to ds - not req atm                                 (NOT USED IN THIS CODE)
#define usbSerialTx     1 //Used for code upload from programmer                    (NOT USED IN THIS CODE)
#define rtcInt          2 //RTC Interrupt 
#define usbSerialRx     3 //Used for code upload from programmer                    (NOT USED IN THIS CODE)
#define en170V          4 //170V enable - Pull to low to enable               
#define errata1nc       5 //NC                                                      (NOT USED IN THIS CODE)
#define en5V           16 //5V enable - Pull to high to enable
#define supervisor5V   17 //5V rail blackout indicator - If low, 5V blackout!!!
#define twimIntSCL     18 //I2C SCK 
#define twimIntSDA     19 //I2C SDA
#define ledBus         21 //WS2812 RGBLED Control Bus
#define sysLed         22 //System LED - UNUSED FOR NOW 
#define opsLed         23 //Operation LED - Blinking if RTC interrupt is triggered and Nixie Tubes are updated
#define devLed         25 //Dev LED - UNUSED FOR NOW
#define pwrLed         26 //Power LED for power system (5V & 170V) status
#define comLed         27 //Fast Blinking if not connected to app... Continuous if connected to app

//Defining cross-function vars
int rtcHour;
int rtcMinute;    
int rtcSecond;

//Init Nixie tube state vars (init with impossible values to avoid start up issues - remove this after testing)
//Nixie digit to be written to
int writeTube1pin = 1000;
int writeTube2pin = 1000;
int writeTube3pin = 1000;
int writeTube4pin = 1000;
int writeTube5pin = 1000;
int writeTube6pin = 1000;

//Nixie digit that is currently on
int currentTube1pin = 2000;
int currentTube2pin = 2000;
int currentTube3pin = 2000;
int currentTube4pin = 2000;
int currentTube5pin = 2000;
int currentTube6pin = 2000;

//Creating an instance of the PCF2129 RTC Lib
pcf2129rtc pcf2129rtcInstance(twimIntSDA, twimIntSCL); //(SDA,SCL)

//Seconds Interrupt Flag
bool secIntFlag = false; 
//RTC Seconds Interrupt ISR
void IRAM_ATTR rtcIntISR() {
  secIntFlag = true;
}

//Declare BT object
BluetoothSerial espBt;

//Unique code that'll be known only to user or will be labelled on the hardware itself (security)
String uniqueCode = "UNIQUE_CODE";

//Create 2 instances of the port expander lib to control 2 port expanders via I2C
PCA9698 expanderChip0(0x20,twimIntSDA,twimIntSCL,1000000); //(I2C_ADDR,SDA,SCL,SPEED)
PCA9698 expanderChip1(0x21,twimIntSDA,twimIntSCL,1000000); 

//Create an instance of the port expander to nixie tube digit pin mapping lib 
//***This lib is specific to the Nixie hardware used in this project***
getNixieExpanderPin getNixieExpanderPinInstance; 

//Create an instance of the RGB LED lib
WS2812FX ws2812fx = WS2812FX(6, ledBus, NEO_GRB + NEO_KHZ800); //(LED_COUNT,LED_PIN,NEO_GRB + NEO_KHZ800)

void setup() {
  Serial.begin(115200);

  //Core 0 Config
  xTaskCreatePinnedToCore(
    codeForTask0, //Task Function
    "Task 0",     //Name of Task
    1000,         //Stack size of task
    NULL,         //Parameter of the task
    1,            //Priority of the task
    &Task0,       //Task handle to keep track of the created task
    0);           //Target Core

  //Core 1 Config
  xTaskCreatePinnedToCore(
    codeForTask1, //Task Function
    "Task 1",     //Name of Task
    1000,         //Stack size of task
    NULL,         //Parameter of the task
    1,            //Priority of the task
    &Task1,       //Task handle to keep track of the created task
    1);           //Target Core

  //IO Definitions
  pinMode(rtcInt,INPUT_PULLUP); //Interrupt pin has to be set as input pullup
  pinMode(en170V,OUTPUT);
  pinMode(en5V,OUTPUT);
  pinMode(supervisor5V,INPUT);
  pinMode(ledBus,OUTPUT);
  pinMode(sysLed,OUTPUT);
  pinMode(opsLed,OUTPUT);
  pinMode(devLed,OUTPUT);
  pinMode(pwrLed,OUTPUT);
  pinMode(comLed,OUTPUT);

  //Turn on the 5V and 170V supplies (3.3V is auto turned on cuz ESP32 run off it)
  enablePowerSupplies(); 

  //Name of BT signal
  espBt.begin("ESP32_BT_Control"); //This will be the name shown to other BT devices in the BT network

  //RTC Initial Config
  pcf2129rtcInstance.rtcInitialConfig(); 

  //Update RTC with current time
  //Manually write the default hour, min and sec to RTC 
  //This will be the default start up time when the clock is reset
  int defaultHour = 0;
  int defaultMin = 0;
  int defaultSec = 0;
  pcf2129rtcInstance.updateCurrentTimeToRTC(defaultHour, defaultMin, defaultSec); //(HOUR,MIN,SEC)

  //Interrupt Definitions
  //attachInterrupt(digitalPinToInterrupt(PIN_NUM), ISR, mode)
  //Mode: LOW/CHANGE/RISING/FALLING/*HIGH(Only for Due,Zero,MKR1000 boards)* 
  attachInterrupt(digitalPinToInterrupt(supervisor5V), pwrSysFailISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rtcInt),rtcIntISR, FALLING);

  //Port expander chips config and port io mode setting
  expanderChip0.configuration();
  expanderChip0.portMode(0,OUTPUT); //(PORT_NUM,INPUT/OUTPUT)
  expanderChip0.portMode(1,OUTPUT);
  expanderChip0.portMode(2,OUTPUT);
  expanderChip0.portMode(3,OUTPUT);
  expanderChip0.portMode(4,OUTPUT);
  
  expanderChip1.configuration();
  expanderChip1.portMode(0,OUTPUT);
  expanderChip1.portMode(1,OUTPUT);
  expanderChip1.portMode(2,OUTPUT);
  expanderChip1.portMode(3,OUTPUT);
  expanderChip1.portMode(4,OUTPUT);

  //RGB LED Config
  ws2812fx.init();
  ws2812fx.setBrightness(50);
  ws2812fx.setSpeed(5);
  ws2812fx.setMode(FX_MODE_RAINBOW_CYCLE);
  ws2812fx.start();
}

void loop() {
  //Loop isn't used as all the code has been separated into tasks and run explicitly on core 1 and 2
}

//Function Definitions
//--------------------
void enablePowerSupplies() {
  digitalWrite(en5V, HIGH);
  digitalWrite(en170V, LOW); 
}

//Run this function until it returns true
//Return true = security verification successful
//Return false = security verification unsuccessful/timeout
bool VerifyBtConnection() {
  int state = 0; //Set the starting state of the state machine
  for(int i=0; i<3; i++) {
    switch(state) {
      case 0:
      //Hang here till a remote device connects...
      Serial.println("Waiting for connection from remote device...");
      while(!espBt.hasClient()) {
        digitalWrite(comLed,HIGH);
        delay(50);
        digitalWrite(comLed,LOW);
        delay(50);
      }

      espBt.print("REQ_CHECK_1");

      //Hang here till level 1 verification key is received... (auto sent by hardware)
      Serial.println("Waiting for Level 1 verification key from remote device...");
      while(!espBt.available()) {
        digitalWrite(comLed,HIGH);
        delay(50);
        digitalWrite(comLed,LOW);
        delay(50);
      }
      
      if(espBt.readString() == "REQ_CONN") {
        Serial.println("Level 1 Pass");
        espBt.print("REQ_CHECK_2"); //Tell hardware that level 1 verification is successful and send level 2 verification
        state = 1; 
      }
      else {
        Serial.println("Level 1 Fail");
        state = 0;
      }
      break;
      
      case 1:
      //Hang here till Level 2 verification key is received... (keyed in by user)
      Serial.println("Waiting for Level 2 verification key from remote device");
      while(!espBt.available()) {
        digitalWrite(comLed,HIGH);
        delay(50);
        digitalWrite(comLed,LOW);
        delay(50);
      }

      if(espBt.readString() == uniqueCode) {
        Serial.println("Level 2 Pass");
        espBt.print("CONN_PASS"); //Tell the hardware that level 2 verification is successful 
        state = 2;
      }
      else {
        Serial.println("Level 2 Fail");
        espBt.print("CONN_FAIL"); //Tell the hardware that level 2 verification is unsuccessful
        state = 0;
      }
      break;
  
      case 2:
      return true;
      break;
    }
  }
}

//THIS FUNCTION IS UNTESTED
//ISR triggered when there is a blackout on the 5V rail
IRAM_ATTR void pwrSysFailISR() {
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
//This function is called by the pwrSysFailISR() which is triggered when there is a blackout on the 5V rail
//This function will switch off the 5V and 170V power supplies and turn off the RGB LEDs... 
//This puts the hardware into safe mode
void disableSubsystems() {
  //Disable power sub systems
  digitalWrite(en170V, HIGH);
  digitalWrite(en5V, LOW);

  //Disable RGB LEDs
  ws2812fx.stop();
}

void offNixieTube1() {
  for(int i=0;i<10;i++) {
    expanderChip0.digitalWrite(getNixieExpanderPinInstance.getPinNumber(1,i),LOW);
  }
}
void offNixieTube2() {
  for(int i=0;i<10;i++) {
    expanderChip0.digitalWrite(getNixieExpanderPinInstance.getPinNumber(2,i),LOW);
  }
}
void offNixieTube3() {
  for(int i=0;i<10;i++) {
    expanderChip0.digitalWrite(getNixieExpanderPinInstance.getPinNumber(3,i),LOW);
  }
}
void offNixieTube4() {
  for(int i=0;i<10;i++) {
    expanderChip1.digitalWrite(getNixieExpanderPinInstance.getPinNumber(4,i),LOW);
  }
}
void offNixieTube5() {
  for(int i=0;i<10;i++) {
    expanderChip1.digitalWrite(getNixieExpanderPinInstance.getPinNumber(5,i),LOW);
  }
}
void offNixieTube6() {
  for(int i=0;i<10;i++) {
    expanderChip1.digitalWrite(getNixieExpanderPinInstance.getPinNumber(6,i),LOW);
  }
}

//Main Code for Core 0 and 1 

/* Core 0 
 *  Comms with Project Nixie android app
 *  Send data received from the app to core 1 so that the Nixies can be updated
 */
void codeForTask0(void * parameter) {
  while(1) {
    //Until hardware verification is successful... hang at this line...
    while(VerifyBtConnection()!=true){};  
    Serial.println("Hardware verification successful");
    while(1) {
      digitalWrite(comLed,HIGH);
      for(int i=0;i<100000;i++){}
      delay(5);
    }
  }
}

/* Core 1
 *  Drive Nixies when seconds interrupt is triggered by RTC
 *  Drive RGB LEDs
 *  Nixie Tube Cathode Protection (TBD)
 */
void codeForTask1(void * parameter) {
  while(1) {
    if(secIntFlag == true) {
      //Get the pin numbers of the nixie digits to turn on for each tube
      //getNixieExpanderPinInstance.getPinNumber(TUBE_NUM,BCD_DIGIT)
      writeTube1pin = getNixieExpanderPinInstance.getPinNumber(1,pcf2129rtcInstance.readRtcHourBCD1());
      writeTube2pin = getNixieExpanderPinInstance.getPinNumber(2,pcf2129rtcInstance.readRtcHourBCD0());
      writeTube3pin = getNixieExpanderPinInstance.getPinNumber(3,pcf2129rtcInstance.readRtcMinBCD1());
      writeTube4pin = getNixieExpanderPinInstance.getPinNumber(4,pcf2129rtcInstance.readRtcMinBCD0());
      writeTube5pin = getNixieExpanderPinInstance.getPinNumber(5,pcf2129rtcInstance.readRtcSecBCD1());
      writeTube6pin = getNixieExpanderPinInstance.getPinNumber(6,pcf2129rtcInstance.readRtcSecBCD0());
  
      //If the tube has to be updated...
      if(writeTube1pin != currentTube1pin) {
        offNixieTube1(); //Turn off all digits on that tube
        expanderChip0.digitalWrite(writeTube1pin,HIGH); //Turn on the updated digit
        //Turning off the tube and then updating the tube is done to prevent double digit display on the tube
      }
      if(writeTube2pin != currentTube2pin) {
        offNixieTube2();
        expanderChip0.digitalWrite(writeTube2pin,HIGH);
      }
      if(writeTube3pin != currentTube3pin) {
        offNixieTube3();
        expanderChip0.digitalWrite(writeTube3pin,HIGH); 
      }
      if(writeTube4pin != currentTube4pin) {
        offNixieTube4();
        expanderChip1.digitalWrite(writeTube4pin,HIGH);  
      }
      if(writeTube5pin != currentTube5pin) {
        offNixieTube5();
        expanderChip1.digitalWrite(writeTube5pin,HIGH);
      }
      if(writeTube6pin != currentTube6pin) {
        offNixieTube6();
        expanderChip1.digitalWrite(writeTube6pin,HIGH);
      }
  
      //Update the tube state (digit currently displayed in each tube)
      currentTube1pin = writeTube1pin;
      currentTube2pin = writeTube2pin;
      currentTube3pin = writeTube3pin;
      currentTube4pin = writeTube4pin;
      currentTube5pin = writeTube5pin;
      currentTube6pin = writeTube6pin;
  
      //Reset interrupt secIntFlag
      secIntFlag = false;
      pcf2129rtcInstance.clearMsf();
      
      //Flash OpsLed to indicate rtc seconds interrupt successful triggering
      digitalWrite(opsLed,HIGH);
      delay(50);
      digitalWrite(opsLed,LOW);
    }
    //Code inside the else loop will execute if the core is not busy driving the nixies
    //Priority is given to driving the nixies!!!
    else {
      //Drive the RGB LEDs 
      ws2812fx.service();
    }
  }
}
