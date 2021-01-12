/*
 * To do:
 * v. set dispensing touch button when long pressed
 * v. only able to dispense when limitswitch is pressed
 * v. otherwise blink the LEDs red
 * 4. have functions to blink LEDs / set it to run etc
 * 5. set limit switch so when pressed, check the status of the bottleflag, and move up/fwd the motor if bottleflag==false till the current sensor is >= 1.5A
 * 6. if bottleflag is true, skip and don't do anything, means all is good and bottle is still in place
 * 7. if bottleflag is true, but limitswitch is notpressed, move up/fwd the motor till the current sensor is >= 1.5A
 * 8. upon short press of touch button, cycle through the LED for OA lifetime
 * 9. Set eeprom for timing of battery, OA and wine
 * 10. Battery is constantly read (not reconnected yet)
 * f. OA is reset when IR reads off , and be placed back
 * v. Wine life is reset everytime bottle limitswitch is reconnected
 * 13. adjust LED behaviours afterwards and check with the UI documents (for now just put in the comment for behaviour)
 * v. count wine volume dispensed by the button pressing length
 * 
 * 
 * TODO: reduce OA based on usage and time unused
 */


 //MAKE UP AND DOWN LEVEL TO BE DIFFERENT. THE UP WAS TOO STRONG, THE LOWER COULDNT COPE

#include "FastLED.h"
#include <Wire.h>
//#include <Adafruit_INA219.h>
#include "INA219_WE.h"
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <Update.h>

WiFiClient client;
//xz add OTA
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;

long contentLength = 0;
bool isValidContentType = false;//wifi

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;//BLE
std::string rxValue;
const char *PSWD;
const char *SSID;
const int buttonPin = 15; // Use GPIO number. See ESP32 board pinouts
int buttonState = 0;
const int ledPin = 23; // Could be different depending on the dev board.
int stringlength;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

int BLEConnect = 0;
int wifikey = 0;
int passkey = 0;
int hostkey = 0;
int binkey = 0;
// S3 Bucket Config
//String host = "bucket-name.s3.ap-south-1.amazonaws.com"; // Host => bucket-name.s3.region.amazonaws.com
int port = 80; // Non https. For HTTPS 443. As of today, HTTPS doesn't work.
//String bin = "/sketch-name.ino.bin"; // bin file name with a slash in front.
String host = "";
String host01 = "";
String host02 = "";
String host03 = "";
String bin = "";
String NAME = "";
String PASSWORD = "";

String contentType = "";
// Utility to extract header value from headers
String getHeaderValue(String header, String headerName) {
  return header.substring(strlen(headerName.c_str()));
}

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// OTA Logic 


//Adafruit_INA219 ina219;
INA219_WE ina219(0x40);

//Low Pass Filter
// keeps values for each analog pin, assuming a basic arduino that maxes 6 pins. 
//   you can save a few bytes of memory by putting this on lower-numbered pins and lowering...
#define LOWPASS_ANALOG_PIN_AMT 1
 
float   lowpass_prev_out[LOWPASS_ANALOG_PIN_AMT], 
         lowpass_cur_out[LOWPASS_ANALOG_PIN_AMT];
int        lowpass_input[LOWPASS_ANALOG_PIN_AMT];

//Lifetime statuses
int preserverLifeMax = 75; //75 days or 5bottles or 25 full-glass servings. So per full-glass servings will reduce 5days worth of preserver
int wineLifeMax = 14; //14 days
int batteryLifeMax = 100; //100%
int wineVolMax = 750; //750ml

int preserverLifeAlarm = 17;
int wineLifeAlarm = 2;
int batteryLifeAlarm = 25; //%, with zero limit at 3.2V
float batteryLiveV;

int wineLifeLeft= 0; 
int preserverLifeLeft = 0;
float batteryLifeLeft;
int statusIndex = 0; //0 for none, 1 for wine, 2 for OA, 3 for battery
int prevStatus = 0;

int hourCounter = 0; //everytime it hits 24, tick down the lifetime
const unsigned long hourInMillis = 3600000;
unsigned long prevHour = 0;

unsigned long displayOnStart = 0;
int displayOnDuration = 5000; //display only for 5seconds for now

//dispensing estimated volume
//est. 12sec per full glass volume, longer when the bottle is emptier(more air to dispense),12sec full up to ~16seconds on last servings, so we will use the average of
unsigned long dispensingDuration;
int wineVolLeft = 750;
const int dispensingConvFull = 14000; ////conversion: 14000milisec dispensing = 150ml = 1 full-glass serving = 5 days worth of preserver 
const int dispensingFullGlass = 150; //150ml per full glass serving
unsigned long wineLastDispensedTime; //the last time dispensing happens 

//circulation frequency settings
unsigned long POWERFUL_CIRC_DUR = 3600000*3; //3hour = 3*3600sec = 3*3600000ms
unsigned long tCircOn;                 //Pump on duration in milliseconds
unsigned long tCircOff;                //Pump off duration in milliseconds
int amp;                               //'Power' of pump 0 - 255
unsigned long t;
unsigned long tprev;
int circPower = 100; //in PWM power

//pin definitions
int limSW = 25;
int senIR1 = 36;
int senIR2 = 39;
int battV = 34;
int lowerBtn = 26;
int pump = 17;
int valCirc = 16;
int valRelease = 27;
int platPin1 = 5;
int platPin2 = 18;
int platPinSpeed = 19;

//WS2812B LED
#define DATA_PIN     4
#define NUM_LEDS    56 // 0-2 = indicator, 3-44 = top LED, 45-55 = downlight
#define MAX_POWER_MILLIAMPS 500
#define MAX_BRIGHTNESS 100
#define LED_TYPE            WS2812B
#define COLOR_ORDER         GRB
CRGB leds[NUM_LEDS];
int firstTopLED = 3;
int midTopLED = 24;
int lastTopLED = 44;
int firstBotLED = 45;
int lastBotLED = NUM_LEDS;

unsigned long bottomLEDstartTime;
unsigned long bottomLEDtimeout = 10000; //10 seconds to turning off
bool bottomLEDon = false;

CRGB purple = CHSV( HUE_PURPLE, 255, 255);
CRGB green  = CHSV( HUE_GREEN, 255, 255);
CRGB black  = CRGB::Black;
CRGB gray = CRGB::Gray;
CRGB warmWhite = CRGB(255,160,140);
CRGB warmerWhite = CRGB(255,140,140);

//PWM channel
int pumpPWM = 0;
int platPWM = 1;
int platPWMval = 0;
bool upTillHit = false;
bool downTillHit = false;

//up&down btn flags //new addition BIGH ACTIVE LOW
bool prevUp = HIGH;
bool prevDown = HIGH;

//limit switch for bottle sensor: LS is active LOW, bottle flag is active True/HIGH when LS is pressed, so must flip
bool bottleFlag = false;
bool bottleLastFlag = false;

bool prevOAstate = true; //set to true so it won't reset on startup
bool currentOAstate = true;
bool downButtonPressed = false;
bool prevDownButtonPressed = false;
unsigned long downPressedTime  = 0;

//Button settings
//touch sensor setup
int touchAmbient = 0;
int touchThres = 1;
//long short press
const int DEBOUNCE_TIME = 50;
const int SHORT_PRESS_TIME = 200; // 100 milliseconds
//const int STARTLONGTHRESTIME = 500; //when the LED twirling is starting
const int LONG_PRESS_TIME  = 2000; // 2001 milliseconds
unsigned long touchPressedTime  = 0;
unsigned long touchReleasedTime = 0;
bool touchLastState = false;  // the previous state from the input pin
bool touchCurrentState = false;     // the current reading from the input pin

//pressure release time
int pressureReleaseTime = 2000; //milisecond

//current sensor variables
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
float readCurCap = 110.0; //stall current is 380mA ish //i burnt the capacitor by making the motor stalls too long
float stopCurUp = 100.0;
float stopCurDown = 100.0;
int countLimit = 20;
int upCount = 0;
int downCount = 0;

//for smoothing current readings
const int numReadings = 5;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int curAverage = 0;                // the average

//colours
int fastBlinkTime = 250; //250ms //unused
unsigned long prevBlinkTime = 0; //unused

//-----------------------------------------------------------------------------------------------------------------------------------------------
//--SETUP----------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------
void execOTA() {
  Serial.println("Connecting to: " + String(host));
  // Connect to S3
  if (client.connect(host.c_str(), port)) {
    // Connection Succeed.
    // Fecthing the bin
    Serial.println("Fetching Bin: " + String(bin));

    // Get the contents of the bin file
    client.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

    // Check what is being sent
    //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
    //                 "Host: " + host + "\r\n" +
    //                 "Cache-Control: no-cache\r\n" +
    //                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (client.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println("Client Timeout !");
        client.stop();
        return;
      }
    }
    // Once the response is available,
    // check stuff

    /*
       Response Structure
        HTTP/1.1 200 OK
        x-amz-id-2: NVKxnU1aIQMmpGKhSwpCBh8y2JPbak18QLIfE+OiUDOos+7UftZKjtCFqrwsGOZRN5Zee0jpTd0=
        x-amz-request-id: 2D56B47560B764EC
        Date: Wed, 14 Jun 2017 03:33:59 GMT
        Last-Modified: Fri, 02 Jun 2017 14:50:11 GMT
        ETag: "d2afebbaaebc38cd669ce36727152af9"
        Accept-Ranges: bytes
        Content-Type: application/octet-stream
        Content-Length: 357280
        Server: AmazonS3
                                   
        {{BIN FILE CONTENTS}}

    */
    while (client.available()) {
      // read line till /n
      String line = client.readStringUntil('\n');
      // remove space, to check if the line is end of headers
      line.trim();

      // if the the line is empty,
      // this is end of headers
      // break the while and feed the
      // remaining `client` to the
      // Update.writeStream();
      if (!line.length()) {
        //headers ended
        break; // and get the OTA started
      }

      // Check if the HTTP Response is 200
      // else break and Exit Update
      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") < 0) {
          Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
          break;
        }
      }

      // extract headers here
      // Start with content length
      if (line.startsWith("Content-Length: ")) {
        contentLength = atol((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("Got " + String(contentLength) + " bytes from server");
      }

      // Next, the content type
      if (line.startsWith("Content-Type: ")) {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("Got " + contentType + " payload.");
        if (contentType == "application/octet-stream") {
          isValidContentType = true;
        }
      }
    }
  } else {
    // Connect to S3 failed
    // May be try?
    // Probably a choppy network?
    Serial.println("Connection to " + String(host) + " failed. Please check your setup");
    // retry??
    // execOTA();
  }

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);

    // If yes, begin
    if (canBegin) {
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      size_t written = Update.writeStream(client);

      if (written == contentLength) {
        Serial.println("Written : " + String(written) + " successfully");
      } else {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?" );
        // retry??
        // execOTA();
      }

      if (Update.end()) {
        Serial.println("OTA done!");
        if (Update.isFinished()) {
          Serial.println("Update successfully completed. Rebooting.");
          ESP.restart();
        } else {
          Serial.println("Update not finished? Something went wrong!");
        }
      } else {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    } else {
      // not enough space to begin OTA
      // Understand the partitions and
      // space availability
      Serial.println("Not enough space to begin OTA");
      //client.flush();
    }
  } else {
    Serial.println("There was no content in the response");
    //client.flush();
  }
}


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      //std::string rxValue = pCharacteristic->getValue();
      rxValue = pCharacteristic->getValue();
      String rxString = rxValue.c_str();
      stringlength = rxValue.length();
      if (rxValue.length() > 0 && rxString[0] == 'S') {
        Serial.println("****WIFI SSID**");
        Serial.print("Received Value: ");
        NAME = rxString.substring(1);
        SSID = NAME.c_str();
        Serial.println("recived wifi name");
        Serial.println(SSID);
        rxValue = "";
      }else if (rxValue.length() > 0 && rxString[0] == 'P'){
        Serial.println("****PSWD**");
        Serial.print("Received Value: ");
        PASSWORD = rxString.substring(1);
        PSWD = PASSWORD.c_str();
        Serial.println("recived PSWD");
        Serial.println(PSWD);
        rxValue = "";
      }else if (rxValue.length() > 0 && rxString[0] == 'H'){
        Serial.println("****HOST01**");
        Serial.print("Received Value: ");
        host01 = rxString.substring(1);
        Serial.println("recived host address");
        Serial.println(host01);
        rxValue = "";
    }else if (rxValue.length() > 0 && rxString[0] == 'I'){
        Serial.println("****HOST02**");
        Serial.print("Received Value: ");
        host02 = rxString.substring(1);
        Serial.println("recived host address");
        Serial.println(host02);
        rxValue = "";
    }else if (rxValue.length() > 0 && rxString[0] == 'J'){
        Serial.println("****HOST03**");
        Serial.print("Received Value: ");
        host03 = rxString.substring(1);
        Serial.println("recived host address");
        Serial.println(host03);
        rxValue = "";
    }else if (rxValue.length() > 0 && rxString[0] == 'B'){
        Serial.println("****BIN**");
        Serial.print("Received Value: ");
        bin = rxString.substring(1);
        Serial.println("recived BIN NAME");
        Serial.println(bin);
        rxValue = "";
    }else if (rxValue.length() > 0 && rxString[0] == 'E'){
        Serial.println("**default*HOST**");
        host = "esp-dev-01.s3-ap-southeast-1.amazonaws.com";

    }else if (rxValue.length() > 0 && rxString[0] == 'X'){
        Serial.println("**no host**");
        host = "NO HOST";

    }else if (rxValue.length() > 0 && rxString[0] == 'Y'){
        Serial.println("**no bin**");
        bin = "NO BIN";

    }
    }
};

void setup()
{
  //Serial setup
  Serial.begin(115200);
  delay(1000); // give me time to bring up serial monitor
  Serial.println(F("\nSANTUS Version UDA7x No DownlightLEDs - 29 Oct 2020 \n"));

  //pinMode setup
  pinMode(limSW, INPUT_PULLUP);
  pinMode(senIR1, INPUT);
  pinMode(senIR2, INPUT);
  pinMode(battV, INPUT);
  pinMode(lowerBtn, INPUT_PULLUP);

  pinMode(pump, OUTPUT);
  pinMode(valCirc, OUTPUT);
  pinMode(valRelease, OUTPUT);
  pinMode(platPin1, OUTPUT);
  pinMode(platPin2, OUTPUT);
  pinMode(platPinSpeed, OUTPUT);

//XZ 
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);

  buttonState = digitalRead(lowerBtn);
  delay(50);
  if (buttonState == LOW){

  // Create the BLE Device
  BLEDevice::init("SANTUS");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );                      
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  while (BLEConnect == 0){
    if (deviceConnected){
      
      while (wifikey == 0){
      Serial.println("ask for wifi ssid"); 
      pTxCharacteristic->setValue("WIFI SSID");
      pTxCharacteristic->notify();
      delay(500);
      if(NAME.length() > 0){
         wifikey = 1;  //received valid SSID    
         Serial.println("SSID RECEIVED CONFIRM");   
      }else {
         wifikey = 0;
         rxValue = "";
      }  
      }
      while (passkey == 0){
      Serial.println("ask for wifi password"); 
      pTxCharacteristic->setValue("WIFI PASSWORD");
      pTxCharacteristic->notify();
      delay(500);
      if(PASSWORD.length() > 0){
         passkey = 1;  //received valid SSID    
         Serial.println("PSWD RECEIVED CONFIRM");   
      }else {
         passkey = 0;
         rxValue = "";
      }  
      }
      while (hostkey == 0){
      Serial.println("Ask for HOST"); 
      pTxCharacteristic->setValue("AWS HOST");
      pTxCharacteristic->notify();
      delay(500);
      if(host01.length() > 0 && host02.length() > 0 && host03.length() > 0){
         host = host01 + host02 + host03;
         hostkey = 1;  //received valid SSID    
         Serial.println(host);
         Serial.println("HOST RECEIVED CONFIRM");   
      }else if(host.length() > 0){
        Serial.println("use default host or dnt upload");
        delay(50);
        hostkey = 1;
      }else {
         hostkey = 0;
         rxValue = "";
      }
      }
      while (binkey == 0){
      Serial.println("Ask for Bin"); 
      pTxCharacteristic->setValue("AWS BIN");
      pTxCharacteristic->notify();
      delay(500);
      if(bin.length() > 0){
         binkey = 1;  //received valid SSID    
         Serial.println("BIN RECEIVED CONFIRM");   
      }else {
         binkey = 0;
         rxValue = "";
      }  
      }
      
      Serial.println("GET ALL 4 VALUE, CAN DISABLE THE BLE");
      BLEConnect = 1;
    }else {
      Serial.println("Not connected");
      delay (500);
    }
  }
//prepare to connect to WIFI and upload from host
//disable the BLE and connect to WIFI
//disconnect from phone 
     //esp_bluedroid_disable()
     esp_bt_controller_disable();
     delay(500);
     Serial.println("BLE disabled");
     Serial.println("Connecting to " + String(SSID));
  // Connect to provided SSID and PSWD
  WiFi.begin(SSID, PSWD);
  delay(20000);
  // Wait for connection to establish
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Need to Restart"); // Keep the serial monitor lit!
    delay(5000);
    ESP.restart();
  }

  // Connection Succeed
  Serial.println("");
  Serial.println("Connected to " + String(SSID));
  delay(50);
  Serial.println(host);
  Serial.println(bin);

  execOTA();
  

}
//END
  //touch PCB setup
  //get an initial gauge on the pin. Assume it may do some initial weirdness so take some time
   //  Takes  approx 300ms (300 samples at approx 1000 samples/sec)
  adcsample_and_lowpass(0, 1000, 300, 0.015, false); // see below
  touchAmbient = adcsample_and_lowpass(0,     // A0
                                             10000,  // aim for 10000Hz sampling
                                             50,    // follow 50 new samples
                                             0.015, // alpha
                                             true); // adapt from stored value for this pin (settled in setup())
  Serial.print("touchAmbient:"); Serial.println(touchAmbient);

  //Setup EEPROM
  EEPROM.begin(5);

//  EEPROM.write(0,0); //reset data eeprom to zero: preserver
//  EEPROM.write(1,0); //reset data eeprom to zero: winelife
//  EEPROM.write(2,0); //reset data eeprom to zero: wine vol largeByte
//  EEPROM.write(3,0); //reset data eeprom to zero: wine vol smallByte
//  EEPROM.write(4,0); //reset data eeprom to zero: hourCounter
//  EEPROM.commit();

  //Read from EEPROM, with various conditions below:
  hourCounter = EEPROM.read(4);
  if(hourCounter > 24) //means eeprom is not formatted properly
  {
    EEPROM.write(4,24); //reset hourcounter to zero
    EEPROM.commit();
    hourCounter = EEPROM.read(4);
  }
  
  //if limitswitch is triggered upon start, means there is a bottle, read the EEPROM for the wineLife, just continue
  if(digitalRead(limSW)==LOW)
  {
    wineLifeLeft = EEPROM.read(1);
    if(wineLifeLeft > wineLifeMax) 
    {
      wineLifeLeft = wineLifeMax;
      EEPROM.write(1,wineLifeMax);
    }
    wineVolLeft = (EEPROM.read(3) << 8) | EEPROM.read(2); //concatenate byte 3 and 2 tgt to get the number
    if(wineVolLeft > 750)
    {
      wineVolLeft = 750;
      EEPROM.write(2, (byte)(wineVolLeft & 0xFF) );
      EEPROM.write(3, (byte)((wineVolLeft >> 8) &0xFF));
      EEPROM.commit();
    }
    bottleFlag = true; //there is a bottle there
  }
  else
  {
    wineLifeLeft = 0;
    EEPROM.write(1,0); //reset wine left in eeprom to zero
    EEPROM.commit();
    wineLifeLeft = EEPROM.read(1);
    
    wineVolLeft = 0;
    //reset wine vol in eeprom to zero
    EEPROM.write(2, (byte)(wineVolLeft & 0xFF) );
    EEPROM.write(3, (byte)((wineVolLeft >> 8) &0xFF));
    EEPROM.commit();
    wineVolLeft = (EEPROM.read(3) << 8) | EEPROM.read(2); //concatenate byte 3 and 2 tgt to get the number
    
    bottleFlag = false;//no bottle there
  }

  //checkOA too
  if( analogRead(senIR1) < 3000 || analogRead(senIR2) < 3000 )
  {
    preserverLifeLeft = EEPROM.read(0);
    if(preserverLifeLeft > preserverLifeMax) 
    {
      preserverLifeLeft = preserverLifeMax;
      EEPROM.write(1,preserverLifeMax);
    }
  }
  else
  {
    preserverLifeLeft = 0;
    EEPROM.write(0,0); //reset OA in eeprom to zero
    EEPROM.commit();
  }

  Serial.print("wineLife: "); Serial.println(wineLifeLeft);
  Serial.print("wineVol: "); Serial.println(wineVolLeft);
  Serial.print("preserverLife: "); Serial.println(preserverLifeLeft);
  Serial.print("current hour: "); Serial.println(hourCounter);
  Serial.print("battery: "); Serial.println(checkBattV());
  Serial.println();

  //PWM setup: ledcAttachPin(channel#, freq, resolution) usu [0-15channels available],[5000],[8, 8 bit for 0-255]
  ledcSetup(pumpPWM, 30000, 8);
  ledcAttachPin(pump,pumpPWM);

  ledcSetup(platPWM, 30000, 8);
  ledcAttachPin(platPinSpeed,platPWM);

  //Current Sensor Setup
//  if (!ina219.begin()) {
//    Serial.println(F("Failed to find INA219 chip. Device is stuck"));
//    for(int i=0; i<=56; i++) 
//    { 
//      leds[i] = CRGB (255, 0,0);
//      FastLED.show();
//    }
//    while(1);
//  }

  //new ina219 https://www.best-microcontroller-projects.com/ina219.html library for 
  Wire.begin();
  if(!ina219.init()){
    Serial.println("INA219 not connected!");
  }
  ina219.setADCMode(SAMPLE_MODE_128);
  ina219.setMeasureMode(CONTINUOUS); // choose mode and uncomment for change of default
  /* Set PGain
  * Gain *  * Shunt Voltage Range *   * Max Current *
   PG_40       40 mV                    0,4 A
   PG_80       80 mV                    0,8 A
   PG_160      160 mV                   1,6 A
   PG_320      320 mV                   3,2 A (DEFAULT)
  */
  // ina219.setPGain(PG_160); // choose gain and uncomment for change of default
  /* If the current values delivered by the INA219 differ by a constant factor
     from values obtained with calibrated equipment you can define a correction factor.
     Correction factor = current delivered from calibrated equipment / current delivered by INA219
  */
  // ina219.setCorrectionFactor(0.98); // insert your correction factor if necessary
  
  //LED Setup
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS)
        .setCorrection( TypicalLEDStrip );
//  FastLED.setMaxPowerInVoltsAndMilliamps( 5, MAX_POWER_MILLIAMPS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  //Test LED
  FastLED.show();
  delay(50);

  

  //convert number to menu  

  for(int b=50; b<=255; b++)
  {
//    for(int i=45; i<=55; i++) 
    for(int i=49; i<=51; i++) 
    { 
      leds[i] = CRGB ( b, b, b);
    }
    leds[47] = CRGB::Black;
    leds[53] = CRGB::Black;
    FastLED.show();
    bottomLEDon = true;
    bottomLEDstartTime = millis();
    delay(10);
  }
  
  for(int i=3; i<=44; i++) 
  { 
    leds[i] = CRGB::White;
    FastLED.show();
    delay(50);
  }

  
  
  

  //test pump
  digitalWrite(valCirc, HIGH);
  delay(200);
  digitalWrite(valCirc, LOW);
  delay(200);
  digitalWrite(valRelease, HIGH);
  delay(200);
  digitalWrite(valRelease, LOW);
  delay(200);
  ledcWrite(pumpPWM, 255);
  delay(1000);
  ledcWrite(pumpPWM, 0);
  delay(100);
  
  
  for(int i=3; i<=44; i++) 
  { 
    leds[i] = CRGB::Black;
  }

  if(wineLifeLeft > 7)
      leds[1] = CRGB::Black;
  else if(wineLifeLeft > 3)
      leds[1] = CRGB::Orange;
  else
      leds[1] = CRGB::Red;     

  if(preserverLifeLeft > 37)
      leds[2] = CRGB::Black;
  else if(preserverLifeLeft > 14)
      leds[2] = CRGB::Orange;
  else 
      leds[2] = CRGB::Red; 
      
  if(batteryLiveV > 3.7)
      leds[0] = CRGB::Black;
  else if(batteryLiveV > 3.4)
      leds[0] = CRGB::Orange;
  else
      leds[0] = CRGB::Red; 

//  for(int i=3; i<=44; i++) 
//  { 
//    leds[i] = CRGB::Black; 
//  } 

 
  FastLED.show();
  delay(10);

  Serial.print("limSwitch: "); Serial.println(digitalRead(limSW));
  Serial.print("prevUp: "); Serial.println(prevUp);
  Serial.print("bottleFlag: "); Serial.println(bottleFlag);
  Serial.println();
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
//--LOOP-----------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------


void loop()
{
  Serial.println("firmware from AWS blink 02");
  //readCurrent not needed anymore, using raw data
//  Serial.print(ina219.getCurrent_mA());
//  Serial.print(",");
  
  //check for the touch button
  checkTouch();

  //show menu
  menuManagement();

  //manage circulation
  circulationManagement();

  //check OA and reset
  checkPreserver();

  //calculate lifetime left & store in memory, for now just do every 24 hours
  calcLifeTime();

  //check for limit switch
  checkDownBtn_Adv();

  //check for platformDownBtn
  checkLimSw_Adv();

  //sleepHandler
  sleepHandler();
  
  
}


//-----------------------------------------------------------------------------------------------------------------------------------------------
//--FUNCTIONS------------------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------------------------

void sleepHandler() //for now just handle the bottom LED sleep, later add device sleep if batt is too low
{
  unsigned long now = millis();
  if(bottomLEDon && (now - bottomLEDstartTime >= bottomLEDtimeout))
  {
    for(int b=255; b>=0; b--)
    {
//      for(int i=45; i<=55; i++) 
      for(int i=49; i<=51; i++)  
      { 
        leds[i] = CRGB ( b, b, b);
      }
      leds[47] = CRGB::Black;
      leds[53] = CRGB::Black;
      leds[49] = CRGB::White;
      leds[50] = CRGB::White;
      leds[51] = CRGB::White;
      FastLED.show();
      delay(10);
    }
    
    bottomLEDon = false;
    bottomLEDstartTime = millis();
    Serial.println("bottomLEDoff");
  }

  if(checkBattV() < 3.2)
  {
    for(int i=45; i<=55; i++) 
    { 
      leds[i] = CRGB::Red;
    }
    FastLED.show();
    delay(10);  
  }
}

void calcLifeTime()
{
  unsigned long now = millis();

  if(now - prevHour >= hourInMillis) //means one hour has passed
  {
    hourCounter = EEPROM.read(4) - 1;
    EEPROM.write(4,hourCounter); //reset hourcounter to zero
    EEPROM.commit();
    
    Serial.print("One hour has passed. current hour: "); Serial.println(hourCounter);
  }

  if(hourCounter <= 0) //means one day has passed, reduce the lifetime by 1 day
  {
    wineLifeLeft -= 1;
    EEPROM.write(1,wineLifeLeft);
    preserverLifeLeft -= 1;
    EEPROM.write(0,preserverLifeLeft);
  
    hourCounter = 24;
    EEPROM.write(4,hourCounter); //reset hourcounter to zero
    
    EEPROM.commit();

    Serial.println("One Day has passed..");
    Serial.print("wineLife: "); Serial.println(wineLifeLeft);
    Serial.print("wineVol: "); Serial.println(wineVolLeft);
    Serial.print("preserverLife: "); Serial.println(preserverLifeLeft);
    Serial.print("current hour: "); Serial.println(hourCounter);
  }


  prevHour = now;
}

void checkTouch() 
{
  int countLEDup;
    //digitalRead raw touch---------------------------------------------------------------------------------
    int touchVal = adcsample_and_lowpass(0,     // ignore this, I modified the function partway
                                               10000,  // aim for 1000Hz sampling
                                               50,    // follow 50 new samples
                                               0.015, // alpha
                                               true); // adapt from stored value for this pin (settled in setup())
//    Serial.println(touchVal);
    
    //set touch current threshold---------------------------------------------------------------------------
    if (touchAmbient - touchVal >= touchThres) //if touch is pressed currently
    {
      touchCurrentState = true; //just set active low for all buttons
//      Serial.println(touchVal);
    }
    else if(touchAmbient - touchVal < touchThres) //if touch is unpressed currently
    {
      touchCurrentState = false; //just set active low for all buttons
    }
    // else do nothing

//     //temp test
//    if(Serial.available())
//    {
//      char cmd = Serial.read();
//      if(cmd == 'q')
//      {
//        touchCurrentState = true;
//      }
//      else if(cmd == 'w')
//      {
//        touchCurrentState = false;
//      }
//    }

    //Record the initial touch button press time--------------------------------------------------------
    if (touchLastState == false && touchCurrentState == true) //Record time when button is pressed for the first time and not before
    {
      touchPressedTime = millis();
        
    }

    //Action points when button is held down------------------------------------------------------------
    unsigned long touchHoldTime = millis() - touchPressedTime;
    if (touchLastState == true && touchCurrentState == true && touchHoldTime > LONG_PRESS_TIME && bottleFlag == true) //if button is longpresshold, with bottle there, dispense the wine
    {
//      Serial.println(F("button is longpressHold after 2 seconds, bottle is present, LONG PRESS means dispense!: circVal HIGH, pump255"));
      digitalWrite(valCirc, HIGH);
      ledcWrite(pumpPWM, 255);
    }
    else if(touchLastState == true && touchCurrentState == true && touchHoldTime > LONG_PRESS_TIME && bottleFlag == false) //if button is longpresshold, with bottle NOT there, blink LED
    {
//      Serial.println(F("button is longpressHold after 2 seconds, bottle is NOT present, do not Dispense, just blink LED!"));
      blinkRed();
    }
    else if(touchLastState == true && touchCurrentState == true && touchHoldTime > SHORT_PRESS_TIME && touchHoldTime <= LONG_PRESS_TIME) //if button is pressHOLD, but time is less than longpressduration
    {
//      Serial.print(F("button is pressHold <2sec, so count down that is translated to LED: "));
//      int numLEDon = map(touchHoldTime, 0, LONG_PRESS_TIME, firstTopLED, lastTopLED); 
//      Serial.print(touchHoldTime);
//      Serial.print("\t");
//      Serial.println(numLEDon);

//      //simple LED ring
//      for(int i = numLEDon+1; i <= lastTopLED; i++) 
//      { 
//        leds[i] = CRGB ( 0,0,0);
//      } 
//      for(int i=firstTopLED; i<=numLEDon+1; i++) 
//      { 
//        leds[i] = CRGB::White;
//      }     

      //for back to front
      countLEDup = map(touchHoldTime, SHORT_PRESS_TIME, LONG_PRESS_TIME, 0, (lastTopLED-firstTopLED)/2+2);
      for(int i = firstTopLED; i <= midTopLED-countLEDup; i++) 
      { 
        leds[i] = CRGB::Black;
      } 
      for(int i = midTopLED+countLEDup; i <= lastTopLED; i++) 
      { 
        leds[i] = CRGB::Black;
      } 
      for(int i=midTopLED-countLEDup; i<midTopLED+countLEDup; i++) 
      { 
        leds[i] = CRGB::White;
      }     
      int bri=map(touchHoldTime, 0, LONG_PRESS_TIME, 0, 255);
      for(int i=0; i<=2; i++) 
      { 
        leds[i] = CRGB(bri,bri,bri);
      }
      FastLED.show();
      delay(20);
    }
    
    //action AFTER BUTTON RELEASE for long-press and short press---------------------------------------
    if (touchLastState == true && touchCurrentState == false && touchHoldTime <= DEBOUNCE_TIME ) //when button released, and duration <debounce
    { 
      //do nothing
    }
    if (touchLastState == true && touchCurrentState == false && touchHoldTime > DEBOUNCE_TIME && touchHoldTime < LONG_PRESS_TIME) //when button released, and duration<LONGPRESS = short press is released, cycle menu
    {  
//        Serial.print(F("A short touch press is detected, cycle through the menu, showing: "));
        statusIndex += 1;
        statusIndex %= 3; //to let it cycle between 0-3
//        Serial.println(statusIndex);
//        displayOnStart = millis();

//      //while countLED wasn't down, lower the LED first
//      while(countLEDup > 0)
//      {
//        countLEDup -= 1;
//        for(int i = firstTopLED; i <= midTopLED-countLEDup; i++) 
//        { 
//          leds[i] = CRGB::Black;
//        } 
//        for(int i = midTopLED+countLEDup; i <= lastTopLED; i++) 
//        { 
//          leds[i] = CRGB::Black;
//        } 
//        for(int i=midTopLED-countLEDup; i<=midTopLED+countLEDup; i++) 
//        { 
//          leds[i] = CRGB::White;
//        }     
//        
//        FastLED.show();
//        delay(50);
//      }

      if(bottomLEDon == false)
      {
//        for(int b=50; b<=255; b++)
        {
//          for(int i=45; i<=55; i++) 
          for(int i=49; i<=51; i++) 
          { 
//            leds[i] = CRGB ( b, b, b);
              leds[i] = CRGB::White;
          }
          leds[47] = CRGB::Black;
          leds[53] = CRGB::Black;
          FastLED.show();
          
          bottomLEDon = true;
          bottomLEDstartTime = millis();
//          delay(10);
        }
      }
      else //just keep it on and extend the millis
      {
        bottomLEDstartTime = millis();
      }
    }

    else if (touchLastState == true && touchCurrentState == false && touchHoldTime > LONG_PRESS_TIME && bottleFlag == false) //when button released, and duration<LONGPRESS == long press is released: turn off pump and do pressure release sequence
    {
      Serial.println(F("A long touch press was just released with bottle not present, blink golden LED then turn off"));
      blinkRed();
    }

    else if (touchLastState == true && touchCurrentState == false && touchHoldTime > LONG_PRESS_TIME && bottleFlag == true && wineVolLeft <= 0) //when button released, and duration<LONGPRESS == long press is released: turn off pump and do pressure release sequence
    {
      Serial.println(F("A long touch press was just released with bottle, but no more wine, blink golden LED then turn off"));
      blinkRed();
    }
  
    else if (touchLastState == true && touchCurrentState == false && touchHoldTime > LONG_PRESS_TIME && bottleFlag == true && wineVolLeft > 0) //when button released, and duration<LONGPRESS == long press is released: turn off pump and do pressure release sequence
    {     
        Serial.println(F("A long touch press was just released with bottle in, turn off dispensing, follow by circulating"));
        Serial.print(F("stopping..."));
        ledcWrite(pumpPWM, 0); //turn off pump
        digitalWrite(valRelease, HIGH); //turn on release valve
        digitalWrite(valCirc, HIGH); //keep on dispensing valve to help release pressure
        unsigned long coolDownStart = millis();
        unsigned long coolDownTimeNow = millis() - coolDownStart;

//        //turn all LEDs first GOLD
//        for(int i=3; i<=44; i++) 
//        { 
//          pixels.setPixelColor(i, GOLD);
//        }
//        pixels.show();   // Send the updated pixel colors to the hardware.
//        delay(10); // Pause before next pass through loop   
        
        while(coolDownTimeNow < pressureReleaseTime)
        {
          coolDownTimeNow = millis() - coolDownStart;
//          int numLEDCountdown = map(coolDownTimeNow, 0, pressureReleaseTime, 44, 3);          
////          Serial.print(coolDownTimeNow);
////          Serial.print(F("\t"));
////          Serial.println(numLEDCountdown);
//          leds[numLEDCountdown] = CRGB::Black;
//          FastLED.show();
//          delay(40);

          int countLEDdown = map(coolDownTimeNow, 0, pressureReleaseTime, (lastTopLED-firstTopLED)/2+1, 0);
          for(int i=midTopLED-countLEDdown; i<midTopLED+countLEDdown; i++) 
          { 
            leds[i] = CRGB::White;
          }   
          for(int i = firstTopLED; i <= midTopLED-countLEDdown; i++) 
          { 
            leds[i] = CRGB::Black;
          } 
          for(int i = midTopLED+countLEDdown; i <= lastTopLED; i++) 
          { 
            leds[i] = CRGB::Black;
          } 
            
          int bri=map(touchHoldTime, 0, pressureReleaseTime, 255, 0);
          for(int i=0; i<=2; i++) 
          { 
            leds[i] = CRGB(bri,bri,bri);
          }
          if(countLEDdown == 0) 
          {
            leds[midTopLED] = CRGB::Black;
            for(int i=0; i<=2; i++) 
            { 
              leds[i] = CRGB::Black;
            }
          }
          FastLED.show();
          delay(20);
        }
          
        Serial.println(F("stopped"));
        digitalWrite(valRelease, LOW); //turn off release valve
        digitalWrite(valCirc, LOW); //also turn off dispensing valve
        

        //calculating the dispensing length
        dispensingDuration = touchHoldTime - LONG_PRESS_TIME;
        int wineUsed = map(dispensingDuration, 0, 14000, 0, 150); //convert 14sec to 150ml
        wineVolLeft -= wineUsed; // reduce from the wine left

        EEPROM.write(2, (byte)(wineVolLeft & 0xFF) );
        EEPROM.write(3, (byte)((wineVolLeft >> 8) &0xFF));
        EEPROM.commit();
        wineVolLeft = (EEPROM.read(3) << 8) | EEPROM.read(2); //concatenate byte 3 and 2 tgt to get the number

        Serial.print(F("wine vol left = "));
        Serial.println(wineVolLeft);

        //set flag if there is wineLeft, (and bottle is still there), tell the system that dispensing has just happened the time
        if(wineVolLeft > 0)
        {
          wineLastDispensedTime = millis();
        }
        
        
        Serial.println(F("ready again"));
        statusIndex = -2; //just give a random number so the menu interface gets cleared out in 5 seconds
      
    }

    //when the button is not touched at all, then.. -----------------------------------------------------------------------------
    else if (touchLastState == false && touchCurrentState == false && wineVolLeft > 0 && bottleFlag == true)
    {
      //do nothing for now
    }
    
    // save the the last state
    touchLastState = touchCurrentState;
}

void blinkRed() //blink 2 times RED now
{
  Serial.println("blinkRedTwice");
  //off
  for(int i=firstTopLED; i<=lastTopLED; i++) 
  { 
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(250);
  
  //on
  for(int i=firstTopLED; i<=lastTopLED; i++) 
  { 
    leds[i] = CRGB::Red;
  }
  FastLED.show();
  delay(250);
  
  
  //off
  for(int i=firstTopLED; i<=lastTopLED; i++) 
  { 
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(250);
  
  //on
  for(int i=firstTopLED; i<=lastTopLED; i++) 
  { 
    leds[i] = CRGB::Red;
  }
  FastLED.show();
  delay(250); 

  //off
  for(int i=firstTopLED; i<=lastTopLED; i++) 
  { 
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(250);
}

void menuManagement()
{
  //change menu and record time if there is any change due to short press
  if(statusIndex != prevStatus) //if there is a change in status
  {
    //record the time it is changed
    displayOnStart = millis();

    //display the next menu
        
    switch(statusIndex)
    { 
      case -1: //turn off all LEDs, and turn on the 3 middle ones 
        Serial.println(F("Menu Summary: minimized all menu to 3 middle buttons"));
         //Test temp 3 middle LED
        if(wineLifeLeft > 7)
            leds[1] = CRGB::Black;
        else if(wineLifeLeft > 3)
            leds[1] = CRGB::Orange;
        else
            leds[1] = CRGB::Red;     
      
        if(preserverLifeLeft > 37)
            leds[2] = CRGB::Black;
        else if(preserverLifeLeft > 14)
            leds[2] = CRGB::Orange;
        else 
            leds[2] = CRGB::Red; 
            
        if(batteryLiveV > 3.7)
            leds[0] = CRGB::Black;
        else if(batteryLiveV > 3.4)
            leds[0] = CRGB::Orange;
        else
            leds[0] = CRGB::Red; 

        for(int i=3; i<=44; i++) 
        { 
          leds[i] = CRGB::Black; 
        } 
        FastLED.show();
        delay(10);
      break;

      case 0: //turn on 2nd LED, and show the preserver on top LEDs
        Serial.print(F("Menu 1: Preserver Life left: "));
        Serial.println(preserverLifeLeft);
         //Test temp 3 middle LED
        leds[2] = CRGB::White;      
        leds[1] = CRGB::Black;  
        leds[0] = CRGB::Black; 
        
        //convert number to menu
        int preserverLED;
        preserverLED = map(preserverLifeLeft, 0, preserverLifeMax, 3, 44); 

        //convert number to menu
        for(int i=preserverLED; i<=44; i++) 
        { 
          leds[i] = CRGB::Black; 
        } 
        for(int i=3; i<= preserverLED; i++)
        {
          if(preserverLifeLeft > 14)
            leds[i] = CRGB::White;
          else
            leds[i] = CRGB::Red; 
        }
        
              
        FastLED.show();
        delay(10); 
      break;

      case 1: //turn on 1st LED, and show the winelife on top LEDs
        Serial.print(F("Menu 2: Wine Life left: "));
        Serial.println(wineLifeLeft);
         //Test temp 3 middle LED
        leds[2] = CRGB::Black;      
        leds[1] = CRGB::White;  
        leds[0] = CRGB::Black; 
        
        
        //convert number to menu
        int wineLED;
        wineLED = map(wineLifeLeft, 0, wineLifeMax, 3, 44); 

        //convert number to menu
        
        for(int i=wineLED; i<=44; i++) 
        { 
          leds[i] = CRGB::Black; 
        } 
        for(int i=3; i<= wineLED; i++)
        {
          if(wineLifeLeft > 3)
            leds[i] = CRGB::White;
          else
            leds[i] = CRGB::Red; 
        }
        FastLED.show();
        delay(10); 
      break;
    
      case 2: //turn on 3rd LED, and show the battery on top LEDs
        Serial.print(F("Menu 3: Battery Life left: "));
        checkBattV(); //check batt voltage
        Serial.println(batteryLiveV);
         //Test temp 3 middle LED
        leds[2] = CRGB::Black;      
        leds[1] = CRGB::Black;  
        leds[0] = CRGB::White; 
        
        
        int battLED = 10 * batteryLiveV;
//        Serial.println(battLED);
        battLED =map(battLED, 32, 42, 3, 44); 

        //convert number to menu
        for(int i=battLED; i<=44; i++) 
        { 
          leds[i] = CRGB::Black; 
        } 
        for(int i=3; i<= battLED; i++)
        {
          if(batteryLiveV > 3.4)
            leds[i] = CRGB::White;
          else
            leds[i] = CRGB::Red; 
        }
        
              
        FastLED.show();
        delay(10); 
      break;
    }
    
    prevStatus = statusIndex;
  }

  //turn off when display time is up
  unsigned long now = millis();
//  Serial.println(now - displayOnStart);
  if(now - displayOnStart >= displayOnDuration)
  {
    statusIndex = -1;
  }
  
}

void circulationManagement()
{
  //first 1 hour since the wineLastDispensedTime, powerful circulation
  unsigned long timeSinceLastDispense = millis() - wineLastDispensedTime;
  if(touchLastState == false && touchCurrentState == false && wineVolLeft > 0 && bottleFlag == true && timeSinceLastDispense <= POWERFUL_CIRC_DUR && downTillHit ==false) //make sure the bottle is in & some volume is left, and no touch is happening
  {
//    Serial.println(F("powerful circulation"));
    digitalWrite(valRelease, LOW); //turn off release valve
    digitalWrite(valCirc, LOW); //turn off dispensing valve to help release pressure
    ledcWrite(pumpPWM, circPower);
  }

  //afterwards, circulation 5mins every 10 mins
  else if(touchLastState == false && touchCurrentState == false && wineVolLeft > 0 && bottleFlag == true && timeSinceLastDispense > POWERFUL_CIRC_DUR) //make sure the bottle is in & some volume is left, and no touch is happening
  {
//    Serial.println(F("normal circulation"));
    digitalWrite(valRelease, LOW); //turn off release valve
    digitalWrite(valCirc, LOW); //turn off dispensing valve to help release pressure
    circFreq(60000, 60000, circPower); //control the frequency
  }

  else if(bottleFlag == false || digitalRead(limSW) == HIGH) //if bottleFlag is not there or the limit switch is unpressed
  {
    digitalWrite(valRelease, LOW); //turn off release valve
    digitalWrite(valCirc, LOW); //turn off dispensing valve to help release pressure
    ledcWrite(pumpPWM, 0);
    bottleFlag = false;
  }
  
  //else do nothing
  
}

void circFreq(unsigned long tCircOn, unsigned long tCircOff, int amp)
{
//  //for regular circulation

    //cycle the tprev to last one period
    t = millis();
//    Serial.print(t - tprev);
//    Serial.print("\t");
//    Serial.print(tprev);
//    Serial.print("\t");
    unsigned long tperiod = (tCircOn + tCircOff);

    //phase reset
    if (t - tprev > tperiod)
    {
      tprev = t;
    }

    //first phase: pump on
    else if (t - tprev <= tCircOn)
    {
      //turn pump on
//      Serial.print(F("circ is On, with amp = "));
//      Serial.print(amp);
      ledcWrite(pumpPWM, amp);
    }

    //second phase: pump off
    else if (t - tprev > tCircOn)
    {
      //keep pump off, turn valve off
//      Serial.println(F("pump is off"));
      ledcWrite(pumpPWM, 0);
    }

}

void checkPreserver()
{
  if( analogRead(senIR1) < 3000 || analogRead(senIR2) < 3000 )
  {
    currentOAstate = true;
//    Serial.println("OA detected!");
  }
  else
  {
    currentOAstate = false;
  }

  if(prevOAstate == false && currentOAstate == true) //plugged out and in back
  {
    Serial.println(F("reset preserver lifetime to MAX"));
    preserverLifeLeft = preserverLifeMax;
    EEPROM.write(0,preserverLifeMax); //reset wine left in eeprom to zero
    EEPROM.commit();
  }

  else if(prevOAstate == true && currentOAstate == false)//plugged out
  {
    Serial.println(F("reset preserver lifetime to Zero"));
    preserverLifeLeft = 0;
    EEPROM.write(0,0); //reset wine left in eeprom to zero
    EEPROM.commit();
  }

  prevOAstate = currentOAstate;
}

void resetUpFlag() //to reset the limit switch flags
{
//  bottleFlag = false;
  prevUp = HIGH;
  upTillHit = false;
}

void resetDownFlag()
{
  prevDown = HIGH;
  downTillHit = false;
}

void resetBottleFlag() //only call this when the platform is at the bottom most, reset the wineLife
{
    bottleFlag = false;//no bottle there
}

void resetWineLifetoMax()
{
  EEPROM.write(1,wineLifeMax); //reset wine left in eeprom to MAX
  EEPROM.commit();
  wineLifeLeft = EEPROM.read(1);

  wineVolLeft = wineVolMax;
  //reset wine vol in eeprom to zero
  EEPROM.write(2, (byte)(wineVolLeft & 0xFF) );
  EEPROM.write(3, (byte)((wineVolLeft >> 8) &0xFF));
  EEPROM.commit();
  wineVolLeft = (EEPROM.read(3) << 8) | EEPROM.read(2); //concatenate byte 3 and 2 tgt to get the number
  
  Serial.printf("wine life replenished: %d",wineLifeLeft);
  Serial.println("");
  Serial.printf("wine vol replenished: %d",wineVolLeft);
  Serial.println("");
}

void resetWineLifetoMin()
{
  EEPROM.write(1,0); //reset wine left in eeprom to zero
  EEPROM.commit();
  wineLifeLeft = EEPROM.read(1);

  wineVolLeft = 0;
  //reset wine vol in eeprom to zero
  EEPROM.write(2, (byte)(wineVolLeft & 0xFF) );
  EEPROM.write(3, (byte)((wineVolLeft >> 8) &0xFF));
  EEPROM.commit();
  wineVolLeft = (EEPROM.read(3) << 8) | EEPROM.read(2); //concatenate byte 3 and 2 tgt to get the number
  
  Serial.printf("wine life zeroed: %d",wineLifeLeft);
  Serial.println("");
  Serial.printf("wine vol zeroed: %d",wineVolLeft);
  Serial.println("");
}

void checkDownBtn_Adv() //down button and prevDown is active LOW; advanced mode for deployment
{
  if(digitalRead(lowerBtn) == LOW && prevDown == HIGH) //only the first time being pressed
  {
    downTillHit = true;
    platPWMval = 100;

    digitalWrite(platPin1, LOW);
    digitalWrite(platPin2, HIGH);
    delay(100);
    
    prevDown = LOW; // this flag is reactivated only when the current limit has been reached

    //also use this button to reset all the limit switch property, including the bottleFlag
    resetUpFlag();
  }

  

    //read current when it is in the while loop
//    currentAveRead();
  
    //THIS FUNCTION IS TO CATCH IF THE CURRENT GOES BEYOND THE STOP CURRENT LIMIT, TURN EVERYTHING OFF BY FORCE IMMEDIATELLY
//    if(abs(curAverage) >= stopCurDown && downTillHit == true)
    if(abs(ina219.getCurrent_mA()) >= stopCurDown && downTillHit == true)
    {
      downCount += 1;
      Serial.printf("count down hit: %d", upCount);

      if(downCount >= countLimit)
      {     
        digitalWrite(platPin1, LOW);
        digitalWrite(platPin2, LOW);
        platPWMval = 0;
        ledcWrite(platPWM, 0);
        downTillHit = false;
        upCount = 0;
        downCount = 0;
      }
      delay(10);
    }

    if (downTillHit == true && prevDown == LOW) //when button is just pressed, and speed is maxed
    {
      ledcWrite(platPWM, 255);
    }
    
//    if(downTillHit == true && prevDown == LOW && platPWMval < 255) //when button is just pressed, to ramp up the speed
//    {
//      platPWMval += 5;
//      ledcWrite(platPWM, platPWMval);
//    }
//    
//    else if (downTillHit == true && prevDown == LOW && platPWMval >= 255) //when button is just pressed, and speed is maxed
//    {
//      ledcWrite(platPWM, 255);
//    }
    
    else if(downTillHit == false && prevDown == LOW)  //when the downTillHit is Called OFF for the first time
    {
      digitalWrite(platPin1, LOW);
      digitalWrite(platPin2, LOW);
      platPWMval = 0;
      ledcWrite(platPWM, 0);
      prevDown = HIGH;  //the button can be pressed again, calling prevDown to be HIGH again
      resetBottleFlag(); //to unlock the limit switch, so it will be functional again
      resetWineLifetoMin();
    }
  
    else if(downTillHit == false && prevDown == HIGH) //do nothing when there is no action
    {
      ; //do nothing
    }
}

void checkLimSw_Adv() //LIMIT SWITCH AND PREVUP IS ACTIVE LOW; advanced mode for deployment
{
  if(digitalRead(limSW) == LOW && prevUp == HIGH && bottleFlag == false) //only the first time being pressed
  {
    Serial.println("limit switch pressed, activate platform up, do only once");
    upTillHit = true;
    platPWMval = 100;
    digitalWrite(platPin1, HIGH);
    digitalWrite(platPin2, LOW);
    delay(100);

    prevUp = LOW; //only do once, prevUp is reactivated only when the current limit has been reached
  }


    //read current when it is in the while loop
//    currentAveRead();
  
    //THIS FUNCTION IS TO CATCH IF THE CURRENT GOES BEYOND THE STOP CURRENT LIMIT, TURN EVERYTHING OFF BY FORCE IMMEDIATELLY
//    if(abs(curAverage) >= stopCurUp && upTillHit == true) 
    if(abs(ina219.getCurrent_mA()) >= stopCurUp && upTillHit == true) 
    {
      upCount += 1;
//      Serial.printf("count up hit: %d", upCount);

      if(upCount >= countLimit)
      {     
        digitalWrite(platPin1, LOW);
        digitalWrite(platPin2, LOW);
        platPWMval = 0;
        ledcWrite(platPWM, 0);
        upTillHit = false;
        upCount = 0;
        downCount = 0;
      }
      delay(10); //to give time
    }

    if(upTillHit == true && prevUp == LOW) //when button is just pressed, and speed is maxed
    {
      ledcWrite(platPWM, 255);
    }
    
//    if(upTillHit == true && prevUp == LOW && platPWMval < 255) //when button is just pressed, to ramp up the speed
//    {
//      platPWMval += 5;
//      ledcWrite(platPWM, platPWMval);
//    }
//    
//    else if(upTillHit == true && prevUp == LOW && platPWMval >= 255) //when button is just pressed, and speed is maxed
//    {
//      ledcWrite(platPWM, 255);
//    }
    
    else if(upTillHit == false && prevUp == LOW)  //when the upTillHit is Called OFF for the first time
    {
      digitalWrite(platPin1, LOW);
      digitalWrite(platPin2, LOW);
      platPWMval = 0;
//      ledcWrite(platPWM, 0);
      
      prevUp = HIGH; 
      bottleFlag = true;
      float batt = checkBattV();
//      Serial.printf("battery level = %f", batt);

      resetWineLifetoMax();
      
    }
  
    else if(upTillHit == false && prevUp == HIGH) //do nothing when there is no action
    {
      ; //do nothing
    }

    
  
}

void smoothingSetup()
{
  for (int thisReading = 0; thisReading < numReadings; thisReading++) 
  {
    readings[thisReading] = 0;
  }
}

int smoothingTouch(int val)
{
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = val;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  int averagedResult = total / numReadings;
  return averagedResult;
  // send it to the computer as ASCII digits
//  Serial.println(curAverage);
//  delay(1);        // delay in between reads for stability
}


float checkBattV()
{
  //Battery volt meter converting 4.2v to 3.3v at battery max
  int battRaw = analogRead(34);  //not working, maxing out, weird..
//  Serial.print(battRaw);
//  Serial.print("  ");
  batteryLiveV = 0;
  for(int i=0; i<50; i++)
  {
    battRaw = analogRead(34);
    batteryLiveV += (float(battRaw))/4095.0*4.2; //max is is 4.2V
  }
  batteryLiveV /= 50.0;
  return batteryLiveV;
}

int adcsample_and_lowpass(int pin, int sample_rate, int samples, float alpha, char use_previous) 
{
  // pin:            arduino analog pin number to sample on   (should be < LOWPASS_ANALOG_PIN_AMT)
  // sample_rate:    approximate rate to sample at (less than ~9000 for default ADC settings)
  // samples:        how many samples to take in this call  (>1 if you want smoother results)
  // alpha:          lowpass alpha
  // use_previous:   If true,  we continue adjusting from the most recent output value.
  //                 If false, we do one extra analogRead here to prime the value.
  //   On noisy signals this non-priming value can be misleading, 
  //     and with few samples per call it may not quite adjust to a realistic value.
  //   If you want to continue with the value we saw last -- which is most valid when the
  //     value is not expected to change significantly between calls, you can use true.
  //   You may still want one initial sampling, possibly in setup(), to start from something real.
 
  float one_minus_alpha = 1.0-alpha;
  int micro_delay=max(100, (1000000/sample_rate) - 160); // 160 being our estimate of how long a loop takes 
               //(~110us for analogRead at the default ~9ksample/sec,  +50 grasped from thin air (TODO: test)  
  if (!use_previous) { 
    //prime with a real value (instead of letting it adjust from the value in the arrays)
    lowpass_input[pin] = touchRead(T3);
    lowpass_prev_out[pin]=lowpass_input[pin]; 
  }
 
  //Do the amount of samples, and lowpass along the way  
  int i;
  for (i=samples;i>0;i--) {
    delayMicroseconds(micro_delay);
    lowpass_input[pin] = touchRead(T3);
    while(lowpass_input[pin] < 10)
      lowpass_input[pin] = touchRead(T3);
    lowpass_cur_out[pin] = alpha*lowpass_input[pin] + one_minus_alpha*lowpass_prev_out[pin];
    lowpass_prev_out[pin]=lowpass_cur_out[pin];
  }
  return lowpass_cur_out[pin];
}

//for new INA219 
void printFixedPointdp(long v, int dp) {

   long dpdiv = 1;
   for(int i=0;i<dp;i++,dpdiv*=10);

   long left = v / dpdiv;
   long rght = v % dpdiv;
   if (left==0) Serial.print('0');  else Serial.print(left);
   Serial.print('.');
   if (rght==0) Serial.print("000");  else Serial.print(rght);
}
//---- UNUSED --------------------------------------------------------------------------------------------------

//
//void currentAveRead()
//{
//  int currentRead = ina219.getCurrent_mA();
//  
//  if(currentRead > readCurCap) // max out high spikes to current max cap
//    currentRead = readCurCap;
//  else if(currentRead < -readCurCap) // max out high (-) spikes to current max cap
//    currentRead = -readCurCap;
//    
//  smoothingCurrent(); //average the reading to eliminate jitters
//  
////  Serial.print(currentRead);
////  Serial.print(",");
//  if(curAverage != 0)
//  {
//    Serial.println(curAverage);
//  }
//
////  //convert number to menu
////  int currentLED;
////  currentLED = map(abs(curAverage), 0, readCurCap, 3, 44); 
////
////  //convert number to menu
////  
////  for(int i=currentLED; i<=44; i++) 
////  { 
////    leds[i] = CRGB::Black; 
////  } 
////  for(int i=3; i<= currentLED; i++)
////  {
////    leds[i] = CRGB (255, 0, 215);
////  }
////  leds[3] = CRGB::Black; //just turn off LED 3 coz it is annoying
////  FastLED.show();
////  delay(20); 
//}
//
//void printAllCurrentSensor()
//{
//  shuntvoltage = ina219.getShuntVoltage_mV();
//  busvoltage = ina219.getBusVoltage_V();
//  current_mA = ina219.getCurrent_mA();
//  power_mW = ina219.getPower_mW();
//  loadvoltage = busvoltage + (shuntvoltage / 1000);
//  
//  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
//  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
//  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
//  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
//  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
//  Serial.println("");
//}
//
//void printCurrent()
//{
//  current_mA = ina219.getCurrent_mA();
//  
//  Serial.print(current_mA); Serial.println(" mA");
//}
//
//void smoothingCurrent()
//{
//  // subtract the last reading:
//  total = total - readings[readIndex];
//  // read from the sensor:
//  readings[readIndex] = ina219.getCurrent_mA();
//  // add the reading to the total:
//  total = total + readings[readIndex];
//  // advance to the next position in the array:
//  readIndex = readIndex + 1;
//
//  // if we're at the end of the array...
//  if (readIndex >= numReadings) {
//    // ...wrap around to the beginning:
//    readIndex = 0;
//  }
//
//  // calculate the average:
//  curAverage = total / numReadings;
//  // send it to the computer as ASCII digits
////  Serial.println(curAverage);
////  delay(1);        // delay in between reads for stability
//}
