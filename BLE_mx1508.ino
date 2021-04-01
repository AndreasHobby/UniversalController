/* (c)2021 Andreas' Hobby
 * UniversalController v1.0
 * Sketch is controlling up to 4 motors via mx1508 driver boards and an ESP32 device 
 *  - controllable via HTTP request (implemented), MQTT possible to adapt
 *  - controllable via mobile phone and finally via Gamepad by connecting to 3rd Party Apps like BrickController 2
 * Based on examples from Neil Kolban and chegewara
 * IMPORTANT:   If you have a compiling error due to Flashrom comsumption, use the setting "Partition Scheme: Minimal SPIFFS", this enables 1.9MB of Flashrom instead of 1.2MB
 * Open topic:  Sometimes a reset is needed to connect to BLE or WLAN devices
*/
//*****************************************************************************************************************
//headers and installation hints
//install ESP32 via Boardverwalter           url=https://dl.espressif.com/dl/package_esp32_index.json
#include <ArduinoOTA.h>                    //url=https://github.com/jandrassy/ArduinoOTA
#include <WiFi.h>                          //built in via ESP32 installation
#include <analogWrite.h>                   //url=https://github.com/ERROPiX/ESP32_AnalogWrite
#include <ESPAsyncWebServer.h>             //url=https://github.com/me-no-dev/ESPAsyncWebServer
#include "PasswordSettings.h"              //define ssid and password String in  external header file
#include <BLEDevice.h>                     //url=https://github.com/nkolban/ESP32_BLE_Arduino (or built in via ESP32 in newer installations)
#include <BLEUtils.h>
#include <BLEServer.h>
//*****************************************************************************************************************
// ** WLAN declarations **
String IP = "";                            //stores current IP address
AsyncWebServer server(80);                 //stores webserver data
//*****************************************************************************************************************
// ** PWM declarations **
#define MOTOR_MAX                   4      //define number of motors to be controlled (can be adapted as long as compatible outputs are free)
int Pin1[MOTOR_MAX] = {13, 14, 26, 33};    //define pin1 for connection to mx1508 driver(s)
int Pin2[MOTOR_MAX] = {12, 27, 25, 32};    //define pin2 for connection to mx1508 driver(s)
int MotorSpeed[MOTOR_MAX];                 //stores the current motor speed (range: -255 .. 0 .. +255)
//*****************************************************************************************************************
// ** WATCHDOG declarations **
#define Config_Watchdog             true   //configure watchdog feature
#define Config_WatchdogTimeoutTime  5000   //define watchdog timeout in Milliseconds
bool WatchdogState[MOTOR_MAX];             //stores state of watchdog
unsigned long WatchdogTimeout[MOTOR_MAX];  //stores time stamp the watchdog will stop the motor if no request is read in
//*****************************************************************************************************************
// ** voltage measuring declarations **
#define Config_VoltageReadInterval  60000  //define voltage read interval in Milliseconds
#define PinVoltage                  A0     //define pin to use for measure voltage via ADC
float AkkuVoltage = 0;                     //stores the current measured Akku voltage (Vin)
unsigned long NextVoltageRead = 0;         //stores time stamp the voltage will be read next time
//*****************************************************************************************************************
// ** BLE declarations (UUID's from https://social.sbrick.com/custom/The_SBrick_BLE_Protocol.pdf) **
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
#define SERVICE_UUID_DEVICE_INFORMATION    "0000180a-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_MODEL                 "00002a24-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_FIRMWARE_REVISION     "00002a26-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_HARDWARE_REVISION     "00002a27-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_SOFTWARE_REVISION     "00002a28-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_NAME                  "00002a29-0000-1000-8000-00805f9b34fb"
#define SERVICE_UUID_REMOTE_CONTROL        "4dc591b0-857c-41de-b5f1-15abda665b0c"
#define CHARACTERISTIC_UUID_REMOTE_CONTROL        "02b8cbcc-0e25-4bda-8790-a15f53e6010f"
#define CHARACTERISTIC_UUID_QUICK_DRIVE           "489a6ae0-c1ab-4c9c-bdb2-11d373c1b7fb"
//*****************************************************************************************************************
//*****************************************************************************************************************
// webserver callbacks
void CALLBACK_notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}
void CALLBACK_handleRoot(AsyncWebServerRequest *request) {
  String head = "<!DOCTYPE HTML><html><head><title>ESP Input Form</title>\
  <meta name='viewport' content='width=device-width, initial-scale=1'></head><body>\
  <form action='/get'>Motor: <input type='text' name='Motor' value='"+String(0)+"'><input type='submit' value='Submit'></form><br>\
  RSSI: "+String(WiFi.RSSI())+"\
  AkkuVoltage: "+String(AkkuVoltage/100)+"\
  MotorSpeed: "+String(MotorSpeed[0])+"\
  <br\></body></html>";
 request->send(200, "text/html", head);    //send web page
}
void CALLBACK_handleGetRequest(AsyncWebServerRequest *request) {
  int input = 0;                         //default value if there is no 'Motor' value requested
  int channel = 0;                       //default value if there is no 'Channel' value requested
  if (request->hasParam("Motor")) {
    String inputString = request->getParam("Motor")->value();
    input = inputString.toInt();
    Serial.println("power req recevied: " + String(input));
  }
  if (request->hasParam("Channel")) {
    String inputString = request->getParam("Channel")->value();
    channel = inputString.toInt();
    Serial.println("channel req recevied: " + String(channel));
  }
  FUN_doControl(channel, input);         //request motor control
  CALLBACK_handleRoot(request);
}
//*****************************************************************************************************************
// init wifi and setup OTA
void initWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
  }
  IP = FUN_IpAddress2String(WiFi.localIP());
  Serial.println("Wifi: Connected, IP: " + IP);
  // default setting of OTA is no auth and Port=8266
  // Hostname defaults to esp8266/ESP32-[ChipID]
  //ArduinoOTA.setHostname("");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
  });
  ArduinoOTA.onEnd([]() {
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) { Serial.println("Auth Failed"); } else if (error == OTA_BEGIN_ERROR) { Serial.println("Begin Failed"); } else if (error == OTA_CONNECT_ERROR) { Serial.println("Connect Failed"); }
    else if (error == OTA_RECEIVE_ERROR) { Serial.println("Receive Failed"); } else if (error == OTA_END_ERROR) { Serial.println("End Failed"); };
  });                                      //OTA show error
  ArduinoOTA.begin();                      //OTA start
  //setup callbacks for simple web server and start it
  server.on("/", CALLBACK_handleRoot);
  server.on("/get", HTTP_GET, CALLBACK_handleGetRequest);
  server.onNotFound(CALLBACK_notFound);
  server.begin();
  Serial.println("init OTA done");
}
//*****************************************************************************************************************
// stop motor of given channel
void FUN_doStop(int channel) {
  WatchdogState[channel] = false;          //reset watchdog state
  MotorSpeed[channel] = 0;
  analogWrite(Pin1[channel], 0);           //set both outputs to low level to stop motor
  analogWrite(Pin2[channel], 0);           //set both outputs to low level to stop motor
}
//*****************************************************************************************************************
// power motor of given channel by given value (range -255 .. 0 .. +255)
void FUN_doControl(int channel, int power) {
  if (power == 0) {
    FUN_doStop(channel);
  } else {
    WatchdogState[channel] =              Config_Watchdog;            //enable watchdog to auto stop motor in case of connection fault
    WatchdogTimeout[channel] = millis() + Config_WatchdogTimeoutTime; //calculate next time stamp (when the time is reached without an update (new request) the motor will be stopped
    MotorSpeed[channel] = power;
    if (power > 0) {
      analogWrite(Pin1[channel], power);
      analogWrite(Pin2[channel], 0);
    } else {
      analogWrite(Pin1[channel], 0);
      analogWrite(Pin2[channel], -power);                             //negative value means that the polarity of the motor needs to be shifted (here we switch the pins); negative values needs to be inverted
    }
  }
}
//*****************************************************************************************************************
// watchdog: checks running time of motor after last request, if time is exceeding the motor will be stopped
void loop_Watchdog() {
  for (int i=0; i < MOTOR_MAX; i++){
    if(WatchdogState[i]) {                      
      if (millis() > WatchdogTimeout[i]){
        FUN_doStop(i);                     //stop motor in case of timeout (no new control request was received after given time
      } 
    }
  }
}
//*****************************************************************************************************************
// read analog voltage function; returns voltage in [V]
void loop_AkkuVoltageRead() {  
  if (millis() > NextVoltageRead){
    NextVoltageRead = millis() + Config_VoltageReadInterval;
    int AkkuVoltageRaw = analogRead(PinVoltage);  //read the analog signal
    AkkuVoltage = 100*AkkuVoltageRaw*0.00625;
    //Pin connected to GND and Vin with 10k Ohm resistor each; constant was calulated from measurement of value 660 at 4.13V (means: ~2.06V at Pin level) => factor=4.13/660
  }
}
//*****************************************************************************************************************
// convert power value from BLE device (MSB byte) into; returns power in (range -255 .. 0 .. +255)
int FUN_ConvertPower(int power) {
  if ((power == 0) ||(power & 1)) {
    return power;                          //  signed value means positive (1,3,....,253,255)
                                           //includes 0
  } else {
    return -power-1;                       //unsigned value means negative (2,4,....,252,254)
  }
}
//*****************************************************************************************************************
// BLE callbacks for communication to emulate SBrick to be used in 3rd party Apps like BrickController2
class CALLBACK_BLE_Model: public BLECharacteristicCallbacks {       void onRead(BLECharacteristic *pCharacteristic) {
      uint8_t value[]={0x30,0x30};                                  pCharacteristic->setValue((uint8_t*)&value, 2);    }};
class CALLBACK_BLE_Firmware: public BLECharacteristicCallbacks {    void onRead(BLECharacteristic *pCharacteristic) {
      uint8_t value[]={0x31,0x32,0x2E,0x32,0x35};                   pCharacteristic->setValue((uint8_t*)&value, 5);    }};
class CALLBACK_BLE_Hardware: public BLECharacteristicCallbacks {    void onRead(BLECharacteristic *pCharacteristic) {
      uint8_t value[]={0x31,0x32,0x2E,0x30};                        pCharacteristic->setValue((uint8_t*)&value, 4);    }};
class CALLBACK_BLE_Software: public BLECharacteristicCallbacks {    void onRead(BLECharacteristic *pCharacteristic) {
      uint8_t value[]={0x31,0x32,0x2E,0x32,0x35};                   pCharacteristic->setValue((uint8_t*)&value, 5);    }};
class CALLBACK_BLE_Name: public BLECharacteristicCallbacks {        void onRead(BLECharacteristic *pCharacteristic) {
      uint8_t value[]={0x30};                                       pCharacteristic->setValue((uint8_t*)&value, 1);   }};
//*****************************************************************************************************************
// BLE callback for returning the voltage
class CALLBACK_BLE_RemoteControl: public BLECharacteristicCallbacks { void onWrite(BLECharacteristic *pCharacteristic) {
      uint8_t* value = pCharacteristic->getData();
      Serial.println(String(value[0]));
    }
    void onRead(BLECharacteristic *pCharacteristic) {
      //{0x00,0xFF}    26,75V
      //{0x00,0x7F}    13,32V
      uint8_t VoltageData = AkkuVoltage * 9.7;
      uint8_t value[]={0x00, VoltageData};            //first byte means values of 0..255; last byte means values of (0..255)*256; first byte is not needed due to less accuracy/high tolerance of voltage chain (resistor, ADC, ..)
      pCharacteristic->setValue((uint8_t*)&value, 2);
    }
};
//*****************************************************************************************************************
// BLE callback for reading the QuickDrive characteristic to control the motor(s)
class CALLBACK_BLE_QuickDrive: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    uint8_t* value = pCharacteristic->getData();
    for (int i=0; i < MOTOR_MAX; i++){
      if (sizeof(value) > i){                         //check if mentioned channel was transmitted (SBrick protocol allows that one the first channel(s) are requested to minimize data fransfer rates
        FUN_doControl(i, FUN_ConvertPower(value[i])); //request motor control (SBrick protocol uses special data format of MSB which we need to convert first!!!)
      }
    }
  }
};
//*****************************************************************************************************************
// init BLE
void initBLE() {
  BLEDevice::init("SBrick");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService_DeviceInformation = pServer->createService(SERVICE_UUID_DEVICE_INFORMATION);
          BLECharacteristic *pCharacteristic_Model = pService_DeviceInformation->createCharacteristic(
                                         CHARACTERISTIC_UUID_MODEL,
                                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                                       );
                  pCharacteristic_Model->setCallbacks(new CALLBACK_BLE_Model());
          BLECharacteristic *pCharacteristic_Firmware = pService_DeviceInformation->createCharacteristic(
                                         CHARACTERISTIC_UUID_FIRMWARE_REVISION,
                                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                                       );
                  pCharacteristic_Firmware->setCallbacks(new CALLBACK_BLE_Firmware());
          BLECharacteristic *pCharacteristic_Hardware = pService_DeviceInformation->createCharacteristic(
                                         CHARACTERISTIC_UUID_HARDWARE_REVISION,
                                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                                       );
                  pCharacteristic_Hardware->setCallbacks(new CALLBACK_BLE_Hardware());
          BLECharacteristic *pCharacteristic_Software = pService_DeviceInformation->createCharacteristic(
                                         CHARACTERISTIC_UUID_SOFTWARE_REVISION,
                                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                                       );
                  pCharacteristic_Software->setCallbacks(new CALLBACK_BLE_Software());
          BLECharacteristic *pCharacteristic_Name = pService_DeviceInformation->createCharacteristic(
                                         CHARACTERISTIC_UUID_NAME,
                                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                                       );
                  pCharacteristic_Name->setCallbacks(new CALLBACK_BLE_Name());
          pService_DeviceInformation->start();
  BLEService *pService_RemoteControl = pServer->createService(SERVICE_UUID_REMOTE_CONTROL);
          BLECharacteristic *pCharacteristic_RemoteControl = pService_RemoteControl->createCharacteristic(
                                         CHARACTERISTIC_UUID_REMOTE_CONTROL,
                                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
                                       );
                  pCharacteristic_RemoteControl->setCallbacks(new CALLBACK_BLE_RemoteControl());
          BLECharacteristic *pCharacteristic_QuickDrive = pService_RemoteControl->createCharacteristic(
                                         CHARACTERISTIC_UUID_QUICK_DRIVE,
                                         BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
                                       );
                  pCharacteristic_QuickDrive->setCallbacks(new CALLBACK_BLE_QuickDrive());
          pService_RemoteControl->start();
  BLEAdvertisementData advert;
          advert.setName("SBrick");
          char value[]={0x98,0x01,0x06};   //BLE standard prefix sequence + length of name (ANYWAY the SBrick app will connect but it will not work !!!
          advert.setManufacturerData(value);
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
          pAdvertising->setAdvertisementData(advert);
          pAdvertising->start();
  Serial.println("init BLE done");
}
//*****************************************************************************************************************
// init MotorDriverPWM
void initMotorDriver() {
  analogWriteFrequency(19500);             //set PWM frequency to 19.5kHz to reduce motor noise
  analogWriteResolution(8);                //set ADC resolution to 8 bits
  for (int i=0; i < MOTOR_MAX; i++){
    pinMode(Pin1[i], OUTPUT);              //define ADC pins as outputs
    pinMode(Pin2[i], OUTPUT);              //define ADC pins as outputs
  }
  for (int i=0; i < MOTOR_MAX; i++){
    FUN_doStop(i);                         //execute stop command for all motors
  }
  Serial.println("init Motor driver done");
}
//*****************************************************************************************************************
// arduino IDE standard calls
void setup() {
  Serial.begin(115200);                    //enable serial interface with 115200 baud
  initMotorDriver();                       //init Motor driver
  initBLE();                               //init BLE
  initWifi();                              //init Wifi and OTA (after one time flashing via USB PORT, next time the ESP32 can be flashed via WLAN
}
void loop() {
  ArduinoOTA.handle();                     //cyclic check OTA requests to allow OTA
  loop_Watchdog();                         //cyclic execute watchdog
  loop_AkkuVoltageRead();                  //cyclic read voltage
}
//*****************************************************************************************************************
// helper function to convert IP address in string
String FUN_IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") + String(ipAddress[1]) + String(".") + String(ipAddress[2]) + String(".") + String(ipAddress[3]);
}
