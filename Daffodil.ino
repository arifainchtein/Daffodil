  // New Version
#include <PowerManager.h>
#include <SolarInfo.h>

#include <NewPing.h>
#include "Arduino.h"
#include <Timer.h>
#include <PCF8563TimeManager.h>
#include <SPI.h>
#include <LoRa.h>
#include <Esp32SecretManager.h>
#include <FastLED.h>
#include <DaffodilWifiManager.h>
#include <DaffodilData.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include <Wire.h>
#include <sha1.h>
#include <totp.h>
#include <SolarPowerData.h>
#include <DigitalStablesData.h>
#include <WeatherForecastManager.h>

#include "ADS1X15.h"

#include <FastLED.h>
#include "CHT8305.h"
String currentSSID;
String ipAddress = "";
boolean initiatedWifi=false;
// #define address 0x40
#define LED_PIN 19
#define OP_MODE 34
#define NUM_LEDS 15
#define LED_CONTROL 23
int brightness1=100;

CRGB leds[NUM_LEDS];
#define WATCHDOG_WDI 18
//const int trigPin = 33;
//const int echoPin = 32;
#define TRIGGER_PIN  33  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     32  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 90 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

#define SCK 14
#define MOSI 13
#define MISO 12
#define LoRa_SS 15
#define LORA_RESET 16
#define LORA_DI0 17
CHT8305 CHT(0x40);

#define SENSOR_INPUT_2 33
#define SENSOR_INPUT_1 32

TaskHandle_t watchdogTask;

ADS1115 ADS(0x48);
volatile bool RDY = false;
uint8_t ads1115channel = 0;
int16_t ads1115val[4] = {0, 0, 0, 0};
#define ADS115_ALERT 35

#define TEMPERATURE 27
#define RTC_BATT_VOLT 36
#define RTC_CLK_OUT 4
OneWire oneWire(TEMPERATURE);
DallasTemperature tempSensor(&oneWire);

Timer viewTimer(5);
bool showTime = false;

//bool internetAvailable;
bool wifiActiveSwitch;
#define uS_TO_S_FACTOR 60000000 /* Conversion factor for micro seconds to minutes */
float operatingStatus = 3;
Timer dsUploadTimer(60);
static volatile int flowMeterPulseCount;
static volatile int flowMeterPulseCount2;

#define FUN_1_FLOW 1
#define FUN_2_FLOW 2
#define FUN_1_FLOW_1_TANK 3
#define FUN_1_TANK 4
#define FUN_2_TANK 5
#define DAFFODIL_SCEPTIC_TANK 6
#define DAFFODIL_WATER_TROUGH 7
#define DAFFODILE_TEMP_SOILMOISTURE 8
#define DAFFODILE_LIGHT_DETECTOR 9


uint8_t displayStatus=0;
#define SHOW_TEMPERATURE 0
#define SHOW_SCEPTIC 1
#define SHOW_INTERNET_STATUS 2
#define SEND_LORA_STATUS 3          
//
// sleeping parameters
//
float sleepingVoltage = 3.5;
float wakingUpVoltage = 3.7;
uint8_t numberSecondsWithMinimumWifiVoltageForStartWifi=30;
uint8_t currentSecondsWithWifiVoltage=0;
float minimumInitWifiVoltage=4.5;
float minimumLoraVoltage=4.2;
uint8_t sleepingTime = 1;
float minimumLEDVoltage= 4.0;
#define MAXIMUM_LED_VOLTAGE 4658
float minimumWifiVoltage=4.25;

uint8_t secondsSinceLastDataSampling = 0;
uint8_t delayTime = 10;
#define UNIQUE_ID_SIZE 8
bool loraActive = false;
DigitalStablesConfigData digitalStablesConfigData;
DigitalStablesData digitalStablesData;
DaffodilCommandData daffodilCommandData;

PCF8563TimeManager timeManager(Serial);
GeneralFunctions generalFunctions;
Esp32SecretManager secretManager(timeManager);
SolarInfo* solarInfo;
PowerManager* powerManager;
WeatherForecastManager* weatherForecastManager; 

DaffodilWifiManager wifiManager(Serial, timeManager, secretManager, digitalStablesData, digitalStablesConfigData);

int badPacketCount = 0;
byte msgCount = 0;        // count of outgoing messages
byte localAddress = 0xFF; // address of this device
byte destination = 0xAA;  // destination to send to

long lastPulseTime = 0;
uint8_t uniqueId[UNIQUE_ID_SIZE];
long lastMillis;
uint8_t SECONDOFFSET = 10;
uint8_t timeZoneHours = 10;
static byte monthDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
String currentIpAddress = "No IP";
bool inPulse = true;

long lastTimeUpdateMillis = 0;
RTCInfoRecord currentTimerRecord;
#define TIME_RECORD_REFRESH_SECONDS 3

volatile bool clockTicked = false;
volatile bool lowVoltageAlert = false;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
RTCInfoRecord lastReceptionRTCInfoRecord;

const int SHARED_SECRET_LENGTH = 27;
char secretCode[SHARED_SECRET_LENGTH];
bool opmode = false;

#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the lcd address to 0x27 for a 16 chars and 2 line display

String serialNumber;
struct TempHum{
float temp=-99;
float hum=-99;
  
} tempHum;

const float R1 = 1000000.0; // Resistance of R1 in ohms (1 MΩ)
const float R2 = 2000000.0; // Resistance of R2 in ohms (2 MΩ)
const float Vref = 3.3; // Reference voltage of the ESP32

int view_milliseconds = 5000;
long lastswitchmillis = 0;
boolean showTemperature = false;
uint8_t color = 0;

const char *display1URL = "http://Ra.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Ra:Purpose:Sensor%20Data:Now:Battery%20Voltage";
void IRAM_ATTR pulseCounter()
{
  flowMeterPulseCount++;
}

void IRAM_ATTR pulseCounter2()
{
  flowMeterPulseCount2++;
}

void IRAM_ATTR clockTick()
{
  portENTER_CRITICAL_ISR(&mux);
  clockTicked = true;
  portEXIT_CRITICAL_ISR(&mux);
}

//
// Lora Functions
//
void onReceive(int packetSize) {
  if (packetSize == 0) return;  // if there's no packet, return
  if (packetSize == sizeof(PanchoCommandData)) {
    LoRa.readBytes((uint8_t*)&daffodilCommandData, sizeof(PanchoCommandData));
    long commandcode = daffodilCommandData.commandcode;
    bool validCode = secretManager.checkCode(commandcode);
    if (validCode) {

      //secretManager.saveSleepPingMinutes(rosieConfigData.sleepPingMinutes);
      //secretManager.saveConfigData(rosieConfigData.fieldId,  stationName );

      int rssi = LoRa.packetRssi();
      float Snr = LoRa.packetSnr();
      Serial.println(" Receive DaffodilCommandData: ");
      Serial.print(" Field Id: ");
      Serial.print(daffodilCommandData.fieldId);
      Serial.print(" commandcode: ");
      Serial.print(daffodilCommandData.commandcode);

    } else {
      Serial.print(" Receive PanchoCommandData but invalid code: ");
      Serial.println(commandcode);
      Serial.print(daffodilCommandData.fieldId);
    }
  } else {
    badPacketCount++;
    Serial.print("Received  invalid data daffodilCommandData data, expected: ");
    Serial.print(sizeof(PanchoCommandData));
    Serial.print("  received");
    Serial.println(packetSize);
  }
}
void LoRa_txMode() {
  LoRa.idle();             // set standby mode
  LoRa.disableInvertIQ();  // normal mode
}
void LoRa_rxMode() {
  LoRa.disableInvertIQ();  // normal mode
  LoRa.receive();          // set receive mode
}
void sendMessage() {
  LoRa_txMode();
  LoRa.beginPacket();  // start packet
  Serial.print("sending lora sn=");
  Serial.println(serialNumber);
   String secret = "J5KFCNCPIRCTGT2UJUZFSMQK";
    TOTP totp = TOTP(secret.c_str());
    char totpCode[7]; // get 6 char code
    long timeVal = timeManager.getTimeForCodeGeneration(currentTimerRecord);
    digitalStablesData.secondsTime = timeVal;
    long code = totp.gen_code(timeVal);
    Serial.print("line 241 sending lora totp=");
    Serial.println(code);
                   
  digitalStablesData.totpcode=code;
  LoRa.write((uint8_t*)&digitalStablesData, sizeof(DaffodilData));
  LoRa.endPacket(true);  // finish packet and send it
  LoRa_rxMode();
  msgCount++;        // increment message ID
}

//
// End of Lora Functions
//
void setup()
{
  Serial.begin(115200); 
  Wire.begin();
  Wire.setClock(400000);
  
  //
// data frm cofigurartion
//
String timezone = "AEST-10AEDT,M10.1.0,M4.1.0/3";
  double latitude = -37.13305556;
  double longitude = 144.47472222;
  double altitude=410.0;
  float capacitorValue=3.0;
  float currentPerLed=.020;
  const char* apiKey = "103df7bb3e4010e033d494f031b483e0";


  setenv("TZ", timezone.c_str(), 1);
  tzset();

  Serial.print("sizeof DigitalStablesData=");
  Serial.println(sizeof(DigitalStablesData));
  
   pinMode(RTC_CLK_OUT, INPUT_PULLUP); // set up interrupt pin
  digitalWrite(RTC_CLK_OUT, HIGH);    // turn on pullup resistors
  // attach interrupt to set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);
  
  
  

  timeManager.start(timezone.c_str());
  timeManager.PCF8563osc1Hz();
  currentTimerRecord = timeManager.now();

//  const char* ntpServer = "pool.ntp.org";
//  const long gmtOffset_sec = 36000;  // Melbourne is UTC+10
//  const int daylightOffset_sec = 3600; // 1 hour during daylight savings

  solarInfo = new SolarInfo(Serial,latitude, longitude, altitude); 
  weatherForecastManager =new WeatherForecastManager(currentTimerRecord ,  latitude,  longitude, apiKey);
  weatherForecastManager->loadForecasts(Serial);
Serial.print("hasValidForecasts=");
Serial.println(weatherForecastManager->hasValidForecasts());



  if (weatherForecastManager->hasValidForecasts()) {
      WeatherForecast* forecasts = weatherForecastManager->getForecasts();
       for (int i = 0; i < 8; i++) {
                
                 Serial.print("line 312 downloadWeatherData secondsTime=");
                Serial.print(forecasts[i].secondsTime);
                Serial.print("  temperature=");
                 Serial.println(forecasts[i].temperature);
                 Serial.print("  cloudiness=");
                Serial.println(forecasts[i].cloudiness);
       }
  } else {
      // Handle case where no forecasts are available
      Serial.println("No forecast data available, dowmnloading..");
      boolean downloadOk=weatherForecastManager->downloadWeatherData(solarInfo, Serial) ;
      Serial.print("downloadWeatherData returnxs=");
    Serial.println(downloadOk);
    if(downloadOk){
       WeatherForecast* forecasts = weatherForecastManager->getForecasts() ;
       for (int i = 0; i < 8; i++) {
                forecasts[i] = WeatherForecast();
                 Serial.print("downloadWeatherData cloudiness=");
                Serial.println(forecasts[i].cloudiness);
       }
    }

  }

  
  powerManager = new PowerManager(Serial, ADS,*solarInfo, latitude, longitude,  capacitorValue, currentPerLed);
  DailySolarData dailySolarData = solarInfo->getDailySolarData(currentTimerRecord);
  
  Serial.print("sunrise=");
  Serial.println(dailySolarData.sunrise);
   Serial.print("sunset=");
  Serial.println(dailySolarData.sunset);

  Serial.print("sunrisetime=");
  Serial.println(dailySolarData.sunrisetime);
   Serial.print("sunsettime=");
  Serial.println(dailySolarData.sunsettime);
  
  HourlySolarPowerData hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);
  Serial.print("efficiency=");
  Serial.println(hourlySolarPowerData.efficiency);
   Serial.print("actualPower=");
  Serial.println(hourlySolarPowerData.actualPower);
   Serial.print("irradiance=");
  Serial.println(hourlySolarPowerData.irradiance);
   Serial.print("temperature=");
  Serial.println(hourlySolarPowerData.temperature);
  
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  
      pinMode(LED_CONTROL, OUTPUT); 
  digitalWrite(LED_CONTROL, HIGH);  
  
//  lcd.init();
//  lcd.backlight();
//  lcd.clear();
//   lcd.setCursor(0, 0);
  CHT.begin();
  ADS.begin();
  if (!ADS.begin())
  {
    Serial.println("invalid address ADS1115 or 0x48 not found");
  }
  else
  {
    Serial.println("found ADS1115");
  }

  Serial.println(CHT.getManufacturer(), HEX);
  Serial.println(CHT.getVersionID(), HEX);
  Serial.println(CHT.getVoltage());

  

  ADS.setGain(0);
  //
  // Capacitor Voltage
  int16_t val_3 = ADS.readADC(3);
  float f = ADS.toVoltage(1); //  voltage factor
  digitalStablesData.capacitorVoltage = val_3 * f;
  lcd.setCursor(0, 0);
  lcd.print("cap=");
  lcd.print(digitalStablesData.capacitorVoltage);
  
  
  //
  if (digitalStablesData.capacitorVoltage>1 && digitalStablesData.capacitorVoltage < wakingUpVoltage)
  {
    Serial.print("Voltage is ");
    Serial.print(digitalStablesData.capacitorVoltage);
    Serial.println(", going to sleep...");
     lcd.print("Volt low ");
    lcd.print(digitalStablesData.capacitorVoltage);
    lcd.println(", sleeping...");
    
      
      
    FastLED.setBrightness(5);
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
     digitalWrite(LED_CONTROL, LOW);
    leds[0] = CRGB(255, 255, 0);
    FastLED.show();
    delay(500);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
    // Convert sleepingTime from minutes to microseconds for deep sleep
    esp_sleep_enable_timer_wakeup(sleepingTime * 60 * 1000000);
    esp_deep_sleep_start();
  }

 

 
  tempSensor.begin();
  // uint8_t address[8];
  tempSensor.getAddress(digitalStablesData.serialnumberarray, 0);
  for (uint8_t i = 0; i < 8; i++)
  {
    serialNumber += String(digitalStablesData.serialnumberarray[i], HEX);
    digitalStablesData.checksum += static_cast<uint8_t>(digitalStablesData.serialnumberarray[i]);
  }
  digitalStablesData.checksum &= 0xFF;
  Serial.print("serial number:");
  Serial.println(serialNumber);

 SPI.begin(SCK, MISO, MOSI);
  pinMode(LoRa_SS, OUTPUT);
  pinMode(LORA_RESET, OUTPUT);
  pinMode(LORA_DI0, INPUT);
  digitalWrite(LoRa_SS, HIGH);
  LoRa.setPins(LoRa_SS, LORA_RESET, LORA_DI0);
  // for (int i = 0; i < NUM_LEDS; i++) {
  //   leds[i] = CRGB(0, 0, 0);
  // }
  // FastLED.show();
 if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    drawLora(false);
    while (1);
    leds[1] = CRGB(255, 0, 0);
  } else {
    Serial.println("Starting LoRa worked!");
   drawLora(true);
    loraActive = true;
  }

 // delay(2000);

for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show(); 
  ADS.setGain(1);
  // Config Switch
  Serial.print("voltage factor=");
  Serial.println(f);
  float val2 = ADS.readADC(2);
  Serial.print("val2=");
  Serial.println(val2);

  lcd.setCursor(0, 1);
  lcd.print("csw=");
  lcd.print(val2);
 
  
  int16_t cswOutput = val2;
  Serial.print("cswOutput=");
  Serial.println(cswOutput);

  if(digitalStablesData.capacitorVoltage>0 ){
    float factor = 5/digitalStablesData.capacitorVoltage;
    cswOutput = cswOutput*factor;
  }
Serial.print("corrected cswOutput=");
  Serial.println(cswOutput);
   lcd.setCursor(0, 2);
  lcd.print(" corr csw=");
  lcd.print(cswOutput);
  //
  // dip switch values
  // 1234
  // 0000 = 7279  F
  // 1000 = 6889  FF
  // 0100 = 6475  FT
  // 1100 = 6062  T
  // 0010 = 5663  TT
  // 1010 = 5212  Daffodil Sceptic
  // 0110 > 4774 Daffodil Water Trough
  // 1110 > 4394 DAFFODILE_TEMP_SOILMOISTURE
  // 0001 > 3471
  // 1001 = 3198
  // 1101 = 2181
  // 1111 = 0
  if (cswOutput >= 7100)
  {
    digitalStablesData.currentFunctionValue = FUN_1_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
  }
  else if (cswOutput>=6700 && cswOutput< 7100 )
  {
    digitalStablesData.currentFunctionValue = FUN_2_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readFlow2Name().toCharArray(digitalStablesData.sensor2name, 12);
  }
  else if (cswOutput>=6300 && cswOutput< 6700)
  {
    digitalStablesData.currentFunctionValue = FUN_1_FLOW_1_TANK;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);

    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
  }
  else if (cswOutput>=5800 && cswOutput<6300)
  {
    digitalStablesData.currentFunctionValue = FUN_1_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
  }
  else if (cswOutput>=5500 && cswOutput<5800)
  {
    digitalStablesData.currentFunctionValue = FUN_2_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
  }else if (cswOutput>=5100 && cswOutput<5500){
    digitalStablesData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;
  }else if (cswOutput>=4600 && cswOutput<5100){
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
  }else if (cswOutput>=4300 && cswOutput<4600){
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
  }else{
     digitalStablesData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;
      lcd.setCursor(0, 3);
      lcd.print("bad data sceptic is def");
  }
 uint8_t brightness = map(digitalStablesData.capacitorVoltage*1000, minimumLEDVoltage*1000, MAXIMUM_LED_VOLTAGE, 1, 125);   // Multiply voltage by 1000 for integer mapping
 brightness = constrain(brightness, 1, 125);
 for (int i = 0; i < 5; i++){
      leds[i] = CRGB(0, 0, 0);
  }
    FastLED.show();
 // digitalWrite(LED_CONTROL, LOW);   
  for (int i = 0; i < digitalStablesData.currentFunctionValue; i++){
    leds[i] = CRGB(255, 255, 0);
  }
  FastLED.show();
  delay(2000);
  operatingStatus = secretManager.getOperatingStatus();
  float pingMinutes = secretManager.getSleepPingMinutes();
  esp_sleep_enable_timer_wakeup(pingMinutes * uS_TO_S_FACTOR);
  digitalStablesData.operatingStatus = operatingStatus;

  String devicename = secretManager.readDeviceName();
  char devicenamearray[devicename.length()];
  devicename.toCharArray(devicenamearray, devicename.length());
  strcpy(digitalStablesData.devicename, devicenamearray);

  String deviceshortname = secretManager.readDeviceShortName();
  char deviceshortnamearray[deviceshortname.length()];
  deviceshortname.toCharArray(deviceshortnamearray, deviceshortname.length());
  strcpy(digitalStablesData.deviceshortname, deviceshortnamearray);


 
  digitalStablesConfigData.sleepPingMinutes = pingMinutes;
  digitalStablesConfigData.operatingStatus = operatingStatus;
  //String grp = secretManager.getGroupIdentifier();
  //char gprid[grp.length()];
  //grp.toCharArray(gprid, grp.length());
  //strcpy(digitalStablesData.groupidentifier, gprid);

  //Serial.print("Starting wifi digitalStablesConfigData.groupidentifier=");
  //Serial.println(digitalStablesData.groupidentifier);

  String identifier = "daffodilTF";
  char ty[identifier.length() + 1];
  identifier.toCharArray(ty, identifier.length() + 1);
  strcpy(digitalStablesData.deviceTypeId, ty);

  digitalStablesConfigData.fieldId = secretManager.getFieldId();
  Serial.print("Starting wifi digitalStablesConfigData.fieldId=");
  Serial.println(digitalStablesConfigData.fieldId);

 // wifiManager.start();

//  if(digitalStablesData.capacitorVoltage>=minimumWifiVoltage){
//    startWifi(); 
//  }
  

  pinMode(RTC_BATT_VOLT, INPUT);
  opmode = digitalRead(OP_MODE);
  digitalStablesData.opMode = opmode;

  dsUploadTimer.start();
  digitalStablesData.dataSamplingSec = 10;
  Serial.print("digitalStablesData.dataSamplingSec=");
  Serial.println(digitalStablesData.dataSamplingSec);
  Serial.print("Daffodil size=");
  Serial.println(sizeof(digitalStablesData));

  Serial.println("Scanning for I2C devices ...");
  bool foundtemp = false;
  byte error, address;
  int nDevices = 0;

  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      if (address == 40)
        foundtemp = true;
      nDevices++;
    }
    else if (error != 2)
    {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }

  if (!foundtemp)
  {
    for (int i = NUM_LEDS - 5; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
  }
//  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
//  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  lastswitchmillis = millis();

  viewTimer.start();
 LoRa.setSyncWord(0xF3);
  Serial.println(F("Finished Setup"));
   xTaskCreatePinnedToCore(
                    watchdogTaskFunction,   /* Task function. */
                    "watchdogTask",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &watchdogTask,      /* Task handle to keep track of created task */
                    0);      
  pinMode(WATCHDOG_WDI, OUTPUT);
  digitalWrite(WATCHDOG_WDI, LOW);
}

void watchdogTaskFunction( void * pvParameters ){
  Serial.print("watchdogTask running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
    digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
    delay(900);
  } 
}

void restartWifi(){
   FastLED.setBrightness(50);
  for (int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB(0, 0, 0);
    }
    leds[1] = CRGB(255, 0, 255);
    leds[2] = CRGB(255, 0, 255);
    leds[3] = CRGB(255, 0, 255);
    leds[5] = CRGB(255, 0, 255);
    leds[9] = CRGB(255, 0, 255);
    leds[11] = CRGB(255, 0, 255);
    leds[12] = CRGB(255, 0, 255);
    leds[13] = CRGB(255, 0, 255);
    FastLED.show();
  if(!initiatedWifi){
    digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
      leds[7] = CRGB(255, 0, 255);
    FastLED.show();
    Serial.print(F("Before Starting Wifi cap="));
    Serial.println(digitalStablesData.capacitorVoltage);
    wifiManager.start(); 
     digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
    Serial.print(F("After Starting Wifi cap="));
    Serial.println(digitalStablesData.capacitorVoltage);
    initiatedWifi=true;
  }
    Serial.println("Starting wifi");

    digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
    
    
        
    wifiManager.restartWifi();
      digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
     Serial.println("getting  stationmode=");
    bool stationmode = wifiManager.getStationMode();
    digitalStablesData.internetAvailable=wifiManager.getInternetAvailable();
     digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
    Serial.print("Starting wifi stationmode=");
    Serial.println(stationmode);
   Serial.print("digitalStablesData.internetAvailable=");
    Serial.println(digitalStablesData.internetAvailable);
  
    //  serialNumber = wifiManager.getMacAddress();
    wifiManager.setSerialNumber(serialNumber);
    wifiManager.setLora(loraActive);
    String ssid = wifiManager.getSSID();
    String ipAddress = "";
    uint8_t ipi;
    if (stationmode){
      ipAddress = wifiManager.getIpAddress();
      Serial.print("ipaddress=");
      Serial.println(ipAddress);
  
      if (ipAddress == "" || ipAddress == "0.0.0.0")
      {
        
        setApMode();
      }
      else
      {
        setStationMode(ipAddress);
      }
    }else{
      setApMode();
    }
    digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
 
    
    digitalStablesData.loraActive = loraActive;
    uint8_t ipl = ipAddress.length() + 1;
    char ipa[ipl];
    ipAddress.toCharArray(ipa, ipl);
   for (int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB(0, 0, 0);
    }
  FastLED.show();
      digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
    Serial.println("in ino Done starting wifi");
}

void drawLora( boolean active)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  if(active){
    leds[1] = CRGB(0, 255, 0);
    leds[6] = CRGB(0, 255, 0);
    leds[11] = CRGB(0, 255, 0);
    leds[12] = CRGB(0, 255, 0);
   // leds[13] = CRGB(0, 255, 0);
  }else{
    leds[1] = CRGB( 255,0, 0);
    leds[6] = CRGB( 255, 0, 0);
    leds[11] = CRGB( 255,0, 0);
    leds[12] = CRGB( 255,0, 0);
   // leds[13] = CRGB( 255,0, 0);
  }
  FastLED.show();

}

void drawTemperature( uint8_t red, uint8_t green, uint8_t blue)
{

  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  uint8_t ld = 0;
  uint8_t hd = 0;

  if (digitalStablesData.outdoortemperature > 0 && digitalStablesData.outdoortemperature < 10)
  {
    ld = digitalStablesData.outdoortemperature;
  }
  else if (digitalStablesData.outdoortemperature >= 10 && digitalStablesData.outdoortemperature < 20)
  {
    ld = digitalStablesData.outdoortemperature - 10;
    hd = 1;
  }
  else if (digitalStablesData.outdoortemperature >= 20 && digitalStablesData.outdoortemperature < 30)
  {
    ld = digitalStablesData.outdoortemperature - 20;
    hd = 2;
  }
  else if (digitalStablesData.outdoortemperature >= 30 && digitalStablesData.outdoortemperature < 40)
  {
    ld = digitalStablesData.outdoortemperature - 30;
    hd = 3;
  }
  else if (digitalStablesData.outdoortemperature >= 40 && digitalStablesData.outdoortemperature < 50)
  {
    ld = digitalStablesData.outdoortemperature - 40;
    hd = 4;
  }
  Serial.print("hd=");
  Serial.print(hd);
  switch (hd)
  {
  case 0:
    break;
  case 1:
    leds[0] = CRGB(red, green, blue);
    break;
  case 2:
    leds[0] = CRGB(red, green, blue);
    leds[5] = CRGB(red, green, blue);
    break;
  case 3:
    leds[0] = CRGB(red, green, blue);
    leds[5] = CRGB(red, green, blue);
    leds[10] = CRGB(red, green, blue);
    break;
  case 4:
    leds[0] = CRGB(red, green, blue);
    leds[1] = CRGB(red, green, blue);
    leds[5] = CRGB(red, green, blue);
    leds[10] = CRGB(red, green, blue);
    break;
  default:
    break;
  }
  Serial.print("ld=");
  Serial.println(ld);

  switch (ld)
  {
  case 0:
    break;
  case 1:
    leds[4] = CRGB(red, green, blue);
    break;
  case 2:
    leds[4] = CRGB(red, green, blue);
    leds[9] = CRGB(red, green, blue);
    break;
  case 3:
    leds[4] = CRGB(red, green, blue);
    leds[9] = CRGB(red, green, blue);
    leds[14] = CRGB(red, green, blue);
    break;
  case 4:
    leds[4] = CRGB(red, green, blue);
    leds[9] = CRGB(red, green, blue);
    leds[14] = CRGB(red, green, blue);
    leds[3] = CRGB(red, green, blue);
    break;
  case 5:
    leds[4] = CRGB(red, green, blue);
    leds[9] = CRGB(red, green, blue);
    leds[14] = CRGB(red, green, blue);
    leds[3] = CRGB(red, green, blue);
    leds[8] = CRGB(red, green, blue);
    break;
  case 6:
    leds[4] = CRGB(red, green, blue);
    leds[9] = CRGB(red, green, blue);
    leds[14] = CRGB(red, green, blue);
    leds[3] = CRGB(red, green, blue);
    leds[8] = CRGB(red, green, blue);
    leds[13] = CRGB(red, green, blue);
    break;
  case 7:
    leds[4] = CRGB(red, green, blue);
    leds[9] = CRGB(red, green, blue);
    leds[14] = CRGB(red, green, blue);
    leds[3] = CRGB(red, green, blue);
    leds[8] = CRGB(red, green, blue);
    leds[13] = CRGB(red, green, blue);
    leds[2] = CRGB(red, green, blue);
    break;
    leds[2] = CRGB(red, green, blue);
    leds[7] = CRGB(red, green, blue);
    leds[12] = CRGB(red, green, blue);
  case 8:
    leds[4] = CRGB(red, green, blue);
    leds[9] = CRGB(red, green, blue);
    leds[14] = CRGB(red, green, blue);
    leds[3] = CRGB(red, green, blue);
    leds[8] = CRGB(red, green, blue);
    leds[13] = CRGB(red, green, blue);
    leds[2] = CRGB(red, green, blue);
    leds[7] = CRGB(red, green, blue);
    break;
  case 9:
    leds[4] = CRGB(red, green, blue);
    leds[9] = CRGB(red, green, blue);
    leds[14] = CRGB(red, green, blue);
    leds[3] = CRGB(red, green, blue);
    leds[8] = CRGB(red, green, blue);
    leds[13] = CRGB(red, green, blue);
    leds[2] = CRGB(red, green, blue);
    leds[7] = CRGB(red, green, blue);
    leds[12] = CRGB(red, green, blue);
    break;

  default:
    // statements
    break;
  }

  FastLED.show();
}
void readI2CTemp()
{
  float temperature = 0;

  //  READ DATA
  uint32_t start = micros();
  int status = CHT.read();
  uint32_t stop = micros();

  Serial.print("CHT8305\t");
  //  DISPLAY DATA, sensor has only one decimal.
  Serial.print(F("Humidity"));
 
 
  temperature = CHT.getTemperature();
  
  digitalStablesData.outdoortemperature=temperature;
   Serial.print("\t\t Temp:");
    Serial.println(temperature, 1);
  digitalStablesData.outdoorhumidity=CHT.getHumidity();
  Serial.print("\t\t Hum:");
   Serial.print(digitalStablesData.outdoorhumidity, 1);
 
  switch (status)
  {
  case CHT8305_OK:
    Serial.print("OK");
    break;
  case CHT8305_ERROR_ADDR:
    Serial.print("Address error");
    break;
  case CHT8305_ERROR_I2C:
    Serial.print("I2C error");
     digitalStablesData.outdoortemperature=-99;
    break;
  case CHT8305_ERROR_CONNECT:
    Serial.print("Connect error");
    break;
  case CHT8305_ERROR_LASTREAD:
    Serial.print("Last read error");
    break;
  default:
    Serial.print("Unknown error");
    break;
  }
  Serial.print("\n");

  
}

void loop()
{
uint16_t dscount;
  if (clockTicked)
  {

    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
     currentTimerRecord = timeManager.now();
    if (currentTimerRecord.second == 0){
      if (currentTimerRecord.minute == 0){
        //  Serial.println(F("New Hour"));
        if (currentTimerRecord.hour == 0){
          //    Serial.println(F("New Day"));
        }
      }
    }
   // digitalWrite(WATCHDOG_WDI, HIGH);
 //   delay(1);
  //  digitalWrite(WATCHDOG_WDI, LOW);
    secondsSinceLastDataSampling++;
  //   Serial.println("ticked");

    viewTimer.tick();

 //
  // read the height

  ADS.setGain(0);
  //
  // Capacitor Voltage
  int16_t val_3 = ADS.readADC(3);
  float f = ADS.toVoltage(1); //  voltage factor
  digitalStablesData.capacitorVoltage = val_3 * f;
   lcd.setCursor(0, 3);
  
   
   if(digitalStablesData.capacitorVoltage> minimumInitWifiVoltage && !wifiManager.getWifiStatus()){
    currentSecondsWithWifiVoltage++;
   }else{
    currentSecondsWithWifiVoltage=0;
   }
   
   tempSensor.requestTemperatures();
    float tempC = tempSensor.getTempCByIndex(0);
    digitalStablesData.temperature=tempC;

 //
    //RTC_BATT_VOLT Voltage
    //
    float total = 0;
    uint8_t samples=10;
    for (int x = 0; x < samples; x++) {           // multiple analogue readings for averaging
      total = total + analogRead(RTC_BATT_VOLT);  // add each value to a total
    }
    float average = total / samples;
    float voltage = (average / 4095.0) * Vref;
    // Calculate the actual voltage using the voltage divider formula
    float rtcBatVoltage = (voltage * (R1 + R2)) / R2;
    digitalStablesData.rtcBatVolt = rtcBatVoltage;
 
 // Serial.print("loop cap ");
 //  Serial.print(digitalStablesData.capacitorVoltage);
//  Serial.print(" currentssec=");
//   Serial.println(currentSecondsWithWifiVoltage);
// Serial.print("wifiManager.getAPStatus()=");
//      Serial.print(wifiManager.getAPStatus());
//      Serial.print(" wifiManager.getWifiStatus()=");
//      Serial.println(wifiManager.getWifiStatus());
//       Serial.print(" internetavaialbnle=");
//      Serial.println(digitalStablesData.internetAvailable);
     dscount = dsUploadTimer.tick();
//     Serial.print("dsupload timer counter= ");
//     Serial.println(dscount);
                    
   
  }
 digitalStablesData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
digitalStablesData.sleepTime= powerManager->calculateOptimalSleepTime(currentTimerRecord);
  //
  // check to see if we need to go to sleep
  //
  if (digitalStablesData.capacitorVoltage>1 && digitalStablesData.capacitorVoltage < sleepingVoltage){
    Serial.print("Voltage is ");
    Serial.print(digitalStablesData.capacitorVoltage);
    Serial.println(", going to sleep...");
    for (int i = 0; i < 5; i++)
    {
      leds[i] = CRGB(255, 255, 0);
    }
    FastLED.show();
    delay(100);
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
     digitalWrite(LED_CONTROL, LOW);   
    // Convert sleepingTime from minutes to microseconds for deep sleep
    esp_sleep_enable_timer_wakeup(sleepingTime * 60 * 1000000);
    esp_deep_sleep_start();
  }

   if(digitalStablesData.capacitorVoltage>=minimumLEDVoltage){
     digitalWrite(LED_CONTROL, HIGH);   
   }else{
     
     Serial.print("line 967 turning off, cap=");
     Serial.println(digitalStablesData.capacitorVoltage);
     for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
     digitalWrite(LED_CONTROL, LOW);
   }
  if(digitalStablesData.capacitorVoltage< minimumWifiVoltage){
    Serial.print("turning off wifi cap voltage=");
    Serial.println(digitalStablesData.capacitorVoltage);
     wifiManager.stop();
     digitalStablesData.internetAvailable=false;
      currentSecondsWithWifiVoltage=0;
      for (int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB(0, 0, 0);
    }
    leds[1] = CRGB(255, 0, 0);
    leds[2] = CRGB(255, 0, 0);
    leds[3] = CRGB(255, 0, 0);
    leds[5] = CRGB(255, 0, 0);
    leds[9] = CRGB(255, 0, 0);
    leds[11] = CRGB(255, 0, 0);
    leds[12] = CRGB(255, 0, 0);
    leds[13] = CRGB(255, 0, 0);
    FastLED.show();
    delay(500);
//     digitalWrite(WATCHDOG_WDI, HIGH);
//    delay(2);
//    digitalWrite(WATCHDOG_WDI, LOW);
    for (int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
    FastLED.setBrightness(brightness1);
    leds[0] = CRGB(255, 255, 0);
    leds[14] = CRGB(255, 255, 0);
    FastLED.show();
    
    
  }else{
     

        if(currentSecondsWithWifiVoltage>=numberSecondsWithMinimumWifiVoltageForStartWifi && !wifiManager.getWifiStatus() && !wifiManager.getAPStatus()){
           Serial.print("turning on  wifi");
          restartWifi();
          
//          digitalWrite(WATCHDOG_WDI, HIGH);
//          delay(2);
//          digitalWrite(WATCHDOG_WDI, LOW);
    
          boolean downloadOk=weatherForecastManager->downloadWeatherData(solarInfo, Serial) ;
          Serial.print("line 1343 downloadWeatherData returnxs=");
          Serial.println(downloadOk);

            weatherForecastManager->loadForecasts(Serial); 
            WeatherForecast* forecasts = weatherForecastManager->getForecasts();
            if (forecasts != nullptr) { 
              for (int i = 0; i < 8; i++) { 
               
                  Serial.print("line 1168 downloadWeatherData secondsTime=");
                Serial.print(forecasts[i].secondsTime);
                Serial.print("  temperature=");
                 Serial.println(forecasts[i].temperature);
                 Serial.print("  cloudiness=");
                Serial.println(forecasts[i].cloudiness);
              } 
            } 
       
//          digitalWrite(WATCHDOG_WDI, HIGH);
//          delay(2);
//          digitalWrite(WATCHDOG_WDI, LOW);
              
        }else if( !wifiManager.getAPStatus() && wifiManager.getWifiStatus()){
          //
          // if we are here is because we are connected to a router
           
//       //   Serial.println(digitalStablesData.capacitorVoltage);
//          Serial.print("   wifiManager.getWifiStatus(=");
//          Serial.print(wifiManager.getWifiStatus());
//          Serial.print("   wifiManager.ipAddress=");
//          Serial.print(wifiManager.getIpAddress());
//          Serial.print("   wifiManager.getInternetAvailable=");
//          Serial.print(wifiManager.getInternetAvailable());
//          Serial.print("   wifiManager.getAPStatus()=");
//          Serial.println(wifiManager.getAPStatus());
           // wifiManager.stop();
            
           
        }
         //Serial.println("line 1121");
        uint8_t red = 255;
        uint8_t green = 255;
        uint8_t blue = 255;
 
    // Check if weather data is stale
    boolean staledata=weatherForecastManager->isWeatherDataStale(currentTimerRecord);
 //Serial.print("l;ine 1128 staledata=");
 // Serial.println(staledata);  
      if (staledata) {
          // Fetch weather data and update SolarInfo
          // Serial.println("line 1129");
          if (  wifiManager.getWifiStatus() && !wifiManager.getAPStatus() && !weatherForecastManager->downloadWeatherData(solarInfo, Serial)) {
              // Handle the error: Weather data could not be retrieved
              Serial.print("Failed to retrieve weather data. Weather forecast is invalidated.ipaddress=");
              Serial.println(ipAddress);
              // You can also take additional actions here if necessary
          } else {
              Serial.println("Weather data updated successfully.");
          }
      } else {
         // Serial.println("Weather data is still valid. No need to update.");
      }
   // Serial.println("line 1139");
        if (viewTimer.status()){
          Serial.print("capacitor voltage=");
          Serial.print(digitalStablesData.capacitorVoltage);
          int brightness = 125;
           Serial.print(" displayStatus=");
          Serial.println(displayStatus);
          int numLedsToLight = 5;
          if (digitalStablesData.capacitorVoltage*1000 >= MAXIMUM_LED_VOLTAGE){
            brightness = 255;
            numLedsToLight = 10;//NUM_LEDS;
          }else{
            brightness = map(digitalStablesData.capacitorVoltage * 1000, minimumLEDVoltage, MAXIMUM_LED_VOLTAGE, 1, 125);   // Multiply voltage by 1000 for integer mapping
            brightness = constrain(brightness, 1, 125);                                                                 // Ensure brightness is within bounds
            numLedsToLight = map(digitalStablesData.capacitorVoltage * 1000, minimumLEDVoltage*1000, MAXIMUM_LED_VOLTAGE, 0, 5); // Map voltage to [0, NUM_LEDS]
            numLedsToLight = constrain(numLedsToLight, 0, 10);
            if(numLedsToLight==0)numLedsToLight=1;
          }
          // Ensure within bounds

//          Serial.print("brightness=");
//          Serial.print(brightness);
//          Serial.print("  numLedsToLight=");
//          Serial.println(numLedsToLight);
          showTemperature = !showTemperature;

          for (int i = 0; i < NUM_LEDS; i++){
            leds[i] = CRGB(0, 0, 0);
          }
          FastLED.show();
          FastLED.setBrightness(brightness1);
          
          
          if(displayStatus==SHOW_TEMPERATURE){
          
              Serial.print("showing temperature=");
              readI2CTemp();
          
              if(digitalStablesData.outdoortemperature==-99){
                for (int i = 0; i < NUM_LEDS; i++){
                  leds[i] = CRGB(0, 0, 0);
                }
                FastLED.show();
                leds[1] = CRGB(255, 0, 0);
                leds[2] = CRGB(255, 0, 0);
                leds[3] = CRGB(255, 0, 0);
                leds[7] = CRGB(255, 0, 0);
                leds[12] = CRGB(255, 0, 0);
                FastLED.show();
              }else{
                if(digitalStablesData.outdoortemperature>0){
                  red = 0;
                  green = 0;
                  blue = 255;
                }else if(digitalStablesData.outdoortemperature<=0){  
                  red = 255;
                  green = 0;
                  blue = 0;
                }
                drawTemperature( red, green, blue);
              } 
              
          }else  if(displayStatus== SHOW_SCEPTIC){
            Serial.println("showing scepotic=");

            
            float distance = sonar.ping_cm();
              Serial.print("Distance: ");
              Serial.println(distance);
             float percentage = distance*100/MAX_DISTANCE;
            Serial.print("percentage: ");
              Serial.println(percentage);
               digitalStablesData.scepticAvailablePercentage=percentage;
               digitalStablesData.measuredHeight=distance;
               
             if(digitalStablesData.currentFunctionValue==DAFFODIL_SCEPTIC_TANK ){
                if (percentage <= 25 ){
                  red = 255;
                  green = 0;
                  blue = 0;
                }else if (percentage>25 && percentage<=50){
                  red = 255;
                  green = 255;
                  blue = 0;
                }else if (percentage>50 && percentage<=75){
                  red = 0;
                  green = 255;
                  blue = 0;
                }else if (percentage>75){
                  red = 0;
                  green = 0;
                  blue = 255;
                  
                }
             }else if(digitalStablesData.currentFunctionValue==DAFFODIL_WATER_TROUGH ){
              if (percentage <= 25 ){
                  red = 255;
                  green = 0;
                  blue = 0;
                }else if (percentage>25 && percentage<=50){
                  red = 255;
                  green = 255;
                  blue = 0;
                }else if (percentage>50 && percentage<=75){
                  red = 0;
                  green = 255;
                  blue = 0;
                }else if (percentage>75){
                  red = 0;
                  green = 0;
                  blue = 255;
                  
                }
             }
//            for (int i = 0; i < numLedsToLight; i++)
//            {
//              leds[i] = CRGB(red, green, blue);
//             
//            }

              leds[1] = CRGB(red, green, blue);
              leds[2] = CRGB(red, green, blue);
              leds[3] = CRGB(red, green, blue);
              leds[6] = CRGB(red, green, blue);
              leds[7] = CRGB(red, green, blue);
              leds[8] = CRGB(red, green, blue);
              leds[11] = CRGB(red, green, blue);
              leds[12] = CRGB(red, green, blue);
              leds[13] = CRGB(red, green, blue);

             FastLED.show();
              
          }else if(displayStatus==SHOW_INTERNET_STATUS){
             Serial.print("line 1112 inside of showintenrnetstatus internetAvailable=");
            Serial.println(digitalStablesData.internetAvailable);
             Serial.print("wifiManager.getAPStatus()=");
              Serial.println(wifiManager.getAPStatus());
              Serial.print("wifiManager.getWifiStatus()=");
              Serial.println(wifiManager.getWifiStatus());
                   Serial.print("dsupload timer counter= ");
                   Serial.println(dscount);

            if( wifiManager.getWifiStatus()){
              if(wifiManager.getAPStatus()){
                leds[1] = CRGB(0, 255, 0);
                leds[2] = CRGB(0, 255, 0);
                leds[3] = CRGB(0, 255, 0);
                leds[5] = CRGB(0, 255, 0);
                leds[9] = CRGB(0, 255, 0);
                leds[11] = CRGB(0, 255, 0);
                leds[12] = CRGB(0, 255, 0);
                leds[13] = CRGB(0, 255, 0);
                FastLED.show();
              }else{
                leds[1] = CRGB(0, 0, 255);
                leds[2] = CRGB(0, 0, 255);
                leds[3] = CRGB(0, 0, 255);
                leds[5] = CRGB(0, 0, 255);
                leds[9] = CRGB(0, 0, 255);
                leds[11] = CRGB(0, 0, 255);
                leds[12] = CRGB(0, 0, 255);
                leds[13] = CRGB(0, 0, 255);
                if(digitalStablesData.internetAvailable){
                  leds[7] = CRGB(0, 0, 255);
                }else{
                  leds[7] = CRGB(255, 0, 0);
                }
                FastLED.show();
  //
  //  
                 
    
                if (digitalStablesData.internetAvailable){
                  if (dsUploadTimer.status()){
                    // char secret[27];
                    //    lcd.print("Uploading to digitalstables");
                    String secret = "J5KFCNCPIRCTGT2UJUZFSMQK";
//                    leds[2] = CRGB(0, 255, 0);
//                    FastLED.show();
                    TOTP totp = TOTP(secret.c_str());
                    char totpCode[7]; // get 6 char code
                    long timeVal = timeManager.getTimeForCodeGeneration(currentTimerRecord);
                      Serial.print("line 1153 timeVal=");
                    Serial.println(timeVal);
                    digitalStablesData.secondsTime = timeVal;
                    long code = totp.gen_code(timeVal);
                      Serial.print("l;ine 1154 totp=");
                    Serial.println(code);
                   
                    wifiManager.setCurrentToTpCode(code);
                    int response = wifiManager.uploadDataToDigitalStables();
                     Serial.print("l;ine 1153 uploading to ds=");
                    Serial.println(response);
                    if (response == 200)
                    {
                      leds[7] = CRGB(0, 0, 255);
                    }
                    else if (response == 500)
                    {
                      leds[7] = CRGB(255, 0, 255);
                    }
                    else
                    {
                      leds[7] = CRGB(255, 0, 0);
                    }
                    FastLED.show();
                    dsUploadTimer.reset();
                  }
                }else{
                  //
                  // if we are here it means that there is an ipaddress
                  // but internet is not available , check again if there is a reconnection
                  wifiManager.checkInternetConnectionAvailable();
                 
                  digitalStablesData.internetAvailable=wifiManager.getInternetAvailable();
                   Serial.print("after rechecking digitalstable,internetConnectionAvailable=");
                  Serial.println(digitalStablesData.internetAvailable);
                  
                  
                }
                
              }
            }else{
              leds[1] = CRGB(255, 0, 0);
              leds[2] = CRGB(255, 0, 0);
              leds[3] = CRGB(255, 0, 0);
              leds[5] = CRGB(255, 0, 0);
              leds[9] = CRGB(255, 0, 0);
              leds[11] = CRGB(255, 0, 0);
              leds[12] = CRGB(255, 0, 0);
              leds[13] = CRGB(255, 0, 0);
              FastLED.show();
            }
             
          }else if(displayStatus==SEND_LORA_STATUS){
            Serial.print("showing Lora Status");
             if (loraActive) {
               if(digitalStablesData.capacitorVoltage>=minimumLoraVoltage){
                  drawLora(true);
                  sendMessage();
               }else{
                drawLora(false);
               }
             }else{
               drawLora(false);
             }
          }
          displayStatus++;
          if(displayStatus>3)displayStatus=0;
        viewTimer.reset();
     }
  }

 

  if (Serial.available() != 0)
  {
    String command = Serial.readString();
    if (command.startsWith("Ping"))
    {
      Serial.println(F("Ok-Ping"));
    }
    else if (command.startsWith("SetFieldId"))
    {
      // fieldId= GeneralFunctions::getValue(command, '#', 1).toInt();
    }
    else if (command.startsWith("SetTime"))
    {
      // SetTime#17#5#20#7#11#06#00
      // SetTime#17#5#20#7#11#06#00
      timeManager.setTime(command);
      Serial.println("Ok-SetTime");
      Serial.flush();//SetTime#22#11#24#6#16#58#10
    }
    else if (command.startsWith("SetDeviceSensorConfig"))
    {
      // SetDeviceSensorConfig#DaffOffice#DAFO#Flow##Top Tank#
      String devicename = generalFunctions.getValue(command, '#', 1);
      String deviceshortname = generalFunctions.getValue(command, '#', 2);
      String sensor1name = generalFunctions.getValue(command, '#', 3);
      String sensor2name = generalFunctions.getValue(command, '#', 4);
      
      String timezone = generalFunctions.getValue(command, '#', 5);
      digitalWrite(WATCHDOG_WDI, HIGH);
      delay(2);
      digitalWrite(WATCHDOG_WDI, LOW);
      uint8_t devicenamelength = devicename.length() + 1;
      devicename.toCharArray(digitalStablesData.devicename, devicenamelength);
      deviceshortname.toCharArray(digitalStablesData.deviceshortname, deviceshortname.length() + 1);
      sensor1name.toCharArray(digitalStablesData.sensor1name, sensor1name.length() + 1);
      sensor2name.toCharArray(digitalStablesData.sensor2name, sensor2name.length() + 1);
     
 digitalWrite(WATCHDOG_WDI, HIGH);
        delay(2);
        digitalWrite(WATCHDOG_WDI, LOW);
      secretManager.saveDeviceSensorConfig(devicename, deviceshortname, sensor1name, sensor2name, timezone);
 digitalWrite(WATCHDOG_WDI, HIGH);
        delay(2);
        digitalWrite(WATCHDOG_WDI, LOW);
      Serial.println(F("Ok-SetDeviceName"));
    }
    else if (command.startsWith("SetDeviceName"))
    {
      String devicename = generalFunctions.getValue(command, '#', 1);
      uint8_t devicenamelength = devicename.length() + 1;
      devicename.toCharArray(digitalStablesData.devicename, devicenamelength);
      Serial.println(F("Ok-SetDeviceName"));
    }
    else if (command.startsWith("SetDeviceShortName"))
    {
      String deviceshortname = generalFunctions.getValue(command, '#', 1);
      uint8_t deviceshortnamelength = deviceshortname.length() + 1;
      deviceshortname.toCharArray(digitalStablesData.deviceshortname, deviceshortnamelength);
      Serial.println(F("Ok-SetDeviceShortName"));
    }
    else if (command.startsWith("SetGroupId"))
    {
      String grpId = generalFunctions.getValue(command, '#', 1);
      secretManager.setGroupIdentifier(grpId);
      Serial.print(F("set group id to "));
      Serial.println(grpId);

      Serial.println(F("Ok-SetGroupId"));
    }
    else if (command.startsWith("ConfigWifiSTA"))
    {
      // ConfigWifiSTA#ssid#password
      // ConfigWifiSTA#MainRouter24##OfficeDaffodil#
      String ssid = generalFunctions.getValue(command, '#', 1);
      String password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool staok = wifiManager.configWifiSTA(ssid, password, hostname);
      if (staok)
      {
        leds[0] = CRGB(0, 0, 255);
      }
      else
      {
        leds[0] = CRGB(255, 0, 0);
      }
      FastLED.show();
      Serial.println("Ok-ConfigWifiSTA");
    }
    else if (command.startsWith("ConfigWifiAP"))
    {
      // ConfigWifiAP#soft_ap_ssid#soft_ap_password#hostaname
      // ConfigWifiAP#RosieBench##RosieBench

      String soft_ap_ssid = generalFunctions.getValue(command, '#', 1);
      String soft_ap_password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);

      bool stat = wifiManager.configWifiAP(soft_ap_ssid, soft_ap_password, hostname);
      if (stat)
      {
        leds[0] = CRGB(0, 255, 0);
      }
      else
      {
        leds[0] = CRGB(255, 0, 0);
      }
      FastLED.show();
      Serial.println("Ok-ConfigWifiAP");
    }
    else if (command.startsWith("GetTime"))
    {
      timeManager.printTimeToSerial(currentTimerRecord);
      Serial.flush();
      Serial.println("Ok-GetTime");
      Serial.flush();
    }
    else if (command.startsWith("GetCommandCode"))
    {
      long code = 123456; // secretManager.generateCode();
      //
      // patch a bug in the totp library
      // if the first digit is a zero, it
      // returns a 5 digit number
      if (code < 100000)
      {
        Serial.print("0");
        Serial.println(code);
      }
      else
      {
        Serial.println(code);
      }

      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("VerifyUserCode"))
    {
      String codeInString = generalFunctions.getValue(command, '#', 1);
      long userCode = codeInString.toInt();
      boolean validCode = true; // secretManager.checkCode( userCode);
      String result = "Failure-Invalid Code";
      if (validCode)
        result = "Ok-Valid Code";
      Serial.println(result);
      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("GetSecret"))
    {
      // char secretCode[SHARED_SECRET_LENGTH];
      // secretManager.readSecret(secretCode);
      // Serial.println(secretCode);
      Serial.println("Ok-GetSecret");
      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("SetSecret"))
    {
      // SetSecret#IZQWS3TDNB2GK2LO#6#30
      String secret = generalFunctions.getValue(command, '#', 1);
      int numberDigits = generalFunctions.getValue(command, '#', 2).toInt();
      int periodSeconds = generalFunctions.getValue(command, '#', 3).toInt();
      Serial.println("about to enter savesecret");
      //   secretManager.saveSecret(secret, numberDigits, periodSeconds);
      Serial.println("storing secret");
      Serial.println(secret);

      Serial.println("Ok-SetSecret");
      Serial.flush();
      delay(delayTime);
    }
    else if (command == "Flush")
    {
      while (Serial.read() >= 0)
        ;
      Serial.println("Ok-Flush");
      Serial.flush();
    } 
    else if (command.startsWith("GetSerialNumber"))
    {
      Serial.println(serialNumber);
      Serial.flush();
    }
    else if (command.startsWith("PulseStart"))
    {
      inPulse = true;
      Serial.println("Ok-PulseStart");
      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("PulseFinished"))
    {
      inPulse = false;
      Serial.println("Ok-PulseFinished");
      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("IPAddr"))
    {
      currentIpAddress = generalFunctions.getValue(command, '#', 1);
      Serial.println("Ok-IPAddr");
      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("SSID"))
    {
      currentSSID = generalFunctions.getValue(command, '#', 1);
      Serial.println("Ok-currentSSID");
      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("GetIpAddress"))
    {
      Serial.println(ipAddress);
      Serial.println("Ok-GetIpAddress");
      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("GetSensorData"))
    {

      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("AsyncData"))
    {
      Serial.print("AsyncCycleUpdate#");
      Serial.println("#");
      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("GetLifeCycleData"))
    {
      Serial.println("Ok-GetLifeCycleData");
      Serial.flush();
    }
    else if (command.startsWith("GetWPSSensorData"))
    {
      Serial.println("Ok-GetWPSSensorData");
      Serial.flush();
    }
    else if (command.equals("GetDailySolarPowerSchedule"))
    {
      DailySolarPowerSchedule schedules[48];
      solarInfo->calculateDailySolarPowerSchedule(schedules, currentTimerRecord);
     // tmElements_t tm;
        char timeStr[6];  // HH:mm\0
      for (int i = 0; i < 48; i++) {    

         long seconds = schedules[i].time % 86400;  // Seconds in a day
         int hours = seconds / 3600;
         int minutes = (seconds % 3600) / 60;
         sprintf(timeStr, "%02d:%02d", hours, minutes);
         //breakTime(schedules[i].epochTime, tm);
         //sprintf(timeStr, "%02d:%02d", tm.Hour, tm.Minute);
         Serial.print(timeStr);
         Serial.print(schedules[i].efficiency);
         Serial.print(schedules[i].power);
      }
      
      Serial.println("Ok-DailySolarPowerSchedule");
      Serial.flush();
    }
    else
    {
      //
      // call read to flush the incoming
      //
      Serial.println("Failure-Command Not Found-" + command);
      Serial.flush();
      delay(delayTime);
    }
  }
}

void setStationMode(String ipAddress)
{
  Serial.println("settting Station mode, address ");
  Serial.println(ipAddress);
  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
}

void setApMode()
{

  leds[0] = CRGB(0, 0, 255);
  FastLED.show();
  Serial.println("settting AP mode");
  //
  // set ap mode
  //
  //  wifiManager.configWifiAP("PanchoTankFlowV1", "", "PanchoTankFlowV1");
  String apAddress = wifiManager.getApAddress();
  Serial.println("settting AP mode, address ");
  Serial.println(apAddress);

  for (int i = 2; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  leds[0] = CRGB(0, 255, 0);
  if (loraActive)
  {
    leds[1] = CRGB(0, 0, 255);
  }
  else
  {
    leds[1] = CRGB(255, 0, 0);
  }

  FastLED.show();
}
