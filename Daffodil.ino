// New Version

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

#include <LoRa.h>
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


uint8_t displayStatus=0;
#define SHOW_TEMPERATURE 0
#define SHOW_SCEPTIC 1
#define SHOW_INTERNET_STATUS 2
          
//
// sleeping parameters
//
float sleepingVoltage = 3.5;
float wakingUpVoltage = 3.7;
uint8_t numberSecondsWithMinimumWifiVoltageForStartWifi=30;
uint8_t currentSecondsWithWifiVoltage=0;
float minimumInitWifiVoltage=4.5;

uint8_t sleepingTime = 1;
#define MINIMUM_LED_VOLTAGE 4250
#define MAXIMUM_LED_VOLTAGE 4658
float minimumWifiVoltage=4.5;

uint8_t secondsSinceLastDataSampling = 0;
uint8_t delayTime = 10;
#define UNIQUE_ID_SIZE 8
bool loraActive = false;
DaffodilConfigData daffodilConfigData;
DaffodilData daffodilData;
DaffodilCommandData daffodilCommandData;

PCF8563TimeManager timeManager(Serial);
GeneralFunctions generalFunctions;
Esp32SecretManager secretManager(timeManager);

DaffodilWifiManager wifiManager(Serial, timeManager, secretManager, daffodilData, daffodilConfigData);

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

  LoRa.write((uint8_t*)&daffodilData, sizeof(DaffodilData));
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
  String zone = "AEST-10AEDT,M10.1.0,M4.1.0/3";
  setenv("TZ", zone.c_str(), 1);
  tzset();

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  lcd.init();
  lcd.backlight();
  lcd.clear();
   lcd.setCursor(0, 0);
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
  daffodilData.capacitorVoltage = val_3 * f;
  lcd.setCursor(0, 0);
  lcd.print("cap=");
  lcd.print(daffodilData.capacitorVoltage);
  
  
  //
  if (daffodilData.capacitorVoltage>1 && daffodilData.capacitorVoltage < wakingUpVoltage)
  {
    Serial.print("Voltage is ");
    Serial.print(daffodilData.capacitorVoltage);
    Serial.println(", going to sleep...");
     lcd.print("Volt low ");
    lcd.print(daffodilData.capacitorVoltage);
    lcd.println(", sleeping...");
    
     
    FastLED.setBrightness(5);
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
    leds[0] = CRGB(255, 255, 0);
    FastLED.show();
    delay(500);
    leds[0] = CRGB(0, 0, 0);
    FastLED.show();
    // Convert sleepingTime from minutes to microseconds for deep sleep
    esp_sleep_enable_timer_wakeup(sleepingTime * 60 * 1000000);
    esp_deep_sleep_start();
  }

  pinMode(RTC_CLK_OUT, INPUT_PULLUP); // set up interrupt pin
  digitalWrite(RTC_CLK_OUT, HIGH);    // turn on pullup resistors
  // attach interrupt to set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);
  timeManager.start();
  timeManager.PCF8563osc1Hz();
  tempSensor.begin();
  // uint8_t address[8];
  tempSensor.getAddress(daffodilData.serialnumberarray, 0);
  for (uint8_t i = 0; i < 8; i++)
  {
    serialNumber += String(daffodilData.serialnumberarray[i], HEX);
    daffodilData.checksum += static_cast<uint8_t>(daffodilData.serialnumberarray[i]);
  }
  daffodilData.checksum &= 0xFF;
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

  delay(2000);

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

  if(daffodilData.capacitorVoltage>0 ){
    float factor = 5/daffodilData.capacitorVoltage;
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
    daffodilData.currentFunctionValue = FUN_1_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    secretManager.readFlow1Name().toCharArray(daffodilData.flow1name, 12);
  }
  else if (cswOutput>=6700 && cswOutput< 7100 )
  {
    daffodilData.currentFunctionValue = FUN_2_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);
    secretManager.readFlow1Name().toCharArray(daffodilData.flow1name, 12);
    secretManager.readFlow2Name().toCharArray(daffodilData.flow2name, 12);
  }
  else if (cswOutput>=6300 && cswOutput< 6700)
  {
    daffodilData.currentFunctionValue = FUN_1_FLOW_1_TANK;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);

    secretManager.readFlow1Name().toCharArray(daffodilData.flow1name, 12);
    secretManager.readTank2Name().toCharArray(daffodilData.tank2name, 12);
  }
  else if (cswOutput>=5800 && cswOutput<6300)
  {
    daffodilData.currentFunctionValue = FUN_1_TANK;
    secretManager.readTank1Name().toCharArray(daffodilData.tank1name, 12);
  }
  else if (cswOutput>=5500 && cswOutput<5800)
  {
    daffodilData.currentFunctionValue = FUN_2_TANK;
    secretManager.readTank1Name().toCharArray(daffodilData.tank1name, 12);
    secretManager.readTank2Name().toCharArray(daffodilData.tank2name, 12);
  }else if (cswOutput>=5100 && cswOutput<5500){
    daffodilData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;
  }else if (cswOutput>=4600 && cswOutput<5100){
    daffodilData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
  }else if (cswOutput>=4300 && cswOutput<4600){
    daffodilData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
  }else{
     daffodilData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;
      lcd.setCursor(0, 3);
      lcd.print("bad data sceptic is def");
  }
 uint8_t brightness = map(daffodilData.capacitorVoltage * 1000, MINIMUM_LED_VOLTAGE, MAXIMUM_LED_VOLTAGE, 1, 125);   // Multiply voltage by 1000 for integer mapping
 brightness = constrain(brightness, 1, 125);
 for (int i = 0; i < 5; i++){
      leds[i] = CRGB(0, 0, 0);
  }
    FastLED.show();
 
  for (int i = 0; i < daffodilData.currentFunctionValue; i++){
    leds[i] = CRGB(255, 255, 0);
  }
  FastLED.show();
  delay(2000);
  operatingStatus = secretManager.getOperatingStatus();
  float pingMinutes = secretManager.getSleepPingMinutes();
  esp_sleep_enable_timer_wakeup(pingMinutes * uS_TO_S_FACTOR);

  daffodilData.operatingStatus = operatingStatus;
  daffodilConfigData.sleepPingMinutes = pingMinutes;
  daffodilConfigData.operatingStatus = operatingStatus;
  String grp = secretManager.getGroupIdentifier();
  char gprid[grp.length()];
  grp.toCharArray(gprid, grp.length());
  strcpy(daffodilData.groupidentifier, gprid);

  Serial.print("Starting wifi daffodilConfigData.groupidentifier=");
  Serial.println(daffodilData.groupidentifier);

  String identifier = "daffodilTF";
  char ty[identifier.length() + 1];
  identifier.toCharArray(ty, identifier.length() + 1);
  strcpy(daffodilData.deviceTypeId, ty);

  daffodilConfigData.fieldId = secretManager.getFieldId();
  Serial.print("Starting wifi daffodilConfigData.fieldId=");
  Serial.println(daffodilConfigData.fieldId);

 // wifiManager.start();

//  if(daffodilData.capacitorVoltage>=minimumWifiVoltage){
//    startWifi(); 
//  }
  

  pinMode(RTC_BATT_VOLT, INPUT);
  opmode = digitalRead(OP_MODE);
  daffodilData.opMode = opmode;

  dsUploadTimer.start();
  daffodilData.dataSamplingSec = 10;
  Serial.print("daffodilData.dataSamplingSec=");
  Serial.println(daffodilData.dataSamplingSec);
  Serial.print("Daffodil size=");
  Serial.println(sizeof(daffodilData));

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
  pinMode(WATCHDOG_WDI, OUTPUT);
  digitalWrite(WATCHDOG_WDI, LOW);
}


void restartWifi(){
  if(!initiatedWifi){
    digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
     for (int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
    Serial.print(F("Before Starting Wifi cap="));
    Serial.println(daffodilData.capacitorVoltage);
    wifiManager.start(); 
    Serial.print(F("After Starting Wifi cap="));
    Serial.println(daffodilData.capacitorVoltage);
    initiatedWifi=true;
  }
    Serial.println("Starting wifi");

    digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
    
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
        
    wifiManager.restartWifi();
      digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
    bool stationmode = wifiManager.getStationMode();
    Serial.print("Starting wifi stationmode=");
    Serial.println(stationmode);
  
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
    daffodilData.internetAvailable = wifiManager.getInternetAvailable();

    
    daffodilData.loraActive = loraActive;
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

  if (daffodilData.outdoortemperature > 0 && daffodilData.outdoortemperature < 10)
  {
    ld = daffodilData.outdoortemperature;
  }
  else if (daffodilData.outdoortemperature >= 10 && daffodilData.outdoortemperature < 20)
  {
    ld = daffodilData.outdoortemperature - 10;
    hd = 1;
  }
  else if (daffodilData.outdoortemperature >= 20 && daffodilData.outdoortemperature < 30)
  {
    ld = daffodilData.outdoortemperature - 20;
    hd = 2;
  }
  else if (daffodilData.outdoortemperature >= 30 && daffodilData.outdoortemperature < 40)
  {
    ld = daffodilData.outdoortemperature - 30;
    hd = 3;
  }
  else if (daffodilData.outdoortemperature >= 40 && daffodilData.outdoortemperature < 50)
  {
    ld = daffodilData.outdoortemperature - 40;
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
  
  daffodilData.outdoortemperature=temperature;
   Serial.print("\t\t Temp:");
    Serial.println(temperature, 1);
  daffodilData.outdoorhumidity=CHT.getHumidity();
  Serial.print("\t\t Hum:");
   Serial.print(daffodilData.outdoorhumidity, 1);
 
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

  if (clockTicked)
  {

    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
    digitalWrite(WATCHDOG_WDI, HIGH);
    delay(1);
    digitalWrite(WATCHDOG_WDI, LOW);
    secondsSinceLastDataSampling++;
    // Serial.println("ticked");

    viewTimer.tick();

 //
  // read the height

  ADS.setGain(0);
  //
  // Capacitor Voltage
  int16_t val_3 = ADS.readADC(3);
  float f = ADS.toVoltage(1); //  voltage factor
  daffodilData.capacitorVoltage = val_3 * f;
   lcd.setCursor(0, 3);
  
   
   if(daffodilData.capacitorVoltage> minimumInitWifiVoltage){
    currentSecondsWithWifiVoltage++;
   }else{
    currentSecondsWithWifiVoltage=0;
   }
   
  Serial.print("loop cap ");
   Serial.print(daffodilData.capacitorVoltage);
  Serial.print(" currentssec=");
   Serial.println(currentSecondsWithWifiVoltage);
 Serial.print("wifiManager.getAPStatus()=");
      Serial.print(wifiManager.getAPStatus());
      Serial.print(" wifiManager.getWifiStatus()=");
      Serial.println(wifiManager.getWifiStatus());
    if (daffodilData.internetAvailable)dsUploadTimer.tick();
    currentTimerRecord = timeManager.now();
    if (currentTimerRecord.second == 0){
      if (currentTimerRecord.minute == 0){
        //  Serial.println(F("New Hour"));
        if (currentTimerRecord.hour == 0){
          //    Serial.println(F("New Day"));
        }
      }
    }
  }
 
      
  //
  // check to see if we need to go to sleep
  //
  if (daffodilData.capacitorVoltage>1 && daffodilData.capacitorVoltage < sleepingVoltage){
    Serial.print("Voltage is ");
    Serial.print(daffodilData.capacitorVoltage);
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
    // Convert sleepingTime from minutes to microseconds for deep sleep
    esp_sleep_enable_timer_wakeup(sleepingTime * 60 * 1000000);
    esp_deep_sleep_start();
  }
      
  if(daffodilData.capacitorVoltage< minimumWifiVoltage){
    Serial.print("turning off wifi cap voltage=");
    Serial.println(daffodilData.capacitorVoltage);
     wifiManager.stop();
     daffodilData.internetAvailable=false;
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
     digitalWrite(WATCHDOG_WDI, HIGH);
    delay(2);
    digitalWrite(WATCHDOG_WDI, LOW);
    for (int i = 0; i < NUM_LEDS; i++){
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
    FastLED.setBrightness(20);
    leds[0] = CRGB(255, 255, 0);
    leds[14] = CRGB(255, 255, 0);
    FastLED.show();
    
    
  }else{
     

        if(currentSecondsWithWifiVoltage>=numberSecondsWithMinimumWifiVoltageForStartWifi && !wifiManager.getWifiStatus() && !wifiManager.getAPStatus()){
           Serial.print("turning on  wifi");
          restartWifi();
        }else if( !wifiManager.getAPStatus() && wifiManager.getWifiStatus()){
            Serial.print("turning off  wifi");
            Serial.print("capacitor voltage=");
          Serial.println(daffodilData.capacitorVoltage);
          Serial.print("wifiManager.getWifiStatus(=");
          Serial.println(wifiManager.getWifiStatus());
          Serial.print("wifiManager.getAPStatus()=");
          Serial.println(wifiManager.getAPStatus());
            wifiManager.stop();
            
           
        }
        
        uint8_t red = 255;
        uint8_t green = 255;
        uint8_t blue = 255;

        if (viewTimer.status()){
          Serial.print("capacitor voltage=");
          Serial.println(daffodilData.capacitorVoltage);
          int brightness = 125;
          int numLedsToLight = 5;
          if (daffodilData.capacitorVoltage*1000 >= MAXIMUM_LED_VOLTAGE){
            brightness = 255;
            numLedsToLight = NUM_LEDS;
          }else{
            brightness = map(daffodilData.capacitorVoltage * 1000, MINIMUM_LED_VOLTAGE, MAXIMUM_LED_VOLTAGE, 1, 125);   // Multiply voltage by 1000 for integer mapping
            brightness = constrain(brightness, 1, 125);                                                                 // Ensure brightness is within bounds
            numLedsToLight = map(daffodilData.capacitorVoltage * 1000, MINIMUM_LED_VOLTAGE, MAXIMUM_LED_VOLTAGE, 0, 5); // Map voltage to [0, NUM_LEDS]
            numLedsToLight = constrain(numLedsToLight, 0, NUM_LEDS);
            if(numLedsToLight==0)numLedsToLight=1;
          }
          // Ensure within bounds

          Serial.print("brightness=");
          Serial.print(brightness);
          Serial.print("  numLedsToLight=");
          Serial.println(numLedsToLight);
          showTemperature = !showTemperature;

          for (int i = 0; i < NUM_LEDS; i++){
            leds[i] = CRGB(0, 0, 0);
          }
          FastLED.show();
          FastLED.setBrightness(brightness);
          
          
          if(displayStatus==SHOW_TEMPERATURE){
           
              Serial.print("showing temperature=");
              readI2CTemp();
           
              red = 255;
              green = 255;
              blue = 255;
              drawTemperature( red, green, blue);
              
          }else  if(displayStatus== SHOW_SCEPTIC){
            Serial.println("showing scepotic=");

            
            float distance = sonar.ping_cm();
              Serial.print("Distance: ");
              Serial.println(distance);
             float percentage = distance*100/MAX_DISTANCE;
            Serial.print("percentage: ");
              Serial.println(percentage);
               daffodilData.scepticAvailablePercentage=percentage;
               daffodilData.measuredHeight=distance;
               
             if(daffodilData.currentFunctionValue==DAFFODIL_SCEPTIC_TANK ){
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
             }else if(daffodilData.currentFunctionValue==DAFFODIL_WATER_TROUGH ){
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
            for (int i = 0; i < numLedsToLight; i++)
            {
              leds[i] = CRGB(red, green, blue);
             
            }
             FastLED.show();
              
          }else if(displayStatus==SHOW_INTERNET_STATUS){
           
            daffodilData.internetAvailable=wifiManager.getWifiStatus();
             Serial.print("inside of showintenrnetstatus wifiManager.getWifiStatus()=");
            Serial.println(daffodilData.internetAvailable);
             Serial.println("wifiManager.getAPStatus()");
              Serial.println(wifiManager.getAPStatus());
            if(daffodilData.internetAvailable){
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
                leds[13] = CRGB(0, 255, 255);
                FastLED.show();
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
             
          }

          displayStatus++;
          if(displayStatus>2)displayStatus=0;
          
        
     
        viewTimer.reset();
     }
  }

  if (loraActive) {
      //      Serial.print(F("Seconds time:"));
      //      Serial.println(daffodilTankFlowData.secondsTime);
      sendMessage();
     
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
      Serial.flush();
    }
    else if (command.startsWith("SetDeviceSensorConfig"))
    {
      // SetDeviceSensorConfig#RosieWall#RWAL#Flow##Top Tank#
      String devicename = generalFunctions.getValue(command, '#', 1);
      String deviceshortname = generalFunctions.getValue(command, '#', 2);
      String flow1name = generalFunctions.getValue(command, '#', 3);
      String flow2name = generalFunctions.getValue(command, '#', 4);
      String tank1name = generalFunctions.getValue(command, '#', 5);
      String tank2name = generalFunctions.getValue(command, '#', 6);
      String timezone = generalFunctions.getValue(command, '#', 7);

      uint8_t devicenamelength = devicename.length() + 1;
      devicename.toCharArray(daffodilData.devicename, devicenamelength);
      deviceshortname.toCharArray(daffodilData.deviceshortname, deviceshortname.length() + 1);
      flow1name.toCharArray(daffodilData.flow1name, flow1name.length() + 1);
      flow2name.toCharArray(daffodilData.flow2name, flow2name.length() + 1);
      tank1name.toCharArray(daffodilData.tank1name, tank1name.length() + 1);
      tank2name.toCharArray(daffodilData.tank2name, tank2name.length() + 1);

      secretManager.saveDeviceSensorConfig(devicename, deviceshortname, flow1name, flow2name, tank1name, tank2name, timezone);

      Serial.println(F("Ok-SetDeviceName"));
    }
    else if (command.startsWith("SetDeviceName"))
    {
      String devicename = generalFunctions.getValue(command, '#', 1);
      uint8_t devicenamelength = devicename.length() + 1;
      devicename.toCharArray(daffodilData.devicename, devicenamelength);
      Serial.println(F("Ok-SetDeviceName"));
    }
    else if (command.startsWith("SetDeviceShortName"))
    {
      String deviceshortname = generalFunctions.getValue(command, '#', 1);
      uint8_t deviceshortnamelength = deviceshortname.length() + 1;
      deviceshortname.toCharArray(daffodilData.deviceshortname, deviceshortnamelength);
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
      // ConfigWifiSTA#MainRouter24##Build4SolarPowerVisualizer#
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
