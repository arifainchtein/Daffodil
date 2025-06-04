/// New Version
#include <PowerManager.h>
#include <SolarInfo.h>
//#include <TimeUtils.h>
#include <NewPing.h>
#include "Arduino.h"
#include <Timer.h>
#include <PCF8563TimeManager.h>
#include <SPI.h>
#include <LoRa.h>
#include <Esp32SecretManager.h>
#include <FastLED.h>
#include <DaffodilWifiManager.h>
//#include <DaffodilData.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include <Wire.h>
#include <sha1.h>
#include <totp.h>
#include <SolarPowerData.h>
#include <DigitalStablesData.h>
#include <WeatherForecastManager.h>
#include <BH1750.h>
#include <DigitalStablesDataSerializer.h>
#include "ADS1X15.h"
#include "CHT8305.h"
#include "rgb_lcd.h"
#include <ErrorManager.h>
#include <ErrorDefinitions.h>
#include <DataManager.h>
#include <LittleFS.h>
//#include <driver/adc.h>
#include <BH1750.h>
#include <Adafruit_INA219.h>
#include <esp_sleep.h>
#include <driver/adc.h>

#define RTC_CLK_OUT 4
#define MISO 12
#define MOSI 13
#define SCK 14
#define LoRa_SS 15
#define LORA_RESET 16
#define LORA_DI0 17
//#define WATCHDOG_WDI 18
#define LED_PIN 19
#define LED_CONTROL 23
#define TPL5010_DONE 25
#define SLEEP_SWITCH_26 26
#define TEMPERATURE 27

// const int trigPin = 32;
// const int echoPin = 33;
#define TRIGGER_PIN 32  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN 33 
#define OP_MODE 34
 #define SENSOR_INPUT_2 33
 #define SENSOR_INPUT_1 32
//#define ADS115_ALERT 35
#define TPL5010_WAKE 35
#define RTC_BATT_VOLT ADC1_CHANNEL_0



#define SHUNT_RESISTANCE 0.050 // Shunt resistor value in Ohm
 Adafruit_INA219 ina219(0x41);  
 float SHUNT_OHMS= 0.05;    // Your shunt resistor value in ohms
#define MAX_CURRENT     1.0     // Maximum expected current in Amps
#define CURRENT_LSB     0.00003 // Current LSB in A/bit (10µA per bit for higher precision)
boolean memoryFull=false;
static volatile bool runWatchdog = true;

String currentSSID;
String ipAddress = "";
boolean initiatedWifi = false;
// #define address 0x40

bool debug=false;
DataManager dataManager(Serial, LittleFS);

HourlySolarPowerData hourlySolarPowerData;
boolean usingSolarPower=true;

#define NUM_LEDS 15
CRGB leds[NUM_LEDS];

    // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 90 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define OPERATING_STATUS_SLEEP 1
#define OPERATING_STATUS_NO_LED 2
#define OPERATING_STATUS_FULL_MODE 3


ErrorManager errorManager;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.



// LoRa parameters, registers and constants



#define REG_OP_MODE            0x01
#define REG_IRQ_FLAGS         0x12
#define REG_RSSI_VALUE        0x1B
#define MODE_CAD              0x87
#define IRQ_CAD_DONE_MASK     0x04
#define IRQ_CAD_DETECTED_MASK 0x02

#define CAD_TIMEOUT      5000    // CAD timeout in milliseconds
#define MAX_RETRIES      5       // Maximum transmission retries
#define MIN_BACKOFF      500     // Minimum backoff time in milliseconds
#define MAX_BACKOFF      1500    // Maximum backoff time in milliseconds
#define RSSI_THRESHOLD   -60     // RSSI threshold in dBm
//define LORA_SAMPLES 3 // Number of samples to take
//define CHECK_LORA_DELAY 2 // Delay between samples in ms

CHT8305 CHT(0x40);
bool loraTxOk=false;


TaskHandle_t watchdogTask;
//i2c addresses:
// 40=DFRobot i2c temperature sensor
// 48= ADS1115
// 51= PCF8563T
// 23= bh1750

BH1750 lightMeter;

ADS1115 ADS(0x48);
volatile bool RDY = false;
uint8_t ads1115channel = 0;
int16_t ads1115val[4] = {0, 0, 0, 0};

OneWire oneWire(TEMPERATURE);
DallasTemperature tempSensor(&oneWire);

Timer viewTimer(5);
Timer remoteMonitorTimer(5);
#define MAXIMUM_STORED_RECORDS 5000

bool showTime = false;


// bool internetAvailable;
bool wifiActiveSwitch;
#define uS_TO_S_FACTOR 60000000 /* Conversion factor for micro seconds to minutes */

Timer dsUploadTimer(60);
static volatile int flowMeterPulseCount;
static volatile int flowMeterPulseCount2;

volatile bool loraReceived=false;
volatile int loraPacketSize=0;


uint8_t displayStatus = 0;
uint8_t loraLastResult=-99;
LoRaError cadResult;
float avgRssi=0;

#define SHOW_TEMPERATURE 0
#define SHOW_SCEPTIC 1
#define SHOW_INTERNET_STATUS 2
#define SEND_LORA_STATUS 3
#define SHOW_ERROR_STATUS 4
//
// sleeping parameters
//
float sleepingVoltage = 4.0;

uint8_t numberSecondsWithMinimumWifiVoltageForStartWifi = 30;
uint8_t currentSecondsWithWifiVoltage = 0;
float minimumInitWifiVoltage = 4.5;
float minimumLoraVoltage = 4.2;
//uint8_t sleepingTime = 1;
float minimumLEDVoltage = 4.3;
#define MAXIMUM_LED_VOLTAGE 4658
float minimumWifiVoltage = 4.25;

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
SolarInfo *solarInfo;
PowerManager *powerManager;
WeatherForecastManager *weatherForecastManager;
double lightMeterCorrectingFactor=3.45;
DaffodilWifiManager wifiManager(Serial, LittleFS,timeManager, secretManager, digitalStablesData, digitalStablesConfigData);

//int badPacketCount = 0;
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

//#include <LiquidCrystal_I2C.h>
//LiquidCrystal_I2C lcd(0x27, 20, 4); // set the lcd address to 0x27 for a 16 chars and 2 line display
rgb_lcd lcd;

bool foundlcd = false;
 bool foundtemp = false;
 bool foundADS = false;
 bool foundBH1750=false;
 bool foundINA219=false;
 bool PCF8563T=false;

 //
 // watchdog
 //
TaskHandle_t refreshTaskHandle = NULL;
volatile bool wakeSignalReceived = false;
unsigned long lastWakeTime = 0;
const unsigned long WAKE_INTERVAL_MS = 1000; // Expected wake interval


String serialNumber;
struct TempHum
{
  float temp = -99;
  float hum = -99;

} tempHum;

const float R1 = 1000000.0; // Resistance of R1 in ohms (1 MΩ)
const float R2 = 2000000.0; // Resistance of R2 in ohms (2 MΩ)
const float Vref = 3.3;     // Reference voltage of the ESP32

int view_milliseconds = 5000;
long lastswitchmillis = 0;
boolean showTemperature = false;
uint8_t color = 0;
String timezone;

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
void onReceive(int packetSize)
{
 //  Serial.print(" Receive lora: ");
 //     Serial.println(packetSize);
      loraReceived=true;
      loraPacketSize=packetSize;
}
void processLora(int packetSize){
   if(debug)Serial.print(" Receive lora: ");
      if(debug)Serial.println(packetSize);

       if(debug)Serial.print(" size of ds: ");
      if(debug)Serial.print(sizeof(DigitalStablesData));

 if(debug)Serial.print(" WeatherForecastUpdate: ");
      if(debug)Serial.print(sizeof(WeatherForecastUpdate));

 if(debug)Serial.print(" s RequestCommand: ");
      if(debug)Serial.println(sizeof(RequestCommand));

  if (packetSize == 0) return; // if there's no packet, return

    
  if (packetSize == sizeof(DigitalStablesData))
  {
    DigitalStablesData receivedDigitalStablesData;
    LoRa.readBytes((uint8_t *)&receivedDigitalStablesData, sizeof(DigitalStablesData));
    long commandcode = receivedDigitalStablesData.totpcode;
    bool validCode = secretManager.checkCode(commandcode);

     if(debug)Serial.print(" Received DigitalStablesData code  : ");
      if(debug)Serial.print(commandcode);
    String receivedSerialNumber;
    for (uint8_t i = 0; i < 8; i++)
      {
        receivedSerialNumber += String(receivedDigitalStablesData.serialnumberarray[i], HEX);
      }

     if(debug)Serial.print(" sn  : ");
      if(debug)Serial.print(receivedSerialNumber);
 if(receivedSerialNumber==serialNumber){
    if(debug)Serial.println("Ignored self reception");
 }else{
    if (validCode)
        {

          // secretManager.saveSleepPingMinutes(rosieConfigData.sleepPingMinutes);
          // secretManager.saveConfigData(rosieConfigData.fieldId,  stationName );

          int rssi = LoRa.packetRssi();
          float Snr = LoRa.packetSnr();
          if(debug)Serial.print(" valid code ");
          if(debug)Serial.print("  from: ");
          if(debug)Serial.println(receivedDigitalStablesData.devicename);
        }
        else
        {
          long currentcode =secretManager.generateCode();
          if(debug)Serial.print(" Receive digitalStablesData but invalid code: ");
          if(debug)Serial.print(commandcode);
          if(debug)Serial.print(" currentcode: ");
          if(debug)Serial.println(currentcode);
        }
 }
   
  }else  if (packetSize == sizeof(RequestCommand)){
    RequestCommand rc;
    LoRa.readBytes((uint8_t *)&rc, sizeof(RequestCommand));
     long totpcode = rc.totpcode;
    bool validCode = secretManager.checkCode(totpcode);
    if (validCode){
      String commandcode = String(rc.commandString);
      if(commandcode=="SendAsyncData"){
        if(debug)Serial.println("received SEND_ASYNC_DATA");

          // void sendDataViaLoRa() {
            // Read all stored data
            const int MAX_RECORDS = 10;  // Adjust based on your memory constraints
            DigitalStablesData dataArray[MAX_RECORDS];
            int actualSize = 0;
            if (!dataManager.readAllDSDData(dataArray, MAX_RECORDS, actualSize)) {
              if(debug)Serial.println("No data to send or error reading data");
                RequestCommand rc;
                rc.totpcode=secretManager.generateCode();
                rc.setCommand("NoData");
                sendMessage(rc);
            }
            if(debug)Serial.printf("Sending %d records via LoRa...\n", actualSize);

            // Send each record as binary data
            for (int i = 0; i < actualSize; i++) {
              dataArray[i].totpcode=secretManager.generateCode();
              sendMessage(dataArray[i]);
            // LoRa.endPacket();
              if(debug)Serial.printf("Sent record %d/%d (%d bytes)\n",
                            i+1, actualSize, sizeof(DigitalStablesData) + 2);  // +2 for marker and index
              // Brief delay to avoid overwhelming the receiver
              delay(200);
            }
            if(debug)Serial.println("All data sent");
          //}


      }else if(commandcode=="ReceivedOK"){
        if(debug)Serial.println("received RECEIVED_OK");
      }else if(commandcode=="ClearedOk"){
        if(debug)Serial.println("received CLEARED_OK");
      }else if(commandcode=="NoData"){
        if(debug)Serial.println("received NO_DATA");
      }else if(commandcode=="SendCurrentData"){
        if(debug)Serial.println("received SendCurrentData, sending ..");
        sendMessage(digitalStablesData);
      }
    } else
    {
      if(debug)Serial.print(" Receive RequestCommand but invalid code: ");
      if(debug)Serial.println(totpcode);
    
    }
  }else  if (packetSize == sizeof(WeatherForecastUpdate)){
    WeatherForecastUpdate weatherForecastUpdate;
    LoRa.readBytes((uint8_t *)&weatherForecastUpdate, sizeof(WeatherForecastUpdate));
     long commandcode = weatherForecastUpdate.totpcode;
    bool validCode = secretManager.checkCode(commandcode);
    if (validCode){
      //WeatherForecast forecasts=weatherForecastUpdate.forecasts;
      weatherForecastManager->saveForecasts(weatherForecastUpdate.forecasts);
      solarInfo->setWeatherForecast(weatherForecastUpdate.forecasts ,4);
       if(debug)Serial.println(" Receive and processed weatherForecast ");
    } else
    {
      if(debug)Serial.print(" Receive WeatherForecastUpdate but invalid code: ");
      if(debug)Serial.println(commandcode);
    
    }
  }else {//if(packetSize==14){
       if(debug)Serial.println("\n--- RECEIVED unknown size  PACKET ---");
    // Create a buffer to store the received bytes
    uint8_t buffer[packetSize];
    // Read all bytes into the buffer
    for (int i = 0; i < packetSize; i++) {
      buffer[i] = LoRa.read();
    }
    // Display as hex values (with position)
    if(debug)Serial.print("HEX: ");
    for (int i = 0; i < packetSize; i++) {
      // Print position
      if(debug)Serial.print("[");
      if(debug)Serial.print(i);
      if(debug)Serial.print("]");
      // Print hex value with leading zero if needed
      if (buffer[i] < 16) if(debug)Serial.print("0");
      if(debug)Serial.print(buffer[i], HEX);
      if(debug)Serial.print(" ");
    }
    if(debug)Serial.println();
    // Display as ASCII (printable characters only)
    if(debug)Serial.print("ASCII: ");
    for (int i = 0; i < packetSize; i++) {
      // Check if it's a printable ASCII character (32-126)
      if (buffer[i] >= 32 && buffer[i] <= 126) {
        if(debug)Serial.print((char)buffer[i]);
      } else {
        if(debug)Serial.print(".");  // Non-printable character
      }
    }
    if(debug)Serial.println();
    // Display as decimal values
    if(debug)Serial.print("DEC: ");
    for (int i = 0; i < packetSize; i++) {
      if(debug)Serial.print("[");
      if(debug)Serial.print(i);
      if(debug)Serial.print("]");
      if(debug)Serial.print(buffer[i]);
      if(debug)Serial.print(" ");
    }
    if(debug)Serial.println();
    // Try to interpret as common data types
    if (packetSize >= 4) {
      // As 32-bit integer (little endian)
      int32_t int32Value = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
      if(debug)Serial.print("As Int32 (LE): ");
      if(debug)Serial.println(int32Value);
      // As 32-bit float (little endian)
      float floatValue;
      memcpy(&floatValue, buffer, 4);
      if(debug)Serial.print("As Float (LE): ");
      if(debug)Serial.println(floatValue);
    }
    // Calculate a simple checksum to see if it's consistent
    uint8_t checksum = 0;
    for (int i = 0; i < packetSize - 1; i++) {
      checksum ^= buffer[i];  // XOR checksum
    }
    if(debug)Serial.print("Last byte: ");
    if(debug)Serial.print(buffer[packetSize-1]);
    if(debug)Serial.print(", XOR Checksum: ");
    if(debug)Serial.println(checksum);
    if(debug)Serial.println("--- END OF PACKET ANALYSIS ---\n");
  }
}
void LoRa_txMode()
{
  LoRa.idle();            // set standby mode
  LoRa.disableInvertIQ(); // normal mode
}
void LoRa_rxMode()
{
  LoRa.disableInvertIQ(); // normal mode
  LoRa.receive();         // set receive mode
}

LoRaError performCAD() {
    if (!loraActive) {
        errorManager.setLoRaError(LORA_INIT_FAILED);
        return LORA_INIT_FAILED;
    }

    // Multiple RSSI checks with averaging
    const int SAMPLES = 3;
    const int CHECK_DELAY = 2; // ms between samples
    float rssiSum = 0;

    // First set of samples
    LoRa.idle();
    LoRa.receive();
    
    for(int i = 0; i < SAMPLES; i++) {
        rssiSum += LoRa.rssi();
        delay(CHECK_DELAY);
    }

    avgRssi = rssiSum / SAMPLES;
    
    // If average RSSI is above threshold, channel is busy
    if (avgRssi > RSSI_THRESHOLD) {
        LoRa.idle();
        errorManager.setLoRaError(LORA_CHANNEL_BUSY);
        return LORA_CHANNEL_BUSY;
    }

    // Double-check with a second set of samples
    rssiSum = 0;
    for(int i = 0; i < SAMPLES; i++) {
        rssiSum += LoRa.rssi();
        delay(CHECK_DELAY);
    }

    avgRssi = rssiSum / SAMPLES;  
    LoRa.idle();
 
    if(debug)Serial.print("checkcad, avgRssi=");
    if(debug)Serial.print(avgRssi);
    if (avgRssi > RSSI_THRESHOLD) {
       // errorMgr.setLoRaError(LORA_CHANNEL_BUSY, avgRssi);  
        return LORA_CHANNEL_BUSY;
    }

    errorManager.clearLoRaError(LORA_CHANNEL_BUSY);  // Clear any previous channel busy error
    return LORA_OK;
}

LoRaError performCADOld() {
    if (!loraActive) return LORA_INIT_FAILED;

    // Multiple RSSI checks with averaging
    const int SAMPLES = 3;
    const int CHECK_DELAY = 2; // ms between samples
    float rssiSum = 0;

    // First set of samples
    LoRa.idle();
    LoRa.receive();
    
    for(int i = 0; i < SAMPLES; i++) {
        rssiSum += LoRa.rssi();
        delay(CHECK_DELAY);
    }

     avgRssi = rssiSum / SAMPLES;
    
    // If average RSSI is above threshold, channel is busy
    if (avgRssi > RSSI_THRESHOLD) {
        LoRa.idle();
        return LORA_CHANNEL_BUSY;
    }

    // Double-check with a second set of samples
    rssiSum = 0;
    for(int i = 0; i < SAMPLES; i++) {
        rssiSum += LoRa.rssi();
        delay(CHECK_DELAY);
    }

    avgRssi = rssiSum / SAMPLES;
    LoRa.idle();
 
   if(debug)Serial.print("  checkcad, avgRssi=");
  if(debug)Serial.println(avgRssi);
    if (avgRssi > RSSI_THRESHOLD) {
        return LORA_CHANNEL_BUSY;
    }

    return LORA_OK;
}

template <typename T>
uint8_t calculateChecksum(const T& data) {
  uint8_t checksum = 0;
  const uint8_t* dataPtr = (const uint8_t*)&data;
  
  // Get the offset of the checksum field
  size_t checksumOffset = 0;
  if constexpr (std::is_same<T, DigitalStablesData>::value) {
    checksumOffset = offsetof(DigitalStablesData, checksum);
  } else if constexpr (std::is_same<T, RequestCommand>::value) {
    checksumOffset = offsetof(RequestCommand, checksum);
  }
  
  // Calculate checksum for bytes before the checksum field
  for (size_t i = 0; i < checksumOffset; i++) {
    checksum ^= dataPtr[i]; // XOR operation
  }
  
  // Calculate checksum for bytes after the checksum field
  for (size_t i = checksumOffset + sizeof(uint8_t); i < sizeof(T); i++) {
    checksum ^= dataPtr[i];
  }
  
  return checksum;
}

template <typename T>
int sendMessage(const T& inputData)
{
    T dataToSend = inputData;
  
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print("Sending Lora");
  if(debug)Serial.print("sending lora sn=");
  if(debug)Serial.print(serialNumber);
  long code = secretManager.generateCode();
  if(debug)Serial.print("  totp=");
  if(debug)Serial.print(code);

   if constexpr (std::is_same<T, DigitalStablesData>::value) {
    dataToSend.totpcode = code;
  } else if constexpr (std::is_same<T, RequestCommand>::value) {
    dataToSend.totpcode = code;
  }

   if constexpr (std::is_same<T, DigitalStablesData>::value) {
    dataToSend.checksum = 0;
  } else if constexpr (std::is_same<T, RequestCommand>::value) {
    dataToSend.checksum = 0;
  }
  uint8_t calculatedChecksum = calculateChecksum(dataToSend);

  if constexpr (std::is_same<T, DigitalStablesData>::value) {
    dataToSend.checksum = calculatedChecksum;
    if(debug)Serial.print(" checksum=");
    if(debug)Serial.print(dataToSend.checksum, HEX);
  } else if constexpr (std::is_same<T, RequestCommand>::value) {
    dataToSend.checksum = calculatedChecksum;
    if(debug)Serial.print(" checksum=");
    if(debug)Serial.print(dataToSend.checksum, HEX);
  }
  
  
  LoRa_txMode();
  uint8_t result = 99;
  int retries = 0;
  boolean keepGoing = true;
  long startsendingtime = millis();
  while (keepGoing) {
    cadResult = performCAD();
    if (cadResult == LORA_OK) {
      // Channel is clear, attempt transmission
      LoRa.beginPacket();
      // Send the provided data object
      LoRa.write((uint8_t *)&dataToSend, sizeof(T));
      if (!LoRa.endPacket(true)) {
        result = LORA_TX_FAILED;
      } else {
        result = LORA_OK;
      }
      if(debug)Serial.print("took ");
      if(debug)Serial.print(millis() - startsendingtime);
      keepGoing = false;
    } else if (cadResult != LORA_CHANNEL_BUSY) {
      // If error is not due to busy channel, return the error
      result = cadResult;
      int backoff = random(MIN_BACKOFF * (1 << retries), MAX_BACKOFF * (1 << retries));
      lcd.setCursor(0, 1);
      lcd.clear();
      lcd.print("Bu ");
      lcd.print(retries + 1);
      lcd.print(" of ");
      lcd.print(MAX_RETRIES);
      lcd.print(". b=");
      lcd.print(backoff);
      lcd.println("ms");
      if(debug)Serial.print("Channel busy, retry ");
      if(debug)Serial.print(retries + 1);
      if(debug)Serial.print(" of ");
      if(debug)Serial.print(MAX_RETRIES);
      if(debug)Serial.print(". Waiting ");
      if(debug)Serial.print(backoff);
      if(debug)Serial.println("ms");
      // Channel is busy, implement exponential backoff
      delay(backoff);
      retries++;
      keepGoing = retries < MAX_RETRIES;
    }
  }
  lcd.setCursor(0, 1);
  if (result == 99) {
    result = LORA_MAX_RETRIES_REACHED;
  }
  lcd.setCursor(0, 1);
  lcd.print("Lora returns ");
  lcd.print(result);
  if(debug)Serial.print(" ,Lora returns ");
  if(debug)Serial.println(result);
  delay(500);
  LoRa_rxMode();
  msgCount++; // increment message ID
  return result;
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      if(debug)Serial.println("Wakeup caused by button press");
      // Do something specific when button wakes the device
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      if(debug)Serial.println("Wakeup caused by timer");
      // Do something specific when timer wakes the device
      break;
    default:
      if(debug)Serial.println("First boot or reset");
      break;
  }
}


//
// End of Lora Functions
//
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  pinMode(SLEEP_SWITCH_26, OUTPUT);
  digitalWrite(SLEEP_SWITCH_26, HIGH);

  
if(!LittleFS.begin(false)) { 
  Serial.println("LittleFS Mount Failed, formatting..."); 
  LittleFS.format(); 
  if(!LittleFS.begin(false)) { 
    Serial.println("LittleFS Mount Failed even after formatting"); 
    return; 
  } 
 }

print_wakeup_reason();
dataManager.start();

  pinMode(LED_CONTROL, OUTPUT);
  digitalWrite(LED_CONTROL, LOW);

  


  lightMeter.setMTreg(32);// 
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  lightMeter.begin();

  //
  // data from cofiguration
  //
  double latitude = -37.13305556;
  double longitude = 144.47472222;
  double altitude = 410.0;
   lightMeterCorrectingFactor=3.45;
  

  secretManager.getDeviceSensorConfig(digitalStablesData.devicename, digitalStablesData.deviceshortname, digitalStablesData.sensor1name, digitalStablesData.sensor2name, timezone, latitude, longitude, altitude,digitalStablesData.minimumEfficiencyForLed, digitalStablesData.minimumEfficiencyForWifi);
  
  // timezone = "AEST-10AEDT,M10.1.0,M4.1.0/3";
   char timezoneinfo[] = "AEST-10AEDT,M10.1.0,M4.1.0/3";
  
  
  
  float capacitorValue = 3.0;
  float currentPerLed = .020;
  const char *apiKey = "103df7bb3e4010e033d494f031b483e0";
   TimeUtils::parseTimezone(timezone);
  setenv("TZ", timezone.c_str(), 1);
  tzset();
  digitalStablesData.latitude=latitude;
  digitalStablesData.longitude=longitude;

if(debug)Serial.print("digitalStablesData.minimumEfficiencyForLed=");
  if(debug)Serial.println(digitalStablesData.minimumEfficiencyForLed);
  
  if(debug)Serial.print("sizeof DigitalStablesData=");
  if(debug)Serial.println(sizeof(DigitalStablesData));

  if(debug)Serial.print("sizeof RequestCommanmd=");
  if(debug)Serial.println(sizeof(RequestCommand));

  if(debug)Serial.print("sizeof WeatherForecastUpdate=");
  if(debug)Serial.println(sizeof(WeatherForecastUpdate));

  pinMode(RTC_CLK_OUT, INPUT_PULLUP); // set up interrupt pin
  digitalWrite(RTC_CLK_OUT, HIGH);    // turn on pullup resistors
  // attach interrupt to set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);
 
  timeManager.start();
  timeManager.PCF8563osc1Hz();
  currentTimerRecord = timeManager.now();
  digitalStablesData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
  digitalStablesData.asyncdata = 1;
  
  if(dataManager.getDSDStoredCount()<MAXIMUM_STORED_RECORDS){
    dataManager.storeDSDData(digitalStablesData);
  }
  
  
  //  const char* ntpServer = "pool.ntp.org";
  //  const long gmtOffset_sec = 36000;  // Melbourne is UTC+10
  //  const int daylightOffset_sec = 3600; // 1 hour during daylight savings

  solarInfo = new SolarInfo(Serial, latitude, longitude, altitude);
  weatherForecastManager = new WeatherForecastManager(Serial,latitude, longitude, apiKey);
  weatherForecastManager->initialize(currentTimerRecord);
  weatherForecastManager->loadForecasts(Serial);
  if(debug)Serial.print("hasValidForecasts=");
  if(debug)Serial.println(weatherForecastManager->hasValidForecasts());

  

  powerManager = new PowerManager(Serial, ADS, *solarInfo, latitude, longitude, capacitorValue, currentPerLed);
  DailySolarData dailySolarData = solarInfo->getDailySolarData(currentTimerRecord);

  if(debug)Serial.print("sunrise=");
  if(debug)Serial.println(dailySolarData.sunrise);
  if(debug)Serial.print("sunset=");
  if(debug)Serial.println(dailySolarData.sunset);

  if(debug)Serial.print("sunrisetime=");
  if(debug)Serial.println(dailySolarData.sunrisetime);
  if(debug)Serial.print("sunsettime=");
  if(debug)Serial.println(dailySolarData.sunsettime);

  
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();

  
  hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);
  if(debug)Serial.print(" line 368 efficiency=");
  if(debug)Serial.println(hourlySolarPowerData.efficiency);
//  if(debug)Serial.print("actualPower=");
//  Serial.println(hourlySolarPowerData.actualPower);
//  Serial.print("irradiance=");
//  Serial.println(hourlySolarPowerData.irradiance);
//  Serial.print("temperature=");
//  Serial.println(hourlySolarPowerData.temperature);

 

  if(debug)Serial.println("Scanning for I2C devices ...");
 
  
  byte error, address;
  int nDevices = 0;

  for (address = 0x01; address < 0x7f; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0)
    {
      if(debug)Serial.print("I2C device found at address ");
      if(debug)Serial.println(address);
      if (address == 3){
        foundlcd = true;
      }else if (address == 64){ // 0x40
        foundtemp = true;
         if(debug)Serial.println("foundtemp");
      }else if (address == 72){ // 0x48
        foundADS = true;
         if(debug)Serial.println("foundADS");
      }else if (address == 65){ // 0x41
        foundINA219 = true;
         if(debug)Serial.println("foundINA219");
      }else if (address == 81){ // 0x51;
        PCF8563T = true;
         if(debug)Serial.println("found PCF8563T");
      }else if (address == 35){  // 0x23
        foundBH1750=true;
         if(debug)Serial.println("foundBH1750");
      }
        
      nDevices++;
    }
    else if (error != 2)
    {
      if(debug)Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }

  if(!foundlcd){
    errorManager.setI2CError(I2C_DEVICE0_ERROR);
  }
  if(!foundtemp)errorManager.setI2CError(I2C_DEVICE1_ERROR);
  if(!foundADS)errorManager.setI2CError(I2C_DEVICE2_ERROR);
  if(!PCF8563T)errorManager.setI2CError(I2C_DEVICE3_ERROR);
  if(!foundBH1750)errorManager.setI2CError(I2C_DEVICE4_ERROR);
  if(!foundINA219)errorManager.setI2CError(I2C_DEVICE5_ERROR);
  

  if (!foundtemp)
  {
    for (int i = NUM_LEDS - 5; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
  }

  lcd.begin(16, 2);
  //lcd.setRGB(colorR, colorG, colorB);
    
  //  lcd.init();
  //  lcd.backlight();
  //  lcd.clear();
     lcd.setCursor(0, 0);
  CHT.begin();
  ADS.begin();
  if (!ADS.begin())
  {
    if(debug)Serial.println("invalid address ADS1115 or 0x48 not found");
    foundADS=false;
  }
  else
  {
    if(debug)Serial.println("found ADS1115");
    foundADS=true;
  }


 if (! ina219.begin()) {
    if(debug)Serial.println("Failed to find INA219 chip");
    foundINA219=false;
  }else{
     foundINA219=true;
    if(debug)Serial.println("initialized INA219");
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
 // ina219.setCalibration_16V_400mA();
 // Reset the device
  Wire.beginTransmission(0x40);
  Wire.write(0x00); // Config register
  Wire.write(0x80); // Reset bit
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50); // Wait for reset
  
  // Custom calibration for 0.05 ohm shunt and 1A max current
 // uint16_t calibrationValue = 10240; // As calculated for 0.05 ohm shunt
  float SHUNT_OHMS=0.05;
  //float Current_LSB = MAX_CURRENT/2^15;
  //float Current_LSB = MAX_CURRENT/32768.0;
  
   uint16_t calibrationValue = (uint16_t)(0.04096 / (CURRENT_LSB * SHUNT_OHMS));
  
  if(debug)Serial.print("Calculated calibration value: ");
  if(debug)Serial.println(calibrationValue);

  
  // Set configuration register - try with different gain settings
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                   INA219_CONFIG_GAIN_8_320MV |  // Higher gain for better resolution
                   INA219_CONFIG_BADCRES_12BIT |
                   INA219_CONFIG_SADCRES_12BIT_1S_532US |
                   INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  
  // Write configuration
  Wire.beginTransmission(0x40);
  Wire.write(0x00); // Config register
  Wire.write((config >> 8) & 0xFF);
  Wire.write(config & 0xFF);
  Wire.endTransmission();
  
  // Write calibration
  Wire.beginTransmission(0x40);
  Wire.write(0x05); // Calibration register
  Wire.write((calibrationValue >> 8) & 0xFF);
  Wire.write(calibrationValue & 0xFF);
  Wire.endTransmission();
  }
  
  



  if(debug)Serial.println(CHT.getManufacturer(), HEX);
  if(debug)Serial.println(CHT.getVersionID(), HEX);
  if(debug)Serial.println(CHT.getVoltage());

  ADS.setGain(0);
  //
  // Capacitor Voltage
  int16_t val_3 = ADS.readADC(3);
  float f = ADS.toVoltage(1); //  voltage factor
  digitalStablesData.capacitorVoltage = val_3 * f;
  lcd.setCursor(0, 0);
  lcd.print("cap=");
  lcd.print(digitalStablesData.capacitorVoltage);
  if(debug)Serial.print("i setup cap=");
  if(debug)Serial.println(digitalStablesData.capacitorVoltage);
   
 
  // Config Switch
 // if(debug)Serial.print("voltage factor=");
 // if(debug)Serial.println(f);
  float rawValue = ADS.readADC(2);
  if(debug)Serial.print("rawValue=");
  if(debug)Serial.println(rawValue);
  float factor =1;
  int16_t cswOutput = rawValue;
  float correctedvalue = factor*rawValue;
   if (digitalStablesData.capacitorVoltage > 0)
  {
     factor = 5 / digitalStablesData.capacitorVoltage;
    cswOutput = rawValue * factor;
  }
  lcd.setCursor(0, 1);
  lcd.print("csw=");
  lcd.print(rawValue);

  


  if(debug)Serial.print("corrected cswOutput=");
  if(debug)Serial.println(cswOutput);
  lcd.setCursor(0, 2);
  lcd.print(" corr csw=");
  lcd.print(cswOutput);
  //
  // dip switch values
  // 1234
  // 0000 =  F
  // 1000 =  FF
  // 0100 =  FT
  // 1100 =  T
  // 0010 =  TT
  // 1010 =  Daffodil Sceptic
  // 0110 =  Daffodil Water Trough
  // 1110 =  DAFFODILE_TEMP_SOILMOISTURE
  // 0001 =  
  // 1001 = 
  // 1101 = 
  // 1111 = 0    // VOLTAGE_MONITOR
 
 if (cswOutput >= 8200 ) {
    // Position 0: 00000 (All OFF)
        digitalStablesData.currentFunctionValue = FUN_1_FLOW;
        attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
        secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
        usingSolarPower=false;
    } else if (cswOutput >= 8000 && cswOutput <= 8200) {
        // Position 1: 00001 (R3 ON)
        digitalStablesData.currentFunctionValue = FUN_2_FLOW;
        attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
        attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);
        secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
        secretManager.readFlow2Name().toCharArray(digitalStablesData.sensor2name, 12);
        usingSolarPower=false;
    } else if (cswOutput >= 7800 && cswOutput < 8000) {
        // Position 2: 00010 (R4 ON)
        digitalStablesData.currentFunctionValue = FUN_1_FLOW_1_TANK;
        attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
        secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
        secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
        usingSolarPower=false;
    } else if (cswOutput >= 7690 && cswOutput < 7800) {
         // Position 3: 00011 (R3+R4 ON)
        digitalStablesData.currentFunctionValue = FUN_1_TANK;
        secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
        usingSolarPower=false;
    } else if (cswOutput >= 7500 && cswOutput < 7690) {
        // Position 4: 00100 (R10 ON)
        digitalStablesData.currentFunctionValue = FUN_2_TANK;
        secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
        secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12); 
        usingSolarPower=false;
    } else if (cswOutput >= 7300 && cswOutput < 7500) {
        // Position 5: 00101 (R3+R10 ON)
        digitalStablesData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;
        usingSolarPower=false;
    } else if (cswOutput >= 7100 && cswOutput < 7300) {
        cswOutput = 6;  // Position 6: 00110 (R4+R10 ON)
        digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
        usingSolarPower=false;
    } else if (cswOutput >= 6900 && cswOutput < 7100) {
        // Position 7: 00111 (R3+R4+R10 ON)
           digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
            usingSolarPower=false;
    } else if (cswOutput >= 6600 && cswOutput < 6900) {
        // Position 8: 01000 (R11 ON)
        usingSolarPower=false;
    } else if (cswOutput >= 6300 && cswOutput < 6600) {
        // Position 9: 01001 (R3+R11 ON)
        usingSolarPower=false;
    } else if (cswOutput >= 6100 && cswOutput < 6300) {
        // Position 10: 01010 (R4+R11 ON)
        usingSolarPower=false;
    } else if (cswOutput >= 5900 && cswOutput < 6100) {
        // Position 11: 01011 (R3+R4+R11 ON)
        usingSolarPower=false;
    } else if (cswOutput >= 5700 && cswOutput < 5900) {
        // Position 12: 01100 (R10+R11 ON)
        usingSolarPower=false;
    } else if (cswOutput >= 5450 && cswOutput < 5700) {
        // Position 13: 01101 (R3+R10+R11 ON)
        usingSolarPower=false;
    } else if (cswOutput >= 5200 && cswOutput < 5450) {
        // Position 14: 01110 (R4+R10+R11 ON)
        usingSolarPower=false;
    } else if (cswOutput >= 4900 && cswOutput < 5200) {
        // Position 15: 01111 (R3+R4+R10+R11 ON)
        usingSolarPower=false;
    } else if (cswOutput >= 4800 && cswOutput < 4900) {
        // Position 16: 10000 (R13 ON)
        digitalStablesData.currentFunctionValue = FUN_1_FLOW;
        attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
        secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
        usingSolarPower=true;
    } else if (cswOutput >= 4500 && cswOutput < 4800) {
         // Position 17: 10001 (R3+R13 ON)
          digitalStablesData.currentFunctionValue = FUN_2_FLOW;
          attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
          attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);
          secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
          secretManager.readFlow2Name().toCharArray(digitalStablesData.sensor2name, 12);
         usingSolarPower=true;
    } else if (cswOutput >= 4200 && cswOutput < 4500) {
        // Position 18: 10010 (R4+R13 ON)
         usingSolarPower=true;
          digitalStablesData.currentFunctionValue = FUN_1_FLOW_1_TANK;
          attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
      
          secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
          secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
    } else if (cswOutput >= 3900 && cswOutput < 4200) {
        // Position 19: 10011 (R3+R4+R13 ON)
         digitalStablesData.currentFunctionValue = FUN_1_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
         usingSolarPower=true;
    } else if (cswOutput >= 3700 && cswOutput < 3900) {
         // Position 20: 10100 (R10+R13 ON)
         digitalStablesData.currentFunctionValue = FUN_2_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
         usingSolarPower=true;
    } else if (cswOutput >= 3400 && cswOutput < 3700) {
         // Position 21: 10101 (R3+R10+R13 ON)
           digitalStablesData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;
 
         usingSolarPower=true;
    } else if (cswOutput >= 3100 && cswOutput < 3400) {
         // Position 22: 10110 (R4+R10+R13 ON)
           digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
 
         usingSolarPower=true;
    } else if (cswOutput >= 2800 && cswOutput < 3100) {
         // Position 23: 10111 (R3+R4+R10+R13 ON)
            digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
 
         usingSolarPower=true;
    } else if (cswOutput >= 2400 && cswOutput < 2800) {
         // Position 24: 11000 (R11+R13 ON)
         usingSolarPower=true;
    } else if (cswOutput >= 2000 && cswOutput < 2400) {
         // Position 25: 11001 (R3+R11+R13 ON)
         usingSolarPower=true;
    } else if (cswOutput >= 1700 && cswOutput < 2000) {
        // Position 26: 11010 (R4+R11+R13 ON)
         usingSolarPower=true;
    } else if (cswOutput >= 1380 && cswOutput < 1700) {
        // Position 27: 11011 (R3+R4+R11+R13 ON)
         usingSolarPower=true;
    } else if (cswOutput >= 1100 && cswOutput < 1380) {
        // Position 28: 11100 (R10+R11+R13 ON)
         usingSolarPower=true;
    } else if (cswOutput >= 700 && cswOutput < 1100) {
         // Position 29: 11101 (R3+R10+R11+R13 ON)
         usingSolarPower=true;
    } else if (cswOutput >= 350 && cswOutput < 700) {
        // Position 30: 11110 (R4+R10+R11+R13 ON)
         usingSolarPower=true;
    } else if (cswOutput < 350) {
         // Position 31: 11111 (All ON) - assuming this is very low
         usingSolarPower=true;
    } 

  
  
  if (cswOutput >= 7100)
  {
    digitalStablesData.currentFunctionValue = FUN_1_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
  }
  else if (cswOutput >= 6700 && cswOutput < 7100)
  {
    digitalStablesData.currentFunctionValue = FUN_2_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readFlow2Name().toCharArray(digitalStablesData.sensor2name, 12);
  }
  else if (cswOutput >= 6300 && cswOutput < 6700)
  {
    digitalStablesData.currentFunctionValue = FUN_1_FLOW_1_TANK;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);

    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
  }
  else if (cswOutput >= 5800 && cswOutput < 6300)
  {
    digitalStablesData.currentFunctionValue = FUN_1_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
  }
  else if (cswOutput >= 5500 && cswOutput < 5800)
  {
    digitalStablesData.currentFunctionValue = FUN_2_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
  }
  else if (cswOutput >= 5100 && cswOutput < 5500)
  {
    digitalStablesData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;
  }
  else if (cswOutput >= 4600 && cswOutput < 5100)
  {
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
  }
  else if (cswOutput >= 4300 && cswOutput < 4600)
  {
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
  }
  else if (cswOutput >= 3400 && cswOutput < 4200)
  {
   
  }
   else if ( cswOutput < 1000)
  {
     digitalStablesData.currentFunctionValue = VOLTAGE_MONITOR;
    usingSolarPower=false;
    if(debug)Serial.println("SELECTED VOLTAGE MONITORING");
  }
  else
  {
    digitalStablesData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;
    lcd.setCursor(0, 3);
    lcd.print("bad data sceptic is def");
  }

 if(usingSolarPower){
    if(hourlySolarPowerData.efficiency*100>digitalStablesData.minimumEfficiencyForLed){
      digitalWrite(LED_CONTROL, HIGH);
      digitalStablesData.operatingStatus = OPERATING_STATUS_FULL_MODE;
    }else{
      digitalWrite(LED_CONTROL, LOW);
      digitalStablesData.operatingStatus = OPERATING_STATUS_NO_LED;
    }
  }else{
    digitalWrite(LED_CONTROL, HIGH);
    digitalStablesData.operatingStatus = OPERATING_STATUS_FULL_MODE;
  }

  tempSensor.begin();
  tempSensor.setWaitForConversion(false); // Don't block during conversion 
  tempSensor.setResolution(9);
  
  // uint8_t address[8];
  tempSensor.getAddress(digitalStablesData.serialnumberarray, 0);
//  for (uint8_t i = 0; i < 8; i++)
//  {
//    serialNumber += String(digitalStablesData.serialnumberarray[i], HEX);
//    digitalStablesData.checksum += static_cast<uint8_t>(digitalStablesData.serialnumberarray[i]);
//  }
//  digitalStablesData.checksum &= 0xFF;
  if(debug)Serial.print("serial number:");
  if(debug)Serial.println(serialNumber);

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
  if (!LoRa.begin(433E6))
  {
   // Serial.println("Starting LoRa failed!");
   // drawLora(false);
    while (1)
      ;
  //  leds[1] = CRGB(255, 0, 0);
  }
  else
  {
  //  Serial.println("Starting LoRa worked!");
   // drawLora(true);
    loraActive = true;
    // Configure LoRa parameters
   // LoRa.setSpreadingFactor(12);
   // LoRa.setSignalBandwidth(125E3);
   // LoRa.setCodingRate4(8);
    if(powerManager->isLoraTxSafe(9,currentTimerRecord)!=powerManager->LORA_TX_NOT_ALLOWED)loraTxOk=true;
  }

  // delay(2000);

  String devicename = secretManager.readDeviceName();
  char devicenamearray[devicename.length()];
  devicename.toCharArray(devicenamearray, devicename.length());
  strcpy(digitalStablesData.devicename, devicenamearray);

  String deviceshortname = secretManager.readDeviceShortName();
 // Serial.print("deviceshortname=");
 // Serial.println(deviceshortname);
  char deviceshortnamearray[deviceshortname.length()];
  deviceshortname.toCharArray(deviceshortnamearray, deviceshortname.length());
  strcpy(digitalStablesData.deviceshortname, deviceshortnamearray);

 
   String grp = secretManager.getGroupIdentifier();
   char gprid[8];
   grp.toCharArray(gprid, grp.length());
   strcpy(digitalStablesData.groupidentifier, gprid);

   if(debug)Serial.print("Starting wifi digitalStablesConfigData.groupidentifier=");
   if(debug)Serial.println(digitalStablesData.groupidentifier);

  String identifier = "daffodilTF";
  char ty[identifier.length() + 1];
  identifier.toCharArray(ty, identifier.length() + 1);
  strcpy(digitalStablesData.deviceTypeId, ty);

  digitalStablesConfigData.fieldId = secretManager.getFieldId();
  if(debug)Serial.print("Starting wifi digitalStablesConfigData.fieldId=");
  if(debug)Serial.println(digitalStablesConfigData.fieldId);

  pinMode(RTC_BATT_VOLT, INPUT);
   adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(RTC_BATT_VOLT, ADC_ATTEN_DB_11);  // 0-3.1V range

  opmode = digitalRead(OP_MODE);
  digitalStablesData.opMode = opmode;

  dsUploadTimer.start();
  digitalStablesData.dataSamplingSec = 10;
  if(debug)Serial.print("digitalStablesData.dataSamplingSec=");
  if(debug)Serial.println(digitalStablesData.dataSamplingSec);
  if(debug)Serial.print("digitalStablesData size=");
  if(debug)Serial.println(sizeof(digitalStablesData));

  
  //  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  //  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  lastswitchmillis = millis();

  viewTimer.start();
  remoteMonitorTimer.start();
  
  LoRa.setSyncWord(0xF3);
  //pinMode(WATCHDOG_WDI, OUTPUT);
 // Configure TPL5010 pins
  pinMode(TPL5010_DONE, OUTPUT);
  digitalWrite(TPL5010_DONE, LOW);
  pinMode(TPL5010_WAKE, INPUT);

  attachInterrupt(digitalPinToInterrupt(TPL5010_WAKE), handleWakeInterrupt, RISING);
  xTaskCreatePinnedToCore(
      refreshTPL5010Task, /* Task function. */
      "TPL5010Refresh",       /* name of task. */
      2048,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      1,                    /* priority of the task */
      &refreshTaskHandle,        /* Task handle to keep track of created task */
      0);
  digitalWrite(WATCHDOG_WDI, LOW);

  boolean isSleepMode=false;
  if(usingSolarPower && hourlySolarPowerData.efficiency*100<digitalStablesData.minimumEfficiencyForLed){
    isSleepMode=true;
    digitalStablesData.operatingStatus=OPERATING_STATUS_SLEEP;
    if(debug)Serial.print("setting sleepmode in setup because of efficiency=");
    if(debug)Serial.println(hourlySolarPowerData.efficiency);
  
    digitalStablesData.asyncdata = 2;
    if(dataManager.getDSDStoredCount()<MAXIMUM_STORED_RECORDS){
      dataManager.storeDSDData(digitalStablesData);
    }
  
  }
  if (usingSolarPower && digitalStablesData.capacitorVoltage > 1 && digitalStablesData.capacitorVoltage < sleepingVoltage){
    if(debug)Serial.print("setting sleepmode in setup because of low voltage=");
    if(debug)Serial.println(hourlySolarPowerData.efficiency);
    isSleepMode=true;
    digitalStablesData.operatingStatus=OPERATING_STATUS_SLEEP;
    digitalStablesData.asyncdata = 3;
    if(dataManager.getDSDStoredCount()<MAXIMUM_STORED_RECORDS){
      dataManager.storeDSDData(digitalStablesData);
    }
  }

 
  if(isSleepMode){
    ;
  }else{
    digitalStablesData.operatingStatus=OPERATING_STATUS_NO_LED;
    if (loraActive) {
      // LoRa_rxMode();
      // LoRa.setSyncWord(0xF3);
      LoRa.onReceive(onReceive);
      // put the radio into receive mode
      LoRa.receive();
    }
  }
   if(debug)Serial.println(F("Finished Setup"));
}

void sleepDS18B20() { // Put OneWire bus in high impedance state pinMode(ONE_WIRE_BUS, INPUT);

  // Force DS18B20 to stop any conversion
  oneWire.reset();
  oneWire.skip();
  oneWire.write(0x44);  // Start conversion command
  oneWire.reset();      // Reset to stop conversion
} 



void goToSleep(){
   
     long seconds_sleep= powerManager->calculateOptimalSleepTime(currentTimerRecord);
     uint64_t sleep_time_us = (uint64_t)(seconds_sleep * 1000000ULL);
    if(debug)Serial.print("sleep_time_us=");
    if(debug)Serial.print(sleep_time_us);
   
     digitalStablesData.sleepTime=seconds_sleep;
    readSensorData();
    digitalStablesData.ledBrightness=powerManager->isLoraTxSafe(0,currentTimerRecord);
    digitalStablesData.operatingStatus=OPERATING_STATUS_SLEEP;
  digitalStablesData.asyncdata=4;
if(dataManager.getDSDStoredCount()<MAXIMUM_STORED_RECORDS){
    dataManager.storeDSDData(digitalStablesData);
  }
  
    
  //   Serial.print("in gotosleep digitalStablesData.ledBrightness=");
   // Serial.print(digitalStablesData.ledBrightness);
    
    if(debug)Serial.print("  digitalStablesData.sleepTime=");
    if(debug)Serial.println(digitalStablesData.sleepTime);
    //  adc_power_off();
      adc_power_acquire();  // Ensure ADC is powered (required for release)
  adc_power_release();  // Power down ADC
    digitalWrite(LED_CONTROL, LOW);
     WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  // Disable Bluetooth
  btStop();
 // adc_power_off();
      sleepDS18B20();
    if(digitalStablesData.ledBrightness!=powerManager->LORA_TX_NOT_ALLOWED){
       
      digitalStablesData.asyncdata = 5;
      if(dataManager.getDSDStoredCount()<MAXIMUM_STORED_RECORDS){
        dataManager.storeDSDData(digitalStablesData);
      }

      
      sendMessage(digitalStablesData);
      delay(500);
     // sendMessage(digitalStablesData);
    }
     LoRa.sleep();
    
      pinMode(LORA_RESET, INPUT);
   ;
    
//Serial.print("about to set runwadopg to false");
//     runWatchdog=false;
     
     const int gpioOutputPins[] = {
         4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 27, 32, 33
      };
     //const int numPins = sizeof(gpioOutputPins) / sizeof(gpioOutputPins[0]);
     for(int i = 0; i < 17; i++) {
      pinMode(gpioOutputPins[i], INPUT);
    }
 // disabling task
 
//  // Try explicitly stopping the watchdog task
//   Serial.println(" about to disable task");
//    if(watchdogTask != NULL) {
//        vTaskDelete(watchdogTask);
//        watchdogTask = NULL;
//    }
    if(debug)Serial.println(" about to put ads1115 to sleep ");
    ADS.setMode(1);   
    
     if(debug)Serial.println("  going to sleep");
     lcd.setCursor(0, 0);
      lcd.clear();
      lcd.print("sleeping ");
      lcd.print(sleep_time_us);
  
 
   // esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
 //   esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
  //  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
 //   esp_sleep_enable_timer_wakeup(sleep_time_us);
    digitalWrite(SLEEP_SWITCH_26, LOW);
    delay(100);
    pinMode(SLEEP_SWITCH_26, INPUT);
    delay(50);
    esp_deep_sleep_start();
}

// void watchdogTaskFunction(void *pvParameters)
// {
//   if(debug)Serial.print("watchdogTask running on core ");
//   if(debug)Serial.println(xPortGetCoreID());
//   TickType_t xLastWakeTime = xTaskGetTickCount();
//   const TickType_t xFrequency = pdMS_TO_TICKS(900);
    
// //  for (;;)
// //  {
//  while (runWatchdog) // Changed from for(;;) to while(runWatchdog)
//     {
//     digitalWrite(WATCHDOG_WDI, HIGH);
//       vTaskDelay(pdMS_TO_TICKS(2));  
//     digitalWrite(WATCHDOG_WDI, LOW);
//    // if(debug)Serial.print("watchdogTask reset ");
//    vTaskDelayUntil(&xLastWakeTime, xFrequency);
//   }
//    digitalWrite(WATCHDOG_WDI, LOW);  // Ensure WDI is low before ending task
//     vTaskDelete(NULL);  // Delete this task
// }

   void refreshTPL5010Task(void *parameter) {
  while (true) {
    // Wait for the wake signal to be acknowledged by the main loop
    if (!wakeSignalReceived) {
      // Send a pulse on the DONE pin to tell TPL5010 we're alive
      digitalWrite(TPL5010_DONE, HIGH);
      delayMicroseconds(100); // Pulse width needs to be at least 20µs
      digitalWrite(TPL5010_DONE, LOW);
      
     // if(debug)Serial.println("Sent DONE signal to TPL5010");
    }
    
    // Wait before checking again
    vTaskDelay(pdMS_TO_TICKS(1500)); // Check every 1500ms
  }
} 
void handleWakeInterrupt() {
  wakeSignalReceived = true;
} 

void readSensorData(){
    //
    // Capacitor Voltage
    //
    ADS.setGain(0);
    int16_t val_3 = ADS.readADC(3);
    float f = ADS.toVoltage(1); //  voltage factor
    digitalStablesData.capacitorVoltage = val_3 * f; 

    if(digitalStablesData.currentFunctionValue==VOLTAGE_MONITOR){
      // read the voltage on sensor 1 ie pin 32
      int16_t val_1 = ADS.readADC(1);
      float f = ADS.toVoltage(1); //  voltage factor
      digitalStablesData.flowRate = val_1 * f; 
    }
    
    tempSensor.requestTemperatures();
    float tempC = tempSensor.getTempCByIndex(0);
    digitalStablesData.temperature = tempC;
    readI2CTemp();
    digitalStablesData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
    digitalStablesData.sleepTime = powerManager->calculateOptimalSleepTime(currentTimerRecord);
    if(foundBH1750){
//      // http://community.heltec.cn/t/bh1750-light-sensor-practical-notes-problems-and-issues/1521
//      double cal=1.13;
//      //digitalStablesData.lux = ((lightMeter.readLightLevel()/cal)/2.5);

      // the value of correcting factor was obtained by comparing the output to the output of the Davies Vantage Pro 2 sensor
      // which it is assume that is correct.3.45
     
      digitalStablesData.lux = lightMeter.readLightLevel()*lightMeterCorrectingFactor;
    }else{
      digitalStablesData.lux = -99;
    }
//    if(debug)Serial.print("lux=");
//    if(debug)Serial.println(digitalStablesData.lux);
    
    digitalStablesData.ledBrightness=powerManager->isLoraTxSafe(9,currentTimerRecord);
    float distance = sonar.ping_cm();
    digitalStablesData.measuredHeight = distance;
    digitalStablesData.scepticAvailablePercentage = distance * 100 / MAX_DISTANCE;
    //
    // RTC_BATT_VOLT Voltage
    //

    float total = 0;
    uint8_t samples = 10;
    for (int x = 0; x < samples; x++)
    {                                            // multiple analogue readings for averaging
      total = total + adc1_get_raw(RTC_BATT_VOLT); // add each value to a total
      delay(1);
    }
    float average = total / samples;
    float voltage = (average / 4095.0) * Vref;
    // Calculate the actual voltage using the voltage divider formula
   // float rtcBatVoltage = (voltage * (R1 + R2)) / R2;
    digitalStablesData.rtcBatVolt = (voltage * (R1 + R2)) / R2;
//
// current
//
// Read raw shunt voltage register for debugging
if(foundINA219){
   Wire.beginTransmission(0x40);
  Wire.write(0x01); // Shunt voltage register
  Wire.endTransmission();
  Wire.requestFrom(0x40, 2);
  int16_t rawShunt = (Wire.read() << 8) | Wire.read();
  
  Wire.beginTransmission(0x40);
  Wire.write(0x04); // Current register
  Wire.endTransmission();
  Wire.requestFrom(0x40, 2);
  int16_t rawCurrent = (Wire.read() << 8) | Wire.read();
  
  // Get readings using library functions
  float shuntvoltage = ina219.getShuntVoltage_mV();
  float busvoltage = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  
  // Calculate current directly from shunt voltage for comparison
  // I = V/R (Ohm's Law)
  float calculated_current_mA = shuntvoltage / SHUNT_OHMS;
  // Calculate current from raw register and calibration
  float direct_current_mA = rawCurrent * CURRENT_LSB * 1000;
  
 if(debug)Serial.println("--- MEASUREMENTS ---");
 if(debug)Serial.print("Raw Shunt Register: 0x");
  if(debug)Serial.print(rawShunt, HEX);
 if(debug)Serial.print(" ("); 
 if(debug)Serial.print(rawShunt); 
 if(debug)Serial.println(")");
 if(debug)Serial.print("Raw Current Register: 0x"); 
 if(debug)Serial.print(rawCurrent, HEX);
if(debug) Serial.print(" ("); 
if(debug)Serial.print(rawCurrent); 
if(debug)Serial.println(")");
 
if(debug) Serial.print("Bus Voltage: "); 
if(debug)Serial.print(busvoltage); 
if(debug)Serial.println(" V");
 if(debug)Serial.print("Shunt Voltage: "); 
 if(debug)Serial.print(shuntvoltage); 
 if(debug)Serial.println(" mV");
 
 if(debug)Serial.print("Library Current: "); 
 if(debug)Serial.print(current_mA); 
 if(debug)Serial.println(" mA");
 if(debug)Serial.print("Calculated Current (V/R): "); 
 if(debug)Serial.print(calculated_current_mA); 
 if(debug)Serial.println(" mA");
if(debug) Serial.print("Direct Register Current: "); 
if(debug)Serial.print(direct_current_mA); 
if(debug)Serial.println(" mA");
 
if(debug) Serial.print("Power: "); 
if(debug)Serial.print(power_mW); 
if(debug)Serial.println(" mW");
 if(debug)Serial.println("");
  
  digitalStablesData.solarVoltage = busvoltage; 
  digitalStablesData.capacitorCurrent = calculated_current_mA;
}else{
  digitalStablesData.solarVoltage = -99; 
  digitalStablesData.capacitorCurrent = -99;
}
  
/*
 * 
 *    float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  */
}

void restartWifi()
{
  //FastLED.setBrightness(50);
  for (int i = 0; i < NUM_LEDS; i++)
  {
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
  if (!initiatedWifi)
  {

    leds[7] = CRGB(255, 0, 255);
    FastLED.show();
   // Serial.print(F("Before Starting Wifi cap="));
   // Serial.println(digitalStablesData.capacitorVoltage);
    wifiManager.start();
    initiatedWifi = true;
  }
  if(debug)Serial.println("Starting wifi");

  wifiManager.restartWifi();
  //      digitalWrite(WATCHDOG_WDI, HIGH);
  //    delay(2);
  //    digitalWrite(WATCHDOG_WDI, LOW);
  if(debug)Serial.println("getting  stationmode=");
  bool stationmode = wifiManager.getStationMode();
  digitalStablesData.internetAvailable = wifiManager.getInternetAvailable();
  //     digitalWrite(WATCHDOG_WDI, HIGH);
  //    delay(2);
  //    digitalWrite(WATCHDOG_WDI, LOW);
  if(debug)Serial.print("Starting wifi stationmode=");
 // Serial.println(stationmode);
 // Serial.print("digitalStablesData.internetAvailable=");
 // Serial.println(digitalStablesData.internetAvailable);

  //  serialNumber = wifiManager.getMacAddress();
  wifiManager.setSerialNumber(serialNumber);
  wifiManager.setLora(loraActive);
  String ssid = wifiManager.getSSID();
  String ipAddress = "";
  uint8_t ipi;
  if (stationmode)
  {
    ipAddress = wifiManager.getIpAddress();
 //   Serial.print("ipaddress=");
  //  Serial.println(ipAddress);

    if (ipAddress == "" || ipAddress == "0.0.0.0")
    {

      setApMode();
    }
    else
    {
      setStationMode(ipAddress);
    }
  }
  else
  {
    setApMode();
  }
  //    digitalWrite(WATCHDOG_WDI, HIGH);
  //    delay(2);
  //    digitalWrite(WATCHDOG_WDI, LOW);

  digitalStablesData.loraActive = loraActive;
  uint8_t ipl = ipAddress.length() + 1;
  char ipa[ipl];
  ipAddress.toCharArray(ipa, ipl);
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
 // Serial.println("in ino Done starting wifi");
}

void drawError(uint8_t code)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();

    leds[0] = CRGB(255,0, 0);
    leds[1] = CRGB(255,0, 0);
    leds[2] = CRGB(255,0, 0);
    leds[5] = CRGB(255, 0, 0);
    leds[6] = CRGB(255, 0, 0);
    leds[10] = CRGB(255, 0, 0);
    leds[11] = CRGB(255, 0, 0);
    leds[12] = CRGB(255, 0, 0);

    // for the error is 4, 9, 13
    switch(code){
      case 0:
        leds[4] = CRGB(0, 0, 255);
        break;
      case 1:
        leds[9] = CRGB(0, 0, 255);
        break;
      case 2:
        leds[13] = CRGB(0, 0, 255);
        break;
      
    }
  FastLED.show();
}

void drawLora(boolean active)
{
  for (int i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  if (active)
  {
    leds[1] = CRGB(0, 255, 0);
    leds[6] = CRGB(0, 255, 0);
    leds[11] = CRGB(0, 255, 0);
    leds[12] = CRGB(0, 255, 0);
    // leds[13] = CRGB(0, 255, 0);
  }
  else
  {
    leds[1] = CRGB(255, 0, 0);
    leds[6] = CRGB(255, 0, 0);
    leds[11] = CRGB(255, 0, 0);
    leds[12] = CRGB(255, 0, 0);
    // leds[13] = CRGB( 255,0, 0);
  }
  FastLED.show();
}

void drawTemperature(uint8_t red, uint8_t green, uint8_t blue)
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
  if(debug)Serial.print("hd=");
  if(debug)Serial.print(hd);
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
  if(debug)Serial.print("ld=");
 if(debug) Serial.println(ld);

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

//  Serial.print("CHT8305\t");
//  //  DISPLAY DATA, sensor has only one decimal.
//  Serial.print(F("  Humidity"));

  temperature = CHT.getTemperature();

  digitalStablesData.outdoortemperature = temperature;
//  Serial.print(" Temp:");
//  Serial.print(temperature, 1);
  digitalStablesData.outdoorhumidity = CHT.getHumidity();
//  Serial.print(" Hum:");
//  Serial.println(digitalStablesData.outdoorhumidity, 1);

  switch (status)
  {
  case CHT8305_OK:
   // Serial.print("OK");
    break;
  case CHT8305_ERROR_ADDR:
    Serial.print("Address error");
    break;
  case CHT8305_ERROR_I2C:
   // Serial.print("Outdoor Temperature I2C error");
    digitalStablesData.outdoortemperature = -99;
    break;
  case CHT8305_ERROR_CONNECT:
    Serial.print("Connect error");
    break;
  case CHT8305_ERROR_LASTREAD:
   // Serial.print("Last read error");
    break;
  default:
    Serial.print("Unknown error");
    break;
  }
  if(debug)Serial.print("\n");
}

void loop()
{
  uint16_t dscount;
  boolean turnOffWifi= false;
   bool wifistatus=wifiManager.getWifiStatus();
  if (clockTicked)
  {

    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
    currentTimerRecord = timeManager.now();
    
    //
    // generate codes so that the history is refresed
    //
    if (currentTimerRecord.second == 0)
    {
       

      if (currentTimerRecord.minute == 0)
      {
        //  Serial.println(F("New Hour"));
        if (currentTimerRecord.hour == 0)
        {
          //    Serial.println(F("New Day"));
        }
      }
    }

if(currentTimerRecord.second%10 == 0){
  secretManager.generateCode();
//   Serial.print("update code history: ");
  //  long* history = secretManager.getCommandCodeHistory();
  // for (int i = 0; i < 5; i++) {
  //   Serial.print(i);
  //   Serial.print(": ");
  //   Serial.println(history[i]);
  // }
}
  if (wakeSignalReceived) {
    // Calculate time since last wake
    unsigned long timeSinceLastWake = millis() - lastWakeTime;
    lastWakeTime = millis();
    
    if(debug)Serial.print("Wake signal received! Time since last wake: ");
    if(debug)Serial.print(timeSinceLastWake);
    if(debug)Serial.println(" ms");
    // Reset the flag
    wakeSignalReceived = false;
    
  }

if(loraReceived){
  loraReceived=false;
  processLora(loraPacketSize);
}
    secondsSinceLastDataSampling++;
    //   Serial.println("ticked");

    viewTimer.tick();
    remoteMonitorTimer.tick();
    hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);

    //
    // read the sensors
    //
    
    readSensorData();    

     int dsdStoredCount=dataManager.getDSDStoredCount();
    if(dsdStoredCount>MAXIMUM_STORED_RECORDS){
      memoryFull=true;
    }else{
      memoryFull=false;
    }
    
      
      if(remoteMonitorTimer.status()){
         remoteMonitorTimer.reset();
         if(digitalStablesData.currentFunctionValue==VOLTAGE_MONITOR &&
            currentTimerRecord.hour>=16 &&
            !memoryFull
          ){
            digitalStablesData.asyncdata=6;
            dataManager.storeDSDData(digitalStablesData);
            remoteMonitorTimer.reset();
        }
      }
    
    
     // if(debug)Serial.print("External Voltage=");
    //  if(debug)Serial.println(digitalStablesData.flowRate);
      
    if (digitalStablesData.capacitorVoltage > minimumInitWifiVoltage && !wifistatus)
    {
      currentSecondsWithWifiVoltage++;
    }
    else
    {
      currentSecondsWithWifiVoltage = 0;
    }
    
    dscount = dsUploadTimer.tick();
    HourlySolarPowerData hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);
    if(usingSolarPower){
      if(hourlySolarPowerData.efficiency*100>digitalStablesData.minimumEfficiencyForLed){
        digitalWrite(LED_CONTROL, HIGH);
        digitalStablesData.ledBrightness=powerManager->isLoraTxSafe(9,currentTimerRecord);
        digitalStablesData.operatingStatus = OPERATING_STATUS_FULL_MODE;
        digitalStablesData.asyncdata=7;
        dataManager.storeDSDData(digitalStablesData);
        turnOffWifi= (hourlySolarPowerData.efficiency*100<digitalStablesData.minimumEfficiencyForWifi ) && wifistatus;
      }else{
        FastLED.clear(true);
        for (int i = 0; i < NUM_LEDS; i++)
        {
          leds[i] = CRGB(0, 0, 0);
        }
        FastLED.show();
        digitalStablesData.ledBrightness=0;
        digitalWrite(LED_CONTROL, LOW);
        digitalStablesData.operatingStatus = OPERATING_STATUS_NO_LED;
        turnOffWifi=true;
        digitalStablesData.asyncdata=8;
        dataManager.storeDSDData(digitalStablesData);
      }
    }else{
      digitalWrite(LED_CONTROL, HIGH);
      digitalStablesData.ledBrightness=powerManager->isLoraTxSafe(9,currentTimerRecord);
      digitalStablesData.operatingStatus = OPERATING_STATUS_FULL_MODE;
      turnOffWifi=false;
    }

  
    
      
     

    if(debug)Serial.print("line 1990  hourlySolarPowerData.efficiency=");
      if(debug)Serial.print(hourlySolarPowerData.efficiency);
      if(debug)Serial.print(" wifistatus=");
      if(debug)Serial.println(wifistatus);
       if(debug)Serial.print(" turnOffWifi=");
      if(debug)Serial.println(turnOffWifi);
      
  } // end of the tick block


  boolean isSleepMode=false;
  if(usingSolarPower && hourlySolarPowerData.efficiency*100<digitalStablesData.minimumEfficiencyForLed)isSleepMode=true;
  if (usingSolarPower && digitalStablesData.capacitorVoltage > 1 && digitalStablesData.capacitorVoltage < sleepingVoltage) isSleepMode=true;
  if(isSleepMode){
    digitalStablesData.operatingStatus=OPERATING_STATUS_SLEEP;
    if(debug)Serial.print("going to sleep because digitalStablesData.capacitorVoltage is less than sleepingVoltage,, dscap=");
    if(debug)Serial.println(digitalStablesData.capacitorVoltage);
    
    goToSleep();
  }



  if (usingSolarPower && digitalStablesData.capacitorVoltage < minimumLEDVoltage && digitalRead(LED_CONTROL))
  {

    if(debug)Serial.print("line 967 turning off leds, cap=");
    if(debug)Serial.println(digitalStablesData.capacitorVoltage);
    FastLED.clear(true);
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
    digitalStablesData.ledBrightness=0;
    digitalWrite(LED_CONTROL, LOW);
    digitalStablesData.operatingStatus = OPERATING_STATUS_NO_LED;
  }
 
  
    lcd.print(" ");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(digitalStablesData.capacitorVoltage);
    lcd.print(" ");
    lcd.print(digitalStablesData.operatingStatus);
    lcd.print(" ");
    lcd.print(displayStatus);
    lcd.print(" L");
    lcd.print(loraLastResult); 
    lcd.print(" ");
    lcd.print(avgRssi);

    
    lcd.setCursor(0, 1);
    lcd.print(digitalStablesData.ledBrightness);
    lcd.print(" ");
    lcd.print(hourlySolarPowerData.efficiency*100);
    lcd.print(" ");
    lcd.print(digitalRead(LED_CONTROL));
    lcd.print(" ");
    lcd.print(cadResult);
    

    

    
     
  if(!turnOffWifi){
    if(debug)Serial.print("line 2055 turning off wifi");
    turnOffWifi=(digitalStablesData.capacitorVoltage < minimumWifiVoltage) && wifistatus && usingSolarPower;
  }
  if (turnOffWifi)
  {
    if(debug)Serial.print("turning off wifi cap voltage=");
    if(debug)Serial.println(digitalStablesData.capacitorVoltage);
    wifiManager.stop();

     if(debug)Serial.print("after wifimanager stop, eifistatus=");
    if(debug)Serial.println(wifiManager.getWifiStatus());
    
    digitalStablesData.internetAvailable = false;
    currentSecondsWithWifiVoltage = 0;
    FastLED.clear(true);
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
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
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
    FastLED.setBrightness(digitalStablesData.ledBrightness);
    leds[0] = CRGB(255, 255, 0);
    leds[14] = CRGB(255, 255, 0);
    FastLED.show();
  }
wifistatus = wifiManager.getWifiStatus();
    boolean turnOnWifi=false; 
    if(!usingSolarPower){
      if(!wifistatus)turnOnWifi=true;
    }else{
      turnOnWifi= (hourlySolarPowerData.efficiency*100>digitalStablesData.minimumEfficiencyForWifi) && (currentSecondsWithWifiVoltage >= numberSecondsWithMinimumWifiVoltageForStartWifi) && !wifistatus; 
    }
    
    if (turnOnWifi)
    {
      if(debug)Serial.print("turning on  wifi");
      restartWifi();
      wifistatus = wifiManager.getWifiStatus();
    }
    // Serial.println("line 1121");
    uint8_t red = 255;
    uint8_t green = 255;
    uint8_t blue = 255;

    // Check if weather data is stale
    boolean staledata = weatherForecastManager->isWeatherDataStale(currentTimerRecord);
    // if(debug)Serial.print("l;ine 1128 staledata=");
    //  if(debug)Serial.println(staledata);
    if (staledata && wifistatus && !wifiManager.getAPStatus())
    {
      // Fetch weather data and update SolarInfo
      weatherForecastManager->downloadWeatherData(solarInfo);
    }
    // if(debug)Serial.println("line 1139");
    boolean showError=false;
    if (viewTimer.status())
    {
      showTemperature = !showTemperature;
      
     if(debug)Serial.print("capacitor voltage=");
     if(debug)Serial.print(digitalStablesData.capacitorVoltage);
     if(debug)Serial.print(" displayStatus=");
     if(debug)Serial.println(displayStatus);
     if(debug)Serial.print("line 1287 digitalStablesData.ledBrightness=");
     if(debug)Serial.print(digitalStablesData.ledBrightness);
     if(debug)Serial.print("  digitalRead(LED_CONTROL)=");
     if(debug)Serial.println(digitalRead(LED_CONTROL));

      if(digitalStablesData.ledBrightness==powerManager->LORA_TX_NOT_ALLOWED){
        loraTxOk=false;
      }else {
        loraTxOk=true; 
      }
    
      FastLED.setBrightness(digitalStablesData.ledBrightness); 
    
      
      if (displayStatus == SHOW_TEMPERATURE)
      {

        if(debug)Serial.print("showing temperature=");
        

        if (digitalStablesData.outdoortemperature == -99)
        {
          for (int i = 0; i < NUM_LEDS; i++)
          {
            leds[i] = CRGB(0, 0, 0);
          }
          FastLED.show();
          leds[1] = CRGB(255, 0, 0);
          leds[2] = CRGB(255, 0, 0);
          leds[3] = CRGB(255, 0, 0);
          leds[7] = CRGB(255, 0, 0);
          leds[12] = CRGB(255, 0, 0);
          FastLED.show();
        }
        else
        {
          if (digitalStablesData.outdoortemperature > 0)
          {
            red = 0;
            green = 0;
            blue = 255;
          }
          else if (digitalStablesData.outdoortemperature <= 0)
          {
            red = 255;
            green = 0;
            blue = 0;
          }
          drawTemperature(red, green, blue);
        }
      }
      else if (displayStatus == SHOW_SCEPTIC)
      {
        if(debug)Serial.print("showing scepotic=");
        if(debug)Serial.print(" percentage: ");
        if(debug)Serial.println(digitalStablesData.scepticAvailablePercentage);
        red = 255;
        green = 0;
        blue = 255;
            
         for (int i = 0; i < NUM_LEDS; i++)
          {
            leds[i] = CRGB(0, 0, 0);
          }
          FastLED.show();
        if (digitalStablesData.currentFunctionValue == DAFFODIL_SCEPTIC_TANK)
        {
          if (digitalStablesData.scepticAvailablePercentage <= 25)
          {
            red = 255;
            green = 0;
            blue = 0;
          }
          else if (digitalStablesData.scepticAvailablePercentage > 25 && digitalStablesData.scepticAvailablePercentage <= 50)
          {
            red = 255;
            green = 255;
            blue = 0;
          }
          else if (digitalStablesData.scepticAvailablePercentage > 50 && digitalStablesData.scepticAvailablePercentage <= 75)
          {
            red = 0;
            green = 255;
            blue = 0;
          }
          else if (digitalStablesData.scepticAvailablePercentage > 75)
          {
            red = 0;
            green = 0;
            blue = 255;
          }
        }
        else if (digitalStablesData.currentFunctionValue == DAFFODIL_WATER_TROUGH)
        {
          if (digitalStablesData.scepticAvailablePercentage <= 25)
          {
            red = 255;
            green = 0;
            blue = 0;
          }
          else if (digitalStablesData.scepticAvailablePercentage > 25 && digitalStablesData.scepticAvailablePercentage <= 50)
          {
            red = 255;
            green = 255;
            blue = 0;
          }
          else if (digitalStablesData.scepticAvailablePercentage > 50 && digitalStablesData.scepticAvailablePercentage <= 75)
          {
            red = 0;
            green = 255;
            blue = 0;
          }
          else if (digitalStablesData.scepticAvailablePercentage > 75)
          {
            red = 0;
            green = 0;
            blue = 255;
          }
        }else if(digitalStablesData.currentFunctionValue==VOLTAGE_MONITOR){
             int dsdStoredCount = dataManager.getDSDStoredCount();
            if ( (dsdStoredCount*100/ MAXIMUM_STORED_RECORDS) <= 25)
          {
            red = 255;
            green = 0;
            blue = 0;
          }
          else if ((dsdStoredCount*100/ MAXIMUM_STORED_RECORDS) > 25 && (dsdStoredCount*100/ MAXIMUM_STORED_RECORDS) <= 50)
          {
            red = 255;
            green = 255;
            blue = 0;
          }
          else if ((dsdStoredCount*100/ MAXIMUM_STORED_RECORDS) > 50 && (dsdStoredCount*100/ MAXIMUM_STORED_RECORDS) <= 75)
          {
            red = 0;
            green = 255;
            blue = 0;
          }
          else if ((dsdStoredCount*100/ MAXIMUM_STORED_RECORDS) > 75)
          {
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
      }
      else if (displayStatus == SHOW_INTERNET_STATUS)
      {
       wifistatus= wifiManager.getWifiStatus();
       if(debug)Serial.print("line 1112 inside of showintenrnetstatus internetAvailable=");
       if(debug)Serial.println(digitalStablesData.internetAvailable);
       if(debug)Serial.print("wifiManager.getAPStatus()=");
       if(debug)Serial.println(wifiManager.getAPStatus());
       if(debug)Serial.print("wifiManager.getWifiStatus()=");
       if(debug)Serial.println(wifistatus);
       if(debug)Serial.print("dsupload timer counter= ");
       if(debug)Serial.println(dscount);
       
      
       
         for (int i = 0; i < NUM_LEDS; i++)
          {
            leds[i] = CRGB(0, 0, 0);
          }
          FastLED.show();

          boolean displayWifi=false;
          if(!usingSolarPower)displayWifi=true;
          else{
            displayWifi=wifistatus && (hourlySolarPowerData.efficiency*100>digitalStablesData.minimumEfficiencyForWifi);
          }
        if (displayWifi)
        {
          if (wifiManager.getAPStatus())
          {
            leds[1] = CRGB(0, 255, 0);
            leds[2] = CRGB(0, 255, 0);
            leds[3] = CRGB(0, 255, 0);
            leds[5] = CRGB(0, 255, 0);
            leds[9] = CRGB(0, 255, 0);
            leds[11] = CRGB(0, 255, 0);
            leds[12] = CRGB(0, 255, 0);
            leds[13] = CRGB(0, 255, 0);
            FastLED.show();
          }
          else
          {
            leds[1] = CRGB(0, 0, 255);
            leds[2] = CRGB(0, 0, 255);
            leds[3] = CRGB(0, 0, 255);
            leds[5] = CRGB(0, 0, 255);
            leds[9] = CRGB(0, 0, 255);
            leds[11] = CRGB(0, 0, 255);
            leds[12] = CRGB(0, 0, 255);
            leds[13] = CRGB(0, 0, 255);
            if (digitalStablesData.internetAvailable)
            {
              leds[7] = CRGB(0, 0, 255);
            }
            else
            {
              leds[7] = CRGB(255, 0, 0);
            }
            FastLED.show();
            //
            //

            if (digitalStablesData.internetAvailable)
            {
              if (dsUploadTimer.status())
              {
                // char secret[27];
                //    lcd.print("Uploading to digitalstables");
                String secret = "J5KFCNCPIRCTGT2UJUZFSMQK";
                //                    leds[2] = CRGB(0, 255, 0);
                //                    FastLED.show();
                TOTP totp = TOTP(secret.c_str());
                char totpCode[7]; // get 6 char code
                long timeVal = timeManager.getTimeForCodeGeneration(currentTimerRecord);
                if(debug)Serial.print("line 1153 timeVal=");
                if(debug)Serial.println(timeVal);
                digitalStablesData.secondsTime = timeVal;
                long code = totp.gen_code(timeVal);
                if(debug)Serial.print("l;ine 1154 totp=");
                if(debug)Serial.println(code);

                wifiManager.setCurrentToTpCode(code);
                int response = wifiManager.uploadDataToDigitalStables();
                if(debug)Serial.print("l;ine 1153 uploading to ds=");
                if(debug)Serial.println(response);
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
            }
            else
            {
              //
              // if we are here it means that there is an ipaddress
              // but internet is not available , check again if there is a reconnection
              wifiManager.checkInternetConnectionAvailable();

              digitalStablesData.internetAvailable = wifiManager.getInternetAvailable();
              if(debug)Serial.print("after rechecking digitalstable,internetConnectionAvailable=");
              if(debug)Serial.println(digitalStablesData.internetAvailable);
            }
          }
        }
        else
        {
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
      else if (displayStatus == SEND_LORA_STATUS)
      {
        if(debug)Serial.print("showing Lora Status loraActive=");
        if(debug)Serial.print( loraActive);
        if(debug)Serial.print(" loraTxOk=");
        if(debug)Serial.println(loraTxOk);
        if (loraActive && loraTxOk)
        {

          readSensorData();
          digitalStablesData.asyncdata=9;
          if(dataManager.getDSDStoredCount()<MAXIMUM_STORED_RECORDS){
            dataManager.storeDSDData(digitalStablesData);
          }
    
          loraLastResult = sendMessage(digitalStablesData);
          lcd.setCursor(10, 0);
          lcd.print(displayStatus);
          if(loraLastResult==LORA_TX_FAILED){
            drawLora(false);
          }else if(loraLastResult==LORA_OK){
            drawLora(true);
          }  
        }
        else
        {
          drawLora(false);
        }
      }else if (displayStatus == SHOW_ERROR_STATUS )
      {
        if(!foundADS){
           if(debug)Serial.println("showing error with ADS");
          drawError(0);
          showError=true;
        }else if( memoryFull){
           if(debug)Serial.println("memory full");
          drawError(1);
          showError=true;
        }else{
          displayStatus++; // to make sure that this is not displayed
          showError=false;
          
        }
      }
      displayStatus++;
      
      if (displayStatus > 5){
        displayStatus = 0;
      }
     
      if(!showError){
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
    else if(command.startsWith("goToSleep"))
    {
        goToSleep();
   
    }
     else if(command.startsWith("usingSolarPower"))
    {
      int usingSolarPowerv = generalFunctions.getValue(command, '#', 1).toInt();
        if(usingSolarPowerv>0)usingSolarPower=true;
        else usingSolarPower=false;
        Serial.println("Ok-usingSolarPower");
        Serial.flush(); 
    }
     else if(command.startsWith("debug"))
    {
      int debugv = generalFunctions.getValue(command, '#', 1).toInt();
        if(debugv>0)debug=true;
        else debug=false;
    Serial.println("Ok-debug");
        Serial.flush(); 
    }
    else if(command.startsWith("storeDSDData"))
    {
      digitalStablesData.asyncdata=10;
        int count = dataManager.storeDSDData(digitalStablesData);
        Serial.println(count);
        Serial.println("Ok-storeDSDData");
        Serial.flush(); 
    }
    else if (command.startsWith("SetFieldId"))
    {
      // fieldId= GeneralFunctions::getValue(command, '#', 1).toInt();
      
    }else if (command.startsWith("clearAllDSDData"))
    {
      dataManager.clearAllDSDData();
      Serial.println("Ok-clearAllDSDData");
      Serial.flush(); 
      
    }else if (command.startsWith("printAllDSDData"))
    {
      dataManager.printAllDSDData();
      Serial.println("Ok-printAllDSDData");
      Serial.flush(); 
    }
    else if (command.startsWith("exportDSDCSV"))
    {
      dataManager.exportDSDCSV();
      Serial.println("Ok-exportDSDCSV");
      Serial.flush(); 
    }
    else if (command.startsWith("getDSDStoredCount"))
    {
      int count = dataManager.getDSDStoredCount();
      Serial.println(count);
      Serial.println("Ok-getDSDStoredCount");
      Serial.flush(); 
    }
    else if (command.startsWith("readStoredDSDData"))
    {
      int count = dataManager.getDSDStoredCount();
      DigitalStablesData dataArray[count];
      int actualSize = 0;
      if (dataManager.readAllDSDData(dataArray, count, actualSize)) {
          Serial.printf("Successfully read %d entries\n", actualSize);
          // Example: Print all entries
          for(int i = 0; i < actualSize; i++) {
              Serial.printf("\nEntry %d:\n", i);
              dataManager.printDigitalStablesData(dataArray[i]);
          }
      }
      Serial.println("Ok-readStoredDSDData");
      Serial.flush(); 
    }
    else if (command.startsWith("SetTime"))
    {
      // SetTime#31#6#25#7#16#22#30
      // SetTime#1#6#25#1#19#43#00
      timeManager.setTime(command);
      Serial.println("Ok-SetTime");
      Serial.flush(); // SetTime#24#1#25#6#17#21#20
    }
    else if(command.startsWith("GetDeviceSensorConfig")){
      // double latitude = 0.0;
     // double longitude = 0.0;
      //secretManager.getDeviceSensorConfig(digitalStablesData.devicename, digitalStablesData.deviceshortname, digitalStablesData.sensor1name, digitalStablesData.sensor2name, timezone, latitude, v);
      Serial.print(digitalStablesData.devicename);
      Serial.print("#");
      Serial.print(digitalStablesData.deviceshortname);
      Serial.print("#");
      Serial.print(digitalStablesData.sensor1name);
      Serial.print("#");
      Serial.print(digitalStablesData.sensor2name);
      Serial.print("#");
      Serial.print(timezone);
      Serial.print("#");
      Serial.print(digitalStablesData.latitude);
      Serial.print("#");
      Serial.print(digitalStablesData.latitude);
      Serial.print("#");
      Serial.print(digitalStablesData.altitude);
      Serial.print("#");
      Serial.print(digitalStablesData.minimumEfficiencyForLed);
      Serial.print("#");
      Serial.print(digitalStablesData.minimumEfficiencyForWifi);
      Serial.print("#");
      Serial.println(F("Ok-GetDeviceSensorConfig"));
    }
    else if (command.startsWith("SetDeviceSensorConfig"))
    {
// SetDeviceSensorConfig#CreekTub #CREE #NoSensor#Temperature#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#
//SetDeviceSensorConfig#Sceptic #SCEP #NoSensor#Temperature#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#40#50#
  //SetDeviceSensorConfig#GH Compost#GHCO #Soil Moist#Soil Temp#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#

      // SetDeviceSensorConfig#DaffOffice#OFDA#NoSensor#Temperature#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#40#50#
      String devicename = generalFunctions.getValue(command, '#', 1);
      String deviceshortname = generalFunctions.getValue(command, '#', 2);
      String sensor1name = generalFunctions.getValue(command, '#', 3);
      String sensor2name = generalFunctions.getValue(command, '#', 4);

      String timezone = generalFunctions.getValue(command, '#', 5);
      Serial.print("deviceshortname=");
      Serial.println(deviceshortname);
      double latitude=generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 6));
      double longitude=generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 7));
      double altitude=generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 8));
      digitalStablesData.minimumEfficiencyForLed=generalFunctions.getValue(command, '#', 9).toInt();
      digitalStablesData.minimumEfficiencyForWifi=generalFunctions.getValue(command, '#', 10).toInt();
        
      uint8_t devicenamelength = devicename.length() + 1;
      devicename.toCharArray(digitalStablesData.devicename, devicenamelength);
      deviceshortname.toCharArray(digitalStablesData.deviceshortname, deviceshortname.length() + 1);
      sensor1name.toCharArray(digitalStablesData.sensor1name, sensor1name.length() + 1);
      sensor2name.toCharArray(digitalStablesData.sensor2name, sensor2name.length() + 1);

     Serial.print(F("digitalStablesData.minimumEfficiencyForLed="));
     Serial.println(digitalStablesData.minimumEfficiencyForLed);
      secretManager.saveDeviceSensorConfig(devicename, deviceshortname, sensor1name, sensor2name, timezone, latitude, longitude, altitude, digitalStablesData.minimumEfficiencyForLed, digitalStablesData.minimumEfficiencyForWifi);
      
      Serial.println(F("Ok-SetDeviceSensorConfig"));
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
      Serial.print(F("digitalStablesData.deviceshortname="));
      Serial.println(digitalStablesData.deviceshortname);
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
      // ConfigWifiAP#CreekTub##CreekTub#

      String soft_ap_ssid = generalFunctions.getValue(command, '#', 1);
      String soft_ap_password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);

      bool stat = wifiManager.configWifiAP(soft_ap_ssid, soft_ap_password, hostname);
      Serial.print("ConfigWifiAP result=");
      Serial.println(stat);
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
      long code =  secretManager.generateCode();
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

 long* history = secretManager.getCommandCodeHistory();
  Serial.println("Using returned array pointer:");
  for (int i = 0; i < 5; i++) {
    Serial.print(i);
    Serial.print(": ");
    Serial.println(history[i]);
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
      String secretCode= secretManager.readSecret();
       Serial.println(secretCode);
      Serial.println("Ok-GetSecret");
      Serial.flush();
      delay(delayTime);
    }
    else if (command.startsWith("SetSecret"))
    {
      // SetSecret#J5KFCNCPIRCTGT2UJUZFSMQ#6#30
      String secret = generalFunctions.getValue(command, '#', 1);
      int numberDigits = generalFunctions.getValue(command, '#', 2).toInt();
      int periodSeconds = generalFunctions.getValue(command, '#', 3).toInt();
      Serial.println("about to enter savesecret");
       secretManager.saveSecret(secret, numberDigits, periodSeconds);
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
      DigitalStablesDataSerializer digitalStablesDataSerializer;
       digitalStablesDataSerializer.pushToSerial(Serial,digitalStablesData );
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
    }else if (command.startsWith("GetHourlySolarPowerData"))
    {
      HourlySolarPowerData hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);
      Serial.print(" line 1807 efficiency=");
      Serial.println(hourlySolarPowerData.efficiency);
      Serial.print("actualPower=");
      Serial.println(hourlySolarPowerData.actualPower);
      Serial.print("irradiance=");
      Serial.println(hourlySolarPowerData.irradiance);
      Serial.print("temperature=");
      Serial.println(hourlySolarPowerData.temperature);
    }
    else if (command.startsWith("GetDailySolarPowerSchedule"))
    {
      DailySolarPowerSchedule schedules[48];
      solarInfo->calculateDailySolarPowerSchedule(schedules, currentTimerRecord);
       Serial.print(" year=");
        Serial.print(currentTimerRecord.year);
        Serial.print(" month=");
        Serial.print(currentTimerRecord.month);
        Serial.print(" date=");
        Serial.print(currentTimerRecord.date);
      // tmElements_t tm;
     //har timeStr[6]; // HH:mm\0

      for (int i = 0; i < 48; i++)
      {       
        Serial.print(schedules[i].time);
         Serial.print(",");
        Serial.print(TimeUtils::epochToString(schedules[i].time));
         Serial.print(",");
        Serial.print(schedules[i].efficiency);
         Serial.print(",");
        Serial.println(schedules[i].power);
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
