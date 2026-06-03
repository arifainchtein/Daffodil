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
#include <DigitalStablesDataSerializer.h>
#include "ADS1X15.h"
#include "SHTSensor.h"

#include <ErrorManager.h>
#include <ErrorDefinitions.h>
#include <DataManager.h>
#include "CommaRecord.h"
#include <LittleFS.h>
//#include <driver/adc.h>
#include <BH1750.h>
#include <Adafruit_INA219.h>
#include <esp_sleep.h>
//#include <driver/adc.h>

#define RTC_CLK_OUT 4
#define MISO 12
#define MOSI 13
#define SCK 14
#define LoRa_SS 15
#define LORA_RESET 16
#define LORA_DI0 17
#define LED_PIN 19
#define LED_CONTROL 23
#define TPL5010_DONE 25
#define SLEEP_SWITCH_26 26
#define TEMPERATURE 27
#define TRIGGER_PIN 18
#define ECHO_PIN 33
#define OP_MODE 34
#define SENSOR_INPUT_2 33
#define SENSOR_INPUT_1 18
//#define ADS115_ALERT 35
#define TPL5010_WAKE 35
#define RTC_BATT_VOLT 36



Adafruit_INA219 ina219(0x41);
float SHUNT_OHMS = 0.050;    // Shunt resistor value in Ohms — JLCPCB C2596036 50mΩ
#define MAX_CURRENT 1.0      // Maximum expected current in Amps
#define CURRENT_LSB 0.0001   // Current LSB in A/bit — Adafruit default (100µA/bit for 32V/2A mode)
boolean memoryFull = false;
static volatile bool runWatchdog = true;

// All survive hardware resets (TPL5010 watchdog, brownout, etc.)
RTC_DATA_ATTR static uint32_t rtc_intended_wakeup_time = 0; // intended wakeup Unix-seconds
RTC_DATA_ATTR static bool     rtc_comma_mode = false;       // battery critically low; skip full boot
RTC_DATA_ATTR static bool     rtc_has_comma_data = false;   // COMMA session just ended, send recovery LoRa
RTC_DATA_ATTR static uint32_t rtc_comma_first_time = 0;     // Unix-seconds when this COMMA session started
RTC_DATA_ATTR static float    rtc_comma_min_voltage = 99.0f;// lowest voltage seen in this COMMA session
RTC_DATA_ATTR static uint32_t rtc_comma_cycle_count = 0;    // number of 10-min cycles in this session
RTC_DATA_ATTR static char     rtc_device_shortname[8] = {0};// device short name, populated on full boot

String currentSSID;
String ipAddress = "";
boolean initiatedWifi = false;
// #define address 0x40
SHTSensor sht;
bool debug = true;
DataManager dataManager(Serial, LittleFS);

HourlySolarPowerData hourlySolarPowerData;
boolean usingSolarPower = true;

#define NUM_LEDS 15
CRGB leds[NUM_LEDS];

// Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 90  // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define OPERATING_STATUS_SLEEP 1
#define OPERATING_STATUS_NO_LED 2
#define OPERATING_STATUS_FULL_MODE 3
#define OPERATING_STATUS_CLOUDY 4
#define OPERATING_STATUS_COMMA 5   // battery critically low — permanent deep sleep


ErrorManager errorManager;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // NewPing setup of pins and maximum distance.



// LoRa parameters, registers and constants



#define REG_OP_MODE 0x01
#define REG_IRQ_FLAGS 0x12
#define REG_RSSI_VALUE 0x1B
#define MODE_CAD 0x87
#define IRQ_CAD_DONE_MASK 0x04
#define IRQ_CAD_DETECTED_MASK 0x02

#define CAD_TIMEOUT 5000    // CAD timeout in milliseconds
#define MAX_RETRIES 5       // Maximum transmission retries
#define MIN_BACKOFF 500     // Minimum backoff time in milliseconds
#define MAX_BACKOFF 1500    // Maximum backoff time in milliseconds
#define RSSI_THRESHOLD -80  // RSSI threshold in dBm





//define LORA_SAMPLES 3 // Number of samples to take
//define CHECK_LORA_DELAY 2 // Delay between samples in ms

//CHT8305 CHT(0x44);
bool loraTxOk = false;


//TaskHandle_t watchdogTask;
//i2c addresses:
// 40=DFRobot i2c temperature sensor
// 48= ADS1115
// 51= PCF8563T
// 23= bh1750

BH1750 lightMeter;

ADS1115 ADS(0x48);
volatile bool RDY = false;

OneWire oneWire(TEMPERATURE);
DallasTemperature tempSensor(&oneWire);

Timer viewTimer(3);
Timer remoteMonitorTimer(5);
#define MAXIMUM_STORED_RECORDS 2000

// bool internetAvailable;
bool wifiActiveSwitch;
#define uS_TO_S_FACTOR 60000000 /* Conversion factor for micro seconds to minutes */

Timer dsUploadTimer(60);
static volatile int flowMeterPulseCount;
static volatile int flowMeterPulseCount2;

volatile bool loraReceived = false;
volatile int loraPacketSize = 0;


uint8_t displayStatus = 0;
uint8_t loraLastResult = -99;
LoRaError cadResult;
float avgRssi = 0;

#define SHOW_TEMPERATURE 0
#define SHOW_SCEPTIC 1
#define SHOW_INTERNET_STATUS 2
#define SEND_LORA_STATUS 3
#define SHOW_ERROR_STATUS 5
#define SHOW_BATTERY_STATUS 4
//
// sleeping parameters
//
// LiFePO4 battery thresholds (Build 7: replaced supercapacitors with 3.2V LiFePO4 cell)
#define BATTERY_CAPACITY_MAH 600.0   // usable capacity of LiFePO4 123A cell
float sleepingVoltage = 3.12;         // Force deep sleep — cliff edge for LiFePO4 123A
float commaVoltage    = 2.80;         // COMMA threshold — below this, skip all work and wait for solar recovery

uint8_t numberSecondsWithMinimumWifiVoltageForStartWifi = 30;
uint8_t currentSecondsWithWifiVoltage = 0;
float minimumInitWifiVoltage = 3.35;  // Battery must sustain this for 30 s before WiFi starts
//uint8_t sleepingTime = 1;
float minimumLEDVoltage = 3.18;       // Turn off LEDs below this — warning before sleep at 3.15V
uint8_t dimLedBrightness = 20;        // Minimum brightness when LoRa TX budget is too low for full power
uint8_t nightLedBrightness = 30;      // Minimum LED brightness (night / zero efficiency)
float luxNightThreshold = 30.0;       // Lux below this is considered actual darkness → cap at nightLedBrightness
float luxCloudyThreshold = 10000.0;  // Lux below this (but above night) means cloudy — used when weather data is stale
float minimumWifiVoltage = 3.28;      // Turn off WiFi first to preserve power for LoRa
uint8_t secondsSinceLastDataSampling = 0;
uint16_t secondsSinceLastWeatherData = 9999; // 9999 = never received
uint8_t cloudyDutyCyclePercent = 50;         // % of display cycles with LEDs on in CLOUDY mode
uint8_t cloudyThreshold = 70;               // forecast cloudiness % to enter CLOUDY mode
bool cloudyLedCycleOn = true;               // toggles each full display cycle in CLOUDY mode
uint8_t delayTime = 10;
#define UNIQUE_ID_SIZE 8
bool loraActive = false;
DigitalStablesConfigData digitalStablesConfigData;
DigitalStablesData digitalStablesData;
DaffodilCommandData daffodilCommandData;

//
// csw variables
//

float rawCSWValue;
float factor = 1;
int16_t cswOutput;

PCF8563TimeManager timeManager(Serial);
GeneralFunctions generalFunctions;
Esp32SecretManager secretManager(timeManager);
SolarInfo *solarInfo;
PowerManager *powerManager;
WeatherForecastManager *weatherForecastManager;
double lightMeterCorrectingFactor = 3.45;
DaffodilWifiManager wifiManager(Serial, LittleFS, timeManager, secretManager, digitalStablesData, digitalStablesConfigData);

//int badPacketCount = 0;
byte msgCount = 0;         // count of outgoing messages
byte localAddress = 0xFF;  // address of this device
byte destination = 0xAA;   // destination to send to

long lastPulseTime = 0;
uint8_t uniqueId[UNIQUE_ID_SIZE];
long lastMillis;
uint8_t SECONDOFFSET = 10;
uint8_t timeZoneHours = 10;
static byte monthDays[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
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



bool foundlcd = false;
bool foundtemp = false;
bool foundADS = false;
bool foundBH1750 = false;
bool foundINA219 = false;
bool PCF8563T = false;
bool foundDS18B20 = false;

//
// watchdog
//
TaskHandle_t refreshTaskHandle = NULL;
volatile bool wakeSignalReceived = false;
unsigned long lastWakeTime = 0;
const unsigned long WAKE_INTERVAL_MS = 1000;  // Expected wake interval


String serialNumber;
struct TempHum {
  float temp = -99;
  float hum = -99;

} tempHum;

const float R1 = 1000000.0;  // Resistance of R1 in ohms (1 MΩ)
const float R2 = 2000000.0;  // Resistance of R2 in ohms (2 MΩ)
const float Vref = 3.3;      // Reference voltage of the ESP32

int view_milliseconds = 5000;
long lastswitchmillis = 0;
boolean showTemperature = false;
uint8_t color = 0;
String timezone;

const char *display1URL = "http://Ra.local/TeleonomeServlet?formName=GetDeneWordValueByIdentity&identity=Ra:Purpose:Sensor%20Data:Now:Battery%20Voltage";
void IRAM_ATTR pulseCounter() {
  flowMeterPulseCount++;
}

void IRAM_ATTR pulseCounter2() {
  flowMeterPulseCount2++;
}

void IRAM_ATTR clockTick() {
  portENTER_CRITICAL_ISR(&mux);
  clockTicked = true;
  portEXIT_CRITICAL_ISR(&mux);
}


//
// Lora Functions
//

template<typename T>
uint8_t calculateChecksum(const T &data) {
  uint8_t checksum = 0;
  const uint8_t *dataPtr = (const uint8_t *)&data;

  size_t checksumOffset = 0;
  if constexpr (std::is_same<T, DigitalStablesData>::value) {
    checksumOffset = offsetof(DigitalStablesData, checksum);
  } else if constexpr (std::is_same<T, RequestCommand>::value) {
    checksumOffset = offsetof(RequestCommand, checksum);
  }

  for (size_t i = 0; i < checksumOffset; i++) {
    checksum ^= dataPtr[i];
  }
  for (size_t i = checksumOffset + sizeof(uint8_t); i < sizeof(T); i++) {
    checksum ^= dataPtr[i];
  }
  return checksum;
}



template<typename T>
int sendMessage(const T &inputData, bool skipCAD = false) {
  T dataToSend = inputData;

  readSensorData();

  long code = secretManager.generateCode();

  if constexpr (std::is_same<T, DigitalStablesData>::value || std::is_same<T, RequestCommand>::value) {
    dataToSend.totpcode = code;
    dataToSend.checksum = 0;
    dataToSend.checksum = calculateChecksum(dataToSend);
  }

  if (debug) {
    Serial.print("Sending LoRa SN=");
    Serial.print(serialNumber);
    Serial.print(" TOTP=");
    Serial.print(code);
    Serial.print(" Checksum=");
    Serial.println(dataToSend.checksum, HEX);
  }

  LoRa_txMode();

  uint8_t result = 99;
  int retries = 0;
  bool keepGoing = true;
  long startsendingtime = millis();
  LoRa.idle();
  LoRa.flush();
  delay(50);
  while (keepGoing) {
    cadResult = skipCAD ? LORA_OK : performCAD();

    if (cadResult == LORA_OK) {
      LoRa.beginPacket();
      LoRa.write((uint8_t *)&dataToSend, sizeof(T));
      long start = millis();
      if (!LoRa.endPacket(false)) {
        result = LORA_TX_FAILED;
      } else {
        result = LORA_OK;
      }
      delay(50);
      if (debug) {
        Serial.print("Handover took=");
        Serial.print(millis() - start);
        Serial.print("TX took ");
        Serial.print(millis() - startsendingtime);
        Serial.println("ms");
      }
      keepGoing = false;
    } else if (cadResult == LORA_CHANNEL_BUSY) {
      int backoff = random(MIN_BACKOFF * (1 << retries), MAX_BACKOFF * (1 << retries));
      if (debug) Serial.println("Busy, waiting " + String(backoff) + "ms");
      delay(backoff);
      retries++;
      keepGoing = (retries < MAX_RETRIES);
      if (!keepGoing) result = LORA_MAX_RETRIES_REACHED;
    } else {
      result = cadResult;
      keepGoing = false;
    }
  }

  delay(50);
  msgCount++;
  LoRa_rxMode();
  return result;
}

void onReceive(int packetSize) {
  //  Serial.print(" Receive lora: ");
  //     Serial.println(packetSize);
  loraReceived = true;
  loraPacketSize = packetSize;
}
void processLora(int packetSize) {
  if (debug) Serial.print(" Receive lora: ");
  if (debug) Serial.println(packetSize);

  if (debug) Serial.print(" size of ds: ");
  if (debug) Serial.print(sizeof(DigitalStablesData));

  if (debug) Serial.print(" WeatherForecastUpdate: ");
  if (debug) Serial.print(sizeof(WeatherForecastUpdate));

  if (debug) Serial.print(" s RequestCommand: ");
  if (debug) Serial.println(sizeof(RequestCommand));

  if (packetSize == 0) return;  // if there's no packet, return


  if (packetSize == sizeof(DigitalStablesData)) {
    DigitalStablesData receivedDigitalStablesData;
    LoRa.readBytes((uint8_t *)&receivedDigitalStablesData, sizeof(DigitalStablesData));
    long commandcode = receivedDigitalStablesData.totpcode;
    bool validCode = secretManager.checkCode(commandcode);

    if (debug) Serial.print(" Received DigitalStablesData code  : ");
    if (debug) Serial.print(commandcode);
    String receivedSerialNumber;
    for (uint8_t i = 0; i < 8; i++) {
      receivedSerialNumber += String(receivedDigitalStablesData.serialnumberarray[i], HEX);
    }

    if (debug) Serial.print(" sn  : ");
    if (debug) Serial.print(receivedSerialNumber);
    if (receivedSerialNumber == serialNumber) {
      if (debug) Serial.println("Ignored self reception");
    } else {
      if (validCode) {

        // secretManager.saveSleepPingMinutes(rosieConfigData.sleepPingMinutes);
        // secretManager.saveConfigData(rosieConfigData.fieldId,  stationName );

        int rssi = LoRa.packetRssi();
        float Snr = LoRa.packetSnr();
        if (debug) Serial.print(" valid code ");
        if (debug) Serial.print("  from: ");
        if (debug) Serial.println(receivedDigitalStablesData.devicename);
      } else {
        long currentcode = secretManager.generateCode();
        if (debug) Serial.print(" Receive digitalStablesData but invalid code: ");
        if (debug) Serial.print(commandcode);
        if (debug) Serial.print(" currentcode: ");
        if (debug) Serial.println(currentcode);
      }
    }

  } else if (packetSize == sizeof(RequestCommand)) {
    RequestCommand rc;
    LoRa.readBytes((uint8_t *)&rc, sizeof(RequestCommand));
    long totpcode = rc.totpcode;
    bool validCode = secretManager.checkCode(totpcode);
    if (validCode) {
      String commandcode = String(rc.commandString);
      if (commandcode == "SendAsyncData") {
        if (debug) Serial.println("received SEND_ASYNC_DATA");

        // void sendDataViaLoRa() {
        // Read all stored data
        const int MAX_RECORDS = 10;  // Adjust based on your memory constraints
        DigitalStablesData dataArray[MAX_RECORDS];
        int actualSize = 0;
        if (!dataManager.readAllDSDData(dataArray, MAX_RECORDS, actualSize)) {
          if (debug) Serial.println("No data to send or error reading data");
          RequestCommand rc;
          rc.totpcode = secretManager.generateCode();
          rc.setCommand("NoData");
          sendMessage(rc,false);
        }
        if (debug) Serial.printf("Sending %d records via LoRa...\n", actualSize);

        // Send each record as binary data
        for (int i = 0; i < actualSize; i++) {
          dataArray[i].totpcode = secretManager.generateCode();
          sendMessage(dataArray[i],false);
          // LoRa.endPacket();
          if (debug) Serial.printf("Sent record %d/%d (%d bytes)\n",
                                   i + 1, actualSize, sizeof(DigitalStablesData) + 2);  // +2 for marker and index
          // Brief delay to avoid overwhelming the receiver
          delay(200);
        }
        if (debug) Serial.println("All data sent");
        //}


      } else if (commandcode == "ReceivedOK") {
        if (debug) Serial.println("received RECEIVED_OK");
      } else if (commandcode == "ClearedOk") {
        if (debug) Serial.println("received CLEARED_OK");
      } else if (commandcode == "NoData") {
        if (debug) Serial.println("received NO_DATA");
      } else if (commandcode == "SendCurrentData") {
        if (debug) Serial.println("received SendCurrentData, sending ..");
        sendMessage(digitalStablesData,false);
      }
    } else {
      if (debug) Serial.print(" Receive RequestCommand but invalid code: ");
      if (debug) Serial.println(totpcode);
    }
  } else if (packetSize == sizeof(WeatherForecastUpdate)) {
    WeatherForecastUpdate weatherForecastUpdate;
    LoRa.readBytes((uint8_t *)&weatherForecastUpdate, sizeof(WeatherForecastUpdate));
    long commandcode = weatherForecastUpdate.totpcode;
    bool validCode = secretManager.checkCode(commandcode);
    if (validCode) {
      //WeatherForecast forecasts=weatherForecastUpdate.forecasts;
      weatherForecastManager->saveForecasts(weatherForecastUpdate.forecasts);
      solarInfo->setWeatherForecast(weatherForecastUpdate.forecasts, 4);
      secondsSinceLastWeatherData = 0;
      if (debug) Serial.println(" Receive and processed weatherForecast ");
    } else {
      if (debug) Serial.print(" Receive WeatherForecastUpdate but invalid code: ");
      if (debug) Serial.println(commandcode);
    }
  } else {  //if(packetSize==14){
    if (debug) Serial.println("\n--- RECEIVED unknown size  PACKET ---");
    // Create a buffer to store the received bytes
    uint8_t buffer[packetSize];
    // Read all bytes into the buffer
    for (int i = 0; i < packetSize; i++) {
      buffer[i] = LoRa.read();
    }
    // Display as hex values (with position)
    if (debug) Serial.print("HEX: ");
    for (int i = 0; i < packetSize; i++) {
      // Print position
      if (debug) Serial.print("[");
      if (debug) Serial.print(i);
      if (debug) Serial.print("]");
      // Print hex value with leading zero if needed
      if (buffer[i] < 16)
        if (debug) Serial.print("0");
      if (debug) Serial.print(buffer[i], HEX);
      if (debug) Serial.print(" ");
    }
    if (debug) Serial.println();
    // Display as ASCII (printable characters only)
    if (debug) Serial.print("ASCII: ");
    for (int i = 0; i < packetSize; i++) {
      // Check if it's a printable ASCII character (32-126)
      if (buffer[i] >= 32 && buffer[i] <= 126) {
        if (debug) Serial.print((char)buffer[i]);
      } else {
        if (debug) Serial.print(".");  // Non-printable character
      }
    }
    if (debug) Serial.println();
    // Display as decimal values
    if (debug) Serial.print("DEC: ");
    for (int i = 0; i < packetSize; i++) {
      if (debug) Serial.print("[");
      if (debug) Serial.print(i);
      if (debug) Serial.print("]");
      if (debug) Serial.print(buffer[i]);
      if (debug) Serial.print(" ");
    }
    if (debug) Serial.println();
    // Try to interpret as common data types
    if (packetSize >= 4) {
      // As 32-bit integer (little endian)
      int32_t int32Value = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
      if (debug) Serial.print("As Int32 (LE): ");
      if (debug) Serial.println(int32Value);
      // As 32-bit float (little endian)
      float floatValue;
      memcpy(&floatValue, buffer, 4);
      if (debug) Serial.print("As Float (LE): ");
      if (debug) Serial.println(floatValue);
    }
    // Calculate a simple checksum to see if it's consistent
    uint8_t checksum = 0;
    for (int i = 0; i < packetSize - 1; i++) {
      checksum ^= buffer[i];  // XOR checksum
    }
    if (debug) Serial.print("Last byte: ");
    if (debug) Serial.print(buffer[packetSize - 1]);
    if (debug) Serial.print(", XOR Checksum: ");
    if (debug) Serial.println(checksum);
    if (debug) Serial.println("--- END OF PACKET ANALYSIS ---\n");
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

LoRaError performCAD() {

  if (!loraActive) {
    return LORA_INIT_FAILED;
  }

  // 1. Prepare for a clean reading
  LoRa.idle();
  LoRa.receive();

  // 2. Faster Sampling
  // We reduce the delay and sample count to minimize "blind time"
  const int SAMPLES = 4;
  float rssiSum = 0;

  for (int i = 0; i < SAMPLES; i++) {
    rssiSum += LoRa.rssi();
    delayMicroseconds(500);  // Very fast check
  }

  avgRssi = rssiSum / SAMPLES;

  // 3. Forgiving Threshold
  // -50 allows operation in high-noise indoor environments.
  // Restore to -85 for outdoor/greenhouse deployment where a real LoRa
  // neighbour on-channel should block transmission.
  if (avgRssi > -50) {
    if (debug) {
      Serial.print("Channel Busy! RSSI: ");
      Serial.println(avgRssi);
    }
    LoRa.idle();
    return LORA_CHANNEL_BUSY;
  }

  // Clear for transmission
  if (debug) {
    Serial.print("Channel Clear. RSSI: ");
    Serial.println(avgRssi);
  }

  errorManager.clearLoRaError(LORA_CHANNEL_BUSY);
  return LORA_OK;
}



void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      if (debug) Serial.println("Wakeup caused by button press");
      // Do something specific when button wakes the device
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      if (debug) Serial.println("Wakeup caused by timer");
      // Do something specific when timer wakes the device
      break;
    default:
      if (debug) Serial.println("First boot or reset");
      break;
  }
}

void listFiles(const char *dirname) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = LittleFS.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void resetI2CDevices() {
  Wire.beginTransmission(0x00); // General Call Address
  Wire.write(0x06);             // Reset command code
  Wire.endTransmission();
  delay(10);                    // Give devices time to reset
}


//
// End of Lora Functions
//
// Read INA219 bus voltage directly over I2C without library initialisation.
// Safe to call as soon as Wire.begin() has run. Returns -1 on I2C error.
// COMMA log — individual per-cycle readings (timestamp + voltage), capped at COMMA_LOG_MAX_RECORDS.
// Session context (first time, min voltage, cycle count) lives in RTC_DATA_ATTR vars above.

void appendCommaRecord(float voltage, uint32_t nowSec) {
  // Update session-level RTC vars
  if (rtc_comma_first_time == 0) rtc_comma_first_time = nowSec;
  if (voltage > 0 && voltage < rtc_comma_min_voltage) rtc_comma_min_voltage = voltage;
  rtc_comma_cycle_count++;

  CommaRecord rec;
  rec.time    = nowSec;
  rec.voltage = voltage;
  strncpy(rec.devicename, rtc_device_shortname, 7);
  rec.devicename[7] = '\0';

  File rlog = LittleFS.open(COMMA_LOG_FILE, "r");
  int existing = (rlog && rlog.size() >= sizeof(CommaRecord))
                 ? (int)(rlog.size() / sizeof(CommaRecord)) : 0;
  if (rlog) rlog.close();

  if (existing >= COMMA_LOG_MAX_RECORDS) {
    // Log full — drop oldest, write back remaining + new
    CommaRecord buf[COMMA_LOG_MAX_RECORDS];
    File rd = LittleFS.open(COMMA_LOG_FILE, "r");
    if (rd) { rd.readBytes((char*)buf, existing * sizeof(CommaRecord)); rd.close(); }
    File wr = LittleFS.open(COMMA_LOG_FILE, "w");
    if (wr) {
      wr.write((uint8_t*)(buf + 1), (existing - 1) * sizeof(CommaRecord));
      wr.write((uint8_t*)&rec, sizeof(rec));
      wr.close();
    }
  } else {
    File log = LittleFS.open(COMMA_LOG_FILE, "a");
    if (log) { log.write((uint8_t*)&rec, sizeof(rec)); log.close(); }
  }
}

void clearAllCommaRecords() {
  LittleFS.remove(COMMA_LOG_FILE);
  rtc_comma_first_time  = 0;
  rtc_comma_min_voltage = 99.0f;
  rtc_comma_cycle_count = 0;
}

// ─────────────────────────────────────────────────────────────────────────────────────────────

// Matches the Adafruit INA219 getBusVoltage_V() calculation exactly:
//   register 0x02, uint16_t, bits 15:3, 4 mV per LSB → divide by 1000 for volts.
float quickReadBusVoltage() {
  Wire.beginTransmission(0x41);
  Wire.write(0x02);  // INA219 bus voltage register
  if (Wire.endTransmission() != 0) return -1;
  Wire.requestFrom(0x41, 2);
  if (Wire.available() < 2) return -1;
  uint16_t raw = ((uint16_t)Wire.read() << 8) | (uint8_t)Wire.read();
  return (int16_t)((raw >> 3) * 4) * 0.001f;  // identical to getBusVoltage_V()
}

void setup() {
  gpio_hold_dis((gpio_num_t)LED_CONTROL);
  gpio_hold_dis((gpio_num_t)SLEEP_SWITCH_26);  // must release or digitalWrite below has no effect

  pinMode(LED_CONTROL, OUTPUT);
  digitalWrite(LED_CONTROL, LOW);

  pinMode(SLEEP_SWITCH_26, OUTPUT);
  digitalWrite(SLEEP_SWITCH_26, HIGH);
  delay(100);
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);  // set global default before any analogRead so channels initialize with 11dB (max ~3.9V) not the default 0dB (max 1.1V)


  if(debug)Serial.print("DigitalStablesData size=");
  if(debug)Serial.println(sizeof(DigitalStablesData));

  if(debug)Serial.print("ChinampaData size=");
  if(debug)Serial.println(sizeof(ChinampaData));


  if(debug)Serial.print("seedlingMonitorData size=");
  if(debug)Serial.println(sizeof(SeedlingMonitorData));
  
  Wire.end(); 
  delay(10); // Give the bus a moment to settle
  Wire.begin();
  Wire.setClock(400000);

  resetI2CDevices();


  // Try to mount LittleFS if (!LittleFS.begin()) { Serial.println("LittleFS mount failed! Formatting..."); if (LittleFS.format()) { Serial.println("LittleFS formatted successfully."); if (LittleFS.begin()) { Serial.println("LittleFS mounted successfully after formatting."); } else { Serial.println("Failed to mount LittleFS after formatting."); } } else { Serial.println("Failed to format LittleFS."); } } else { Serial.println("LittleFS mounted successfully."); } }

  if (!LittleFS.begin(false)) {
    Serial.println("LittleFS Mount Failed, formatting...");
    LittleFS.format();
    if (!LittleFS.begin(false)) {
      Serial.println("LittleFS Mount Failed even after formatting");
      return;
    } else {
      Serial.println("LittleFS Mount Succces after formating");
    }
  } else {
    Serial.println("LittleFS Mount Succces");
  }


  listFiles("/");
  listFiles("/data/");  // If you have a data folder


  // List all files
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  Serial.println("Files in LittleFS:");
  while (file) {
    Serial.print("  FILE: ");
    Serial.print(file.name());
    Serial.print("  SIZE: ");
    Serial.println(file.size());
    file = root.openNextFile();
  }

  // Check if index.html exists specifically
  if (LittleFS.exists("/index.html")) {
    Serial.println("/index.html exists!");
    File f = LittleFS.open("/index.html", "r");
    Serial.print("File size: ");
    Serial.println(f.size());
    f.close();
  } else {
    Serial.println("/index.html NOT FOUND!");
  }

  print_wakeup_reason();
  dataManager.start();





  lightMeter.setMTreg(32);  //
  lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);
  lightMeter.begin();

  //
  // data from cofiguration
  //
  double latitude = -37.13305556;
  double longitude = 144.47472222;
  double altitude = 410.0;
  lightMeterCorrectingFactor = 3.45;
  double maximumScepticHeight = 0;
  double troughlevelminimumcm = 0;
  double troughlevelmaximumcm = 0;


  secretManager.getDeviceSensorConfig(digitalStablesData.devicename, digitalStablesData.deviceshortname, digitalStablesData.sensor1name, digitalStablesData.sensor2name, timezone, latitude, longitude, altitude, digitalStablesData.minimumEfficiencyForLed, digitalStablesData.minimumEfficiencyForWifi);
  strncpy(rtc_device_shortname, digitalStablesData.deviceshortname, 7);
  rtc_device_shortname[7] = '\0';
  secretManager.getTroughParameters(maximumScepticHeight, troughlevelminimumcm, troughlevelmaximumcm);

  Serial.println("line 819, maximumScepticHeight=" + String(maximumScepticHeight));
  Serial.println("line 819, troughlevelminimumcm=" + String(troughlevelminimumcm));
  Serial.println("line 819, troughlevelmaximumcm=" + String(troughlevelmaximumcm));
  digitalStablesData.maximumScepticHeight = maximumScepticHeight;
  digitalStablesData.troughlevelminimumcm = troughlevelminimumcm;
  digitalStablesData.troughlevelmaximumcm = troughlevelmaximumcm;
  // timezone = "AEST-10AEDT,M10.1.0,M4.1.0/3";
  char timezoneinfo[] = "AEST-10AEDT,M10.1.0,M4.1.0/3";



  float capacitorValue = 3.0;
  float currentPerLed = .020;
  const char *apiKey = "103df7bb3e4010e033d494f031b483e0";
  TimeUtils::parseTimezone(timezone);
  setenv("TZ", timezone.c_str(), 1);
  tzset();
  digitalStablesData.latitude = latitude;
  digitalStablesData.longitude = longitude;

  if (debug) Serial.print("digitalStablesData.minimumEfficiencyForLed=");
  if (debug) Serial.println(digitalStablesData.minimumEfficiencyForLed);

  if (debug) Serial.print("sizeof DigitalStablesData=");
  if (debug) Serial.println(sizeof(DigitalStablesData));

  if (debug) Serial.print("sizeof RequestCommanmd=");
  if (debug) Serial.println(sizeof(RequestCommand));

  if (debug) Serial.print("sizeof WeatherForecastUpdate=");
  if (debug) Serial.println(sizeof(WeatherForecastUpdate));

  pinMode(RTC_CLK_OUT, INPUT_PULLUP);  // set up interrupt pin
  digitalWrite(RTC_CLK_OUT, HIGH);     // turn on pullup resistors
  // attach interrupt to set_tick_tock callback on rising edge of INT0
  attachInterrupt(digitalPinToInterrupt(RTC_CLK_OUT), clockTick, RISING);

  timeManager.start();
  timeManager.PCF8563osc1Hz();
  currentTimerRecord = timeManager.now();

  // ── Early-exit checks (Wire + RTC are ready; nothing else initialised yet) ──────────────────

  uint32_t _nowSec = timeManager.getCurrentTimeInSeconds(currentTimerRecord);

  // 1. TPL5010 watchdog reset during deep sleep: return to sleep for the remaining intended time.
  //    Petting the watchdog here resets its 15-min window so it won't fire again mid-sleep.
  //    This lets PowerManager sleep times longer than the watchdog period work transparently.
  if (rtc_intended_wakeup_time > 0 && _nowSec < rtc_intended_wakeup_time) {
    uint32_t _remaining = rtc_intended_wakeup_time - _nowSec;
    if (_remaining > 0 && _remaining <= 7200) {
      if (debug) { Serial.printf("Early wakeup — %us remaining, returning to sleep.\n", _remaining); Serial.flush(); }
      pinMode(TPL5010_DONE, OUTPUT);
      digitalWrite(TPL5010_DONE, HIGH);
      delayMicroseconds(100);
      digitalWrite(TPL5010_DONE, LOW);
      gpio_hold_en((gpio_num_t)LED_CONTROL);
      gpio_hold_en((gpio_num_t)SLEEP_SWITCH_26);
      gpio_deep_sleep_hold_en();
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      esp_sleep_enable_timer_wakeup((uint64_t)_remaining * 1000000ULL);
      esp_deep_sleep_start();
    }
  }

  // 2. COMMA mode already active: quick voltage check — if still critically low, sleep again.
  if (rtc_comma_mode) {
    float _qv = quickReadBusVoltage();
    if (_qv < 0 || _qv < commaVoltage) {
      if (debug) { Serial.printf("COMMA: %.2fV — returning to sleep.\n", _qv); Serial.flush(); }
      // Pet the TPL5010 watchdog so it gets a fresh 15-min window from now.
      pinMode(TPL5010_DONE, OUTPUT);
      digitalWrite(TPL5010_DONE, HIGH);
      delayMicroseconds(100);
      digitalWrite(TPL5010_DONE, LOW);
      // Append this cycle's reading to the COMMA log.
      appendCommaRecord(_qv, _nowSec);
      gpio_hold_en((gpio_num_t)LED_CONTROL);
      gpio_hold_en((gpio_num_t)SLEEP_SWITCH_26);
      gpio_deep_sleep_hold_en();
      rtc_intended_wakeup_time = _nowSec + 600;
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      esp_sleep_enable_timer_wakeup(600ULL * 1000000ULL);
      esp_deep_sleep_start();
    }
    // Voltage recovered — exit COMMA and proceed with normal boot.
    if (debug) { Serial.printf("COMMA exiting: voltage recovered to %.2fV\n", _qv); Serial.flush(); }
    rtc_comma_mode = false;
    rtc_intended_wakeup_time = 0;
    rtc_has_comma_data = true;  // trigger LoRa summary send after LoRa is initialised
  }

  // ────────────────────────────────────────────────────────────────────────────────────────────

  digitalStablesData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
  digitalStablesData.asyncdata = 1;

 


  //  if(dataManager.getDSDStoredCount()<MAXIMUM_STORED_RECORDS){
  // dataManager.storeDSDData(digitalStablesData);
  //}
  Serial.println("line 898");

  //  const char* ntpServer = "pool.ntp.org";
  //  const long gmtOffset_sec = 36000;  // Melbourne is UTC+10
  //  const int daylightOffset_sec = 3600; // 1 hour during daylight savings

  solarInfo = new SolarInfo(Serial, latitude, longitude, altitude);
  weatherForecastManager = new WeatherForecastManager(Serial, latitude, longitude, apiKey);
  weatherForecastManager->initialize(currentTimerRecord);
  weatherForecastManager->loadForecasts(Serial);
  if (debug) Serial.print("hasValidForecasts=");
  if (debug) Serial.println(weatherForecastManager->hasValidForecasts());



  powerManager = new PowerManager(Serial, ADS, *solarInfo, latitude, longitude, capacitorValue, currentPerLed);
  DailySolarData dailySolarData = solarInfo->getDailySolarData(currentTimerRecord);

  if (debug) Serial.print("sunrise=");
  if (debug) Serial.println(dailySolarData.sunrise);
  if (debug) Serial.print("sunset=");
  if (debug) Serial.println(dailySolarData.sunset);

  if (debug) Serial.print("sunrisetime=");
  if (debug) Serial.println(dailySolarData.sunrisetime);
  if (debug) Serial.print("sunsettime=");
  if (debug) Serial.println(dailySolarData.sunsettime);


  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  digitalStablesData.ledBrightness = dimLedBrightness;
  FastLED.setBrightness(digitalStablesData.ledBrightness);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();


  hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);
  if (debug) Serial.print(" line 368 efficiency=");
  if (debug) Serial.println(hourlySolarPowerData.efficiency);
  //  if(debug)Serial.print("actualPower=");
  //  Serial.println(hourlySolarPowerData.actualPower);
  //  Serial.print("irradiance=");
  //  Serial.println(hourlySolarPowerData.irradiance);
  //  Serial.print("temperature=");
  //  Serial.println(hourlySolarPowerData.temperature);



  if (debug) Serial.println("Scanning for I2C devices ...");


  byte error, address;
  int nDevices = 0;

  for (address = 0x01; address < 0x7f; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      if (debug) Serial.print("I2C device found at address ");
      if (debug) Serial.println(address);
      if (address == 3) {
        foundlcd = true;
      } else if (address == 68) {  //64){ // 0x40
        foundtemp = true;
        if (debug) Serial.println("foundtemp");
      } else if (address == 72) {  // 0x48
        foundADS = true;
        if (debug) Serial.println("foundADS");
      } else if (address == 65) {  // 0x41
        foundINA219 = true;
        if (debug) Serial.println("foundINA219");
      } else if (address == 81) {  // 0x51;
        PCF8563T = true;
        if (debug) Serial.println("found PCF8563T");
      } else if (address == 35) {  // 0x23
        foundBH1750 = true;
        if (debug) Serial.println("foundBH1750");
      }

      nDevices++;
    } else if (error != 2) {
      if (debug) Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }

  if (!foundlcd) {
    errorManager.setI2CError(I2C_DEVICE0_ERROR);
  }
  if (!foundtemp) errorManager.setI2CError(I2C_DEVICE1_ERROR);
  if (!foundADS) errorManager.setI2CError(I2C_DEVICE2_ERROR);
  if (!PCF8563T) errorManager.setI2CError(I2C_DEVICE3_ERROR);
  if (!foundBH1750) errorManager.setI2CError(I2C_DEVICE4_ERROR);
  if (!foundINA219) errorManager.setI2CError(I2C_DEVICE5_ERROR);


  if (!foundtemp) {
    for (int i = NUM_LEDS - 5; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 0, 0);
    }
  }



  //CHT.begin();
  if (sht.init()) {
    Serial.print("sht init(): success\n");
  } else {
    Serial.print("sht init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);  // only supported by SHT3x

  if (!ADS.begin()) {
    if (debug) Serial.println("invalid address ADS1115 or 0x48 not found");
    foundADS = false;
  } else {
    if (debug) Serial.println("found ADS1115");
    foundADS = true;
  }


  if (!ina219.begin()) {
    if (debug) Serial.println("Failed to find INA219 chip");
    foundINA219 = false;
  } else {
    foundINA219 = true;
    if (debug) Serial.println("initialized INA219");
    // To use a slightly lower 32V, 1A range (higher precision on amps):
    //ina219.setCalibration_32V_1A();
    // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
    // ina219.setCalibration_16V_400mA();
    // Reset the device
    Wire.beginTransmission(0x41);
    Wire.write(0x00);  // Config register
    Wire.write(0x80);  // Reset bit
    Wire.write(0x00);
    Wire.endTransmission();
    delay(50);  // Wait for reset

    // Custom calibration for 50mΩ shunt (C2596036) and 1A max current
    //float Current_LSB = MAX_CURRENT/2^15;
    //float Current_LSB = MAX_CURRENT/32768.0;

    uint16_t calibrationValue = (uint16_t)(0.04096 / (CURRENT_LSB * SHUNT_OHMS));

    if (debug) Serial.print("Calculated calibration value: ");
    if (debug) Serial.println(calibrationValue);


    // Set configuration register - try with different gain settings
    uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V | INA219_CONFIG_GAIN_8_320MV |  // Higher gain for better resolution
                      INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US | INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    // Write configuration
    Wire.beginTransmission(0x41);
    Wire.write(0x00);  // Config register
    Wire.write((config >> 8) & 0xFF);
    Wire.write(config & 0xFF);
    Wire.endTransmission();

    // Write calibration
    Wire.beginTransmission(0x41);
    Wire.write(0x05);  // Calibration register
    Wire.write((calibrationValue >> 8) & 0xFF);
    Wire.write(calibrationValue & 0xFF);
    Wire.endTransmission();
  }



  if (sht.readSample()) {
    if (debug) Serial.print("SHT:");
    if (debug) Serial.print("  RH: ");
    if (debug) Serial.print(sht.getHumidity(), 2);
    if (debug) Serial.print("   ");
    if (debug) Serial.print("  T:  ");
    if (debug) Serial.print(sht.getTemperature(), 2);
    if (debug) Serial.print("\n");
  } else {
    if (debug) Serial.print("Error in sht.readSample()\n");
  }


  //  if(debug)Serial.println(CHT.getManufacturer(), HEX);
  //  if(debug)Serial.println(CHT.getVersionID(), HEX);
  //  if(debug)Serial.println(CHT.getVoltage());

  ADS.setGain(0);
  //
  // Battery Voltage (Build 7: LiFePO4 cell replaces supercapacitors)
  int16_t val_3 = ADS.readADC(3);
  float f = ADS.toVoltage(1);  //  voltage factor
  digitalStablesData.v50Voltage = ADS.toVoltage(val_3);

  if (debug) Serial.print("i setup battery=");
  if (debug) Serial.println(digitalStablesData.v50Voltage);

  // Config Switch
  // if(debug)Serial.print("voltage factor=");
  // if(debug)Serial.println(f);
  rawCSWValue = ADS.readADC(2);
  if (debug) Serial.print("rawCSWValue=");
  if (debug) Serial.println(rawCSWValue);
  // V50 is always 5V in Build 7 (from solar through diode OR from battery+boost converter)
  // Battery voltage must NOT be used here — it is a separate 3.2V rail
  factor = 1.0;
  cswOutput = rawCSWValue;


  if (debug) Serial.print("corrected cswOutput=");
  if (debug) Serial.println(cswOutput);

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

  if (cswOutput >= 8300) {
    // Position 0: 00000 (All OFF)
    digitalStablesData.currentFunctionValue = FUN_1_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
    usingSolarPower = false;
  } else if (cswOutput >= 8000 && cswOutput <= 8200) {
    // Position 1: 10000 (R3 ON)
    digitalStablesData.currentFunctionValue = FUN_2_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readFlow2Name().toCharArray(digitalStablesData.sensor2name, 12);
    usingSolarPower = false;
  } else if (cswOutput >= 7800 && cswOutput < 8000) {
    // Position 2: 01000 (R4 ON)
    digitalStablesData.currentFunctionValue = FUN_1_FLOW_1_TANK;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
    usingSolarPower = false;
  } else if (cswOutput >= 7690 && cswOutput < 7800) {
    // Position 3: 11000 (R3+R4 ON)
    digitalStablesData.currentFunctionValue = FUN_1_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
    usingSolarPower = false;
  } else if (cswOutput >= 7500 && cswOutput < 7690) {
    // Position 4: 00100 (R10 ON)
    digitalStablesData.currentFunctionValue = FUN_2_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
    usingSolarPower = false;
  } else if (cswOutput >= 7300 && cswOutput < 7500) {
    // Position 5: 10100 (R3+R10 ON)
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;  //DAFFODIL_SCEPTIC_TANK;
    usingSolarPower = false;
  } else if (cswOutput >= 7100 && cswOutput < 7300) {
    // Position 6: 01100 (R4+R10 ON)
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = false;
  } else if (cswOutput >= 6900 && cswOutput < 7100) {
    // Position 7: 11100 (R3+R4+R10 ON)
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = false;
  } else if (cswOutput >= 6600 && cswOutput < 6900) {
    // Position 8: 00010 (R11 ON)
    usingSolarPower = false;
  } else if (cswOutput >= 6300 && cswOutput < 6600) {
    // Position 9: 10010 (R3+R11 ON)
    usingSolarPower = false;
  } else if (cswOutput >= 6100 && cswOutput < 6300) {
    // Position 10: 01010 (R4+R11 ON)
    usingSolarPower = false;
  } else if (cswOutput >= 5900 && cswOutput < 6100) {
    // Position 11: 11010 (R3+R4+R11 ON)
    usingSolarPower = false;
  } else if (cswOutput >= 5700 && cswOutput < 5900) {
    // 00110
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = false;
  } else if (cswOutput >= 5450 && cswOutput < 5700) {
    // Position 13: 10110 (R3+R10+R11 ON)
    usingSolarPower = false;
  } else if (cswOutput >= 5200 && cswOutput < 5450) {
    // Position 14: 01110 (R4+R10+R11 ON)
    usingSolarPower = false;
  } else if (cswOutput >= 4900 && cswOutput < 5200) {
    // Position 15: 11110 (R3+R4+R10+R11 ON)
    usingSolarPower = false;
  } else if (cswOutput >= 4800 && cswOutput < 4900) {
    // Position 16: 00001 (R13 ON)
    digitalStablesData.currentFunctionValue = FUN_1_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
    usingSolarPower = true;
  } else if (cswOutput >= 4500 && cswOutput < 4800) {
    // Position 17: 10001 (R3+R13 ON)
    digitalStablesData.currentFunctionValue = FUN_2_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readFlow2Name().toCharArray(digitalStablesData.sensor2name, 12);
    usingSolarPower = true;
  } else if (cswOutput >= 4200 && cswOutput < 4500) {
    // Position 18: 01001 (R4+R13 ON)
    usingSolarPower = true;
    digitalStablesData.currentFunctionValue = FUN_1_FLOW_1_TANK;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);

    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
  } else if (cswOutput >= 3900 && cswOutput < 4200) {
    // Position 19: 11001 (R3+R4+R13 ON)
    digitalStablesData.currentFunctionValue = FUN_1_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
    usingSolarPower = true;
  } else if (cswOutput >= 3700 && cswOutput < 3900) {
    // Position 20: 00101 (R10+R13 ON)
    digitalStablesData.currentFunctionValue = FUN_2_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, 12);
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, 12);
    usingSolarPower = true;
  } else if (cswOutput >= 3400 && cswOutput < 3700) {
    // Position 21: 10101 (R3+R10+R13 ON)
    digitalStablesData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;

    usingSolarPower = true;
  } else if (cswOutput >= 3100 && cswOutput < 3400) {
    // Position 22: 01101 (R4+R10+R13 ON)
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;

    usingSolarPower = true;
  } else if (cswOutput >= 2800 && cswOutput < 3100) {
    // Position 23: 11101 (R3+R4+R10+R13 ON)
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;

    usingSolarPower = true;
  } else if (cswOutput >= 2400 && cswOutput < 2800) {
    // Position 24: 00011 (R11+R13 ON)
    usingSolarPower = true;
  } else if (cswOutput >= 2000 && cswOutput < 2400) {
    // Position 25: 10011 (R3+R11+R13 ON)
    usingSolarPower = true;
  } else if (cswOutput >= 1700 && cswOutput < 2000) {
    // Position 26: 01011 (R4+R11+R13 ON)
    usingSolarPower = true;
  } else if (cswOutput >= 1380 && cswOutput < 1700) {
    // Position 27: 11011 (R3+R4+R11+R13 ON)
    usingSolarPower = true;
  } else if (cswOutput >= 1100 && cswOutput < 1380) {
    // Position 6: 00111 (R4+R10 ON)
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = true;
  } else if (cswOutput >= 700 && cswOutput < 1100) {

  } else if (cswOutput >= 350 && cswOutput < 700) {
    // Position 30:  01111 (R4+R10+R11+R13 ON)
    usingSolarPower = true;
  } else if (cswOutput >= 0 && cswOutput < 350) {
    // Position 31: 11111 (All ON) - assuming this is very low
    usingSolarPower = true;
  } else if (cswOutput < 0) {
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = true;
  }

  // digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;//DAFFODIL_SCEPTIC_TANK;
  //     usingSolarPower=false;



  if (usingSolarPower) {
    if (hourlySolarPowerData.efficiency * 100 > digitalStablesData.minimumEfficiencyForLed) {
      digitalWrite(LED_CONTROL, HIGH);
      digitalStablesData.operatingStatus = OPERATING_STATUS_FULL_MODE;
    } else {
      digitalWrite(LED_CONTROL, LOW);
      digitalStablesData.operatingStatus = OPERATING_STATUS_NO_LED;
    }
  } else {
    digitalWrite(LED_CONTROL, HIGH);
    digitalStablesData.operatingStatus = OPERATING_STATUS_FULL_MODE;
  }

  tempSensor.begin();
  tempSensor.setWaitForConversion(false);  // Don't block during conversion
  tempSensor.setResolution(9);

  // uint8_t address[8];
  tempSensor.getAddress(digitalStablesData.serialnumberarray, 0);
  foundDS18B20 = tempSensor.getDeviceCount() > 0;
  //  for (uint8_t i = 0; i < 8; i++)
  //  {
  //    serialNumber += String(digitalStablesData.serialnumberarray[i], HEX);
  //    digitalStablesData.checksum += static_cast<uint8_t>(digitalStablesData.serialnumberarray[i]);
  //  }
  //  digitalStablesData.checksum &= 0xFF;
  if (debug) Serial.print("serial number:");
  if (debug) Serial.println(serialNumber);

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
    // Serial.println("Starting LoRa failed!");
    // drawLora(0);
    while (1)
      ;
    //  leds[1] = CRGB(255, 0, 0);
  } else {
    //  Serial.println("Starting LoRa worked!");
    // drawLora(1);
    loraActive = true;


    // Configure LoRa parameters
    LoRa.setSPIFrequency(1000000);
    LoRa.setTxPower(5);
    LoRa.setSpreadingFactor(9);
    LoRa.enableCrc();
    LoRa.setSignalBandwidth(125E3);
    // LoRa.setCodingRate4(8);
    loraTxOk = true;
  }

  // delay(2000);

  String devicename = secretManager.readDeviceName();
 char devicenamearray[devicename.length() + 1];
  devicename.toCharArray(devicenamearray, devicename.length() + 1);

  strcpy(digitalStablesData.devicename, devicenamearray);

  String deviceshortname = secretManager.readDeviceShortName();
  // Serial.print("deviceshortname=");
  // Serial.println(deviceshortname);
  char deviceshortnamearray[deviceshortname.length() + 1];
  deviceshortname.toCharArray(deviceshortnamearray, deviceshortname.length() + 1);


  strcpy(digitalStablesData.deviceshortname, deviceshortnamearray);


  String grp = secretManager.getGroupIdentifier();
  char gprid[5];
  grp.toCharArray(gprid, 5);
  strcpy(digitalStablesData.groupidentifier, gprid);

  if (debug) Serial.print("Starting wifi digitalStablesConfigData.groupidentifier=");
  if (debug) Serial.println(digitalStablesData.groupidentifier);

  String identifier = "daffodilTF";
  char ty[identifier.length() + 1];
  identifier.toCharArray(ty, identifier.length() + 1);
  strcpy(digitalStablesData.deviceTypeId, ty);

  digitalStablesConfigData.fieldId = secretManager.getFieldId();
  if (debug) Serial.print("Starting wifi digitalStablesConfigData.fieldId=");
  if (debug) Serial.println(digitalStablesConfigData.fieldId);

  pinMode(RTC_BATT_VOLT, INPUT);
  analogSetPinAttenuation(RTC_BATT_VOLT, ADC_11db);  // must be set before first analogRead; default ADC_0db saturates at 1.1V giving 4.95V false reading

  opmode = digitalRead(OP_MODE);
  // Encode all device status into opMode byte (bit 1 updated each tick in loop).
  digitalStablesData.opMode = (opmode       ? 0x01 : 0x00)
                             | (foundINA219  ? 0x04 : 0x00)
                             | (foundBH1750  ? 0x08 : 0x00)
                             | (foundADS     ? 0x10 : 0x00)
                             | (PCF8563T     ? 0x20 : 0x00)
                             | (foundDS18B20 ? 0x40 : 0x00)
                             | (foundtemp    ? 0x80 : 0x00);

  dsUploadTimer.start();
  digitalStablesData.dataSamplingSec = 10;
  if (debug) Serial.print("digitalStablesData.dataSamplingSec=");
  if (debug) Serial.println(digitalStablesData.dataSamplingSec);
  if (debug) Serial.print("digitalStablesData size=");
  if (debug) Serial.println(sizeof(digitalStablesData));


  //  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  //  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  lastswitchmillis = millis();

  viewTimer.start();
  remoteMonitorTimer.start();

  //  LoRa.setSyncWord(0xF3);
  //pinMode(WATCHDOG_WDI, OUTPUT);
  // Configure TPL5010 pins
  pinMode(TPL5010_DONE, OUTPUT);
  digitalWrite(TPL5010_DONE, LOW);
  pinMode(TPL5010_WAKE, INPUT);

  attachInterrupt(digitalPinToInterrupt(TPL5010_WAKE), handleWakeInterrupt, RISING);
  //  xTaskCreatePinnedToCore(
  //      refreshTPL5010Task, /* Task function. */
  //      "TPL5010Refresh",       /* name of task. */
  //      2048,                /* Stack size of task */
  //      NULL,                 /* parameter of the task */
  //      1,                    /* priority of the task */
  //      &refreshTaskHandle,        /* Task handle to keep track of created task */
  //      0);
  // digitalWrite(WATCHDOG_WDI, LOW);




  // COMMA recovery: send the session summary via LoRa using the RTC-preserved stats.
  if (rtc_has_comma_data && loraActive) {
    DigitalStablesData commaData = digitalStablesData;
    commaData.batteryVoltage  = rtc_comma_min_voltage;
    commaData.secondsTime     = rtc_comma_first_time;
    commaData.sleepTime       = _nowSec - rtc_comma_first_time;
    commaData.operatingStatus = OPERATING_STATUS_COMMA;
    commaData.asyncdata       = 11;  // COMMA recovery summary
    sendMessage(commaData, true);    // skipCAD — critical overnight report
    if (debug) Serial.printf("COMMA recovery sent: minV=%.2f cycles=%u duration=%us\n",
                             rtc_comma_min_voltage, rtc_comma_cycle_count,
                             _nowSec - rtc_comma_first_time);
    rtc_comma_first_time  = 0;
    rtc_comma_min_voltage = 99.0f;
    rtc_comma_cycle_count = 0;
    rtc_has_comma_data    = false;
  }

  // First-time COMMA detection: LoRa is initialised here so the final message can be sent.
  // goToSleep() will transmit operating_status=COMMA and then the device stays in
  // the permanent quick-check loop handled by the early-exit block above.
  if (usingSolarPower && !rtc_comma_mode) {
    float _qv = quickReadBusVoltage();
    if (_qv > 0 && _qv < commaVoltage) {
      if (debug) { Serial.printf("Entering COMMA mode at %.2fV\n", _qv); Serial.flush(); }
      rtc_comma_mode = true;
      digitalStablesData.batteryVoltage = _qv;
      digitalStablesData.operatingStatus = OPERATING_STATUS_COMMA;
      appendCommaRecord(_qv, _nowSec);  // log the entry voltage
      goToSleep();  // sends final LoRa with COMMA status, then deep sleeps
    }
  }

  boolean isSleepMode = false;
  if (usingSolarPower && hourlySolarPowerData.efficiency * 100 < digitalStablesData.minimumEfficiencyForLed) {
    isSleepMode = true;
    digitalStablesData.operatingStatus = OPERATING_STATUS_SLEEP;
    if (debug) Serial.print("setting sleepmode in setup because of efficiency=");
    if (debug) Serial.println(hourlySolarPowerData.efficiency);

    digitalStablesData.asyncdata = 2;
    if (dataManager.getDSDStoredCount() < MAXIMUM_STORED_RECORDS) {
      dataManager.storeDSDData(digitalStablesData);
    }
  }
  // Protect battery from over-discharge (only when on solar/battery, not wall power).
  // batteryVoltage is 0 here (readSensorData hasn't run yet) so read it directly.
  if (!isSleepMode && usingSolarPower) {
    float _setupBatV = quickReadBusVoltage();
    if (_setupBatV > 0) digitalStablesData.batteryVoltage = _setupBatV;
    if (_setupBatV >= commaVoltage && _setupBatV < sleepingVoltage) {
      if (debug) Serial.print("setting sleepmode in setup because of low battery voltage=");
      if (debug) Serial.println(_setupBatV);
      isSleepMode = true;
      digitalStablesData.operatingStatus = OPERATING_STATUS_SLEEP;
      digitalStablesData.asyncdata = 3;
      if (dataManager.getDSDStoredCount() < MAXIMUM_STORED_RECORDS) {
        dataManager.storeDSDData(digitalStablesData);
      }
    }
  }


  if (isSleepMode) {
    Serial.println("Calling deepsleep line 1378");
    goToSleep();
  } else {
    digitalStablesData.operatingStatus = OPERATING_STATUS_NO_LED;
    if (loraActive) {
      // LoRa_rxMode();
      // LoRa.setSyncWord(0xF3);
      LoRa.onReceive(onReceive);
      // put the radio into receive mode
      LoRa.receive();
    }
  }
  if (debug) Serial.println(F("Finished Setup"));
}

void sleepDS18B20() {  // Put OneWire bus in high impedance state pinMode(ONE_WIRE_BUS, INPUT);

  // Force DS18B20 to stop any conversion
  oneWire.reset();
  oneWire.skip();
  oneWire.write(0x44);  // Start conversion command
  oneWire.reset();      // Reset to stop conversion
}

// Modified by Claude
void goToSleep() {
  // Heartbeat: show "B" battery status while LEDs are powered, then cut power.
  digitalWrite(LED_CONTROL, HIGH);
  delay(20);  // let MOSFET turn on and WS2812 power supply stabilise
  readSensorData();                     // get fresh voltage/current before display
  drawBatteryStatus(digitalStablesData.batteryVoltage, digitalStablesData.batteryCurrent);
  delay(1000);                          // hold the display for 1 s so it is visible
  FastLED.clear(true);
  FastLED.show();
  delay(10);
  digitalWrite(LED_CONTROL, LOW);

  // 1. Calculate sleep timing.
  //    PowerManager returns 60 s whenever theoretical solar efficiency > 0.3 (sun is up).
  //    On heavily overcast days this is wrong — actual lux can be 4000 while efficiency
  //    reads 0.4 from the geometric model. When lux is below the cloudy threshold, apply
  //    the same night formula so sleep time reflects how dark it actually is.
  long seconds_sleep = powerManager->calculateOptimalSleepTime(currentTimerRecord);
  if (usingSolarPower && digitalStablesData.lux >= 0 && digitalStablesData.lux < luxCloudyThreshold) {
    DailySolarData _dsd = solarInfo->getDailySolarData(currentTimerRecord);
    int _currentMin  = currentTimerRecord.hour * 60 + currentTimerRecord.minute;
    int _toSunrise   = (int)_dsd.sunrise - _currentMin;
    if (_toSunrise < 0) _toSunrise += 24 * 60;
    if (_toSunrise < 1) _toSunrise = 1;
    if (_toSunrise > 90) {
      int _dayLen   = max((int)_dsd.sunset - (int)_dsd.sunrise, 1);
      int _nightMin = max(24 * 60 - _dayLen, 1);
      long _cloudySleep = (long)(450.0f * _nightMin / _toSunrise);
      if (_cloudySleep < 90) _cloudySleep = 90;
      if (_cloudySleep > seconds_sleep) seconds_sleep = _cloudySleep;
    }
  }
  if (seconds_sleep < 30) seconds_sleep = 30;
  uint64_t sleep_time_us = (uint64_t)(seconds_sleep * 1000000ULL);
  if (debug) Serial.printf("Preparing sleep for %lld seconds\n", seconds_sleep);

  // 2. Store final record (sensors already read above for the heartbeat display).
  // Preserve OPERATING_STATUS_COMMA if this is the first entry into COMMA mode.
  if (digitalStablesData.operatingStatus != OPERATING_STATUS_COMMA &&
      digitalStablesData.operatingStatus != OPERATING_STATUS_CLOUDY) {
    digitalStablesData.operatingStatus = OPERATING_STATUS_SLEEP;
  }
  digitalStablesData.sleepTime = seconds_sleep;
  if (dataManager.getDSDStoredCount() < MAXIMUM_STORED_RECORDS) {
    dataManager.storeDSDData(digitalStablesData);
  }

  // 3. Send final LoRa message so the hub knows we are sleeping and for how long.
  // skipCAD=true: this is a critical notification — don't let a busy channel silence it.
  if (loraActive) {
    sendMessage(digitalStablesData, true);
  }
  LoRa.sleep();

  // 4. Shut down WiFi and wait for it to fully stop before cutting power
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  while (WiFi.getMode() != WIFI_OFF || WiFi.status() == WL_CONNECTED) {
    delay(100);
  }

  // 5. Shut down remaining peripherals
  btStop();
  ADS.setMode(1);  // ADS1115 power-down mode
  sleepDS18B20();

  // 6. Configure SPI pins to safe states to prevent leakage into sensors
  pinMode(MISO, INPUT_PULLDOWN);
  pinMode(MOSI, INPUT_PULLDOWN);
  pinMode(SCK, INPUT_PULLDOWN);
  pinMode(LoRa_SS, INPUT_PULLUP);  // keep CS high so LoRa stays deselected

  // 7. Drive power control pins LOW and hold them through deep sleep
  digitalWrite(LED_CONTROL, LOW);
  digitalWrite(SLEEP_SWITCH_26, LOW);
  gpio_hold_en((gpio_num_t)LED_CONTROL);
  gpio_hold_en((gpio_num_t)SLEEP_SWITCH_26);
  gpio_deep_sleep_hold_en();

  // 8. Enter deep sleep
  if (debug) {
    Serial.println("Entering deep sleep.");
    Serial.flush();
  }
  // Pet the TPL5010 watchdog immediately before sleeping so it gets a fresh 15-min window.
  digitalWrite(TPL5010_DONE, HIGH);
  delayMicroseconds(100);
  digitalWrite(TPL5010_DONE, LOW);

  // Record the Unix-seconds time we expect to wake, so the early-exit block in setup()
  // can detect and immediately dismiss TPL5010 watchdog resets during sleep.
  {
    RTCInfoRecord _now = timeManager.now();
    rtc_intended_wakeup_time = timeManager.getCurrentTimeInSeconds(_now) + seconds_sleep;
  }
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_timer_wakeup(sleep_time_us);
  esp_deep_sleep_start();
}


void handleWakeInterrupt() {
  wakeSignalReceived = true;
}



void readSensorData() {
  //
  // Capacitor Voltage
  //
  ADS.setGain(0);
  int16_t val_3 = ADS.readADC(3);
  float f = ADS.toVoltage(1);  //  voltage factor
  digitalStablesData.v50Voltage = val_3 * f;

  if (digitalStablesData.currentFunctionValue == VOLTAGE_MONITOR) {
    // read the voltage on sensor 1 ie pin 32
    int16_t val_1 = ADS.readADC(1);
    float f = ADS.toVoltage(1);  //  voltage factor
    digitalStablesData.flowRate = val_1 * f;
  }

  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);
  digitalStablesData.temperature = tempC;
  readI2CTemp();
  digitalStablesData.secondsTime = timeManager.getCurrentTimeInSeconds(currentTimerRecord);
  // sleepTime is NOT set here — only goToSleep() knows the actual intended sleep duration
  // (after applying the sanity floor). Setting it here would corrupt records stored before
  // goToSleep() is called (e.g. the asyncdata=2 setup path).
  if (foundBH1750) {
    //      // http://community.heltec.cn/t/bh1750-light-sensor-practical-notes-problems-and-issues/1521
    //      double cal=1.13;
    //      //digitalStablesData.lux = ((lightMeter.readLightLevel()/cal)/2.5);

    // the value of correcting factor was obtained by comparing the output to the output of the Davies Vantage Pro 2 sensor
    // which it is assume that is correct.3.45

    digitalStablesData.lux = lightMeter.readLightLevel() * lightMeterCorrectingFactor;
  } else {
    digitalStablesData.lux = -99;
  }
  //    if(debug)Serial.print("lux=");
  //    if(debug)Serial.println(digitalStablesData.lux);

  {
    uint8_t br = 255;
    // 1. Actual darkness: BH1750 measures genuine dark (night / deep shade)
    if (foundBH1750 && digitalStablesData.lux >= 0 && digitalStablesData.lux < luxNightThreshold) {
      br = nightLedBrightness;
    }
    // 2. Solar efficiency: scale within the usable [minimumEfficiencyForLed..100%] range
    if (usingSolarPower) {
      HourlySolarPowerData hspd = solarInfo->calculateActualPower(currentTimerRecord);
      float minEff = digitalStablesData.minimumEfficiencyForLed / 100.0f;
      float scaled = constrain((hspd.efficiency - minEff) / max(1.0f - minEff, 0.01f), 0.0f, 1.0f);
      br = min(br, (uint8_t)(nightLedBrightness + scaled * (255 - nightLedBrightness)));
    }
    // 3. Low battery: cap to protect remaining charge
    if (digitalStablesData.batteryVoltage < minimumWifiVoltage) {
      br = min(br, dimLedBrightness);
    }
    digitalStablesData.ledBrightness = br;
  }
  float distance = sonar.ping_cm();
  digitalStablesData.measuredHeight = distance;
  digitalStablesData.scepticAvailablePercentage = distance * 100 / MAX_DISTANCE;
  if (debug) Serial.print("line 1654 measuredHeight=");
  if (debug) Serial.println(digitalStablesData.measuredHeight);

  //
  // RTC_BATT_VOLT Voltage
  //
  analogSetPinAttenuation(RTC_BATT_VOLT, ADC_11db);
  float total = 0;
  uint8_t samples = 20;
  for (int x = 0; x < samples; x++) {           // multiple analogue readings for averaging
    total = total + analogRead(RTC_BATT_VOLT);  // add each value to a total
    delay(2);
  }
  float average = total / samples;
  if (debug) Serial.println("RTC average=" + String(average));
  if (average >= 4090) {
    // ADC saturated — pin 36 voltage >= 3.9V; cannot be a 3V coin cell.
    // Likely: floating pin, wrong signal source, or divider not installed.
    digitalStablesData.rtcBatVolt = -1;
  } else {
    float voltage = (average / 4095.0) * Vref;
    digitalStablesData.rtcBatVolt = (voltage * (R1 + R2)) / R2;
  }
  if (debug) Serial.println("RTC rtcBatVolt=" + String(digitalStablesData.rtcBatVolt));
  //
  // current
  //
  // Read raw shunt voltage register for debugging
  if (foundINA219) {
    Wire.beginTransmission(0x41);
    Wire.write(0x01);  // Shunt voltage register
    Wire.endTransmission();
    Wire.requestFrom(0x41, 2);
    int16_t rawShunt = (Wire.read() << 8) | Wire.read();

    Wire.beginTransmission(0x41);
    Wire.write(0x04);  // Current register
    Wire.endTransmission();
    Wire.requestFrom(0x41, 2);
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

    if (debug) Serial.println("--- MEASUREMENTS ---");
    if (debug) Serial.print("Raw Shunt Register: 0x");
    if (debug) Serial.print(rawShunt, HEX);
    if (debug) Serial.print(" (");
    if (debug) Serial.print(rawShunt);
    if (debug) Serial.println(")");
    if (debug) Serial.print("Raw Current Register: 0x");
    if (debug) Serial.print(rawCurrent, HEX);
    if (debug) Serial.print(" (");
    if (debug) Serial.print(rawCurrent);
    if (debug) Serial.println(")");

    if (debug) Serial.print("Bus Voltage: ");
    if (debug) Serial.print(busvoltage);
    if (debug) Serial.println(" V");
    if (debug) Serial.print("Shunt Voltage: ");
    if (debug) Serial.print(shuntvoltage);
    if (debug) Serial.println(" mV");

    if (debug) Serial.print("Library Current: ");
    if (debug) Serial.print(current_mA);
    if (debug) Serial.println(" mA");
    if (debug) Serial.print("Calculated Current (V/R): ");
    if (debug) Serial.print(calculated_current_mA);
    if (debug) Serial.println(" mA");
    if (debug) Serial.print("Direct Register Current: ");
    if (debug) Serial.print(direct_current_mA);
    if (debug) Serial.println(" mA");

    if (debug) Serial.print("Power: ");
    if (debug) Serial.print(power_mW);
    if (debug) Serial.println(" mW");
    if (debug) Serial.println("");

    digitalStablesData.batteryVoltage = busvoltage;
    digitalStablesData.batteryCurrent = calculated_current_mA;

    // Estimate runtime: only meaningful when discharging (positive current = battery powering load)
    if (calculated_current_mA > 1.0) {
      float dischargeMa = abs(calculated_current_mA);
      // Cycle-average: active phase (time since wakeup) + 60s deep sleep
      float activeSec = millis() / 1000.0;
      float cycleSec = activeSec + 60.0;
      float avgCurrentMa = dischargeMa * (activeSec / cycleSec);  // sleep draw ≈ 0
      uint8_t soc = generalFunctions.getBatteryStateOfCharge(busvoltage);
      float remainingMah = (soc / 100.0) * BATTERY_CAPACITY_MAH;
      digitalStablesData.estimatedRuntime = (avgCurrentMa > 0) ? (remainingMah / avgCurrentMa) : 0.0;
    } else {
      digitalStablesData.estimatedRuntime = 0.0;  // charging or unknown
    }
  } else {
    // INA219 library not available — fall back to direct I2C voltage read so that
    // all voltage-based protections (sleep, COMMA, WiFi shutoff) still work.
    float _fallbackV = quickReadBusVoltage();
    digitalStablesData.batteryVoltage = (_fallbackV > 0) ? _fallbackV : -99;
    digitalStablesData.batteryCurrent = -99;
    digitalStablesData.estimatedRuntime = 0.0;
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

void restartWifi() {
  //FastLED.setBrightness(50);
  for (int i = 0; i < NUM_LEDS; i++) {
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
  if (!initiatedWifi) {

    leds[7] = CRGB(255, 0, 255);
    FastLED.show();
    // Serial.print(F("Before Starting Wifi cap="));
    // Serial.println(digitalStablesData.v50Voltage);
    wifiManager.start();
    initiatedWifi = true;
  }
  if (debug) Serial.println("Starting wifi");

  wifiManager.restartWifi();
  //      digitalWrite(WATCHDOG_WDI, HIGH);
  //    delay(2);
  //    digitalWrite(WATCHDOG_WDI, LOW);
  if (debug) Serial.println("getting  stationmode=");
  bool stationmode = wifiManager.getStationMode();
  digitalStablesData.internetAvailable = wifiManager.getInternetAvailable();
  //     digitalWrite(WATCHDOG_WDI, HIGH);
  //    delay(2);
  //    digitalWrite(WATCHDOG_WDI, LOW);
  if (debug) Serial.print("Starting wifi stationmode=");
  // Serial.println(stationmode);
  // Serial.print("digitalStablesData.internetAvailable=");
  // Serial.println(digitalStablesData.internetAvailable);

  //  serialNumber = wifiManager.getMacAddress();
  wifiManager.setSerialNumber(serialNumber);
  wifiManager.setLora(loraActive);
  String ssid = wifiManager.getSSID();
  String ipAddress = "";
  uint8_t ipi;
  if (stationmode) {
    ipAddress = wifiManager.getIpAddress();
    //   Serial.print("ipaddress=");
    //  Serial.println(ipAddress);

    if (ipAddress == "" || ipAddress == "0.0.0.0") {

      setApMode();
    } else {
      setStationMode(ipAddress);
    }
  } else {
    setApMode();
  }
  //    digitalWrite(WATCHDOG_WDI, HIGH);
  //    delay(2);
  //    digitalWrite(WATCHDOG_WDI, LOW);

  digitalStablesData.loraActive = loraActive;
  uint8_t ipl = ipAddress.length() + 1;
  char ipa[ipl];
  ipAddress.toCharArray(ipa, ipl);
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
  // Serial.println("in ino Done starting wifi");
}

void drawBatteryStatus(float voltage, float current) {
  // Battery voltage color (LiFePO4 123A zones):
  //   >= 3.35V  green  — stable, WiFi possible
  //   >= 3.25V  yellow — WiFi off, LoRa still running
  //    < 3.25V  red    — approaching sleep cliff
  CRGB batColor;
  if (voltage >= minimumWifiVoltage) {
    batColor = CRGB(0, 0, 255);
  } else if (voltage >= minimumLEDVoltage && voltage <=minimumWifiVoltage) {
    batColor = CRGB(0, 255, 0);
  } else if (voltage >= 3.10 && voltage <= minimumLEDVoltage) {
    batColor = CRGB(255, 200, 0);
  } else {
    batColor = CRGB(255, 0, 0);
  }

  // Power source indicator (INA219 current sign):
  //   < -10mA  → solar charging battery   → green
  //   > +10mA  → battery discharging/boost → red
  //   near 0   → transition / uncertain    → blue
  CRGB srcColor;
  if (current < -1.0) {
    srcColor = CRGB(0, 255, 0);
  } else if (current > 1.0) {
    srcColor = CRGB(255, 0, 0);
  } else {
    srcColor = CRGB(0, 0, 255);
  }

  // Layout — 5-col x 3-row grid (LEDs 0-14):
  //  col:  0  1  .  .  4
  //  row0: #  #  .  .  S   → 0,1=batColor  4=srcColor
  //  row1: #  .  .  .  .   → 5
  //  row2: #  #  .  .  .   → 10,11
  // cols 2-3 are dark — 2-column gap between B and indicator
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB(0, 0, 0);
  leds[1]  = batColor;    // B top bar
  leds[4]  = srcColor;                         // power source indicator
  leds[6]  = batColor;   leds[7]  = batColor;  // B spine
  leds[9]  = (digitalStablesData.operatingStatus == OPERATING_STATUS_FULL_MODE) ? CRGB(0, 255, 0) :
             (digitalStablesData.operatingStatus == OPERATING_STATUS_CLOUDY)    ? CRGB(255, 200, 0) : CRGB(0, 0, 0);
  leds[11] = batColor;  leds[12] = batColor;   // B bottom bar
  // LED 14: weather data freshness — green < 31 min, red otherwise (never received = red)
  leds[14] = (secondsSinceLastWeatherData < 1860) ? CRGB(0, 255, 0) : CRGB(255, 0, 0);
  FastLED.setBrightness(digitalStablesData.ledBrightness);
  FastLED.show();
}

void drawError(uint8_t code, uint8_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }

  if (color == 0) {
    leds[0] = CRGB(255, 0, 0);
    leds[1] = CRGB(255, 0, 0);
    leds[2] = CRGB(255, 0, 0);
    leds[5] = CRGB(255, 0, 0);
    leds[6] = CRGB(255, 0, 0);
    leds[10] = CRGB(255, 0, 0);
    leds[11] = CRGB(255, 0, 0);
    leds[12] = CRGB(255, 0, 0);
  } else if (color == 1) {
    leds[0] = CRGB(255, 255, 0);
    leds[1] = CRGB(255, 255, 0);
    leds[2] = CRGB(255, 255, 0);
    leds[5] = CRGB(255, 255, 0);
    leds[6] = CRGB(255, 255, 0);
    leds[10] = CRGB(255, 255, 0);
    leds[11] = CRGB(255, 255, 0);
    leds[12] = CRGB(255, 255, 0);
  } else {
    leds[0] = CRGB(0, 0, 255);
    leds[1] = CRGB(0, 0, 255);
    leds[2] = CRGB(0, 0, 255);
    leds[5] = CRGB(0, 0, 255);
    leds[6] = CRGB(0, 0, 255);
    leds[10] = CRGB(0, 0, 255);
    leds[11] = CRGB(0, 0, 255);
    leds[12] = CRGB(0, 0, 255);
  }


  // for the error is 4, 9, 13
  switch (code) {
    case 0:
      leds[4] = CRGB(0, 0, 255);
      break;
    case 1:
      leds[9] = CRGB(0, 0, 255);
      break;
    case 2:
      leds[14] = CRGB(0, 0, 255);
      break;
  }
  FastLED.show();
}

void drawLora(int status) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  if (status == 1) {
    leds[1] = CRGB(0, 255, 0);
    leds[6] = CRGB(0, 255, 0);
    leds[11] = CRGB(0, 255, 0);
    leds[12] = CRGB(0, 255, 0);
    // leds[13] = CRGB(0, 255, 0);
  } else if (status == 2) {
    leds[1] = CRGB(255, 255, 0);
    leds[6] = CRGB(255, 255, 0);
    leds[11] = CRGB(255, 255, 0);
    leds[12] = CRGB(255, 255, 0);
    // leds[13] = CRGB(0, 255, 0);
  } else if (status == 0) {
    leds[1] = CRGB(255, 0, 0);
    leds[6] = CRGB(255, 0, 0);
    leds[11] = CRGB(255, 0, 0);
    leds[12] = CRGB(255, 0, 0);
    // leds[13] = CRGB( 255,0, 0);
  }
  FastLED.show();
}

void drawTemperature(uint8_t red, uint8_t green, uint8_t blue) {

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  uint8_t ld = 0;
  uint8_t hd = 0;
  float t = abs(digitalStablesData.outdoortemperature);

  if (t > 0 && t < 10) {
    ld = t;
  } else if (t >= 10 && t < 20) {
    ld = t - 10;
    hd = 1;
  } else if (t >= 20 && t < 30) {
    ld = t - 20;
    hd = 2;
  } else if (t >= 30 && t < 40) {
    ld = t - 30;
    hd = 3;
  } else if (t >= 40 && t < 50) {
    ld = t - 40;
    hd = 4;
  }
  if (debug) Serial.print("hd=");
  if (debug) Serial.print(hd);
  switch (hd) {
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
  if (debug) Serial.print("ld=");
  if (debug) Serial.println(ld);

  switch (ld) {
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
void readI2CTemp() {
  float temperature = 0;

  //  READ DATA
  // uint32_t start = micros();
  //  int status = CHT.read();
  //  uint32_t stop = micros();
  //
  ////  Serial.print("CHT8305\t");
  ////  //  DISPLAY DATA, sensor has only one decimal.
  ////-  Serial.print(F("  Humidity"));
  //Serial.println(CHT.getLastError());
  //Serial.println(CHT.getManufacturer(), HEX);
  //  Serial.println(CHT.getVersionID(), HEX);
  //  Serial.println(CHT.getVoltage());

  if (sht.readSample()) {
    digitalStablesData.outdoortemperature = sht.getTemperature();
    digitalStablesData.outdoorhumidity = sht.getHumidity();
    if (debug) Serial.print("SHT:\n");
    if (debug) Serial.print("  RH: ");
    if (debug) Serial.print(sht.getHumidity(), 2);
    if (debug) Serial.print("\n");
    if (debug) Serial.print("  T:  ");
    if (debug) Serial.print(sht.getTemperature(), 2);
    if (debug) Serial.print("\n");
  } else {
    if (debug) Serial.print("Error in readSample()\n");
    digitalStablesData.outdoortemperature = -99;
  }

  // temperature = sht.getTemperature();// CHT.getTemperature();

  // digitalStablesData.outdoortemperature = temperature;
  if (debug) Serial.print(" Temp:");
  if (debug) Serial.print(digitalStablesData.outdoortemperature, 1);
  //digitalStablesData.outdoorhumidity = sht.getHumidity();//CHT.getHumidity();
  if (debug) Serial.print(" Hum:");
  if (debug) Serial.println(digitalStablesData.outdoorhumidity, 1);

  //  switch (status)
  //  {
  //  case CHT8305_OK:
  //    Serial.print("OK");
  //    break;
  //  case CHT8305_ERROR_ADDR:
  //    Serial.print("Address error");
  //    break;
  //  case CHT8305_ERROR_I2C:
  //    Serial.print("Outdoor Temperature I2C error");
  //    digitalStablesData.outdoortemperature = -99;
  //    break;
  //  case CHT8305_ERROR_CONNECT:
  //    Serial.print("Connect error");
  //    break;
  //  case CHT8305_ERROR_LASTREAD:
  //    Serial.print("Last read error");
  //    break;
  //  default:
  //    Serial.print("Unknown error");
  //    break;
  //  }
  if (debug) Serial.print("\n");
}

void loop() {
  uint16_t dscount;
  boolean turnOffWifi = false;
  bool wifistatus = wifiManager.getWifiStatus();
  if (clockTicked) {

    portENTER_CRITICAL(&mux);
    clockTicked = false;
    portEXIT_CRITICAL(&mux);
    currentTimerRecord = timeManager.now();

    if (wakeSignalReceived) {
      wakeSignalReceived = false;
      digitalWrite(TPL5010_DONE, HIGH);
      delayMicroseconds(100);  // Pulse width needs to be at least 20µs
      digitalWrite(TPL5010_DONE, LOW);
    }
    //
    // generate codes so that the history is refresed
    //
    if (currentTimerRecord.second == 0) {


      if (currentTimerRecord.minute == 0) {
        //  Serial.println(F("New Hour"));
        if (currentTimerRecord.hour == 0) {
          //    Serial.println(F("New Day"));
        }
      }
    }

    if (currentTimerRecord.second % 10 == 0) {
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

      if (debug) Serial.print("Wake signal received! Time since last wake: ");
      if (debug) Serial.print(timeSinceLastWake);
      if (debug) Serial.println(" ms");
      // Reset the flag
      wakeSignalReceived = false;
    }

    if (loraReceived) {
      loraReceived = false;
      processLora(loraPacketSize);
    }
    secondsSinceLastDataSampling++;
    if (secondsSinceLastWeatherData < 9999) secondsSinceLastWeatherData++;
    //   Serial.println("ticked");

    viewTimer.tick();
    remoteMonitorTimer.tick();
    hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);

    //
    // read the sensors
    //

    readSensorData();
    if (debug) Serial.println("linr 2219");
    //  int dsdStoredCount=dataManager.getDSDStoredCount();
    // if(dsdStoredCount>(int)(.7*MAXIMUM_STORED_RECORDS)){
    //   memoryFull=true;
    // }else{
    //   memoryFull=false;
    // }

 

    if (remoteMonitorTimer.status()) {
      remoteMonitorTimer.reset();
      if (digitalStablesData.currentFunctionValue == VOLTAGE_MONITOR && currentTimerRecord.hour >= 16 && !memoryFull) {
        digitalStablesData.asyncdata = 6;
        dataManager.storeDSDData(digitalStablesData);
        remoteMonitorTimer.reset();
      }
    }


    // if(debug)Serial.print("External Voltage=");
    //  if(debug)Serial.println(digitalStablesData.flowRate);

    if (digitalStablesData.batteryVoltage > minimumInitWifiVoltage && !wifistatus) {
      currentSecondsWithWifiVoltage++;
    } else {
      currentSecondsWithWifiVoltage = 0;
    }

    dscount = dsUploadTimer.tick();
    HourlySolarPowerData hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);
    if (usingSolarPower) {
      if (hourlySolarPowerData.efficiency * 100 > digitalStablesData.minimumEfficiencyForLed) {
        digitalWrite(LED_CONTROL, HIGH);
        {
          uint8_t br = 255;
          // 1. Actual darkness: BH1750 measures genuine dark (night / deep shade)
          if (foundBH1750 && digitalStablesData.lux >= 0 && digitalStablesData.lux < luxNightThreshold) {
            br = nightLedBrightness;
          }
          // 2. Solar efficiency: scale within [minimumEfficiencyForLed..100%] range
          {
            float minEff = digitalStablesData.minimumEfficiencyForLed / 100.0f;
            float scaled = constrain((hourlySolarPowerData.efficiency - minEff) / max(1.0f - minEff, 0.01f), 0.0f, 1.0f);
            br = min(br, (uint8_t)(nightLedBrightness + scaled * (255 - nightLedBrightness)));
          }
          // 3. Low battery: cap to protect remaining charge
          if (digitalStablesData.batteryVoltage < minimumWifiVoltage) {
            br = min(br, dimLedBrightness);
          }
          digitalStablesData.ledBrightness = br;
          FastLED.setBrightness(br);
        }
        {
          bool luxSaysCloudy = foundBH1750
                             && digitalStablesData.lux >= luxNightThreshold
                             && digitalStablesData.lux < luxCloudyThreshold;
          bool forecastSaysCloudy = false;
          if (secondsSinceLastWeatherData < 1860) {
            WeatherForecast* forecasts = weatherForecastManager->getForecasts();
            forecastSaysCloudy = (forecasts != nullptr) && (forecasts[0].cloudiness >= cloudyThreshold);
          }
          digitalStablesData.operatingStatus = (luxSaysCloudy || forecastSaysCloudy)
                                               ? OPERATING_STATUS_CLOUDY : OPERATING_STATUS_FULL_MODE;
          // Update bit 1 (weather freshness) — all other bits set once in setup.
          if (secondsSinceLastWeatherData < 1860)
            digitalStablesData.opMode |=  0x02;
          else
            digitalStablesData.opMode &= ~0x02;
        }
        turnOffWifi = (hourlySolarPowerData.efficiency * 100 < digitalStablesData.minimumEfficiencyForWifi) && wifistatus;
      } else {
        FastLED.clear(true);
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CRGB(0, 0, 0);
        }
        FastLED.show();
        digitalStablesData.ledBrightness = 0;
        digitalWrite(LED_CONTROL, LOW);
        digitalStablesData.operatingStatus = OPERATING_STATUS_NO_LED;
        if(debug)Serial.println("line 2162 turning off leds because efficiency is " + String(hourlySolarPowerData.efficiency) + " and te minimum is " + digitalStablesData.minimumEfficiencyForLed);
        turnOffWifi = true;
        //         if(dataManager.getDSDStoredCount()<MAXIMUM_STORED_RECORDS){
        //            digitalStablesData.asyncdata=8;
        //         }
        //        dataManager.storeDSDData(digitalStablesData);
      }
    } else {
      digitalWrite(LED_CONTROL, HIGH);
      digitalStablesData.ledBrightness = 255;
      FastLED.setBrightness(255);
      digitalStablesData.operatingStatus = OPERATING_STATUS_FULL_MODE;
      turnOffWifi = false;
    }






    if (debug) Serial.print("line 1990  hourlySolarPowerData.efficiency=");
    if (debug) Serial.print(hourlySolarPowerData.efficiency);
    if (debug) Serial.print(" wifistatus=");
    if (debug) Serial.println(wifistatus);
    if (debug) Serial.print(" turnOffWifi=");
    if (debug) Serial.println(turnOffWifi);

  }  // end of the tick block



  // COMMA check in the main loop (battery may drop during active WiFi/LoRa operation).
  if (usingSolarPower && !rtc_comma_mode &&
      digitalStablesData.batteryVoltage > 0 && digitalStablesData.batteryVoltage < commaVoltage) {
    if (debug) { Serial.printf("COMMA from loop at %.2fV\n", digitalStablesData.batteryVoltage); }
    rtc_comma_mode = true;
    digitalStablesData.operatingStatus = OPERATING_STATUS_COMMA;
    appendCommaRecord(digitalStablesData.batteryVoltage, digitalStablesData.secondsTime);
    goToSleep();
  }

  boolean isSleepMode = false;
  if (usingSolarPower && hourlySolarPowerData.efficiency * 100 < digitalStablesData.minimumEfficiencyForLed) isSleepMode = true;
  // Protect battery from over-discharge (only when on solar/battery, not wall power)
  if (usingSolarPower && digitalStablesData.batteryVoltage >= commaVoltage && digitalStablesData.batteryVoltage < sleepingVoltage) isSleepMode = true;
  // On overcast days the theoretical efficiency may still be above threshold but actual solar
  // is insufficient to sustain continuous operation. Sleep between each LoRa pulse.
  if (usingSolarPower && digitalStablesData.operatingStatus == OPERATING_STATUS_CLOUDY) isSleepMode = true;
  if (isSleepMode) {
    if (digitalStablesData.operatingStatus != OPERATING_STATUS_CLOUDY) {
      digitalStablesData.operatingStatus = OPERATING_STATUS_SLEEP;
    }
    if (debug) Serial.print("going to sleep because batteryVoltage is less than sleepingVoltage, bat=");
    if (debug) Serial.println(digitalStablesData.batteryVoltage);
    digitalStablesData.asyncdata = 7;
    if (dataManager.getDSDStoredCount() < MAXIMUM_STORED_RECORDS) {
      dataManager.storeDSDData(digitalStablesData);
    }
    Serial.println("Calling deepsleep line 2185");
    goToSleep();
  }



  if (usingSolarPower && digitalStablesData.batteryVoltage > commaVoltage && digitalStablesData.batteryVoltage < minimumLEDVoltage && digitalRead(LED_CONTROL)) {

    if (debug) Serial.print("line 967 turning off leds, battery=");
    if (debug) Serial.println(digitalStablesData.batteryVoltage);
    FastLED.clear(true);
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(0, 0, 0);
    }
    FastLED.show();
    digitalStablesData.ledBrightness = 0;
    digitalWrite(LED_CONTROL, LOW);
    digitalStablesData.operatingStatus = OPERATING_STATUS_NO_LED;
  }



  // Unconditional voltage-based WiFi shutoff: if battery is below minimumWifiVoltage,
  // always call stop() — even if wifistatus is false (WiFi may be retrying and drawing current).
  if (usingSolarPower && digitalStablesData.batteryVoltage > commaVoltage && digitalStablesData.batteryVoltage < minimumWifiVoltage) {
    turnOffWifi = true;
  }
  if (turnOffWifi) {
    wifiManager.stop();
    WiFi.setAutoReconnect(false);
    WiFi.disconnect(true);

    if (debug) Serial.print("turning off wifi battery voltage=");
    if (debug) Serial.println(digitalStablesData.v50Voltage);
    if (debug) Serial.print("after wifimanager stop, wifistatus=");
    if (debug) Serial.println(wifiManager.getWifiStatus());

    // Only show the disconnect animation and reset the counter once, when actually transitioning
    // from connected to disconnected. Without this guard, every tick below minimumWifiVoltage
    // would re-run the animation (red WiFi → yellow 0/14 loop visible to the user).
    if (wifistatus) {
      digitalStablesData.internetAvailable = false;
      currentSecondsWithWifiVoltage = 0;
      FastLED.clear(true);
      for (int i = 0; i < NUM_LEDS; i++) {
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
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 0, 0);
      }
      FastLED.show();
      FastLED.setBrightness(digitalStablesData.ledBrightness);
      leds[0] = CRGB(255, 255, 0);
      leds[14] = CRGB(255, 255, 0);
      FastLED.show();
    }
  }
  wifistatus = wifiManager.getWifiStatus();
  boolean turnOnWifi = false;
  if (!usingSolarPower) {
    if (!wifistatus) turnOnWifi = true;
  } else {
    turnOnWifi = (hourlySolarPowerData.efficiency * 100 > digitalStablesData.minimumEfficiencyForWifi)
              && (currentSecondsWithWifiVoltage >= numberSecondsWithMinimumWifiVoltageForStartWifi)
              && (digitalStablesData.batteryVoltage >= minimumInitWifiVoltage)
              && !wifistatus;
  }

  if (turnOnWifi) {
    if (debug) Serial.print("turning on  wifi");
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
  if (staledata && wifistatus && !wifiManager.getAPStatus()) {
    // Fetch weather data and update SolarInfo
    weatherForecastManager->downloadWeatherData(solarInfo);
    secondsSinceLastWeatherData = 0;
  }
  // if(debug)Serial.println("line 1139");
  boolean showError = false;
  if (viewTimer.status()) {
    showTemperature = !showTemperature;
    bool cloudySkip = (digitalStablesData.operatingStatus == OPERATING_STATUS_CLOUDY && !cloudyLedCycleOn);

    if (debug) Serial.print("battery voltage=");
    if (debug) Serial.print(digitalStablesData.v50Voltage);
    if (debug) Serial.print(" displayStatus=");
    if (debug) Serial.println(displayStatus);
    if (debug) Serial.print("line 1287 digitalStablesData.ledBrightness=");
    if (debug) Serial.print(digitalStablesData.ledBrightness);
    if (debug) Serial.print("  digitalRead(LED_CONTROL)=");
    if (debug) Serial.println(digitalRead(LED_CONTROL));

    if (!cloudySkip) {
    loraTxOk = loraActive;
    FastLED.setBrightness(digitalStablesData.ledBrightness);


    if (displayStatus == SHOW_TEMPERATURE) {

      if (debug) Serial.print("showing temperature=");


      if (digitalStablesData.outdoortemperature == -99) {
        for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CRGB(0, 0, 0);
        }
        leds[1] = CRGB(255, 0, 0);
        leds[2] = CRGB(255, 0, 0);
        leds[3] = CRGB(255, 0, 0);
        leds[7] = CRGB(255, 0, 0);
        leds[12] = CRGB(255, 0, 0);
        FastLED.show();
      } else {
        if (digitalStablesData.outdoortemperature > 0) {
          red = 0;
          green = 255;
          blue = 0;
        } else if (digitalStablesData.outdoortemperature < 0) {
          red = 0;
          green = 0;
          blue = 255;
        } else {
          red = 255;
          green = 255;
          blue = 0;
        }
        drawTemperature(red, green, blue);
      }
    } else if (displayStatus == SHOW_SCEPTIC) {
      if (debug) Serial.print("showing scepotic=");
      if (debug) Serial.print(" percentage: ");
      if (debug) Serial.println(digitalStablesData.scepticAvailablePercentage);
      red = 255;
      green = 0;
      blue = 255;

      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 0, 0);
      }
      if (digitalStablesData.currentFunctionValue == DAFFODIL_SCEPTIC_TANK) {
        if (digitalStablesData.scepticAvailablePercentage <= 25) {
          red = 255;
          green = 0;
          blue = 0;
        } else if (digitalStablesData.scepticAvailablePercentage > 25 && digitalStablesData.scepticAvailablePercentage <= 50) {
          red = 255;
          green = 255;
          blue = 0;
        } else if (digitalStablesData.scepticAvailablePercentage > 50 && digitalStablesData.scepticAvailablePercentage <= 75) {
          red = 0;
          green = 255;
          blue = 0;
        } else if (digitalStablesData.scepticAvailablePercentage > 75) {
          red = 0;
          green = 0;
          blue = 255;
        }
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_WATER_TROUGH) {
        if (digitalStablesData.measuredHeight >= (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelminimumcm)) {
          red = 255;
          green = 0;
          blue = 0;
        } else if (digitalStablesData.measuredHeight < (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelminimumcm) && digitalStablesData.measuredHeight >= (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelmaximumcm)) {
          red = 0;
          green = 255;
          blue = 0;
        } else if (digitalStablesData.measuredHeight < (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelmaximumcm)) {
          red = 0;
          green = 0;
          blue = 255;
        }
      } else if (digitalStablesData.currentFunctionValue == VOLTAGE_MONITOR) {
        int dsdStoredCount = dataManager.getDSDStoredCount();
        if ((dsdStoredCount * 100 / MAXIMUM_STORED_RECORDS) <= 25) {
          red = 255;
          green = 0;
          blue = 0;
        } else if ((dsdStoredCount * 100 / MAXIMUM_STORED_RECORDS) > 25 && (dsdStoredCount * 100 / MAXIMUM_STORED_RECORDS) <= 50) {
          red = 255;
          green = 255;
          blue = 0;
        } else if ((dsdStoredCount * 100 / MAXIMUM_STORED_RECORDS) > 50 && (dsdStoredCount * 100 / MAXIMUM_STORED_RECORDS) <= 75) {
          red = 0;
          green = 255;
          blue = 0;
        } else if ((dsdStoredCount * 100 / MAXIMUM_STORED_RECORDS) > 75) {
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
    } else if (displayStatus == SHOW_INTERNET_STATUS) {
      wifistatus = wifiManager.getWifiStatus();
      if (debug) Serial.print("line 1112 inside of showintenrnetstatus internetAvailable=");
      if (debug) Serial.println(digitalStablesData.internetAvailable);
      if (debug) Serial.print("wifiManager.getAPStatus()=");
      if (debug) Serial.println(wifiManager.getAPStatus());
      if (debug) Serial.print("wifiManager.getWifiStatus()=");
      if (debug) Serial.println(wifistatus);
      if (debug) Serial.print("dsupload timer counter= ");
      if (debug) Serial.println(dscount);



      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 0, 0);
      }

      boolean displayWifi = false;
      if (!usingSolarPower) displayWifi = true;
      else {
        displayWifi = wifistatus && (hourlySolarPowerData.efficiency * 100 > digitalStablesData.minimumEfficiencyForWifi);
      }
      if (displayWifi) {
        if (wifiManager.getAPStatus()) {
          leds[1] = CRGB(0, 255, 0);
          leds[2] = CRGB(0, 255, 0);
          leds[3] = CRGB(0, 255, 0);
          leds[5] = CRGB(0, 255, 0);
          leds[9] = CRGB(0, 255, 0);
          leds[11] = CRGB(0, 255, 0);
          leds[12] = CRGB(0, 255, 0);
          leds[13] = CRGB(0, 255, 0);
          FastLED.show();
        } else {
          leds[1] = CRGB(0, 0, 255);
          leds[2] = CRGB(0, 0, 255);
          leds[3] = CRGB(0, 0, 255);
          leds[5] = CRGB(0, 0, 255);
          leds[9] = CRGB(0, 0, 255);
          leds[11] = CRGB(0, 0, 255);
          leds[12] = CRGB(0, 0, 255);
          leds[13] = CRGB(0, 0, 255);
          if (digitalStablesData.internetAvailable) {
            leds[7] = CRGB(0, 0, 255);
          } else {
            leds[7] = CRGB(255, 0, 0);
          }
          FastLED.show();
          //
          //

          if (digitalStablesData.internetAvailable) {
            if (dsUploadTimer.status()) {
              // char secret[27];

              String secret = "J5KFCNCPIRCTGT2UJUZFSMQK";
              //                    leds[2] = CRGB(0, 255, 0);
              //                    FastLED.show();
              TOTP totp = TOTP(secret.c_str());
              char totpCode[7];  // get 6 char code
              long timeVal = timeManager.getTimeForCodeGeneration(currentTimerRecord);
              if (debug) Serial.print("line 1153 timeVal=");
              if (debug) Serial.println(timeVal);
              digitalStablesData.secondsTime = timeVal;
              long code = totp.gen_code(timeVal);
              if (debug) Serial.print("l;ine 1154 totp=");
              if (debug) Serial.println(code);

              wifiManager.setCurrentToTpCode(code);
              int response = wifiManager.uploadDataToDigitalStables();
              if (debug) Serial.print("l;ine 1153 uploading to ds=");
              if (debug) Serial.println(response);
              if (response == 200) {
                leds[7] = CRGB(0, 0, 255);
              } else if (response == 500) {
                leds[7] = CRGB(255, 0, 255);
              } else {
                leds[7] = CRGB(255, 0, 0);
              }
              FastLED.show();
              dsUploadTimer.reset();
            }
          } else {
            //
            // if we are here it means that there is an ipaddress
            // but internet is not available , check again if there is a reconnection
            wifiManager.checkInternetConnectionAvailable();

            digitalStablesData.internetAvailable = wifiManager.getInternetAvailable();
            if (debug) Serial.print("after rechecking digitalstable,internetConnectionAvailable=");
            if (debug) Serial.println(digitalStablesData.internetAvailable);
          }
        }
      } else {
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
    } else if (displayStatus == SEND_LORA_STATUS) {
      if (debug) Serial.print("showing Lora Status loraActive=");
      if (debug) Serial.print(loraActive);
      if (debug) Serial.print(" loraTxOk=");
      if (debug) Serial.println(loraTxOk);
      if (loraActive) {
        delay(random(100, 3000));
        readSensorData();
        if (debug) Serial.println("line 2708");



        digitalStablesData.asyncdata = 9;
        // if(dataManager.getDSDStoredCount()<MAXIMUM_STORED_RECORDS){
        //   dataManager.storeDSDData(digitalStablesData);
        // }

        loraLastResult = sendMessage(digitalStablesData);

        if (loraLastResult == LORA_TX_FAILED) {
          drawLora(0);
        } else if (loraLastResult == LORA_OK) {
          drawLora(1);
        }
      } else {
        drawLora(0);
      }
    } else if (displayStatus == SHOW_ERROR_STATUS) {
      //
      // color 0=red;
      // 1=yellow
      // 2=green
      uint8_t color = 0;
      if (!foundADS) {
        if (debug) Serial.println("showing error with ADS");
        drawError(0, 0);
        showError = true;
      } else if (memoryFull) {
        if (debug) Serial.println("memory full");
        int dsdStoredCount = dataManager.getDSDStoredCount();

        if (dsdStoredCount > (int)(.7 * MAXIMUM_STORED_RECORDS)) {
          memoryFull = true;
        } else if (dsdStoredCount > (int)(.7 * MAXIMUM_STORED_RECORDS)) {
          memoryFull = true;
        }
        drawError(1, 0);
        showError = true;
      } else {
        displayStatus++;  // to make sure that this is not displayed
        showError = false;
      }
    } else if (displayStatus == SHOW_BATTERY_STATUS) {
      drawBatteryStatus(digitalStablesData.batteryVoltage, digitalStablesData.batteryCurrent);
    }
    } else {
      // CLOUDY dark cycle — LEDs off for this full display cycle
      for (int i = 0; i < NUM_LEDS; i++) leds[i] = CRGB(0, 0, 0);
      FastLED.clear(true);
      FastLED.show();
    }
    displayStatus++;

    if (displayStatus == SHOW_ERROR_STATUS && foundADS && !memoryFull) {
      displayStatus = 0;
      if (digitalStablesData.operatingStatus == OPERATING_STATUS_CLOUDY) cloudyLedCycleOn = !cloudyLedCycleOn;
    } else if (displayStatus > 5) {
      displayStatus = 0;
      if (digitalStablesData.operatingStatus == OPERATING_STATUS_CLOUDY) cloudyLedCycleOn = !cloudyLedCycleOn;
    }

    if (!showError) {
      viewTimer.reset();
      clockTicked = false;  // discard ISR ticks accumulated during long LoRa TX/delay
    }
  }


  if (Serial.available() != 0) {
    String command = Serial.readString();
    if (command.startsWith("Ping")) {
      Serial.println(F("Ok-Ping"));
    } else if (command.startsWith("debug")) {
      int debugv = generalFunctions.getValue(command, '#', 1).toInt();
      if (debugv > 0) debug = true;
      else debug = false;
      Serial.println("Ok-debug");
      Serial.flush();
    } else if (command.startsWith("goToSleep")) {
      Serial.println("Calling deepsleep line 2584");
      goToSleep();

    } else if (command.startsWith("SetTroughParameters")) {
      // for s
      //SetTroughParameters#troughheight#troughlevelminimumcm#troughlevelmaximumcm#
      // for sumptrough
      //SetTroughParameters#29#39#45#
      //
      // for                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            tank
      //SetTroughParameters#69#42#50#

      digitalStablesData.maximumScepticHeight = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 1));
      digitalStablesData.troughlevelminimumcm = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 2));
      digitalStablesData.troughlevelmaximumcm = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 3));
      Serial.println("line 2753, maximumScepticHeight=" + String(digitalStablesData.maximumScepticHeight));
      Serial.println("line 2753, troughlevelminimumcm=" + String(digitalStablesData.troughlevelminimumcm));
      Serial.println("line 2753, troughlevelmaximumcm=" + String(digitalStablesData.troughlevelmaximumcm));

      secretManager.saveTroughParameters(digitalStablesData.maximumScepticHeight, digitalStablesData.troughlevelminimumcm, digitalStablesData.troughlevelmaximumcm);


      Serial.println("Ok-SetTroughParameters");
      Serial.flush();
    } else if (command.startsWith("usingSolarPower")) {
      int usingSolarPowerv = generalFunctions.getValue(command, '#', 1).toInt();
      if (usingSolarPowerv > 0) usingSolarPower = true;
      else usingSolarPower = false;
      Serial.println("Ok-usingSolarPower");
      Serial.flush();
    } else if (command.startsWith("getUsingSolarPower")) {
      Serial.print("usingSolarPower:");
      Serial.println(usingSolarPower);
      Serial.println("Ok-usingSolarPower");
      Serial.flush();
    }

    else if (command.startsWith("storeDSDData")) {

      digitalStablesData.asyncdata = 10;
      int count = dataManager.storeDSDData(digitalStablesData);
      Serial.println(count);
      Serial.println("Ok-storeDSDData");
      Serial.flush();
    } else if (command.startsWith("SetFieldId")) {
      // fieldId= GeneralFunctions::getValue(command, '#', 1).toInt();

    } else if (command.startsWith("clearAllDSDData")) {
      dataManager.clearAllDSDData();
      Serial.println("Ok-clearAllDSDData");
      Serial.flush();

    } else if (command.startsWith("printCommaRecord")) {
      File log = LittleFS.open(COMMA_LOG_FILE, "r");
      int n = log ? (int)(log.size() / sizeof(CommaRecord)) : 0;
      if (n == 0) {
        Serial.println("No CommaRecords found");
      } else {
        Serial.println(String(n) + " records (newest last):");
        for (int i = 0; i < n; i++) {
          CommaRecord cr;
          log.readBytes((char*)&cr, sizeof(cr));
          Serial.println("[" + String(i + 1) + "] " + String(cr.devicename)
                         + "  t=" + String(cr.time)
                         + "  (" + TimeUtils::epochToString(cr.time) + ")"
                         + "  v=" + String(cr.voltage, 3) + "V");
        }
        log.close();
      }
      Serial.println("  rtc_comma_mode=" + String(rtc_comma_mode));
      Serial.println("  rtc_has_comma_data=" + String(rtc_has_comma_data));
      if (rtc_comma_mode) {
        Serial.println("  session_firstTime=" + String(rtc_comma_first_time));
        Serial.println("  session_minVoltage=" + String(rtc_comma_min_voltage, 3) + "V");
        Serial.println("  session_cycles=" + String(rtc_comma_cycle_count));
      }
      Serial.println("Ok-printCommaRecord");
      Serial.flush();

    } else if (command.startsWith("clearAllCommaRecords")) {
      clearAllCommaRecords();
      Serial.println("Ok-clearAllCommaRecords");
      Serial.flush();

    } else if (command.startsWith("printAllDSDData")) {
      dataManager.printAllDSDData();
      Serial.println("Ok-printAllDSDData");
      Serial.flush();
    } else if (command.startsWith("printCurrentDSDData")) {
      dataManager.printDigitalStablesData(digitalStablesData);
      Serial.println("--- Runtime ---");
      Serial.println("rawCSWValue=" + String(rawCSWValue));
      Serial.println("cswOutput=" + String(cswOutput));
      Serial.println("usingSolarPower=" + String(usingSolarPower));

      // Operating status (human-readable)
      String osName;
      switch (digitalStablesData.operatingStatus) {
        case OPERATING_STATUS_SLEEP:    osName = "SLEEP";    break;
        case OPERATING_STATUS_NO_LED:   osName = "NO_LED";   break;
        case OPERATING_STATUS_FULL_MODE:osName = "FULL_MODE";break;
        case OPERATING_STATUS_CLOUDY:   osName = "CLOUDY";   break;
        case OPERATING_STATUS_COMMA:    osName = "COMMA";    break;
        default: osName = "UNKNOWN(" + String(digitalStablesData.operatingStatus) + ")";
      }
      Serial.println("operatingStatus=" + osName);

      // opMode bits decoded
      uint8_t om = digitalStablesData.opMode;
      Serial.println("opMode=0x" + String(om, HEX) + " (0b" + String(om, BIN) + ")");
      Serial.println("  bit0 hwPin="        + String(om & 0x01 ? "1"   : "0"));
      Serial.println("  bit1 weatherFresh=" + String(om & 0x02 ? "YES" : "NO"));
      Serial.println("  bit2 INA219="       + String(om & 0x04 ? "OK"  : "MISSING"));
      Serial.println("  bit3 BH1750="       + String(om & 0x08 ? "OK"  : "MISSING"));
      Serial.println("  bit4 ADS1115="      + String(om & 0x10 ? "OK"  : "MISSING"));
      Serial.println("  bit5 RTC="          + String(om & 0x20 ? "OK"  : "MISSING"));
      Serial.println("  bit6 DS18B20="      + String(om & 0x40 ? "OK"  : "MISSING"));
      Serial.println("  bit7 SHT="          + String(om & 0x80 ? "OK"  : "MISSING"));

      // Weather forecast
      Serial.println("secondsSinceLastWeatherData=" + String(secondsSinceLastWeatherData)
                     + (secondsSinceLastWeatherData == 9999 ? " (never received)"
                       : secondsSinceLastWeatherData < 1860  ? " (fresh)"
                                                             : " (stale)"));

      // Solar
      HourlySolarPowerData _hspd = solarInfo->calculateActualPower(currentTimerRecord);
      Serial.println("solarEfficiency=" + String(_hspd.efficiency * 100, 1) + "%"
                     + "  minForLed=" + String(digitalStablesData.minimumEfficiencyForLed) + "%"
                     + "  minForWifi=" + String(digitalStablesData.minimumEfficiencyForWifi) + "%");
      Serial.println("lux=" + String(digitalStablesData.lux, 1)
                     + "  luxNight<" + String(luxNightThreshold)
                     + "  luxCloudy<" + String(luxCloudyThreshold));

      // Battery thresholds
      Serial.println("sleepingVoltage=" + String(sleepingVoltage)
                     + "  commaVoltage=" + String(commaVoltage)
                     + "  minWifiV=" + String(minimumWifiVoltage)
                     + "  minInitWifiV=" + String(minimumInitWifiVoltage));
      Serial.println("currentSecondsWithWifiVoltage=" + String(currentSecondsWithWifiVoltage)
                     + "/" + String(numberSecondsWithMinimumWifiVoltageForStartWifi));

      // COMMA state
      Serial.println("rtc_comma_mode=" + String(rtc_comma_mode)
                     + "  rtc_has_comma_data=" + String(rtc_has_comma_data));
      if (rtc_comma_mode) {
        Serial.println("  comma_firstTime=" + String(rtc_comma_first_time)
                       + "  minV=" + String(rtc_comma_min_voltage, 3) + "V"
                       + "  cycles=" + String(rtc_comma_cycle_count));
      }
      Serial.println("Ok-printCurrentDSDData");
      Serial.flush();
    } else if (command.startsWith("printCSWData")) {
      Serial.println("rawCSWValue=" + String(rawCSWValue));
      Serial.println("cswV50Voltage=5.0 (fixed, Build 7)");
      Serial.println("factor=" + String(factor));
      Serial.println("cswOutput=" + String(cswOutput));
      String functionname = "";
      if (digitalStablesData.currentFunctionValue == FUN_1_FLOW) {
        functionname = "FUN_1_FLOW";
      } else if (digitalStablesData.currentFunctionValue == FUN_2_FLOW) {
        functionname = "FUN_2_FLOW";
      } else if (digitalStablesData.currentFunctionValue == FUN_1_FLOW_1_TANK) {
        functionname = "FUN_1_FLOW_1_TANK";
      } else if (digitalStablesData.currentFunctionValue == FUN_1_TANK) {
        functionname = "FUN_1_TANK";
      } else if (digitalStablesData.currentFunctionValue == FUN_2_TANK) {
        functionname = "FUN_2_TANK";
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_SCEPTIC_TANK) {
        functionname = "DAFFODIL_SCEPTIC_TANK";
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_WATER_TROUGH) {
        functionname = "DAFFODIL_WATER_TROUGH";
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_TEMP_SOILMOISTURE) {
        functionname = "DAFFODIL_TEMP_SOILMOISTURE";
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_LIGHT_DETECTOR) {
        functionname = "DAFFODIL_LIGHT_DETECTOR";
      } else if (digitalStablesData.currentFunctionValue == VOLTAGE_MONITOR) {
        functionname = "VOLTAGE_MONITOR";
      }
      Serial.println("Current Function Value: " + functionname);
      Serial.println("Ok-printCSWData");
      Serial.flush();
    } else if (command.startsWith("exportDSDCSV")) {
      dataManager.exportDSDCSV();
      Serial.println("Ok-exportDSDCSV");
      Serial.flush();
    } else if (command.startsWith("GenerateDSDReport")) {
      Serial.print("Device Time:");
      //GenerateReport#1
      timeManager.printTimeToSerial(currentTimerRecord);
      Serial.println("");
      Serial.println("rawCSWValue=" + String(rawCSWValue));
      Serial.println("cswV50Voltage=5.0 (fixed, Build 7)");
      Serial.println("factor=" + String(factor));
      Serial.println("cswOutput=" + String(cswOutput));
      Serial.println("");
      Serial.println("");

      int cleardata = generalFunctions.getValue(command, '#', 1).toInt();
      String functionname = "";
      if (digitalStablesData.currentFunctionValue == FUN_1_FLOW) {
        functionname = "FUN_1_FLOW";
      } else if (digitalStablesData.currentFunctionValue == FUN_2_FLOW) {
        functionname = "FUN_2_FLOW";
      } else if (digitalStablesData.currentFunctionValue == FUN_1_FLOW_1_TANK) {
        functionname = "FUN_1_FLOW_1_TANK";
      } else if (digitalStablesData.currentFunctionValue == FUN_1_TANK) {
        functionname = "FUN_1_TANK";
      } else if (digitalStablesData.currentFunctionValue == FUN_2_TANK) {
        functionname = "FUN_2_TANK";
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_SCEPTIC_TANK) {
        functionname = "DAFFODIL_SCEPTIC_TANK";
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_WATER_TROUGH) {
        functionname = "DAFFODIL_WATER_TROUGH";
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_TEMP_SOILMOISTURE) {
        functionname = "DAFFODIL_TEMP_SOILMOISTURE";
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_LIGHT_DETECTOR) {
        functionname = "DAFFODIL_LIGHT_DETECTOR";
      } else if (digitalStablesData.currentFunctionValue == VOLTAGE_MONITOR) {
        functionname = "VOLTAGE_MONITOR";
      }
      Serial.println("Current Function Value: " + functionname);
      Serial.println("");
      Serial.println("");

      dataManager.exportDSDCSV();
      if (cleardata) dataManager.clearAllDSDData();

      Serial.println("Ok-GenerateDSDReport");
      Serial.flush();
    } else if (command.startsWith("getDSDStoredCount")) {
      int count = dataManager.getDSDStoredCount();
      Serial.println(count);
      Serial.println("Ok-getDSDStoredCount");
      Serial.flush();
    } else if (command.startsWith("readStoredDSDData")) {
      int count = dataManager.getDSDStoredCount();
      DigitalStablesData dataArray[count];
      int actualSize = 0;
      if (dataManager.readAllDSDData(dataArray, count, actualSize)) {
        Serial.printf("Successfully read %d entries\n", actualSize);
        // Example: Print all entries
        for (int i = 0; i < actualSize; i++) {
          Serial.printf("\nEntry %d:\n", i);
          dataManager.printDigitalStablesData(dataArray[i]);
        }
      }
      Serial.println("Ok-readStoredDSDData");
      Serial.flush();
    } else if (command.startsWith("SetTime")) {
      //SetTime#25#5#25#2#22#24#30
      // SetTime#26#5#26#3#14#29#50
      // SetTime#1#6#25#1#19#43#00
      // SetTime#29#5#26#6#11#05#40
      
      timeManager.setTime(command);
      Serial.println("Ok-SetTime");
      Serial.flush();  // SetTime#24#1#25#6#17#21#20
    } else if (command.startsWith("GetDeviceSensorConfig")) {
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
    } else if (command.startsWith("SetDeviceSensorConfig")) {
      //SetDeviceSensorConfig#TopTank#TOPT#Flow#Tank#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#
      // SetDeviceSensorConfig#FISHTANK #FISH #Tank#Temp#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#
      // SetDeviceSensorConfig#SumpTrough #SUMP #Tank#Temp#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#
      // SetDeviceSensorConfig#Seedling #SEED #NoSensor#Temperature#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#
      //SetDeviceSensorConfig#Sceptic #SCEP #NoSensor#Temperature#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#40#50#
      //SetDeviceSensorConfig#GH Tank#GHTP #Tank#Temp#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#
      //SetDeviceSensorConfig#Creek Trough #CREEK #No Sensor#No Sensor#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#
      // SetDeviceSensorConfig#Big Cap #BIGC #No Sensor#No Sensor#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#20#50#
      // SetDeviceSensorConfig#DaffOffice#OFDA#NoSensor#Temperature#AEST-10AEDT,M10.1.0,M4.1.0/3#-37.13305556#144.47472222#410#40#50#
      String devicename = generalFunctions.getValue(command, '#', 1);
      String deviceshortname = generalFunctions.getValue(command, '#', 2);
      String sensor1name = generalFunctions.getValue(command, '#', 3);
      String sensor2name = generalFunctions.getValue(command, '#', 4);

      String timezone = generalFunctions.getValue(command, '#', 5);
      Serial.print("deviceshortname=");
      Serial.println(deviceshortname);
      Serial.print("devicename=");
      Serial.println(devicename);
      double latitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 6));
      double longitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 7));
      double altitude = generalFunctions.stringToDouble(generalFunctions.getValue(command, '#', 8));
      digitalStablesData.minimumEfficiencyForLed = generalFunctions.getValue(command, '#', 9).toInt();
      digitalStablesData.minimumEfficiencyForWifi = generalFunctions.getValue(command, '#', 10).toInt();

      uint8_t devicenamelength = devicename.length() + 1;
      devicename.toCharArray(digitalStablesData.devicename, devicenamelength);
      deviceshortname.toCharArray(digitalStablesData.deviceshortname, deviceshortname.length() + 1);
      sensor1name.toCharArray(digitalStablesData.sensor1name, sensor1name.length() + 1);
      sensor2name.toCharArray(digitalStablesData.sensor2name, sensor2name.length() + 1);

      Serial.print(F("digitalStablesData.minimumEfficiencyForLed="));
      Serial.println(digitalStablesData.minimumEfficiencyForLed);
      secretManager.saveDeviceSensorConfig(devicename, deviceshortname, sensor1name, sensor2name, timezone, latitude, longitude, altitude, digitalStablesData.minimumEfficiencyForLed, digitalStablesData.minimumEfficiencyForWifi);

      Serial.println(F("Ok-SetDeviceSensorConfig"));
    } else if (command.startsWith("SetDeviceName")) {
      String devicename = generalFunctions.getValue(command, '#', 1);
      uint8_t devicenamelength = devicename.length() + 1;
      devicename.toCharArray(digitalStablesData.devicename, devicenamelength);
      Serial.println(F("Ok-SetDeviceName"));
    } else if (command.startsWith("SetDeviceShortName")) {
      String deviceshortname = generalFunctions.getValue(command, '#', 1);
      uint8_t deviceshortnamelength = deviceshortname.length() + 1;
      deviceshortname.toCharArray(digitalStablesData.deviceshortname, deviceshortnamelength);
      Serial.print(F("digitalStablesData.deviceshortname="));
      Serial.println(digitalStablesData.deviceshortname);
      Serial.println(F("Ok-SetDeviceShortName"));
    } else if (command.startsWith("SetGroupId")) {
      String grpId = generalFunctions.getValue(command, '#', 1);
      secretManager.setGroupIdentifier(grpId);
      Serial.print(F("set group id to "));
      Serial.println(grpId);

      Serial.println(F("Ok-SetGroupId"));
    } else if (command.startsWith("ConfigWifiSTA")) {
      // ConfigWifiSTA#ssid#password
      // ConfigWifiSTA#MainRouter24##GardenShed#
      // ConfigWifiSTA#MainRouter24##TestOffice#

      String ssid = generalFunctions.getValue(command, '#', 1);
      String password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);
      bool staok = wifiManager.configWifiSTA(ssid, password, hostname);
      if (staok) {
        leds[0] = CRGB(0, 0, 255);
      } else {
        leds[0] = CRGB(255, 0, 0);
      }
      FastLED.show();
      Serial.println("Ok-ConfigWifiSTA");
    } else if (command.startsWith("ConfigWifiAP")) {
      // ConfigWifiAP#soft_ap_ssid#soft_ap_password#hostaname
      // ConfigWifiAP#GHTank##GHTank#
      // ConfigWifiAP#TopTank##TopTank#
      // ConfigWifiAP#TestOffice##TestOffice#
      //ConfigWifiAP#SumpTrough##SumpTrough#

      String soft_ap_ssid = generalFunctions.getValue(command, '#', 1);
      String soft_ap_password = generalFunctions.getValue(command, '#', 2);
      String hostname = generalFunctions.getValue(command, '#', 3);

      bool stat = wifiManager.configWifiAP(soft_ap_ssid, soft_ap_password, hostname);
      Serial.print("ConfigWifiAP result=");
      Serial.println(stat);
      if (stat) {
        leds[0] = CRGB(0, 255, 0);
      } else {
        leds[0] = CRGB(255, 0, 0);
      }
      FastLED.show();
      Serial.println("Ok-ConfigWifiAP");
    } else if (command.startsWith("GetTime")) {
      timeManager.printTimeToSerial(currentTimerRecord);
      Serial.flush();
      Serial.println("Ok-GetTime");
      Serial.flush();
    } else if (command.startsWith("GetCommandCode")) {
      long code = secretManager.generateCode();
      //
      // patch a bug in the totp library
      // if the first digit is a zero, it
      // returns a 5 digit number
      if (code < 100000) {
        Serial.print("0");
        Serial.println(code);
      } else {
        Serial.println(code);
      }

      long *history = secretManager.getCommandCodeHistory();
      Serial.println("Using returned array pointer:");
      for (int i = 0; i < 5; i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.println(history[i]);
      }

      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("VerifyUserCode")) {
      String codeInString = generalFunctions.getValue(command, '#', 1);
      long userCode = codeInString.toInt();
      boolean validCode = true;  // secretManager.checkCode( userCode);
      String result = "Failure-Invalid Code";
      if (validCode)
        result = "Ok-Valid Code";
      Serial.println(result);
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetSecret")) {
      // char secretCode[SHARED_SECRET_LENGTH];
      String secretCode = secretManager.readSecret();
      Serial.println(secretCode);
      Serial.println("Ok-GetSecret");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("SetSecret")) {
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
    } else if (command == "Flush") {
      while (Serial.read() >= 0)
        ;
      Serial.println("Ok-Flush");
      Serial.flush();
    } else if (command.startsWith("GetSerialNumber")) {
      Serial.println(serialNumber);
      Serial.flush();
    } else if (command.startsWith("PulseStart")) {
      inPulse = true;
      Serial.println("Ok-PulseStart");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("PulseFinished")) {
      inPulse = false;
      Serial.println("Ok-PulseFinished");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("IPAddr")) {
      currentIpAddress = generalFunctions.getValue(command, '#', 1);
      Serial.println("Ok-IPAddr");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("SSID")) {
      currentSSID = generalFunctions.getValue(command, '#', 1);
      Serial.println("Ok-currentSSID");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetIpAddress")) {
      Serial.println(ipAddress);
      Serial.println("Ok-GetIpAddress");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetSensorData")) {
      DigitalStablesDataSerializer digitalStablesDataSerializer;
      digitalStablesDataSerializer.pushToSerial(Serial, digitalStablesData);
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("AsyncData")) {
      Serial.print("AsyncCycleUpdate#");
      Serial.println("#");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetLifeCycleData")) {
      Serial.println("Ok-GetLifeCycleData");
      Serial.flush();
    } else if (command.startsWith("GetWPSSensorData")) {
      Serial.println("Ok-GetWPSSensorData");
      Serial.flush();
    } else if (command.startsWith("GetHourlySolarPowerData")) {
      HourlySolarPowerData hourlySolarPowerData = solarInfo->calculateActualPower(currentTimerRecord);
      Serial.print(" line 1807 efficiency=");
      Serial.println(hourlySolarPowerData.efficiency);
      Serial.print("actualPower=");
      Serial.println(hourlySolarPowerData.actualPower);
      Serial.print("irradiance=");
      Serial.println(hourlySolarPowerData.irradiance);
      Serial.print("temperature=");
      Serial.println(hourlySolarPowerData.temperature);
    } else if (command.startsWith("GetDailySolarPowerSchedule")) {
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

      for (int i = 0; i < 48; i++) {
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
    } else {
      //
      // call read to flush the incoming
      //
      Serial.println("Failure-Command Not Found-" + command);
      Serial.flush();
      delay(delayTime);
    }
  }
}

void setStationMode(String ipAddress) {
  Serial.println("settting Station mode, address ");
  Serial.println(ipAddress);
}

void setApMode() {
  Serial.println("settting AP mode");
  String apAddress = wifiManager.getApAddress();
  Serial.println("settting AP mode, address ");
  Serial.println(apAddress);
}
