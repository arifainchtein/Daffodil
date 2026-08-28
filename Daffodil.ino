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
Adafruit_INA219 solarIna219(0x45);  // Wally USB/panel current sensor — same 50mΩ shunt (LVK12R050CER) as ina219(0x41)
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
RTC_DATA_ATTR static bool     rtc_diagnosticsEnabled = false;      // set/cleared remotely via EnableDiagnostics/DisableDiagnostics
RTC_DATA_ATTR static uint8_t  rtc_activeDiagnosticType = DIAGNOSTIC_TYPE_NONE;

TxCurrentDiagnosticPayload pendingTxDiagnostic = {};  // filled during sendMessage(), consumed right after

String currentSSID;
String ipAddress = "";
boolean initiatedWifi = false;
// #define address 0x40
SHTSensor sht;
bool debug = false;
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
static unsigned long flowMeterPreviousMillis = 0;
static unsigned long flowMeterPreviousMillis2 = 0;

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
float v50iCloudyThreshold = 3.5;     // V50_I below this during solar hours means the panel isn't harvesting enough — cloudy. Lowered 2026-07-24: field data (TopTank, clear midday sun, actively charging) showed V50_I sitting at 4.09-4.47V — the old 4.5V threshold was false-triggering CLOUDY on every wake. Still wants validation against real overcast-day readings.
float panelCurrentCloudyThreshold_mA = 15.0;  // Wally 0x45 panelCurrent below this during solar hours means cloudy. Lowered 2026-07-24: field data (TopTank, clear midday sun) showed panelCurrent as low as 22-80mA during normal charging — the old 60mA threshold was false-triggering CLOUDY. Still wants validation against real overcast-day readings.
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
bool foundINA219Solar = false;  // Wally 0x45 panel/USB current sensor — optional, may be absent on unverified/older Wally boards
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
// Alternates the sensor-value LED display between slot 1 (pin 18) and slot 2 (pin 33) on
// two-slot modes (FUN_2_FLOW, FUN_2_TANK, FUN_1_FLOW_1_TANK, DAFFODIL_WATER_TROUGH_TANK1) —
// same toggle-per-cycle pattern as showTemperature.
boolean showSensorSlot2 = false;
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
  } else if constexpr (std::is_same<T, DiagnosticRecord>::value) {
    checksumOffset = offsetof(DiagnosticRecord, checksum);
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

  if constexpr (std::is_same<T, DigitalStablesData>::value || std::is_same<T, RequestCommand>::value || std::is_same<T, DiagnosticRecord>::value) {
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
      bool ledsWereOn = digitalRead(LED_CONTROL);
      if (ledsWereOn) digitalWrite(LED_CONTROL, LOW);  // kill LEDs during TX to reduce V50 sag
      int beginPacketResult = LoRa.beginPacket();
      // beginPacketResult==0 means LoRa.isTransmitting() already read true at this point —
      // beginPacket() then skips resetting the FIFO pointer/length entirely, so the write()
      // below lands on stale FIFO state. Never checked before; pollIterations has been
      // reading 0 (radio never actually shows MODE_TX) every time this session so far.
      if (debug) {
        Serial.print("beginPacketResult=");
        Serial.println(beginPacketResult);
      }
      LoRa.write((uint8_t *)&dataToSend, sizeof(T));
      long start = millis();
      bool capturingTxDiagnostic = rtc_diagnosticsEnabled && rtc_activeDiagnosticType == DIAGNOSTIC_TYPE_TX_CURRENT && foundINA219;
      // Only take the async+poll path during an actual TX-current diagnostic capture — it was
      // previously also triggered by `debug` alone (i.e. every send all session), but the async
      // LoRa.endPacket(true) + isTransmitting() polling never once observed the radio as busy on
      // this hardware/library combo (confirmed via pollIterations==0, every test, 2026-07-24).
      // LoRa.endPacket(false) (the else branch below) blocks on the real IRQ_TX_DONE_MASK flag,
      // so it's the one that's actually been transmitting correctly.
      if (capturingTxDiagnostic && foundINA219) {
        if (capturingTxDiagnostic) {
          pendingTxDiagnostic.sampleCount = 0;
          pendingTxDiagnostic.v50i_mV = (uint16_t)(digitalStablesData.v50Voltage * 1000);
          pendingTxDiagnostic.mAPre = (uint16_t)ina219.getCurrent_mA();
        }
        if (debug) {
          Serial.print("TX-CURRENT pre v50i=");
          Serial.print(digitalStablesData.v50Voltage);
          Serial.print(" mA=");
          Serial.println(ina219.getCurrent_mA());
        }
        LoRa.endPacket(true);
        uint16_t pollIterations = 0;
        while (LoRa.isTransmitting()) {
          pollIterations++;
          uint16_t sampleMa = (uint16_t)ina219.getCurrent_mA();
          if (debug) {
            Serial.print("TX-CURRENT t=");
            Serial.print(millis() - start);
            Serial.print("ms mA=");
            Serial.println(sampleMa);
          }
          if (capturingTxDiagnostic && pendingTxDiagnostic.sampleCount < DIAGNOSTIC_TX_MAX_SAMPLES) {
            uint8_t i = pendingTxDiagnostic.sampleCount;
            pendingTxDiagnostic.samples[i].offsetMs = (uint16_t)(millis() - start);
            pendingTxDiagnostic.samples[i].milliamps = sampleMa;
            pendingTxDiagnostic.sampleCount++;
          }
        }
        if (capturingTxDiagnostic) {
          pendingTxDiagnostic.mAPost = (uint16_t)ina219.getCurrent_mA();
        }
        if (debug) {
          Serial.print("TX-CURRENT post mA=");
          Serial.print(ina219.getCurrent_mA());
          Serial.print(" pollIterations=");
          Serial.print(pollIterations);
          Serial.print(" isTransmittingElapsedMs=");
          Serial.println(millis() - start);
          if (pollIterations == 0) {
            // Never observed MODE_TX (0x03) in REG_OP_MODE (0x01) — dump every register so we
            // can see what mode/IRQ state the chip actually reports right after the TX write,
            // instead of just the boolean isTransmitting() check.
            Serial.println("pollIterations==0 — dumping LoRa registers:");
            LoRa.dumpRegisters(Serial);
          }
        }
        // pollIterations==0 means isTransmitting() never once read the radio as busy —
        // i.e. REG_OP_MODE never showed MODE_TX after we wrote it, meaning the chip likely
        // never actually radiated this packet even though endPacket()/the code path here
        // reports success unconditionally (its return value is never itself meaningful).
        result = LORA_OK;
      } else {
        if (!LoRa.endPacket(false)) {
          result = LORA_TX_FAILED;
        } else {
          result = LORA_OK;
        }
      }
      if (ledsWereOn) digitalWrite(LED_CONTROL, HIGH);  // restore LEDs after TX
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
      } else if (commandcode.startsWith("EnableDiagnostics")) {
        int idx = commandcode.indexOf('#');
        rtc_activeDiagnosticType = (idx >= 0) ? (uint8_t)commandcode.substring(idx + 1).toInt() : DIAGNOSTIC_TYPE_TX_CURRENT;
        rtc_diagnosticsEnabled = true;
        if (debug) Serial.print("Diagnostics enabled, type=");
        if (debug) Serial.println(rtc_activeDiagnosticType);
      } else if (commandcode == "DisableDiagnostics") {
        rtc_diagnosticsEnabled = false;
        rtc_activeDiagnosticType = DIAGNOSTIC_TYPE_NONE;
        if (debug) Serial.println("Diagnostics disabled");
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
    // Log full — drop the oldest record. Previously read the WHOLE log into a
    // CommaRecord[COMMA_LOG_MAX_RECORDS] stack array (650 * 31 bytes ≈ 20KB) — that overflowed
    // the ~8KB default loop-task stack the instant the log actually filled up, hard-resetting
    // the device, which then hit the exact same overflow again on every subsequent boot (the
    // oversized log file survives a reset). Stream the trim in small fixed-size chunks instead,
    // so memory use stays constant regardless of COMMA_LOG_MAX_RECORDS.
    File rd = LittleFS.open(COMMA_LOG_FILE, "r");
    File wr = LittleFS.open("/comma_log.tmp", "w");
    if (rd && wr) {
      rd.seek(sizeof(CommaRecord));  // skip the oldest record
      uint8_t chunk[sizeof(CommaRecord) * 8];
      int n;
      while ((n = rd.read(chunk, sizeof(chunk))) > 0) {
        wr.write(chunk, n);
      }
      wr.write((uint8_t*)&rec, sizeof(rec));
    }
    if (rd) rd.close();
    if (wr) wr.close();
    LittleFS.remove(COMMA_LOG_FILE);
    LittleFS.rename("/comma_log.tmp", COMMA_LOG_FILE);
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

// Averages several single-shot ADS1115 reads of one channel. Each call to ADS.readADC() blocks
// for a fresh conversion (confirmed: this library's single-shot mode always requests a new
// conversion and waits for it — no stale-mux-read risk), so this just reduces sensitivity to
// any one noisy sample (e.g. charge-circuit switching transients) by averaging several.
int16_t readADCAveraged(uint8_t channel, uint8_t samples) {
  long total = 0;
  for (uint8_t i = 0; i < samples; i++) {
    total += ADS.readADC(channel);
  }
  return (int16_t)(total / samples);
}

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

// Resets an INA219 and writes the custom 50mΩ-shunt / 1A-range calibration used by both
// ina219(0x41, battery) and solarIna219(0x45, Wally panel/USB) — same shunt part on both boards.
void configureINA219Calibration(uint8_t i2cAddr) {
  Wire.beginTransmission(i2cAddr);
  Wire.write(0x00);  // Config register
  Wire.write(0x80);  // Reset bit
  Wire.write(0x00);
  Wire.endTransmission();
  delay(50);  // Wait for reset

  uint16_t calibrationValue = (uint16_t)(0.04096 / (CURRENT_LSB * SHUNT_OHMS));

  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V | INA219_CONFIG_GAIN_8_320MV |  // Higher gain for better resolution
                    INA219_CONFIG_BADCRES_12BIT | INA219_CONFIG_SADCRES_12BIT_1S_532US | INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  Wire.beginTransmission(i2cAddr);
  Wire.write(0x00);  // Config register
  Wire.write((config >> 8) & 0xFF);
  Wire.write(config & 0xFF);
  Wire.endTransmission();

  Wire.beginTransmission(i2cAddr);
  Wire.write(0x05);  // Calibration register
  Wire.write((calibrationValue >> 8) & 0xFF);
  Wire.write(calibrationValue & 0xFF);
  Wire.endTransmission();
}

// Packs the boot-time I2C found* flags for DIAGNOSTIC_TYPE_I2C_STATUS — see bit layout
// comment on I2CStatusDiagnosticPayload in DigitalStablesData.h.
uint8_t buildI2CStatusMask() {
  uint8_t mask = 0;  // bit7 reserved — DS18B20 is OneWire, not I2C, deliberately excluded here
  if (foundlcd)         mask |= 0x01;
  if (foundtemp)        mask |= 0x02;
  if (foundADS)         mask |= 0x04;
  if (foundBH1750)      mask |= 0x08;
  if (foundINA219)      mask |= 0x10;
  if (PCF8563T)         mask |= 0x20;
  if (foundINA219Solar) mask |= 0x40;
  return mask;
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
  // getDeviceSensorConfig() above leaves timezone as the Preferences default "NoData" unless
  // it was ever explicitly set (SetDeviceSensorConfig, or now SetTimezone) - parseTimezone()
  // then finds no +/- and silently leaves baseOffset at 0, so every epoch computed from the
  // RTC (getCurrentTimeInSeconds(), and TOTP code generation) treated local wall-clock time
  // as if it were UTC, landing exactly 10h/11h off. Fall back to a sane default (Melbourne)
  // rather than leaving it broken - but only if nothing valid has actually been configured,
  // so a real SetTimezone selection persists across reboots.
  if(timezone=="" || timezone=="NoData"){
    timezone = "AEST-10AEDT,M10.1.0,M4.1.0/3";
  }



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
if (debug) Serial.println( timeManager.printTimeToSerial(  currentTimerRecord));
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
      } else if (address == 69) {  // 0x45 — Wally USB/panel current sensor, optional
        foundINA219Solar = true;
        if (debug) Serial.println("foundINA219Solar");
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
    configureINA219Calibration(0x41);
  }

  // Wally USB/panel current sensor — optional. Absent on boards without the current-sensor
  // upgrade (e.g. Wally builds before 18) or if that hardware hasn't been verified yet;
  // panelVoltage/panelCurrent fall back to -99 in readSensorData() when this is false.
  if (foundINA219Solar) {
    if (!solarIna219.begin()) {
      if (debug) Serial.println("Failed to find solar INA219 (0x45) chip");
      foundINA219Solar = false;
    } else {
      if (debug) Serial.println("initialized solar INA219 (0x45)");
      configureINA219Calibration(0x45);
    }
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
  // V50_I (raw solar/USB input, pre-diode) — NOT battery voltage, despite the field name.
  // On boards where R1 feeds this channel from V50_I instead of the regulated V50 rail, it
  // tracks actual panel output: ~0V at night, sags with cloud cover, ~5V+ in full sun. Used
  // as a real power-harvest signal for cloudy detection (see v50iCloudyThreshold below).
  // Averaged over 8 samples — a single-shot read here was landing switch positions in the wrong
  // threshold band under real battery-attached load (charge-circuit switching transients riding
  // on this same rail), even though the switch/resistor network itself checked out correct on
  // a multimeter. See 2026-08-25 hardware debugging: 00001 (solar bit only) intermittently read
  // as if the solar resistor wasn't bypassed even with confirmed continuity to GND.
  int16_t val_3 = readADCAveraged(3, 8);
  float f = ADS.toVoltage(1);  //  voltage factor
  digitalStablesData.v50Voltage = ADS.toVoltage(val_3);

  if (debug) Serial.print("i setup v50Voltage(V50_I)=");
  if (debug) Serial.println(digitalStablesData.v50Voltage);

  // Config Switch
  // if(debug)Serial.print("voltage factor=");
  // if(debug)Serial.println(f);
  rawCSWValue = readADCAveraged(2, 8);
  if (debug) Serial.print("rawCSWValue=");
  if (debug) Serial.println(rawCSWValue);
  // V50/V50_I is NOT a stable 5V rail on this board: it tracks the solar/USB input and sags
  // with cloud cover, battery charge/discharge current, etc. Normalizing cswOutput to "what it
  // would read at 5.00V" cancels most of that drift so the thresholds below stay meaningful
  // regardless of current sun/battery conditions.
  //
  // 2026-08-25/26: abandoned trying to derive the threshold table from the R2/R3/R4/R10/R11/R13
  // resistor math — real hardware repeatedly disagreed with it in ways that didn't fit a single
  // consistent model (see chat/git history for the full trail: a stack-overflow bug, a false
  // "battery vs no-battery regime" theory, etc.). Replaced entirely with a full 32-position
  // empirical sweep (battery attached, DaffodilCSWTest sketch, 8x-averaged reads), normalized
  // here to K=5.00 and to a per-reading v50Voltage rather than one assumed constant — the whole
  // dataset was taken at a stable ~4.33-4.38V, so K itself barely matters here; what matters is
  // the *shape* of the 32 real readings, which is what the thresholds below now encode directly.
  //
  // Two real hardware quirks the data exposed (not measurement error — both reproduced across
  // repeated tests, and the second was independently confirmed with an unpowered multimeter):
  //  - Switch 1 or switch 2 alone (i.e. the *only* other switch besides solar) does something
  //    other than simply bypassing its resistor: 10001 reads *higher* than 00001 (should be
  //    lower), and 01001 reads almost identical to 00001 (should be clearly lower). Both switches
  //    work correctly in every other combination tested (e.g. with switch 4 also on).
  //  - This makes FUN_1_FLOW and FUN_1_FLOW_1_TANK, with solar on, land only ~120 raw counts
  //    apart (00001 vs 01001) — the narrowest margin in the whole table by a wide margin, and
  //    genuinely unreliable. If you rely on distinguishing these two specifically with solar
  //    power active, treat that as a known weak spot, not a solved threshold-tuning problem.
  factor = (digitalStablesData.v50Voltage > 0.5) ? (5.00 / digitalStablesData.v50Voltage) : 1.0;
  cswOutput = rawCSWValue * factor;


  //if (debug)
   Serial.print("corrected cswOutput=");
  //if (debug) 
  Serial.println(cswOutput);

  //
  // DIP switch bit patterns — bits 1-4 = switches bypassing R3/R4/R10/R11, 5th = solar
  // (bypasses R13). Function assignment is identical for both solar states of a given 4-bit
  // pattern. Thresholds below come from a full 32-position empirical sweep (2026-08-27, fresh
  // battery, DaffodilCSWTest sketch, 8x-averaged reads, normalized to K=5.00 via each reading's
  // own v50Voltage).
  //
  // This REPLACES a 2026-08-26 sweep done with a different (since-discovered-dead) battery. That
  // one showed real, reproducible quirks — switch 1 or 2 alone with solar reading backwards from
  // the simple bypass model — that made FUN_1_FLOW and FUN_1_FLOW_1_TANK (solar on) only ~120
  // raw counts apart. This fresh-battery sweep is completely clean and monotonic with NO such
  // quirks — every position landed exactly where the simple bypass model predicts, in normal
  // bit-pattern order. Conclusion: that quirk was an artifact of the old, failing battery's
  // loading behavior, not a board defect. However: swapping battery units also shifted the
  // *absolute* values substantially and non-uniformly (e.g. 00000 shifted +25%, 00001 shifted
  // +50% between the two batteries) — current draw and static resistance were both ruled out as
  // the cause, and it wasn't explained before moving on. If positions drift off-threshold again
  // after a battery swap in the future, don't assume these numbers still hold — re-sweep with
  // DaffodilCSWTest rather than trying to patch the existing thresholds.
  //
  // 1234 solar | raw@v50            | function
  // 0000  0    | 8326 @ 4.390       | FUN_1_FLOW
  // 1000  0    | 8140 @ 4.394       | FUN_2_FLOW
  // 0100  0    | 7950 @ 4.393       | FUN_1_FLOW_1_TANK
  // 1100  0    | 7758 @ 4.395       | FUN_1_TANK
  // 0010  0    | 7581 @ 4.392       | FUN_2_TANK
  // 1010  0    | 7379 @ 4.393       | DAFFODIL_SCEPTIC_TANK
  // 0110  0    | 7174 @ 4.391       | DAFFODIL_WATER_TROUGH
  // 1110  0    | 6965 @ 4.398       | DAFFODIL_WATER_TROUGH
  // 0001,1001,0101,1101  0 (merged) | 6689-6012 @ ~4.39 | (unassigned)
  // 0011  0    | 5800 @ 4.394       | DAFFODIL_WATER_TROUGH
  // 1011,0111,1111  0 (merged)      | 5558-5060 @ ~4.39 | (unassigned)
  // 0000  1    | 4806 @ 4.392       | FUN_1_FLOW
  // 1000  1    | 4541 @ 4.390       | FUN_2_FLOW
  // 0100  1    | 4271 @ 4.392       | FUN_1_FLOW_1_TANK
  // 1100  1    | 3993 @ 4.403       | FUN_1_TANK
  // 0010  1    | 3737 @ 4.399       | FUN_2_TANK
  // 1010  1    | 3445 @ 4.400       | DAFFODIL_SCEPTIC_TANK
  // 0110  1    | 3147 @ 4.400       | DAFFODIL_WATER_TROUGH
  // 1110  1    | 2841 @ 4.400       | DAFFODIL_WATER_TROUGH
  // 0001,1001,0101,1101  1 (merged) | 2437-1433 @ ~4.40 | (unassigned)
  // 0011  1    | 1116 @ 4.403       | DAFFODIL_WATER_TROUGH
  // 1011,0111,1111  1 (merged)      | 753,-3,-3 @ ~4.40 | (unassigned, saturates near 0)
  if (cswOutput >= 9373) {
    // 00000, solar off — FUN_1_FLOW
    digitalStablesData.currentFunctionValue = FUN_1_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, sizeof(digitalStablesData.sensor1name));
    usingSolarPower = false;
  } else if (cswOutput >= 9156 && cswOutput < 9373) {
    // 10000, solar off — FUN_2_FLOW
    digitalStablesData.currentFunctionValue = FUN_2_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, sizeof(digitalStablesData.sensor1name));
    secretManager.readFlow2Name().toCharArray(digitalStablesData.sensor2name, sizeof(digitalStablesData.sensor2name));
    usingSolarPower = false;
  } else if (cswOutput >= 8937 && cswOutput < 9156) {
    // 01000, solar off — FUN_1_FLOW_1_TANK
    digitalStablesData.currentFunctionValue = FUN_1_FLOW_1_TANK;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, sizeof(digitalStablesData.sensor1name));
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, sizeof(digitalStablesData.sensor2name));
    usingSolarPower = false;
  } else if (cswOutput >= 8728 && cswOutput < 8937) {
    // 11000, solar off — FUN_1_TANK
    digitalStablesData.currentFunctionValue = FUN_1_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, sizeof(digitalStablesData.sensor1name));
    usingSolarPower = false;
  } else if (cswOutput >= 8515 && cswOutput < 8728) {
    // 00100, solar off — FUN_2_TANK
    digitalStablesData.currentFunctionValue = FUN_2_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, sizeof(digitalStablesData.sensor1name));
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, sizeof(digitalStablesData.sensor2name));
    usingSolarPower = false;
  } else if (cswOutput >= 8284 && cswOutput < 8515) {
    // 10100, solar off — DAFFODIL_SCEPTIC_TANK
    digitalStablesData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;
    usingSolarPower = false;
  } else if (cswOutput >= 8044 && cswOutput < 8284) {
    // 01100, solar off — DAFFODIL_WATER_TROUGH
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = false;
  } else if (cswOutput >= 7766 && cswOutput < 8044) {
    // 11100, solar off — DAFFODIL_WATER_TROUGH
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = false;
  } else if (cswOutput >= 6721 && cswOutput < 7766) {
    // 0001/1001/0101/1101, solar off — unassigned (merged: none of these four carry a
    // function, so one wide band is as safe as four narrow ones and far simpler)
    usingSolarPower = false;
  } else if (cswOutput >= 6464 && cswOutput < 6721) {
    // 00110, solar off — DAFFODIL_WATER_TROUGH
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = false;
  } else if (cswOutput >= 5617 && cswOutput < 6464) {
    // 1011/0111/1111, solar off — unassigned (merged, same reasoning as above)
    usingSolarPower = false;
  } else if (cswOutput >= 5322 && cswOutput < 5617) {
    // 00001, solar on — FUN_1_FLOW
    digitalStablesData.currentFunctionValue = FUN_1_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, sizeof(digitalStablesData.sensor1name));
    usingSolarPower = true;
  } else if (cswOutput >= 5017 && cswOutput < 5322) {
    // 10001, solar on — FUN_2_FLOW
    digitalStablesData.currentFunctionValue = FUN_2_FLOW;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    attachInterrupt(SENSOR_INPUT_2, pulseCounter2, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, sizeof(digitalStablesData.sensor1name));
    secretManager.readFlow2Name().toCharArray(digitalStablesData.sensor2name, sizeof(digitalStablesData.sensor2name));
    usingSolarPower = true;
  } else if (cswOutput >= 4698 && cswOutput < 5017) {
    // 01001, solar on — FUN_1_FLOW_1_TANK
    digitalStablesData.currentFunctionValue = FUN_1_FLOW_1_TANK;
    attachInterrupt(SENSOR_INPUT_1, pulseCounter, FALLING);
    secretManager.readFlow1Name().toCharArray(digitalStablesData.sensor1name, sizeof(digitalStablesData.sensor1name));
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, sizeof(digitalStablesData.sensor2name));
    usingSolarPower = true;
  } else if (cswOutput >= 4391 && cswOutput < 4698) {
    // 11001, solar on — FUN_1_TANK
    digitalStablesData.currentFunctionValue = FUN_1_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, sizeof(digitalStablesData.sensor1name));
    usingSolarPower = true;
  } else if (cswOutput >= 4081 && cswOutput < 4391) {
    // 00101, solar on — FUN_2_TANK
    digitalStablesData.currentFunctionValue = FUN_2_TANK;
    secretManager.readTank1Name().toCharArray(digitalStablesData.sensor1name, sizeof(digitalStablesData.sensor1name));
    secretManager.readTank2Name().toCharArray(digitalStablesData.sensor2name, sizeof(digitalStablesData.sensor2name));
    usingSolarPower = true;
  } else if (cswOutput >= 3745 && cswOutput < 4081) {
    // 10101, solar on — DAFFODIL_SCEPTIC_TANK
    digitalStablesData.currentFunctionValue = DAFFODIL_SCEPTIC_TANK;
    usingSolarPower = true;
  } else if (cswOutput >= 3402 && cswOutput < 3745) {
    // 01101, solar on — DAFFODIL_WATER_TROUGH
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = true;
  } else if (cswOutput >= 2997 && cswOutput < 3402) {
    // 11101, solar on — DAFFODIL_WATER_TROUGH
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = true;
  } else if (cswOutput >= 1447 && cswOutput < 2997) {
    // 0001/1001/0101/1101, solar on — unassigned (merged, same reasoning as the solar-off gaps)
    usingSolarPower = true;
  } else if (cswOutput >= 1061 && cswOutput < 1447) {
    // 00111, solar on — DAFFODIL_WATER_TROUGH
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = true;
  } else if (cswOutput >= 0 && cswOutput < 1061) {
    // 10111/01111/11111, solar on — unassigned, saturates near 0 raw
    usingSolarPower = true;
  } else if (cswOutput < 0) {
    digitalStablesData.currentFunctionValue = DAFFODIL_WATER_TROUGH;
    usingSolarPower = true;
  }
//  if (debug)
  Serial.print("currentFunctionValue=");
  // if (debug)
   Serial.println(digitalStablesData.currentFunctionValue);

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
    if(debug)Serial.println("Starting LoRa failed!");
    // drawLora(0);
    while (1)
      ;
    //  leds[1] = CRGB(255, 0, 0);
  } else {
    if(debug)  Serial.println("Starting LoRa worked!");
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
  // Set once per wake cycle here rather than relying solely on restartWifi(), which only
  // runs when WiFi is actually (re)started — otherwise digitalStablesData.loraActive stays
  // at its zero-init default on cycles where WiFi doesn't turn on, even though LoRa is active.
  digitalStablesData.loraActive = loraActive;

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

  String identifier = "Daffodil";
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


  // Cycle the radio through receive() at least once before any TX is attempted this boot —
  // going straight from LoRa.begin() to a cold transmit (as the isSleepMode branch below used
  // to do) left REG_OP_MODE never actually showing MODE_TX (pollIterations=0, confirmed via
  // TX-CURRENT diagnostics 2026-07-24): the SX127x's PLL/AGC apparently needs a receive cycle
  // first. sendMessage() itself switches back to idle/TX mode via LoRa_txMode(), so this is
  // safe to do unconditionally before goToSleep()'s send too.
  if (loraActive) {
    LoRa.onReceive(onReceive);
    LoRa.receive();
  }

  if (isSleepMode) {
    Serial.println("Calling deepsleep line 1378");
    goToSleep();
  } else {
    digitalStablesData.operatingStatus = OPERATING_STATUS_NO_LED;
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


void goToSleep() {
  // Heartbeat: show "B" battery status while LEDs are powered, then cut power.
  digitalWrite(LED_CONTROL, HIGH);
  delay(20);  // let MOSFET turn on and WS2812 power supply stabilise
  readSensorData();                     // get fresh voltage/current before display
  digitalStablesData.ledBrightness = 125;  // fixed heartbeat brightness — loop()'s adaptive value never ran on this path
  drawBatteryStatus(digitalStablesData.batteryVoltage, digitalStablesData.batteryCurrent);
  delay(1000);                          // hold the display for 1 s so it is visible
  FastLED.clear(true);
  FastLED.show();
  delay(10);
  digitalWrite(LED_CONTROL, LOW);

  // 1. Calculate sleep timing.
  //    PowerManager returns 60 s whenever theoretical solar efficiency > 0.3 (sun is up).
  //    On heavily overcast days this is wrong — V50_I (real panel output) or panelCurrent
  //    can be below their cloudy thresholds despite the geometric efficiency model saying
  //    sun is up. When either says cloudy, apply the same night formula so sleep time
  //    reflects how dark/underpowered it actually is.
  long seconds_sleep = powerManager->calculateOptimalSleepTime(currentTimerRecord);
  bool _lowV50IForSleep = foundADS && digitalStablesData.v50Voltage > 0 && digitalStablesData.v50Voltage < v50iCloudyThreshold;
  bool _lowPanelCurrentForSleep = foundINA219Solar && digitalStablesData.panelCurrent >= 0 && digitalStablesData.panelCurrent < panelCurrentCloudyThreshold_mA;
  if (usingSolarPower && (_lowV50IForSleep || _lowPanelCurrentForSleep)) {
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
  if (debug) Serial.printf("Preparing sleep for %ld seconds\n", seconds_sleep);

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

  // 3. Shut down WiFi and wait for it to fully stop BEFORE the final LoRa send — WiFi AP
  // activity (beaconing, or still ramping up if just started this boot) can draw enough
  // current to destabilize the power rail during TX (this board has already shown a real
  // brownout during ConfigWifiAP), which could silently corrupt/suppress the LoRa transmit
  // without either endPacket() call's return value being checked.
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  while (WiFi.getMode() != WIFI_OFF || WiFi.status() == WL_CONNECTED) {
    delay(100);
  }

  // 4. Send final LoRa message so the hub knows we are sleeping and for how long.
  // skipCAD=true: this is a critical notification — don't let a busy channel silence it.
  // WiFi is fully off by this point (see step 3) so it can't contend for power during TX.
  if (loraActive) {
    sendMessage(digitalStablesData, true);
  }
  LoRa.sleep();

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

//
// Flow meter 1 — SENSOR_INPUT_1 (pin 18), pulses counted by pulseCounter().
// Same convention as the sibling Rosie/Gloria/Pancho boards: pulses/sec scaled by qfactor1
// gives L/min, then converted to mL and accumulated for this sampling interval.
//
void readFlowMeter1() {
  noInterrupts();
  int pulseCount = flowMeterPulseCount;
  flowMeterPulseCount = 0;
  interrupts();

  unsigned long now = millis();
  long elapsedMs = now - flowMeterPreviousMillis;
  flowMeterPreviousMillis = now;
  if (elapsedMs <= 0) elapsedMs = 1;

  float flowRate = (1000.0 / elapsedMs) * pulseCount / digitalStablesData.qfactor1;
  if (flowRate > 0) {
    float flowMilliLitres = digitalStablesData.dataSamplingSec * (flowRate / 60) * 1000;
    digitalStablesData.totalMilliLitres += flowMilliLitres;
  }
  digitalStablesData.flowRate = flowRate;
  if (debug) Serial.print("flow1 pulses=");
  if (debug) Serial.print(pulseCount);
  if (debug) Serial.print(" flowRate=");
  if (debug) Serial.println(digitalStablesData.flowRate);
}

// Flow meter 2 — SENSOR_INPUT_2 (pin 33), pulses counted by pulseCounter2(). Same math as
// readFlowMeter1() against qfactor2/totalMilliLitres2.
void readFlowMeter2() {
  noInterrupts();
  int pulseCount = flowMeterPulseCount2;
  flowMeterPulseCount2 = 0;
  interrupts();

  unsigned long now = millis();
  long elapsedMs = now - flowMeterPreviousMillis2;
  flowMeterPreviousMillis2 = now;
  if (elapsedMs <= 0) elapsedMs = 1;

  float flowRate2 = (1000.0 / elapsedMs) * pulseCount / digitalStablesData.qfactor2;
  if (flowRate2 > 0) {
    float flowMilliLitres2 = digitalStablesData.dataSamplingSec * (flowRate2 / 60) * 1000;
    digitalStablesData.totalMilliLitres2 += flowMilliLitres2;
  }
  digitalStablesData.flowRate2 = flowRate2;
  if (debug) Serial.print("flow2 pulses=");
  if (debug) Serial.print(pulseCount);
  if (debug) Serial.print(" flowRate2=");
  if (debug) Serial.println(digitalStablesData.flowRate2);
}

//
// Tank pressure sensors — the screw terminals shared with SENSOR_INPUT_1/2 (pins 18/33) can be
// jumpered to route to the ADS1115 instead of the GPIO, for a 0.5V-4.5V/0-5psi transducer.
// ch1 = terminal shared with pin 18 (sensor 1 / tank1). ch0 = terminal shared with pin 33
// (sensor 2 / tank2). 0.5V=0psi, 4.5V=5psi (live-zero transducer, so 0V reads as a fault).
//
void readTankPressure1() {
  ADS.setGain(0);
  int16_t raw = ADS.readADC(1);
  float volts = raw * ADS.toVoltage(1);
  digitalStablesData.tank1PressurePsi = (volts - 0.5) * 1.25;
  if (debug) Serial.print("tank1 volts=");
  if (debug) Serial.print(volts);
  if (debug) Serial.print(" psi=");
  if (debug) Serial.println(digitalStablesData.tank1PressurePsi);
}

void readTankPressure2() {
  ADS.setGain(0);
  int16_t raw = ADS.readADC(0);
  float volts = raw * ADS.toVoltage(1);
  digitalStablesData.tank2PressurePsi = (volts - 0.5) * 1.25;
  if (debug) Serial.print("tank2 volts=");
  if (debug) Serial.print(volts);
  if (debug) Serial.print(" psi=");
  if (debug) Serial.println(digitalStablesData.tank2PressurePsi);
}

void readSensorData() {
  //
  // V50_I (raw solar/USB input, pre-diode) — see setup() for why this isn't battery voltage.
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
  // Ultrasonic shares TRIGGER/ECHO with pins 18/33 (SENSOR_INPUT_1/2), which double as the
  // flow-meter interrupts and tank-pressure analog terminals in the other modes — only trigger
  // it in the two modes where those pins are actually wired to the sonar (see Known Issues).
  if (digitalStablesData.currentFunctionValue == DAFFODIL_WATER_TROUGH || digitalStablesData.currentFunctionValue == DAFFODIL_SCEPTIC_TANK) {
    digitalStablesData.measuredHeight = sonar.ping_cm();
  } else {
    digitalStablesData.measuredHeight = -99;
  }
  if (debug) Serial.print("line 1654 measuredHeight=");
  if (debug) Serial.println(digitalStablesData.measuredHeight);

  switch (digitalStablesData.currentFunctionValue) {
    case FUN_1_FLOW:
      readFlowMeter1();
      break;
    case FUN_2_FLOW:
      readFlowMeter1();
      readFlowMeter2();
      break;
    case FUN_1_FLOW_1_TANK:
      readFlowMeter1();
      readTankPressure2();
      break;
    case FUN_1_TANK:
      readTankPressure1();
      break;
    case FUN_2_TANK:
      readTankPressure1();
      readTankPressure2();
      break;
    case DAFFODIL_WATER_TROUGH_TANK1:
      // tank1 (ch1/pin18) is safe to read here. The trough half is NOT wired up yet: `sonar`
      // is still a 2-pin NewPing instance hardwired to TRIGGER_PIN(18)/ECHO_PIN(33), which
      // would collide with tank1's pin18 pressure wiring in this mode. Needs a real single-wire
      // ultrasonic driver (not NewPing) before measuredHeight is meaningful for this mode —
      // until then it stays at the -99 sentinel set above.
      readTankPressure1();
      break;
  }

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

  // Wally USB/panel current sensor (0x45) — optional, not present on every board yet.
  // -99 sentinel (matches batteryCurrent's convention) when the sensor wasn't detected.
  if (foundINA219Solar) {
    digitalStablesData.panelVoltage = solarIna219.getBusVoltage_V();
    digitalStablesData.panelCurrent = solarIna219.getCurrent_mA();
    if (debug) {
      Serial.print("panelVoltage=");
      Serial.print(digitalStablesData.panelVoltage);
      Serial.print(" panelCurrent=");
      Serial.println(digitalStablesData.panelCurrent);
    }
  } else {
    digitalStablesData.panelVoltage = -99;
    digitalStablesData.panelCurrent = -99;
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
  digitalStablesData.wifiStatus = wifiManager.getWifiStatus();
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

  // Local ipAddress shadows the file-scope one GetIpAddress reports over serial -
  // without this the global stayed permanently "" no matter what happened above.
  ::ipAddress = ipAddress;

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

// Same red/yellow/green/blue fill-level bucketing already used for septic/voltage-monitor,
// factored out so the tank/trough LED code below doesn't re-derive it per mode.
CRGB percentBucketColor(float percent) {
  if (percent <= 25) return CRGB(255, 0, 0);
  if (percent <= 50) return CRGB(255, 255, 0);
  if (percent <= 75) return CRGB(0, 255, 0);
  return CRGB(0, 0, 255);
}

// Flow has no natural "fill level" — just whether it's currently moving.
CRGB flowStatusColor(float flowRate) {
  return flowRate > 0 ? CRGB(0, 0, 255) : CRGB(255, 0, 0);
}

// psi -> percent-full, using the same 1psi=0.7m head conversion as readTankPressure1/2().
float tankPercentFull(float psi, float heightMeters) {
  if (heightMeters <= 0) return 0;
  return constrain((psi * 0.7f) / heightMeters * 100.0f, 0.0f, 100.0f);
}

// The "tank/level tower" — unshifted (cols 1-3) for the three original single-value modes
// (septic, trough, voltage monitor), shifted (cols 0-2) for every mode that also lights an
// led4/led9 sensor-slot marker, so the marker never overlaps the tower itself.
void drawTower(CRGB c, bool shifted) {
  static const uint8_t kUnshifted[9] = { 1, 2, 3, 6, 7, 8, 11, 12, 13 };
  static const uint8_t kShifted[9] = { 0, 1, 2, 5, 6, 7, 10, 11, 12 };
  const uint8_t *idx = shifted ? kShifted : kUnshifted;
  for (uint8_t i = 0; i < 9; i++) leds[idx[i]] = c;
}

// "F" icon for flow-family modes.
void drawFlowSymbol(CRGB c) {
  static const uint8_t kIdx[6] = { 0, 1, 5, 6, 9, 10 };
  for (uint8_t i = 0; i < 6; i++) leds[kIdx[i]] = c;
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
          // We're already inside the "solar should be up" branch (efficiency > minimumEfficiencyForLed),
          // so a depressed V50_I here means the panel isn't harvesting despite theoretical daylight —
          // a direct power-harvest reading, unlike lux which only measures ambient brightness.
          bool v50iSaysCloudy = foundADS
                              && digitalStablesData.v50Voltage > 0
                              && digitalStablesData.v50Voltage < v50iCloudyThreshold;
          // Wally USB/panel current — a direct current reading, immune to the leakage-floor
          // ambiguity V50_I has (see panelVoltage/panelCurrent design notes): genuinely ~0mA
          // with no real panel output, unlike voltage which can float from charger leakage.
          bool panelCurrentSaysCloudy = foundINA219Solar
                              && digitalStablesData.panelCurrent >= 0
                              && digitalStablesData.panelCurrent < panelCurrentCloudyThreshold_mA;
          bool forecastSaysCloudy = false;
          if (secondsSinceLastWeatherData < 1860) {
            WeatherForecast* forecasts = weatherForecastManager->getForecasts();
            forecastSaysCloudy = (forecasts != nullptr) && (forecasts[0].cloudiness >= cloudyThreshold);
          }
          // Forecast is a prediction, not a measurement — it must never override direct
          // evidence that the panel is actually harvesting right now. If either live sensor
          // confirms real sun (current/voltage comfortably above their cloudy thresholds),
          // ignore the forecast entirely.
          bool panelCurrentConfirmsSun = foundINA219Solar && digitalStablesData.panelCurrent >= panelCurrentCloudyThreshold_mA;
          bool v50iConfirmsSun = foundADS && digitalStablesData.v50Voltage >= v50iCloudyThreshold;
          bool liveConfirmsSun = panelCurrentConfirmsSun || v50iConfirmsSun;
          digitalStablesData.operatingStatus = (v50iSaysCloudy || panelCurrentSaysCloudy || (forecastSaysCloudy && !liveConfirmsSun))
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
  if (usingSolarPower && hourlySolarPowerData.efficiency * 100 < digitalStablesData.minimumEfficiencyForLed){
    isSleepMode = true;
      if (debug) Serial.print("line 2601 isSleepMode=true because of effciency");
  } 
  // Protect battery from over-discharge (only when on solar/battery, not wall power)
  if (usingSolarPower && digitalStablesData.batteryVoltage >= commaVoltage && digitalStablesData.batteryVoltage < sleepingVoltage) {
    isSleepMode = true;
    if (debug) Serial.print("line 2604 isSleepMode=true because of low battery voltage");
  }
  // On overcast days the theoretical efficiency may still be above threshold but actual solar
  // is insufficient to sustain continuous operation. Sleep between each LoRa pulse.
  if (usingSolarPower && digitalStablesData.operatingStatus == OPERATING_STATUS_CLOUDY){
     isSleepMode = true;
      if (debug) Serial.print("line 2609 isSleepMode=true because of is cloudy");
  }
  if (isSleepMode) {
    if (digitalStablesData.operatingStatus != OPERATING_STATUS_CLOUDY) {
      digitalStablesData.operatingStatus = OPERATING_STATUS_SLEEP;
    }
    if (debug) Serial.print("going to sleep, bat=");
    if (debug) Serial.print(digitalStablesData.batteryVoltage);
    if (debug) Serial.print(" efficiency=");
    if (debug) Serial.print(hourlySolarPowerData.efficiency);
    if (debug) Serial.print(" operatingStatus=");
    if (debug) Serial.println(digitalStablesData.operatingStatus);
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
      digitalStablesData.wifiStatus = 0;
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
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 0, 0);
      }

      uint8_t fn = digitalStablesData.currentFunctionValue;

      if (fn == DAFFODIL_SCEPTIC_TANK || fn == DAFFODIL_WATER_TROUGH || fn == VOLTAGE_MONITOR) {
        // Original single-value display — untouched. scepticAvailablePercentage was dropped
        // from DigitalStablesData (purely derived, never needed on the wire) — recomputed here.
        float scepticAvailablePercentage = digitalStablesData.measuredHeight * 100 / MAX_DISTANCE;
        red = 255;
        green = 0;
        blue = 255;
        if (fn == DAFFODIL_SCEPTIC_TANK) {
          if (scepticAvailablePercentage <= 25) {
            red = 255;
            green = 0;
            blue = 0;
          } else if (scepticAvailablePercentage > 25 && scepticAvailablePercentage <= 50) {
            red = 255;
            green = 255;
            blue = 0;
          } else if (scepticAvailablePercentage > 50 && scepticAvailablePercentage <= 75) {
            red = 0;
            green = 255;
            blue = 0;
          } else if (scepticAvailablePercentage > 75) {
            red = 0;
            green = 0;
            blue = 255;
          }
        } else if (fn == DAFFODIL_WATER_TROUGH) {
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
        } else if (fn == VOLTAGE_MONITOR) {
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
        drawTower(CRGB(red, green, blue), false);

      } else if (fn == FUN_1_FLOW || fn == FUN_2_FLOW || fn == FUN_1_FLOW_1_TANK || fn == FUN_1_TANK || fn == FUN_2_TANK || fn == DAFFODIL_WATER_TROUGH_TANK1) {
        // Sensor-slot modes: slot1 = pin18/sensor1, slot2 = pin33/sensor2. A single-slot mode
        // just shows slot1 with led4 lit. A two-slot mode alternates each display cycle between
        // slot1 (led4) and slot2 (led9) — same toggle-per-cycle pattern as showTemperature.
        // led4/led9 are always blue: they only mark which slot is currently on screen, the
        // symbol's own color carries the actual status.
        bool twoSlots = (fn == FUN_2_FLOW || fn == FUN_2_TANK || fn == FUN_1_FLOW_1_TANK || fn == DAFFODIL_WATER_TROUGH_TANK1);
        bool slot2Now = twoSlots && showSensorSlot2;
        if (twoSlots) showSensorSlot2 = !showSensorSlot2;

        if (!slot2Now) {
          // slot 1
          if (fn == FUN_1_FLOW || fn == FUN_2_FLOW || fn == FUN_1_FLOW_1_TANK) {
            drawFlowSymbol(flowStatusColor(digitalStablesData.flowRate));
          } else {
            // FUN_1_TANK, FUN_2_TANK, DAFFODIL_WATER_TROUGH_TANK1 — slot1 is always tank1
            // (ch1/pin18); for the trough+tank1 mode that matches the mode's own name.
            drawTower(percentBucketColor(tankPercentFull(digitalStablesData.tank1PressurePsi, digitalStablesData.tank1HeightMeters)), true);
          }
          leds[4] = CRGB(0, 0, 255);
        } else {
          // slot 2
          if (fn == FUN_2_FLOW) {
            drawFlowSymbol(flowStatusColor(digitalStablesData.flowRate2));
          } else if (fn == FUN_1_FLOW_1_TANK) {
            drawTower(percentBucketColor(tankPercentFull(digitalStablesData.tank2PressurePsi, digitalStablesData.tank2HeightMeters)), true);
          } else if (fn == FUN_2_TANK) {
            drawTower(percentBucketColor(tankPercentFull(digitalStablesData.tank2PressurePsi, digitalStablesData.tank2HeightMeters)), true);
          } else if (fn == DAFFODIL_WATER_TROUGH_TANK1) {
            // slot2 here is the trough (ultrasonic) — see readSensorData() for why
            // measuredHeight is still -99 until the single-wire ultrasonic driver lands.
            CRGB c;
            if (digitalStablesData.measuredHeight >= (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelminimumcm)) {
              c = CRGB(255, 0, 0);
            } else if (digitalStablesData.measuredHeight >= (digitalStablesData.maximumScepticHeight - digitalStablesData.troughlevelmaximumcm)) {
              c = CRGB(0, 255, 0);
            } else {
              c = CRGB(0, 0, 255);
            }
            drawTower(c, true);
          }
          leds[9] = CRGB(0, 0, 255);
        }
      }

      FastLED.show();
    } else if (displayStatus == SHOW_INTERNET_STATUS) {
      wifistatus = wifiManager.getWifiStatus();
      if (debug) Serial.print("line 1112 inside of showintenrnetstatus wifiStatus=");
      if (debug) Serial.println(digitalStablesData.wifiStatus);
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
          if (digitalStablesData.wifiStatus==2) {
            leds[7] = CRGB(0, 0, 255);
          } else {
            leds[7] = CRGB(255, 0, 0);
          }
          FastLED.show();
          //
          //

          if (digitalStablesData.wifiStatus==2) {
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

            digitalStablesData.wifiStatus = wifiManager.getWifiStatus();
            if (debug) Serial.print("after rechecking digitalstable,wifiStatus=");
            if (debug) Serial.println(digitalStablesData.wifiStatus);
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

        if (rtc_diagnosticsEnabled && rtc_activeDiagnosticType == DIAGNOSTIC_TYPE_TX_CURRENT && pendingTxDiagnostic.sampleCount > 0) {
          DiagnosticRecord diagRecord;
          memcpy(diagRecord.serialnumberarray, digitalStablesData.serialnumberarray, 8);
          diagRecord.diagnosticType = rtc_activeDiagnosticType;
          diagRecord.payload.txCurrent = pendingTxDiagnostic;
          sendMessage(diagRecord);
          pendingTxDiagnostic.sampleCount = 0;  // consumed
        } else if (rtc_diagnosticsEnabled && rtc_activeDiagnosticType == DIAGNOSTIC_TYPE_I2C_STATUS) {
          // No sampling window needed — the found* flags are already known from the boot-time
          // I2C scan, so this sends on the very next cycle after EnableDiagnostics#2.
          DiagnosticRecord diagRecord;
          memcpy(diagRecord.serialnumberarray, digitalStablesData.serialnumberarray, 8);
          diagRecord.diagnosticType = rtc_activeDiagnosticType;
          diagRecord.payload.i2cStatus.deviceFoundMask = buildI2CStatusMask();
          sendMessage(diagRecord);
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
                     + "  luxNight<" + String(luxNightThreshold));

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
      Serial.println("cswV50Voltage=" + String(digitalStablesData.v50Voltage) + " (normalized to 5.445 via factor, not fixed)");
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
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_WATER_TROUGH_TANK1) {
        functionname = "DAFFODIL_WATER_TROUGH_TANK1";
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
      Serial.println("cswV50Voltage=" + String(digitalStablesData.v50Voltage) + " (normalized to 5.445 via factor, not fixed)");
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
      } else if (digitalStablesData.currentFunctionValue == DAFFODIL_WATER_TROUGH_TANK1) {
        functionname = "DAFFODIL_WATER_TROUGH_TANK1";
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
      sensor1name.toCharArray(digitalStablesData.sensor1name, min(sensor1name.length() + 1, sizeof(digitalStablesData.sensor1name)));
      sensor2name.toCharArray(digitalStablesData.sensor2name, min(sensor2name.length() + 1, sizeof(digitalStablesData.sensor2name)));

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
    } else if (command.startsWith("SetTimezone")) {
      // SetTimezone#AEST-10AEDT,M10.1.0,M4.1.0/3
      String tz = generalFunctions.getValue(command, '#', 1);
      secretManager.setTimeZone(tz);
      TimeUtils::parseTimezone(tz);
      Serial.println(F("Ok-SetTimezone"));
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
    } else if (command.startsWith("SetProductDefinition")) {
      // SetProductDefinition#<name>#<powerSource>#<battery>#<pcbs>#<firmware>
      String pdName = generalFunctions.getValue(command, '#', 1);
      String pdPowerSource = generalFunctions.getValue(command, '#', 2);
      String pdBattery = generalFunctions.getValue(command, '#', 3);
      String pdPcbs = generalFunctions.getValue(command, '#', 4);
      String pdFirmware = generalFunctions.getValue(command, '#', 5);
      secretManager.saveProductDefinition(pdName, pdPowerSource, pdBattery, pdPcbs, pdFirmware);
      Serial.println("Ok-SetProductDefinition");
      Serial.flush();
      delay(delayTime);
    } else if (command.startsWith("GetProductDefinition")) {
      String pdName, pdPowerSource, pdBattery, pdPcbs, pdFirmware;
      secretManager.getProductDefinition(pdName, pdPowerSource, pdBattery, pdPcbs, pdFirmware);
      unsigned long commissionDate = secretManager.getCommissionDate();
      // 1704067200 = 2024-01-01 - anything before that is the RTC's power-on/reset default
      // (e.g. year 2000), not a real commission date. Treat it the same as never-set and
      // recapture, instead of permanently keeping whatever garbage the RTC had the first
      // time this was called (e.g. before SetTime had ever actually run correctly).
      if (commissionDate < 1704067200UL) {
        commissionDate = timeManager.getCurrentTimeInSeconds(timeManager.now());
        secretManager.setCommissionDate(commissionDate);
      }
      // WiFi params configured via Upload Firmware (ConfigWifiSTA/ConfigWifiAP), persisted
      // by WifiManager::configWifiSTA/configWifiAP through secretManager.saveWifiParameters().
      String pdSSID = secretManager.getSSID();
      String pdWifiPassword = secretManager.getWifiPassword();
      String pdSoftAPSSID = secretManager.getSoftAPSSID();
      String pdSoftAPPassword = secretManager.getSoftAPPASS();
      String pdHostName = secretManager.getHostName();
      String pdStationMode = secretManager.getStationMode() ? "Station" : "AccessPoint";
      unsigned long pdCurrentTime = timeManager.getCurrentTimeInSeconds(timeManager.now());
      String pdDeviceName = String(digitalStablesData.devicename);
      String pdDeviceShortName = String(digitalStablesData.deviceshortname);
      char pdSerialNumberBuf[13];
      snprintf(pdSerialNumberBuf, sizeof(pdSerialNumberBuf), "%012llx", ESP.getEfuseMac());
      String pdSerialNumber = String(pdSerialNumberBuf);
      Serial.println("Ok-GetProductDefinition#" + pdName + "#" + pdPowerSource + "#" + pdBattery + "#" + pdPcbs + "#" + pdFirmware + "#" + String(commissionDate) + "#" + pdSSID + "#" + pdWifiPassword + "#" + pdSoftAPSSID + "#" + pdSoftAPPassword + "#" + pdHostName + "#" + pdStationMode + "#" + String(pdCurrentTime) + "#" + pdDeviceName + "#" + pdDeviceShortName + "#" + pdSerialNumber);
      Serial.flush();
      delay(delayTime);
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
    } else if (command.startsWith("RestartWifi")) {
      // Was never wired up to a serial command - UploadFirmwareProcessingHandler has
      // been sending this and getting the generic "Command Not Found" fallback.
      restartWifi();
      Serial.println("Ok-RestartWifi");
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
