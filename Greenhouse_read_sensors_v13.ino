#include <Key.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//#include "utility/NetTime.h"
#include <string.h>
//#include "utility/sntp.h"

#include <Adafruit_SleepyDog.h>
#include <Adafruit_CC3000.h>
#include <SPI.h>
#include "utility/debug.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_CC3000.h"
#include <ccspi.h>

#include <TimeLib.h>
#include <string.h>

#include "Statistic.h"

// Pot-specific sensor statistics
Statistic pot01_StatisticX; //declare Statistics object for photocells
Statistic pot02_StatisticX; //declare Statistics object for photocells
Statistic pot01_StatisticY; //declare Statistics object for humidity sensors
Statistic pot02_StatisticY; //declare Statistics object for humidity sensors
Statistic pot01_StatisticZ; //declare Statistics object for temperature sensors
Statistic pot02_StatisticZ; //declare Statistics object for temperature sensors

// *** LCD DEFINITIONS ***
LiquidCrystal_I2C lcd(0x3F, 16, 2); // I2C-display integr-module address: 0x3F / 63 (16x2 char LCD)

//*** KEYBOAD DEFINITIONS ***
char KBSwitch;

const byte ROWS = 4; //four rows
const byte COLS = 3; //three columns

char keys[ROWS][COLS] =
{ {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
byte rowPins[ROWS] = {44, 34, 36, 40}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {42, 46, 38}; //connect to the column pinouts of the keypad
char eventKey;
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

//const int sensorTypeCount = 3;
int potCount = 1;
int maxPotCount = 2;
const int classCount = 20;
int classIndex;

// Sensor declarations
int pot_ID = 1;
int sensorType;
int sensorCount;
int maxsensorCount = 6;
const int sensorTypeCount = 3;
int currentSensorID;
int flag = 0;

//for sensor statistics the last measurements
static int pot01_Measurements[sensorTypeCount][classCount] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // photocell
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // humidity sensor
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // temperature sensor
};
static int pot02_Measurements[sensorTypeCount][classCount] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // photocell
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // humidity sensor
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // temperature sensor
};

//Cumulative measurement result of the pot (3 sensors measured)
static int pot01_MeasurementClassTotals[sensorTypeCount][classCount] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Count of the sensorX measurements per class.
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Count of the sensorY measurements per class.
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // Count of the sensorZ measurements per class.
};
static int pot02_MeasurementClassTotals[sensorTypeCount][classCount] = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Count of the sensorX measurements per class.
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Count of the sensorY measurements per class.
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // Count of the sensorZ measurements per class.
};

// Sensor measuring intervals
int delay1 = 50;
int delay2 = 100;
int delay3 = 300;

// UI-delay
int delay4 = 2000; // For UI notification texts
int delay5 = 5000; // For UI notification texts
int delay6 = 5000; // For UI notification texts

// Communication delay (MQTT)
int delay7 = 1500;

// give greenhouse sensorLEDs a name: digital OUTPUTs
int led2 = 23;
int led3 = 25;
int led4 = 27;
int pin5 = 29;

//_________________________________________________________
// SENSOR CONFIGURATIONS
//3 sensors in two pots in the demo
//[sensorPin][0]: 0 = no pot, 1 = pot_1, 2 = pot_2
//[sensorPin][1]: 0 - 6 = pin number
//[sensorPin][2]: 0 = photosensor, 1 = humiditysensor 2 = temperaturesensor
//[sensorPin][3]: 0 = not measured, 1 = reported only by front-end, 2 = reported also by backend, 3 = also alarms sent to backend
//[sensorPin][4]: 0 = beeper off, 1 = beeper on
//[sensorPin][5]: 0 = interval_0, 1 = interval_1, 2 = interval_2

static byte sensorConfig[6][6] {
  {1, 0, 2, 3, 1, 1},
  {2, 1, 2, 3, 1, 1},
  {1, 2, 1, 3, 1, 1},
  {2, 3, 1, 3, 1, 1},
  {1, 4, 0, 3, 1, 1},
  {2, 5, 0, 3, 1, 1}
};
//_____________________________________________________________
int notAllowedToEdit[6] = {0, 0, 0, 0, 0, 0}; // 1 = not allowed to edit

int sensorPin;

// For measurements statistics
unsigned long measurementTimeMs = 0;

// MUUTA:
//Pot01: Flowers: PUT THESE ON FLASH_MEMORY OR TO WEB SITE
byte flowers_GroundProfile01[8][3] = {
  {120, 110, 105},    // alarm_H photocell
  { 95, 90, 80},      // alarm_L photocell
  {130, 120, 110},   // alarm_H humidity
  { 90, 80, 70},      // alarm_L humidity
  {125, 115, 105},    // alarm_H temperature
  { 95, 85, 75},    // alarm_L temperature
  {0, 0, 0},        // No limits.
  {0, 0, 0}         // No limits.
};

byte roots_GroundProfile01[8][3] = {
  {115, 110, 105},   // alarm_H photocell
  { 95, 90, 85},     // alarm_L photocell
  {130, 120, 110},   // alarm_H humidity
  { 90, 80, 70},    // alarm_L humidity
  {115, 110, 105},   // alarm_H temperature
  { 95, 90, 85},    // alarm_L temperature
  {0, 0, 0},        // No limits.
  {0, 0, 0}         // No limits.
};

byte potStatFields = 4; //History: pot-Id, sensorX alarm, sensorY alarm, sensorZ alarm
//Current: pot_Id, sensorX alarms, sensorY alarms, sensorZ alarms
static int potAlarms [6][4] = { // 2 pots, max 3 sensors in each
  {1, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
  {1, 0, 0, 0}, //LOW LIMIT RED-ALARMS
  {1, 0, 0, 0}, //LOW LIMIT YELLOW-ALARMS
  {2, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
  {2, 0, 0, 0}, //LOW LIMIT RED-ALARMS
  {2, 0, 0, 0}  //LOW LIMIT YELLOW-ALARMS
};

//History: pot-Id, sensorX alarms, sensorY alarms, sensorZ alarms
static int potAlarmTotals [6][4] = { // 2 pots, max 3 sensors in each
  {1, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
  {1, 0, 0, 0}, //LOW LIMIT RED-ALARMS
  {1, 0, 0, 0}, //YELLOW-ALARMS
  {2, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
  {2, 0, 0, 0}, //LOW LIMIT RED-ALARMS
  {2, 0, 0, 0}   //YELLOW-ALARMS
};
char alarm;
// MUUTA:
// give greenhouse photocellPins a name: analog INPUTs
int photocellPinA0 = 4;    // the Pot1-sensor and 10Kohm pulldown are connected to A0
int photocellPinA1 = 5;    // the Pot2-sensor and 10Kohm pulldown are connected to A1
int humidityPinA2 = 2;     // the Pot1-sensor is connected to A2
int humidityPinA3 = 3;     // the Pot2-the sensor is connected to A3
int temperaturePinA4 = 0;  // the Pot1-the sensor is connected to A4
int temperaturePinA5 = 1;  // the Pot2-the sensor is connected to A5

// give greenhouse's photocell-readings a name:
int photocellReadA0 = 0;   // the analog  (A0) reading from the photocell (0 - 1023)
int photocellReadA1 = 0;   // the analog  (A1) reading from the photocell (0 - 1023)
int humidityReadA2 = 0;    // the analog  (A2) reading from the humidity sensor (0 - 1023)
int humidityReadA3 = 0;    // the analog  (A3) reading from the humidity sensor (0 - 1023)
int temperatureReadA4 = 0;   // the analog  (A4) reading from the temperature sensor (0 - 1023)
int temperatureReadA5 = 0;   // the analog  (A5) reading from the temperature sensor (0 - 1023)

int samples = 0;
int sampleCycleinterval = 50; // Delay in ms in main sensor loop

int maxsensorSettingsNumber = 3; // Max 3 sensors per pot. (for histograms)
byte sampleCount = 5; // Samples read for reference
int calibrInterval = 30; // ms

byte maxReCalibrationNeed = 2; // 2 sensor have 2 out of 6 invalid samples percycle
byte maxReCalibrationNeedperSensor = 2; // 2 invalid samples per sensor per cycle
byte reCalibrationNeed = 100;

int currentpotID;
int currentSensorPinID;
//int pot_ID = 1;

// sensor reference levels
int averXrefA0 = 0;
int averXrefA1 = 0;
int averYrefA2 = 0;
int averYrefA3 = 0;
int averZrefA4 = 0;
int averZrefA5 = 0;

// sensor average-readings
int averXreadA0 = 0;
int averXreadA1 = 0;
int averYreadA2 = 0;
int averYreadA3 = 0;
int averZreadA4 = 0;
int averZreadA5 = 0;

byte reCalibrationNeedperSensorA0 = 0;
byte reCalibrationNeedperSensorA1 = 0;
byte reCalibrationNeedperSensorA2 = 0;
byte reCalibrationNeedperSensorA3 = 0;
byte reCalibrationNeedperSensorA4 = 0;
byte reCalibrationNeedperSensorA5 = 0;


/*************************** CC3000 Pins ***********************************/

#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
#define ADAFRUIT_CC3000_VBAT  5  // VBAT & CS can be any digital pins.
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "CSGuestNet_optout"  // can't be longer than 32 characters!
#define WLAN_PASS       "4AGgVB-8tL9A"
#define WLAN_SECURITY   WLAN_SEC_WPA2  // Can be: WLAN_SEC_UNSEC, WLAN_SEC_WEP,
//         WLAN_SEC_WPA or WLAN_SEC_WPA2
Adafruit_CC3000_Client client;
/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "thy01"
#define AIO_KEY         "10fab204b83b4c63823e44c1b04014fb"

/************ Global State (you don't need to change this!) ******************/

// Setup the main CC3000 class, just like a normal CC3000 sketch.
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT);

// Store the MQTT server, username, and password in flash memory.
// This is required for using the Adafruit MQTT library.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the CC3000 MQTT class by passing in the CC3000 class and MQTT server and login details.
Adafruit_MQTT_CC3000 mqtt(&cc3000, MQTT_SERVER, AIO_SERVERPORT, MQTT_USERNAME, MQTT_PASSWORD);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }

// CC3000connect is a helper function that sets up the CC3000 and connects to
// the WiFi network. See the cc3000helper.cpp tab above for the source!
////#define STATICIP
//
//#define halt(s) { Serial.println(F( s )); while(1);  }
//
//uint16_t checkFirmwareVersion(void);
//bool displayConnectionDetails(void);
//
//extern Adafruit_CC3000 cc3000;
//boolean CC3000connect(const char* wlan_ssid, const char* wlan_pass, uint8_t wlan_security);

/****************************** Feeds ***************************************/

// Setup a feed called 'lightsensorA0' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char LIGHT_A0[] PROGMEM = AIO_USERNAME "/feeds/LIGHT_A0";
Adafruit_MQTT_Publish lightsensorA0 = Adafruit_MQTT_Publish(&mqtt, LIGHT_A0);

// Setup a feed called 'lightsensorA1' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char LIGHT_A1[] PROGMEM = AIO_USERNAME "/feeds/LIGHT_A1";
Adafruit_MQTT_Publish lightsensorA1 = Adafruit_MQTT_Publish(&mqtt, LIGHT_A1);

// Setup a feed called 'humiditysensorA2' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char HUMID_A2[] PROGMEM = AIO_USERNAME "/feeds/HUMID_A2";
Adafruit_MQTT_Publish humiditysensorA2 = Adafruit_MQTT_Publish(&mqtt, HUMID_A2);

// Setup a feed called 'humiditysensorA3' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char HUMID_A3[] PROGMEM = AIO_USERNAME "/feeds/HUMID_A3";
Adafruit_MQTT_Publish humiditysensorA3 = Adafruit_MQTT_Publish(&mqtt, HUMID_A3);


// Setup a feed called 'temperaturesensorA4' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char TEMPE_A4[] PROGMEM = AIO_USERNAME "/feeds/TEMPE_A4";
Adafruit_MQTT_Publish temperaturesensorA4 = Adafruit_MQTT_Publish(&mqtt, TEMPE_A4);

// Setup a feed called 'temperaturesensorA5' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char TEMPE_A5[] PROGMEM = AIO_USERNAME "/feeds/TEMPE_A5";
Adafruit_MQTT_Publish temperaturesensorA5 = Adafruit_MQTT_Publish(&mqtt, TEMPE_A5);


// Alarm-feeds

// Setup a feed called 'lightsensorA0alarm' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char LIGHT_A0_alarm[] PROGMEM = AIO_USERNAME "/feeds/LIGHT_A0_alarm";
Adafruit_MQTT_Publish lightsensorA0alarm = Adafruit_MQTT_Publish(&mqtt, LIGHT_A0_alarm);

// Setup a feed called 'lightsensorA1alarm' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char LIGHT_A1_alarm[] PROGMEM = AIO_USERNAME "/feeds/LIGHT_A1_alarm";
Adafruit_MQTT_Publish lightsensorA1alarm = Adafruit_MQTT_Publish(&mqtt, LIGHT_A1_alarm);


// Setup a feed called 'humiditysensorA2alarm' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char HUMID_A2_alarm[] PROGMEM = AIO_USERNAME "/feeds/HUMID_A2_alarm";
Adafruit_MQTT_Publish humiditysensorA2alarm = Adafruit_MQTT_Publish(&mqtt, HUMID_A2_alarm);

// Setup a feed called 'humiditysensorA3alarm' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char HUMID_A3_alarm[] PROGMEM = AIO_USERNAME "/feeds/HUMID_A3_alarm";
Adafruit_MQTT_Publish humiditysensorA3alarm = Adafruit_MQTT_Publish(&mqtt, HUMID_A3_alarm);

// Setup a feed called 'temperaturesensorA4alarm' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char TEMPE_A4_alarm[] PROGMEM = AIO_USERNAME "/feeds/TEMPE_A4_alarm";
Adafruit_MQTT_Publish temperaturesensorA4alarm = Adafruit_MQTT_Publish(&mqtt, TEMPE_A4_alarm);

// Setup a feed called 'temperaturesensorA5alarm' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char TEMPE_A5_alarm[] PROGMEM = AIO_USERNAME "/feeds/TEMPE_A5_alarm";
Adafruit_MQTT_Publish temperaturesensorA5alarm = Adafruit_MQTT_Publish(&mqtt, TEMPE_A5_alarm);

// TOTALS FOR HISTOGRAMS FEEDS

//Measurement totals-feeds:
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char A0_LIGHT_CLASS_TOTALS[] PROGMEM = AIO_USERNAME "/feeds/A0_LIGHT_CLASS_TOTALS";
Adafruit_MQTT_Publish A0lightClassTot = Adafruit_MQTT_Publish(&mqtt, A0_LIGHT_CLASS_TOTALS);

////Measurement totals-feeds:
//// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//const char A1_LIGHT_CLASS_TOTALS[] PROGMEM = AIO_USERNAME "/feeds/A1_LIGHT_CLASS_TOTALS";
//Adafruit_MQTT_Publish A1lightClassTot = Adafruit_MQTT_Publish(&mqtt, A1_LIGHT_CLASS_TOTALS);

//Measurement totals-feeds:
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char A2_HUMID_CLASS_TOTALS[] PROGMEM = AIO_USERNAME "/feeds/A2_HUMID_CLASS_TOTALS";
Adafruit_MQTT_Publish A2humidClassTot = Adafruit_MQTT_Publish(&mqtt, A2_HUMID_CLASS_TOTALS);

////Measurement totals-feeds:
//// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//const char A3_HUMID_CLASS_TOTALS[] PROGMEM = AIO_USERNAME "/feeds/A3_HUMID_CLASS_TOTALS";
//Adafruit_MQTT_Publish A3humidClassTot = Adafruit_MQTT_Publish(&mqtt, A3_HUMID_CLASS_TOTALS);

//Measurement totals-feeds:
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char A4_TEMPE_CLASS_TOTALS[] PROGMEM = AIO_USERNAME "/feeds/A4_TEMPE_CLASS_TOTALS";
Adafruit_MQTT_Publish A4tempClassTot = Adafruit_MQTT_Publish(&mqtt, A4_TEMPE_CLASS_TOTALS);

////Measurement totals-feeds:
//// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
//const char A5_TEMPE_CLASS_TOTALS[] PROGMEM = AIO_USERNAME "/feeds/A5_TEMPE_CLASS_TOTALS";
//Adafruit_MQTT_Publish A5tempClassTot = Adafruit_MQTT_Publish(&mqtt, A5_TEMPE_CLASS_TOTALS);


// END OF FEEDS DEFINITION *****************************************************

// For NTP:
uint32_t ip;
const unsigned long
connectTimeout  = 15L * 1000L, // Max time to wait for server connection
responseTimeout = 15L * 1000L; // Max time to wait for data from server
int
countdown       = 0;  // loop() iterations until next time server query
unsigned long
lastPolledTime  = 0L, // Last value retrieved from time server
sketchTime      = 0L; // CPU milliseconds since last server query

long startTime;
//********************* SETUP() *******************************
void setup() {
  //start serial protocol
  Serial.begin(115200);
  while (!Serial) {
    // wait for serial port to connect. Needed for native USB port only
  }
  // Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
  // KEYPAD EVENT LISTENER
  
  keypad.addEventListener(keypadEvent); //add an event listener for this keypad

  //    ************* LED TESTS **************
  digitalWrite(led2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second
  digitalWrite(led2, LOW);    // turn the LED off by making the voltage LOW
  delay(50);
  digitalWrite(led3, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second
  digitalWrite(led3, LOW);    // turn the LED off by making the voltage LOW
  delay(50);
  digitalWrite(led4, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second
  digitalWrite(led4, LOW);    // turn the LED off by making the voltage LOW
  delay(50);

  // GREEN HOUSE MAIN MENU
  Serial.println();
  Serial.println(F("  GREENHOUSE MAIN MENU"));
  Serial.println(F("  SELECT FUNCTION KEY, PLEASE !"));
  Serial.println();
  lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
  lcd.print(F("MAIN MENU"));
  lcd.setCursor(0, 1);
  lcd.print(F("SELECT FUNCTION"));
  // Syncronous:
  eventKey =  keypad.waitForKey();

  //char keypadEvent = keypad.getKey();

  // CC3000 WiFi STARTS:
  Serial.println(F("  CC3000!\n"));
  Serial.print("  Free RAM: "); Serial.println(getFreeRam(), DEC);

  displayDriverMode();

  Serial.println(F("  Initialising the CC3000 ..."));
  if (!cc3000.begin()) {
    Serial.println(F("  Unable to initialise the CC3000! Check your wiring?"));
    for (;;);
  }
  //
  //  uint16_t firmware = checkFirmwareVersion();
  //  if (firmware < 0x113) {
  //    Serial.println(F("Wrong firmware version!"));
  //    for (;;);
  //  }

  displayMACAddress();

  //  Serial.println(F("  Deleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("  Failed!"));
    while (1);
  }
  // Optional SSID scan
  listSSIDResults();

  /* Attempt to connect to an access point */
  char *ssid = WLAN_SSID;             /* Max 32 chars */
  Serial.print(F("  Attempting to connect to ")); Serial.println(WLAN_SSID);

  /* NOTE: Secure connections are not available in 'Tiny' mode! */
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("  Failed!"));
    while (1);
  }

  Serial.println(F("  Connected!"));

  /* Wait for DHCP to complete */
  Serial.println(F("  Request DHCP"));
  while (!cc3000.checkDHCP()) {
    delay(100); // ToDo: Insert a DHCP timeout!
  }

  /* Display the IP address DNS, Gateway, etc. */
  while (!displayConnectionDetails()) {
    delay(1000);
  }

  //********************
  // Connect to AIO
  int8_t ret;
  Serial.print("  Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
    Serial.println(mqtt.connectErrorString(ret));
    if (ret < 0) {
      //CC3000connect(WLAN_SSID, WLAN_PASS, WLAN_SECURITY);  // y0w, lets connect to wifi again
      /* Attempt to connect to an access point */
      char *ssid = WLAN_SSID;             /* Max 32 chars */
      Serial.print(F("\n  Attempting to connect to ")); Serial.println(WLAN_SSID);

      /* NOTE: Secure connections are not available in 'Tiny' mode! */
      if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
        Serial.println(F("  Failed!"));
        while (1);
      }

      Serial.println(F("  Connected!"));

      /* Wait for DHCP to complete */
      Serial.println(F("  Request DHCP"));
      while (!cc3000.checkDHCP()) {
        delay(100); // ToDo: Insert a DHCP timeout!
      }

      /* Display the IP address DNS, Gateway, etc. */
      while (!displayConnectionDetails()) {
        delay(1000);
      }
    }
    Serial.println("  Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
  }
  Serial.println("  MQTT Connected!");
  //  mqtt.subscribe(&onoffbuttonLight);
  //  mqtt.subscribe(&onoffbuttonHumid);
  //  mqtt.subscribe(&onoffbuttonTemp);

  //********************

  pinMode(pin5, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led2, OUTPUT);
}

//================================================================
void loop() {

  unsigned long secsSince1970; // Current UNIX time

  //  Serial.println();
  //  Serial.println(F("  GREENHOUSE MAIN MENU"));
  //  Serial.println(F("  SELECT FUNCTION KEY, PLEASE !"));
  //  Serial.println();
  //  lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
  //  lcd.print(F("MAIN MENU"));
  //  lcd.setCursor(0, 1);
  //  lcd.print(F("SELECT FUNCTION"));
  //
  //  char key =  keypad.waitForKey();
  //
  //  //char key = keypad.getKey();


  //initialize: CLEAR SENSOR INPUT STATISTIC ********
  pot01_StatisticX.clear();
  pot02_StatisticX.clear();
  pot01_StatisticY.clear();
  pot02_StatisticY.clear();
  pot01_StatisticZ.clear();
  pot02_StatisticZ.clear();

  reCalibrationNeedperSensorA0 = 0;
  reCalibrationNeedperSensorA1 = 0;
  reCalibrationNeedperSensorA2 = 0;
  reCalibrationNeedperSensorA3 = 0;
  reCalibrationNeedperSensorA4 = 0;
  reCalibrationNeedperSensorA5 = 0;

  int measurementCycleCounter;
  for (measurementCycleCounter = 0; measurementCycleCounter < 12; measurementCycleCounter++) {

    // To reduce load on NTP servers, time is polled once per roughly 24 hour period.
    // Otherwise use millis() to estimate time since last query.

    if (countdown == 0) {           // Time's up?
      unsigned long t  = getTime(); // Query time server
      if (t) {                      // Success?
        lastPolledTime = t;         // Save time
        sketchTime     = millis();  // Save sketch time of last valid time query
        countdown      = 24 * 60 * 4 - 1; // Reset counter: 24 hours * 15-second intervals
      }
    } else {
      countdown--;                  // Don't poll; use math to figure current time
    }

    secsSince1970 = lastPolledTime + (millis() - sketchTime) / 1000;
    printTimeStamp(secsSince1970);
    Serial.println();

    // Initialise alarms:
    // sensor X = photocell
    potAlarms [0][1]  = 0; // in pot1
    potAlarms [1][1]  = 0;
    potAlarms [2][1]  = 0;
    potAlarms [3][1]  = 0; // in pot2
    potAlarms [4][1]  = 0;
    potAlarms [5][1]  = 0;

    //  sensor Y = humidity
    potAlarms [0][2]  = 0; // in pot1
    potAlarms [1][2]  = 0;
    potAlarms [2][2]  = 0;
    potAlarms [3][2]  = 0; // in pot2
    potAlarms [4][2]  = 0;
    potAlarms [5][2]  = 0;

    // sensor Z = temperature
    potAlarms [0][3]  = 0; // in pot1
    potAlarms [1][3]  = 0;
    potAlarms [2][3]  = 0;
    potAlarms [3][3]  = 0; // in pot2
    potAlarms [4][3]  = 0;
    potAlarms [5][3]  = 0;

    // MUUTA:
    if (reCalibrationNeed >= maxReCalibrationNeed) {
      reCalibrationNeed = 0;
      Serial.println("");
      Serial.println(F("  Initializing and reading new sensor reference levels !"));
      Serial.println("");
      //delay (delay3);

      //initialize: CLEAR SENSOR INPUT STATISTIC ********
      pot01_StatisticX.clear();
      pot02_StatisticX.clear();
      pot01_StatisticY.clear();
      pot02_StatisticY.clear();
      pot01_StatisticZ.clear();
      pot02_StatisticZ.clear();

      reCalibrationNeedperSensorA0 = 0;
      reCalibrationNeedperSensorA1 = 0;
      reCalibrationNeedperSensorA2 = 0;
      reCalibrationNeedperSensorA3 = 0;
      reCalibrationNeedperSensorA4 = 0;
      reCalibrationNeedperSensorA5 = 0;

      for (int i = 0; i < sampleCount; i++) // READ REFERENCE LEVELS
      {
        photocellReadA0 = analogRead(photocellPinA0);
        //        Serial.print(" photocell A0 reading = ");
        //        Serial.println(photocellReadA0);
        photocellReadA1 = analogRead(photocellPinA1);
        //        Serial.print(" photocell A1 reading = ");
        //        Serial.println(photocellReadA1);
        humidityReadA2 = analogRead(humidityPinA2);
        //        Serial.print(" humidity A2 reading = ");
        //        Serial.println(humidityReadA2);
        humidityReadA3 = analogRead(humidityPinA3);
        //        Serial.print(" humidityRead A3 reading = ");
        //        Serial.println(humidityReadA3);
        temperatureReadA4 = analogRead(temperaturePinA4);
        //        Serial.print(" temperature A4 reading = ");
        //        Serial.println(temperatureReadA4);
        temperatureReadA5 = analogRead(temperaturePinA5);
        //        Serial.print(" temperature A5 reading = ");
        //        Serial.println(temperatureReadA5);
        delay(delay6);

        // Add single sensor reading pot_Statistics
        pot01_StatisticX.add(photocellReadA0);
        pot02_StatisticX.add(photocellReadA1);
        pot01_StatisticY.add(humidityReadA2);
        pot02_StatisticY.add(humidityReadA3);
        pot01_StatisticZ.add(temperatureReadA4);
        pot02_StatisticZ.add(temperatureReadA5);
      } //End of sampling

      //Store reference averages per sensor
      averXrefA0 = pot01_StatisticX.average();
      averXrefA1 = pot02_StatisticX.average();
      averYrefA2 = pot01_StatisticY.average();
      averYrefA3 = pot02_StatisticY.average();
      averZrefA4 = pot01_StatisticZ.average();
      averZrefA5 = pot02_StatisticZ.average();

      Serial.println();
      Serial.println("  New reference levels for the sensors (average)");
      Serial.println();
      Serial.print(F("  averXrefA0 = "));
      Serial.println(averXrefA0);
      Serial.print(F("  averXrefA1 = "));
      Serial.println(averXrefA1);
      Serial.print(F("  averYrefA2 = "));
      Serial.println(averYrefA2);
      Serial.print(F("  averYrefA3 = "));
      Serial.println(averYrefA3);
      Serial.print(F("  averZrefA4 = "));
      Serial.println(averZrefA4);
      Serial.print(F("  averZrefA5 = "));
      Serial.println(averZrefA5);
      Serial.println();
    }

    //***************************************************
    //********FIRST SENSOR READING STARTS ***************

    currentpotID = measurementCycleCounter % 2;
    //    Serial.println();
    //    Serial.print(F("  currentpotID = "));
    //    Serial.println(currentpotID);

    currentSensorPinID = measurementCycleCounter % 6;
    //    Serial.println();
    //    Serial.print(F("  currentSensorPinID = "));
    //    Serial.println(currentSensorPinID);
    //    Serial.println();

    // MQTT STARTS HERE:

    // Make sure to reset watchdog every loop iteration!
    Watchdog.reset();

    // Ensure the connection to the MQTT server is alive (this will make the first
    // connection and automatically reconnect when disconnected).  See the MQTT_connect
    // function definition further below.
    // Stop if already connected.
    if (!mqtt.connected()) {
      int8_t ret;
      Serial.print("  Connecting to MQTT... ");

      while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
        Serial.println(mqtt.connectErrorString(ret));

        if (ret < 0) {
          //CC3000connect(WLAN_SSID, WLAN_PASS, WLAN_SECURITY);  // y0w, lets connect to wifi again
          /* Attempt to connect to an access point */
          char *ssid = WLAN_SSID;             /* Max 32 chars */
          Serial.print(F("\n  Attempting to connect to ")); Serial.println(WLAN_SSID);

          /* NOTE: Secure connections are not available in 'Tiny' mode! */
          if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
            Serial.println(F("  Failed!"));
            while (1);
          }

          Serial.println(F("  Connected!"));

          /* Wait for DHCP to complete */
          Serial.println(F("  Request DHCP"));
          while (!cc3000.checkDHCP()) {
            delay(100); // ToDo: Insert a DHCP timeout!
          }

          /* Display the IP address DNS, Gateway, etc. */
          while (!displayConnectionDetails()) {
            delay(1000);
          }
        }
        Serial.println("  Retrying MQTT connection in 5 seconds...");
        mqtt.disconnect();
        delay(5000);  // wait 5 seconds
      }
      Serial.println("  MQTT Connected!");
    }
    //MQTT_connect();
    //    mqtt.subscribe(&onoffbuttonLight);
    //    mqtt.subscribe(&onoffbuttonHumid);
    //    mqtt.subscribe(&onoffbuttonTemp);

    // this is our 'wait for incoming subscription packets' busy subloop
    //    Adafruit_MQTT_Subscribe *subscription;
    //    while ((subscription = mqtt.readSubscription(1000))) {
    //      if (subscription == &onoffbuttonLight_A1) {
    //        Serial.print(F("Got: "));
    //        Serial.println((char *)onoffbuttonLight_A1.lastread);
    //      }
    //      if (subscription == &onoffbuttonHumid_A3) {
    //        Serial.print(F("Got: "));
    //        Serial.println((char *)onoffbuttonHumid_A3.lastread);
    //      }
    //      if (subscription == &onoffbuttonTemp_A5) {
    //        Serial.print(F("Got: "));
    //        Serial.println((char *)onoffbuttonTemp_A5.lastread);
    //      }
    //    }

    unsigned long measurementTimeMs = millis(); // Measurements start time (ms)

    switch (currentSensorPinID) {

      case 0: //photocellPinA0 = X-sensor in POT1
        if (sensorConfig[photocellPinA0][3] > 0)
        {
          // Read the sensor average measurement
          float calibConstantA0 = 1.5;
          photocellReadA0 = calibConstantA0 * read_sensorValue(photocellPinA0);
          pot01_StatisticX.add(photocellReadA0);
          Serial.println();
          Serial.print(F("  PHOTOCELL A0 IN POT1, NUMBER OF READINGS:  "));
          Serial.println(pot01_StatisticX.count());
          Serial.print(F("  PHOTOCELL A0 IN POT1, VALUE OF READING:    "));
          Serial.println((1023 - photocellReadA0));
          classIndex = map(1023 - photocellReadA0, 0 , 1023, 1, classCount);
          Serial.print(F("  PHOTOCELL A0 IN POT1, CLASS OF READ VALUE: "));
          Serial.println(classIndex);
          Serial.println();
          delay(delay6);

          pot_ID = currentpotID + 1;
          store_sensorValue(secsSince1970, pot_ID, measurementTimeMs, photocellPinA0, photocellReadA0, classCount, pot01_Measurements, pot01_MeasurementClassTotals, pot02_Measurements, pot02_MeasurementClassTotals);

          averXreadA0 = pot01_StatisticX.average();
          //photocellPinA0 = X-sensor in POT1
          reCalibrationNeedperSensorA0 = test_sensorValue(secsSince1970, pot_ID, measurementTimeMs, photocellPinA0, photocellReadA0, reCalibrationNeedperSensorA0, averXreadA0, averXrefA0, potAlarms, potAlarmTotals, pot01_StatisticX, flowers_GroundProfile01);

          if (sensorConfig[photocellPinA0][3] > 1) {
            // Publishing sensor measurement via MQTT
            Serial.print(F("  Sending lightsensor val "));
            Serial.print(classIndex);
            Serial.print("...");
            int32_t cIndex = classIndex;
            if (! lightsensorA0.publish(cIndex))
            {
              Serial.println(F("  Failed"));
            } else {
              Serial.println(F("  OK!"));
            }
            delay(delay7);
          }

          if (sensorConfig[photocellPinA0][3] > 2) {
            // Publishing potAlarmTotals via MQTT
            //{1, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
            //{1, 0, 0, 0}, //LOW LIMIT RED-ALARMS
            //{1, 0, 0, 0}, //YELLOW-ALARMS

            int sensorType = photocellPinA0 + 1;
            if (sensorType > 3) {
              sensorType = sensorType - 3;
            }

            Serial.println(F("  Sending sensorA0 alarms "));
            for (int iAlarm = 0; iAlarm < 3; iAlarm++) {
              Serial.print("  total A0 alarms...");
              Serial.print(potAlarmTotals[iAlarm][sensorType]);
              int32_t potAlarmTotalsA0 = potAlarmTotals[iAlarm][sensorType];
              if (! lightsensorA0alarm.publish(potAlarmTotalsA0))
              {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
            for (int iAlarm = 0; iAlarm < 3; iAlarm++)
            {
              Serial.print("  sensorA0 alarms...");
              Serial.print(potAlarms[iAlarm][sensorType]);
              int32_t potAlarmsA0 = potAlarms[iAlarm][sensorType];
              if (! lightsensorA0alarm.publish(potAlarmsA0))
              {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
          }

          //MUUTA:
          if (reCalibrationNeedperSensorA0 >= maxReCalibrationNeedperSensor)
          {
            reCalibrationNeed = reCalibrationNeed + 1;

            // go to calibration
            Serial.println();
            Serial.println();
            Serial.println(F(" Plant needs your care:  "));
            Serial.println();
            Serial.print(F(" See alarm-report of POT = "));
            Serial.print(pot_ID);
            Serial.println(F(" Sensor alarming = A0"));
            Serial.println(F(" (Adjust lightness-conditions, please.) "));
            delay(delay6);
          }
          // MUUTA: averXrefA0 = oikeaksi referenssitasoksi (sijoita aikaisemmin käyttäjän syötteestä)
          int measureA0DevFromRef = averXrefA0 - photocellReadA0;
          break;
        }
      case 1:
        if (sensorConfig[photocellPinA0][3] > 0)
        {
          float calibConstantA1 = 1.1;
          photocellReadA1 = calibConstantA1 * read_sensorValue(photocellPinA1);
          pot02_StatisticX.add(photocellReadA1);
          Serial.println();
          Serial.print(F("  PHOTOCELL A1 IN POT2, NUMBER OF READINGS:  "));
          Serial.println(pot02_StatisticX.count());
          Serial.print(F("  PHOTOCELL A1 IN POT2, VALUE OF READING:    "));
          Serial.println((photocellReadA1));
          classIndex = map(photocellReadA1, 0 , 1023, 1, classCount);
          Serial.print(F("  PHOTOCELL A1 IN POT2, CLASS OF READ VALUE: "));
          Serial.println(classIndex);
          Serial.println();
          delay(delay6);
          pot_ID = currentpotID + 1;
          store_sensorValue(secsSince1970, pot_ID, measurementTimeMs, photocellPinA1, photocellReadA1, classCount, pot01_Measurements, pot01_MeasurementClassTotals, pot02_Measurements, pot02_MeasurementClassTotals);

          averXreadA1 = pot02_StatisticX.average();
          //photocellPinA1 = X-sensor in POT2
          reCalibrationNeedperSensorA1 = test_sensorValue(secsSince1970, pot_ID, measurementTimeMs, photocellPinA1, photocellReadA1, reCalibrationNeedperSensorA1, averXreadA1, averXrefA1, potAlarms, potAlarmTotals, pot02_StatisticX, roots_GroundProfile01);

          if (sensorConfig[photocellPinA1][3] > 1) {
            // Publishing sensor measurement via MQTT
            Serial.print(F("  Sending lightsensor val "));
            Serial.print(classIndex);
            Serial.print("...");
            int32_t cIndex = classIndex;
            if (! lightsensorA1.publish(cIndex))
            {
              Serial.println(F("  Failed"));
            } else {
              Serial.println(F("  OK!"));
            }
            delay(delay7);
          }

          if (sensorConfig[photocellPinA1][3] > 2) {
            // Publishing potAlarmTotals via MQTT
            //{2, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
            //{2, 0, 0, 0}, //LOW LIMIT RED-ALARMS
            //{2, 0, 0, 0}, //YELLOW-ALARMS

            sensorType = photocellPinA1 + 1;
            if (sensorType > 3) {
              sensorType = sensorType - 3;
            }

            Serial.println(F("  Sending sensorA1 alarms "));
            for (int iAlarm = 3; iAlarm < 6; iAlarm++)
            {
              Serial.print("  total A1 alarms...");
              Serial.print(potAlarmTotals[iAlarm][sensorType]);
              int32_t potAlarmTotalsA1 = potAlarmTotals[iAlarm][sensorType];
              if (! lightsensorA1alarm.publish(potAlarmTotalsA1))
              {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
            for (int iAlarm = 3; iAlarm < 6; iAlarm++)
            {
              Serial.print("  sensorA1 alarms...");
              Serial.print(potAlarms[iAlarm][sensorType]);
              int32_t potAlarmsA1 = potAlarms[iAlarm][sensorType];
              if (! lightsensorA1alarm.publish(potAlarmsA1))
              {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
          }

          // MUUTA:
          if (reCalibrationNeedperSensorA1 >= maxReCalibrationNeedperSensor)
          {
            reCalibrationNeed = reCalibrationNeed + 1;
            // go to calibration
            Serial.println();
            Serial.println();
            Serial.println(F("  Plant needs your care:  "));
            Serial.print(F("  See alarm-report of POT = "));
            Serial.print(pot_ID);
            Serial.println(F("  Sensor alarming = A1"));
            Serial.println(F("  (Adjust lightness-conditions, please.) "));
            delay(delay6);
          }
          int measureA1DevFromRef = averXrefA1 - photocellReadA1;
         
          break;
        }
      case 2: //humidityPinA2 = Y-sensor in POT1
        if (sensorConfig[humidityPinA2][3] > 0)
        {
          humidityReadA2 = read_sensorValue(humidityPinA2);
          pot01_StatisticY.add(humidityReadA2);
          Serial.println();
          Serial.print(F("  HUMIDITY A2 IN POT1, NUMBER OF READINGS:  "));
          Serial.println(pot01_StatisticY.count());
          Serial.print(F("  HUMIDITY A2 IN POT1, VALUE OF READING:    "));
          Serial.println(humidityReadA2);
          classIndex = map(humidityReadA2, 0 , 1023, 1, classCount);
          Serial.print(F("  HUMIDITY A2 IN POT1, CLASS OF READ VALUE: "));
          Serial.println(classIndex);
          Serial.println();
          delay(delay6);

          pot_ID = currentpotID + 1;
          store_sensorValue(secsSince1970, pot_ID, measurementTimeMs, humidityPinA2, humidityReadA2, classCount, pot01_Measurements, pot01_MeasurementClassTotals, pot02_Measurements, pot02_MeasurementClassTotals);

          averYreadA2 = pot01_StatisticY.average();
          reCalibrationNeedperSensorA2;
          //humidityPinA2 = Y-sensor in POT1
          reCalibrationNeedperSensorA2 = test_sensorValue(secsSince1970, pot_ID, measurementTimeMs, humidityPinA2, humidityReadA2, reCalibrationNeedperSensorA2, averYreadA2, averYrefA2, potAlarms, potAlarmTotals, pot01_StatisticY, flowers_GroundProfile01);

          if (sensorConfig[humidityPinA2][3] > 1)
          {
            Serial.print(F("  Sending humiditysensor val "));
            Serial.print(classIndex);
            Serial.print("...");
            int32_t cIndex = classIndex;
            if (! humiditysensorA2.publish(cIndex))
            {
              Serial.println(F("  Failed"));
            } else {
              Serial.println(F("  OK!"));
            }
            delay(delay7);
          }

          if (sensorConfig[humidityPinA2][3] > 2) {
            // Publishing potAlarmTotals via MQTT
            //{1, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
            //{1, 0, 0, 0}, //LOW LIMIT RED-ALARMS
            //{1, 0, 0, 0}, //YELLOW-ALARMS

            sensorType = humidityPinA2 + 1;
            if (sensorType > 3) {
              sensorType = sensorType - 3;
            }

            Serial.println(F("  Sending sensorA2 alarms "));
            for (int iAlarm = 0; iAlarm < 3; iAlarm++) {
              Serial.print("  total A2 alarms...");
              Serial.print(potAlarmTotals[iAlarm][sensorType]);
              int32_t potAlarmTotalsA2 = potAlarmTotals[iAlarm][sensorType];
              if (! humiditysensorA2alarm.publish(potAlarmTotalsA2))
              {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
            for (int iAlarm = 0; iAlarm < 3; iAlarm++)
            {
              Serial.print("  sensorA2 alarms...");
              Serial.print(potAlarms[iAlarm][sensorType]);
              int32_t potAlarmsA2 = potAlarmTotals[iAlarm][sensorType];
              if (! humiditysensorA2alarm.publish(potAlarmsA2))
              {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
          }
          // MUUTA:
          if (reCalibrationNeedperSensorA2 >= maxReCalibrationNeedperSensor)
          {
            reCalibrationNeed = reCalibrationNeed + 1;
            // go to calibration
            Serial.println();
            Serial.println();
            Serial.println(F("  Plant needs your care:  "));
            Serial.print(F("  See alarm-report of POT = "));
            Serial.print(pot_ID);
            Serial.println(F("  Sensor alarming = A2"));
            Serial.println(F("  (Adjust humidity-conditions, please.) "));
            delay(delay6);
          }
          int measureA2DevFromRef = averYrefA2 - humidityReadA2;
          
          break;
        }
      case 3: //humidityPinA3 = Y-sensor in POT2
        if (sensorConfig[humidityPinA3][3] > 0)
        {
          // *** RANDOM FUNCTION ****
          long simulValue = random(10, 11); // will be 10 to 11
          float calibConstantA3 = simulValue / 10; // simulated measurement
          humidityReadA3 = calibConstantA3 * read_sensorValue(humidityPinA3);
          pot02_StatisticY.add(humidityReadA3);
          Serial.println();
          Serial.print(F("  HUMIDITY A3 IN POT2, NUMBER OF READINGS:  "));
          Serial.println(pot02_StatisticY.count());
          Serial.print(F("  HUMIDITY A3 IN POT2, VALUE OF READING:    "));
          Serial.println(humidityReadA3);
          classIndex = map(humidityReadA3, 0 , 1023, 1, classCount);
          Serial.print(F("  HUMIDITY A3 IN POT2, CLASS OF READ VALUE: "));
          Serial.println(classIndex);
          Serial.println();
          delay(delay6);

          pot_ID = currentpotID + 1;
          store_sensorValue(secsSince1970, pot_ID, measurementTimeMs, humidityPinA3, humidityReadA3, classCount, pot01_Measurements, pot01_MeasurementClassTotals, pot02_Measurements, pot02_MeasurementClassTotals);

          averYreadA3 = pot02_StatisticY.average();
          reCalibrationNeedperSensorA3;
          //humidityPinA3 = Y-sensor in POT2
          reCalibrationNeedperSensorA3 = test_sensorValue(secsSince1970, pot_ID, measurementTimeMs, humidityPinA3, humidityReadA3, reCalibrationNeedperSensorA3, averYreadA3, averYrefA3, potAlarms, potAlarmTotals, pot02_StatisticY, roots_GroundProfile01);

          if (sensorConfig[humidityPinA3][3] > 1)
          {
            Serial.print(F("  Sending humiditysensor val "));
            Serial.print(classIndex);
            Serial.print("...");
            int32_t cIndex = classIndex;
            if (! humiditysensorA3.publish(cIndex)) {
              Serial.println(F("  Failed"));
            } else {
              Serial.println(F("  OK!"));
            }
            delay(delay7);
          }

          if (sensorConfig[humidityPinA3][3] > 2) {
            // Publishing potAlarmTotals via MQTT
            //{2, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
            //{2, 0, 0, 0}, //LOW LIMIT RED-ALARMS
            //{2, 0, 0, 0}, //YELLOW-ALARMS

            sensorType = humidityPinA3 + 1;
            if (sensorType > 3) {
              sensorType = sensorType - 3;
            }

            Serial.println(F("  Sending sensorA3 alarms "));
            for (int iAlarm = 3; iAlarm < 6; iAlarm++)
            {
              Serial.print("  total A3 alarms...");
              Serial.print(potAlarmTotals[iAlarm][sensorType]);
              int32_t potAlarmTotalsA3 = potAlarmTotals[iAlarm][sensorType];
              if (! humiditysensorA3alarm.publish(potAlarmTotalsA3))
              {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
            for (int iAlarm = 3; iAlarm < 6; iAlarm++) {
              Serial.print("  sensorA3 alarms...");
              Serial.print(potAlarms[iAlarm][sensorType]);
              int32_t potAlarmsA3 = potAlarms[iAlarm][sensorType];
              if (! humiditysensorA3alarm.publish(potAlarmsA3))
              {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
          }
          // MUUTA:
          if (reCalibrationNeedperSensorA3 >= maxReCalibrationNeedperSensor)
          {
            reCalibrationNeed = reCalibrationNeed + 1;
            // go to calibration
            Serial.println();
            Serial.println();
            Serial.println(F("  Plant needs your care:  "));
            Serial.print(F("  See alarm-report of POT = "));
            Serial.print(pot_ID);
            Serial.println(F("  Sensor alarming = A3"));
            Serial.println(F("  (Adjust humidity-conditions, please.) "));
            delay(delay6);
          }
          int measureA3DevFromRef = averYrefA3 - humidityReadA3;
          
          break;
        }
      case 4: //temperaturePinA4 = Z-sensor in POT1
        if (sensorConfig[temperaturePinA4][3] > 0)
        {
          //float calibConstantA4 = 0.4883;
          float temperatureReadA4 = ( read_sensorValue(temperaturePinA4) / 1024.0) * 5000;
          temperatureReadA4 = temperatureReadA4 / 10; // Celsiusta
          //float farh = (cel * 9) / 5 + 32;
          pot01_StatisticZ.add(temperatureReadA4);
          Serial.println();
          Serial.print(F("  TEMPERATURE SENSOR A4 IN POT1, NUMBER OF READINGS:  "));
          Serial.println(pot01_StatisticZ.count());
          Serial.print(F(" TEMPERATURE SENSOR A4 IN POT1, VALUE OF READING:     "));
          Serial.println((temperatureReadA4));
          classIndex = map(temperatureReadA4, 0 , 1023, 1, classCount);
          Serial.print(F("  TEMPERATURE SENSOR A4 IN POT1, CLASS OF READ VALUE: "));
          Serial.println(classIndex);
          Serial.println();
          delay(delay6);

          pot_ID = currentpotID + 1;
          store_sensorValue(secsSince1970, pot_ID, measurementTimeMs, temperaturePinA4, temperatureReadA4, classCount, pot01_Measurements, pot01_MeasurementClassTotals, pot02_Measurements, pot02_MeasurementClassTotals);

          averZreadA4 = pot01_StatisticZ.average();
          reCalibrationNeedperSensorA4;
          //temperaturePinA4 = Z-sensor in POT1
          reCalibrationNeedperSensorA4 = test_sensorValue(secsSince1970, pot_ID, measurementTimeMs, temperaturePinA4, temperatureReadA4, reCalibrationNeedperSensorA4, averZreadA4, averZrefA4, potAlarms, potAlarmTotals, pot01_StatisticZ, flowers_GroundProfile01);

          if (sensorConfig[temperaturePinA4][3] > 1)
          {
            Serial.print(F("  Sending temperaturesensor val "));
            Serial.print(classIndex);
            Serial.print("...");
            int32_t cIndex = classIndex;
            if (! temperaturesensorA4.publish(cIndex)) {
              Serial.println(F("Failed"));
            } else {
              Serial.println(F("OK!"));
            }
            delay(delay7);
          }

          if (sensorConfig[temperaturePinA4][3] > 2) {
            // Publishing potAlarmTotals via MQTT
            //{1, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
            //{1, 0, 0, 0}, //LOW LIMIT RED-ALARMS
            //{1, 0, 0, 0}, //YELLOW-ALARMS

            sensorType = temperaturePinA4 + 1;
            if (sensorType > 3) {
              sensorType = sensorType - 3;
            }

            Serial.println(F("  Sending sensorA4 alarms "));
            for (int iAlarm = 0; iAlarm < 3; iAlarm++)
            {
              Serial.print("  total A4 alarms...");
              Serial.print(potAlarmTotals[iAlarm][sensorType]);
              int32_t potAlarmTotalsA4 = potAlarmTotals[iAlarm][sensorType];
              if (! temperaturesensorA4alarm.publish(potAlarmTotalsA4))
              {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
            for (int iAlarm = 0; iAlarm < 3; iAlarm++)
            {
              Serial.print("  sensorA4 alarms...");
              Serial.print(potAlarms[iAlarm][sensorType]);
              int32_t potAlarmsA4 = potAlarms[iAlarm][sensorType];
              if (! temperaturesensorA4alarm.publish(potAlarmsA4)) {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
          }
          // MUUTA:
          if (reCalibrationNeedperSensorA4 >= maxReCalibrationNeedperSensor)
          {
            reCalibrationNeed = reCalibrationNeed + 1;
            // go to calibration
            Serial.println();
            Serial.println();
            Serial.println(F("  Plant needs your care:  "));
            Serial.print(F("  See alarm-report of POT = "));
            Serial.print(pot_ID);
            Serial.println(F("  Sensor alarming = A4"));
            Serial.println(F("  (Adjust temperature-conditions, please.) "));
            delay(delay6);
          }
          int measureA4DevFromRef = averZrefA4 - temperatureReadA4;
         
          break;
        }
      case 5: //temperaturePinA5 = Z-sensor in POT2
        if (sensorConfig[temperaturePinA5][3] > 0)
        {
          //float calibConstantA5 = 0.4883;

          // *** RANDOM FUNCTION ****
          long simulValue = random(10, 11); // will be 9 to 10
          float calibConstantA5 = simulValue / 10; // simulated measurement
          temperatureReadA5 = calibConstantA5 * read_sensorValue(temperaturePinA4);//simulated from A4
          float temperatureReadA5 = ( read_sensorValue(temperaturePinA5) / 1024.0) * 5000;
          temperatureReadA5 = temperatureReadA5 / 10; // Celsiusta
          //float farh = (cel * 9) / 5 + 32;
          pot02_StatisticZ.add(temperatureReadA5);
          Serial.println();
          Serial.print(F("  TEMPERATURE A5 IN POT2, NUMBER OF READINGS:  "));
          Serial.println(pot02_StatisticZ.count());
          Serial.print(F("  TEMPERATURE A5 IN POT2, VALUE OF READING:    "));
          Serial.println(temperatureReadA5);
          classIndex = map(temperatureReadA5, 0 , 1023, 1, classCount);
          Serial.print(F("  TEMPERATURE A5 IN POT2, CLASS OF READ VALUE: "));
          Serial.println(classIndex);
          Serial.println();
          delay(delay6);

          pot_ID = currentpotID + 1;
          store_sensorValue(secsSince1970, pot_ID, measurementTimeMs, temperaturePinA5, temperatureReadA5, classCount, pot01_Measurements, pot01_MeasurementClassTotals, pot02_Measurements, pot02_MeasurementClassTotals);

          averZreadA5 = pot02_StatisticZ.average();
          reCalibrationNeedperSensorA5;
          //temperaturePinA5 = Z-sensor in POT2
          reCalibrationNeedperSensorA5 = test_sensorValue(secsSince1970, pot_ID, measurementTimeMs, temperaturePinA5, temperatureReadA5, reCalibrationNeedperSensorA5, averZreadA5, averZrefA5, potAlarms, potAlarmTotals, pot02_StatisticZ, roots_GroundProfile01);

          if (sensorConfig[temperaturePinA5][3] > 1)
          {
            Serial.print(F("  Sending temperaturesensor val "));
            Serial.print(classIndex);
            Serial.print("...");
            int32_t cIndex = classIndex;
            if (! temperaturesensorA5.publish(cIndex))
            {
              Serial.println(F("Failed"));
            } else {
              Serial.println(F("OK!"));
            }
            delay(delay7);
          }

          if (sensorConfig[temperaturePinA5][3] > 2) {
            // Publishing potAlarmTotals via MQTT
            //{2, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
            //{2, 0, 0, 0}, //LOW LIMIT RED-ALARMS
            //{2, 0, 0, 0}, //YELLOW-ALARMS

            sensorType = temperaturePinA4 + 1;
            if (sensorType > 3) {
              sensorType = sensorType - 3;
            }

            Serial.println(F("  Sending sensorA5 alarms "));
            for (int iAlarm = 3; iAlarm < 6; iAlarm++) {
              Serial.print("  total A5 alarms...");
              Serial.print(potAlarmTotals[iAlarm][sensorType]);
              int32_t potAlarmTotals05 = potAlarmTotals[iAlarm][sensorType];
              if (! temperaturesensorA5alarm.publish(potAlarmTotals05))
              {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
            for (int iAlarm = 3; iAlarm < 6; iAlarm++)
            {
              Serial.print("  sensorA5 alarms...");
              Serial.print(potAlarms[iAlarm][sensorType]);
              int32_t potAlarms05 = potAlarmTotals[iAlarm][sensorType];
              if (! temperaturesensorA5alarm.publish(potAlarms05)) {
                Serial.println(F("...Failed"));
              } else {
                Serial.println(F("...OK!"));
              }
              delay(delay7);
            }
          }
          // MUUTA:
          if (reCalibrationNeedperSensorA5 >= maxReCalibrationNeedperSensor)
          {
            reCalibrationNeed = reCalibrationNeed + 1;
            // go to calibration
            Serial.println();
            Serial.println(F("  Plant needs your care:  "));
            Serial.print(F("  See alarm-report of POT = "));
            Serial.print(pot_ID);
            Serial.println(F("  Sensor alarming = A5"));
            Serial.println(F("  (Adjust temperature - conditions, please.) "));
            delay(delay6);
          }
          int measureA5DevFromRef = averZrefA5 - temperatureReadA5;
          
          break;
        } // END OF if
    } // END OF switch (currentSensorPinID)

    // ping the server to keep the mqtt connection alive
    if (! mqtt.ping()) {
      Serial.println(F("  MQTT Ping failed."));
    }
    delay(delay7);

  } // END OF for (measurementCycleCounter = 0; measurementCycleCounter < 30; measurementCycleCounter++)

  // HISTOGRAMS OF THE MEASUREMENT CYCLE
    pot_ID = 1;
    sensorStatistics(secsSince1970, pot_ID, pot01_MeasurementClassTotals, maxsensorSettingsNumber, potAlarmTotals, classCount, pot01_StatisticX, pot01_StatisticY, pot01_StatisticZ);
    pot_ID = 2;
    sensorStatistics(secsSince1970, pot_ID, pot02_MeasurementClassTotals, maxsensorSettingsNumber, potAlarmTotals, classCount, pot02_StatisticX, pot02_StatisticY, pot02_StatisticZ);

  pot_ID = 1; // New cycle starts wit this
  Serial.println();
  Serial.println(F(" *** NEW MEASUREMENT CYCLE STARTS HERE. ***"));
  Serial.println();
  delay(delay6);

}


int read_sensorValue(int sensorPin) {
  //******************** read_sensorValue **************************
  Statistic read_Statistic;
  read_Statistic.clear();
  int sensorRead;
  int startT = millis();
  int sample = 0;
  int samplesPerSensor = 5;
  int sampleInterval = 20; // ms
  //int delay2 = 2000; // TESTING

  while (sample <= samplesPerSensor) {
    sensorRead = analogRead(sensorPin); //Read sensor
    read_Statistic.add(sensorRead);
    sample = sample + 1;
    delay(sampleInterval);
  } //END OF while (sample <= samplesPerSensor)

  //Serial.print(F(" Measurement time per sensor = ")); // TESTING ***
  //Serial.println(((millis() - startT))); // TESTING ***
  //delay (delay2);  // TESTING ***

  return read_Statistic.average();

} //END OF read_sensorValue ***************************************

//******************** store_sensorValue ****************************
void store_sensorValue(unsigned long secsSince1970, byte pot_ID, int measurementTimeMs, int sensorPin, int sensorRead, int classCount, int pot01_Measurements[][20], int pot01_MeasurementClassTotals[][20], int pot02_Measurements[][20], int pot02_MeasurementClassTotals[][20] )
{
  //  //for sensor statistics the last measurements
  //static int pot01_Measurements[3][100] = {
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // photocell
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // humidity sensor
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} // temperature sensor
  //
  //  };
  //static int pot02_Measurements[3][100] = {
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // photocell
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // humidity sensor
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // temperature sensor
  //  };
  //
  //  //Cumulative measurement result of the pot (3 sensors measured)
  //  static int pot01_MeasurementClassTotals[3][100] = {
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Count of the sensorX measurements per class.
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Count of the sensorY measurements per class.
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // Count of the sensorZ measurements per class.
  //  };
  //  static int pot02_MeasurementClassTotals[3][100] = {
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Count of the sensorX measurements per class.
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // Count of the sensorY measurements per class.
  //    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}  // Count of the sensorZ measurements per class.
  //  };

  int classIndex = map(sensorRead, 0 , 1023, 1, classCount);
  if (pot_ID == 1) {
    switch (sensorPin) {
      case 0:
        pot01_Measurements [0][classIndex] = sensorRead;
        pot01_MeasurementClassTotals[0][classIndex] = pot01_MeasurementClassTotals[0][classIndex] + 1;
        break;
      case 2:
        pot01_Measurements [1][classIndex] = sensorRead;
        pot01_MeasurementClassTotals[1][classIndex] = pot01_MeasurementClassTotals[1][classIndex] + 1;
        break;
      case 4:
        pot01_Measurements [2][classIndex] = sensorRead;
        pot01_MeasurementClassTotals[2][classIndex] = pot01_MeasurementClassTotals[2][classIndex] + 1;
        break;
    }
    //for sensor statistics the last measurement
    Serial.println();
    printTimeStamp(secsSince1970);
    Serial.println("  pot01_Measurements: ");
    for (int i = 0; i < 3; i++) {
      Serial.print("  "); // Two spaces in the beginning of the row
      for (int j = 0; j < 20; j++) {
        Serial.print(pot01_Measurements[i][j]);
        Serial.print(", ");
      }
      Serial.println();
    }
    Serial.println();
    Serial.println("  pot01_MeasurementsClassTotals: ");
    for (int i = 0; i < 3; i++) {
      Serial.print("  "); // Two spaces in the beginning of the row
      for (int j = 0; j < 20; j++) {
        Serial.print(pot01_MeasurementClassTotals[i][j]);
        Serial.print(", ");
      }
      Serial.println();
    }
  }
  if (pot_ID == 2) {
    switch (sensorPin) {
      case 1:
        pot02_Measurements [0][classIndex] = sensorRead;
        pot02_MeasurementClassTotals[0][classIndex] = pot01_MeasurementClassTotals[0][classIndex] + 1;
        break;
      case 3:
        pot02_Measurements [1][classIndex] = sensorRead;
        pot02_MeasurementClassTotals[1][classIndex] = pot01_MeasurementClassTotals[1][classIndex] + 1;
        break;
        //      case 5:
        pot02_Measurements [2][classIndex] = sensorRead;
        pot02_MeasurementClassTotals[2][classIndex] = pot01_MeasurementClassTotals[2][classIndex] + 1;
        break;
    }
    //for sensor statistics the last measurement
    Serial.println();
    printTimeStamp(secsSince1970);
    Serial.println("  pot02_Measurements: ");
    for (int i = 0; i < 3; i++) {
      Serial.print("  "); // Two spaces in the beginning of the row
      for (int j = 0; j < 20; j++) {
        Serial.print(pot01_Measurements[i][j]);
        Serial.print(", ");
      }
      Serial.println();
    }
    Serial.println();
    Serial.println("  pot02_MeasurementsClassTotals: ");
    for (int i = 0; i < 3; i++) {
      Serial.print("  "); // Two spaces in the beginning of the row
      for (int j = 0; j < 20; j++) {
        Serial.print(pot01_MeasurementClassTotals[i][j]);
        Serial.print(", ");
      }
      Serial.println();
    }
  }
} //END OF store_sensorValue ***


//******************** test_sensorValue *****************************
byte test_sensorValue(unsigned long secsSince1970, int p_ID, int measurementTimeMs, int sensorPin, int sensorRead, byte reCalibrationNeedperSensor, int averSensorRead, int averSensorRef, int potAlarms [][4], int potAlarmTotals [][4], Statistic pot_Statistic, byte plant_Profile[][3]) {

  int highRedAlarm = 3;
  int lowRedAlarm = -3;
  int highRedYellowAlarm = 2;
  int lowYellowAlarm = -2;

  int delay6 = 1500;
  int sensorRatio;
  sensorRatio = sensorRead / averSensorRead; // Long term ratio
  int rel_ReadAver = 100 * (sensorRead / averSensorRef); // measurement ratio in %

  int sensorType = sensorPin + 1;
  if (sensorType > 3) {
    sensorType = sensorType - 3;
  }
  int iAlarm = 0;
  // TEST WHICH IF POT2
  if (sensorPin % 2 == 1) {
    iAlarm = 3;
  }

  //              // TEST WHICH POT
  //              byte iAlarm = 0;
  //              if (p_ID > 1) {
  //                iAlarm = 3;

Serial.println("");
if (rel_ReadAver > plant_Profile[0][0]) {
  Serial.println(F(" Sensor reading above the highest - limit: "));
  Serial.println(F(" RED - LED = value far too high "));
  potAlarms [iAlarm][ sensorType]  = 3; //3
  potAlarmTotals [iAlarm][ sensorType]  = potAlarmTotals [iAlarm][ sensorType] + 1;
  //playAlarms(pot_ID, sensorType, potAlarms, classCount, sensorConfig);
  reCalibrationNeedperSensor = reCalibrationNeedperSensor + 1;
  Serial.print(F("  Action needed level: "));
  Serial.println(reCalibrationNeedperSensor);
  //Serial.println("");
}
else if (rel_ReadAver < plant_Profile[1][2])
{
  Serial.println(F(" Sensor average reading below the lowest - limit: "));
  Serial.println(F(" RED - LED = value far too low "));
  potAlarms [iAlarm][ sensorType]  = -3; //-3
  potAlarmTotals [iAlarm][ sensorType]  = potAlarmTotals [iAlarm][ sensorType] + 1;
  //playAlarms(pot_ID, sensorType, potAlarms, classCount, sensorConfig);
  reCalibrationNeedperSensor = reCalibrationNeedperSensor + 1;
  Serial.print(F("  Action needed level: "));
  Serial.println(reCalibrationNeedperSensor);
  //Serial.println("");
}
else if ((rel_ReadAver > plant_Profile[0][1]) && (rel_ReadAver < plant_Profile[0][0]))
{
  Serial.println(F(" Sensor average reading above the second highest - limit: "));
  Serial.println(F(" RED - LED = sensor value too high "));
  potAlarms [iAlarm + 1][ sensorType]  = 2; //2
  potAlarmTotals [iAlarm + 1][ sensorType]  = potAlarmTotals [iAlarm + 1][ sensorType] + 1;
  //playAlarms(pot_ID, sensorType, potAlarms, classCount, sensorConfig);
  Serial.print(F("  Action needed level: "));
  Serial.println(reCalibrationNeedperSensor);
  Serial.println("");
}
else if ((rel_ReadAver < plant_Profile[1][1]) && (rel_ReadAver > plant_Profile[1][2]))
{
  Serial.println(F(" Sensor average reading below the second lowest - limit: "));
  Serial.println(F(" RED - LED = sensor value too low "));
  potAlarms [iAlarm + 1][ sensorType]  = -2; //-2
  potAlarmTotals [iAlarm + 1][ sensorType]  = potAlarmTotals [iAlarm + 1][ sensorType] + 1;
  //playAlarms(pot_ID, sensorType, potAlarms, classCount, sensorConfig);
  Serial.print(F("  Action needed level: "));
  Serial.println(reCalibrationNeedperSensor);
  Serial.println("");
}
else if ((rel_ReadAver < plant_Profile[0][1]) && (rel_ReadAver > plant_Profile[0][2]))
{
  Serial.println(F(" Sensor average reading within the upper yellow zone: "));
  Serial.println(F(" YELLOW - LED, sensor value a little high"));
  potAlarms [iAlarm + 2][ sensorType]  = 1;
  potAlarmTotals [iAlarm + 2][ sensorType]  = potAlarmTotals [iAlarm + 2][ sensorType] + 1;
  //playAlarms(pot_ID, sensorType, potAlarms, classCount, sensorConfig);
  Serial.println("");
}
else if ((rel_ReadAver > plant_Profile[1][1]) && (rel_ReadAver < plant_Profile[1][0]))
{
  Serial.println(F(" Sensor average reading within the lower yellow zone: "));
  Serial.println(F(" YELLOW - LED, sensor value a little low"));
  potAlarms [iAlarm + 2][ sensorType]  = -1;
  potAlarmTotals [iAlarm + 2][ sensorType]  = potAlarmTotals [iAlarm + 2][ sensorType] + 1;
  //playAlarms(pot_ID, sensorType, potAlarms, classCount, sensorConfig);
}
else
{
  Serial.println();
}

switch (sensorPin) {
case 4:
  Serial.println(F(" ================ LIGHTNESS ON POT1 ================= ") );
  printTimeStamp(secsSince1970);
  Serial.print(F("  Photocell sensor in POT: "));
  Serial.println(p_ID);
  Serial.print(F("  Sensor pin:               "));
  Serial.println(sensorPin);
  break;
case 5:
  Serial.println(F(" ================ LIGHTNESS ON POT2 ================= ") );
  printTimeStamp(secsSince1970);
  Serial.print(F("  Photocell sensor in POT: "));
  Serial.println(p_ID);
  Serial.print(F("  Sensor pin:               "));
  Serial.println(sensorPin);
  break;
case 2:
  Serial.println(F(" ================ HUMIDITY IN POT1 ================== ") );
  printTimeStamp(secsSince1970);
  Serial.print(F("  Humidity sensor in POT: "));
  Serial.println(p_ID);
  Serial.print(F("  Sensor pin:              "));
  Serial.println(sensorPin);
  break;
case 3:
  Serial.println(F(" ================ HUMIDITY IN POT2 ================== ") );
  printTimeStamp(secsSince1970);
  Serial.print(F("  Humidity sensor in POT: "));
  Serial.println(p_ID);
  Serial.print(F("  Sensor pin:              "));
  Serial.println(sensorPin);
  break;
case 0:
  Serial.println(F(" ================ TEMPERATURE IN POT1 =============== ") );
  printTimeStamp(secsSince1970);
  Serial.print(F("  Temperature sensor in POT: "));
  Serial.println(p_ID);
  Serial.print(F("  Sensor pin:                 "));
  Serial.println(sensorPin);
  break;
case 1:
  Serial.println(F(" ================ TEMPERATURE IN POT2 =============== ") );
  printTimeStamp(secsSince1970);
  Serial.print(F("  Temperature sensor in POT: "));
  Serial.println(p_ID);
  Serial.print(F("  Sensor pin:                 "));
  Serial.println(sensorPin);
  break;
}
Serial.print(F("  sensorRead        "));
Serial.println(sensorRead);
Serial.print(F("  averSensorRead    "));
Serial.println((averSensorRead));
Serial.print(F("  averSensorRef     "));
Serial.println((averSensorRef));
//  Serial.print(F("  sensorRead / averSensorRead "));
//  Serial.println(float(sensorRatio));
//  Serial.print(F("  sensorRead / averSensorRef  "));
//  Serial.println(float(rel_ReadAver));

sensorType = sensorPin + 1;
if (sensorType > 3) {
  sensorType = sensorType - 3;
}

// TEST WHICH POT
iAlarm = 0;
if (p_ID > 1) {
  iAlarm = 3;
}

Serial.print(F("  RED ALARM    = "));
Serial.println(potAlarms [iAlarm][ sensorType] );

Serial.print(F("  ORANGE ALARM = "));
Serial.println(potAlarms [iAlarm + 1][ sensorType] );

Serial.print(F("  YELLOW ALARM = ") );
Serial.println(potAlarms [iAlarm + 2][ sensorType] );

Serial.print(F("  RED TOTAL    = "));
Serial.println(potAlarmTotals [iAlarm][ sensorType] );

Serial.print(F("  ORANGE TOTAL = "));
Serial.println(potAlarmTotals [iAlarm + 1][ sensorType] );

Serial.print(F("  YELLOW TOTAL = ") );
Serial.println(potAlarmTotals [iAlarm + 2][ sensorType] );

Serial.println(F(" ================================================ ") );
//*************** SENSOR READINGS STATISTICS **********************
Serial.print(F("  Sensor reading count "));
Serial.println(pot_Statistic.count());
Serial.print(F("  Reading average      "));
Serial.println((pot_Statistic.average()), 0);
Serial.print(F("  Standard deviation   "));
Serial.println((pot_Statistic.pop_stdev()), 0);
Serial.println(F(" ================================================ " ));
delay(delay6);

return reCalibrationNeedperSensor;
}// END OF test_sensorValue

void playAlarms(int p_ID, int sensorPin, int potAlarms[][4], const int classCount, byte sensorConfig[][6]) {
  //************************** playAlarms  ****************************************************
  // blinks
  byte blinkCount1 = 1; // Number of LED-BLINKs, when green alarm
  byte blinkCount2 = 2; // Number of LED-BLINKs, when yellow alarm
  byte blinkCount3 = 3; // Number of LED-BLINKs, when red alarm (3)
  byte blinkCount4 = 4; // Number of LED-BLINKs, when red alarm (4)
  int blinkDelay = 200; // ms

  int A_tone = 440;
  int tooLowTone = 110; //A2_tone
  int tooHighTone = 880; //A5_tone

  // beep delays
  int delay5 = 1000; // For beeper

  int sensorType = sensorPin + 1;
  if (sensorType > 3) {
    sensorType = sensorType - 3;
  }
//  // TEST IF POT2
//   int iAlarm = 0;
//  if (sensorPin % 2 == 1) {
//    iAlarm = 3;
//  }

  //              int iAlarm = 0;
  //              if (p_ID > 1) {
  //                int iAlarm = iAlarm + 3;
  //              }

  int i = 0;
  for (int iAlarm = 0; iAlarm < 6; i++){
  switch (potAlarms [iAlarm][sensorType]) {

    case 0:
      digitalWrite(led3, LOW);   // turn the yellow LED off (LOW is the voltage level)
      digitalWrite(led4, LOW);   // turn the red LED off (LOW is the voltage level)
      //digitalWrite(led2, HIGH);   // turn the green LED on (HIGH is the voltage level)
      for (i = 0; i < blinkCount1; i++) {
        Serial.println("  GREEN LED blink");
        digitalWrite(led2, LOW);    // turn the LED off by making the voltage LOW
        delay(blinkDelay);               // wait for 50 milliseconds
        digitalWrite(led2, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(blinkDelay);               // wait for 50 milliseconds
      }
      digitalWrite(led2, LOW);
      break;

    case 1:
      digitalWrite(led2, LOW);   // turn the green LED off (LOW is the voltage level)
      digitalWrite(led4, LOW);   // turn the red LED off
      //digitalWrite(led3, HIGH);   // turn the yellow LED on (HIGH is the voltage level)
      for (i = 0; i < blinkCount2; i++) {
        Serial.println("  YELLOW LED blink");
        digitalWrite(led3, LOW);    // turn the LED off by making the voltage LOW
        delay(blinkDelay);               // wait for 50 milliseconds
        digitalWrite(led3, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(blinkDelay);               // wait for 50 milliseconds
        if (sensorConfig[sensorPin][4] == 1) {
          tone(pin5, A_tone); //play sound
          delay(delay5);
          noTone(pin5);
        }
      }
      if (sensorConfig[sensorPin][4] == 1) {
        tone(pin5, tooHighTone); //play sound
        delay(delay5);
        noTone(pin5);
      }
      digitalWrite(led3, LOW);   // turn the red LED off
      digitalWrite(led3, LOW);
      break;

    case 2:
      digitalWrite(led2, LOW);   // turn the green LED off (LOW is the voltage level)
      digitalWrite(led3, LOW);   // turn the yellow LED off (LOW is the voltage level)
      //digitalWrite(led4, HIGH);   // turn the red LED on (HIGH is the voltage level)
      for (i = 0; i < blinkCount3; i++) {
        Serial.println("  RED LED blink");
        digitalWrite(led4, LOW);    // turn the LED off by making the voltage LOW
        delay(blinkDelay);               // wait for 50 milliseconds
        digitalWrite(led4, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(blinkDelay);               // wait for 50 milliseconds
        if (sensorConfig[sensorPin][4] == 1) {
          tone(pin5, A_tone); //play sound
          delay(delay5);
          noTone(pin5);
        }
      }
      if (sensorConfig[sensorPin][4] == 1) {
        tone(pin5, tooHighTone); //play sound
        delay(delay5);
        noTone(pin5);
      }
      digitalWrite(led4, LOW);
      break;

    case 3:
      digitalWrite(led2, LOW);   // turn the green LED off (LOW is the voltage level)
      digitalWrite(led3, LOW);   // turn the yellow LED off (LOW is the voltage level)
      //digitalWrite(led4, HIGH);   // turn the red LED on (HIGH is the voltage level)
      for (i = 0; i < (blinkCount4); i++) {
        Serial.println("  RED LED blink");
        digitalWrite(led4, LOW);    // turn the LED off by making the voltage LOW
        delay(blinkDelay);               // wait for 50 milliseconds
        digitalWrite(led4, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(blinkDelay);               // wait for 50 milliseconds
        if (sensorConfig[sensorPin][4] == 1) {
          tone(pin5, A_tone); //play sound
          delay(delay5);
          noTone(pin5);
        }
      }
      if (sensorConfig[sensorPin][4] == 1) {
        tone(pin5, tooHighTone); //play sound
        delay(delay5);
        noTone(pin5);
      }
      digitalWrite(led4, LOW);
      break;
    case -1:
      digitalWrite(led2, LOW);   // turn the green LED off (LOW is the voltage level)
      digitalWrite(led4, LOW);   // turn the green LED off (LOW is the voltage level)
      //digitalWrite(led3, HIGH);   // turn the yellow LED on (HIGH is the voltage level)
      for (i = 0; i < blinkCount2; i++) {
        Serial.println("  YELLOW LED blink");
        digitalWrite(led3, LOW);    // turn the LED off by making the voltage LOW
        delay(blinkDelay);               // wait for 50 milliseconds
        digitalWrite(led3, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(blinkDelay);               // wait for 50 milliseconds
        if (sensorConfig[sensorPin][4] == 1) {
          tone(pin5, A_tone); //play sound
          delay(delay5);
          noTone(pin5);
        }
      }
      if (sensorConfig[sensorPin][4] == 1) {
        tone(pin5, tooLowTone); //play sound
        delay(delay5);
        noTone(pin5);
      }
      digitalWrite(led3, LOW);   // turn the red LED off                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     (HIGH is the voltage level)
      break;
    case -2:
      digitalWrite(led2, LOW);   // turn the green LED off (LOW is the voltage level)
      digitalWrite(led3, LOW);   // turn the yellow LED off (LOW is the voltage level)
      //digitalWrite(led4, HIGH);   // turn the red LED on (HIGH is the voltage level)
      for (i = 0; i < blinkCount3; i++) {
        Serial.println("  RED LED blink");
        digitalWrite(led4, LOW);    // turn the LED off by making the voltage LOW
        delay(blinkDelay);               // wait for 50 milliseconds
        digitalWrite(led4, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(blinkDelay);               // wait for 50 milliseconds
        if (sensorConfig[sensorPin][4] == 1) {
          tone(pin5, A_tone); //play sound
          delay(delay5);
          noTone(pin5);
        }
      }
      if (sensorConfig[sensorPin][4] == 1) {
        tone(pin5, tooLowTone); //play sound
        delay(delay5);
        noTone(pin5);
      }
      digitalWrite(led4, LOW);   // turn the red LED off
      break;
    case -3:
      digitalWrite(led2, LOW);   // turn the green LED off (LOW is the voltage level)
      digitalWrite(led3, LOW);   // turn the yellow LED off (LOW is the voltage level)
      //digitalWrite(led4, HIGH);   // turn the red LED on (HIGH is the voltage level)
      for (i = 0; i < (blinkCount4); i++) {
        Serial.println("  RED LED blink");
        digitalWrite(led4, LOW);    // turn the LED off by making the voltage LOW
        delay(blinkDelay);               // wait for 50 milliseconds
        digitalWrite(led4, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(blinkDelay);               // wait for 50 milliseconds
        if (sensorConfig[sensorPin][4] == 1) {
          tone(pin5, A_tone); //play sound
          delay(delay5);
          noTone(pin5);
        }
      }
      if (sensorConfig[sensorPin][4] == 1) {
        tone(pin5, tooLowTone); //play sound
        delay(delay5);
        noTone(pin5);
      }
      digitalWrite(led4, LOW);
      break;
    default:
      digitalWrite(led2, LOW);   // turn the green LED off (LOW is the voltage level)
      digitalWrite(led3, LOW);   // turn the yellow LED off (LOW is the voltage level)
      digitalWrite(led4, LOW);   // turn the red LED on (HIGH is the voltage level)
      noTone(pin5);
      break;
  }
  }
}

//********** SENSOR MEASUREMENT STATISTICS: averages, alarms, histograms, deviations **************************
void sensorStatistics(unsigned long secsSince1970, int pot_ID, int MeasurementClassTotals [][20], const int maxsensorSettingsNumber, int potAlarmTotals [][4], int classCount, Statistic Pot_StatisticX, Statistic Pot_StatisticY, Statistic Pot_StatisticZ) {

  int delay3 = 3000;
  uint32_t cIndexTot;

  // int potAlarms [6][4] = { // 2 pots, max 3 sensors in each
  //  {1, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
  //  {1, 0, 0, 0}, //LOW LIMIT RED-ALARMS
  //  {1, 0, 0, 0}, //YELLOW-ALARMS
  //  {2, 0, 0, 0}, //HIGH LIMIT RED-ALARMS
  //  {2, 0, 0, 0}, //LOW LIMIT RED-ALARMS
  //  {2, 0, 0, 0}  //YELLOW-ALARMS
  //};

  Serial.println(" ");
  Serial.print("  POT-STATISTICS: POT_ID             = ");
  Serial.println(pot_ID);
  Serial.println("  HISTOGRAM OF THE LIGHT-SENSOR ");
  printTimeStamp(secsSince1970);
  Serial.print("  TOTAL MEASUREMENT-COUNT           = ");
  Serial.println(Pot_StatisticX.count());

  switch (pot_ID) {
    case 1:
      Serial.print("  TOTAL OF HIGH LIMIT RED-ALARMS = ");
      Serial.println(potAlarmTotals [pot_ID - 1][1]);
      Serial.print("  % OF HIGH LIMIT RED-ALARMS     = ");
      Serial.println(100 * potAlarmTotals [pot_ID - 1][1] / (Pot_StatisticX.count()));
      Serial.println();
      Serial.print("  TOTAL OF LOW LIMIT RED-ALARMS  = ");
      Serial.println(potAlarmTotals [pot_ID][1]);
      Serial.print("  % OF LOW LIMIT RED-ALARMS      = ");
      Serial.println(100 * potAlarmTotals [pot_ID][1] / (Pot_StatisticX.count()));
      Serial.println();
      Serial.print("  TOTAL OF YELLOW-ALARMS         = ");
      Serial.println(potAlarmTotals [pot_ID + 1][1]);
      Serial.print("  % OF YELLOW-ALARMS             = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 1][1] / (Pot_StatisticX.count()));
      Serial.println();
      break;
    case 2:
      Serial.print("  TOTAL OF HIGH LIMIT RED-ALARMS = ");
      Serial.println(potAlarmTotals [pot_ID + 1][1]);
      Serial.print("  % OF HIGH LIMIT RED-ALARMS     = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 1][1] / (Pot_StatisticX.count()));
      Serial.println();
      Serial.print("  TOTAL OF LOW LIMIT RED-ALARMS  = ");
      Serial.println(potAlarmTotals [pot_ID + 2][1]);
      Serial.print("  % OF HIGH LIMIT RED-ALARMS     = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 2][1] / (Pot_StatisticX.count()));
      Serial.println();
      Serial.print("  TOTAL OF YELLOW-ALARMS         = ");
      Serial.println(potAlarmTotals [pot_ID + 3][1]);
      Serial.print("  % OF YELLOW-ALARMS             = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 3][1] / (Pot_StatisticX.count()));
      Serial.println();
      break;
  }
  Serial.print(F("  AVERAGE OF THE SENSOR           = "));
  Serial.println(Pot_StatisticX.average(), 0);

  Serial.print(F("  STANDARD DEVIATION              = "));
  Serial.println(Pot_StatisticX.pop_stdev(), 0);

  //for (int i = 0; i < maxsensorSettingsNumber; i++) {
  //print histogram
  int i = 0;
  for (int j = 0; j < classCount; j++) {
    Serial.print("  ");
    Serial.print(MeasurementClassTotals [i][j]);
    Serial.print(" ");
    for (int k = 0; 2 * k < MeasurementClassTotals [i][j]; k++) {
      Serial.print("*");
    }
    Serial.print(" ");
    Serial.println(j + 1);

    // Send measurementClassTotals via MQTT to reporting
    switch (pot_ID) {
      case 1:
        Serial.print(F("\n  Sending POT_1 lightness_sensor val "));
        Serial.print(MeasurementClassTotals [i][j]);
        Serial.print("...");
        cIndexTot = MeasurementClassTotals [i][j];
        if (! A0lightClassTot.publish(cIndexTot)) {
          Serial.println(F("  Failed"));
        } else {
          Serial.println(F("  OK!"));
        }
        delay(1500);
        break;
      case 2:
        //        Serial.print(F("\nSending POT_2 lightness_sensor val "));
        //        Serial.print(MeasurementClassTotals [i][j]);
        //        Serial.print("...");
        //        cIndexTot = MeasurementClassTotals [i][j];
        //        if (! A1lightClassTot.publish(cIndexTot)) {
        //          Serial.println(F("Failed"));
        //        } else {
        //          Serial.println(F("OK!"));
        //        }
        //        delay(1000);
        break;
    }
  }
  Serial.println();
  //}// END OF for j loop

  Serial.println();
  delay(delay3);
  //=========================================================

  Serial.print(F("  POT-STATISTICS: POT_ID           = "));
  Serial.println(pot_ID);
  Serial.println(F("  HISTOGRAM OF THE HUMIDITY-SENSOR "));
  printTimeStamp(secsSince1970);
  Serial.print(F("  TOTAL MEASUREMENT-COUNT         = "));
  Serial.println(Pot_StatisticY.count());
  switch (pot_ID) {
    case 1:
      Serial.print("  TOTAL OF HIGH LIMIT RED-ALARMS = ");
      Serial.println(potAlarmTotals [pot_ID - 1][2]);
      Serial.print("  % OF HIGH LIMIT RED-ALARMS     = ");
      Serial.println(100 * potAlarmTotals [pot_ID - 1][2] / (Pot_StatisticY.count()));
      Serial.println();
      Serial.print("  TOTAL OF LOW LIMIT RED-ALARMS  = ");
      Serial.println(potAlarmTotals [pot_ID][2]);
      Serial.print("  % OF LOW LIMIT RED-ALARMS      = ");
      Serial.println(100 * potAlarmTotals [pot_ID][2] / (Pot_StatisticY.count()));
      Serial.println();
      Serial.print("  TOTAL OF YELLOW-ALARMS         = ");
      Serial.println(potAlarmTotals [pot_ID + 1][2]);
      Serial.print("  % OF YELLOW-ALARMS             = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 1][2] / (Pot_StatisticY.count()));
      Serial.println();
      break;
    case 2:
      Serial.print("  TOTAL OF HIGH LIMIT RED-ALARMS = ");
      Serial.println(potAlarmTotals [pot_ID + 1][2]);
      Serial.print("  % OF HIGH LIMIT RED-ALARMS     = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 1][2] / (Pot_StatisticY.count()));
      Serial.println();
      Serial.print("  TOTAL OF LOW LIMIT RED-ALARMS  = ");
      Serial.println(potAlarmTotals [pot_ID + 2][2]);
      Serial.print("  % OF LOW LIMIT RED-ALARMS      = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 2][2] / (Pot_StatisticY.count()));
      Serial.println();
      Serial.print("  TOTAL OF YELLOW-ALARMS         = ");
      Serial.println(potAlarmTotals [pot_ID + 3][2]);
      Serial.print("  % OF YELLOW-ALARMS             = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 3][2] / (Pot_StatisticY.count()));
      Serial.println();
      break;
  }
  Serial.print(F("  AVERAGE OF THE SENSOR           =  "));
  Serial.println(Pot_StatisticY.average(), 0);

  Serial.print(F("  STANDARD DEVIATION              =  "));
  Serial.println(Pot_StatisticY.pop_stdev(), 0);
  Serial.println();

  //for (int i = 0; i < maxsensorSettingsNumber; i++) {
  //print histogram
  i = 1;
  for (int j = 0; j < classCount; j++) {
    Serial.print("  ");
    Serial.print(MeasurementClassTotals [i][j]);
    Serial.print(" ");
    for (int k = 0; 2 * k < MeasurementClassTotals [i][j]; k++) {
      Serial.print("@");
    }
    Serial.print(" ");
    Serial.println(j + 1);

    // Send measurementClassTotals via MQTT to reporting
    switch (pot_ID) {
      case 1:
        Serial.print(F("\n  Sending POT_1 humiditysensor val "));
        Serial.print(MeasurementClassTotals [i][j]);
        Serial.print("...");
        cIndexTot = MeasurementClassTotals [i][j];
        if (! A2humidClassTot.publish(cIndexTot)) {
          Serial.println(F("Failed"));
        } else {
          Serial.println(F("  OK!"));
        }
        delay(1000);
        break;
        //      case 2:
        //        Serial.print(F("\n  Sending POT_2 humiditysensor val "));
        //        Serial.print(MeasurementClassTotals [i][j]);
        //        Serial.print("...");
        //        cIndexTot = MeasurementClassTotals [i][j];
        //        if (! A3humidClassTot.publish(cIndexTot)) {
        //          Serial.println(F("  Failed"));
        //        } else {
        //          Serial.println(F("  OK!"));
        //        }
        //        delay(1000);
        //        break;
    }
  }
  Serial.println();
  //}// END OF for i loop

  delay(delay3);
  // ============================================================

  Serial.print(F("  POT-STATISTICS: POT_ID          = "));
  Serial.println(pot_ID);
  Serial.println(F("  HISTOGRAM OF THE TEMPERATURE-SENSOR "));
  printTimeStamp(secsSince1970);
  Serial.print(F("  TOTAL MEASUREMENT-COUNT         = "));
  Serial.println(Pot_StatisticZ.count());
  switch (pot_ID) {
    case 1:
      Serial.print("  TOTAL OF HIGH LIMIT RED-ALARMS = ");
      Serial.println(potAlarmTotals [pot_ID - 1][3]);
      Serial.print("  % OF HIGH LIMIT RED-ALARMS     = ");
      Serial.println(100 * potAlarmTotals [pot_ID - 1][3] / (Pot_StatisticZ.count()));
      Serial.println();
      Serial.print("  TOTAL OF LOW LIMIT RED-ALARMS  = ");
      Serial.println(potAlarmTotals [pot_ID][3]);
      Serial.print("  % OF LOW LIMIT RED-ALARMS      = ");
      Serial.println(100 * potAlarmTotals [pot_ID][3] / (Pot_StatisticZ.count()));
      Serial.println();
      Serial.print("  TOTAL OF YELLOW-ALARMS         = ");
      Serial.println(potAlarmTotals [pot_ID + 1][3]);
      Serial.print("  % OF YELLOW-ALARMS             = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 1][3] / (Pot_StatisticZ.count()));
      Serial.println();
      break;
    case 2:
      Serial.print("  TOTAL OF HIGH LIMIT RED-ALARMS = ");
      Serial.println(potAlarmTotals [pot_ID + 1][3]);
      Serial.print("  % OF HIGH LIMIT RED-ALARMS     = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 1][3] / (Pot_StatisticZ.count()));
      Serial.println();
      Serial.print("  TOTAL OF HIGH LIMIT YELLOW-AL. = ");
      Serial.println(potAlarmTotals [pot_ID + 2][3]);
      Serial.print("  % OF HIGH LIMIT YELLOW-ALARMS  = ");
      Serial.println(100 * potAlarmTotals [pot_ID + 2][3] / (Pot_StatisticZ.count()));
      Serial.println();
      Serial.print("  TOTAL OF YELLOW-ALARMS          = ");
      Serial.println(potAlarmTotals [pot_ID + 3][3]);
      Serial.print("  % OF YELLOW-ALARMS =             ");
      Serial.println(100 * potAlarmTotals [pot_ID + 3][3] / (Pot_StatisticZ.count()));
      Serial.println();
      break;
  }
  Serial.print(F("  AVERAGE OF THE SENSOR           = "));
  Serial.println(Pot_StatisticZ.average(), 0);

  Serial.print(F("  STANDARD DEVIATION              = "));
  Serial.println(Pot_StatisticZ.pop_stdev(), 0);

  //for (int i = 0; i < maxsensorSettingsNumber; i++) {
  //print histogram
  i = 2;
  for (int j = 0; j < classCount; j++) {
    Serial.print("  ");
    Serial.print(MeasurementClassTotals [i][j]);
    Serial.print(" ");
    for (int k = 0; 2 * k < MeasurementClassTotals [i][j]; k++) {
      Serial.print("#");
    }
    Serial.print(" ");
    Serial.println(j + 1);

    // Send measurementClassTotals via MQTT to reporting
    switch (pot_ID) {
      case 1:
        Serial.print(F("\n  Sending POT_1 temperaturesensor val "));
        Serial.print(MeasurementClassTotals [i][j]);
        Serial.print("...");
        cIndexTot = MeasurementClassTotals [i][j];
        if (! A4tempClassTot.publish(cIndexTot)) {
          Serial.println(F("  Failed"));
        } else {
          Serial.println(F("  OK!"));
        }
        delay(1000);
        break;
        //      case 2:
        //        Serial.print(F("\n  Sending POT_2 temperaturesensor val "));
        //        Serial.print(MeasurementClassTotals [i][j]);
        //        Serial.print("...");
        //        cIndexTot = MeasurementClassTotals [i][j];
        //        if (! A5tempClassTot.publish(cIndexTot)) {
        //          Serial.println(F("  Failed"));
        //        } else {
        //          Serial.println(F("  OK!"));
        //        }
        //        delay(1000);
        //        break;
    }
  }
  Serial.println();
  //}// END OF for i loop

  Serial.println();
  delay(delay3);
}

/**************************************************************************/
/*!
    @brief  Displays the driver mode (tiny of normal), and the buffer
            size if tiny mode is not being used

    @note   The buffer size and driver mode are defined in cc3000_common.h
*/
/**************************************************************************/
void displayDriverMode(void)
{
#ifdef CC3000_TINY_DRIVER
  Serial.println(F("CC3000 is configure in 'Tiny' mode"));
#else
  //  Serial.print(F("RX Buffer : "));
  //  Serial.print(CC3000_RX_BUFFER_SIZE);
  //  Serial.println(F(" bytes"));
  //  Serial.print(F("TX Buffer : "));
  //  Serial.print(CC3000_TX_BUFFER_SIZE);
  //  Serial.println(F(" bytes"));
#endif
}

/**************************************************************************/
/*!
    @brief  Tries to read the CC3000's internal firmware patch ID
*/
/**************************************************************************/
uint16_t checkFirmwareVersion(void)
{
  uint8_t major, minor;
  uint16_t version;

#ifndef CC3000_TINY_DRIVER
  if (!cc3000.getFirmwareVersion(&major, &minor))
  {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

/**************************************************************************/
/*!
    @brief  Tries to read the 6-byte MAC address of the CC3000 module
*/
/**************************************************************************/
void displayMACAddress(void)
{
  uint8_t macAddress[6];

  if (!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}


/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;

  if (!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("  Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    return true;
  }
}

// Minimalist time server query; adapted from Adafruit Gutenbird sketch,
// which in turn has roots in Arduino UdpNTPClient tutorial.
unsigned long getTime(void) {

  uint8_t       buf[48];
  unsigned long ip, startTime, t = 0L;

  Serial.print(F("  Locating time server..."));

  // Hostname to IP lookup; use NTP pool (rotates through servers)
  if (cc3000.getHostByName("pool.ntp.org", &ip)) {
    static const char PROGMEM
    timeReqA[] = { 227,  0,  6, 236 },
                 timeReqB[] = {  49, 78, 49,  52 };

    Serial.println(F("\r\n  Attempting connection..."));
    startTime = millis();
    do {
      client = cc3000.connectUDP(ip, 123);
    } while ((!client.connected()) &&
             ((millis() - startTime) < connectTimeout));

    if (client.connected()) {
      Serial.print(F("  connected!\r\nIssuing request..."));

      // Assemble and issue request packet
      memset(buf, 0, sizeof(buf));
      memcpy_P( buf    , timeReqA, sizeof(timeReqA));
      memcpy_P(&buf[12], timeReqB, sizeof(timeReqB));
      client.write(buf, sizeof(buf));

      Serial.print(F("\r\n  Awaiting response..."));
      memset(buf, 0, sizeof(buf));
      startTime = millis();
      while ((!client.available()) &&
             ((millis() - startTime) < responseTimeout));
      if (client.available()) {
        client.read(buf, sizeof(buf));
        t = (((unsigned long)buf[40] << 24) |
             ((unsigned long)buf[41] << 16) |
             ((unsigned long)buf[42] <<  8) |
             (unsigned long)buf[43]) - 2208988800UL;
        Serial.print(F("  OK\r\n"));
      }
      client.close();
    }
  }
  if (!t) Serial.println(F("  error"));
  return t;
}

//**************************************************
void printTimeStamp(unsigned long secsSince1970) {

  // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
  Serial.print(F("  Current UNIX time: "));
  Serial.print(secsSince1970);
  Serial.println(F(" (since 1/1/1970 UTC)"));

  // convert NTP time into human readable clock-time:
  // print the hour, minute and second:
  Serial.print("  The local time is "); // Local time = 3h + UTC is the time at Greenwich Meridian (GMT)
  Serial.print(3 + (secsSince1970 % 86400L) / 3600); // print the hour (86400 equals secs per day)
  Serial.print(':');

  if (((secsSince1970 % 3600) / 60) < 10) {
    // In the first 10 minutes of each hour, we'll want a leading '0'
    Serial.print('0');
  }
  Serial.print((secsSince1970  % 3600) / 60); // print the minute (3600 equals secs per hour)
  Serial.print(':');

  if ((secsSince1970 % 60) < 10) {
    // In the first 10 seconds of each minute, we'll want a leading '0'
    Serial.print('0');
  }
  Serial.println(secsSince1970 % 60); // print the second
  Serial.println();
  delay(delay3);
}
/**************************************************************************/
/*!
    @brief  Begins an SSID scan and prints out all the visible networks
*/
/**************************************************************************/

void listSSIDResults(void)
{
  uint32_t index;
  uint8_t valid, rssi, sec;
  char ssidname[33];

  if (!cc3000.startSSIDscan(&index)) {
    Serial.println(F("SSID scan failed!"));
    return;
  }

  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));

  while (index) {
    index--;

    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);

    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    Serial.println();
    Serial.print(F("RSSI         : "));
    Serial.println(rssi);
    Serial.print(F("Security Mode: "));
    Serial.println(sec);
    Serial.println();
  }
  Serial.println(F("================================================"));

  cc3000.stopSSIDscan();
}
//******************* MOSQUITTO *******************
// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
//void MQTT_connect() {
//  int8_t ret;
//
//  // Stop if already connected.
//  if (mqtt.connected()) {
//    return;
//  }
//boolean CC3000connect(const char* wlan_ssid, const char* wlan_pass, uint8_t wlan_security);
//
//  Serial.print("Connecting to MQTT... ");
//
//  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
//    Serial.println(mqtt.connectErrorString(ret));
//    if (ret < 0)
//
//      CC3000connect(WLAN_SSID, WLAN_PASS, WLAN_SECURITY);  // y0w, lets connect to wifi again
//
//    Serial.println("  Retrying MQTT connection in 5 seconds...");
//    mqtt.disconnect();
//    delay(5000);  // wait 5 seconds
//  }
//  Serial.println("MQTT Connected!");
//}

//boolean CC3000connect(const char* wlan_ssid, const char* wlan_pass, uint8_t wlan_security) {
//  Watchdog.reset();
//
//  // Check for compatible firmware
//  if (checkFirmwareVersion() < 0x113)   halt("  Wrong firmware version!");
//
//  // Delete any old connection data on the module
//  Serial.println(F("\nDeleting old connection profiles"));
//  if (!cc3000.deleteProfiles())     halt("  Failed!");
//
//#ifdef STATICIP
//  Serial.println(F("  Setting static IP"));
//  uint32_t ipAddress = cc3000.IP2U32(10, 0, 1, 19);
//  uint32_t netMask = cc3000.IP2U32(255, 255, 255, 0);
//  uint32_t defaultGateway = cc3000.IP2U32(10, 0, 1, 1);
//  uint32_t dns = cc3000.IP2U32(8, 8, 4, 4);
//
//  if (!cc3000.setStaticIPAddress(ipAddress, netMask, defaultGateway, dns)) {
//    Serial.println(F("  Failed to set static IP!"));
//    while(1);
//  }
//#endif
//
//  // Attempt to connect to an access point
//  Serial.print(F("\n  Attempting to connect to "));
//  Serial.print(wlan_ssid); Serial.print(F("..."));
//
//  Watchdog.disable();
//  // try 3 times
//  if (!cc3000.connectToAP(wlan_ssid, wlan_pass, wlan_security, 3)) {
//    return false;
//  }
//
//  Watchdog.enable(8000);
//  Serial.println(F("  Connected!"));
//
//uint8_t retries;
//#ifndef STATICIP
//  /* Wait for DHCP to complete */
//  Serial.println(F("  Requesting DHCP"));
//  retries = 10;
//  while (!cc3000.checkDHCP())
//  {
//    Watchdog.reset();
//    delay(1000);
//    retries--;
//    if (!retries) return false;
//  }
//#endif
//  /* Display the IP address DNS, Gateway, etc.  */
//  retries = 10;
//  while (! displayConnectionDetails()) {
//    Watchdog.reset();
//    delay(1000);
//    retries--;
//    if (!retries) return false;
//  }
//
//  Watchdog.reset();
//
//  return true;
//}


/**************************************************************************/
/*!
    @brief  Tries to read the CC3000's internal firmware patch ID
*/
/**************************************************************************/
//uint16_t checkFirmwareVersion(void)
//{
//  uint8_t major, minor;
//  uint16_t version;
//
//  if(!cc3000.getFirmwareVersion(&major, &minor))
//  {
//    Serial.println(F("  Unable to retrieve the firmware version!\r\n"));
//    version = 0;
//  }
//  else
//  {
//    Serial.print(F("  Firmware V. : "));
//    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
//    version = major; version <<= 8; version |= minor;
//  }
//  return version;
//}


/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
//bool displayConnectionDetails(void)
//{
//  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
//
//  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
//  {
//    Serial.println(F("  Unable to retrieve the IP Address!\r\n"));
//    return false;
//  }
//  else
//  {
//    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
//    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
//    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
//    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
//    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
//    Serial.println();
//    return true;
//  }
//}
//*************************************************************************
//// Taking care of special key-events.
void keypadEvent(KeypadEvent eventKey) {
  //if (flag == 0){
  switch (keypad.getState()) {
    case PRESSED:

      if (eventKey == '8') {
        Serial.print(F("  "));
        Serial.print(eventKey);
        Serial.println(F(" => FUNCTION_2"));
        Serial.println();
        lcd.init();   lcd.backlight();
        lcd.print(F("FUNCTION_2      "));
        //lcd.setCursor(0, 1);
        // lcd.print(F("     "));
        //Serial.println();
        //printTimeStamp(secsSince1970);
        Serial.println();
        delay(delay6);
        break;
      }
      if (eventKey == '*') {
        Serial.print(F("  "));
        Serial.print(eventKey);
        Serial.println(F(" => CONFIGURE SENSORS"));
        flag = 0;
        configureSensor(maxsensorCount, sensorConfig, notAllowedToEdit, flag);
        break;
      }

      if (eventKey == '9') {
        Serial.print(F("  "));
        Serial.print(eventKey);
        Serial.println(F(" => FUNCTION_3"));
        Serial.println();
        lcd.init();   lcd.backlight();
        lcd.print(F("FUNCTION_3      "));
        //        lcd.setCursor(0, 1);
        //        lcd.print(F(""));
        delay(delay6);

        break;
      }
      if (eventKey == '7') {
        Serial.print(F("  "));
        Serial.print(eventKey);
        Serial.println(F(" => FUNCTION_1"));
        Serial.println();
        lcd.init();   lcd.backlight();
        lcd.print(F("FUNCTION_1     "));
        //        lcd.setCursor(0, 1);
        //        lcd.print(F("PUSH 9 TO START"));
        delay(delay6);
        break;
      }
      if (eventKey == '#') {
        Serial.print(F("  "));
        Serial.print(eventKey);
        Serial.print(F(" => PRINT ALL SENSOR CONFIGURATIONS "));
        Serial.println();
        lcd.init();   lcd.backlight();
        lcd.print(F("PRINT ALL SENSOR"));
        lcd.setCursor(0, 1);
        lcd.print(F("CONFIGURATIONS  "));
        delay(delay6);
        lcd.init();   lcd.backlight();

        flag = 1;
        sensorConfPrint(sensorPin, sensorConfig, flag);

        //       Serial.println(F("UpdateNTPTime"));
        //        if (mysntp.UpdateNTPTime())
        //        {
        //          Serial.println(F("Current local time is:"));
        //          mysntp.ExtractNTPTime(mysntp.NTPGetTime(&now, true), &timeExtract);
        //
        //          Serial.print(timeExtract.hour); Serial.print(F(":")); Serial.print(timeExtract.min); Serial.print(F(":")); Serial.print(timeExtract.sec); Serial.print(F(".")); Serial.println(timeExtract.millis);
        //          Serial.print(pF(&dayStrs[timeExtract.wday])); Serial.print(F(", ")); Serial.print(pF(&monthStrs[timeExtract.mon])); Serial.print(F(" ")); Serial.print(timeExtract.mday); Serial.print(F(", ")); Serial.println(timeExtract.year);
        //          Serial.print(F("Day of year: ")); Serial.println(timeExtract.yday + 1);
        //
        //          lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
        //          lcd.print(timeExtract.hour); lcd.print(F(":")); lcd.print(timeExtract.min); lcd.print(F(":")); lcd.print(timeExtract.sec);
        //          lcd.print(F(".")); lcd.println(timeExtract.millis);
        //
        //          lcd.setCursor(0, 1);
        //          lcd.print(pF(&dayStrs[timeExtract.wday])); lcd.print(F(", ")); lcd.print(pF(&monthStrs[timeExtract.mon]));
        //          lcd.print(F(" ")); lcd.print(timeExtract.mday); lcd.print(F(", ")); lcd.println(timeExtract.year);
        //          lcd.print(F("Day of year: ")); lcd.println(timeExtract.yday + 1);
        //
        //          lcd.print(F("GIVE POTS-ID:     ")); //potId on LCD
        //
        //        }
        break;
      }
  }
  //} END OF if (flag == 0)
}
////********************************************************************
////**************  SET-UP OF POTS and SENSORS  ************************
////********************************************************************
//
//int setUpPotsAndSensors(int pot_ID, int potCount, int maxPotCount, int sensorCount, int maxsensorCount, byte sensorConfig[][6], int sensorTypeCount) {
//  // Select the method of input the controls
//
//  Serial.println(F("  ENTER NUMBER OF POTS IN THE GREENHOUSE, PLEASE:"));
//  lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
//  lcd.print(F("ENTER NUMBER OF  "));
//  lcd.setCursor(0, 1);
//  lcd.print(F("POTS, PLEASE     "));
//
//  potCount = 100; //Just for while-loop entry
//  while (potCount > maxPotCount) {
//    char key =  keypad.waitForKey();
//    //key = keypad.getKey();
//    potCount = key - '0';
//    //    long potCnt = key;
//    //    potCount = (int)potCnt;
//
//    if (potCount > maxPotCount) {
//      if ((sensorCount == '#') ||  (sensorCount == '*')) {
//        Serial.print(F(" SORRY; ENTRY MUST BE A NUMBER"));
//        Serial.println(maxsensorCount);
//        lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
//        lcd.print(F("MUST BE A NUMBER")); //potId on LCD
//        lcd.setCursor(0, 1);
//        lcd.print(F("                ")); //potId on LCD
//      }
//      Serial.print(F(" SORRY; BUT MAKSIMUM NUMBER OF POTS IS:"));
//      Serial.println(maxPotCount);
//      lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
//      lcd.print(F("MAKSIMUM NUMBER  "));
//      lcd.setCursor(0, 1);
//      lcd.print(F("OF POTS IS  "));
//      //lcd.setCursor(10, 1);
//      lcd.print( maxPotCount);
//      //delay (delay3);
//
//      Serial.println(F(" ENTER NUMBER OF POTS IN THE GREENHOUSE, PLEASE:"));
//      lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
//      lcd.print(F("ENTER NUMBER OF  "));
//      lcd.setCursor(0, 1);
//      lcd.print(F("POTS, PLEASE     "));
//
//    }
//  } //END OF POTS COUNT WHILE
//  Serial.print(F(" NUMBER OF POTS IN THE GREENHOUSE IS: "));
//  Serial.println(potCount);
//  lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
//  lcd.print(F("POTS IN G-HOUSE "));
//  lcd.setCursor(0, 1);
//  lcd.print(potCount);
//  //delay(delay3);
//
//
//  //Serial.println(F(" "));
//  // Select the method of input the controls
//  Serial.println(F("  ENTER THE NUMBER OF THE SENSORS , PLEASE:"));
//  lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
//  lcd.print(F("ENTER NUMBER OF  ")); //potId on LCD
//  lcd.setCursor(0, 1);
//  lcd.print(F("SENSORS, PLEASE  ")); //potId on LCD
//
//
//  sensorCount = 100; //Just for while-loop entry
//  while (sensorCount > maxsensorCount) {
//    char key =  keypad.waitForKey();
//    sensorCount = key - '0';
//    //key = keypad.getKey();
//    //    long sensorXCnt = key;
//    //    potCount = (int)sensorXCnt;
//
//    if (sensorCount > maxsensorCount) {
//      if ((sensorCount == '#') ||  (sensorCount == '*')) {
//        Serial.print(F(" SORRY; ENTRY MUST BE A NUMBER"));
//        Serial.println(maxsensorCount);
//        lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
//        lcd.print(F("MUST BE A NUMBER")); //potId on LCD
//        lcd.setCursor(0, 1);
//        lcd.print(F("                ")); //potId on LCD
//      }
//      Serial.print(F(" SORRY; BUT MAKSIMUM NUMBER OF SENSORS IS:"));
//      Serial.println(maxsensorCount);
//      lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
//      lcd.print(F("MAKSIMUM NUMBER  ")); //potId on LCD
//      lcd.setCursor(0, 1);
//      lcd.print(F("OF SENSORS IS  ")); //potId on LCD
//      //lcd.setCursor(13, 1);
//      lcd.print( maxsensorCount); //potId on LCD
//      //delay (delay3);
//
//      Serial.println(F(" ENTER THE NUMBER OF THE SENSORS, PLEASE:"));
//      lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
//      lcd.print(F("ENTER NUMBER OF  ")); //potId on LCD
//      lcd.setCursor(0, 1);
//      lcd.print(F("SENSORS, PLEASE  "));
//    }
//  }
//
//  Serial.print(F(" NUMBER OF SENSORS IN GREENHOUSE IS: "));
//  Serial.println(potCount);
//  lcd.init();   lcd.backlight(); // INITIALIZE 16x2 CHARACTER LCD (I2C-display address: 0x3F / dec: 63 )
//  lcd.print(F("NBR OF SENSORS   "));
//  lcd.setCursor(0, 1);
//  lcd.print(sensorCount);
//delay(delay3);

//********************************************************************
//**************  CONFIGURE SENSOR  ************************
//********************************************************************

void configureSensor(int maxsensorCount, byte sensorConfig[][6], int notAllowedToEdit[], int flag) {
  //_________________________________________________________
  //3 sensors in two pots in the demo
  //[sensorPin][0]: 0 = no pot, 1 = pot_1, 2 = pot_2
  //[sensorPin][1]: 0 - 6 = pin number
  //[sensorPin][2]: 0 = photosensor, 1 = humiditysensor 2 = temperaturesensor
  //[sensorPin][3]: 0 = not measured, 1 = reported only by front-end, 2 = reported also by backend, 3 = also alarms sent to backend
  //[sensorPin][4]: 0 = beeper off, 1 = beeper on
  //[sensorPin][5]: 0 = interval_0, 1 = interval_1, 2 = interval_2

  //  /static byte sensorConfig[6][6] {
  //  {1, 0, 0, 3, 1, 1},
  //  {2, 1, 0, 3, 1, 1}
  //  {1, 2, 1, 3, 1, 1}
  //  {2, 3, 1, 3, 1, 1}
  //  {1, 4, 2, 3, 1, 1},
  //  {2, 5, 2, 3, 1, 1}
  //};
  //_____________________________________________________________

  //Serial.println();
  //printTimeStamp(secsSince1970);

  Serial.println(F(" ENTER SENSOR PIN-NUMBER, PLEASE !"));
  lcd.init();   lcd.backlight();
  lcd.print(F("CONFIGURE SENSOR"));
  lcd.setCursor(0, 1);
  lcd.print(F("ENTER SENSOR PIN"));

  char key =  keypad.waitForKey();
  sensorPin = key - '0';
  //key = keypad.getKey();

  while (!checkInputRange(sensorPin, maxsensorCount, notAllowedToEdit)) {

    Serial.println(F(" ENTER THE NUMBER OF THE SENSOR, PLEASE:"));
    lcd.init();   lcd.backlight();
    lcd.print(F("ENTER NUMBER OF  ")); //potId on LCD
    lcd.setCursor(0, 1);
    lcd.print(F("SENSOR, PLEASE  "));
    key =  keypad.waitForKey();
    sensorPin = key - '0';
    //key = keypad.getKey();
  }
  Serial.print(F("  SELECTED SENSOR PIN: "));
  Serial.println(sensorPin);
  lcd.init();   lcd.backlight();
  lcd.print(F("SENSOR PIN IS:  "));
  lcd.setCursor(0, 1);
  lcd.print(sensorPin);
  delay(delay6);
  sensorConfPrint(sensorPin, sensorConfig, flag);
  int parameterNumber;
  parameterNumber = editParam(sensorPin, sensorConfig, notAllowedToEdit);
  editValue(parameterNumber, sensorPin, sensorConfig, notAllowedToEdit);
  sensorConfPrint(sensorPin, sensorConfig, flag);
  flag = 0;

} // END OF configureSensors
//********************************************************************
void sensorConfPrint(int sensorPin, byte sensorConfig[][6], int flag) {

  Serial.println(F("  CURRENT SENSOR CONFIGURATION: "));
  Serial.println(F("  ---------------------------------------------------------------"));
  //3 sensors in two pots in the demo
  //[sensorPin][0]: 0 = no pot, 1 = pot_1, 2 = pot_2
  //[sensorPin][1]: 0 - 6 = pin number
  //[sensorPin][2]: 0 = photosensor, 1 = humiditysensor 2 = temperaturesensor
  //[sensorPin][3]: 0 = not measured, 1 = reported only by front-end, 2 = reported also by backend
  //[sensorPin][4]: 0 = beeper off, 1 = beeper on
  //[sensorPin][5]: 0 = interval_0, 1 = interval_1, 2 = interval_2

  int sensorP = sensorPin;
  if (flag == 1) {
    for ( sensorP = 0; sensorP < 6; sensorP++) {
      Serial.print(F("  "));
      Serial.print(sensorConfig[sensorP][0]);
      Serial.println(F(" | 0 = no pot, 1 = pot_1, 2 = pot_2"));
      Serial.print(F("  "));
      Serial.print(sensorConfig[sensorP][1]);
      Serial.println(F(" | 0 - 6 = pin number"));
      Serial.print(F("  "));
      Serial.print(sensorConfig[sensorP][2]);
      Serial.println(F(" | 0 = photosensor, 1 = humiditysensor 2 = temperaturesensor"));
      Serial.print(F("  "));
      Serial.print(sensorConfig[sensorP][3]);
      Serial.println(F(" | 0 = not measured, 1 = reported only by front-end, 2 = reported also by backend, 3 = also alarms sent to backend"));
      Serial.print(F("  "));
      Serial.print(sensorConfig[sensorP][4]);
      Serial.println(F(" | 0 = beeper off, 1 = beeper on"));
      Serial.print(F("  "));
      Serial.print(sensorConfig[sensorP][5]);
      Serial.println(F(" | 0 = interval_0, 1 = interval_1, 2 = interval_2"));
      Serial.println();
    }
  } else {
    Serial.print(F("  "));
    Serial.print(sensorConfig[sensorPin][0]);
    Serial.println(F(" | 0 = no pot, 1 = pot_1, 2 = pot_2"));
    Serial.print(F("  "));
    Serial.print(sensorConfig[sensorPin][1]);
    Serial.println(F(" | 0 - 6 = pin number"));
    Serial.print(F("  "));
    Serial.print(sensorConfig[sensorPin][2]);
    Serial.println(F(" | 0 = photosensor, 1 = humiditysensor 2 = temperaturesensor"));
    Serial.print(F("  "));
    Serial.print(sensorConfig[sensorPin][3]);
    Serial.println(F(" | 0 = not measured, 1 = reported only by front-end, 2 = reported also by backend, 3 = also alarms sent to backend"));
    Serial.print(F("  "));
    Serial.print(sensorConfig[sensorPin][4]);
    Serial.println(F(" | 0 = beeper off, 1 = beeper on"));
    Serial.print(F("  "));
    Serial.print(sensorConfig[sensorPin][5]);
    Serial.println(F(" | 0 = interval_0, 1 = interval_1, 2 = interval_2"));
    Serial.println();

    lcd.init();   lcd.backlight();
    lcd.print(F("SENSOR:"));
    lcd.setCursor(8, 0);
    lcd.print(sensorPin);
    lcd.setCursor(0, 1);
    for (int j = 0; j < 6; j++) {
      lcd.print(sensorConfig[sensorPin][j]);
      lcd.print(", ");
    }
  }

  flag = 0;
  delay(delay5);
}
//************************************************************
int editParam(int sensorPin, byte sensorConfig[][6], int notAllowedToEdit[6]) {
  int maxParamNumber = 4;
  int notAllowedToEdit_1[6] = {1, 1, 1, 0, 0, 1}; // 1 = not allowed to edit
  int maxValue = 4;


  //Serial.println();
  Serial.println(F(" SELECT PARAMETER TO EDIT (par 3 or 4)"));
  lcd.init();   lcd.backlight();
  lcd.print(F("SELECT PARAMETER")); //potId on LCD
  lcd.setCursor(0, 1);
  lcd.print(F("3 OR 4         ")); //potId on LCD

  char key =  keypad.waitForKey();
  int paramNumber = key - '0';
  //key = keypad.getKey();

  while (!checkInputRange(paramNumber, maxParamNumber, notAllowedToEdit_1)) {

    Serial.println(F(" SELECT PARAMETER TO EDIT"));
    lcd.init();   lcd.backlight();
    lcd.print(F("SELECT PARAMETER")); //potId on LCD
    lcd.setCursor(0, 1);
    lcd.print(F("                ")); //potId on LCD

    key =  keypad.waitForKey();
    paramNumber = key - '0';
    //key = keypad.getKey();
  }
  Serial.print(F("  PARAMETER SELECTED TO EDIT IS: "));
  Serial.println(paramNumber);
  lcd.init();   lcd.backlight();
  lcd.print(F("PARAMETER IS:   "));
  lcd.setCursor(0, 1);
  lcd.print(paramNumber);
  delay(delay6);

  return paramNumber;
}

//************************************************************
void editValue(int paramNumber, int sensorPin, byte sensorConfig[][6], int notAllowedToEdit[]) {

  Serial.println();
  Serial.println(F(" INPUT VALUE, PLEASE !"));
  lcd.init();   lcd.backlight();
  lcd.print(F("INPUT VALUE     ")); //potId on LCD
  lcd.setCursor(0, 1);
  lcd.print(F("                ")); //potId on LCD

  char key =  keypad.waitForKey();
  int value = key - '0';
  //key = keypad.getKey();
  int maxValue = 4;
  while (!checkInputRange(value , maxValue, notAllowedToEdit )) {

    Serial.println();
    Serial.println(F(" INPUT PARAMETER VALUE, PLEASE !"));
    lcd.init();   lcd.backlight();
    lcd.print(F("INPUT PAR. VALUE")); //potId on LCD
    lcd.setCursor(0, 1);
    lcd.print(F("                ")); //potId on LCD

    char key =  keypad.waitForKey();
    int value = key - '0';
    //key = keypad.getKey();
  }
  Serial.print(F("  PARAMETER VALUE IS: "));
  Serial.println(value);
  lcd.init();   lcd.backlight();
  lcd.print(F("PARAM VALUE IS: "));
  lcd.setCursor(0, 1);
  lcd.print(value);
  delay(delay6);

  // UPDATE SENSOR CONFIGURATION ! *****************************
  sensorConfig[sensorPin][paramNumber] = value;

  Serial.print(F("  SENSOR CONFIGURATION UPDATED ! "));
  //Serial.println(value);
  lcd.init();   lcd.backlight();
  lcd.print(F("CONFIG UPDATED !"));
  lcd.setCursor(0, 1);
  lcd.print(value);
  delay(delay6);
}
//****************************************************
boolean checkInputRange(int input, int maxValue, int notAllowedToEdit[]) {
  //if ((input > maxValue) || (input < 0)) {
  if ((input > maxValue)) {
    if ((input == '#') ||  (input == '*')) {
      Serial.print(F(" ENTRY MUST BE A NUMBER"));
      lcd.init();   lcd.backlight();
      lcd.print(F("MUST BE A NUMBER")); //potId on LCD
      lcd.setCursor(0, 1);
      lcd.print(F("                ")); //potId on LCD
      return false;
    }
    Serial.print(F(" SORRY, BUT MAXIMUM VALUE OF INPUT IS: "));
    Serial.println(maxValue);
    lcd.init();   lcd.backlight();
    lcd.print(F("MAXIMUM VALUE   ")); //potId on LCD
    lcd.setCursor(0, 1);
    lcd.print(F("OF INPUT IS     ")); //potId on LCD
    lcd.setCursor(12, 1);
    lcd.print( maxValue); //potId on LCD
    delay (delay6);
    return false;
  }
  if ((notAllowedToEdit[input]) == 1) {
    Serial.println(F(" SORRY, BUT PARAMETER IS NOT EDITABLE:"));
    Serial.println("  Editable parameters are: 3 and 4");
    lcd.init();   lcd.backlight();
    lcd.print(F("PAR NOT EDITABLE")); //potId on LCD
    lcd.setCursor(0, 1);
    lcd.print(F("ENTER PAR 3 OR 4")); //potId on LCD
    delay (delay5);
    return false;
  }
  return true;
}
//***************************************************
//void digitalClockDisplay() {
//  // digital clock display of the time
//  Serial.print(hour());
//  printDigits(minute());
//  printDigits(second());
//  Serial.print(" ");
//  Serial.print(day());
//  Serial.print(" ");
//  Serial.print(month());
//  Serial.print(" ");
//  Serial.print(year());
//  Serial.println();
//}
//
//void printDigits(int digits) {
//  // utility function for digital clock display: prints preceding colon and leading 0
//  Serial.print(":");
//  if (digits < 10)
//    Serial.print('0');
//  Serial.print(digits);
//}

