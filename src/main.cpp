#include <Arduino.h>
// #include <vector>
#include <esp_sleep.h>
#include <esp_system.h> //esp_reset_reason()
#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>
#include "SparkFunBME280.h"
#include "Wire.h"
#include <Preferences.h> //Flash memory library

#define SLEEP_TIME 60 // 172.8 // seconds (500samples = 24h)
#define SENSORNUM 2

#define HELPER(x) #x
#define STR(x) HELPER(x)

// Humidity calibration values (add the cal value to the measured value)
#if SENSORNUM == 1
#define NAME GumbaTHP1
//#define HUMIDITY_CAL (75.0 - 74.93657895)
#define USE_BME280_PINS_5678 // beware, GPIO8 is connected to LED and GPIO9 to the Boot button
#elif SENSORNUM == 2
#define NAME GumbaTHP2
//#define HUMIDITY_CAL (75.0 - 82.3172973)
#define USE_BME280_PINS_5678 // beware, GPIO8 is connected to LED and GPIO9 to the Boot button
#elif SENSORNUM == 3
#define NAME GumbaTHP3
//#define HUMIDITY_CAL 0
//(75.0 - 80.25918919)
#define USE_BME280_PINS_8765 // beware, GPIO8 is connected to LED and GPIO9 to the Boot button
#elif
#define NAME GumbaTHP
//#define HUMIDITY_CAL 0.0
#define USE_BME280_PINS_0123
#endif

// Global variables
// RTC_DATA_ATTR int bootCount = 0;
float humidityCal = 0.0;
float tempCal = 0.0;
float pressureCal = 0.0;
float batt = 0.0;


// put function declarations here:
// void wifiConnect();
// void mqttConnect();
// void lcdInit();
void setupBME280();
void loop();
void blink(int times, int onTime, int offTime);
void callback(char* topic, byte* payload, unsigned int length);


//Flash memory
Preferences preferences;

// WiFi
WiFiClient wifiClient;
struct WIFICREDENTIALS
{
  const char *ssid;
  const char *password;
};

// std::vector<WIFICREDENTIALS> wifiCredentials = {
//   {"WINGNET", "2136044570448027"},
//   {"WingTetherNet", "Ingo.Ingo"},
//   {"StradaDelOveste", "Voggu.Rischl.2018"},
//   {"TeraGuest", "We Measure Quality"}
// };
const char *ssid = "WINGNET";
const char *password = "2136044570448027";
// const char *ssid = "WingTetherNet";//"WINGNET";
// const char *password = "Ingo.Ingo";//"2136044570448027";
// const char* ssid = "StradaDelOveste";
// const char* password = "Voggu.Rischl.2018";
// const char *ssid = "TeraGuest";
// const char *password = "We Measure Quality";

// MQTT Broker
PubSubClient mqttClient(wifiClient);
const char *mqtt_server = "broker.hivemq.com"; //"test.mosquitto.org";
const int mqtt_port = 1883;
// const char* mqttTopic = "gumba/temp";
// const char* mqttuser = "freddyspencer@gmx.at";
// const char* mqttpassword = "MQTTSpencer#";

// NTP Time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "de.pool.ntp.org"); //, 0, 1000); // UTC time with a 1-second update interval
// String weekDays[7]={"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
// String months[12]={"January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December"};
String weekDays[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
String months[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

// Reset Reason Strings
String resetReasons[] = {"Unknown", "Power On", "External", "Software", "Panic", "Interrupt Watchdog", "Task Watchdog", "Other Watchdog", "Deep Sleep", "Brownout", "SDIO"};
// ESP_RST_UNKNOWN,    //!< Reset reason can not be determined
// ESP_RST_POWERON,    //!< Reset due to power-on event
// ESP_RST_EXT,        //!< Reset by external pin (not applicable for ESP32)
// ESP_RST_SW,         //!< Software reset via esp_restart
// ESP_RST_PANIC,      //!< Software reset due to exception/panic
// ESP_RST_INT_WDT,    //!< Reset (software or hardware) due to interrupt watchdog
// ESP_RST_TASK_WDT,   //!< Reset due to task watchdog
// ESP_RST_WDT,        //!< Reset due to other watchdogs
// ESP_RST_DEEPSLEEP,  //!< Reset after exiting deep sleep mode
// ESP_RST_BROWNOUT,   //!< Brownout reset (software or hardware)
// ESP_RST_SDIO,       //!< Reset over SDIO

// BME280 Sensor
BME280 bme280;
// if using GPIO 8 or 9 for BME280 note that GPIO8 is connected to LED and GPIO9 to the Boot button
#ifdef USE_BME280_PINS_5678
#define BME_SDA 5
#define BME_SCL 6
#define BME_GND 7
#define BME_PWR 8
#endif

#ifdef USE_BME280_PINS_8765
#define BME_SDA 8
#define BME_SCL 7
#define BME_GND 6
#define BME_PWR 5
#endif

#ifdef USE_BME280_PINS_0123
#define BME_PWR 0
#define BME_SCL 1
#define BME_SDA 2
#define BME_GND 3
#endif

// #define BME_SDA 9
// #define BME_SCL 10
// #define BME_GND 20
// #define BME_PWR 21

void setup()
{
  // read microseconds since boot
  unsigned long micros = esp_timer_get_time();
  Serial.begin(115200);

  pinMode(BME_GND, INPUT); // remove BME280 GND connection to leave it off while toggling LED (in case BME is powered from GPIO8)
  //digitalWrite(LED, LOW);  // turn LED on
  //pinMode(LED, OUTPUT);

  if (esp_reset_reason() != ESP_RST_DEEPSLEEP)
  {
    Serial.println("  *** Hello Gumba! ***  ");
    blink(3, 500, 500);
    delay(1000);
  }

#ifdef SLEEP_TIME
  loop(); // call loop once if we go to deep sleep
  mqttClient.disconnect();
  WiFi.disconnect();
  Serial.println("Going to deep sleep...");
  Serial.flush();

  // setup deep sleep for lowest power consumption
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);    // allow wake up from timer
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF); // no RTC slow memory (to achieve 5uA deep sleep current)
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_OFF);

  long sleepTime = SLEEP_TIME * 1E6 - (esp_timer_get_time() - micros);
  if (sleepTime < 0) // if we are late, sleep for 1 second
    sleepTime = SLEEP_TIME * 1E6;
  esp_sleep_enable_timer_wakeup(sleepTime); // sleep for SLEEP_TIME seconds
  esp_deep_sleep_start();
  // //Serial.println("Deep_Sleep skipped - This will normallywise never be printed");
#endif
}


void loop()
{

  // Make sensor measurements now when sensor and controller are still cold after wakeup
  // will be set in setupBME280: digitalWrite(LED, HIGH); // turn LED off  //#undef LED // Make sure to leave LED OFF while measuring if GPIO8 is used for BME280
  setupBME280(); // startup sensor and let it settle
  delay(100);
  BME280_SensorMeasurements measurements;
  bme280.readAllMeasurements(&measurements, 0);
  digitalWrite(BME_PWR, LOW); // turn BME280 off (this may turn the LED on!)
  pinMode(BME_GND, INPUT);    // remove BME280 GND connection to leave it off while toggling LED (in case BME is powered from GPIO8)



  // read battery voltage
  // (connect 180k ohms from battery to ADC pin and turn on pulldown resistor (45k) --> 5V --> )
  
  ///how? analogReference(AR_INTERNAL); // set reference voltage to 1.1V
  analogReadResolution(12);      // set adc resolution to 12bit arduino
  analogSetAttenuation(ADC_11db); // ADC_0db 0dB attenuation gives full-scale voltage 0-1.1V // For full-scale voltage 0-3.3V, use ADC_11db
  //adcAttachPin(0);               // attach ADC pin to GPIO0

  //looks like the internal pulldown is inactive when using the ADC on the same pin --> use bridge to GPIO1 and enable internal pulldown on GPIO1
  // use 2x 180k divider and ground low end through GPIO1 (force L)

  pinMode(0,INPUT); //init HiZ
  digitalWrite(1, LOW); //drive low level to ground the low end of the voltage divider
  pinMode(1, OUTPUT); //drive low level to ground the low end of the voltage divider
  //pinMode(0,INPUT_PULLDOWN); //enable internal 45k pulldown resistor
  
  delay(1000);
  for (int i=0;i<10;i++){
  delay(100);
  /*float*/ batt  = analogRead(0) *3.3 / 4095.0  ;//*((180.0+45.0)/45.0);//* 3.3; //* 2; // 2x voltage divider
  Serial.printf("Battery(%d): %fV\n", i, batt);
  }

  pinMode(0,INPUT);
  pinMode(1,INPUT);

  // read ESP32-C3 internal temperature sensor





  //get cal values stored in flash memory (do it while WiFi is not connected to minimize peak current draw from battery) (read should be minimal but better here)
  //call mqttloop to poll if there was a calibration value set (this will write to flash memory if there is a cal message) - and this will draw ~100mA
  //to cal post a retained message to mqtt: mosquitto_pub.exe' -h broker.hivemq.com -p 1883 -t GumbaTHP3/SetTempCal -m -1.234567 -r
  // & 'C:\Program Files\mosquitto\mosquitto_pub.exe' -h broker.hivemq.com -p 1883 -t GumbaTHP3/SetTempCal -m 0.7 -r
  // & 'C:\Program Files\mosquitto\mosquitto_pub.exe' -h broker.hivemq.com -p 1883 -t GumbaTHP3/SetHumidityCal -m -0.8 -r
  // & 'C:\Program Files\mosquitto\mosquitto_pub.exe' -h broker.hivemq.com -p 1883 -t GumbaTHP3/SetPressureCal -m 0.9 -r
  Serial.println("Reading calibration values from flash memory...");
  preferences.begin(STR(NAME), false);
  tempCal = preferences.getFloat("TempCal", 0.0);
  humidityCal = preferences.getFloat("HumidityCal", 0.0);
  pressureCal = preferences.getFloat("PressureCal", 0.0);
  preferences.end();
  Serial.printf("  %f\n  %f\n  %f\n", tempCal, humidityCal, pressureCal);


  // Connect to WiFI
  if (WiFi.status() != WL_CONNECTED)
  {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
    //  wait 20 times for connection (10sec max)
    for (int i = 0; i < 20; i++)
    {
      if (WiFi.status() == WL_CONNECTED)
        break;
      // toggle LED while waiting for WiFi
      digitalWrite(LED, digitalRead(LED) ^ 1);
      delay(500);
      Serial.print(WiFi.status());
    }
    digitalWrite(LED, HIGH); // turn LED off
    Serial.println();
  }

  if (WiFi.status() == WL_CONNECTED)
  {
  //Connect to MQTT
    if (!mqttClient.connected())
    {
      Serial.print("Connecting ");
      Serial.print(STR(NAME));
      Serial.println(" to MQTT...");
      mqttClient.disconnect();
      mqttClient.setServer(mqtt_server, mqtt_port);
      mqttClient.setCallback(callback);
      mqttClient.connect(STR(NAME), NULL, NULL);

      Serial.println("Subscribing to MQTT topics...");
      Serial.printf("  %s\n",STR(NAME/SetTempCal));
      Serial.printf("  %s\n",STR(NAME/SetHumidityCal));
      Serial.printf("  %s\n",STR(NAME/SetPressureCal));

      mqttClient.subscribe(STR(NAME/Status));
      mqttClient.subscribe(STR(NAME/SetTempCal));
      mqttClient.subscribe(STR(NAME/SetHumidityCal));
      mqttClient.subscribe(STR(NAME/SetPressureCal));  
    }

    if (!mqttClient.connected()) // Error-blink 'MQTT still not connected'
    {
      Serial.println(">>> MQTT connection FAILED!");
      delay(500);
      blink(4, 200, 200); // blink error for no connection to MQTT but continue
    }
    else // Check for MQTT messages
    {
      Serial.print("Checking for MQTT Messages: ");
      for (int i = 0; i < 5; i++)
      {
        Serial.printf("%d, ", i);
        mqttClient.loop();
        delay(100);
      }
      Serial.println("done.");
    }
  }
  else // Error-blink 'WiFi not connected'
  {
    Serial.println(">>> WiFi connection FAILED!");
    delay(500);
    blink(3, 50, 200);
  }


  // Get Time and Date
  String timestamp;
  String timeAndDate; // used to print on lcd or Terminal
  char buf[25];
  if (WiFi.status() == WL_CONNECTED)
  {
    // Print time and date
    timeClient.begin();
    timeClient.setTimeOffset(7200); // MEZ +2h
    timeClient.update();
    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime((time_t *)&epochTime); // Get a time structure // int monthDay = ptm->tm_mday;
    snprintf(buf, sizeof(buf), " %3s, %2d %3s ", weekDays[timeClient.getDay()].c_str(), ptm->tm_mday, months[ptm->tm_mon].c_str());
    timeAndDate = buf;
    snprintf(buf, sizeof(buf), " %02d:%02d:%02d ", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    timeAndDate += buf;
    snprintf(buf, sizeof(buf), "%4d-%02d-%02d %02d:%02d:%02d", ptm->tm_year + 1900, ptm->tm_mon, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    timestamp = buf;
  }
  else
  {
    // Get time from internal RTC
    struct tm timeinfo;
    if (getLocalTime(&timeinfo))
    {
      snprintf(buf, sizeof(buf), " %3s, %2d %3s ", weekDays[timeinfo.tm_wday].c_str(), timeinfo.tm_mday, months[timeinfo.tm_mon + 1].c_str());
      timeAndDate = buf;
      snprintf(buf, sizeof(buf), "~%02d:%02d:%02d ", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      timeAndDate += buf;

      snprintf(buf, sizeof(buf), "%4d-%02d-%02d %02d:%02d:%02d", timeinfo.tm_year + 1900, timeinfo.tm_mon, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      timestamp = buf;
    }
    else
    {
      timestamp = "No time available.";
      timeAndDate = "No time available.";
    }
  }


  // Post to MQTT
  if (mqttClient.connected())
  {
    mqttClient.publish(STR(NAME/Temp), String(measurements.temperature+tempCal).c_str());
    delay(100);
    mqttClient.publish(STR(NAME/Humidity), String(measurements.humidity + humidityCal).c_str());
    delay(100);
    mqttClient.publish(STR(NAME/Pressure), String((measurements.pressure / 100)+pressureCal).c_str()); // /100 to converst Pa to mbar
    delay(100);
    mqttClient.publish(STR(NAME/Timestamp), timestamp.c_str());
    delay(100);
    mqttClient.publish(STR(NAME/Batt), String(batt).c_str());  
    //if (esp_reset_reason() != ESP_RST_DEEPSLEEP)
    {
      //mqttClient.publish(STR(NAME/Status), (String(STR(NAME has been started. Reset reason: )) + resetReasons[esp_reset_reason()] + " (" + String(esp_reset_reason()) + ").").c_str());
      mqttClient.publish(STR(NAME/Status), (String(STR(NAME started: )) + resetReasons[esp_reset_reason()] 
      + " (" + String(esp_reset_reason()) 
      + ") \nTcal="+String(tempCal)
      +"\nHcal="+String(humidityCal)
      +"\nPcal="+String(pressureCal)
      +"\nBatt="+String(batt,4) ).c_str());
      delay(100);
    }
    Serial.println("Posted to MQTT:");
    Serial.println("  "+String(STR(NAME/Temp:))  + String(measurements.temperature+tempCal));
    Serial.println("  "+String(STR(NAME/Humidity:))  + String(measurements.humidity+humidityCal));
    Serial.println("  "+String(STR(NAME/Pressure:))  + String((measurements.pressure / 100)+pressureCal) ); // /100 to converst Pa to mbar
    Serial.println("  "+String(STR(NAME/Timestamp:)) + timestamp);
    Serial.println("  "+String(STR(NAME/Batt:)) + String(batt,4));
    Serial.println("  "+String(STR(NAME/Status:)) + (String(STR(NAME has been started. Reset reason: )) + resetReasons[esp_reset_reason()] 
    +" (" + String(esp_reset_reason()) 
    +") \n    Tcal="+String(tempCal)
    +"\n    Hcal="+String(humidityCal)
    +"\n    Pcal="+String(pressureCal) 
    +"\n    Batt="+String(batt,4) ).c_str());
  }
  else //Error-blink 'No MQTT connection!'
  {
    Serial.println(">>> No MQTT connection...");
    blink(2, 50, 100);
    delay(250);
    blink(2, 50, 100);
    delay(250);
    blink(2, 50, 100);
    delay(250);
  }

  Serial.flush();
  // digitalWrite(LED, HIGH); // turn LED off

#ifndef SLEEP_TIME
  delay(1000);
#endif
}

void setupBME280()
{

  // BME280 sensor setup
  digitalWrite(BME_GND, LOW);
  digitalWrite(BME_PWR, LOW); // needed to reset sensor when starting up
  pinMode(BME_GND, OUTPUT);
  pinMode(BME_PWR, OUTPUT);
  delay(10); // Startup time for BME280: 2ms (spec)

  digitalWrite(BME_PWR, HIGH);
  Wire.begin(BME_SDA, BME_SCL);

  bme280.setI2CAddress(0x76); // The default for the bme module is 0x76
  if (!bme280.beginI2C())
  { // ERROR: BME280 not found
    WiFi.disconnect();
    mqttClient.disconnect();
    while (1)
    {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      //  1 dah = 3*dit, 1 dit between tones, 3 dit after a letter, 7 dit between words
      const int dit = 100;
      const int dah = 3 * dit; // 3*dit
      blink(3, dah, dit);      // - - -
      delay(2 * dit);          // 3 dit break between letters
      blink(3, dit, dit);      // . . .
      delay(2 * dit);          // 3 dit break between letters
      blink(3, dah, dit);      // - - -
      delay(7 * dit);          // 7 dit break between words
      delay(500);
    };
  }

  bme280.setReferencePressure(101325); // SEA_LEVEL_PRESSURE default is 1013.25 hPa
  bme280.setTemperatureCorrection(0);  // no correction
  // uint bme280.readRegister( BME280_CHIP_ID_REG);
}

void blink(int times, int onTime, int offTime)
{
  for (int i = 0; i < times; i++)
  {
    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW); // turn LED on
    delay(onTime);
    digitalWrite(LED, HIGH); // turn LED off
    delay(offTime);
  }
}


void callback(char* topic, byte* payload, unsigned int length) {
  char buf[length+1];
  char topicbuf[100];
  char printbuf[100];
  
  for(int i=0; i<length; i++) { buf[i] = payload[i];}
  // memcpy(buf, payload, length);
   buf[length] = '\0';
  strncpy(topicbuf, topic, 99);

  snprintf(printbuf, 99, "Callback called[%s]:%s\0\0", topic, buf);
  Serial.println(printbuf);
  mqttClient.publish(STR(NAME/Status), printbuf);

  Serial.println("Checking...");
  Serial.println(STR(NAME/SetTempCal));
  Serial.println(topic); //doesn't print the topic string! (output:"0>")
  Serial.println(topicbuf);//this works

  // Compare if topic is NAME/SetTempCal
  if (strcmp(topicbuf, STR(NAME/SetTempCal)) == 0) {
    tempCal = atof(buf);
    //write to flash memory
    preferences.begin(STR(NAME), false);
    preferences.putFloat("TempCal", tempCal);
    preferences.end();
    Serial.println("Temperature calibration set to: " + String(tempCal));
    mqttClient.publish(STR(NAME/Status), (String(STR(NAME/TempCal set to: )) + String(tempCal)).c_str());
    mqttClient.publish(STR(NAME/SetTempCal), "", true); //clear retained cal message
  }
  // compare if topic is NAME/SetHumidityCal
  //if (strcmp(topic, STR(NAME/SetHumidityCal)) == 0) {
  if (strcmp(topicbuf, STR(NAME/SetHumidityCal)) == 0) {    
    humidityCal = atof(buf);
    //write to flash memory
    preferences.begin(STR(NAME), false);
    preferences.putFloat("HumidityCal", humidityCal);
    preferences.end();
    Serial.println("Humidity calibration set to: " + String(humidityCal));
    mqttClient.publish(STR(NAME/Status), (String(STR(NAME/HumidityCal set to: )) + String(humidityCal)).c_str());
    mqttClient.publish(STR(NAME/SetHumidityCal), "", true); //clear retained cal message
  }
  // if (strcmp(topic, String(STR(NAME/SetPressureCal)).c_str()) == 0) {
  if (strcmp(topicbuf, STR(NAME/SetPressureCal)) == 0) {
    pressureCal = atof(buf);
    //write to flash memory
    preferences.begin(STR(NAME), false);
    preferences.putFloat("PressureCal", pressureCal);
    preferences.end();
    Serial.println("Pressure calibration set to: " + String(pressureCal));
    mqttClient.publish(STR(NAME/Status), (String(STR(NAME/PressureCal set to: )) + String(pressureCal)).c_str());
    mqttClient.publish(STR(NAME/SetPressureCal), "", true); //clear retained cal message
  }

}