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



#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
// created from https://eu-central-1-1.aws.cloud2.influxdata.com/orgs/c77989a2f9f581e2/new-user-setup/arduino
#define INFLUXDB_URL "https://eu-central-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "r1LYXVQWqVJh-xAcvCyGdu8295im3fkEa_5Oru5fRtS6LvMd4cUJ-wcPIglB_d7xKDPtsdffkNeFgIBXkMpFrg=="
#define INFLUXDB_ORG "c77989a2f9f581e2"
#define INFLUXDB_BUCKET "HrbrBucket"
// InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
// Declare Data point
Point point("HrbrTHP");

#pragma region Constants and Globals
#define SLEEP_TIME 60 // 172.8 // seconds (500samples = 24h)
#define SENSORNUM 5
// sensornum 3 =HrbrTHP1 (Wohnzimmer)
// sensornum 4 =HrbrTHP2 (WC_oben)
// sensornum 5 =HrbrTHP3 (WC_unten)

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
//#define NAME GumbaTHP3
#define NAME HrbrTHP1
#define ROOM Wohnzimmer
//#define HUMIDITY_CAL 0
//(75.0 - 80.25918919)
#define USE_BME280_PINS_8765 // beware, GPIO8 is connected to LED and GPIO9 to the Boot button
#elif SENSORNUM == 4
#define NAME HrbrTHP2
#define ROOM WC_oben
//#define HUMIDITY_CAL 0
//(75.0 - 80.25918919)
#define USE_BME280_PINS_5678 // beware, GPIO8 is connected to LED and GPIO9 to the Boot button
#elif SENSORNUM == 5
#define NAME HrbrTHP3
#define ROOM WC_unten
//#define HUMIDITY_CAL 0
//(75.0 - tbd)
#define USE_BME280_PINS_5678 // beware, GPIO8 is connected to LED and GPIO9 to the Boot button
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
volatile bool incomingMsgReceived = false;
void hexDump(const void* data, uint32_t length);


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
// const char *ssid = "WINGNET";
// const char *password = "2136044570448027";
// const char *ssid = "WingTetherNet";//"WINGNET";
// const char *password = "Ingo.Ingo";//"2136044570448027";
const char* ssid = "StradaDelOveste";
const char* password = "Voggu.Rischl.2018";
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
#pragma endregion


#pragma region History buffer
  // receive and send to and from this buffer
  //
  //Keep it simple for now: post tuples of epoch time(32Bit), float value(32Bit) = 8 bytes per value 
  // (1msg/min * 60min/h * 24h/d * 365d/y = 4,204,800 Bytes)
  // (1msg/min * 60min/h * 24h/d * 30d/month * 8 bytes/msg = 345,600)
  // 1 week = 60*24*7 = 10,080
  //
  // limit of HiveMQ (Google Gemini)
  // - Maximum message size: 5 MB per message.
  // - 10 GB data traffic per month in total (both directions).
  // (this limits to 5 MB per single message.)
  //
  // Theoretical Transfer times:
  //  1 Mbit/s upstream connection → ~ 40 seconds (5 MB = 40 Mbit; 40 Mbit / 1 Mbit/s = 40s).
  //  10 Mbit/s upstream → ~ 4 seconds.
  //  100 Mbit/s upstream → ~ 0.4 s.

  typedef struct __attribute__((packed)) TempHistorySTRUCT{ //__attribute__ to avoid stuffing in case this will be compiled on a non 32Bit system
    unsigned long epochTime; //unsigned long (32Bit=4Byte): absolute start time in unix time
    float value; // single precision floating point (32Bit)
  }TempHistorySTRUCT;
  const unsigned int MAXMEASUREMENTS = 25;//60*24*2; //20;//60*24*7; //reserve data for 1 week = 10,080 measurements
  union{
    volatile TempHistorySTRUCT Temp[MAXMEASUREMENTS]={0};
    uint8_t TempByteBuffer[MAXMEASUREMENTS*sizeof(TempHistorySTRUCT)];//[sizeof(Temp)];        //maps byte array over array of structs
  } History;
  volatile int numMeas=-1;

#pragma endregion History buffer



void setup()
{
  // read microseconds since boot
  unsigned long micros = esp_timer_get_time();
  //done before polling// numMeasReceived=-1;//also reset if not coming back from reset deep sleep (some sleep mode retains global variables - this happened when ESP_PD_DOMAIN_RTC_SLOW_MEM=ESP_PD_OPTION_OFF (only ESP_PD_DOMAIN_RTC_PERIPH=ESP_PD_OPTION_ON))
  Serial.begin(115200); //266092 bytes free.

  pinMode(BME_GND, INPUT); // remove BME280 GND connection to leave it off while toggling LED (in case BME is powered from GPIO8)
  //digitalWrite(LED, LOW);  // turn LED on
  //pinMode(LED, OUTPUT);

  if (esp_reset_reason() != ESP_RST_DEEPSLEEP)
  {
    Serial.println("  *** Hello Gumba! ***  ");
    Serial.printf( "I am: %s\n",STR(NAME) );
    Serial.printf("%d bytes free.\n",ESP.getFreeHeap());
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


#ifdef READBATTERY
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

#endif //READBATTERY


  // read ESP32-C3 internal temperature sensor
  // to do...




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

  //Connect to MQTT
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!mqttClient.connected()) //connect to MQTT and Subscribe to topics
    {
      Serial.print("Connecting ");
      Serial.print(STR(NAME));
      Serial.println(" to MQTT...");
      mqttClient.disconnect();
      mqttClient.setServer(mqtt_server, mqtt_port);
      mqttClient.setCallback(callback);
      mqttClient.connect(STR(NAME), NULL, NULL);

      Serial.println("Subscribing to MQTT topics...");
      Serial.printf("  %s\n",STR(NAME/GetTempHistory));
      Serial.printf("  %s\n",STR(NAME/SetTempCal));
      Serial.printf("  %s\n",STR(NAME/SetHumidityCal));
      Serial.printf("  %s\n",STR(NAME/SetPressureCal));

      //subscribe to History values
      mqttClient.subscribe(STR(NAME/GetTempHistory),1);


      //subscribe to Cal messages
      mqttClient.subscribe(STR(NAME/SetTempCal));
      mqttClient.subscribe(STR(NAME/SetHumidityCal));
      mqttClient.subscribe(STR(NAME/SetPressureCal));  
      //mqttClient.subscribe(STR(NAME/Status));

    }

    if (!mqttClient.connected()) // Error-blink 'MQTT still not connected'
    {
      Serial.println(">>> MQTT connection FAILED!");
      delay(500);
      blink(4, 200, 200); // blink error for no connection to MQTT but continue
    }
    else // Check for MQTT messages
    {
      numMeas=-1;
      Serial.print("Checking for MQTT Messages:\n");
      for (int i = 0; i < 10; i++)
      {
        incomingMsgReceived = false;
        Serial.printf("  %d:", i);
        mqttClient.loop();
        //Serial.print(incomingMsgReceived?"Msg received!\n":"No Msg.\n");
        if (incomingMsgReceived==false) Serial.print("No Msg.\n");
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
  /*time_t*/ unsigned long epochTime = 0;//0=ref time 1.1.1970 UTC //NTPClient.getEpochTime() returns number of seconds since 1.1.1970Z
  char buf[25];
  if (WiFi.status() == WL_CONNECTED)
  {
    // Print time and date
    timeClient.begin();
    timeClient.setTimeOffset(7200); // MEZ +2h
    timeClient.update();
    epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime((time_t *)&epochTime); // Get a time structure // int monthDay = ptm->tm_mday;
    snprintf(buf, sizeof(buf), " %3s, %2d %3s ", weekDays[timeClient.getDay()].c_str(), ptm->tm_mday, months[ptm->tm_mon].c_str());
    timeAndDate = buf;
    snprintf(buf, sizeof(buf), " %02d:%02d:%02d ", ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    timeAndDate += buf;
    snprintf(buf, sizeof(buf), "%4d-%02d-%02d %02d:%02d:%02d", ptm->tm_year + 1900, ptm->tm_mon, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec);
    timestamp = buf;

    #pragma region InfluxDB
    //timeSync("UTC1","pool.ntp.org","time.nis.gov");
    ////Post measurements to InfluxDB
        // Check server connection
    if (client.validateConnection()) {
      Serial.print("Connected to InfluxDB: ");
      Serial.println(client.getServerUrl());

      point.addTag("sensor", STR(NAME));
      point.addTag("room", STR(ROOM));
      point.addTag("SSID", WiFi.SSID());

      point.clearFields(); //optional here, neede for running in loop() to reuse measurement point
      point.addField("Temperature",measurements.temperature + tempCal);
      point.addField("Humidity",measurements.humidity + humidityCal);
      point.addField("Pressure",(measurements.pressure / 100)+pressureCal);
      point.addField("RSSI",WiFi.RSSI());
      
      Serial.println("Sending to InFluxDB:");
      Serial.println(point.toLineProtocol());

      if(!client.writePoint(point)){
        Serial.print("InfluxDB write failed:");
        Serial.println(client.getLastErrorMessage());
      }
      else{
        Serial.println("InfluxDB write SUCCEEDED!");
      }


    } else {
      Serial.print("InfluxDB connection failed: ");
      Serial.println(client.getLastErrorMessage());
    }
    #pragma endregion InfluxDB

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
    mqttClient.publish(STR(NAME/Temp), String(measurements.temperature + tempCal).c_str());
    delay(100);
    mqttClient.publish(STR(NAME/Humidity), String(measurements.humidity + humidityCal).c_str());
    delay(100);
    mqttClient.publish(STR(NAME/Pressure), String((measurements.pressure / 100)+pressureCal).c_str()); // /100 to converst Pa to mbar
    delay(100);
    mqttClient.publish(STR(NAME/Timestamp), timestamp.c_str());
    #ifdef READBATTERY
    delay(100);
    mqttClient.publish(STR(NAME/Batt), String(batt).c_str());  
    #endif //READBATTERY

#pragma region Send New History
    //Update retained message for TempHistory
    // note: sizeof(History.Temp) returns the number of bytes that the Temp Struct array allocates
    // now use #define MAXMEASUREMENTS: size_t maxMeasurements = sizeof(History.Temp) / sizeof(History.Temp[0]);

    int numMeasCounted;//count number of tuples in TempStruct array and determine how many there are (now tracked by global numMeasReceived)
    for(numMeasCounted=0;numMeasCounted < MAXMEASUREMENTS && History.Temp[numMeasCounted].epochTime!=0;numMeasCounted++){}
    Serial.printf("Number of Measurements in History buffer: %d (counted: %d)\n",numMeas, numMeasCounted);
    Serial.printf("numMeasurementsReceived=%d (max:%d)\n",numMeas,MAXMEASUREMENTS);
    Serial.printf("Record length:%d\n",sizeof(TempHistorySTRUCT));
    
    if (numMeas >= 0)
    //numMeasReceived logic: only append a message if somethinggot received 
    // numMeasReceived =-1: no retained message received - cannot append - skip sending Histroy
    // numMeasReceived >=1 (>0): messages in History buffer - append current measurement and send updated retained message
    // numMeasReceived =0: a message with a "clear command" was received - start collecting  
    //                    (numMeasReceived =0 gets set in the callback when any message that is shorter than sizeof(HistoryStruct)(=2*8) bytes was received)
    {
      //if (numMeasReceived < MAXMEASUREMENTS - 1){
        History.Temp[numMeas].epochTime = epochTime;
        History.Temp[numMeas].value = measurements.temperature + tempCal;
        numMeas++;
        Serial.printf("\nNew measurement #%d added...\n",numMeas);
        //History.Temp[numMeasReceived+1].epochTime = 0;
        //History.Temp[numMeasReceived+1].value = 0;

      //}
      
      //send 
      unsigned long bytesToSend = numMeas * sizeof(TempHistorySTRUCT);

      //can't do this - this will end up in a loop with numMeas=-1
      // mqttClient.publish(STR(NAME/GetTempHistory), "", true);
      // for(int i=0;i<10;i++){
      //   mqttClient.loop();
      //   delay(100);
      // }

      mqttClient.publish(STR(NAME/GetTempHistory), (const uint8_t*)History.TempByteBuffer, bytesToSend, true);
     
      Serial.printf("History: %d of %d messages posted to MQTT (%d bytes sent):\n",numMeas,MAXMEASUREMENTS,bytesToSend);
      hexDump(History.TempByteBuffer,numMeas*sizeof(TempHistorySTRUCT));
    }
#pragma endregion Send New History

    //if (esp_reset_reason() != ESP_RST_DEEPSLEEP)
    //{
      //mqttClient.publish(STR(NAME/Status), (String(STR(NAME has been started. Reset reason: )) + resetReasons[esp_reset_reason()] + " (" + String(esp_reset_reason()) + ").").c_str());
      String statusMsg = (String(STR(NAME started: )) + resetReasons[esp_reset_reason()] 
      + " (" + String(esp_reset_reason()) 
      + ") \nTcal="+String(tempCal)
      +"\nHcal="+String(humidityCal)
      +"\nPcal="+String(pressureCal)
      +"\nBatt="+String(batt,4))
      +"\nMeas sent:"+String(numMeas);
      mqttClient.publish(STR(NAME/Status),statusMsg.c_str()); 
      delay(100);
      //}
      
      Serial.println("Posted to MQTT:");
      Serial.println("  "+String(STR(NAME/Temp:))  + String(measurements.temperature+tempCal));
      Serial.println("  "+String(STR(NAME/Humidity:))  + String(measurements.humidity+humidityCal));
      Serial.println("  "+String(STR(NAME/Pressure:))  + String((measurements.pressure / 100)+pressureCal) ); // /100 to converst Pa to mbar
      Serial.println("  "+String(STR(NAME/Timestamp:)) + timestamp);
    #ifdef READBATTERY
    Serial.println("  "+String(STR(NAME/Batt:)) + String(batt,4));
    #endif //READBATTERY
    Serial.printf("  %s\n",statusMsg.c_str());
    // Serial.println("  "+String(STR(NAME/Status:)) + (String(STR(NAME has been started. Reset reason: )) + resetReasons[esp_reset_reason()] 
    // +" (" + String(esp_reset_reason()) 
    // +") \n    Tcal="+String(tempCal)
    // +"\n    Hcal="+String(humidityCal)
    // +"\n    Pcal="+String(pressureCal) 
    // +"\n    Batt="+String(batt,4) ).c_str());
    
    ///numMeas=-1;//ready to receive new message
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
  //char buf[length+1];
  //char topicbuf[100];
  //char printbuf[100];
  //String msg((char*)payload, length);  // length-limited constructor
  String topicstr((char*)topic, 100);  // length-limited constructor
  Serial.printf("->Callback called with topic '%s'\n",topicstr.c_str());

  incomingMsgReceived = true;

  // // //payload buffer & buf are OK
  // // mqttClient.publish(STR(NAME/Status), printbuf);//this corrupts the payload buffer!
  // // //payload buffer has been corrupted here!

  // Compare if topic is NAME/SetTempCal
  if (strcmp(topic, STR(NAME/SetTempCal)) == 0) {
    String msg((char*)payload, length);  // length-limited constructor
    tempCal = msg.toFloat();// atof((char*)payload);//(buf);
    //write to flash memory
    preferences.begin(STR(NAME), false);
    preferences.putFloat("TempCal", tempCal);
    preferences.end();
    Serial.println("Temperature calibration set to: " + String(tempCal));
    mqttClient.publish(STR(NAME/Status), (String(STR(NAME/TempCal set to: )) + String(tempCal)).c_str());
    mqttClient.publish(STR(NAME/SetTempCal), "", true); //clear retained cal message
  }
  // compare if topic is NAME/SetHumidityCal
  if (strcmp(topic, STR(NAME/SetHumidityCal)) == 0) {   
    String msg((char*)payload, length);  // length-limited constructor
    humidityCal = msg.toFloat();//atof(buf);
    //write to flash memory
    preferences.begin(STR(NAME), false);
    preferences.putFloat("HumidityCal", humidityCal);
    preferences.end();
    Serial.println("Humidity calibration set to: " + String(humidityCal));
    mqttClient.publish(STR(NAME/Status), (String(STR(NAME/HumidityCal set to: )) + String(humidityCal)).c_str());
    mqttClient.publish(STR(NAME/SetHumidityCal), "", true); //clear retained cal message
  }
  // if (strcmp(topic, String(STR(NAME/SetPressureCal)).c_str()) == 0) {
  if (strcmp(topic, STR(NAME/SetPressureCal)) == 0) {
    String msg((char*)payload, length);  // length-limited constructor    pressureCal = msg.toFloat();//atof(buf);
    // write to flash memory
    preferences.begin(STR(NAME), false);
    preferences.putFloat("PressureCal", pressureCal);
    preferences.end();
    Serial.println("Pressure calibration set to: " + String(pressureCal));
    mqttClient.publish(STR(NAME/Status), (String(STR(NAME/PressureCal set to:)) + String(pressureCal)).c_str());
    mqttClient.publish(STR(NAME/SetPressureCal), "", true); // clear retained cal message
  }

#pragma region Receive History
  if (strcmp(topic, STR(NAME/GetTempHistory)) == 0) {
    // set numMeas{Received} =0: when a message contains "clr"
    // all messages that are shorter than sizeof(HistoryStruct)(=2*8) bytes will be ignored.
    // if this message never got received, then numMeasReceived =-1; as initialized in setup(); -> this will prohibit an update post above
    // if (strcmp((char*)payload, "ign") == 0) {
    // if (length < sizeof(TempHistorySTRUCT)){
    if (strcmp((char *)payload, "clr") == 0){
      Serial.println("Received 'clr': Clearing History buffer.");
      numMeas = 0;
      History.Temp[numMeas].epochTime = 0;
      History.Temp[numMeas].value = 0;
      return;
    }
    ////if(length>=sizeof(TempHistorySTRUCT)){//skip all empty and too short messages
    else{
      unsigned int bufLenInBytes = sizeof(History.Temp); // rem: number of elements of an array of structs: size_t count = sizeof(X) / sizeof(X[0]);
      unsigned int bytesToCopy = 0;
      unsigned int copyOffset = 0;
      if (length <= (bufLenInBytes - sizeof(TempHistorySTRUCT))){ // check if TempHistory buffer still has room for another tuple
        copyOffset = 0;
        bytesToCopy = length; // static_cast<unsigned int>(length/sizeof(History.Temp[0]));
        Serial.printf("Still room in history array. copyOffset=%d, payload: %d bytes, history buf: %d bytes - bytes to copy:%d\n",copyOffset,length, bufLenInBytes, bytesToCopy);
      }
      else{ // drop first element
        copyOffset = sizeof(TempHistorySTRUCT);
        bytesToCopy = length - copyOffset;
        Serial.printf("Dropping first element! copyOffset=%d, payload: %d bytes, history buf: %d bytes - bytes to copy:%d\n",copyOffset, length, bufLenInBytes, bytesToCopy);
      }

      Serial.printf("memcopy: %d bytes, offset:%d\n", bytesToCopy, copyOffset);
      Serial.printf("Target bufLenInBytes:%d, record size:%d, numRecords:%d\n", bufLenInBytes, sizeof(TempHistorySTRUCT), bufLenInBytes / sizeof(TempHistorySTRUCT));
      memcpy(History.TempByteBuffer, payload + copyOffset, bytesToCopy);

      Serial.printf("Memcopy raw: (offset:%d)\n",copyOffset);
      hexDump(payload,length);
      Serial.printf("Memcopy Target: (numMeas:%d)\n",numMeas);
      hexDump(History.TempByteBuffer,bytesToCopy);


      // Serial.printf("Before:\n");
      // hexDump(History.TempByteBuffer,length);

      if (copyOffset > 0){ //(numMeas <= MAXMEASUREMENTS){ // clear element after last to indicate end of measurements in array (should be optional)
        History.Temp[numMeas].epochTime = 0;
        History.Temp[numMeas].value = 0;
      }

      numMeas = bytesToCopy / sizeof(TempHistorySTRUCT);
      Serial.printf("Received %d bytes. Copied %d measurements\n", length, numMeas);

      // Serial.printf("After:\n");
      // hexDump(History.TempByteBuffer,length);
    }
    #pragma endregion Receive History

  } // GetTempHistory

  //Post status (at end!)
  //mqttClient.publish(STR(NAME/Status), printbuf);//this corrupts the payload buffer! (call at end)
}

