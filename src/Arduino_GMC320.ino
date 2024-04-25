#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <NTPClient.h>
#include "secrets.h"

WiFiClient wifiClient;

// --------- Username and passwords ----------------------------------------------------------
// Declare the following variables in include/secrets.h 
//
//  #define STASSID "YourSSID"
//  #define STAPSK "YourPASS"
//  #define RADMONUSER "YourUSER"
//  #define RADMONPASS "YourPASS"
//  #define GMCMAPUSER "YourUSER"
//  #define GMCMAPID "YourUploadID"
//  #define AIO_SERVER "io.adafruit.com"
//  #define AIO_SERVERPORT 1883  // use 8883 for SSL
//  #define AIO_USERNAME "YourUSER"
//  #define AIO_KEY "UploadKey"
//  #define CPM_FEED "/your/feed.data"
//  #define IOT_INTERVAL 1 * 61 * 1000  // 1 minute
//  #define NTPSERVER "some.ip.address"
//  #define NTPINTERVAL 5 * 60 * 1000  // 5 minutes

// SoftwareSerial initialization
SoftwareSerial mySerial(5, 4);

// Adafruit IoT initialization
Adafruit_MQTT_Client mqttClient(&wifiClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish feedCpm = Adafruit_MQTT_Publish(&mqttClient, AIO_USERNAME CPM_FEED);

// NTP client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTPSERVER);
int cntNtp;

// https://radmon.org/index.php
String radMonUser = RADMONUSER;
String radMonPass = RADMONPASS;

// https://gmcmap.com/
String gmcMapUser = GMCMAPUSER;
String gmcMapId = GMCMAPID;

// Wifi data
const char* ssid = STASSID;
const char* password = STAPSK;

// Global variables
int curCpm;
int lastCpm;
uint64_t iotUpdateTimestamp = 0;
uint64_t ntpUpdateTimestamp = 0;

// --------- code ------------------
void setup() {
  // Define a counter for no WiFi
  int wifiCnt = 0;
  // Initializing SoftwareSerial
  mySerial.begin(1200);
  //WiFi.setTxPower(WIFI_POWER_7dBm);
  WiFi.setOutputPower(0);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    delay(1000);
    ++wifiCnt;
    // If no connection for 30 secs at startup
    if (wifiCnt >= 30) {
      // Get to deep sleep
      ESP.deepSleep(0);
    }
  }

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("GMC-320");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }
  });
  ArduinoOTA.begin();

  // Initialize timeClient library
  timeClient.begin();
  // Set timezone
  timeClient.setTimeOffset(3600);
  // Request NTP update
  delay(5000);
  setTime();
}

void loop() {
  // Manage MQTT connection
  MQTT_connect();
  // Manage OTA Updates
  ArduinoOTA.handle();

  // Get current time
  uint64_t now = millis();
  if ((now - iotUpdateTimestamp) > IOT_INTERVAL) {
    // Read current CPM
    getCpm();
    // Publish data to AdafruitIO
    feedCpm.publish(curCpm);
    // Publish data to RadMon
    radMonPub();
    // Publish data to gmcmap
    gmcMapPub();
    // Update last sent package
    iotUpdateTimestamp = now;
  }

  if ((now - ntpUpdateTimestamp) > NTPINTERVAL) {
    setTime();
    ntpUpdateTimestamp = now;
  }
}

void setTime() {
  // Get current time from NTP
  ntpUpdate();
  // Wait a little
  delay(100);
  // Set year date and time
  time_t epochTime = timeClient.getEpochTime();
  struct tm* ptm = gmtime((time_t*)&epochTime);
  // Send year
  sendData("<SETDATEYY", byte(ptm->tm_year - (int)100));
  // Send month
  sendData("<SETDATEMM", byte(ptm->tm_mon + 1));
  // Send day
  sendData("<SETDATEDD", byte(ptm->tm_mday));
  // Send hour
  sendData("<SETDATEHH", byte(ptm->tm_hour));
  // Send minute
  sendData("<SETDATEMM", byte(ptm->tm_min));
  // Send seconds
  sendData("<SETDATESS", byte(ptm->tm_sec));
}

bool sendData(const char *id, byte value) {
  // Send data to virtual serial
  mySerial.print(id);
  mySerial.write(value);
  mySerial.println(">>");
  delay(10);
  byte reply[1];
  mySerial.readBytes(reply, 1);
  mySerial.flush();
  if (reply[0] == 0xAA) {
    return true;
  } else {
    return false;
  }
}

void ntpUpdate() {
  // Update NTP time
  if (!timeClient.update()) {
    ++cntNtp;
  } else {
    cntNtp = 0;
  }
  // If fail for 5 times reboot ESP
  if (cntNtp > 5) {
    ESP.restart();
  }
}

void getCpm() {
  // 2. Get current CPM value
  // Command:  <GETCPM>>
  char buf[10];
  // Get latest CPM count
  mySerial.println("<GETCPM>>");
  delay(1);
  mySerial.readBytes(buf, 2);
  int cpmVal = (int(buf[1]) + (int(buf[0] * 256)));
  if (cpmVal == 0) {
    cpmVal = 20;
    lastCpm = cpmVal;
  } else if (lastCpm == 0) {
    if (cpmVal < 100) {
      curCpm = cpmVal;
      lastCpm = cpmVal;
    } else {
      cpmVal = 20;
      lastCpm = cpmVal;
    }
  } else if ((cpmVal > (lastCpm / 2)) && (cpmVal < (lastCpm * 2))) {
    curCpm = cpmVal;
    lastCpm = curCpm;
  } else {
    curCpm = lastCpm + 1;
  }
}

void radMonPub() {
  if (wifiClient.connect("radmon.org", 80)) {
    wifiClient.print("GET /radmon.php?function=submit&user=");
    wifiClient.print(radMonUser);
    wifiClient.print("&password=");
    wifiClient.print(radMonPass);
    wifiClient.print("&value=");
    wifiClient.print(String(curCpm));
    wifiClient.print("&unit=CPM");
    wifiClient.println(" HTTP/1.0");
    wifiClient.println("HOST: radmon.org");
    wifiClient.println("Connection: close");
    wifiClient.println();
  }
}

void gmcMapPub() {
  if (wifiClient.connect("gmcmap.com", 80)) {
    wifiClient.print("GET /log2.asp?AID=");
    wifiClient.print(gmcMapUser);
    wifiClient.print("&GID=");
    wifiClient.print(gmcMapId);
    wifiClient.print("&CPM=");
    wifiClient.print(String(curCpm));
    wifiClient.println(" HTTP/1.0");
    wifiClient.println("HOST: gmcmap.com");
    wifiClient.println("Connection: close");
    wifiClient.println();
  }
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqttClient.connected()) {
    return;
  }

  uint8_t retries = 3;
  while ((ret = mqttClient.connect()) != 0) {  // connect will return 0 for connected
    //Serial.println(mqtt.connectErrorString(ret));
    //Serial.println("Retrying MQTT connection in 5 seconds...");
    mqttClient.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      //while (1);
    }
  }
}
