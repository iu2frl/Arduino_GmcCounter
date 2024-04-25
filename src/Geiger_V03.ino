#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <NTPClient.h>
#include "secrets.h"

WiFiClient client;

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

SoftwareSerial mySerial(5, 4);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish FeedCpm = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME CPM_FEED);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTPSERVER);
int cntNtp;

String radMonUser = RADMONUSER;
String radMonPass = RADMONPASS;

String gmcMapUser = GMCMAPUSER;
String gmcMapId = GMCMAPID;

const char* ssid = STASSID;
const char* password = STAPSK;

int curCpm;
int lastCpm;
uint64_t iotUpdateTimestamp = 0;
uint64_t ntpUpdateTimestamp = 0;

void setup() {
  // Define a counter for no WiFi
  int wifiCnt = 0;
  // Initializing SoftwareSerial
  mySerial.begin(1200);
  //WiFi.setTxPower(WIFI_POWER_7dBm);
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

  WiFi.setOutputPower(0);

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
    FeedCpm.publish(curCpm);
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
  if (lastCpm == 0) {
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
  if (client.connect("radmon.org", 80)) {
    client.print("GET /radmon.php?function=submit&user=");
    client.print(radMonUser);
    client.print("&password=");
    client.print(radMonPass);
    client.print("&value=");
    client.print(String(curCpm));
    client.print("&unit=CPM");
    client.println(" HTTP/1.0");
    client.println("HOST: radmon.org");
    client.println("Connection: close");
    client.println();
  }
}

void gmcMapPub() {
  if (client.connect("gmcmap.com", 80)) {
    client.print("GET /log2.asp?AID=");
    client.print(gmcMapUser);
    client.print("&GID=");
    client.print(gmcMapId);
    client.print("&CPM=");
    client.print(String(curCpm));
    client.println(" HTTP/1.0");
    client.println("HOST: gmcmap.com");
    client.println("Connection: close");
    client.println();
  }
}

void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) {  // connect will return 0 for connected
    //Serial.println(mqtt.connectErrorString(ret));
    //Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);  // wait 5 seconds
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      //while (1);
    }
  }
}
