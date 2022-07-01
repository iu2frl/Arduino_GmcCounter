#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <NTPClient.h>

#ifndef STASSID
#define STASSID "MySSID"
#define STAPSK  "MyPASS"
#endif

WiFiClient client;

// --------- SoftwareSerial parameters -------------------------------------------------------
SoftwareSerial mySerial(5, 4);

// --------- Information to Connect to Adafruit IO -------------------------------------------
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883                   // use 8883 for SSL
#define AIO_USERNAME    "MyUser"
#define AIO_KEY         "MyApiKey"
#define CPM_FEED       "/feeds/geiger.cpm"
#define IOT_INTERVAL   1*61*1000 // 1 minute
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish FeedCpm = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME CPM_FEED);

//------------ Clock parameters -------------------------------------------------------------
#define NtpServer "192.168.0.1"
#define NTP_INTERVAL 5*60*1000 // 5 minutes
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NtpServer);
int cntNtp;

//------------ RadMon Parameters -------------------------------------------------------------
String radMonUser = "MyUser";
String radMonPass = "MyPass";

//------------ GMCMap Parameters -------------------------------------------------------------
String gmcMapUser = "MyUser";
String gmcMapId = "MyPass";

//--------------------------------------------------------------------------------------------
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

  // Big delay to allow reprogramming in case of bad signal
  //delay(10000);
  // Now that we are connected lower power to lower consumption
  WiFi.setOutputPower(0);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("GMC-320");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
  });
  ArduinoOTA.begin();

  // Initialize timeClient library
  timeClient.begin();
  // Set timezone
  timeClient.setTimeOffset(3600);
  // Request NTP update
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

  if ((now - ntpUpdateTimestamp) > NTP_INTERVAL) {
    setTime();
    ntpUpdateTimestamp = now;
  }
}

void setTime() {
  // Get current time from NTP
  ntpUpdate();
  // Wait a little
  delay(100);
  int tempVal = 0;
  //22. Set year date and time
  //command: <SETDATETIME[YYMMDDHHMMSS]>>
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime ((time_t *)&epochTime);
  // Command header
  mySerial.print("<SETDATETIME");
  // Format year to YY
  mySerial.write(byte((int)ptm->tm_year - (int)100));
  // Format month to DD
  mySerial.write(byte(ptm->tm_mday));
  // Format day to MM
  mySerial.write(byte(ptm->tm_mon + 1));
  // Format hours to HH. Check if DST
  //if (ptm->tm_isdst) {
  //  mySerial.write(byte(timeClient.getHours() + (int)1));
  //} else {
    mySerial.write(byte(timeClient.getHours()));
  //}
  // Format minutes to MM
  mySerial.write(byte(timeClient.getMinutes()));
  // Format seconds to SS
  mySerial.write(byte(timeClient.getSeconds()));
  // Terminating character
  mySerial.println(">>");
  delay(1);
  byte reply[1];
  mySerial.readBytes(reply, 1);
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
    client.print(radMonUser); client.print("&password=");
    client.print(radMonPass); client.print("&value=");
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
    client.print(gmcMapUser); client.print("&GID=");
    client.print(gmcMapId); client.print("&CPM=");
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
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
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

/*
***************************************************
  http://www.gqelectronicsllc.com/download/GQ-RFC1201.txt
  GQ-RFC1201
  GQ Geiger Counter Communication Protocol
***************************************************
  Ver 1.40    Jan-2015

  Status of this Memo
   This document specifies a GQ GMC Geiger Counter Communication Protocol for the
   communication between GQ GMC Geiger Counter and a computer host via serial port, and requests discussion and suggestions for
   improvements.  Distribution of this memo is unlimited.

  Copyright Notice
   Copyright (C) GQ Electronics LLC (2012).  All Rights Reserved.

  Abstract
   This document defines a GQ GMC Geiger Counter Communication Protocol
   to support communication between GMC Geiger Counter and a computer host via serial port.  The protocol allows to send data request command from a computer host to a GQ GMC geiger counter.

**************************
  Serial Port configuration
**************************

  For the GMC-300 V3.xx and earlier version, the serial port communication is based on a fixed baud rate.

  Baud: 57600
  Data bit: 8
  Parity: None
  Stop bit: 1
  Control: None

  For the GMC-300 Plus V4.xx and later version firmware,  115200 BPS is used.
  For GMC-320, the serial port communication baud rate is variable. It should be one of the followings:
  1200,2400,4800,9600,14400,19200,28800,38400,57600,115200 BPS.  The factory default is 115200 BPS.

**************************
  Command format
**************************

  A valid command start with ASCII '<'  and ended with ASCII '>>'. Both command and parameters are in between '<' and '>>'.
  Command is a ASCII string. All parameters of command are true value in hexadecimal.

  Direction: All commands are initiated from computer(HOST).

**************************
  Commands
**************************

  1. Get hardware model and version
  Command:  <GETVER>>

  Return:   total 14 bytes ASCII chars from GQ GMC unit. It includes 7 bytes hardware model and 7 bytes firmware version.

   e.g.:  GMC-300Re 2.10

  Firmware supported:  GMC-280, GMC-300 Re.2.0x, Re.2.10 or later


  2. Get current CPM value


  Command:  <GETCPM>>

  Return:   A 16 bit unsigned integer is returned. In total 2 bytes data return from GQ GMC unit. The first byte is MSB byte data and second byte is LSB byte data.

    e.g.: 00 1C     the returned CPM is 28.

  Firmware supported:  GMC-280, GMC-300 Re.2.0x, Re.2.10 or later



  3. Turn on the GQ GMC heartbeat

  Note:     This command enable the GQ GMC unit to send count per second data to host every second automatically.

  Command:  <HEARTBEAT1>>

  Return:   A 16 bit unsigned integer is returned every second automatically. Each data package consist of 2 bytes data from GQ GMC unit. The first byte is MSB byte data and second byte is LSB byte data.


   e.g.: 10 1C     the returned 1 second count is 28.   Only lowest 14 bits are used for the valid data bit.  The highest bit 15 and bit 14 are reserved data bits.



  Firmware supported:  GMC-280, GMC-300  Re.2.10 or later


  4. Turn off the GQ GMC heartbeat


  Command:  <HEARTBEAT0>>

  Return:   None

  Firmware supported:  Re.2.10 or later


  5. Get battery voltage status


  Command:  <GETVOLT>>

  Return:   one byte voltage value of battery (X 10V)

          e.g.: return 62(hex) is 9.8V

  Firmware supported:  GMC-280, GMC-300 Re.2.0x, Re.2.10 or later


  6. Request history data from internal flash memory

  Command:  <SPIR[A2][A1][A0][L1][L0]>>

  A2,A1,A0 are three bytes address data, from MSB to LSB.  The L1,L0 are the data length requested.  L1 is high byte of 16 bit integer and L0 is low byte.

  The length normally not exceed 4096 bytes in each request.

  Return: The history data in raw byte array.

  Comment: The minimum address is 0, and maximum address value is the size of the flash memory of the GQ GMC Geiger count. Check the user manual for particular model flash size.

  Firmware supported:  GMC-300 Re.2.0x, Re.2.10 or later


  7. Get configuration data

  Command:  <GETCFG>>

  Return: The configuration data.  Total 256 bytes will be returned.


  Firmware supported:  GMC-280, GMC-300 Re.2.10 or later



  8. Erase all configuration data

  Command:  <ECFG>>

  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.2.10 or later


  9. Write configuration data

  Command:  <WCFG[A0][D0]>>

  A0 is the address and the D0 is the data byte(hex).

  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.2.10 or later



  10. send a key

  Command:  <key[D0]>>

  D0 is the key value from 0 to 3. It represents software key S1~S4.
  Return: none

  Firmware supported:  GMC-280, GMC-300 Re.2.0x, Re.2.10 or later

  Note: for Re.2.11 or later, each key can be a ASCII string: <KEY0>>,<KEY1>>,<KEY2>>,<KEY3>>



  11. get serial number

  Command: <GETSERIAL>>

  Return: serial number in 7 bytes.

  Firmware supported: GMC-280, GMC-300 Re.2.11 or later



  12. Power OFF

  Command: <POWEROFF>>

  Return: none

  Firmware supported: GMC-280, GMC-300 Re.2.11 or later


  13. Reload/Update/Refresh Configuration


  Command: <CFGUPDATE>>

  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.2.20 or later


  14. Set realtime clock year

  command: <SETDATEYY[D0]>>

  D0 is the year value in hexdecimal

  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.2.23 or later


  15. Set realtime clock month

  command: <SETDATEMM[D0]>>

  D0 is the month value in hexdecimal

  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.2.23 or later


  16. Set realtime clock day

  command: <SETDATEDD[D0]>>

  D0 is the day of the month value in hexdecimal

  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.2.23 or later

  17. Set realtime clock hour

  command: <SETTIMEHH[D0]>>

  D0 is the hourvalue in hexdecimal

  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.2.23 or later

  18. Set realtime clock minute

  command: <SETTIMEMM[D0]>>

  D0 is the minute value in hexdecimal

  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.2.23 or later



  19. Set realtime clock second

  command: <SETTIMESS[D0]>>

  D0 is the second value in hexdecimal

  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.2.23 or later


  20. Reset unit to factory default

  command: <FACTORYRESET>>


  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.3.00 or later


  21. Reboot unit

  command: <REBOOT>>


  Return: None

  Firmware supported: GMC-280, GMC-300 Re.3.00 or later


  22. Set year date and time

  command: <SETDATETIME[YYMMDDHHMMSS]>>


  Return: 0xAA

  Firmware supported: GMC-280, GMC-300 Re.3.00 or later



  23. Get year date and time

  command: <GETDATETIME>>


  Return: Seven bytes data: YY MM DD HH MM SS 0xAA

  Firmware supported: GMC-280, GMC-300 Re.3.00 or later



  24. Get temperature

  command: <GETTEMP>>

  Return: Four bytes celsius degree data in hexdecimal: BYTE1,BYTE2,BYTE3,BYTE4
  Here: BYTE1 is the integer part of the temperature.
        BYTE2 is the decimal part of the temperature.
        BYTE3 is the negative signe if it is not 0.  If this byte is 0, the then current temperture is greater than 0, otherwise the temperature is below 0.
  `       BYTE4 always 0xAA

  Firmware supported: GMC-320 Re.3.01 or later


  25. Get gyroscope data

  command: <GETGYRO>>

  Return: Seven bytes gyroscope data in hexdecimal: BYTE1,BYTE2,BYTE3,BYTE4,BYTE5,BYTE6,BYTE7
  Here: BYTE1,BYTE2 are the X position data in 16 bits value. The first byte is MSB byte data and second byte is LSB byte data.
        BYTE3,BYTE4 are the Y position data in 16 bits value. The first byte is MSB byte data and second byte is LSB byte data.
        BYTE5,BYTE6 are the Z position data in 16 bits value. The first byte is MSB byte data and second byte is LSB byte data.
        BYTE7 always 0xAA

  Firmware supported: GMC-320 Re.3.01 or later


  26. Power ON

  Command: <POWERON>>

  Return: none

  Firmware supported: GMC-280, GMC-300, GMC-320 Re.3.10 or later
*/
