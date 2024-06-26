# Arduino_GmcCounter

Arduino software to control GMC geiger counters

## Description

[https://www.iu2frl.it](https://www.iu2frl.it/interfaccia-wifi-per-contatore-geiger-gmc-320-con-esp-01s/)

## Features

+ NTP Time sync
+ Report to RadMon
+ Report to GmcMap
+ ESP32 Auto poweroff if outside
+ MQTT reporting to Adafruit IO

## Documentation of the GMC protocol

```txt
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
```
