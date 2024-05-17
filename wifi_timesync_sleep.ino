/**
 * 
 * This example shows how to set the RTC (Real Time Clock) on the UNO R4 WiFi
 * to the current date and time retrieved from an NTP server on the Internet (pool.ntp.org).
 * Then the current time from the RTC is printed to the Serial port.
 * 
 * Instructions:
 * 1. Download the NTPClient library (https://github.com/arduino-libraries/NTPClient) through the Library Manager
 * 2. Change the WiFi credentials to match your WiFi network.
 * 3. Upload this sketch to UNO R4 WiFi.
 * 4. Open the Serial Monitor.
 * 
 * WI-FI connection code borrowed from: https://docs.arduino.cc/tutorials/uno-r4-wifi/wifi-examples/
 * RTC and alarm code borrowed from: https://docs.arduino.cc/tutorials/uno-r4-wifi/rtc
 * Low power mode reference: https://cdn.sparkfun.com/assets/b/1/d/3/6/RA4M1_Datasheet.pdf
 * LED Matrix: https://docs.arduino.cc/tutorials/uno-r4-wifi/led-matrix/
 * 
 */

/*******************************************************************************/
/* Includes */
/*******************************************************************************/

#include "RTC.h"
#include <NTPClient.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include "Arduino_LED_Matrix.h"

/*******************************************************************************/
/* Macros */
/*******************************************************************************/

#define SYSTEM          0x40010000 // System Registers
#define SYSTEM_SBYCR    ((volatile unsigned short *)(SYSTEM + 0xE00C))  // Standby Control Register
#define SYSTEM_PRCR     ((volatile unsigned short *)(SYSTEM + 0xE3FE))  // Protect Register

#define ICU_WPEN        ((volatile unsigned *)0x400061A0)

// PDT
#define TIMEZONE_OFFSET_HOURS (-7)

// D
// #define WIFI_SSID 
// #define WIFI_PSWD 

/*******************************************************************************/
/* WI-FI Connection */
/*******************************************************************************/

char ssid[] = WIFI_SSID;        // your network SSID (name)
char pass[] = WIFI_PSWD;    // your network password (use for WPA, or use as key for WEP)

int wifiStatus = WL_IDLE_STATUS;
WiFiUDP Udp; // A UDP instance to let us send and receive packets over UDP
NTPClient timeClient(Udp);

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void connectToWiFi(){
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (wifiStatus != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    wifiStatus = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to WiFi");
  printWifiStatus();
}

/*******************************************************************************/
/* Time Synchronization */
/*******************************************************************************/

void synchronize_time()
{
  RTC.begin();
  Serial.println("\nStarting connection to server...");
  timeClient.begin();
  timeClient.update();

  // Get the current date and time from an NTP server and convert
  // it to UTC +timezone by passing the time zone offset in hours.
  // You may change the time zone offset to your local one.
  auto timeZoneOffsetHours = TIMEZONE_OFFSET_HOURS;
  auto unixTime = timeClient.getEpochTime() + (timeZoneOffsetHours * 3600);
  Serial.print("Unix time = ");
  Serial.println(unixTime);
  RTCTime timeToSet = RTCTime(unixTime);
  RTC.setTime(timeToSet);
}

/*******************************************************************************/
/* LED Matrix */
/*******************************************************************************/

const uint32_t logo[] = {
    0x18c21,
		0x2d62521,
		0x8c000000,
};

void light_up_matrix()
{
  ArduinoLEDMatrix matrix;
  matrix.begin();
  matrix.loadFrame(logo);
}

/*******************************************************************************/
/* Sleep */
/*******************************************************************************/

void go_to_sleep()
{
  // Retrieve the date and time from the RTC and print them
  RTCTime currentTime;
  AlarmMatch alarm;
  RTC.getTime(currentTime);
  int seconds = currentTime.getSeconds();
  Serial.println("The RTC is: " + String(currentTime));

  // Trigger the alarm every time the seconds are zero
  RTCTime alarmTime;
  alarmTime.setSecond(seconds);
  // Make sure to only match on the seconds in this example - not on any other parts of the date/time
  AlarmMatch matchTime;
  matchTime.addMatchSecond();

  //sets the alarm callback
  RTC.setAlarmCallback(alarmCallback, alarmTime, matchTime);

  delay(100);   // Wait for previous operations to finish
  asm volatile("wfi");           // Stop here and wait for an interrupt
}

void setup_low_power_mode()
{
  *SYSTEM_PRCR = 0xA502u;   // Disable write protection
  *SYSTEM_SBYCR = (volatile unsigned short) 1u << 15u;    // Use Software Standby mode.
  *SYSTEM_PRCR = 0xA500u;   // Enable write protection

  *ICU_WPEN = 1u << 24u;  // Software standby returns by RTC alarm interrupt enabled
}

bool ledState = false;
// this function activates every minute
// and changes the ledState boolean
void alarmCallback() {
  if (!ledState) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  ledState = !ledState;
}

/*******************************************************************************/
/* Main */
/*******************************************************************************/

void setup(){
  Serial.begin(9600);
  while (!Serial);
  Serial.println("pre");

  connectToWiFi();
  synchronize_time();
  
  //define LED as output
  pinMode(LED_BUILTIN, OUTPUT);

  setup_low_power_mode();
}

void loop()
{
  Serial.println("Going to sleep zzzzz");
  go_to_sleep();
  Serial.println("Back from sleep!");
}