/**
 * 
 * This example shows how to set the RTC (Real Time Clock) on the UNO R4 WiFi
 * to the current date and time retrieved from an NTP server on the Internet (pool.ntp.org).
 * Then the current time from the RTC is printed to the Serial port.
 * 
 * Instructions:
 * 1. Download the libraries through the Library Manager
 * 2. Change the WiFi credentials to match your WiFi network.
 * 3. Upload this sketch to UNO R4 WiFi.
 * 4. Open the Serial Monitor.
 * 
 * Code references:
 *    WI-FI connection code borrowed from: https://docs.arduino.cc/tutorials/uno-r4-wifi/wifi-examples/
 *    RTC and alarm code borrowed from: https://docs.arduino.cc/tutorials/uno-r4-wifi/rtc
 *    Low power mode reference: https://cdn.sparkfun.com/assets/b/1/d/3/6/RA4M1_Datasheet.pdf
 *    LED Matrix: https://docs.arduino.cc/tutorials/uno-r4-wifi/led-matrix/
 * Libraries:
 *    BH1750_WE: https://github.com/wollewald/BH1750_WE/tree/master
 *    DHT: https://github.com/adafruit/DHT-sensor-library
 *    NTPClient: https://github.com/arduino-libraries/NTPClient
 * 
 */

/*******************************************************************************/
/* WI-FI Connection */
/*******************************************************************************/

/*******************************************************************************/
/* Includes */
/*******************************************************************************/


#include "RTC.h"
#include <NTPClient.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include "Arduino_LED_Matrix.h"
#include <EEPROM.h>
#include <Servo.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750_WE.h>

/*******************************************************************************/
/* Macros */
/*******************************************************************************/

// Register addresses
#define SYSTEM          0x40010000 // System Registers
#define SYSTEM_SBYCR    ((volatile unsigned short *)(SYSTEM + 0xE00C))  // Standby Control Register
#define SYSTEM_PRCR     ((volatile unsigned short *)(SYSTEM + 0xE3FE))  // Protect Register
#define ICU_WPEN        ((volatile unsigned *)0x400061A0)

// PDT
#define TIMEZONE_OFFSET_HOURS (-7)

// D
#define WIFI_SSID 
#define WIFI_PSWD 

// Sensor thresholds
#define BATTERY_THRESHOLD       1.0
#define MOISTURE_THRESHOLD      1.0
#define LUX_DAYLIGHT_THRESHOLD  1.0
// Pins
#define BH1750_ADDRESS          0x23
#define BATTERY_PIN             A1
#define DHT_PIN                 A2
#define ligh
#define MOISTURE_PIN            A0
#define DHT_TYPE                DHT11
#define WATER_LEVEL_PIN         7
#define SERVO_PIN               3
// Valve constants
#define VALVE_OPEN_DEGREES      10
#define VALVE_CLOSED_DEGREES    100
#define ACTUATE_DURATION        4

#define DEBUG_PRINT_ENABLED     1
#if DEBUG_PRINT_ENABLED
#define DEBUG_PRINT_INIT(baud)  Serial.begin(9600);  while (!Serial);
#define DEBUG_PRINT(msg)        Serial.print(msg);
#define DEBUG_PRINTLN(msg)      Serial.println(msg);
#else
#define DEBUG_PRINT_INIT(baud)  do {} while(0)
#define DEBUG_PRINT(msg)        do {} while(0)
#define DEBUG_PRINTLN(msg)      do {} while(0)
#endif


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
  DEBUG_PRINT("SSID: ");
  DEBUG_PRINTLN(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  DEBUG_PRINT("IP Address: ");
  DEBUG_PRINTLN(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  DEBUG_PRINT("signal strength (RSSI):");
  DEBUG_PRINT(rssi);
  DEBUG_PRINTLN(" dBm");
}

void connectToWiFi(){
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    DEBUG_PRINTLN("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    DEBUG_PRINTLN("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (wifiStatus != WL_CONNECTED) {
    DEBUG_PRINT("Attempting to connect to SSID: ");
    DEBUG_PRINTLN(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    wifiStatus = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  DEBUG_PRINTLN("Connected to WiFi");
  printWifiStatus();
}

/*******************************************************************************/
/* Time Synchronization */
/*******************************************************************************/

void synchronize_time()
{
  RTC.begin();
  DEBUG_PRINTLN("\nStarting connection to server...");
  timeClient.begin();
  timeClient.update();

  // Get the current date and time from an NTP server and convert
  // it to UTC +timezone by passing the time zone offset in hours.
  // You may change the time zone offset to your local one.
  auto timeZoneOffsetHours = TIMEZONE_OFFSET_HOURS;
  auto unixTime = timeClient.getEpochTime() + (timeZoneOffsetHours * 3600);
  DEBUG_PRINT("Unix time = ");
  DEBUG_PRINTLN(unixTime);
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
/* Sense & Actuate */
/*******************************************************************************/

// Size of structure: 32 bytes
struct {
  time_t  unix_time;    // Timestamp 8 bytes
  float   temperature;  // 4 bytes
  float   humidity;     // 4 bytes
  float   lux;          // 4 bytes
  int     water_level;  // 4 bytes
  float   moisture;     // 4 bytes
  float   battery;      // 4 bytes
} sensor_data;

// Sensor objects
BH1750_WE myBH1750(BH1750_ADDRESS);
DHT dht(DHT_PIN, DHT_TYPE);
boolean valveOpen;
Servo myServo;

void sense_and_actuate()
{
    // 1. TODO Power up sensors
    power_up_sensors();

    // 2. TODO Synchronize time
    RTCTime currentTime;
    RTC.getTime(currentTime);
    sensor_data.unix_time = currentTime.getUnixTime();

    // 3. Do measurements
    measure_sensors();

    // 4. Actuate
    actuate();

    // 5. Power down sensors()
    power_down_sensors();
}

void power_up_sensors()
{

}

void setup_sensors()
{
    myServo.attach(SERVO_PIN);
    dht.begin();
    Wire.begin();
    if (!myBH1750.init()) {
      DEBUG_PRINTLN("BH1750 Connection Failed!");
      // while (1) {}
    }
    pinMode(WATER_LEVEL_PIN,INPUT);
    closeValve(); // Ensure the valve is closed on startup
}

void measure_sensors()
{
    sensor_data.temperature = dht.readTemperature();
    sensor_data.humidity = dht.readHumidity();
    sensor_data.lux = myBH1750.getLux();
    sensor_data.water_level = digitalRead(WATER_LEVEL_PIN);
    sensor_data.moisture = analogRead(MOISTURE_PIN);

    char buffer[200];
    sprintf(buffer, "[%ld, %f, %f, %f, %d, %f, %f]", sensor_data.unix_time, sensor_data.temperature, sensor_data.humidity, sensor_data.lux,
    sensor_data.water_level, sensor_data.moisture, sensor_data.battery);
    DEBUG_PRINTLN(buffer);
    // delay(10000);
}

void actuate()
{
    if ((sensor_data.moisture < MOISTURE_THRESHOLD) &&
        (sensor_data.lux > LUX_DAYLIGHT_THRESHOLD))
    {
        // TODO Actuate        
    }
}

void openValve() {
  myServo.write(VALVE_OPEN_DEGREES);
  valveOpen = true;
  DEBUG_PRINTLN("Valve opened.");
}

void closeValve() {
  myServo.write(VALVE_CLOSED_DEGREES);
  valveOpen = false;
  DEBUG_PRINTLN("Valve closed.");
}

bool isValveOpen() {
  return valveOpen;
}

void power_down_sensors()
{

}


/*******************************************************************************/
/* Data Storage */
/*******************************************************************************/

void store_data()
{
    int data_length = EEPROM.read(0);
    int data_addr = (data_length + 1) * sizeof(sensor_data);

    if (data_addr < 8000)
    {
      DEBUG_PRINTLN(data_length);
      DEBUG_PRINTLN(data_addr);
      EEPROM.put(data_addr, sensor_data);
      data_length += 1;
      EEPROM.write(0, data_length);
    }
    else // EEPROM full
    {

    }
}

/*******************************************************************************/
/* Sleep */
/*******************************************************************************/

void enter_low_power_mode()
{
  // Retrieve the date and time from the RTC and print them
  RTCTime currentTime;
  AlarmMatch alarm;
  RTC.getTime(currentTime);
  int minutes = currentTime.getMinutes();
  DEBUG_PRINTLN("The RTC is: " + String(currentTime));

  // Trigger the alarm every time the seconds are zero
  RTCTime alarmTime;
  alarmTime.setMinute(minutes+1);   // Wake up when minute matches
  // Make sure to only match on the seconds in this example - not on any other parts of the date/time
  AlarmMatch matchTime;
  matchTime.addMatchMinute();

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
  // 1. Initialize Serial comm if needed
  DEBUG_PRINT_INIT(9600);

  // 2. Connect to WIFI
  connectToWiFi();
  // 3. Synchronize time
  synchronize_time();
  // 4. DIsconnect from WIFI
  WiFi.disconnect();
  //define LED as output
  pinMode(LED_BUILTIN, OUTPUT);
  // 5. One time setup of sensors
  setup_sensors();
  // 6. One time setup of low power mode
  setup_low_power_mode();
  DEBUG_PRINTLN("Setup done");
}

void loop()
{
  // 1. TODO Read battery

  // 2. Sense and actuate
  sense_and_actuate();

  // 3. Store/Send data
  store_data();

  // 4. Enter low power mode
  DEBUG_PRINTLN("Going to sleep zzzzz");
  enter_low_power_mode();
  DEBUG_PRINTLN("Back from sleep!");
}