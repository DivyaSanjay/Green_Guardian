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
#define SYSTEM_OPCCR ((volatile unsigned *)0x4001E0A0)
#define SYSTEM_SCKDIVCR ((volatile unsigned *)0x4001E020)

// PDT
#define TIMEZONE_OFFSET_HOURS (-7)

// D

// Sensor thresholds
#define BATTERY_THRESHOLD       30
#define MOISTURE_THRESHOLD      25 //Percentage
#define MOISTURE_HIGH_THRESHOLD      70 //Percentage
#define LUX_LOW_THRESHOLD 6,522.0
#define LUX_HIGH_THRESHOLD  26,087.0
#define TEMPERATURE_LOW_THRESHOLD  10   // THE PLANT NEEDS TO BE MOVED INMEDIATELLY
#define TEMPERATURE_HIGH_THRESHOLD  32  // THE PLANT NEEDS TO BE MOVED INMEDIATELLY
#define TEMPERATURE_LOW_IDEAL 21
#define TEMPERATURE_HIGH_IDEAL  26
#define HUMIDITY_LOW_IDEAL 40
#define HUMIDITY_HIGH_IDEAL  60
#define MAX_NOT_IDEAL_DAYS  3
#define INITIAL_WATERING_DURATION  300 //(5 minutes)
#define MAX_WATERING_CYCLES  3
// Pins
#define BH1750_ADDRESS          0x23
#define BATTERY_PIN             A3
#define DHT_PIN                 A2
#define MOISTURE_PIN_1          A1
#define MOISTURE_PIN_2          A0
#define DHT_TYPE                DHT11
#define WATER_LEVEL_PIN         7
#define SERVO_PIN               3
#define SENSOR_ENABLE_PIN       9
// Valve constants
#define VALVE_OPEN_DEGREES      10
#define VALVE_CLOSED_DEGREES    100
#define ACTUATE_DURATION        4
// Calibration values for soil moisture sensor (inverse relationship)
#define VALUEAT0                500
#define VALUEAT100              270

// Calibration values for soil moisture sensor (inverse relationship)
#define VALUEAT0BAT             0
#define VALUEAT100BAT           850

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

// Matrix states
#define MATRIX_NORMAL           1
#define MATRIX_ERROR            2
#define MATRIX_NEED_WATER       3
#define MATRIX_NEED_SUN         4
#define MATRIX_NEED_SHADE       5
#define MATRIX_NEED_WARM        6
#define MATRIX_NEED_COLD        7

#define MATRIX_NEED_SUN_NOW     8
#define MATRIX_NEED_WARM_NOW    9
#define MATRIX_NEED_COLD_NOW    10
#define MATRIX_NEED_SHADE_NOW   11



/*******************************************************************************/
/* WI-FI Connection */
/*******************************************************************************/

char ssid[] = "iPhone de Guille";        // your network SSID (name)
char pass[] = "12345678";    // your network password (use for WPA, or use as key for WEP)

ArduinoLEDMatrix matrix;


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
  if (unixTime < 0)
  {
    synchronize_time();
  }
}

/*******************************************************************************/
/* LED Matrix */
/*******************************************************************************/

const uint32_t logo[] = {
    0x18c21,
    0x2d62521,
    0x8c000000,
};

const uint32_t bad_state[] = {
    0x62655,
    0x57755556,
    0x56000000,
  };

const uint32_t drop[] = {
    0x0000400a,
    0x01101101,
    0x100e0000
};


const uint32_t sun[] = {
    0x2481500e,
    0x03f80e01,
    0x50248000
};

const uint32_t shade[] = {
    0x6ac4aa4a,
    0xa6aa2ea2,
    0xaa2aa6ac
};

const uint32_t too_cold[] = {
    0x3b80a00a,
    0x03a02202,
    0x202203b8
};



const uint32_t too_warm[] = {
    0x3a20a20a,
    0x23a222a2,
    0x2a22a3be
};

const uint32_t need_sun_now[] = {
    0x0002a41c,
    0x43e41c02,
    0xa4000000
};

const uint32_t too_cold_now[] = {
    0x77414414,
    0x47444444,
    0x44440774
};

const uint32_t too_warm_now[] = {
    0x74514514,
    0x57454454,
    0x5545477d
};

const uint32_t shade_now[] = {
    0xe0125523,
    0x9e7d8398,
    0x55800e01
};

void light_up_matrix(unsigned int matrix_state)
{

  switch(matrix_state) {
        case MATRIX_NORMAL:
            matrix.loadFrame(logo);
            break;
        case MATRIX_ERROR :
            matrix.loadFrame(bad_state);
            break;
        case MATRIX_NEED_WATER:
            matrix.loadFrame(drop);
            break;
        case MATRIX_NEED_SUN:
            matrix.loadFrame(sun);
            break;
        case MATRIX_NEED_SHADE :
            matrix.loadFrame(shade);
            break;
        case MATRIX_NEED_WARM :
            matrix.loadFrame(too_cold);
            break;
        case MATRIX_NEED_COLD :
            matrix.loadFrame(too_warm);
            break;
        case MATRIX_NEED_SUN_NOW :
            matrix.loadFrame(need_sun_now);
            break;
        case MATRIX_NEED_WARM_NOW :
            matrix.loadFrame(too_cold_now);
            break;
        case MATRIX_NEED_COLD_NOW :
            matrix.loadFrame(too_warm);
            break;
        case MATRIX_NEED_SHADE_NOW :
            matrix.loadFrame(shade_now);
            break;
        default:
            matrix.loadFrame(bad_state);

    }

  delay(4000); 
}

/*******************************************************************************/
/* Sense & Actuate */
/*******************************************************************************/

/*
 aDDR 0 => DATA LENGTH
 32 => STRUCTURE [1]
 64 [2]

*/

// Size of structure: 32 bytes
struct {
  time_t  unix_time;    // Timestamp 8 bytes
  float   temperature;  // 4 bytes
  float   humidity;     // 4 bytes
  float   lux;          // 4 bytes   // I suppose this is the average
  int     water_level;  // 4 bytes
  short int   moisture_1;     // 2 bytes
  short int   moisture_2;     // 2 bytes
  int   battery;      // 4 bytes
} sensor_data;

// Sensor objects
BH1750_WE myBH1750(BH1750_ADDRESS);
DHT dht(DHT_PIN, DHT_TYPE);
boolean valveOpen;
Servo myServo;
short int hot_days = 0;
short int cold_days = 0;
short int shade_days = 0;
short int light_days = 0;
bool different_day_temp = false;


void sense_and_actuate()
{
    // 1. Power up sensors
    power_up_sensors();
   dht.begin();
    Wire.begin();
    if (!myBH1750.init()) {
      DEBUG_PRINTLN("BH1750 Connection Failed!");
      // while (1) {}
    }

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
  digitalWrite(SENSOR_ENABLE_PIN, HIGH);
}

void setup_sensors()
{
    myServo.attach(SERVO_PIN);
    dht.begin();
    Wire.begin();
    if (!myBH1750.init()) {
      // DEBUG_PRINTLN("BH1750 Connection Failed!");
      // while (1) {}
    }
    pinMode(WATER_LEVEL_PIN,INPUT);
    closeValve(); // Ensure the valve is closed on startup
    delay(2000);
}

void measure_sensors()
{
    sensor_data.temperature = dht.readTemperature();
    sensor_data.humidity = dht.readHumidity();
    sensor_data.lux = myBH1750.getLux();
    sensor_data.water_level = digitalRead(WATER_LEVEL_PIN);
    sensor_data.moisture_1 = (short int)readSoilMoisture(MOISTURE_PIN_1);
    sensor_data.moisture_2 = (short int)readSoilMoisture(MOISTURE_PIN_2);
    

    // Display if everything is being read correctly
    unsigned int normal = 1;
    normal &= sensor_data.moisture_1 > 0 ? 1 : 0;
    normal &= sensor_data.moisture_2 > 0 ? 1 : 0;
    normal &= sensor_data.temperature > 0.0 ? 1 : 0;
    normal &= sensor_data.humidity > 0.0 ? 1 : 0;
    normal &= sensor_data.lux > 0.0 ? 1 : 0;
    //if (normal)
    //  light_up_matrix(MATRIX_NORMAL);
    //else
    //  light_up_matrix(MATRIX_ERROR);

    char buffer[200];
    sprintf(buffer, "[%ld, %f, %f, %f, %d, %d, %d, %d]", sensor_data.unix_time, sensor_data.temperature, sensor_data.humidity, sensor_data.lux,
    sensor_data.water_level, sensor_data.moisture_1, sensor_data.moisture_2, sensor_data.battery);
    DEBUG_PRINTLN(buffer);
    // delay(10000);
}

float readSoilMoisture(int pin) {
  float rawValue = analogRead(pin);
  DEBUG_PRINT("raw");
  DEBUG_PRINTLN(rawValue);
  
  // Calculate the soil moisture percentage (accounting for inverse relationship)
  float percentage = map(rawValue, VALUEAT0, VALUEAT100, 0, 100);
  //DEBUG_PRINT("pct1");
  //DEBUG_PRINTLN(percentage);
  
  // Constrain the value to make sure it's between 0 and 100
  percentage = constrain(percentage, 0, 100);
  //DEBUG_PRINT("pct");
  //DEBUG_PRINTLN(percentage);
  
  return percentage;
}

float readBat() {
  float rawValue = analogRead(BATTERY_PIN);
  
  // Calculate the soil moisture percentage (accounting for inverse relationship)
  float percentage = map(rawValue, VALUEAT0BAT, VALUEAT100BAT, 0, 100);
  
  // Constrain the value to make sure it's between 0 and 100
  percentage = constrain(percentage, 0, 100);
  
  return percentage;
}

void actuate()
{
  RTCTime currentTime;
  RTC.getTime(currentTime);
  int hours = currentTime.getHour();
  control_light(hours);
  control_temperature(hours);
  control_bottle_sensor();
  control_soil_moisture(hours);
}


// CONTROL SOIL MOISTURE

void control_soil_moisture(int currentTime) {
  float soil_moisture = sensor_data.moisture_1;
  if(soil_moisture > MOISTURE_HIGH_THRESHOLD){
    DEBUG_PRINTLN("Move the plant indoors. Excesive moisture. ");
  }

  //Water only in between 8:00 A.M. and 7:00 P.M.
  else if(currentTime >= 8 && currentTime <= 19) {
    short int cycles = 0;
    float watering_duration = INITIAL_WATERING_DURATION;

    // Adjust watering duration based on humidity
    if (sensor_data.humidity < HUMIDITY_LOW_IDEAL) 
      watering_duration *= 1.15;
    else if (sensor_data.humidity > HUMIDITY_HIGH_IDEAL)
      watering_duration *= 0.85;

    // Principal cycle
    while (soil_moisture < MOISTURE_THRESHOLD && cycles < MAX_WATERING_CYCLES){
      openValve(watering_duration);

      delay(1000*60);  // Wait for 1 minute to allow water to percolate
      measure_sensors();
      soil_moisture = sensor_data.moisture_1;
      cycles += 1;
      if (soil_moisture < MOISTURE_THRESHOLD)
        // Increase watering duration if the soil is still dry
        watering_duration *= 0.85;
    }
  }
}


// CONTROL TEMPERATURE 

void control_temperature(int currentTime) {

    // Handle low temperature
    if (sensor_data.temperature < TEMPERATURE_LOW_IDEAL) {
        handle_low_temperature();
    } else {
      if (!different_day_temp && currentTime == 23)
        cold_days = 0; // Reset cold days counter
    }

    // Handle high temperature
    if (sensor_data.temperature > TEMPERATURE_HIGH_IDEAL) {
        handle_high_temperature();
    } else {
      if (!different_day_temp && currentTime == 23)
        hot_days = 0; // Reset hot days counter
    }
}

void handle_low_temperature() {
    if (sensor_data.temperature < TEMPERATURE_LOW_THRESHOLD) {
        light_up_matrix(MATRIX_NEED_WARM_NOW);
        DEBUG_PRINTLN("Move the plant to a warmer location now.");
        return;
    }

    if (cold_days < MAX_NOT_IDEAL_DAYS && !different_day_temp) {
        cold_days++;
        different_day_temp = true;
    } else {
        light_up_matrix(MATRIX_NEED_WARM);
        DEBUG_PRINTLN("Move the plant to a warmer location.");
    }
}

void handle_high_temperature() {
    if (sensor_data.temperature > TEMPERATURE_HIGH_THRESHOLD) {
        DEBUG_PRINTLN("Move the plant to a cooler location now.");
        light_up_matrix(MATRIX_NEED_COLD_NOW);
        return;
    }

    if (hot_days < MAX_NOT_IDEAL_DAYS && !different_day_temp) {
        hot_days++;
        different_day_temp = true;
    } else {
        DEBUG_PRINTLN("Move the plant to a cooler location.");
        light_up_matrix(MATRIX_NEED_COLD);
    }
}

// CONTROL LIGHT

void control_light(int currentTime) {

    // Handle low light
    if (sensor_data.lux < LUX_LOW_THRESHOLD) {
        handle_low_light(currentTime);
    } else {
      if (currentTime == 23)
        shade_days = 0; // Reset shade days counter 
    }

    // Handle high light
    if (sensor_data.lux > LUX_HIGH_THRESHOLD) {
        handle_high_light(currentTime);
    } else {
      if (currentTime == 23)
        light_days = 0; // Reset light days counter 
    }
}

void handle_low_light(int currentTime) {
    if (shade_days < MAX_NOT_IDEAL_DAYS && currentTime == 23) {
        shade_days++;
    } else {
        //DEBUG_PRINTLN("Move the plant to a lighter location.");
        //light_up_matrix(MATRIX_NEED_SUN);
    }
}

void handle_high_light(int currentTime) {
    if (light_days < MAX_NOT_IDEAL_DAYS && currentTime == 23) {
        light_days++;
    } else {
        //DEBUG_PRINTLN("Move the plant to a location with more shade.");
        //light_up_matrix(MATRIX_NEED_SHADE);
    }
}

// CONTROL BOTTLE SENSOR

void control_bottle_sensor() {
  if (sensor_data.water_level == 0) {
    DEBUG_PRINTLN("Fill the bottle with water. ");
    light_up_matrix(MATRIX_NEED_WATER);
  }

}

void openValve(float watering_duration) {
  myServo.write(VALVE_OPEN_DEGREES);
  valveOpen = true;
  DEBUG_PRINTLN("Valve opened.");
  delay(1000*watering_duration);
  closeValve();
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
  digitalWrite(SENSOR_ENABLE_PIN, LOW);
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
      DEBUG_PRINTLN("Writing sensor data to EEPROM");
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

void enter_low_power_mode(bool wake_up_hour)
{
  // Retrieve the date and time from the RTC and print them
  RTCTime currentTime, alarmTime;
  AlarmMatch alarm, matchTime;
  RTC.getTime(currentTime);
  int minutes, hour;
  if (wake_up_hour == 0){
    hour = currentTime.getHour();
    alarmTime.setHour(hour + 3);
    matchTime.addMatchHour();
    DEBUG_PRINTLN("Dark Times: GreenGuardian will wake up in 3 hours");
  }
  else
  {
    minutes = currentTime.getMinutes();
    alarmTime.setMinute(minutes);
    matchTime.addMatchMinute();
    DEBUG_PRINTLN("Lumos: GreenGuardian will wake up in 1 hour");
  }

  //sets the alarm callback
  RTC.setAlarmCallback(alarmCallback, alarmTime, matchTime);

  delay(500);   // Wait for previous operations to finish
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
 
}

/*******************************************************************************/
/* Main */
/*******************************************************************************/

void setup(){
  matrix.begin();
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
  pinMode(SENSOR_ENABLE_PIN, OUTPUT);
  power_up_sensors();
  setup_sensors();
  // 6. One time setup of low power mode
  setup_low_power_mode();
  DEBUG_PRINTLN("Setup done");
}

void loop()
{
  //sense_and_actuate();
  // 1. TODO Read battery
  sensor_data.battery = (int)readBat();
  bool wake_up_hour = 1;
  if(sensor_data.battery > BATTERY_THRESHOLD) {
    // 2. Sense and actuate
    DEBUG_PRINTLN("Battery ok");
    sense_and_actuate();
    wake_up_hour = 1;

    // 3. Store/Send data
   store_data();
  }
  else if(sensor_data.battery < BATTERY_THRESHOLD && sensor_data.lux > LUX_LOW_THRESHOLD){
  // 2. Sense and actuate
    DEBUG_PRINTLN("Light ok");
    sense_and_actuate();
     // 3. Store/Send data
    store_data();
    wake_up_hour = 1;
  }
  else{
    DEBUG_PRINTLN("Nothing okay");
     wake_up_hour = 0;
     store_data();
  }

  // 4. Enter low power mode
  DEBUG_PRINTLN("Going to sleep zzzzz");
  enter_low_power_mode(wake_up_hour);
  DEBUG_PRINTLN("Back from sleep!");
}
