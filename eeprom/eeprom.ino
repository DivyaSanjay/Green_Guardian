#include <EEPROM.h>

// Size of structure: 32 bytes
struct {
  time_t  unix_time;    // Timestamp 8 bytes
  float   temperature;  // 4 bytes
  float   humidity;     // 4 bytes
  float   lux;          // 4 bytes
  int     water_level;  // 4 bytes
  short int   moisture_1;     // 2 bytes
  short int   moisture_2;     // 2 bytes
  int   battery;      // 4 bytes
} sensor_data;

void setup() {
  Serial.begin(9600);
  while (!Serial) {

  }
  // wait before printing
  delay(1000);

  int data_length = EEPROM.read(0);
  Serial.print("Address 0: ");
  Serial.println(data_length);

  int data_addr;
  char buffer[200];
  for(int i = 0; i < data_length; i++)
  {
      data_addr = (i + 1) * sizeof(sensor_data);
      EEPROM.get(data_addr, sensor_data);
      sprintf(buffer, "[%ld, %f, %f, %f, %d, %d, %d, %d]", sensor_data.unix_time, sensor_data.temperature, sensor_data.humidity, sensor_data.lux,
        sensor_data.water_level, sensor_data.moisture_1, sensor_data.moisture_2, sensor_data.battery);
      Serial.println(buffer);
  }
  // Clear index
 //  EEPROM.write(0, 0);
}

void loop() {
}