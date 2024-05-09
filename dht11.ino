#include <DHT11.h>

#define DHT11_PIN           7
#define HUMIDITY_OFFSET     3
#define TEMPERATURE_OFFSET  -2

DHT11 DHT(DHT11_PIN);
int temperature, humidity;

void setup(){
  Serial.begin(9600);
}

void loop(){
  Serial.print("Temperature = ");
  temperature = DHT.readTemperature() + TEMPERATURE_OFFSET;
  Serial.println(temperature);

  Serial.print("Humidity = ");
  humidity = DHT.readHumidity() + HUMIDITY_OFFSET;
  Serial.println(humidity);
  
  delay(1000);
}