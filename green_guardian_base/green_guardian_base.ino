#include <Servo.h>
#include <DHT.h>
#include <Wire.h>
#include <BH1750_WE.h>
// Pins
#define BH1750_ADDRESS 0x23
#define DHTPIN A2
#define DHTTYPE DHT11
#define WATER_LEVEL_PIN 7
#define MOISTURE_PIN A0
// Valve constants
#define VALVE_OPEN_DEGREES 10
#define VALVE_CLOSED_DEGREES 100
// Sensor objects
BH1750_WE myBH1750(BH1750_ADDRESS);
DHT dht(DHTPIN, DHTTYPE);
Servo myServo;
// Global state variables
bool valveOpen = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup started");

  myServo.attach(3);
  dht.begin();
  Wire.begin();
  if (!myBH1750.init()) {
    Serial.println("BH1750 Connection Failed!");
  }
  pinMode(WATER_LEVEL_PIN, INPUT);
  closeValve();  // Ensure the valve is closed on startup
}

void openValve() {
  myServo.write(VALVE_OPEN_DEGREES);
  valveOpen = true;
  Serial.println("Valve opened.");
}

void closeValve() {
  myServo.write(VALVE_CLOSED_DEGREES);
  valveOpen = false;
  Serial.println("Valve closed.");
}

bool isValveOpen() {
  return valveOpen;
}

void loop() {
  unsigned long currentTime = millis();
  int moisture = analogRead(0);
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float lightIntensity = myBH1750.getLux();
  int waterOnLevel = digitalRead(WATER_LEVEL_PIN);


  char dataString[200];  // Buffer for the data string
  sprintf(dataString, "%lu ms, Moist: %.2i , Temp: %.2f °C, Hum: %.2f %%, Light: %.2f Lux, Lvl: %.2i ",
           currentTime, moisture, temperature, humidity, lightIntensity, waterOnLevel);

  if (isnan(moisture) || isnan(temperature) || isnan(humidity) || isnan(lightIntensity) || isnan(waterOnLevel)) {
    Serial.println("Failed to read sensors!");
  } else {
    Serial.println(dataString);
  }


  delay(1500);
}
