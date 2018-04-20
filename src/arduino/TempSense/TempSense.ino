#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN            2         // Pin which is connected to the DHT sensor.
#define RELAYPIN 3
#define ESPPIN 4

// Uncomment the type of sensor in use:
#define DHTTYPE           DHT11     // DHT 11 

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

uint32_t upperLimit = 30;
uint32_t lowerlimit = 25;
bool powerStatus = false;

void setup() {
  Serial.begin(9600); 
  // Initialize device.
  dht.begin();
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.println("Temperature");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" *C");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" *C");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" *C");  
  Serial.println("------------------------------------");
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

  // Initilize relay Output
  pinMode(RELAYPIN, OUTPUT);
  digitalWrite(RELAYPIN, HIGH);

  // Initilize ESP INPUT
  pinMode(ESPPIN, INPUT);
}

void loop() {
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println("Error reading temperature!");
  }
  else {
    uint32_t temperature = event.temperature;
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" *C");

    if(temperature < lowerlimit){
      powerStatus = false;
    } else if(temperature > upperLimit){
      powerStatus = true;
    }
  }
  
  // turn Relay on when dht11 Temperature is higher then 25
  if(powerStatus && digitalRead(ESPPIN) == HIGH){
    // relay Pin has to be low if relay has to switch electrical ports
    digitalWrite(RELAYPIN, LOW);
  } else {
    digitalWrite(RELAYPIN, HIGH);
  }

}
