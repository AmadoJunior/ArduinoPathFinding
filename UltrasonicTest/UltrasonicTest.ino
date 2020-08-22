#include <HCSR04.h>
#define TRIG_PIN 12
#define ECHO_PIN 6 

long distance;
UltraSonicDistanceSensor sensor(TRIG_PIN, ECHO_PIN);

void setup() {
    Serial.begin(9600);
}

void loop() {
  distance = sensor.measureDistanceCm();
  Serial.println(distance);
  delay(500);
}
