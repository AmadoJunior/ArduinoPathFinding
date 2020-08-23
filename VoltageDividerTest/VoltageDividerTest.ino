
void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  float percentage  = voltage / 5;
  float outOf9v = 9 * (percentage + 0.06);
  Serial.println(outOf9v);
  delay(2000);
}
