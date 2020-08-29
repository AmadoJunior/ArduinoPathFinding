const int GPIO1 = 2;
void setup() {
  // put your setup code here, to run once:
  digitalWrite(GPIO1, LOW);
}

void loop() {
  digitalWrite(GPIO1, HIGH);
  delay(2000);
  digitalWrite(GPIO1, LOW);
  delay(2000);
}
