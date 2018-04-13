void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
}

void loop() {
  delay(20);
  Serial.println(analogRead(0));

}
