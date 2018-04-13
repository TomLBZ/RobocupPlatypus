void setup() {
  Serial.begin(115200);

}

void loop() {
  while (true) {
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
  }
}
