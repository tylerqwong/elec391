void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
}

void loop() {
  int val = analogRead(A0);             // A0 is pin14 on the board
  Serial.print("analog A0 is: ");
  Serial.println(val*3.3/1023);
  delay(250);
}
