int batteryPin = A0;
float voltage = 0;

void setup() {
  pinMode(batteryPin, INPUT);
}

void loop() {
  voltage = analogRead(batteryPin)/1016.4*16.8;
  Serial.println(voltage);
}
