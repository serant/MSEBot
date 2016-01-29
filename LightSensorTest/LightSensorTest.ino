const unsigned ci_Light_Sensor = A3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(ci_Light_Sensor));
}
