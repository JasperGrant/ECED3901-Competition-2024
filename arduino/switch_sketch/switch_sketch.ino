#define SW_PIN 12

void setup() {
  pinMode(12, INPUT_PULLUP);
  Serial.begin(9600);


}

void loop() {
  if(digitalRead(SW_PIN) == LOW){
    Serial.print("1\n");
  }
  else{
    Serial.print("2\n");
  }
  delay(1000);

}
