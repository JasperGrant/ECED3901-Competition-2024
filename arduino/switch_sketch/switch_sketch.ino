#define SW_PIN 12

void setup() {
  pinMode(12, INPUT_PULLUP);
  Serial.begin(9600);


}

void loop() {
  if(digitalRead(SW_PIN) == LOW){
    Serial.print(1);
  }
  else{
    Serial.print(0);
  }
  delay(1000);

}
