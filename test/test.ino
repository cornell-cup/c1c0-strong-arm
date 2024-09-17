
#define PIN 8
#define PIN2 9
void setup(){
  pinMode(PIN,OUTPUT);
  pinMode(PIN2,OUTPUT);
}

void loop(){
  //digitalWrite(PIN,LOW);
  //delay(2000);
  digitalWrite(PIN,HIGH);
  digitalWrite(PIN2,HIGH);
  //delay(2000);
}