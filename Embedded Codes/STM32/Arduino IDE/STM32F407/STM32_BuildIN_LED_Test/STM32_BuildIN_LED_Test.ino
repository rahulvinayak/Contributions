#define led PB1 // Buildin LED PIN Number Custom Board

void setup() {

  pinMode(led,OUTPUT); //Defined pin as output
}

void loop() {
  // Code to Turn on builtin LED ON and OFF at Interval of 1 sec
  digitalWrite(led,HIGH);
  delay(1000);
  digitalWrite(led,LOW);
  delay(1000);

}
