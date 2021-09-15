int a,b = 0;

#define PIN7 7
void setup() {
  pinMode(PIN7, OUTPUT);
}

void loop() {
  while (a < 1) {
    digitalWrite(PIN7, HIGH);
    delay(1000);
    digitalWrite(PIN7, LOW);
    delay(1000);
    a++;
  }
  
  while (b < 5) {
    digitalWrite(PIN7, HIGH);
    delay(100);
    digitalWrite(PIN7, LOW);
    delay(100);
    b++;
  }
  digitalWrite(PIN7, HIGH);
  while (1) {}
}
