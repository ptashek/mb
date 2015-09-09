int D_PIN = 13;
int freq = 53;

void setup() {
  pinMode(D_PIN, OUTPUT);
}

void loop() {
  int state = 0;
  while (1) {
    if (state == LOW) { 
      digitalWrite(D_PIN, LOW); 
      state = HIGH; 
    } else { 
      digitalWrite(D_PIN, HIGH); 
      state = LOW; 
   } 
   delay(1000 / freq);
  }
}
