void blip() {
  digitalWrite(BUZZER, HIGH);
  delay(50);
  digitalWrite(BUZZER, LOW);
}

void shortBeep() {
  digitalWrite(BUZZER, HIGH);
  delay(250);
  digitalWrite(BUZZER, LOW);
  
}

void longBeep() {
  digitalWrite(BUZZER, HIGH);
  delay(750);
  digitalWrite(BUZZER, LOW);
  
}
