#define OUT_8 (8)
#define LED_13 (13)

#define PCICR_PORTD_MASK (1 << 2)
#define PCICR_PORTD_CLEAR_FLAG (~(1 << 2))
#define PCMSK2_PCINT23_ENALBE_MASK (1 << 7)

ISR(PCINT2_vect) {
  PCIFR &= PCICR_PORTD_CLEAR_FLAG;  // Clear interrupt flag of PORTD

  digitalRead(LED_13) ? digitalWrite(LED_13, LOW) : digitalWrite(LED_13, HIGH);
}

void setup() {
  pinMode(OUT_8, OUTPUT);
  pinMode(LED_13, OUTPUT);
  pinMode(7, INPUT);

  sei(); // Enable interrupts

  PCICR |= PCICR_PORTD_MASK; // Enable external interrupt on PORTD
  PCIFR &= PCICR_PORTD_CLEAR_FLAG; // Clear interrupt flag of PORTD
  PCMSK2 |= PCMSK2_PCINT23_ENALBE_MASK; // Enable interrupt on pin 7 (PCINT23)
}

void loop() {
  digitalWrite(OUT_8, HIGH);
  delay(500);
  digitalWrite(OUT_8, LOW);
  delay(500);
}
