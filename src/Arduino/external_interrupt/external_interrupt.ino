#define OUT_8 (8)
#define LED_13 (13)

#define PCICR_PORTD_MASK (1 << 1)
#define PCICR_PORTD_CLEAR_FLAG (~(1 << 1))
#define PCMSK2_PCINT23_ENALBE_MASK (1 << 0)

ISR(PCINT1_vect) {
  PCIFR &= PCICR_PORTD_CLEAR_FLAG;  // Clear interrupt flag of PORTD

  digitalRead(LED_13) ? digitalWrite(LED_13, LOW) : digitalWrite(LED_13, HIGH);
  Serial.println("aaaa");


}

void setup() {
  Serial.begin(9600);
  pinMode(OUT_8, OUTPUT);
  pinMode(LED_13, OUTPUT);
  pinMode(7, INPUT);

  sei(); // Enable interrupts

  PCICR |= PCICR_PORTD_MASK; // Enable external interrupt on PORTD
  PCIFR &= PCICR_PORTD_CLEAR_FLAG; // Clear interrupt flag of PORTD
  PCMSK1 |= PCMSK2_PCINT23_ENALBE_MASK; // Enable interrupt on pin 7 (PCINT23)
}

void loop() {
  digitalWrite(OUT_8, HIGH);
  delay(500);
  digitalWrite(OUT_8, LOW);
  delay(500);
}
