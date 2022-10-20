#include <HCSR04.h>

#include <PID_v2.h>

// C++ code
//

#define intG 2
#define intD 3
#define mot1A 4
#define mot1B 5  //PWM
#define mot2A 7
#define mot2B 6  //PWM
#define intProxi 0

#define PCICR_PORTD_MASK (1 << 1)
#define PCICR_PORTD_CLEAR_FLAG (~(1 << 1))
#define PCMSK2_PCINT23_ENALBE_MASK (1 << 0)

enum {
Running,
Stopped,
Turning
}State;

struct {
  bool WallForward;
  bool WallLeft;
  bool WallRight;
} Data;

struct {
  HCSR04 hc,
  int last_time,
  int dist,
} HC;

struct {
  bool dir = 1;
  volatile int Count = 0;
  int Speed = 0;
} motG;

struct {
  bool dir = 1;
  volatile int Count = 0;
  int Speed = 0;
} motD;

double InputSpeed, OutputSpeed, SetpointSpeed;
double InputRotation, OutputRotation, SetpointRotation;

PID PIDSpeed(&InputSpeed, &OutputSpeed, &SetpointSpeed, 2, 0, 0, DIRECT);

PID PIDRotation(&InputRotation, &OutputRotation, &SetpointRotation, 2, 0, 0, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(mot1A, OUTPUT);
  pinMode(mot1B, OUTPUT);
  pinMode(mot2A, OUTPUT);
  pinMode(mot2B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(intG), EncodeurG, RISING);
  attachInterrupt(digitalPinToInterrupt(intD), EncodeurD, RISING);
  //attachInterrupt(digitalPinToInterrupt(intD), EncodeurD, RISING);

  //interupt
  sei(); // Enable interrupts

  PCICR |= PCICR_PORTD_MASK; // Enable external interrupt on PORTD
  PCIFR &= PCICR_PORTD_CLEAR_FLAG; // Clear interrupt flag of PORTD
  PCMSK1 |= PCMSK2_PCINT23_ENALBE_MASK; // Enable interrupt on pin 7 (PCINT23)

  //PID
  PIDSpeed.SetSampleTime(200);
  PIDSpeed.SetOutputLimits(0, 255);

  PIDRotation.SetSampleTime(200);
  PIDRotation.SetOutputLimits(0, 255);


}

void loop() {
  move(127, 127);
  delay(1000);
  move(0, 0);
  Serial.println(CalculateDistance(motG.Count));
  Serial.println(CalculateDistance(motD.Count));
  delay(10000);
}


ISR(PCINT2_vect) {
  PCIFR &= PCICR_PORTD_CLEAR_FLAG;  // Clear interrupt flag of PORTD

  digitalRead(LED_13) ? digitalWrite(LED_13, LOW) : digitalWrite(LED_13, HIGH);
}

  
void EncodeurG() {
  if (motG.dir) {
    motG.Count++;
  } else {
    motG.Count--;
  }
}

void EncodeurD() {
  if (motD.dir) {
    motD.Count++;
  } else {
    motD.Count--;
  }
}

int CalculateDistance(int count) {
  return count * 5;
}

void move(int left, int right) {
  if (left > 0) {
    motG.dir = 1;
    digitalWrite(mot1A, HIGH);
    analogWrite(mot1B, left);
  } else {
    motG.dir = 0;
    digitalWrite(mot1A, LOW);
    analogWrite(mot1B, -left);
  }
  if (right > 0) {
    motD.dir = 1;
    digitalWrite(mot2A, HIGH);
    analogWrite(mot2B, right);
  } else {
    motD.dir = 0;
    digitalWrite(mot2A, LOW);
    analogWrite(mot2B, -right);
  }
}

void RunPIDS() {
  InputSpeed = motG.Count + motD.Count;
  InputRotation = motG.Count - motD.Count;
  PIDSpeed.Compute();
  PIDRotation.Compute();

  motG.Speed = (OutputSpeed + OutputRotation) / 2;
  motD.Speed = (OutputSpeed - OutputRotation) / 2;
}

void getDistance(HC hc){
  if (millis()- hc.last_time > 60){
    hc.last_time = millis();
    hc.dist = hc.hc.dist();
  }
}