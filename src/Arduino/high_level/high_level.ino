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

//void loop() {
  // if (safe){
  //   runPIDS();
  //   move(motG.Speed,motD.Speed);
  // }
  // else{
  //   Rotate(180);
  // }
//}


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