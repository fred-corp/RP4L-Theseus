#include <PID_v2.h>

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
  bool dir = 1;
  volatile int Count = 0;
  float Speed = 0;
  float MSpeed = 0; 
} motG;

struct {
  bool dir = 1;
  volatile int Count = 0;
  int Speed = 0;
  float MSpeed = 0;
} motD;

double InputSpeed, OutputSpeed, SetpointSpeed;
double InputRotation, OutputRotation, SetpointRotation;

PID PIDSpeed(&InputSpeed, &OutputSpeed, &SetpointSpeed, 3, 0, 0, PID::Reverse);

PID PIDRotation(&InputRotation, &OutputRotation, &SetpointRotation, 3, 0, 0, PID::Reverse);

void setup() {
  // put your setup code here, to run once:
  //PID
  Serial.begin(9600);
  pinMode(mot1A, OUTPUT);
  pinMode(mot1B, OUTPUT);
  pinMode(mot2A, OUTPUT);
  pinMode(mot2B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(intG), EncodeurG, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intD), EncodeurD, CHANGE);

  PIDSpeed.SetSampleTime(200);
  PIDSpeed.SetOutputLimits(0, 700);

  PIDRotation.SetSampleTime(200);
  PIDRotation.SetOutputLimits(0, 700);
  PIDRotation.SetMode(PID::Automatic);
  PIDSpeed.SetMode(PID::Automatic);
  SetpointSpeed = 600;
  SetpointRotation = 0;

}

void loop() {
  // put your main code here, to run repeatedly:
    //PID
    RunPIDS();
    getspeed();
    move(255*(motG.Speed/688),255*(motD.Speed/580));


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

void getspeed(){
  motG.Count=0;
  motD.Count=0;
  delay(200);
  motG.MSpeed = motG.Count*5*5.1;
  motD.MSpeed = motD.Count*5*5.1;
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
  InputSpeed = motG.MSpeed + motD.MSpeed;
  InputRotation = motG.MSpeed - motD.MSpeed;
  
  PIDSpeed.Compute();
  PIDRotation.Compute();

  motG.Speed = (OutputSpeed + OutputRotation) / 2;
  motD.Speed = (OutputSpeed - OutputRotation) / 2;
}
