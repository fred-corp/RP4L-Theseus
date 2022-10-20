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
  int Speed = 0;
} motG;

struct {
  bool dir = 1;
  volatile int Count = 0;
  int Speed = 0;
} motD;


double InputSpeed, OutputSpeed, SetpointSpeed;
double InputRotation, OutputRotation, SetpointRotation;

unsigned long start, stop;

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

}

void loop() {
  // // put your main code here, to run repeatedly:

  // delay(4000);
  // move(150,150);
  // delay(300);
  // move(150,150);
  // delay(1000);
  // getspeed();
  // Serial.print("slow G,D: ");
  // Serial.println(motG.Speed);
  // Serial.println(motD.Speed);
  
  // move(50,50);
  // delay(1000);
  // getspeed();
  // Serial.print("fast G,D: ");
  // Serial.println(motG.Speed);
  // Serial.println(motD.Speed);
  delay(2000);
  move(1,1);
  delay(1000);
  getspeed();
    Serial.print("fast G,D: ");
  Serial.println(motG.Speed);
  Serial.println(motD.Speed);
  move(255,255);
  delay(4000);
  // Serial.print("spped G,D: ");
  // move(255,255);
  // delay(5000);
  // Serial.print("spped G,D: ");
  // move(150,150);
  // delay(5000);
  // Serial.print("spped G,D: ");
  // move(50,50);
  // delay(5000);
  // Serial.print("spped G,D: ");
  // move(0,0);
  // delay(5000);
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
  motG.Speed = motG.Count*5*5.1;
  motD.Speed = motD.Count*5*5.1;
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

