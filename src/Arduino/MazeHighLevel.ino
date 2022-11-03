
#include <HCSR04lib.h>

#include <PID_v2.h>

#define intG 2
#define intD 3
#define mot1A 4
#define mot1B 5  //PWM
#define mot2A 7
#define mot2B 6  //PWM
#define intProxi 0

volatile long int lastG = 0;
volatile long int lastD = 0;

double I_lastG = 0;
double I_lastD = 0;


enum {
  Running,
  Stopped,
  Turning
} State;

struct {
  bool dir = 1;
  volatile int Count = 0;
  volatile int Count_T = 0;
  float Speed = 0;
  float MSpeed = 0;
} motG;

struct {
  bool dir = 1;
  volatile int Count = 0;
  volatile int Count_T = 0;
  float Speed = 0;
  float MSpeed = 0;
} motD;

double InputSpeedG, OutputSpeedG, SetpointSpeedG;
double InputSpeedD, OutputSpeedD, SetpointSpeedD;

bool rightWall;
bool frontWall;
long rightDistance;
long frontDistance;

// Define the pins used for the sensors
#define TRIGGER_PIN_1  9
#define ECHO_PIN_1    10
#define TRIGGER_PIN_2 11
#define ECHO_PIN_2    12

// Define the sensor object
HCSR04lib sensor1(TRIGGER_PIN_1, ECHO_PIN_1);
HCSR04lib sensor2(TRIGGER_PIN_2, ECHO_PIN_2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(mot1A, OUTPUT);
  pinMode(mot1B, OUTPUT);
  pinMode(mot2A, OUTPUT);
  pinMode(mot2B, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(intG), EncodeurG, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intD), EncodeurD, CHANGE);

  rightDistance = sensor1.getDistance();
  frontDistance = sensor2.getDistance();
  if (rightDistance < 15) {
    rightWall = 1;
  }
  else if (rightDistance < 1000) {
    rightWall = 0;
  }
  if (frontDistance < 15 ) {
    frontWall = 1;
  }
  else if (frontDistance < 1000) {
    frontWall = 0;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  while (rightWall && !frontWall) {
    RunPIDS(200, 200);
    getspeed();
    move(255 * (motG.Speed / 1500), 255 * (motD.Speed / 1500));

    rightDistance = sensor1.getDistance();
    frontDistance = sensor2.getDistance();
    if (rightDistance < 15) {
      rightWall = 1;
    }
    else if (rightDistance < 1500 && rightDistance >= 15) {
      rightWall = 0;
    }
    if (frontDistance < 15) {
      frontWall = 1;
    }
    else if (frontDistance < 1500 && frontDistance >= 15) {
      frontWall = 0;
    }
  }

  move(0, 0);

  // if path right -> turn right
  if (!rightWall) {
    //avancer de 15cm
    move(100, 100);
    delay(1000);
    move(0, 0);
    delay(1000);
    Rotate(90,1);
    // avancer de 30 cm
    move(100, 100);
    delay(1000);
    move(0, 0);
  }
  else if (rightWall && frontWall) {
    Serial.print("turn left");
    delay(1000);
    Rotate(90,0);
    delay(1000);
  }
  rightDistance = sensor1.getDistance();
  frontDistance = sensor2.getDistance();

  if (rightDistance < 15) {
    rightWall = 1;
  }
  else if (rightDistance < 1500 && rightDistance >= 15) {
    rightWall = 0;
  }
  if (frontDistance < 15) {
    frontWall = 1;
  }
  else if (frontDistance < 1500 && frontDistance >= 15) {
    frontWall = 0;
  }

  delay(1000);
}

void EncodeurG() {
  if (motG.dir) {
    if (millis() - lastG > 10) {
      motG.Count++;
      motG.Count_T++;
      lastG = millis();
    }

  } else {
    motG.Count--;
  }
}

void EncodeurD() {
  if (motD.dir) {
    if (millis() - lastD > 10) {
      motD.Count++;
      motD.Count_T++;
      lastD = millis();
    }
  } else {
    motD.Count--;
  }
}

void getspeed() {
  motG.Count = 0;
  motD.Count = 0;
  delay(200);

  motG.MSpeed = motG.Count * 5 * 5.1;
  motD.MSpeed = motD.Count * 5 * 5.1;

  InputSpeedG = motG.MSpeed;
  InputSpeedD = motD.MSpeed;
}

int CalculateDistance(int count) {
  return count * 5;
}

void move(int left, int right) {
  left = 255 - left;
  right = 255 - right;
  if (left == 0) {
    left = 1;
  }
  if (right == 0) {
    right = 1;
  }
  if (left > 0) {
    //motG.dir = 1;
    digitalWrite(mot1A, HIGH);
    analogWrite(mot1B, left);
  } else {
    //motG.dir = 0;
    digitalWrite(mot1A, LOW);
    analogWrite(mot1B, -left);
  }
  if (right > 0) {
    //motD.dir = 1;
    digitalWrite(mot2A, HIGH);
    analogWrite(mot2B, right);
  } else {
    //motD.dir = 0;
    digitalWrite(mot2A, LOW);
    analogWrite(mot2B, -right);
  }
}

void RunPIDS(float sG, float sD) {
  InputSpeedG = motG.MSpeed;
  InputSpeedD = motD.MSpeed;

  float kiG = 2;
  float kiD = 2;
  float kpg = 0.7;
  float kpd = 0.7;

  float max = 1500;
  float min = -1500;

  float MPG = kpg * (sG - motG.MSpeed);
  float MPD = kpd * (sD - motD.MSpeed);

  float MIG = I_lastG + kiG * (sG - motG.MSpeed);
  float MID = I_lastD + kiD * (sD - motD.MSpeed);

  if (MPG + MIG > max) {
    MIG = max - MPG;
  }
  else if (MPG + MIG < min) {
    MIG = min - MPG;
  }

  if (MPD + MID > max) {
    MID = max - MPD;
  }
  else if (MPD + MID < min) {
    MID = min - MPD;
  }

  motG.Speed = MPG + MIG;
  motD.Speed = MPD + MID;

  I_lastG = MIG;
  I_lastD = MID;

  // motG.Speed = kpg*(setpoint-motG.MSpeed)+I_lastG+kiG*(setpoint-motG.MSpeed);
  // motD.Speed = kpd*(setpoint-motD.MSpeed)+I_lastD+kiD*(setpoint-motD.MSpeed);
  Serial.println("aaa");
  Serial.println(motG.MSpeed);
  Serial.println(motD.MSpeed);
  // I_lastG = I_lastG+kiG*(setpoint-motG.MSpeed);
  // I_lastD = I_lastD+kiD*(setpoint-motD.MSpeed);



  // if (motG.Speed >1500){motG.Speed = 1500;}
  // if (motD.Speed >1500){motD.Speed = 1500;}

  //motG.Speed = OutputSpeedG;
  //motD.Speed = OutputSpeedD;
}
//
//void Rotate(float deg) {
//  float ticks = abs(deg) * (3.14 / 180) * 122 / 5.1;
//  if (deg > 0) {
//    motG.Count = 0;
//    while (motG.Count < ticks) {
//      move(100, 0);
//    }
//    move(0, 0);
//  }
//  if (deg < 0) {
//    motD.Count = 0;
//    while (motD.Count < ticks) {
//      move(0, 100);
//    }
//    move(0, 0);
//  }
//}
void Rotate(float deg, bool dir){
  float ticks = deg*(3.14/180)*122/5.1;
  if (dir){
    motG.Count = 0;
    while (motG.Count < ticks){
        move(100,0);
    }
    move(0,0);
  }
  else{
    motD.Count = 0;
    while (motD.Count < ticks){
        move(0,100);
    }
    move(0,0);
  }

}
