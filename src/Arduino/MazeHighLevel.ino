#include <HCSR04lib.h>  // Include the library for the ultrasonic sensor



//Define all the pins used for the motor

#define mot1A 4
#define mot1B 5  //PWM
#define mot2A 7
#define mot2B 6  //PWM

//Define all the pins used for the interupts
#define intG 2
#define intD 3
#define intProxi 0
State
// Define the pins used for the ultrasonic sensors
#define TRIGGER_PIN_1  9
#define ECHO_PIN_1    10
#define TRIGGER_PIN_2 11
#define ECHO_PIN_2    12


//last time each encopder was triggered
volatile long int lastG = 0;
volatile long int lastD = 0;

//previous value of the integral part of the pi regulator
double I_lastG = 0;
double I_lastD = 0;

//Struct containing the data of the left motor
struct {
  bool dir = 1;             //Direction of the motor
  volatile int Count = 0;   //Number of pulses received from the encoder since the last reset
  volatile int Count_T = 0; //Total number of pulses received from the encoder since the beginning of the program
  float Speed = 0;          //Speed command sent to the motor
  float MSpeed = 0;         //Measured speed of the motor
} motG;

//Struct containing the data of the left motor
struct {
  bool dir = 1;             //Direction of the motor
  volatile int Count = 0;   //Number of pulses received from the encoder since the last reset
  volatile int Count_T = 0; //Total number of pulses received from the encoder since the beginning of the program
  float Speed = 0;          //Speed command sent to the motor
  float MSpeed = 0;         //Measured speed of the motor
} motD;

bool rightWall;             //True if a wall is detected too close on the right
bool frontWall;             //True if a wall is detected too close in front
long rightDistance;         //Distance measured by the ultrasonic sensor on the right
long frontDistance;         //Distance measured by the ultrasonic sensor in front



// Define the sensor object
HCSR04lib sensor1(TRIGGER_PIN_1, ECHO_PIN_1);
HCSR04lib sensor2(TRIGGER_PIN_2, ECHO_PIN_2);

void setup() {

  Serial.begin(9600);

  //Set the pins used for the motors as outputs
  pinMode(mot1A, OUTPUT);
  pinMode(mot1B, OUTPUT);
  pinMode(mot2A, OUTPUT);
  pinMode(mot2B, OUTPUT);

  //attach the interupts to the pins
  attachInterrupt(digitalPinToInterrupt(intG), EncodeurG, CHANGE);
  attachInterrupt(digitalPinToInterrupt(intD), EncodeurD, CHANGE);

  //get the first ditance from the ultrasonics sensors
  rightDistance = sensor1.getDistance();
  frontDistance = sensor2.getDistance();

  //Set the wall variables according to measured distances
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

  //implements the right wall follower algorithm

  while (rightWall && !frontWall) { //If there is a wall on the right and no wall in front keep going straight

    //set the motor to go straight
    RunPIDS(200, 200);
    getspeed();
    move(255 * (motG.Speed / 1500), 255 * (motD.Speed / 1500));


    //Set the wall variables according to measured distances
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


  //if the situation changed, stop the robot and asses the situation
  move(0, 0);

  // if path right -> turn right
  if (!rightWall) {
    //go forward for 15cm
    move(100, 100);
    delay(1000);
    move(0, 0);
    delay(1000);
    //turn right
    Rotate(90,1);
    // go forward for 30 cm
    move(100, 100);
    delay(1000);
    move(0, 0);
  }
  else if (rightWall && frontWall) {    //if there is a wall on the right and in front turn left
    Serial.print("turn left");
    delay(1000);
    Rotate(90,0);
    delay(1000);
  }

  //Set the wall variables according to measured distances
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



//Function linked to the interupt pin connected to the left encorder
void EncodeurG() {
  if (motG.dir) {                 //If the motor is going forward    
    if (millis() - lastG > 10) {  //If the last pulse was received more than 10ms ago (debounce)
      motG.Count++;
      motG.Count_T++;
      lastG = millis();           //Update the last time a pulse was received
    }

  } else {
    if (millis() - lastG > 10) {  //If the last pulse was received more than 10ms ago (debounce)
      motG.Count--;
      motG.Count_T--;
      lastG = millis();           //Update the last time a pulse was received
    }
  }
}


//Function linked to the interupt pin connected to the right encorder
void EncodeurD() {
  if (motD.dir) {                 //If the motor is going forward
    if (millis() - lastD > 10) {  //If the last pulse was received more than 10ms ago (debounce)
      motD.Count++;
      motD.Count_T++;
      lastD = millis();           //Update the last time a pulse was received
    }
  } else {
    if (millis() - lastD > 10) {  //If the last pulse was received more than 10ms ago (debounce)
      motD.Count--;
      motD.Count_T--;
      lastD = millis();           //Update the last time a pulse was received
    }
  }
}


//Function to get the speed of the motors
void getspeed() {
  motG.Count = 0;   //resets the number of pulses received from the encoders
  motD.Count = 0;
  delay(200);       //counts the number of pulses received during 200ms

  motG.MSpeed = motG.Count * 5 * 5.1;   //calculates the speed in mm per second (*5 to get to 1s  *5.1 as 1 pulse equals to 5.1 mm)
  motD.MSpeed = motD.Count * 5 * 5.1;
}


//Function to set the speed of the motors
void move(int left, int right) {
  left = 255 - left;        //inverts the speed as the H-bridge take low pwm for high speed
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


//Function to run the pi regulator
void RunPIDS(float sG, float sD) {

  //sets the PI parameters
  float kiG = 2;
  float kiD = 2;
  float kpg = 0.7;
  float kpd = 0.7;

  float max = 1500;
  float min = -1500;


  //calculates the proportional part of the PI regulator
  float MPG = kpg * (sG - motG.MSpeed);
  float MPD = kpd * (sD - motD.MSpeed);

  //calculates the integral part of the PI regulator
  float MIG = I_lastG + kiG * (sG - motG.MSpeed);
  float MID = I_lastD + kiD * (sD - motD.MSpeed);


  //anti windup
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


  //sets the speed in the motors structs
  motG.Speed = MPG + MIG;
  motD.Speed = MPD + MID;


  //keeps the integral part for the next loop
  I_lastG = MIG;
  I_lastD = MID;


  Serial.println(motG.MSpeed);
  Serial.println(motD.MSpeed);
}

//Function to rotate the robot
void Rotate(float deg, bool dir){
  float ticks = deg*(3.14/180)*122/5.1;   //calculates the number of pulse from the encoder needed to rotate the robot of deg degrees
  if (dir){
    motG.Count = 0;                       //resets the number of pulses received from the encoder
    while (motG.Count < ticks){           //while the robot has not rotated enough
        move(100,0);                      //turns the robot
    }
    move(0,0);                            //stops the robot             
  }
  else{
    motD.Count = 0;
    while (motD.Count < ticks){
        move(0,100);
    }
    move(0,0);
  }

}
