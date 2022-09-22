#define mot1A 2
#define mot1B 3 //PWM
#define mot2A 4
#define mot2B 5 //PWM

void setup(){
  pinMode(mot1A, OUTPUT);
  pinMode(mot1B, OUTPUT);
  pinMode(mot2A, OUTPUT);
  pinMode(mot2B, OUTPUT);
}

void loop() {
  move(127, 127);
  delay(1000);
  move(-127, -127);
  delay(1000);
}

void move(int left, int right){
  if(left > 0){
    digitalWrite(mot1A, HIGH);
    analogWrite(mot1B, left);
  }else{
    digitalWrite(mot1A, LOW);
    analogWrite(mot1B, -left);
  }
  if(right > 0){
    digitalWrite(mot2A, HIGH);
    analogWrite(mot2B, right);
  }else{
    digitalWrite(mot2A, LOW);
    analogWrite(mot2B, -right);
  }
}
