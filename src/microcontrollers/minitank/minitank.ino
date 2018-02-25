// connect motor controller pins to Arduino digital pins
// motor A - right tread
int enA = 9;
int inA1 = 7;
int inA2 = 6;
// motor B - left tread
int enB = 3;
int inB3 = 5;
int inB4 = 4;

void setup()
{
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB3, OUTPUT);
  pinMode(inB4, OUTPUT);
}

// Move forwards
void forwards(int speed) { 
  // set speed out of possible range 0~255
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  // turn on motor A 
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);
    // turn on motor B
  digitalWrite(inB3, HIGH);
  digitalWrite(inB4, LOW);
  delay(2000);
}

// Move backwards 
void backwards(int speed) {  
  // set speed 
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  // motor A - backwards
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);  
  // motor B - forward
  digitalWrite(inB3, LOW);
  digitalWrite(inB4, HIGH); 
  delay(2000);
}

// Turn left 
void left(int speed) {
  // set speed 
  analogWrite(enA, speed);
  analogWrite(enB, speed);  
  // motor A - forwards
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, LOW);  
  // motor B - forward
  digitalWrite(inB3, LOW);
  digitalWrite(inB4, HIGH); 
  delay(2000);  
}  
  
// Turn right 
void right(int speed) {  
  // set speed 
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  // motor A - backwards
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);  
  // motor B - forward
  digitalWrite(inB3, HIGH);
  digitalWrite(inB4, LOW); 
  delay(2000);
}

// Set motor speed increments

// accelerate from zero to maximum speed
void maxAccel(int speed) {
  for (int i = 0; i < 256; i++)
  {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  } 
}

// decelerate from maximum speed to zero
void maxDecel(int speed) {
  for (int i = 255; i >= 0; --i)
  {
    analogWrite(enA, i);
    analogWrite(enB, i);
    delay(20);
  } 
}

void motorOff() {  
  // now turn off motors
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, LOW);  
  digitalWrite(inB3, LOW);
  digitalWrite(inB4, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  right(100);
  delay(1000);
  forwards(150);
  delay(1000);
  left(200);
  delay(1000);
  backwards(50);
  delay(1000);  
  motorOff();
  delay(1000);
}
