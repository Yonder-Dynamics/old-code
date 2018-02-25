// connect motor controller pins to Arduino digital pins
// motor A - right tread
int enA = 9;
int inA1 = 7;
int inA2 = 6;
// motor B - left tread
int enB = 3;
int inB3 = 5;
int inB4 = 4;

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

// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>

void setup() {
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB3, OUTPUT);
  pinMode(inB4, OUTPUT);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(9600);           // start serial for output
}

void loop() {
  delay(10);
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  while (Wire.available()) { // loop through all 
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
    if(c == 'g'){
      forwards(100);
    } else if(c == 's'){
      motorOff();
    }
  }
}


