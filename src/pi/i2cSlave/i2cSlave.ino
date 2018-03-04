// connect motor controller pins to Arduino digital pins
// motor A - right tread
int enA = 5; //9
int inA1 = 7;
int inA2 = 6;
// motor B - left tread
int enB = 3; //3
int inB3 = 8;
int inB4 = 4;

//wire configs
int address = 8;

//motor configs
int motorSpeed = 80;

//timeout configs
long timeout = 0;
long maxTimeout = 1000; //how long tank will run without new input before stopping

/*
 * MOVEMENT
 */

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
}

/*
 * Set motor speed increments
 */

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
  Wire.begin(address);          // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Wire.onRequest(sendData);
  Serial.begin(9600);           // start serial for output
}


void loop() {

  //receiveEvent(10);

  //turns off motor if maxTimeout ms has passed since last input
  if(millis() - timeout > maxTimeout){
    motorOff();
  }
}


char c = ' ';
// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {

  while (Wire.available()) { // loop through all 
    c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
    
    //controls: WASD
    if(c == 'w'){
      forwards(motorSpeed);
    } else if(c == 'a') {
      left(motorSpeed);
    } else if(c == 's') {
      backwards(motorSpeed);
    } else if(c == 'd') {
      right(motorSpeed);
    } else if(c == ' ') {
      motorOff();
    }

    timeout = millis();
  }
}

// callback for sending data
void sendData(){
  Wire.write(c);
}


