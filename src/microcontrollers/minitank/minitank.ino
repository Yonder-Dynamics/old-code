/* Author: Karen, Simon Fong
 * 
 */

// Connect motor controller pins to Arduino digital pins
// Motor A - right tread
int enA = 9;
int inA1 = 7;
int inA2 = 6;

// Motor B - left tread
int enB = 3;
int inB3 = 5;
int inB4 = 4;

class Motor{
  /* Abstracts motor pins to methods for easier use.
   * 
   */
public:
  Motor(int a_direction_pin_1, int a_direction_pin_2, int an_enable_pin){
    direction_pin_1 = a_direction_pin_1;
    direction_pin_2 = a_direction_pin_2;
    enable_pin = an_enable_pin;

    pinMode(direction_pin_1, OUTPUT);
    pinMode(direction_pin_2, OUTPUT);
    pinMode(enable_pin, OUTPUT);
  }

  void set_clockwise(){
    /* Sets the motor to be ready to turn clockwise. */
    if(!inverted){
      digitalWrite(direction_pin_1, HIGH);
      digitalWrite(direction_pin_2, LOW);
    }else{
      digitalWrite(direction_pin_1, LOW);
      digitalWrite(direction_pin_2, HIGH);
    }
  }

  void set_anti_clockwise(){
    /* Sets the motor to be ready to turn anti-clockwise. */
    if(!inverted){
      digitalWrite(direction_pin_1, LOW);
      digitalWrite(direction_pin_2, HIGH);
    }else{
      digitalWrite(direction_pin_1, HIGH);
      digitalWrite(direction_pin_2, LOW);
    }
  }

  void set_speed(int pwm_length){
    /* Writes the voltage to the motors using PWM.
     * @param pwm_length An int from 0-255 which represents voltages from 0 - (Max Voltage)
     */
    analogWrite(enable_pin, pwm_length);
  }

  void set_off(){
    digitalWrite(direction_pin_1, LOW);
    digitalWrite(direction_pin_2, LOW);
    analogWrite(enable_pin, 0);
  }

  void set_inverted(){
    /* Inverts the clockwise and anti-clockwise commands in case the wires are mis-wired. */
    inverted = true;
  }
private:
  int direction_pin_1;  // Controls which direction wheels will turn.
  int direction_pin_2;  // Controls which direction wheels will turn.
  int enable_pin;       // Controls the speed of the motor through PWM.
  int inverted = false; // Controls whether not to invert the direction.

};

// Declare motor objects.
Motor motor_right(inA1,inA2,enA);
Motor motor_left(inB3, inB4, enB);



void setup()
{
  motor_left.set_inverted(); // Only inverted because wires are currently inverted.
}

// Move forwards
void forwards(int aSpeed, int aTime) { 
  // Set speed out of possible range 0~255
  motor_right.set_speed(aSpeed);
  motor_left.set_speed(aSpeed);
  
  // Turn on left motor
  motor_right.set_clockwise();
  // Turn on right motor
  motor_left.set_anti_clockwise();

  // Wait some time, then turn off.
  delay(aTime);
  motorOff();
}

// Move backwards 
void backwards(int aSpeed, int aTime) {  
  // Set speed 
  motor_right.set_speed(aSpeed);
  motor_left.set_speed(aSpeed);
  
  // Right motor - backwards
  motor_right.set_anti_clockwise();
  // Left motor - forward
  motor_left.set_clockwise();
  
  delay(aTime);
  motorOff();
}

// Turn left 
void left(int aSpeed, int aTime) {
  // Set speed 
  motor_right.set_speed(aSpeed);
  motor_left.set_speed(aSpeed);  
  
  // Right motor - forwards
  motor_right.set_clockwise();
  // Left motor - backward
  motor_left.set_clockwise();

  delay(aTime);
  motorOff();
}  
  
// Turn right 
void right(int aSpeed, int aTime) {  
  // Set speed 
  motor_right.set_speed(aSpeed);
  motor_left.set_speed(aSpeed);
  
  // Right motor - backwards
  motor_right.set_anti_clockwise(); 
  // motor B - forward
  motor_left.set_clockwise();

  delay(aTime);
  motorOff();
}

// Set motor speed increments

// accelerate from zero to maximum speed
void maxAccel() {
  for (int i = 0; i < 256; i++)
  {
    motor_right.set_speed(i);
    motor_left.set_speed(i);
    delay(20);
  }
  motorOff();
}

// decelerate from maximum speed to zero
void maxDecel() {
  for (int i = 255; i >= 0; --i)
  {
    motor_right.set_speed(i);
    motor_left.set_speed(i);
    delay(20);
  } 
  motorOff();
}

void motorOff() {  
  // Now turn off motors
  motor_right.set_off();
  motor_left.set_off();
}

void loop() {
  // put your main code here, to run repeatedly:
  right(100, 2000);
  delay(1000);
  forwards(150, 2000);
  delay(1000);
  left(200, 2000);
  delay(1000);
  backwards(50, 2000);
  delay(1000);  
  motorOff();
  delay(1000);
}
