#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;
char aWord[100];

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  Serial.println("Ready!");
}

void loop() {
  delay(100);
}

// callback for received data
int i = 0;
void receiveData(int byteCount){
  
  while(Wire.available()) {
    number = Wire.read();
    aWord[i] = number;
    i++;
    Serial.print("data received: ");
    Serial.println(number);
    
    if (number == 'c'){
    
      if (state == 0){
        digitalWrite(13, HIGH); // set the LED on
        state = 1;
      }
      else{
        digitalWrite(13, LOW); // set the LED off
        state = 0;
      }
    }
  }
  if(number == '\0'){
    Serial.println(aWord);
    i = 0;
  }
}

// callback for sending data
void sendData(){
  for(int i = 0; i < strlen(aWord); i++){
    Wire.write(aWord[i]);
  }
}


