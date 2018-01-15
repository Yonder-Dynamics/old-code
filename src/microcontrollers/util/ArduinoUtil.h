/** 
 *	Arduino utilities file
 */
#ifndef ARDUINOUTIL_H
#define ARDUINOUTIL_H

#include <Arduino.h>
#include "MicroControllerInterface.h"

int sender(int argc,char* arg){
	if(!Serial){//do nothing if serial isnt ready
		return -1;
	}

	if(argc){//do noting if no data is given
		int result = Serial.write(arg,argc);
		// Serial.flush();
		return result;
	}
}

int receiver(char* dest,int max){
	delay(100);
	if(!Serial || Serial.available() <= 0){//do nothing if serial isnt ready
		return -1;
	}

	if(max){
		return Serial.readBytes(dest,max);
	}
}

void initSerial(){
	Serial.begin(9600);
	while(!Serial);
}

void arduinoHandshake(bool role){
	initSerial();
	handshake(role,sender,receiver);
}

#endif