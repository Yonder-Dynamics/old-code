/**
 *	This is microcontroller interface code for communication between ROS nodes
 *	on the TX2 and microcontrollers mounted on the chassis. Arduino projects
 *	should create symbolic links to this file and #include it in order to use 
 * 	protocol macros and other common code for setting up and communicating with
 *	microcontrollers.
 */
#ifndef MCI_H
#define MCI_H

//handshake macros
#define HANDSHAKE_INIT true
#define HANDSHAKE_WAIT false

//controller type macros
#define NULL_CONTROLLER		0
#define ARM_CONTROLLER		1
//etc..

//type specifiers
#define END_DATA_TRANSFER 0

/*
 *	data transfer format:
 *	int type_specifier
 *	int data_size
 *	[int]	data_specifier
 */

void handshake(bool role, int(*sender)(int,char*), int(*receiver)(char*,int)){
	char startBit[] = {'a',0};
	char ackBit[] = {'b',0};
	char endBit[] = {'c',0};
	char receiveBit[] = {0,0};
	int status = 0;
	if(role){ //caller will initiate handshake
		int result;
		while(((*receiver)(receiveBit,1))==-1 || receiveBit[0] != ackBit[0]){
			result = (*sender)(1,startBit);
			// receiveBit[0] = 0;
			// result = (*sender)(1,receiveBit); //echo the input
		}
		result = (*sender)(1,endBit);
	}else{ //caller will respond to handshake
		int dataSize = 255;
		char data[dataSize];

		//wait for the start bit
		while(((*receiver)(receiveBit,1))==-1 || receiveBit[0] != startBit[0]);
		int result = (*sender)(1,ackBit);
		while(((*receiver)(receiveBit,1))==-1 || receiveBit[0] != endBit[0]);

	}	
}

#endif