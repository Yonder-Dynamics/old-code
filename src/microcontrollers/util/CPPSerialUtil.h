/**
 * C++ Serial Communication utilities
 */

#ifndef CPPSERIALUTIL_H
#define CPPSERIALUTIL_H

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

int serialOpen(char* port){
	int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1 ){
		perror("CPPSerialUtil-->open: Unable to open port");
	}else{
		fcntl(fd, F_SETFL,0);
		printf("Port 1 has been sucessfully opened and %d is the file description\n",fd);
	}

	return fd;
}

int serialRead(int fd,char* buffer,int maxSize){
	int result = read(fd,buffer,maxSize);
	usleep(1000);
	return result;
}

int serialWrite(int fd,char* data,int len){
	return write(fd,data,len);
}

void serialClose(int fileDescriptor){
	close(fileDescriptor);
}

#endif