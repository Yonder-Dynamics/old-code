#include "ros/ros.h"
#include "std_msgs/String.h"

#include "MicroControllerInterface.h"
#include "util/CPPSerialUtil.h"

#include <sstream>

void publish(ros::Publisher pub,char* data){
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "data: " << std::string(data);
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    pub.publish(msg);
}

int GLOBAL_FD = -1;

int nodeSender(int size,char* data){
	printf("sending %d bytes of data: %s\n",size,data);
	serialWrite(GLOBAL_FD,data,size);
}

int nodeReceiver(char* buffer,int maxSize){
	int size;
	if(size=serialRead(GLOBAL_FD,buffer,maxSize)!=-1){
		buffer[size] = 0;
		printf("got %d bytes of data: %d\n",size,buffer[0]);
	}
	return size;
}

int main(int argv,char** argc){
	ros::init(argv,argc,"MCI");

	ros::NodeHandle n;

	ros::Publisher testPublisher = n.advertise<std_msgs::String>("microcontroller_out", 1000);

	ros::Rate loopRate(10);

	int port = serialOpen("/dev/ttyUSB0");
	GLOBAL_FD = port;

	int bufferSize = 255;
	char buffer[bufferSize];

	handshake(HANDSHAKE_INIT,nodeSender,nodeReceiver);

	publish(testPublisher,"Handshake finished!");

	while(ros::ok()){
		int read;
		if(read=serialRead(port,buffer,bufferSize)){
			buffer[read] = 0;
			publish(testPublisher,buffer);
		}
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}