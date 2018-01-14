#include "ros/ros.h"
#include "std_msgs/String.h"

#include "MicroControllerInterface.h"

#include <sstream>


int main(int argv,char** argc){
	ros::init(argv,argc,"MCI");

	ros::NodeHandle n;

	ros::Publisher testPublisher = n.advertise<std_msgs::String>("microcontroller_out", 1000);

	ros::Rate loopRate(10);

	while(ros::ok()){
	    /**
	     * This is a message object. You stuff it with data, and then publish it.
	     */
	    std_msgs::String msg;

	    std::stringstream ss;
	    ss << "hello world!";
	    msg.data = ss.str();

	    ROS_INFO("%s", msg.data.c_str());

	    /**
	     * The publish() function is how you send messages. The parameter
	     * is the message object. The type of this object must agree with the type
	     * given as a template parameter to the advertise<>() call, as was done
	     * in the constructor above.
	     */
	    testPublisher.publish(msg);

		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}