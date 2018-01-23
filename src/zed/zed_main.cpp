#include <ros/ros.h>
#include <std_msgs/String.h>

#include<sl/Camera.hpp>

#include<string>

int main(int argc,char** argv){

  std::cout << "Starting up ZED node...\n";

  ros::init(argc,argv,"ZED");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("zed_out",1000);


  ros::Rate loopRate(10);

  sl::ERROR_CODE err;
  sl::Camera zed;

  // Set configuration parameters
  sl::InitParameters init_params;
  // Use HD720 video mode (default fps: 60)
  init_params.camera_resolution = sl::RESOLUTION_HD720;
  // Use a right-handed Y-up coordinate system
  init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
  // Set units in meters
  init_params.coordinate_units = sl::UNIT_METER; 

  err = zed.open(init_params);
  if(err != sl::SUCCESS){
    exit(-1);
  }
  std::cout << "opened ZED camera successfully\n";

  sl::TrackingParameters tracking_parameters;
  err = zed.enableTracking(tracking_parameters);
  if(err != sl::SUCCESS){
    exit(-1);
  }
  std::cout << "camera tracking enabled\n";

  sl::Pose zed_pose;
  std_msgs::String msg;
  char msg_data[255];
  while(ros::ok()){
    if(zed.grab() == sl::SUCCESS){
      sl::TRACKING_STATE state =
        zed.getPosition(zed_pose, sl::REFERENCE_FRAME_WORLD);
      // Display translation and timestamp
      int end;
      end = sprintf(msg_data,
        "Translation: tx: %.3f, ty:  %.3f, tz:  %.3f, timestamp: %llu\r",
        zed_pose.getTranslation().tx,
        zed_pose.getTranslation().ty,
        zed_pose.getTranslation().tz,
        zed_pose.timestamp);
      msg_data[end] = 0;
      msg.data = std::string(msg_data);
      pub.publish(msg);
      // Display orientation quaternion
      end = sprintf(msg_data,
        "Orientation: ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f\r",
        zed_pose.getOrientation().ox,
        zed_pose.getOrientation().oy,
        zed_pose.getOrientation().oz,
        zed_pose.getOrientation().ow);
      msg_data[end] = 0;
      msg.data = std::string(msg_data);
      pub.publish(msg);
    }

    ros::spinOnce();
    loopRate.sleep();
  }

  std::cout << "Closing camera...\n";
  zed.close();
  return 0;
}
