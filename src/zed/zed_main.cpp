#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include<sl/Camera.hpp>

#include<string>

int main(int argc,char** argv){

  std::cout << "Starting up ZED node...\n";

  ros::init(argc,argv,"ZED");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("zed_out",1000);


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
  geometry_msgs::PoseStamped pose_out;
  pose_out.header.frame_id = "map";
  std_msgs::String msg;
  char msg_data[255];
  int count = 0;
  while(ros::ok()){
    if(zed.grab() == sl::SUCCESS){
      sl::TRACKING_STATE state =
        zed.getPosition(zed_pose, sl::REFERENCE_FRAME_WORLD);
      // Display translation and timestamp
      /*
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
      */

      pose_out.header.stamp.sec = zed_pose.timestamp/1000000000;
      pose_out.header.stamp.nsec = zed_pose.timestamp%1000000000;
      //std::cout << "time: " << zed_pose.timestamp << "\n";
      pose_out.pose.position.x = zed_pose.getTranslation().tx;
      pose_out.pose.position.y = zed_pose.getTranslation().ty;
      pose_out.pose.position.z = zed_pose.getTranslation().tz;

      pose_out.pose.orientation.x = zed_pose.getOrientation().ox;
      pose_out.pose.orientation.y = zed_pose.getOrientation().oy;
      pose_out.pose.orientation.z = zed_pose.getOrientation().oz;
      pose_out.pose.orientation.w = zed_pose.getOrientation().ow;

      pub.publish(pose_out);
    }

    count++;
    ros::spinOnce();
    loopRate.sleep();
  }

  std::cout << "Closing camera...\n";
  zed.close();
  return 0;
}
