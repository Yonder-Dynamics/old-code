#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

imu_count = 0
cum_x = 0
cum_y = 0
cum_z = 0

def ros_callback_imu(data):
    global cum_x
    global cum_y
    global cum_z
    global imu_count
    imu_count += 1
    cum_x += data.angular_velocity.x
    cum_y += data.angular_velocity.y
    cum_z += data.angular_velocity.z
    if imu_count == 50:
        print(str(cum_x) + ' ' + str(cum_y) + ' ' + str(cum_z))
        cum_x = 0
        cum_y = 0
        cum_z = 0
        imu_count = 0

rospy.init_node("listener")
rospy.Subscriber("imu/data", Imu, ros_callback_imu)
rospy.spin()
