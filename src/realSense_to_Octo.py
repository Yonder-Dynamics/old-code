import rospy
from sensor_msgs.msg import PointCloud2

pub = rospy.Publisher('cloud_in', PointCloud2, queue_size=10)

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    #print(type(data))
    pub.publish(data)

def listener():
    rospy.init_node('listener', anonymous=True)
    # Sets up a subscriber from RealSense R200 publisher channel and takes in a
    # PointCloud2 type
    rospy.Subscriber('/camera/depth/points', PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
