

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    pub.publish( data.data )

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    # Sets up a subscriber from RealSense R200 publisher channel and takes in a
    # PointCloud2 type
    rospy.Subscriber('/camera/depth/points', sensor_msgs/PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def talker():
    # Publish PointCloud2 from RealSense R200 to OctoMap
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    #rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(10) # 10hz

    # while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        # rate.sleep()

if __name__ == '__main__':
    try:
        # talker()
        listener()
    except rospy.ROSInterruptException:
        pass
