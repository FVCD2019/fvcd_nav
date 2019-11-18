#!/usr/bin/env python  
import roslib
import rospy
import tf
from std_msgs.msg import Int16MultiArray

pose_x = 5.5
pose_y = 1.0

def poseCB(data):
	pose_x = data[0]
	pose_y = data[1]

if __name__ == '__main__':
    print("init")
    rospy.init_node('map_broadcaster')
    #rospy.Subscriber("/detector/pose", Int16MultiArray, poseCB)
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        print("chk")
        br.sendTransform((pose_x, pose_y, 0.0),
                         (0.0, 0.0, 1.0, 1.0),
                         rospy.Time.now(),
                         "odom",
                         "map")
        rate.sleep()
