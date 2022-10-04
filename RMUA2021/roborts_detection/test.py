#!/usr/bin/env python3
import rospy
from roborts_msgs.msg import GimbalAngle
def A():

    rospy.init_node('armor_detection_node',anonymous=True)
    _ctrlinfo_pub = rospy.Publisher('/cmd_gimbal_angle', 
                                             GimbalAngle, queue_size=10)
    angle=[-0.2,-0.2]
    while not rospy.is_shutdown():
        _ctrlinfo_pub.publish(False,False,angle[0], angle[1])
        rospy.loginfo('pitch '+str(angle[0])+' yaw '+str(angle[1]))


if __name__ == '__main__':
    try:
        A()
    except rospy.ROSInterruptException:
        pass