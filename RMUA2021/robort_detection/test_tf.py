#!/usr/bin/env python3
import rospy
import tf


def test_tf():
    rate = rospy.Rate(10.0)
    listener = tf.TransformListener()
    get_pose = False
    while not rospy.is_shutdown() and not get_pose:
      try:
        now = rospy.Time.now()
        listener.waitForTransform("base_link", "gimbal", now, rospy.Duration(0.3))
        (trans, rot) = listener.lookupTransform("base_link", "gimbal", now)
        
        _,_,yaw = tf.transformations.euler_from_quaternion(rot)
        print("yaw:",yaw)
        # print(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3])
        get_pose = True
      except (tf.Exception, tf.LookupException, tf.ConnectivityException):
        print("tf echo error")
        continue
      rate.sleep()


test_tf()