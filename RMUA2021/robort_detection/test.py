#!/usr/bin/env python3
import pycuda.gpuarray as gpuarray
import pycuda.driver as drv
import pycuda.autoinit
import numpy as np
import time
import actionlib
import rospy
import roborts_msgs.msg
from roborts_msgs.msg import GimbalSwingAction

def gimbal_client():
    client=actionlib.SimpleActionClient('gimbal_swing_',roborts_msgs.msg.GimbalSwingAction)
    print('init')
    #client.wait_for_server()
    print('wait')
    client.send_goal(1)
    print('success')
    client.cancel_goal()
    print('cancel')
    client.wait_for_result()


if __name__ == '__main__':
    try:
        rospy.init_node('fibonacci_client_py')
        gimbal_client()
    except rospy.ROSInterruptException:
        print('error')