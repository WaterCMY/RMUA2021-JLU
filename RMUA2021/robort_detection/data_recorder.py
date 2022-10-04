#!/usr/bin/env python3
import rospy
import os
import numpy as np
import time
from roborts_msgs.msg import YawData

class DataRecordor():
    def __init__(self):
        self.timestamp=[]
        self.yaw_angle=[]
        self.filter_angle_yaw=[]
        self.pitch_angle=[]
        self.filter_angle_pitch=[]
        self.t1=time.time()
        self.t2=time.time()
        self.sub_yaw_data=rospy.Subscriber('/yaw_data_recordor',YawData,self.yaw_data_callback)


    def yaw_data_callback(self,yaw_data):
        self.timestamp.append(yaw_data.timestamp)
        self.yaw_angle.append(yaw_data.yaw_angle)
        self.filter_angle_yaw.append(yaw_data.filter_angle_yaw)
        self.pitch_angle.append(yaw_data.pitch_angle)
        self.filter_angle_pitch.append(yaw_data.filter_angle_pitch)
        self.write_data()
        rospy.loginfo("receive yaw_data success: %f , %f , %f, %f, %f", yaw_data.timestamp,yaw_data.yaw_angle, yaw_data.filter_angle_yaw,
        yaw_data.pitch_angle, yaw_data.filter_angle_pitch)


    def write_data(self):
        self.t2=time.time()
        if self.t2-self.t1 >= 30:
            timestamp_array = np.array(self.timestamp).reshape(len(self.timestamp),1)
            yaw_array = np.array(self.yaw_angle).reshape(len(self.yaw_angle),1)
            filter_array_yaw = np.array(self.filter_angle_yaw).reshape(len(self.filter_angle_yaw),1)
            pitch_array = np.array(self.pitch_angle).reshape(len(self.pitch_angle),1)
            filter_array_pitch = np.array(self.filter_angle_pitch).reshape(len(self.filter_angle_pitch),1)

            data_merge = np.hstack((timestamp_array,yaw_array, filter_array_yaw,pitch_array,filter_array_pitch))
            data = data_merge.tolist()
            with open("data.txt",'a+') as f:
                for line in data:
                    f.write(str(line)+'\n')
            self.t1=time.time()
            self.timestamp=[]
            self.yaw_angle=[]
            self.filter_angle_yaw=[]
            self.pitch_angle=[]
            self.filter_angle_pitch=[]
            rospy.loginfo("write data to txt success!!")


    def main(self):
        rospy.spin()



if __name__=='__main__':
    rospy.init_node('yaw_data_recorder')
    node = DataRecordor()
    node.main()

