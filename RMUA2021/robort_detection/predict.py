#!/usr/bin/env python3
import os
import sys
import time
import cv2
import gxipy as gx
import numpy as np
import rospy
import copy
import actionlib
import roborts_msgs.msg
from roborts_msgs.msg import GimbalAngle,PyArmorInfo,GameStatus,RobotHeat,RobotAngle,RobotAngleTest,RobotStatus,GameRobotBullet,kalman,YawData,GimbalCtrAction,GimbalSwingActionGoal
from roborts_msgs.srv import FricWhl, ShootCmd,ShootSpeed


class KalmanFilter(object):
    def __init__(self, dt, u, std_acc, std_meas):
        self.dt = dt
        self.u = u
        self.std_acc = std_acc

        self.A = np.array([[1, self.dt],
                            [0, 1]])
        self.B = np.array([[(self.dt ** 2) / 2], [self.dt]])

        self.H = np.array([[1, 0]])

        self.Q = np.array([[(self.dt ** 3) / 3, (self.dt ** 2) / 2],
                            [(self.dt ** 2) / 2, self.dt]]) * self.std_acc ** 2

        self.R = std_meas ** 2

        self.P = np.eye(self.A.shape[1])

        self.x = np.array([[0], [0]])
        print(self.Q)

    def predict(self):
        # Ref :Eq.(9) and Eq.(10)

        # Update time state
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # Calculate error covariance
        # P= A*P*A' + Q
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        # Ref :Eq.(11) , Eq.(11) and Eq.(13)
        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Eq.(11)

        # self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))  # Eq.(12)
        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))

        I = np.eye(self.H.shape[1])
        # self.P = (I - (K * self.H)) * self.P  # Eq.(13)
        self.P = np.dot(I - (np.dot(K, self.H)), self.P)  

class Detector():
    def __init__(self):
        rospy.init_node('armor_detection_node')
        #camera initialization
        self.classic_object_3d_points=np.array(([-71, -30, 0],                 #xmin ymin
                                     [-71, 30, 0],                  #xmin ymax
                                     [71, 30, 0],                  #xmax ymax
                                     [71, -30, 0]), dtype=np.double) #xmax ymin
        self.camera_matrix_2_up= np.array(([1716.0988996628, 1.43642153020885, 370.009711441846],
                                       [0, 1720.87835300139, 270.743138066265],
                                       [0, 0, 1.0]), dtype=np.double)
        self.dist_coefs_2_up = np.array([-0.158618771341394, 1.74637984736916, -0.00436517631477145, 0.00459322920933054, 0], dtype=np.double)
        
        self.camera_matrix_1_up= np.array(([1736.20124520037, -1.07150598978613, 446.539045954127],
                                       [0, 1743.43636415232, 346.273232875011],
                                       [0, 0, 1.0]), dtype=np.double)
        self.dist_coefs_1_up = np.array([ -0.172879564681552, 6.9040664418158, 0.00673427025530622, 0.00209497360898932,0],dtype=np.double)




        self.camera_matrix_1_up_new= np.array(([1736.279089353611, -5.592859222382724,413.6427998402262],
                                       [0, 1729.986870457292,359.6450899943676],
                                       [0, 0, 1.0]), dtype=np.double)
        self.dist_coefs_1_up_new = np.array([0.1128,-5.7055,0.0072,0.0006,0],dtype=np.double)

        self.camera_matrix_2_up_new= np.array(([1749.105951195480, -11.238130994954375,391.6516881619215],
                                       [0, 1749.803519207192,408.5141982752361],
                                       [0, 0, 1.0]), dtype=np.double)
        self.dist_coefs_2_up_new = np.array([0.0872,-2.1353,0.0145,-0.0028,0],dtype=np.double)
        self._enemy_pub=rospy.Publisher('/PyArmorInfo',
                                        PyArmorInfo,queue_size=1)
        self._ctrlinfo_pub = rospy.Publisher('/cmd_gimbal_angle',
                                             GimbalAngle, queue_size=1)
        self._fricwhl_client = rospy.ServiceProxy("/cmd_fric_wheel",FricWhl)
        self._shoot_client = rospy.ServiceProxy("/cmd_shoot",ShootCmd)
        self.speed_client=rospy.ServiceProxy("/set_shoot_speed",ShootSpeed)
        #ros subscriber initialization
        rospy.Subscriber("/kalman",kalman,self.kalman_callback)
        rospy.Subscriber("/game_status",GameStatus,self.game_callback)
        rospy.Subscriber("/robot_heat",RobotHeat,self.heat_callback)
        rospy.Subscriber("/robot_angle_pub",RobotAngle,self.angle_callback)
        #rospy.Subscriber("/robot_angle_pub1",RobotAngleTest,self.angle_test_callback)
        rospy.Subscriber("/robot_status",RobotStatus,self.id_callback)
        rospy.Subscriber("/game_robot_bullet",GameRobotBullet,self.bullet_callback)
        #ros action initialization
        self.gimbal_client=actionlib.SimpleActionClient('gimbalswing',roborts_msgs.msg.GimbalCtrAction)

        self.far=0
        self.my_id=0
        self.loss_enemy=0
        self.find_enemy=0
        self.gimbal_angle=0
        self.last_pitch=0.1
        self.enemycolor=0
        self.shoot_speed=1340
        self.far=0
        self.last_far=0
        self.pub_yaw_data = rospy.Publisher("/yaw_data_recordor",YawData,queue_size=1)
        self.last_bg=np.array([[399],[263]])
        self.DT=0.02
        self.U=0
        self.bullet=10
        self.perspectice_distance=0
        self.perspective_pitch=0.13
        self.gimbalswing=False
        self.heat=0
        self.hot=False

        #kalman initialization
        # self.STD_ACC_yaw=11
        # self.STD_MEAS_yaw=0.135
        # self.STD_ACC_pitch=112
        # self.STD_MEAS_pitch=1.28
        self.t1 = time.time()
        self.t2 = time.time()
        # self.kf_filter_yaw=KalmanFilter(self.DT, self.U, self.STD_ACC_yaw, self.STD_MEAS_yaw)
        # self.kf_filter_pitch=KalmanFilter(self.DT,self.U, self.STD_ACC_pitch,self.STD_MEAS_pitch)
        # self.yaw_angle_abs = np.array([[0]])
        # self.last_yaw = np.array([[0]])
        self.Ux=0
        self.Uy=0
        self.STD_ACC=30
        self.x_STD_MEANS=0.1
        self.y_STD_MEANS=0.1
        self.kf2D_filter=KalmanFilter2D(self.DT, self.Ux, self.Uy, self.STD_ACC, self.x_STD_MEANS,self.y_STD_MEANS)

        self.STD_ACC_bg = 30
        self.x_STD_MEANS_bg = 0.05
        self.y_STD_MEANS_bg = 0.05
        self.kf2D_filter_bg = KalmanFilter2D(self.DT, self.Ux, self.Uy, self.STD_ACC_bg, self.x_STD_MEANS_bg, self.y_STD_MEANS_bg)
    def _set_fricwhl(self,can_start,speed):
        try:
            resp = self._fricwhl_client.call(can_start,speed)
            rospy.loginfo("Message From fricwheelserver:%s"%resp.received)
        except rospy.ServiceException:

            rospy.logwarn("Service call failed")
        except:
            pass
    def bullet_callback(self,data):
        if(self.my_id==1):
            self.bullet=data.red1
        elif(self.my_id==2):
            self.bullet=data.red2
        elif(self.my_id==3):
            self.bullet==data.blue1
        elif(self.my_id==4):
            self.bullet==data.blue2
    def _shoot(self,shoot_mode, shoot_number):

        #rospy.wait_for_service("cmd_fric_wheel")
        try:
            resp = self._shoot_client.call(shoot_mode, shoot_number)
            #rospy.loginfo("Message From shootserver:%s"%resp.received)
        except rospy.ServiceException:
            #rospy.logwarn("Service call failed")
            pass
        except:
            pass
    def speed_srv(self,speed):
        try:
            self.speed_client.call(speed)
        except rospy.ServiceException:
            #rospy.logwarn("Service call failed")
            pass
    def infer(self,binary):
        contour = []
        C_lights = []

        B, G, R = cv2.split(binary)
        if self.enemycolor==0:
            dst = cv2.subtract(B, R)
        elif self.enemycolor==1:
            dst=cv2.subtract(R,B)
        else:
            return 0,None
        ret, binary = cv2.threshold(dst, 80, 255, cv2.THRESH_BINARY)

        ##--------------------------------1. find contours-----------------------------##
        contours, heriachy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        ##--------------------------------2. fit ellipse and RECT rcBound--------------##
        ##judge by calculate solidity of contours and area.
        for i in range(len(contours)):
            if len(contours[i]) >= 5:
                ellipse = cv2.fitEllipse(contours[i])  # 用一个椭圆来匹配目标。它返回一个旋转了的矩形的内接椭圆
                (x, y), (MA, ma), angle = ellipse[0], ellipse[1], ellipse[2]
                ellipseArea = 3.14 * MA * ma / 4  # 计算外接椭圆面积
                area = cv2.contourArea(contours[i])  # 计算轮廓面积
                if ellipseArea == 0:
                    continue
                else:
                    #if area / ellipseArea >= 0.2 and area >= 20 and area <= 2000:  # 判断凸度(Solidity) 和轮廓面积大小
                    if area >= 10 and area <= 60000:
                        rect = cv2.minAreaRect(contours[i])  # 根据轮廓得到外接矩阵 长宽不固定，靠近x轴定义长
                        if (rect[1][0]/rect[1][1]>=0.1 and rect[1][0]/rect[1][1] <= 0.8) or (rect[1][0]/rect[1][1]<=10 and rect[1][0]/rect[1][1] >= 1.25):      #判断矩阵长宽比？？？
                            a = tuple(((rect[1][0] * 1.1, rect[1][1] * 1.1)))  # 外接矩形(宽，长)？？？
                            
                            rect = tuple((rect[0], a, rect[2]))
                            
                            box = cv2.boxPoints(rect)
                            box = np.int0(box)
                            contour.append((rect, box, area, x, y))  # 保存符合要求的外接矩形及四点坐标，面积和中心x坐
                        else:
                            continue
        ##-----------------------------3. Find the right pair of lights----------------##
        ##campare each pair of contours, The approximate parallel, length-width ratio

        if len(contour) >= 2:
            for i in range(len(contour)):
                j = i + 1
                while j < len(contour):

                    if abs(contour[i][4]-contour[j][4] )> 40:
                        j+=1
                        continue
                    if contour[i][2] / contour[j][2] >= 0.25 and contour[i][2] / contour[j][2] <= 4 and abs(contour[i][3]-contour[j][3])/((contour[i][3]+contour[j][3])/2) <= 8 and abs(contour[i][4]-contour[j][4])/((contour[i][4]+contour[j][4])/2) <= 0.8 :  # 判断矩形面积比
                        # if abs(contour[i][3]-contour[j][3]) > 50 and abs(contour[i][4]-contour[j][4])<100 and abs(contour[i][3]-contour[j][3]) < 150:
                        max_i = max(contour[i][0][1][0],contour[i][0][1][1])
                        max_j = max(contour[j][0][1][0],contour[j][0][1][1])
                        if max_i / max_j >= 2 or max_i / max_j <= 0.5:
                            j+=1
                            continue

                        couple_light = tuple((contour[i], contour[j]))
                        C_lights.append(couple_light)
                    j += 1

        ##----------------------------4. Find best pair of lights----------------------##


        if len(C_lights) != 0:
            light_len=[]

            for i in range(len(C_lights)):
                light_len.append(min(min(C_lights[i][0][0][1][0], C_lights[i][0][0][1][1]),min(C_lights[i][1][0][1][0], C_lights[i][1][0][1][1])))
            if light_len != []:
                length_i = 0
                max_light_len = max(light_len)
                while (length_i <= (len(light_len) - 1)):
                    if light_len[length_i] * 1.2 <= max_light_len:
                        #max_light_len = max(light_len)
                        #length_i = 0
                        del light_len[length_i]
                        del C_lights[length_i]
                    else:
                        length_i += 1

        armor=[]
        if len(C_lights) != 0 :
            for index in range(len(C_lights)):
                if C_lights[index][0][0][1][0] >= C_lights[index][0][0][1][1] and C_lights[index][1][0][1][0] >= \
                        C_lights[index][1][0][1][1]:  # 判断旋转角是否异常
                    angle = (C_lights[index][0][0][2] + C_lights[index][1][0][2]) / 2
                    l_light = cv2.boxPoints(tuple((C_lights[index][0][0][0], C_lights[index][0][0][1], angle - 10)))
                    r_light = cv2.boxPoints(tuple((C_lights[index][1][0][0], C_lights[index][1][0][1], angle)))
                else:
                    l_light = C_lights[index][0][1]
                    r_light = C_lights[index][1][1]
                ##---------------------------5. Find Armor through pair of lights--------------##

                n = len(l_light) + len(r_light)
                cnt = np.zeros((n, 1, 2))
                for i in range(len(l_light)):
                    cnt[i][0] = l_light[i]
                for j in range(len(r_light)):
                    cnt[len(l_light) + j][0] = r_light[j]
                cnt = cnt.astype(int)
                Rect = cv2.minAreaRect(cnt)
                if (abs(Rect[2])>10 and abs(Rect[2])<80) :
                    continue
                if (Rect[1][0] / Rect[1][1] >= 1.6 and Rect[1][0] / Rect[1][1] <= 2.8) or (Rect[1][0] / Rect[1][1] >= 0.35714 and Rect[1][0] / Rect[1][1] <= 0.625):

                    box = cv2.boxPoints(Rect)
                    box = np.int0(box)
                    box=box.tolist()
                    box.append(box.pop(0))
                    armor.append(box)
                else:
                    continue
        else:
            return 0,None

        return len(armor),armor
    def classic_calcu_angle_car2(self,bbox):
        
        x_min, x_max, y_min, y_max=self.sortbbox(bbox)

        self.classic_object_2d_point=np.array(([x_min,y_min],[x_min,y_max],[x_max,y_max],[x_max,y_min]),dtype=np.double)
        _, _, tvec = cv2.solvePnP(self.classic_object_3d_points, self.classic_object_2d_point,
                                    self.camera_matrix_2_up_new, self.dist_coefs_2_up_new,
                                    cv2.SOLVEPNP_EPNP)
        x=tvec[0][0]
        y=tvec[1][0]
        z=tvec[2][0]
        d=np.sqrt(x*x+z*z)
        distance=np.sqrt(x*x+y*y+z*z)

        yaw   = float(np.arctan2(x, z))
        z1=np.sqrt(distance*distance-80089)
        #283-90
        pitch=float(np.arctan(193/z1))

        if distance >=5000:
            pitch=pitch-0.16
            self.far=6
        elif distance<5000 and distance >=3500:
            pitch = pitch - 0.15
            self.far=5
        elif distance<3500 and distance >= 3000:
            pitch = pitch - 0.14
            self.far=4
        elif distance<3000 and distance >=2000:
            pitch=pitch - 0.13
            self.far=3
        elif distance<2000 and distance >= 1500:
            if (distance <=1550 and self.last_far)==1 :
                pass
            else:
                pitch = pitch -0.08
            self.far=2
        elif distance < 1500:
            if (distance >= 1450 and self.last_far)==2 :
                pitch = pitch-0.08
            self.far=1
        yaw=yaw-0.012
        pitch=0.8*pitch
        #print("distance",distance)
        return [pitch,yaw,distance]
    
    def classic_calcu_angle_car1(self,bbox):
        
        x_min, x_max, y_min, y_max=self.sortbbox(bbox)

        self.classic_object_2d_point=np.array(([x_min,y_min],[x_min,y_max],[x_max,y_max],[x_max,y_min]),dtype=np.double)
        _, _, tvec = cv2.solvePnP(self.classic_object_3d_points, self.classic_object_2d_point,
                                    self.camera_matrix_1_up_new, self.dist_coefs_1_up_new,
                                    cv2.SOLVEPNP_EPNP)
        x=tvec[0][0]
        y=tvec[1][0]
        z=tvec[2][0]
        d=np.sqrt(x*x+z*z)
        distance=np.sqrt(x*x+y*y+z*z)

        yaw   = float(np.arctan2(x, z))
        z1=np.sqrt(distance*distance-80089)
        #283-90
        pitch=float(np.arctan(193/z1))

        if distance >=5000:
            pitch=pitch-0.12
            self.far=6
        elif distance<5000 and distance >=3500:
            pitch = pitch - 0.11
            self.far=5
        elif distance<3500 and distance >= 3000:
            pitch = pitch - 0.08
            self.far=4
        elif distance<3000 and distance >=2000:
            pitch=pitch - 0.06
            self.far=3
        elif distance<2000 and distance >= 1550:
            if (distance <=1700 and self.last_far==1) :
                pass
            else:
                pitch = pitch -0.04
            self.far=2
        elif distance < 1550:
            if (distance >= 1400 and self.last_far==2) :
                pitch = pitch-0.04
            self.far=1
        
        if self.far!=1:
            pitch=0.8*pitch
        #print("far",self.far)
        #print("distance",distance)
        return [pitch,yaw,distance]
    def id_callback(self,data):
        if(data.id==1):
            self.my_id=1
            self.enemycolor=0
        elif(data.id==2):
            self.my_id=2
            self.enemycolor=0
        elif(data.id==101):
            self.my_id=3
            self.enemycolor=1
        elif(data.id==102):
            self.my_id=4
            self.enemycolor=1

    def kalman_callback(self,data):
        self.DT=data.DT
        self.U=data.U
        self.STD_ACC_yaw=data.STD_ACC
        self.STD_MEAS_yaw=data.STD_MEAS
        self.kf_filter=KalmanFilter(self.DT,self.U,self.STD_ACC,self.STD_MEAS)
        rospy.loginfo("receive kalman %f %f %f %f ",data.DT,data.U,data.STD_ACC,data.STD_MEAS)
    def select_enemy(self,angle):
        min_distance=100000
        index=0
        for i in range(len(angle)):
            if angle[i][2]<min_distance:
                min_distance=angle[i][2]
                index=i
        return index
    def control_2(self,angle):
        if (angle is not None) :
            if self.gimbalswing==False:
                self.gimbal_client.cancel_goal()
                self.gimbalswing=True
                return 0
            self.t2 = time.time()
            delta_T = self.t2 - self.t1
            if(delta_T < 0):
                rospy.error("ERROR:delta_T < 0")
            # if self.find_enemy == 0:
            #     self.kf_filter_yaw.x[0][0]=angle[1]
            #     self.kf_filter_yaw.x[1][0]=0
            #     for i in range(1,5):
            #         self.kf_filter_yaw.predict()
            #         if self.yaw_angle_abs!=0:
            #             self.kf_filter_yaw.update(self.yaw_angle_abs)
            #             angle[1]=0
            #         else:
            #             self.kf_filter_yaw.update(angle[1])

            #     #self.kf_filter_yaw.update(0)
                
            #     self.last_yaw = angle[1]
            #     self.find_enemy =1
            # else:
            #     self.yaw_angle_abs = self.yaw_angle_abs + self.last_yaw
            #     yaw_predict_angle = self.kf_filter_yaw.predict()
            #     self.kf_filter_yaw.update(self.yaw_angle_abs)
            #     angle[1] = np.asscalar(yaw_predict_angle[0] - self.yaw_angle_abs)
            #     self.last_yaw = angle[1]
            
            
            # yaw_predict_angle=self.kf_filter_yaw.predict()                  
            # self.kf_filter_yaw.update(angle[1])
            # pitch_predict_angle=self.kf_filter_pitch.predict()
            # self.kf_filter_pitch.update(angle[0])

            # self.pub_yaw_data.publish(delta_T,angle[1],yaw_predict_angle[0],angle[0],pitch_predict_angle[0])
            # angle[1]=yaw_predict_angle[0]
            # angle[0]=pitch_predict_angle[0]
            
            if (self.far-self.last_far)!=0:
                if self.far==1:
                    self.shoot_speed=1250
                elif self.far==2:
                    self.shoot_speed=1265
                elif self.far==3:
                    self.shoot_speed=1285
                elif self.far==4:
                    self.shoot_speed=1305
                elif self.far==5:
                    self.shoot_speed=1320
                elif self.far==6:
                    self.shoot_speed=1330
            self._ctrlinfo_pub.publish(True,False,-angle[1], angle[0])

            if self.far==1 and self.heat<=120:
                self._shoot(1,10)
            elif self.far==2 and self.heat<130:
                self._shoot(1,8)
            elif self.far>=3 and self.far<=5 and self.heat<180:
                self._shoot(1,5)
            elif self.far>5 and self.heat<200:
                self._shoot(1,3)
                
            rospy.loginfo('pitch '+str(angle[0])+' yaw '+str(angle[1]))
            # self.loss_enemy=0
            self.last_pitch=angle[0]
            self.last_far=self.far
        else:
            if(self.gimbal_angle<3.14/2 and self.gimbal_angle>-3.14/2 and self.gimbal_angle!=0 and self.loss_enemy>50):
                self._ctrlinfo_pub.publish(False,False,self.gimbal_angle,0.13)
                #self._ctrlinfo_pub.publish(False,False,self.gimbal_angle,self.perspective_pitch)
                if self.gimbalswing==False:
                    self.gimbal_client.cancel_goal()
                    self.gimbalswing=True
            elif self.loss_enemy>100:
                
                self.gimbal_client.send_goal(GimbalSwingActionGoal(goal=True))
                self.gimbalswing=False
                #self._ctrlinfo_pub.publish(False,False,0,0.15)
                # self.kf_filter=KalmanFilter(self.DT, self.U, self.STD_ACC, self.STD_MEAS)
                self.loss_enemy=0
    def game_callback(self,data):
        #print("game statue",data.game_status)
        #rospy.loginfo("game status %d"%data.game_status)
        if(data.game_status==4):
            self.start=True
            self._set_fricwhl(True,self.shoot_speed)
    def heat_callback(self,data):
        #print(data.shooter_heat)
        #rospy.loginfo("heat %d"%data.shooter_heat)
        # if(data.shooter_heat>120):
        #     self.hot=True
        # else:
        #     self.hot=False
        self.heat=data.shooter_heat
        if self.heat>=225:
            self._shoot(0,0)
    def ROI(self,src,points):
        x_min, x_max, y_min, y_max=self.sortbbox(points)
        width=x_max-x_min
        height=y_max-y_min

        # x_min = int(x_min - 0.8 * width)
        # x_max = int(x_max + 0.8 * width)
        # y_min = int(y_min - 0.8 * height)
        # y_max = int(y_max + 0.8 * height)

        x_min = int(x_min -  width)
        x_max = int(x_max +  width)
        y_min = int(y_min -  height)
        y_max = int(y_max +  height)
        if y_max>599:
            y_max=599
        if y_min<0:
            y_min=0
        if x_max>799:
            x_max=799
        if x_min<0:
            x_min=0

        lt=[x_min,y_min]
        rb=[x_max,y_max]

        src = src[lt[1]:rb[1], lt[0]:rb[0]]
        armor_len,box=self.infer(src)
        if armor_len!=0:
            for i in range(len(box)):
                box[i][0][0] += lt[0]
                box[i][0][1] += lt[1]
                box[i][1][0] += lt[0]
                box[i][1][1] += lt[1]

                box[i][2][0] += lt[0]
                box[i][2][1] += lt[1]
                box[i][3][0] += lt[0]
                box[i][3][1] += lt[1]
        if armor_len==0:
            return False,0,None
        else:
            return True,len(box),box
    def sortbbox(self,bbox):
        x_list = [bbox[0][0], bbox[1][0], bbox[2][0], bbox[3][0]]
        y_list = [bbox[0][1], bbox[1][1], bbox[2][1], bbox[3][1]]
        x_min = min(x_list)
        x_max = max(x_list)
        y_min = min(y_list)
        y_max = max(y_list)

        return x_min, x_max, y_min, y_max

    def control_1(self,angle):
        if (angle is not None) :
            if self.gimbalswing==False:
                self.gimbal_client.cancel_goal()
                self.gimbalswing=True
                return 0
            self.t2 = time.time()
            delta_T = self.t2 - self.t1
            if(delta_T < 0):
                rospy.error("ERROR:delta_T < 0")
            # if self.find_enemy == 0:
            #     self.kf_filter_yaw.x[0][0]=angle[1]
            #     self.kf_filter_yaw.x[1][0]=0
            #     for i in range(1,5):
            #         self.kf_filter_yaw.predict()
            #         if self.yaw_angle_abs!=0:
            #             self.kf_filter_yaw.update(self.yaw_angle_abs)
            #             angle[1]=0
            #         else:
            #             self.kf_filter_yaw.update(angle[1])

            #     #self.kf_filter_yaw.update(0)
                
            #     self.last_yaw = angle[1]
            #     self.find_enemy =1
            # else:
            #     self.yaw_angle_abs = self.yaw_angle_abs + self.last_yaw
            #     yaw_predict_angle = self.kf_filter_yaw.predict()
            #     self.kf_filter_yaw.update(self.yaw_angle_abs)
            #     angle[1] = np.asscalar(yaw_predict_angle[0] - self.yaw_angle_abs)
            #     self.last_yaw = angle[1]
            
            
            # yaw_predict_angle=self.kf_filter_yaw.predict()                  
            # self.kf_filter_yaw.update(angle[1])
            # pitch_predict_angle=self.kf_filter_pitch.predict()
            # self.kf_filter_pitch.update(angle[0])

            # self.pub_yaw_data.publish(delta_T,angle[1],yaw_predict_angle[0],angle[0],pitch_predict_angle[0])
            # angle[1]=yaw_predict_angle[0]
            # angle[0]=pitch_predict_angle[0]
            
            if (self.far-self.last_far)!=0:
                if self.far==1:
                    self.shoot_speed=1340
                elif self.far==2:
                    self.shoot_speed=1360
                elif self.far==3:
                    self.shoot_speed=1380
                elif self.far==4:
                    self.shoot_speed=1400
                elif self.far==5:
                    self.shoot_speed=1420
                elif self.far==6:
                    self.shoot_speed=1425
            self._ctrlinfo_pub.publish(True,False,-angle[1], angle[0])

            if self.far==1 and self.heat<=120:
                self._shoot(1,10)
            elif self.far==2 and self.heat<130:
                self._shoot(1,8)
            elif self.far>=3 and self.far<=5 and self.heat<180:
                self._shoot(1,5)
            elif self.far>5 and self.heat<200:
                self._shoot(1,3)
                
            rospy.loginfo('pitch '+str(angle[0])+' yaw '+str(angle[1]))
            # self.loss_enemy=0
            self.last_pitch=angle[0]
            self.last_far=self.far
        else:
            if(self.gimbal_angle<3.14/2 and self.gimbal_angle>-3.14/2 and self.gimbal_angle!=0 and self.loss_enemy>50):
                
                #self._ctrlinfo_pub.publish(False,False,self.gimbal_angle,self.perspective_pitch)
                if self.gimbalswing==False:
                    self.gimbal_client.cancel_goal()
                    self.gimbalswing=True
                self._ctrlinfo_pub.publish(False,False,self.gimbal_angle,0.13)
            elif self.loss_enemy>100:
                
                self.gimbal_client.send_goal(GimbalSwingActionGoal(goal=True))
                self.gimbalswing=False
                #self._ctrlinfo_pub.publish(False,False,0,0.15)
                # self.kf_filter=KalmanFilter(self.DT, self.U, self.STD_ACC, self.STD_MEAS)
                self.loss_enemy=0
    def get_center(self, points):
        x_min, x_max, y_min, y_max = self.sortbbox(points)
        x_c = int((x_min+x_max)/2)
        y_c = int((y_min+y_max)/2)
        center_points = [x_c, y_c]
        return center_points

    def get_filter_points(self, points):
        center_points = self.get_center(points)
        (x_filter, y_filter, v_x, v_y) = self.kf2D_filter.predict()
        self.kf2D_filter.update(center_points)
        x_err = int(x_filter.tolist()[0] - center_points[0] + 0.05 * v_x)
        y_err = int(y_filter.tolist()[0] - center_points[1] + 0.05 * v_y)
        error = [x_err, y_err]
        for i in range(0, len(points)):
            for j in range(0, 2):
                points[i][j] = error[j] + points[i][j]
        return points
    def get_bg_filter_feedback(self, img):
        move_bg = get_background_move(img)
        (x_filter_bg, y_filter_bg, v_x_bg, v_y_bg) = self.kf2D_filter_bg.predict()
        self.kf2D_filter_bg.update(move_bg)
        x_filter_bg = x_filter_bg + 0.05 * v_x_bg
        y_filter_bg = y_filter_bg + 0.05 * v_y_bg
        filter_mobe_bg = [x_filter_bg, y_filter_bg]
        x_err_bg = np.array(filter_mobe_bg) - self.last_bg
        self.last_bg = filter_mobe_bg
        return x_err_bg
    def angle_callback(self,data):
        self.gimbal_angle=data.enemy_threa-data.self_threa
        #self.perspective_distance=data.
        if(self.gimbal_angle>1.5*3.14 and self.gimbal_angle<2*3.14):
            self.gimbal_angle=self.gimbal_angle-2*3.14
        elif(self.gimbal_angle<-1.5*3.14 and self.gimbal_angle>-2*3.14):
            self.gimbal_angle= 2*3.14+self.gimbal_angle
            
        #pitch=arctan(distance/height)  height=30cm
        #self.perspective_pitch=np.arctan(self.perspetice_distance/30)  
        
    def angle_test_callback(self,data):
        self.gimbal_angle=data.enemy_threa-data.self_threa
        self.perspectice_distance=data.distance
        if(self.gimbal_angle>1.5*3.14 and self.gimbal_angle<2*3.14):
            self.gimbal_angle=self.gimbal_angle-2*3.14
        elif(self.gimbal_angle<-1.5*3.14 and self.gimbal_angle>-2*3.14):
            self.gimbal_angle= 2*3.14+self.gimbal_angle
            
        #pitch=arctan(distance/height)  height=30cm
        self.perspective_pitch=np.arctan(300/self.perspectice_distance)  
class KalmanFilter2D():
    def __init__(self, dt, u_x, u_y, std_acc, x_std_meas, y_std_meas):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u_x: acceleration in x-direction
        :param u_y: acceleration in y-direction
        :param std_acc: process noise magnitude
        :param x_std_meas: standard deviation of the measurement in x-direction
        :param y_std_meas: standard deviation of the measurement in y-direction
        """

        # Define sampling time
        self.dt = dt

        # Define the  control input variables
        self.u = np.array([[u_x], [u_y]])

        # Intial State
        self.x = np.array([[400], [300], [0], [0]])

        # Define the State Transition Matrix A
        self.A = np.array([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        # Define the Control Input Matrix B
        self.B = np.array([[(self.dt ** 2) / 2, 0],
                            [0, (self.dt ** 2) / 2],
                            [self.dt, 0],
                            [0, self.dt]])

        # Define Measurement Mapping Matrix
        self.H = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        # Initial Process Noise Covariance
        self.Q = np.array([[(self.dt ** 3) / 3, 0, (self.dt ** 3) / 2, 0],
                            [0, (self.dt ** 3) / 3, 0, (self.dt ** 2) / 2],
                            [(self.dt ** 2) / 2, 0, self.dt, 0],
                            [0, (self.dt ** 2) / 2, 0, self.dt]]) * std_acc ** 2

        # Initial Measurement Noise Covariance
        self.R = np.array([[x_std_meas ** 2, 0],
                            [0, y_std_meas ** 2]])

        # Initial Covariance Matrix
        self.P = np.eye(self.A.shape[1])

    def predict(self):
        # Update time state
        # x_k =Ax_(k-1) + Bu_(k-1)     Eq.(9)
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # Calculate error covariance
        # P= A*P*A' + Q               Eq.(10)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):

        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  # Eq.(11)

        z = np.array(z).reshape((2,1))
        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))  # Eq.(12)

        I = np.eye(self.H.shape[1])

        # Update error covariance matrix
        self.P = np.dot(I - (np.dot(K, self.H)), self.P)  # Eq.(13)
        return self.x

    def position_reset(self, x, y):
        self.x[0] = copy.deepcopy(x)
        self.x[1] = copy.deepcopy(y)
        print('kalman init position para Reset !!')

    def velocity_reset(self, x_v, v_y):
        self.x[2] = copy.deepcopy(x_v)
        self.x[3] = copy.deepcopy(v_y)
        print('kalman init velocity para Reset !!')

vx = np.arange(1,801,dtype=np.float32).reshape(800,1)
vy = np.arange(1,601,dtype=np.float32).reshape(1,600)
def get_background_move(img):
    t1= time.time()
    # img = gpuarray.to_gpu(img)
    I = np.sum(img)
    x_sum = np.dot(np.dot(img, vx),np.ones((1,800)))[0][0]
    y_sum = np.dot(np.ones((600,1)),np.dot(vy, img))[0][0]
    t1 = time.time()-t1
    print(f"time assume: {t1}")
    move_bg = [int(x_sum / I), int(y_sum / I)]
    return move_bg


if __name__ == "__main__":
    detector = Detector()
    device_manager = gx.DeviceManager()
    dev_num, dev_info_list = device_manager.update_device_list()
    if dev_num == 0:
        print('no device')
        sys.exit(1)
    str_sn = dev_info_list[0].get("sn")
    cam = device_manager.open_device_by_sn(str_sn)
    cam.BalanceWhiteAuto.set(gx.GxAutoEntry.CONTINUOUS)
    cam.ExposureTime.set(800)
    cam.stream_on()
    
    # fourcc = cv2.VideoWriter_fourcc(*'XVID' )
    # # fps = cam.AcquisitionFrameRate.get()
    # size = (cam.Width.get(),cam.Height.get())
    # filename = './out.avi'
    # out = cv2.VideoWriter(filename, fourcc, 30, size)
    
    rospy.loginfo("Camera Config Done")
    detector.t1 = time.time()

    ROI_find=False
    while(not rospy.is_shutdown()):
        t1=time.time()
        try:
            cam.data_stream[0].flush_queue()
            t1 = time.time()
            raw1_image = cam.data_stream[0].get_image()
            rgb1_image = raw1_image.convert("RGB")
            numpy1_image = rgb1_image.get_numpy_array()
        except Exception as exception:
            #print("device error:%s" %exception)
            try:
                cam.stream_off()
                cam.close_device()
            except:
                while (1):
                    device_manager=gx.DeviceManager()
                    dev_num,dev_info=device_manager.update_device_list()
                    if dev_num==1:
                        break
                    else:
                        time.sleep(1)
            while(1):
                try:
                    device_manager=gx.DeviceManager()
                    dev_num,_=device_manager.update_device_list()
                    cam=device_manager.open_device_by_sn(str_sn)
                    cam.BalanceWhiteAuto.set(gx.GxAutoEntry.CONTINUOUS)
                    cam.ExposureTime.set(800)
                    break
                except:
                    try:
                        cam.close_device()
                    except:
                        continue
                    time.sleep(1)
            cam.stream_on()
            continue
        image_raw = cv2.cvtColor(numpy1_image, cv2.COLOR_RGB2BGR)
        # gray = cv2.cvtColor(numpy1_image, cv2.COLOR_RGB2GRAY)
        if ROI_find:
            ROI_find,len_points,points=detector.ROI(image_raw,last_points)
            if len_points!=0 :
                # bg_bias = detector.get_bg_filter_feedback(gray)
                # for i in range(0, len(points[0])):
                #     for j in range(0, 2):
                #         points[0][i][j] = int(points[0][i][j] - bg_bias[j])
                # filter_points = detector.get_filter_points(copy.deepcopy(points[0]))
                # last_points = points[0]
                # if detector.loss_enemy >= 1:
                #     for i in range(0, 5):
                #         filter_points = detector.get_filter_points(copy.deepcopy(points[0]))
                #     detector.loss_enemy = 0
                # angle = detector.classic_calcu_angle(filter_points)
                if len_points==1:
                    angle = detector.classic_calcu_angle_car2(points[0])
                    detector.control_2(angle)
                    if angle[2]>5000:
                        ROI_find=False
                else:
                    min_distance=1000000
                    min_distance_enemy=[]
                    index=0
                    for i in range(len_points):
                        angle=detector.classic_calcu_angle_car2(points[i])
                        if(angle[2]<min_distance):
                            min_distance=angle[2]
                            min_distance_enemy=copy.deepcopy(angle)
                            
                    detector.control_2(min_distance_enemy)
                    if min_distance_enemy[2]>5000:
                        ROI_find=False      
                    detector.loss_enemy=0
                    detector._enemy_pub.publish(0,0,0,0,True)
            else:
                rospy.loginfo('in ROI No detection')
                detector._shoot(0,0)
                detector.loss_enemy = detector.loss_enemy + 1
                detector.control_2(None)
                detector.find_enemy=0 
                detector._enemy_pub.publish(0,0,0,0,False)
        else:
            len_points,points=detector.infer(image_raw)
            if len_points!=0:
                # bg_bias = detector.get_bg_filter_feedback(gray)
                # for  i in range(0, len(points[0])):
                #     for j in range(0, 2):
                #         points[0][i][j] = int(points[0][i][j] - bg_bias[j])                
                # filter_points = detector.get_filter_points(copy.deepcopy(points[0]))
                last_points = points[0]
                # if detector.loss_enemy >= 1:
                #     for i in range(0, 5):
                #         filter_points = detector.get_filter_points(copy.deepcopy(points[0]))
                #     detector.loss_enemy = 0
                # angle = detector.classic_calcu_angle(filter_points)
                if len_points==1:
                    angle = detector.classic_calcu_angle_car2(points[0])
                    detector.control_2(angle)
                    if angle[2]>5000:
                        ROI_find=False
                else:
                    min_distance=1000000
                    min_distance_enemy=[]
                    #index=0
                    for i in range(len_points):
                        angle=detector.classic_calcu_angle_car2(points[i])
                        if(angle[2]<min_distance):
                            min_distance=angle[2]
                            min_distance_enemy=copy.deepcopy(angle)
                            
                    detector.control_2(min_distance_enemy)
                    if min_distance_enemy[2]>5000:
                        ROI_find=False      
                    detector.loss_enemy=0
                    detector._enemy_pub.publish(0,0,0,0,True)
                detector.loss_enemy=0
                detector._enemy_pub.publish(0,0,0,0,True)
            else:
                rospy.loginfo('No enemy')
                detector._shoot(0,0)
                detector.loss_enemy = detector.loss_enemy + 1
                detector.control_2(None)
                detector.find_enemy=0 
                detector._enemy_pub.publish(0,0,0,0,False)
    
        '''
        enemy_number,bbox=detector.infer(image_raw)
        print("box:",bbox)
        if bbox!=None:
            if enemy_number == 1:
                angle = detector.classic_calcu_angle(bbox[0])
                detector.control(angle)
                
                cv2.line(image_raw,tuple(bbox[0]),tuple(bbox[1]),(255,0,0))
                cv2.line(image_raw,tuple(bbox[1]),tuple(bbox[2]),(255,0,0))
                cv2.line(image_raw,tuple(bbox[2]),tuple(bbox[3]),(255,0,0))
                cv2.line(image_raw,tuple(bbox[3]),tuple(bbox[0]),(255,0,0))
                print("angle",angle)
                
            else:
                angle_list=[]
                for i in range(enemy_number):
                    angle=detector.classic_calcu_angle(bbox[i])
                    angle_list.append(angle)
                num=detector.select_enemy(angle_list)
                
                detector.control(angle_list[num])
            # out.write(image_raw)
        '''
        #out.write(image_raw)

        t2=time.time()
        print("time:",1000*(t2-t1))
    cam.stream_off()
