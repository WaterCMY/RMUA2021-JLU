#!/usr/bin/env python3
import ctypes
import os
import random
import sys
import threading
import time
import math
import cv2
import gxipy as gx
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import rospy
import tensorrt as trt
import torch
import csv
import torchvision
from roborts_msgs.msg import GimbalAngle,PyArmorInfo,GameStatus,RobotHeat,RobotAngle,RobotStatus,GameRobotBullet,kalman,YawData
from roborts_msgs.srv import FricWhl, ShootCmd,ShootSpeed

INPUT_W = 608
INPUT_H = 608
CONF_THRESH = 0.4
IOU_THRESHOLD = 0.4
GUN_Y=135

DT=0.02
U=1
STD_ACC=10
STD_MEAS=0.5

#1-blue 2-red
enemy_color=0
object_3d_points = np.array(([-71, -63.5, 0],                 #xmin ymin
                                     [-71, 63.5, 0],                  #xmin ymax
                                     [71, 63.5, 0],                  #xmax ymax
                                     [71, -63.5, 0]), dtype=np.double) #xmax ymin
classic_object_3d_points=np.array(([-71, -27.5, 0],                 #xmin ymin
                                     [-71, 27.5, 0],                  #xmin ymax
                                     [71, 27.5, 0],                  #xmax ymax
                                     [71, -27.5, 0]), dtype=np.double) #xmax ymin
yaw_list=[]
pitch_list=[]
angle_list=[]

def select_target(box_list, cls_list, score_list, ENEMY_COLOR):
    #armor_box=[0,0,0,0]
    direction=7
    tmp_armor_score = 0
    see=False
    #print("select:",box_list)
    #print("select",cls_list)
    if len(box_list)==0:
        armor_box=[0,0,0,0]
        #return [0,0,0,0],direction
    tmp_armor_score = 0
    for box, cls, score in zip(box_list, cls_list, score_list):
        #tmp_armor_score = 0

        if int(cls.cpu().numpy()) == ENEMY_COLOR and score > tmp_armor_score:
            tmp_armor_score = score
            armor_box = box
            see=True

    if tmp_armor_score ==0:
        armor_box=[0,0,0,0]

    for box, cls, score in zip(box_list, cls_list, score_list):
        tmp_direc_score = 0
        if cls >=2 and score > tmp_direc_score:
            if box[0] < armor_box[0] and box[2] > armor_box[2]:
                direction = [box, cls]
    return armor_box, direction,see


def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    param:
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return

    """
    tl = (
        line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    #print('box: (%d,%d), (%d,%d) ' %(c1[0],c1[1],c2[0],c2[1]))

    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )
        #print('box: (%d,%d), (%d,%d) ' %(c1[0],c1[1],c2[0],c2[1]))

def calcu_angle(self, bbox):

    if bbox[0]!=0:
        # [ymin xmin ymax xmax]
        #box = [bbox[1], bbox[0], bbox[1]+bbox[3], bbox[0]+bbox[2]]
        box=[bbox[1],bbox[0],bbox[3],bbox[2]]
        object_2d_point = np.array(([box[1],box[0]],[box[1],box[2]],
                                        [box[3],box[2]],[box[3],box[0]]),
                                        dtype=np.double)

        _, _, tvec = cv2.solvePnP(object_3d_points, object_2d_point,
                                        self.camera_matrix, self.dist_coefs,
                                        cv2.SOLVEPNP_EPNP)
        x=tvec[0][0]
        y=tvec[1][0]
        #y=tvec[1][0]-GUN_Y
        z=tvec[2][0]
        d=np.sqrt(x*x+z*z)
        distance=np.sqrt(x*x+y*y+z*z)
        yaw   = float(np.arctan2(x, z))


        '''
        if 283/d<1 or 283/d >-1 :
            pitch=math.asin(283/d)
        else:
            return None
        #print("first pitch ",pitch)
        camera_height=distance*np.sin(pitch)
        gun_height=camera_height-GUN_Y
        #gun_pitch=gun_height/(distance*np.cos(pitch))
        #pitch=float(np.arctan(gun_pitch))
        pitch=float(np.arctan2(gun_height,(distance*np.cos(pitch))))
        #print("second pitch ",pitch)
        '''


        z1=np.sqrt(distance*distance-80089)
        #283-90
        pitch=float(np.arctan(193/z1))
        print("distance",distance)

        if distance>2200 and distance<2980:

            pitch=pitch-0.08
            self.far=3
        elif distance >2980 and distance<3700:
            pitch=pitch-0.09
            self.far=4
        elif distance<2200 and distance >1500:
            pitch=pitch-0.07
            self.far=2
        elif distance>3700:
            pitch=pitch-0.095
            self.far=5
        elif distance<1000:
            pitch=pitch
            self.far=1
        else:
            pitch=pitch-0.06
            self.far=1
        yaw=yaw-0.015
        #pitch=float(np.arctan((y-90)/d))

        yaw=0.8*yaw
        return [pitch, yaw]
    else:
        return None


def classic_calcu_angle(self,bbox):
    classic_object_2d_point=np.array((bbox[0],bbox[1],bbox[2],bbox[3]),dtype=np.double)
    _, _, tvec = cv2.solvePnP(classic_object_3d_points, classic_object_2d_point,
                                self.camera_matrix, self.dist_coefs,
                                cv2.SOLVEPNP_EPNP)

    x=tvec[0][0]
    y=tvec[1][0]
    #y=tvec[1][0]-GUN_Y
    z=tvec[2][0]
    d=np.sqrt(x*x+z*z)
    distance=np.sqrt(x*x+y*y+z*z)
    yaw   = float(np.arctan2(x, z))

    z1=np.sqrt(distance*distance-80089)
    #283-90
    pitch=float(np.arctan(193/z1))

    return [pitch,yaw]

class Detector(object):
    """
    description: A YOLOv5 class that warps TensorRT ops, preprocess and postprocess ops.
    """

    def __init__(self, engine_file_path):
        # Create a Context on this device
        rospy.init_node('armor_detection_node')

        self.cfx = cuda.Device(0).make_context()
        self._enemy_pub=rospy.Publisher('/PyArmorInfo',
                                        PyArmorInfo,queue_size=1)
        self._ctrlinfo_pub = rospy.Publisher('/cmd_gimbal_angle',
                                             GimbalAngle, queue_size=5)

        self._fricwhl_client = rospy.ServiceProxy("/cmd_fric_wheel",FricWhl)
        self._shoot_client = rospy.ServiceProxy("/cmd_shoot",ShootCmd)
        self.speed_client=rospy.ServiceProxy("/set_shoot_speed",ShootSpeed)
        self._can_ctrl = True
        undet_count = 40
        #num_bullets = 100
        self.bullet=10
        self.camera_matrix = np.array(([1716.0988996628, 1.43642153020885, 370.009711441846],
                                       [0, 1720.87835300139, 270.743138066265],
                                       [0, 0, 1.0]), dtype=np.double)
        self.dist_coefs = np.array([-0.158618771341394, 1.74637984736916, -0.00436517631477145, 0.00459322920933054, 0], dtype=np.double)
        stream = cuda.Stream()
        TRT_LOGGER = trt.Logger(trt.Logger.INFO)
        runtime = trt.Runtime(TRT_LOGGER)

        # Deserialize the engine from file
        with open(engine_file_path, "rb") as f:
            engine = runtime.deserialize_cuda_engine(f.read())
        context = engine.create_execution_context()

        host_inputs = []
        cuda_inputs = []
        host_outputs = []
        cuda_outputs = []
        bindings = []

        for binding in engine:
            size = trt.volume(engine.get_binding_shape(binding)) * engine.max_batch_size
            dtype = trt.nptype(engine.get_binding_dtype(binding))
            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)
            # Append the device buffer to device bindings.
            bindings.append(int(cuda_mem))
            # Append to the appropriate list.
            if engine.binding_is_input(binding):
                host_inputs.append(host_mem)
                cuda_inputs.append(cuda_mem)
            else:
                host_outputs.append(host_mem)
                cuda_outputs.append(cuda_mem)

        # Store
        self.stream = stream
        self.context = context
        self.engine = engine
        self.host_inputs = host_inputs
        self.cuda_inputs = cuda_inputs
        self.host_outputs = host_outputs
        self.cuda_outputs = cuda_outputs
        self.bindings = bindings
        self._ctrlinfo_pub.publish(False,False,0,0.1)
        #self._set_fricwhl(True)
        self.loss_enemy=0
        self.hot=False
        self.start=False
        self.gimbal_angle=0
        self.last_pitch=0.1
        self.my_id=0
        self.enemycolor=0
        self.far=0
        self.last_far=0
        #self.file=open('./angle.csv','w',newline='')
        #self.writer=csv.writer(self.file)
        rospy.Subscriber("/game_status",GameStatus,self.game_callback)
        rospy.Subscriber("/robot_heat",RobotHeat,self.heat_callback)
        #rospy.Subscriber("/robot_angle_pub",RobotAngle,self.angle_callback)
        rospy.Subscriber("/robot_status",RobotStatus,self.id_callback)
        rospy.Subscriber("/game_robot_bullet",GameRobotBullet,self.bullet_callback)
        rospy.Subscriber("/kalman",kalman,self.kalman_callback)
        self.pub_yaw_data = rospy.Publisher("/yaw_data_recordor",YawData,queue_size=1)
        self.DT=0.05
        self.U=1
        self.STD_ACC=2
        self.STD_MEAS=0.5

        self.kf_filter = KalmanFilter(self.DT, self.U, self.STD_ACC, self.STD_MEAS)
    def kalman_callback(self,data):
        self.DT=data.DT
        self.U=data.U
        self.STD_ACC=data.STD_ACC
        self.STD_MEAS=data.STD_MEAS
        self.kf_filter=KalmanFilter(self.DT,self.U,self.STD_ACC,self.STD_MEAS)
        rospy.loginfo("receive kalman %f %f %f %f ",data.DT,data.U,data.STD_ACC,data.STD_MEAS)
    def game_callback(self,data):
        #print("game statue",data.game_status)
        #rospy.loginfo("game status %d"%data.game_status)
        if(data.game_status==4):
            self.start=True
            self._set_fricwhl(True)

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
    def bullet_callback(self,data):
        if(self.my_id==1):
            self.bullet=data.red1
        elif(self.my_id==2):
            self.bullet=data.red2
        elif(self.my_id==3):
            self.bullet==data.blue1
        elif(self.my_id==4):
            self.bullet==data.blue2
    def angle_callback(self,data):
        self.gimbal_angle=data.enemy_threa-data.self_threa
        if(self.gimbal_angle>1.5*3.14 and self.gimbal_angle<2*3.14):
            self.gimbal_angle=self.gimbal_angle-2*3.14
        elif(self.gimbal_angle<-1.5*3.14 and self.gimbal_angle>-2*3.14):
            self.gimbal_angle=2*3.14+self.gimbal_angle

    def heat_callback(self,data):
        #print(data.shooter_heat)
        #rospy.loginfo("heat %d"%data.shooter_heat)
        if(data.shooter_heat>200):
            self.hot=True
        else:
            self.hot=False

    def _set_fricwhl(self,can_start):
        #rospy.wait_for_service("cmd_fric_wheel")
        try:
            resp = self._fricwhl_client.call(can_start)
            rospy.loginfo("Message From fricwheelserver:%s"%resp.received)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")

    def _shoot(self,shoot_mode, shoot_number):

        #rospy.wait_for_service("cmd_fric_wheel")
        try:
            resp = self._shoot_client.call(shoot_mode, shoot_number)
            #rospy.loginfo("Message From shootserver:%s"%resp.received)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")
    def speed_srv(self,speed):
        try:
            self.speed_client.call(speed)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")

    def find_contours_withlightbox(binary, lt,rb,enemy_color):  # find contours and main screening section
        contour = []
        C_lights = []
        binary = binary[lt[1]:rb[1], lt[0]:rb[0]]
        B, G, R = cv2.split(binary)
        if enemy_color==0:
            dst = cv2.subtract(B, R)
        ret, binary = cv2.threshold(dst, 40, 255, cv2.THRESH_BINARY)
        ##--------------------------------1. find contours-----------------------------##
        contours, heriachy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(binary,contours,-1,(0,0,255),3)

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
                    if area / ellipseArea >= 0.7 and area >= 200 and area <= 3000:  # 判断凸度(Solidity) 和轮廓面积大小
                        rect = cv2.minAreaRect(contours[i])  # 根据轮廓得到外接矩阵 长宽不固定，靠近x轴定义长
                        # if rect[1][0]/rect[1][1] < 0.8 or rect[1][0]/rect[1][1] > 1.25:      #判断矩阵长宽比？？？
                        a = tuple(((rect[1][0] * 1.1, rect[1][1] * 1.1)))  # 外接矩形(宽，长)？？？
                        rect = tuple((rect[0], a, rect[2]))
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        contour.append((rect, box, area, x, y))  # 保存符合要求的外接矩形及四点坐标，面积和中心x坐标

        ##-----------------------------3. Find the right pair of lights----------------##
        ##campare each pair of contours, The approximate parallel, length-width ratio

        if len(contour) >= 2:
            for i in range(len(contour)):
                j = i + 1
                while j < len(contour):
                    if contour[i][0][2] > -45:  # 外接矩形定义：与碰到的矩形的第一条边的夹角。并且这个边的边长是width，另一条边边长是height。对旋转角修正
                        orientation_i = contour[i][0][2] - 90
                    else:
                        orientation_i = contour[i][0][2]
                    if contour[j][0][2] > -45:
                        orientation_j = contour[j][0][2] - 90
                    else:
                        orientation_j = contour[j][0][2]
                    # if abs(orientation_i-orientation_j) < 40.0:   #判断是否平行
                    if contour[i][2] / contour[j][2] >= 0.4 and contour[i][2] / contour[j][2] <= 2.5:  # 判断矩形面积比
                        # if abs(contour[i][3]-contour[j][3]) > 50 and abs(contour[i][4]-contour[j][4])<100 and abs(contour[i][3]-contour[j][3]) < 150:
                        if abs(contour[i][3] - contour[j][3]) > 50 and abs(contour[i][4] - contour[j][4]) < 200:
                            couple_light = tuple((contour[i], contour[j]))
                            C_lights.append(couple_light)

                    j += 1

        ##----------------------------4. Find best pair of lights----------------------##

        if len(C_lights) > 1:
            distance = []
            for i in range(len(C_lights)):
                d = np.sqrt((C_lights[i][0][3] - C_lights[i][1][3]) ** 2 + (
                            C_lights[i][0][4] - C_lights[i][1][4]) ** 2)  # 找到两个x坐标最相近的灯条
                distance.append(d)

            index = np.argmin(distance)
        elif len(C_lights) == 1:
            index = 0
        else:
            return 0, None
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
        box = cv2.boxPoints(Rect)
        box = np.int0(box)
        box=box.tolist()
        box.append(box.pop(0))

        box[0][0]+=lt[0]
        box[0][1] += lt[1]
        box[1][0] += lt[0]
        box[1][1] += lt[1]

        box[2][0]+=lt[0]
        box[2][1] += lt[1]
        box[3][0] += lt[0]
        box[3][1] += lt[1]
        return C_lights, box

    
    def find_light(self,src,lt,rb,enemy_color):
        src=src[lt[1]:rb[1],lt[0]:rb[0]]
        if enemy_color==0:
            B, G, R = cv2.split(src)
            dst = cv2.subtract(B, R)
            ret, binay = cv2.threshold(dst, 40, 255, cv2.THRESH_BINARY)
            ctrs, hier = cv2.findContours(binay.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            left_x_list = []
            left_y_list = []
            right_x_list = []
            right_y_list = []
            left_x_max = 0
            left_x_min = 0
            left_y_max = 0
            left_y_min = 0
            right_x_max = 0
            right_x_min = 0
            right_y_max = 0
            right_y_min = 0
            if len(ctrs)==2:
                for i in range(len(ctrs[0])):
                    left_x_list.append(ctrs[0][i][0][0])
                    left_y_list.append(ctrs[0][i][0][1])
                for i in range(len(ctrs[1])):
                    right_x_list.append(ctrs[1][i][0][0])
                    right_y_list.append(ctrs[1][i][0][1])
            else:
                return None
            left_x_min = min(left_x_list)

            left_y_max = max(left_y_list)
            left_y_min = min(left_y_list)

            right_x_max = max(right_x_list)

            right_y_max = max(right_y_list)
            right_y_min = min(right_y_list)

            left_top=((lt[0]+left_x_min),(lt[1]+left_y_min))
            left_bottom=((lt[0]+left_x_min),(lt[1]+left_y_max))
            right_bottom=((lt[0]+right_x_max),(lt[1]+right_y_max))
            right_top=((lt[0]+right_x_max),(lt[1]+right_y_min))
            return [left_top,left_bottom,right_bottom,right_top]

    def infer(self, image_raw):
        #threading.Thread.__init__(self)
        # Make self the active context, pushing it on top of the context stack.
        self.cfx.push()
        # Restore
        stream = self.stream
        context = self.context
        engine = self.engine
        host_inputs = self.host_inputs
        cuda_inputs = self.cuda_inputs
        host_outputs = self.host_outputs
        cuda_outputs = self.cuda_outputs
        bindings = self.bindings
        # Do image preprocess
        input_image, image_raw, origin_h, origin_w = self.preprocess_image(
            image_raw
        )
        # Copy input image to host buffer
        np.copyto(host_inputs[0], input_image.ravel())
        # Transfer input data  to the GPU.
        cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)
        # Run inference.
        context.execute_async(bindings=bindings, stream_handle=stream.handle)
        # Transfer predictions back from the GPU.
        cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
        # Synchronize the stream
        stream.synchronize()
        # Remove any context from the top of the context stack, deactivating it.
        self.cfx.pop()
        # Here we use the first row of output in that batch_size = 1
        output = host_outputs[0]
        # Do postprocess
        result_boxes, result_scores, result_classid = self.post_process(
            output, origin_h, origin_w
        )
        # Draw rectangles and labels on the original image
        '''
        for i in range(len(result_boxes)):
            box = result_boxes[i]
            plot_one_box(
                box,
                image_raw,
                label="{}:{:.2f}".format(
                    categories[int(result_classid[i])], result_scores[i]
                ),
            )
        '''
        #cv2.imshow("video",image_raw)

        #box, direc,can_see= select_target(result_boxes,  result_classid,result_scores, self.enemycolor)
        box,direc,can_see=select_target(result_boxes,result_classid,result_scores,0)
        boundingbox=[0,0,0,0]
        boundingbox[:]=box
        print("bbox",boundingbox)
        points=None
        if boundingbox!=[0,0,0,0]:
            
            try:
                #points=self.find_light(image_raw,[int(boundingbox[0]),int(boundingbox[1])],[int(boundingbox[2]),int(boundingbox[3])],0)
                _,points=self.find_contours_withlightbox(image_raw,[int(boundingbox[0]),int(boundingbox[1])],[int(boundingbox[2]),int(boundingbox[3])],0)
            except:
                print("error")
            print("points",points)
        self._enemy_pub.publish(0,0,0,0,can_see)
        if not rospy.is_shutdown():
            #print('bbox',boundingbox)
            if points==None:
                angle = calcu_angle(self,bbox=boundingbox)
            else:
                rospy.loginfo("classic calculate angle")
                angle = classic_calcu_angle(self,points)
            if self._can_ctrl :
                if (angle is not None) :
                    #yaw_predict_angle=self.kf_filter.predict()
                    #print(angle[1])
                    #self.kf_filter.update(angle[1])
                    #angle[1]=yaw_predict_angle[0]
                    
                    yaw_predict_angle=self.kf_filter.predict()
                    angle_expand = 100*angle[1]                   
                    self.kf_filter.update(angle_expand)
                    self.pub_yaw_data.publish(angle[1],yaw_predict_angle[0]/100)
                    # print(angle_expand)
                    # angle[1]=yaw_predict_angle[0]/100
                    
                    if (self.far-self.last_far)!=0:
                        if self.far==1:
                            self.speed_srv(1250)

                        elif self.far==2:
                            self.speed_srv(1280)

                        elif self.far==3:
                            self.speed_srv(1340)
                        elif self.far==4:
                            self.speed_srv(1350)
                        elif self.far==5:
                            self.speed_srv(1360)
                    self._ctrlinfo_pub.publish(True,False,-angle[1], angle[0])
                    print("far:",self.far)
                    print("last far:",self.last_far)
                    if self.far>=2:
                        self._shoot(2,3)
                    else:
                        self._shoot(2,5)
                    rospy.loginfo('pitch '+str(angle[0])+' yaw '+str(angle[1]))
                    self.loss_enemy=0
                    self.last_pitch=angle[0]
                    self.last_far=self.far
                else:
                    rospy.loginfo("no enemy ")
                    self._shoot(0,0)
                    self.loss_enemy=self.loss_enemy+1
                    if(self.gimbal_angle<3.14/2 and self.gimbal_angle>-3.14/2 and self.gimbal_angle!=0 and self.loss_enemy>10):
                        self._ctrlinfo_pub.publish(False,False,self.gimbal_angle,self.last_pitch)
                    elif self.loss_enemy>30:
                        #self._ctrlinfo_pub.publish(False,False,0,0.1)
                        self.loss_enemy=0

        return box

    def destroy(self):
        # Remove any context from the top of the context stack, deactivating it.
        self.cfx.pop()

    def preprocess_image(self, image_raw):
        """
        description: Read an image from image path, convert it to RGB,
                     resize and pad it to target size, normalize to [0,1],
                     transform to NCHW format.
        param:
            input_image_path: str, image path
        return:
            image:  the processed image
            image_raw: the original image
            h: original height
            w: original width
        """
        h, w, c = image_raw.shape
        image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)
        # Calculate widht and height and paddings
        r_w = INPUT_W / w
        r_h = INPUT_H / h
        if r_h > r_w:
            tw = INPUT_W
            th = int(r_w * h)
            tx1 = tx2 = 0
            ty1 = int((INPUT_H - th) / 2)
            ty2 = INPUT_H - th - ty1
        else:
            tw = int(r_h * w)
            th = INPUT_H
            tx1 = int((INPUT_W - tw) / 2)
            tx2 = INPUT_W - tw - tx1
            ty1 = ty2 = 0
        # Resize the image with long side while maintaining ratio
        image = cv2.resize(image, (tw, th))
        # Pad the short side with (128,128,128)
        image = cv2.copyMakeBorder(
            image, ty1, ty2, tx1, tx2, cv2.BORDER_CONSTANT, (128, 128, 128)
        )
        image = image.astype(np.float32)
        # Normalize to [0,1]
        image /= 255.0
        # HWC to CHW format:
        image = np.transpose(image, [2, 0, 1])
        # CHW to NCHW format
        image = np.expand_dims(image, axis=0)
        # Convert the image to row-major order, also known as "C order":
        image = np.ascontiguousarray(image)
        return image, image_raw, h, w

    def xywh2xyxy(self, origin_h, origin_w, x):
        """
        description:    Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
        param:
            origin_h:   height of original image
            origin_w:   width of original image
            x:          A boxes tensor, each row is a box [center_x, center_y, w, h]
        return:
            y:          A boxes tensor, each row is a box [x1, y1, x2, y2]
        """
        y = torch.zeros_like(x) if isinstance(x, torch.Tensor) else np.zeros_like(x)
        r_w = INPUT_W / origin_w
        r_h = INPUT_H / origin_h
        if r_h > r_w:
            y[:, 0] = x[:, 0] - x[:, 2] / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2 - (INPUT_H - r_w * origin_h) / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2 - (INPUT_H - r_w * origin_h) / 2
            y /= r_w
        else:
            y[:, 0] = x[:, 0] - x[:, 2] / 2 - (INPUT_W - r_h * origin_w) / 2
            y[:, 2] = x[:, 0] + x[:, 2] / 2 - (INPUT_W - r_h * origin_w) / 2
            y[:, 1] = x[:, 1] - x[:, 3] / 2
            y[:, 3] = x[:, 1] + x[:, 3] / 2
            y /= r_h

        return y

    def post_process(self, output, origin_h, origin_w):
        """
        description: postprocess the prediction
        param:
            output:     A tensor likes [num_boxes,cx,cy,w,h,conf,cls_id, cx,cy,w,h,conf,cls_id, ...]
            origin_h:   height of original image
            origin_w:   width of original image
        return:
            result_boxes: finally boxes, a boxes tensor, each row is a box [x1, y1, x2, y2]
            result_scores: finally scores, a tensor, each element is the score correspoing to box
            result_classid: finally classid, a tensor, each element is the classid correspoing to box
        """
        # Get the num of boxes detected
        num = int(output[0])
        # Reshape to a two dimentional ndarray
        pred = np.reshape(output[1:], (-1, 6))[:num, :]
        # to a torch Tensor
        pred = torch.Tensor(pred).cuda()
        # Get the boxes
        boxes = pred[:, :4]
        # Get the scores
        scores = pred[:, 4]
        # Get the classid
        classid = pred[:, 5]
        # Choose those boxes that score > CONF_THRESH
        si = scores > CONF_THRESH
        boxes = boxes[si, :]
        scores = scores[si]
        classid = classid[si]
        # Trandform bbox from [center_x, center_y, w, h] to [x1, y1, x2, y2]
        boxes = self.xywh2xyxy(origin_h, origin_w, boxes)
        # Do nms
        indices = torchvision.ops.nms(boxes, scores, iou_threshold=IOU_THRESHOLD).cpu()
        result_boxes = boxes[indices, :].cpu()
        result_scores = scores[indices].cpu()
        result_classid = classid[indices].cpu()
        return result_boxes, result_scores, result_classid



class Tracker():
    def __init__(self):

        rospy.init_node('armor_detection_node',anonymous=True)

        self._ctrlinfo_pub = rospy.Publisher('/cmd_gimbal_angle',
                                             GimbalAngle, queue_size=10)
        self._fricwhl_client = rospy.ServiceProxy("/cmd_fric_wheel",FricWhl)
        self._shoot_client = rospy.ServiceProxy("/cmd_shoot",ShootCmd)
        self._can_ctrl = True
        undet_count = 40
        #num_bullets = 100
        self.camera_matrix = np.array(([1716.0988996628, 1.43642153020885, 370.009711441846],
                                       [0, 1720.87835300139, 270.743138066265],
                                       [0, 0, 1.0]), dtype=np.double)
        self.dist_coefs = np.array([-0.158618771341394, 1.74637984736916, -0.00436517631477145, 0.00459322920933054, 0], dtype=np.double)
        self.object_3d_points = np.array(([-72, -32, 0],                 #xmin ymin
                                     [-58, 32, 0],                  #xmin ymax
                                     [58, -32, 0],                  #xmax ymax
                                     [72, 32, 0]), dtype=np.double) #xmax ymin
        #self.t=cv2.TrackerKCF_create()

    def track(self,image_raw):

        (success, box) = self.t.update(image_raw)
        print(box)
        if not rospy.is_shutdown():
            angle = calcu_angle(self,box)
            print(angle)
            if self._can_ctrl:
                if angle is not None:
                    self._set_fricwhl(True)
                    #angle=[0.1,0.1]
                    self._ctrlinfo_pub.publish(False,False,angle[0], angle[1])
                    self._shoot(1,1)
                    rospy.loginfo('pitch '+str(angle[0])+' yaw '+str(angle[1]))
                    '''
                elif undet_count != 0:
                    self._set_fricwhl(False)
                    self._shoot(0,0)
                    undet_count -= 1
                    self._ctrlinfo_pub.publish(angle[0],angle[1])
                    rospy.loginfo('pitch '+str(angle[0])+' yaw '+str(angle[1]))
                    '''
                else:
                    print('angle is none')

                    self._set_fricwhl(False)
                    self._shoot(0,0)
                    #TODO: define searching mode
                    #searching_mode()
                    rospy.loginfo('searching')
            else:
                rospy.loginfo('decision node needs to control the gimbal')

    def first(self,image_raw,box):
        print('init')
        self.t=cv2.TrackerKCF_create()
        box=[1,1,2,2]
        self.t.init(image_raw,box)


    def _set_fricwhl(self,can_start):
        #rospy.wait_for_service("cmd_fric_wheel")
        try:
            resp = self._fricwhl_client.call(can_start)
            rospy.loginfo("Message From fricwheelserver:%s"%resp.received)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")

    def _shoot(self,shoot_mode, shoot_number):
        #rospy.wait_for_service("cmd_fric_wheel")
        try:
            resp = self._shoot_client.call(shoot_mode, shoot_number)
            rospy.loginfo("Message From shootserver:%s"%resp.received)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")


# class KalmanFilter(object):
#     def __init__(self, dt, u_x,u_y, std_acc, x_std_meas, y_std_meas):
#         """
#         :param dt: sampling time (time for 1 cycle)
#         :param u_x: acceleration in x-direction
#         :param u_y: acceleration in y-direction
#         :param std_acc: process noise magnitude
#         :param x_std_meas: standard deviation of the measurement in x-direction
#         :param y_std_meas: standard deviation of the measurement in y-direction
#         """

#         # Define sampling time
#         self.dt = dt

#         # Define the  control input variables
#         self.u = np.matrix([[u_x],[u_y]])

#         # Intial State
#         self.x = np.array([[0, 0, 0, 0],
#                            [0, 0, 0, 0],
#                            [0, 0, 0, 0],
#                            [0, 0, 0, 0]
#         ])

#         # Define the State Transition Matrix A
#         self.A = np.matrix([[1, 0, self.dt, 0],
#                             [0, 1, 0, self.dt],
#                             [0, 0, 1, 0],
#                             [0, 0, 0, 1]])

#         # Define the Control Input Matrix B
#         self.B = np.matrix([[(self.dt**2)/2, 0],
#                             [0,(self.dt**2)/2],
#                             [self.dt,0],
#                             [0,self.dt]])

#         # Define Measurement Mapping Matrix
#         self.H = np.matrix([[1, 0, 0, 0],
#                             [0, 1, 0, 0]])

#         #Initial Process Noise Covariance
#         self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
#                             [0, (self.dt**4)/4, 0, (self.dt**3)/2],
#                             [(self.dt**3)/2, 0, self.dt**2, 0],
#                             [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2

#         #Initial Measurement Noise Covariance
#         self.R = np.matrix([[x_std_meas**2,0],
#                            [0, y_std_meas**2]])

#         #Initial Covariance Matrix
#         self.P = np.eye(self.A.shape[1])

#     def predict(self):

#         # Update time state
#         #x_k =Ax_(k-1) + Bu_(k-1)
#         self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

#         # Calculate error covariance
#         # P= A*P*A' + Q
#         self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
#         return self.x[0:2]

#     def update(self, z):

#         # S = H*P*H'+R
#         S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

#         # Calculate the Kalman Gain
#         # K = P * H'* inv(H*P*H'+R)
#         K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

#         self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))

#         I = np.eye(self.H.shape[1])

#         # Update error covariance matrix
#         self.P = (I - (K * self.H)) * self.P   #Eq.(13)
#         return self.x[0:2]


class KalmanFilter(object):
    def __init__(self, dt, u, std_acc, std_meas):
        """
        :param dt: sampling time (time for 1 cycle)
        :param u: control input related to the acceleration
        :param std_acc: standard deviation of the acceleration
        :param std_meas: standard deviation of the measurement
        """
        self.dt = dt
        self.u = u
        self.std_acc = std_acc

        self.A = np.matrix([[1, self.dt],
                            [0, 1]])
        self.B = np.matrix([[(self.dt**2)/2], [self.dt]])

        self.H = np.matrix([[1, 0]])

        self.Q = np.matrix([[(self.dt**4)/4, (self.dt**3)/2],
                            [(self.dt**3)/2, self.dt**2]]) * self.std_acc**2

        self.R = std_meas**2

        self.P = np.eye(self.A.shape[1])
        
        self.x = np.matrix([[0], [0]])

        # print(self.Q)


    def predict(self):

        # Update time state
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        # Calculate error covariance
        # P= A*P*A' + Q
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        # S = H*P*H'+R
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Calculate the Kalman Gain
        # K = P * H'* inv(H*P*H'+R)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S)) 

        self.x = np.round(self.x + np.dot(K, (z - np.dot(self.H, self.x))))  

        I = np.eye(self.H.shape[1])
        self.P = (I - (K * self.H)) * self.P
        return self.x  

if __name__ == "__main__":
    # load custom plugins
    #PLUGIN_LIBRARY = "./libmyplugins.so"
    PLUGIN_LIBRARY = "/home/dji/robort_ws/src/RoboRTS/robort_detection/src/libmyplugins.so"
    ctypes.CDLL(PLUGIN_LIBRARY)
    #engine_file_path = "./yolov5s.engine"
    #engine_file_path = "/home/dji/robort_ws/src/RoboRTS/robort_detection/yolov5s.engine"
    engine_file_path = "/home/dji/robort_ws/src/RoboRTS/robort_detection/src/yolov5s.engine"
    categories = ["blue","red","front","back","left","rights","tracking","death"]

    # a  YoLov5TRT instance
    yolov5_wrapper = Detector(engine_file_path)
    #kcf = Tracker()

    #initialize kalmanfilter instance
    # KF = KalmanFilter(0.05, 1, 1, 1)


    device_manager = gx.DeviceManager()
    dev_num, dev_info_list = device_manager.update_device_list()
    if dev_num == 0:
        print('no device')
        sys.exit(1)
    str_sn = dev_info_list[0].get("sn")
    cam = device_manager.open_device_by_sn(str_sn)
    #cam.BalanceWhiteAuto=EnumFeature()
    #cam.data_stream[0].
    cam.stream_on()
    '''
    #cam.data_stream[0].
    fourcc = cv2.VideoWriter_fourcc(*'XVID' )
    fps = cam.AcquisitionFrameRate.get()
    size = (cam.Width.get(),cam.Height.get())
    print(cam.Width.get(),cam.Height.get())
    filename = './out.avi'
    out = cv2.VideoWriter(filename, fourcc, 30, size)
    '''
    print(1)

    detection=True
    tracker=False
    n=0
    while(1):
        try:
            cam.data_stream[0].flush_queue()
            t1 = time.clock()
            #print("raw before")
            raw1_image = cam.data_stream[0].get_image()
            #print("raw after")
            rgb1_image = raw1_image.convert("RGB")
            #print("convert")
            numpy1_image = rgb1_image.get_numpy_array()
            #print("get")
        except Exception as exception:

            print("device error:%s" %exception)

            '''
            device_manager=gx.DeviceManager()
            dev_num,_=device_manager.update_device_list()
            '''
            try:
                cam.stream_off()
                #print("close stream")
                cam.close_device()
                #print("close device")


            except:
                while (1):

                    device_manager=gx.DeviceManager()
                    dev_num,dev_info=device_manager.update_device_list()
                    if dev_num==1:
                        #cam=device_manager.open_device_by_sn(str_sn)

                        break
                    else:
                        time.sleep(1)
                        '''
                            try:
                                #device_manager=gx.DeviceManager()
                                #dev_num,dev_info=device_manager.update_device_list()
                        #cam.stream_off()
                        #cam.close_device()
                        cam=device_manager.open_device_by_sn(str_sn)
                        print("open")
                        break
                    except Exception as exception:
                        print("sleep")
                        print("device error :",exception)
                        time.sleep(1)
                        '''
            while(1):
                try:
                    device_manager=gx.DeviceManager()
                    dev_num,_=device_manager.update_device_list()
                    cam=device_manager.open_device_by_sn(str_sn)
                    #print(" open 2")
                    break
                except:
                    try:

                        #cam.stream_off()
                        #print("close 1")
                        cam.close_device()
                        #print("close 2")
                    except:
                        #print("close error")
                        continue
                    time.sleep(1)
                    #print("reboot")
            cam.stream_on()
            #print("stream on")
            continue
        image_raw = cv2.cvtColor(numpy1_image, cv2.COLOR_RGB2BGR)
        if detection==True:
            bbox=yolov5_wrapper.infer(image_raw)
            #print("detection")

        else:
            kcf.track(image_raw)
            count-=1
            if count==0:
                tracker=False
                detection=True
                count=10



        t2 = time.clock()
        '''
        #print('Done. (%.3fs)' % (t2 - t1))
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            cam.stream_off()
            cam.close_device()
            break
        '''
        #print("over")
    cam.stream_off()
    # destroy the instance
    yolov5_wrapper.destroy()
    #cam.stream_off()
    #cam.close_device()
