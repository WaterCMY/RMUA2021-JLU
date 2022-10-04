#!/usr/bin/env python3
import ctypes
import os
import rospy
import random
import sys
import threading
import time
import cv2
import numpy as np
import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt
import torch
import torchvision
import gxipy as gx

from roborts_msgs.msg import GimbalAngle
from roborts_msgs.srv import FricWhl, ShootCmd
INPUT_W = 608
INPUT_H = 608
CONF_THRESH = 0.6
IOU_THRESHOLD = 0.4

#1-blue 2-red
enemy_color=1

def select_target(box_list, cls_list, score_list, ENEMY_COLOR):
    armor_box=[0,0,0,0]
    direction=7
    for box, cls, score in zip(box_list, cls_list, score_list):
        tmp_armor_score = 0
        if cls == ENEMY_COLOR and score > tmp_armor_score:
            mp_armor_score = score
            armor_box = box
    for box, cls, score in zip(box_list, cls_list, score_list):
        tmp_direc_score = 0
        if cls >=3 and score > tmp_direc_score:
            if box[0] < armor_box[0] and box[2] > armor_box[2]:
                direction = [box, cls]
    return armor_box, direction

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
        print('box: (%d,%d), (%d,%d) ' %(c1[0],c1[1],c2[0],c2[1]))

def calcu_angle(self, bbox):
    if bbox[2] == 0:
        return None
    else:
        # [ymin xmin ymax xmax]
        box = [bbox[1], bbox[0], bbox[1]+bbox[3], bbox[0]+bbox[2]] 
        object_2d_point = np.array(([box[1],box[0]],[box[1],box[2]],
                                        [box[3],box[2]],[box[3],box[0]]),
                                        dtype=np.double)
        _, _, tvec = cv2.solvePnP(self.object_3d_points, object_2d_point, 
                                      self.camera_matrix, self.dist_coefs, 
                                      cv2.SOLVEPNP_EPNP)
        pitch = float(np.arctan2(tvec[1][0], tvec[2][0])) 
        yaw   = float(np.arctan2(tvec[0][0], tvec[2][0]))
    return [pitch, yaw]


class Detector(object):
    """
    description: A YOLOv5 class that warps TensorRT ops, preprocess and postprocess ops.
    """

    def __init__(self, engine_file_path):
        # Create a Context on this device
        rospy.init_node('armor_detection_node')

        self.cfx = cuda.Device(0).make_context()

        self._ctrlinfo_pub = rospy.Publisher('/cmd_gimbal_angle', 
                                             GimbalAngle, queue_size=1, 
                                             tcp_nodelay=True)
        self._fricwhl_client = rospy.ServiceProxy("/cmd_fric_wheel",FricWhl)
        self._shoot_client = rospy.ServiceProxy("/cmd_shoot",ShootCmd)
        self._can_ctrl = True
        undet_count = 40
        #num_bullets = 100
        self.camera_matrix = np.array(([1750, 0, 356.3],
                                       [0, 1756, 375.9],
                                       [0, 0, 1.0]), dtype=np.double)
        self.dist_coefs = np.array([0, 0, 0, 0, 0], dtype=np.double)
        object_3d_points = np.array(([-72, -32, 0],                 #xmin ymin
                                     [-58, 32, 0],                  #xmin ymax
                                     [58, -32, 0],                  #xmax ymax 
                                     [72, 32, 0]), dtype=np.double) #xmax ymin


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

    def infer(self, image_raw):
        threading.Thread.__init__(self)
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
        for i in range(len(result_boxes)):
            box = result_boxes[i]
            plot_one_box(
                box,
                image_raw,
                label="{}:{:.2f}".format(
                    categories[int(result_classid[i])], result_scores[i]
                ),
            )
        
        #cv2.imshow("video",image_raw)
        box, direc = select_target(result_boxes, result_scores, result_classid, enemy_color)
        print(box)
        print(direc)
        boundingbox=[0,0,0,0]
        boundingbox[:] = [box[1], box[0], box[3]-box[1], box[2]-box[0]] 


        if not rospy.is_shutdown():
            angle = calcu_angle(self,bbox=boundingbox)
            if self._can_ctrl:
                if angle is not None:
                    self._set_fricwhl(True)
                    self._ctrlinfo_pub.publish(angle[0], angle[1])
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
                    print('start')
                    self._set_fricwhl(False)
                    print('wheel')
                    self._shoot(0,0)
                    print('shoot')
                    #TODO: define searching mode
                    #searching_mode()
                    
                    rospy.loginfo('searching')
                    print('ok')
            else:
                rospy.loginfo('decision node needs to control the gimbal')

        return box

        #return result_boxes, result_classid, result_scores






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
        #image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2RGB)
        image=image_raw
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

        rospy.init_node('armor_detection_node')

        self._ctrlinfo_pub = rospy.Publisher('/cmd_gimbal_angle', 
                                             GimbalAngle, queue_size=1, 
                                             tcp_nodelay=True)
        self._fricwhl_client = rospy.ServiceProxy("/cmd_fric_wheel",FricWhl)
        self._shoot_client = rospy.ServiceProxy("/cmd_shoot",ShootCmd)
        self._can_ctrl = True
        undet_count = 40
        #num_bullets = 100
        self.camera_matrix = np.array(([1750, 0, 356.3],
                                       [0, 1756, 375.9],
                                       [0, 0, 1.0]), dtype=np.double)
        self.dist_coefs = np.array([0, 0, 0, 0, 0], dtype=np.double)
        self.object_3d_points = np.array(([-72, -32, 0],                 #xmin ymin
                                     [-58, 32, 0],                  #xmin ymax
                                     [58, -32, 0],                  #xmax ymax 
                                     [72, 32, 0]), dtype=np.double) #xmax ymin
        #self.t=cv2.TrackerKCF_create()

    def track(self,image_raw):

        (success, box) = self.t.update(image_raw)
        if not rospy.is_shutdown():
            angle = calcu_angle(self,box)
            if self._can_ctrl:
                if angle is not None:
                    self._set_fricwhl(True)
                    angle=[0.1,0.1]
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
                    self._set_fricwhl(False)
                    self._shoot(0,0)
                    #TODO: define searching mode
                    #searching_mode()
                    #rospy.loginfo('searching')
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


if __name__ == "__main__":
    # load custom plugins
    PLUGIN_LIBRARY = "/home/dji/robort_ws/src/RoboRTS/roborts_detection/detector.py"
    ctypes.CDLL(PLUGIN_LIBRARY)
    engine_file_path = "/home/dji/robort_ws/src/RoboRTS/roborts_detection/yolov5s.engine"

    # load coco labels

    categories = ["blue","red","front","back","left","rights","tracking"]

    # a  YoLov5TRT instance
    yolov5_wrapper = Detector(engine_file_path)
    kcf = Tracker()

    print('2')
    
    device_manager = gx.DeviceManager() 
    dev_num, dev_info_list = device_manager.update_device_list()
    if dev_num == 0:
        print('no device')
        sys.exit(1)
    str_sn = dev_info_list[0].get("sn")
    cam = device_manager.open_device_by_sn(str_sn)
    cam.stream_on()
    
    detection=True
    tracker=False
    
    count=10

    while(1):

        
        cam.data_stream[0].flush_queue()
        t1 = time.clock()
        raw1_image = cam.data_stream[0].get_image(timeout=10000000)
        rgb1_image = raw1_image.convert("RGB")
        numpy1_image = rgb1_image.get_numpy_array()
        image_raw = cv2.cvtColor(numpy1_image, cv2.COLOR_RGB2BGR)
        

        #image_raw=np.zeros((640,480,3),dtype=float)
        if detection==True:
            bbox=yolov5_wrapper.infer(image_raw)
            kcf.first(image_raw,bbox)
            detection=False
            Tracker=True
            
        else:
            kcf.track(image_raw)
            count-=1
            if count==0:
                tracker=False
                detection=True
                count=10
        


        t2 = time.clock()
        print('Done. (%.3fs)' % (t2 - t1))
        if cv2.waitKey(1) == ord('q'): 
            cv2.destroyAllWindows()
            cam.stream_off()
            cam.close_device()
            break
    cam.stream_off()
    # destroy the instance
    yolov5_wrapper.destroy()
    #cam.stream_off()
    #cam.close_device()
