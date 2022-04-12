#!/usr/bin/env python3
from glob import glob
import rospkg
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest, SetBool, SetBoolResponse
import rospy
import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
import warnings
from darknet_ros_msgs.msg import BoundingBoxes
warnings.filterwarnings("ignore")

rospy.init_node('img_pub')

pred_done = False

def callback_darknet(msg):
    box_list = msg.bounding_boxes
    boxes = []
    names = []
    global depth_nan
    tt = np.zeros((224,224))
    tt = tt+255
    for i in range(len(box_list)):
        if(box_list[i].Class != 'person') and (box_list[i].Class != 'chair'):
            depth_mean = np.nanmean(depth_nan[box_list[i].ymin : box_list[i].ymax, box_list[i].xmin : box_list[i].xmax])
            print(box_list[i].Class,' / ',depth_mean)
            # tt[box_list[i].ymin : box_list[i].ymax, box_list[i].xmin : box_list[i].xmax] = 0
            # print(box_list[i].Class)
            # plt.imshow(tt)
            # plt.show()
            if np.isnan(depth_mean):
                # print('88 ',box_list[i].Class)
                continue
            else:
                names.append(box_list[i].Class)
                boxes.append([box_list[i].xmin, box_list[i].ymin, box_list[i].xmax, box_list[i].ymax, depth_mean])

    # print(boxes)
    global pred_done
    global id
    f = open('/home/kl/Pick-and-Place-with-RL/catkin_ws/src/integration/yolo_result.txt', "a")
    f.write('=========='+id+'\n')
    if(len(boxes) != 0):
        boxes = np.asarray(boxes)
        # for i in range(len(boxes)):
        #     print('ID : ',str(i),' ',names[i])
        # IDX = int(input('enter id : '+'\n'))
        # best_box = boxes[IDX,:]
        best_box = boxes[np.argmin(boxes[:,4]),:]
        print(best_box)
        for i in range(5):
            f.write(str(best_box[i])+'\n')
    else:
        for i in range(5):
            f.write(str(-10)+'\n')
        print('Detect failed')
    f.close()
    pred_done = True

def _onShutdown(self):
        rospy.sleep(0.5)
        rospy.loginfo("Shutdown.")


img_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
depth_pub = rospy.Publisher('/camera/aligned_depth_to_color/image_raw', Image, queue_size=10)
# rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback_darknet, queue_size=1)
bridge = CvBridge()



f = open('/home/kl/Pick-and-Place-with-RL/catkin_ws/src/integration/list.txt', "r")
for i, line in enumerate(f):
    pred_done = False
    id = line.replace("\n", "")
    rgb_source = cv2.imread('/home/kl/Pick-and-Place-with-RL/catkin_ws/src/integration/color/color_'+id+'.jpg')[:,:,[2,1,0]]
    depth_source = np.load('/home/kl/Pick-and-Place-with-RL/catkin_ws/src/integration/depth/depth_'+id+'.npy')
    # depth = depth/1000.0
    # print(np.where(depth==np.nan))
    # mask_hand = cv2.imread('/home/kl/Pick-and-Place-with-RL/catkin_ws/src/integration/mask_hand/mask_hand_'+id+'.png', cv2.IMREAD_GRAYSCALE)
    # mask_body = cv2.imread('/home/kl/Pick-and-Place-with-RL/catkin_ws/src/integration/mask_body/mask_body_'+id+'.png', cv2.IMREAD_GRAYSCALE)
    # mask_hand[mask_hand==30] = 0
    # mask_body[mask_body==30] = 0
    # depth_nan = depth.copy().astype(np.float32)
    # depth_nan[depth == 0] = np.nan
    # depth_nan[mask_body != 0] = np.nan
    # depth_nan[mask_hand != 0] = np.nan




    rgb = bridge.cv2_to_imgmsg(rgb_source, "bgr8")
    depth = bridge.cv2_to_imgmsg(depth_source, "16UC1")
    print('==========================================================')
    print('pub color_'+id)

    while pred_done == False:
        img_pub.publish(rgb)
        depth_pub.publish(depth)
        # print('wait..',end='\r')

print('done')
rospy.on_shutdown(_onShutdown)
rospy.spin()