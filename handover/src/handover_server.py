#!/usr/bin/env python3

import rospy
import time
import sys
import argparse
# sys.path.append()
sys.path.append('/home/kl/Pick-and-Place-with-RL/catkin_ws/src/ros_handover/handover/src/HANet')
from HANet.utils import Affordance_predict
import message_filters
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import rospy
import warnings
from smach_tutorials.msg import TestAction
from actionlib import *
from actionlib_msgs.msg import *
from vx300s_bringup.srv import *
warnings.filterwarnings("ignore")

# Create a trivial action server
class HandoverServer:
    def __init__(self, name, mode, arm):
        self._sas = SimpleActionServer(name, TestAction, execute_cb=self.execute_cb, auto_start=False)
        self._sas.start()
        self.r = TriggerRequest()
        info = rospy.wait_for_message('camera_right/color/camera_info', CameraInfo)
        self.mode = mode
        self.arm = arm
        self.color = None
        self.depth = None
        self.target = None
        self.model_on = False
        self.f_x = 0
        self.f_y = 0
        self.f_z = 0
        self.count = 0
        self.dis = None
        self.pred = Affordance_predict(self.arm, info.P[0], info.P[5], info.P[2], info.P[6], self.mode)
        
        # Subscriber:
        self.force = rospy.Subscriber("/robotiq_ft_wrench", WrenchStamped, self.callback_force_msgs)

        self.color_sub_r = message_filters.Subscriber('/camera_right/color/image_raw/compressed', CompressedImage)
        self.depth_sub_r = message_filters.Subscriber('/camera_right/aligned_depth_to_color/image_raw', Image)
        self.color_sub_l = message_filters.Subscriber('/camera_left/color/image_raw/compressed', CompressedImage)
        self.depth_sub_l = message_filters.Subscriber('/camera_left/aligned_depth_to_color/image_raw', Image)

        ts = message_filters.ApproximateTimeSynchronizer([self.color_sub_r, self.depth_sub_r, self.color_sub_l, self.depth_sub_l], 5, 5)
        ts.registerCallback(self.callback_img_msgs)

        # Publisher prediction image
        self.aff_pub_right = rospy.Publisher("~affordance_map_right", Image, queue_size=10)
        self.aff_pub_left = rospy.Publisher("~affordance_map_left", Image, queue_size=10)

        # Service
        rospy.Service('~switch_loop', Trigger, self.switch_loop)
        rospy.Service('~switch_model', Trigger, self.switch_model)
        rospy.Service('~switch_hand', Trigger, self.switch_hand)
        rospy.loginfo("Server Initial Complete")

        # Start prediction
        self.realtime_pred()

        """
        goal = 0 : Init
             = 1 : Detect and Go target
             = 2 : Grasp_back
             = 3 : Check_dis
             = 4 : Wait_object
             = 5 : To_parallel
             = 6 : Open_cap
        """

    def realtime_pred(self):
        while True:
            if self.model_on:
                if self.color != None and self.depth!= None:
                    self.target, _, self.dis, aff_map, _ = self.pred.predict(self.color, self.depth)
                    if self.target != None:
                        if self.arm == 'right_arm':
                            self.aff_pub_right.publish(aff_map)
                        else:
                            self.aff_pub_left.publish(aff_map)
            if self.model_on == 'stop':
                break

    def switch_loop(self, req):
        res = TriggerResponse()
        self.pred.switch()
        res.success = True

        return res

    def switch_hand(self, req):
        res = TriggerResponse()
        
        self.pred.switch_hand()

        if self.arm == 'right_arm':
            self.arm = 'left_arm'
        else:
            self.arm = 'right_arm'

        res.success = True

        return res

    def switch_model(self, req):
        res = TriggerResponse()
        if self.model_on:
            self.model_on = False
            rospy.loginfo("Model Off")
        else:
            self.model_on = True
            rospy.loginfo("Model On")
    
        res.success = True

        return res
    
    def callback_img_msgs(self, color_msg_r, depth_msg_r, color_msg_l, depth_msg_l):
        if self.arm == 'right_arm':
            self.color = color_msg_r
            self.depth = depth_msg_r
        else:
            self.color = color_msg_l
            self.depth = depth_msg_l

    def callback_force_msgs(self, msg):
        self.f_x = int(msg.wrench.force.x)
        self.f_y = int(msg.wrench.force.y)
        self.f_z = int(msg.wrench.force.z)

    def execute_cb(self, msg):
        # Init
        if msg.goal == 0:
            # Go initial pose
            self.count = 0
            try:
                go_pose = rospy.ServiceProxy("/{0}/go_handover".format(self.arm), Trigger)
                resp = go_pose(self.r)
            except rospy.ServiceException as exc:
                print("service did not process request: " + str(exc))
                self._sas.set_aborted()

            # open gripper
            try:
                go_pose = rospy.ServiceProxy("/{0}/gripper_open".format(self.arm), Trigger)
                resp = go_pose(self.r)
            except rospy.ServiceException as exc:
                print("service did not process request: " + str(exc))
                self._sas.set_aborted()
            
            self._sas.set_succeeded()
            time.sleep(1)
        # Detect and Go target
        elif msg.goal == 1:
            if self.target == None:
                self._sas.set_aborted()
            else:
                try:
                    go_pose = rospy.ServiceProxy("/{0}/go_pose".format(self.arm), ee_pose)
                    resp = go_pose(self.target)
                except rospy.ServiceException as exc:
                    print("service did not process request: " + str(exc))

            time.sleep(0.5)
        # Grasp and back
        elif msg.goal == 2:
            try:
                go_pose = rospy.ServiceProxy("/{0}/gripper_close".format(self.arm), Trigger)
                resp = go_pose(self.r)
            except rospy.ServiceException as exc:
                print("service did not process request: " + str(exc))
                self._sas.set_aborted()

            rospy.sleep(0.5)

            try:
                go_pose = rospy.ServiceProxy("/{0}/go_place".format(self.arm), Trigger)
                resp = go_pose(self.r)
            except rospy.ServiceException as exc:
                print("service did not process request: " + str(exc))
                self._sas.set_aborted()

            rospy.sleep(0.5)

            try:
                go_pose = rospy.ServiceProxy("/{0}/gripper_open".format(self.arm), Trigger)
                resp = go_pose(self.r)
            except rospy.ServiceException as exc:
                print("service did not process request: " + str(exc))
                self._sas.set_aborted()

            rospy.sleep(0.5)

            # Back
            try:
                go_pose = rospy.ServiceProxy("/{0}/go_sleep".format(self.arm), Trigger)
                resp = go_pose(self.r)
            except rospy.ServiceException as exc:
                print("service did not process request: " + str(exc))
                self._sas.set_aborted()
            
            self._sas.set_succeeded()
        # Check distance
        elif msg.goal == 3:
            rospy.loginfo(str(self.dis))
            if self.dis <= 0.02:
                self._sas.set_succeeded()
            elif self.count == 2:
                self.pred.switch()
                self._sas.set_succeeded()
            else:
                self._sas.set_aborted()
            rospy.sleep(0.5)
        # Wait object
        elif msg.goal == 4:
            rospy.sleep(1)
            x = self.f_x
            while True:
                if x != self.f_x:
                    break

            self._sas.set_succeeded()
        # To parallel pose
        elif msg.goal == 5:
            try:
                go_pose = rospy.ServiceProxy("/{0}/gripper_open".format(self.arm), Trigger)
                resp = go_pose(self.r)
            except rospy.ServiceException as exc:
                print("service did not process request: " + str(exc))
                self._sas.set_aborted()
        

            try:
                go_pose = rospy.ServiceProxy("/{0}/turnto0".format(self.arm), Trigger)
                resp = go_pose(self.r)
            except rospy.ServiceException as exc:
                print("service did not process request: " + str(exc))
                self._sas.set_aborted()

            self._sas.set_succeeded()
        # Open cap
        elif msg.goal == 6:
            try:
                go_pose = rospy.ServiceProxy("/{0}/gripper_close".format(self.arm), Trigger)
                resp = go_pose(self.r)
            except rospy.ServiceException as exc:
                print("service did not process request: " + str(exc))
                self._sas.set_aborted()

            try:
                go_pose = rospy.ServiceProxy("/{0}/turnto90".format(self.arm), Trigger)

                resp = go_pose(self.r)
            except rospy.ServiceException as exc:
                print("service did not process request: " + str(exc))
                self._sas.set_aborted()

            self._sas.set_succeeded()
            time.sleep(0.5)

    def onShutdown(self):
        self.model_on = 'stop'
        try:
            go_pose = rospy.ServiceProxy("/{0}/go_sleep".format('right_arm'), Trigger)
            resp = go_pose(self.r)
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))

        try:
            go_pose = rospy.ServiceProxy("/{0}/go_sleep".format('left_arm'), Trigger)
            resp = go_pose(self.r)
        except rospy.ServiceException as exc:
            print("service did not process request: " + str(exc))

        rospy.sleep(0.5)
        rospy.loginfo("Shutdown.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Set up')
    parser.add_argument('--mode', type=str, default = 'handover')
    parser.add_argument('--arm', type=str, default = 'right_arm')
    args = parser.parse_args()

    rospy.init_node('handover_server')
    server = HandoverServer(name='handover_action', mode=args.mode, arm=args.arm)
    rospy.on_shutdown(server.onShutdown)
    rospy.spin()