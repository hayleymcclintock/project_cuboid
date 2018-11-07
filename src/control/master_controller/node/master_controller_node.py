#! /usr/bin/python

from std_srvs.srv import SetBool
from common_msgs_gl.srv import SendInt
from common_msgs_gl.msg import IntList

import rospy, rospkg
import time
from random import *
import numpy as np
import IPython
import os.path
import os

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

perform_arduino_action = rospy.ServiceProxy("/hardware_coms/do_something", SendInt)

#fake_keyboard_pub_ = rospy.Publisher('/keyboard_input', UInt32, queue_size=1)

class MasterCollectorNode():

    def __init__(self):
        self.initialized = True

        #rospy.Subscriber("/gripper/load", Float32MultiArray, self.gripperLoadCallback)
        enable_srv_ = rospy.Service("/master_control/enable_collection", SetBool, self.enable_data_save)
        data_sub_ = rospy.Subscriber('/vision_processor/processed_data', IntList, self.callbackProcessData, queue_size=1)
        self.image_sub = rospy.Subscriber('/vision_processor/parsed_img', Image, self.callbackImage, queue_size=1)

        self.bridge = CvBridge()

        self.hz = 30
        self.counter = 0;
        self.inflate_delay = 1; #This would be how many full hz cycles you want to wait
        self.first_image = 4;
        self.second_image = 6;
        self.third_image = 8;
        self.fourth_image = 10;
        self.reset_ = 15

        SIDE = 1

        #Create a csv file with a header, then write to it every time we get a new thing

        r = rospy.Rate(self.hz) # 30hz
        while not rospy.is_shutdown():
            if self.initialized == True:
                image_name = dirnamePics +str(int(time.time()))+".png"
                if self.counter == self.inflate_delay*self.hz: #Inflate
                    perform_arduino_action(SIDE)
                    print "Inflating..."
                elif self.counter == (self.inflate_delay+2)*self.hz: #Deflate
                    perform_arduino_action(SIDE+4)
                    print "Deflating..."

                #Light up and take picture for Edge 1
                elif self.counter == self.first_image*self.hz: #Light Up Edge 1
                    perform_arduino_action(SIDE*10 + 1)
                elif self.counter == (self.first_image+1)*self.hz: #Take Edge 1
                    cv2.imwrite(image_name,self.cv_image)
                    print "Took Picture 1..."
                    #TODO: Save the Data
                    perform_arduino_action(SIDE*10 + 1 + 4)

                #Light up and take picture for Edge 2
                elif self.counter == self.second_image*self.hz: #Light Up Edge 2
                    perform_arduino_action(SIDE*10 + 2)
                elif self.counter == (self.second_image+1)*self.hz: #Take Edge 2
                    cv2.imwrite(image_name,self.cv_image)
                    print "Took Picture 2..."
                    #TODO: Save the Data
                    perform_arduino_action(SIDE*10 + 2 + 4)

                #Light up and take picture for Edge 3
                elif self.counter == self.third_image*self.hz: #Light Up Edge 3
                    perform_arduino_action(SIDE*10 + 3)
                elif self.counter == (self.third_image+1)*self.hz: #Take Edge 3
                    cv2.imwrite(image_name,self.cv_image)
                    print "Took Picture 3..."
                    #TODO: Save the Data
                    perform_arduino_action(SIDE*10 + 3 + 4)

                #Light up and take picture for Edge 4
                elif self.counter == self.fourth_image*self.hz: #Light Up Edge 4
                    perform_arduino_action(SIDE*10 + 4)
                elif self.counter == (self.fourth_image+1)*self.hz: #Take Edge 4
                    cv2.imwrite(image_name,self.cv_image)
                    print "Took Picture 4..."
                    #TODO: Save the Data
                    perform_arduino_action(SIDE*10 + 4 + 4)

                #Reset the counter back to zero and the side once we finish one side
                if self.counter > self.hz * self.reset_: #reset after 10 seconds
                    self.counter = 0
                    SIDE = SIDE+1 #Say we are on the next side
                    SIDE = (SIDE-1)%4+1 #Scale this between 1 and 4

                #Continue (and always perform) counting
                self.counter = self.counter +1
            r.sleep()

        cv2.destroyAllWindows()
        rospy.spin()

    def callbackImage(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            rospy.logerr('[ros-video-recorder][VideoFrames] Converting Image Error. ' + str(e))
            return

    def enable_data_save(self,req):
        self.initialized = req.data
        return [self.initialized, "Successfully changed enable bool"]

    def callbackProcessData(self, data):
        self.currentImgData = data



if __name__ == "__main__":

    rospy.init_node('MasterControlNode')

    rospack = rospkg.RosPack()
    file_path = rospack.get_path("master_controller")
    cur_time = int(time.time())
    dirName = file_path + "/data/data_save_" + str(cur_time)+"/"
    if not os.path.exists(dirName):
        os.mkdir(dirName)
        rospy.loginfo("Directory " + dirName +  " Created ")

    dirnamePics = dirName+"pics/"
    if not os.path.exists(dirnamePics):
        os.mkdir(dirnamePics)
        rospy.loginfo("Directory " + dirnamePics +  " Created ")

    dirnameData = dirName+"data/"
    if not os.path.exists(dirnameData):
        os.mkdir(dirnameData)
        rospy.loginfo("Directory " + dirnameData +  " Created ")

    #Create the node to run in ROS
    MasterCollectorNode()
