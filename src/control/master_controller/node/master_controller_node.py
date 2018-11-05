#! /usr/bin/python

from std_srvs.srv import SetBool
from common_msgs_gl.srv import SendInt
from std_msgs.msg import Int32MultiArray

import rospy
import time
from random import *
import numpy as np
import IPython

perform_arduino_action = rospy.ServiceProxy("/hardware_coms/do_something", SendInt)

#fake_keyboard_pub_ = rospy.Publisher('/keyboard_input', UInt32, queue_size=1)

class AutoDataCollectorNode():

    def __init__(self):
        self.initialized = False

        #rospy.Subscriber("/gripper/load", Float32MultiArray, self.gripperLoadCallback)
        enable_srv_ = rospy.Service("/master_control/enable_collection", SetBool, self.enable_data_save)
        data_sub_ = rospy.Subscriber('/vision_processor/processed_data', Int32MultiArray, self.callbackProcessData, queue_size=1)

        self.hz = 30
        self.counter = 0;
        self.inflate_delay = 1;
        self.first_image = 4;
        self.second_image = 6;
        self.third_image = 8;
        self.fourth_image = 10;
        self.reset_ = 15


        r = rospy.Rate(self.hz) # 30hz
        while not rospy.is_shutdown():
            if self.initialized == True:
                if self.counter == self.inflate_delay*self.hz:
                    perform_arduino_action(0)
                elif self.counter == self.first_image*self.hz:
                    perform_arduino_action(1)
                    time.sleep(0.2)
                elif self.counter == self.second_image*self.hz:
                    perform_arduino_action(2)
                    time.sleep(0.2)
                elif self.counter == self.third_image*self.hz:
                    perform_arduino_action(3)
                    time.sleep(0.2)
                elif self.counter == self.fourth_image*self.hz:
                    perform_arduino_action(4)
                    time.sleep(0.2)

                self.counter = self.counter +1
                if self.counter > self.hz * self.reset_: #reset after 10 seconds
                    self.counter = 0
            r.sleep()

        rospy.spin()


    def enable_data_save(self,req):
        self.initialized = req.data
        return [self.initialized, "Successfully changed enable bool"]

    def callbackProcessData(self, data):
        self.currentImgData = data



    def fake_keyboard_call_(self):

        if self.time_taken >= self.time_to_live:
            self.time_to_live = randint(5,25) #between 1.5-2.5 seconds
            self.current_selection = randint(1,len(self.selection_choices)-1)
            self.time_taken =0
            new_value = True
        #elif self.time_taken >= self.time_to_live-2:
        #    self.current_selection = 0 #this will stop for 2 cycles

        fake_keyboard_pub_.publish(self.keyboardDict[self.selection_choices[self.current_selection]])
        self.time_taken = self.time_taken + 1


    def gripperLoadCallback (self, msg): #We will only keep track of when load history is exceeded
        history = msg.data
        if len(history)>0:
            #Do the two motor case first
            if len(self.load_history_)>=self.load_history_length_:
                 self.load_history_.pop(0) #Remove the first one

            if history[0]<(-800) and history[1]<(-800):
                self.load_history_.append(1)
            else:
                self.load_history_.append(0)

            #Now do the two motor case
            if len(self.single_motor_save_)>=self.load_history_length_+20:
                 self.single_motor_save_.pop(0) #Remove the first

            temp0 = history[0] <(-800)
            temp1 = history[1] <(-800)

            self.single_motor_save_.append([temp0,temp1])
            arrayed = np.asarray(self.single_motor_save_)


            #First check: Are they both being pulled really hard
            #Second check: Has either been pulled hard for the past while
            if np.mean(self.load_history_) >=0.9 or np.mean(arrayed[:,0])>=0.9 or np.mean(arrayed[:,1])>=0.9 :
                #print 'Load 1: ', np.mean(arrayed[:,0])>=0.9
                #print 'Load2: ', np.mean(arrayed[:,1])>=0.9
                #print 'Both: ', np.mean(self.load_history_) >=0.9
                self.reset_to_save_motors = True
            else:
                self.reset_to_save_motors = False


    def itemDroppedCallback (self, msg):
        self.is_dropped_ = msg.data

    def itemStuckCallback (self, msg):
        self.is_stuck_ = msg.data

    def itemSlidingCallback (self, msg):
        self.is_sliding_ = msg.data

    def objectMoveDist (self, msg):
        self.object_move_dist_= msg.data

    def resetObject(self):
        time_to_break = False
        self.enable_fake_keyboard_= False
        self.enable = False
        fake_keyboard_pub_.publish(self.keyboardDict["KEY_S_"])
        initialize_hand_srv_(OPEN_GRIPPER)
        time.sleep(8)

        while time_to_break == False:
            resp =  reset_obj_srv_(RAISE_CRANE_RESET)
            time.sleep(5)
            tic = time.time()
            while self.object_move_dist_ >15:
                toc = time.time()
                time.sleep(0.5)
                if (toc - tic > seconds_until_next_reset):
                    break
            if self.object_move_dist_ <=15:
                time_to_break=True

        #Now we can close the fingers and wait for the grasp
        initialize_hand_srv_(GET_GRIPPER_READY)
        time.sleep(8)
        reset_obj_srv_(LOWER_CRANE_OBJECT_READY)
        time.sleep(1.5)
        initialize_hand_srv_(ALLOW_GRIPPER_TO_MOVE)
        self.enable_fake_keyboard_ = True
        self.enable = True
        self.master_tic = time.time()




if __name__ == "__main__":

    rospy.init_node('MasterControlNode')

    try:
        AutoDataCollectorNode()
    except:
        rospy.logerr('Could not instantiate AutoCollectorNode')
