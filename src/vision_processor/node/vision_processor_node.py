#! /usr/bin/python

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from common_msgs_gl.msg import IntList
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

result_pub_ = rospy.Publisher('/vision_processor/parsed_img',Image, queue_size=1)
data_pub_ = rospy.Publisher('/vision_processor/processed_data',IntList, queue_size=1)



class VisionProcessorNode():

    def __init__ (self):

        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.callbackImage, queue_size=1)

        self.bridge = CvBridge()

        r = rospy.Rate(30) # 30hz
        self.Imagein = False
        self.total_count =0

        while not rospy.is_shutdown():
            if self.Imagein == True:
                self.image = self.cv_image
                self.img_result_, self.data_ , self.row_, self.col_= self.process_image(self.image)
                result_pub_.publish(self.bridge.cv2_to_imgmsg(self.img_result_, 'bgr8'))

                msg = IntList()
                msg.data =self.data_
                msg.row = self.row_
                msg.col = self.col_
                data_pub_.publish(msg)
                self.Imagein = False
            r.sleep()

        cv2.destroyAllWindows()
        rospy.spin()

    def callbackImage(self,data):
        self.Imagein = True
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            rospy.logerr('[ros-video-recorder][VideoFrames] Converting Image Error. ' + str(e))
            return

    def process_image(self,frame):
        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v = hsv.copy()
        v[:,:,0] = 0
        v[:,:,1] = 0

        # Calculate wanted variables
        result, data, row, col = divide_image(v, 50,50,2, self.total_count) #channel 0 for hue, 1 for sat, and 2 for value
        self.total_count = self.total_count+1
        return result, data, row, col

##########################################
#Helper functions that do not need to be a apart of a class class
##########################################
def divide_image(img, nrow, ncol, channel,total_count):
    row_pix = img.shape[0]
    col_pix = img.shape[1]
    row_size = int(row_pix/nrow)
    col_size = int(col_pix/ncol)

    full_data = [] #This includes the x,y pixel locations
    data = []
    row_list = []
    col_list = []
    new_img = img.copy()
    for row in range(nrow):
        for col in range(ncol):
             patch = img[row*row_size:(row+1)*row_size,col*col_size:(col+1)*col_size,channel]
             mean_patch = np.mean(patch)
             center_of_patch = [int((row*row_size+(row+1)*row_size)/2), int((col*col_size+(col+1)*col_size)/2)]
             cv2.putText(new_img,str(int(mean_patch)),(center_of_patch[1],center_of_patch[0]), cv2.FONT_HERSHEY_SIMPLEX, 0.3,(255,255,255),1,cv2.LINE_AA)
             full_data.append([center_of_patch[0], center_of_patch[1],int(mean_patch)])
             data.append(int(mean_patch))
             row_list.append(int(center_of_patch[0]))
             col_list.append(int(center_of_patch[1]))

    #if total_count <4:
    #    nupied = np.asarray(data)
    #    name = 'process' + str(total_count) + '.csv'
    #    np.savetxt(str(name),nupied, delimiter =',')
    #    print 'Just saved: ', name
    #    time.sleep(10)
    #total_count = total_count+1

    return new_img, data, row_list, col_list


##########################################
#Main to start program
##########################################
if __name__ == "__main__":

    rospy.init_node('VisionProcessorNode')



    VisionProcessorNode()
    #except:
    #    rospy.logerr('Could not initialize VisionProcessorNode!')
