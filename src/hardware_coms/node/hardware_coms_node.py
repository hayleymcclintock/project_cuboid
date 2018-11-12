#!/usr/bin/python

import hardware_coms.communications_control as lib
from common_msgs_gl.srv import SendInt
import rospy
import IPython
import struct

class HardwareComsNode():

    def __init__(self, communication):
        self.communication = communication

        do_serial_comms = rospy.Service("/hardware_coms/do_something", SendInt, self.callback_do_something)

        r = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            r.sleep()

        rospy.spin()


    def callback_do_something(self, req):
        data_in = req.data
        if isinstance(data_in, (int, long)):
            self.communication.send_serial_msgs(struct.pack("B", data_in))
        return []

if __name__ == "__main__":

    rospy.init_node('HardwareComsNode')

    #Read in params from reset_param.yaml
    if rospy.has_param('/HardwareComsNode/dev_name'):
        baudrate= rospy.get_param('/HardwareComsNode/baudrate',9600)
        dev_name = rospy.get_param('/HardwareComsNode/dev_name','/dev/ttyACM0')
    else:
        rospy.logerr('Ensure that you have properly filled in the .yaml file')


    communication=lib.SerialComsSingle(dev_name, baudrate)

    HardwareComsNode(communication)
