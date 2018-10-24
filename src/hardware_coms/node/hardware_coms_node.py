#!/usr/bin/python

import hardware_coms.communications_control as lib
from common_msgs_gl.srv import SendInt
import rospy
import IPython

class HardwareComsNode():

    def __init__(self, communication):
        self.communication = communication

        if rospy.has_param('/SystemResetNode/bridge_move_reset'):
            self.bridge_move_reset = rospy.get_param('/SystemResetNode/bridge_move_reset')
            self.crane_move_reset = rospy.get_param('/SystemResetNode/crane_move_reset')

        reset_system_ = rospy.Service("/hardware_coms/do_something", SendInt, self.callback_do_something)

        r = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            r.sleep()

        rospy.spin()


    def callback_do_something(self, req):
        #if req.data == RAISE_CRANE_RESET:
        #    self.communication.attach_object()
        #elif req.data == LOWER_CRANE_OBJECT_READY:
        #    self.communication.detach_object()
        #else:
        #    logerr("[ERR] Could not recognize reset command - 0 object grab, 1 object release")
        print 'attempted callback'
        a = req.data
        return []

if __name__ == "__main__":

    rospy.init_node('HardwareComsNode')

    #Read in params from reset_param.yaml
    if rospy.has_param('/HardwareComsNode/dev_name'):
        baudrate= rospy.get_param('/HardwareComsNode/baudrate',9600)
        dev_name = rospy.get_param('/HardwareComsNode/dev_name','/dev/ttyACM0')
    else:
        rospy.logerr('Ensure that you have properly filled in the .yaml file')

    #try:
    communication=lib.SerialComsSingle(dev_name, baudrate)
    #except:
    #    rospy.logerr('[ERR] - could not open serial port. Ensure mutex...')

    HardwareComsNode(communication)
