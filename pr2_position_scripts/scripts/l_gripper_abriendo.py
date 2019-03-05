#! /usr/bin/python

import rospy
from pr2_controllers_msgs.msg import *
 
pub = rospy.Publisher('/l_gripper_controller/command', Pr2GripperCommand, queue_size=1)
rospy.init_node('move_r_gripper', anonymous=True)

pub.publish(Pr2GripperCommand(0.08,-1.0))
rospy.sleep(0.5)
print "Abriendo"
pub.publish(Pr2GripperCommand(0.08,-1.0))
