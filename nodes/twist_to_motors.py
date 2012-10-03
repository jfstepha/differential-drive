#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
"""

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist    

#############################################################
def twistCallback(msg):
#############################################################
    global target
    global ticks_since_target
    global dx
    global dr
    ticks_since_target = 0
    dx = msg.linear.x
    dr = msg.angular.z
    
    dy = msg.linear.y
    
#############################################################
#############################################################
if __name__ == '__main__':
    """ main """
    global ticks_since_target
    rospy.init_node("twist_to_motors")
    nodename = rospy.get_name()
    rospy.loginfo("%s started" % nodename)
    
    w = rospy.get_param("~base_width")
    
    pub_lmotor = rospy.Publisher('lwheel_vtarget', Float32)
    pub_rmotor = rospy.Publisher('rwheel_vtarget', Float32)
    rospy.Subscriber('twist', Twist, twistCallback)
    
    
    rate = rospy.get_param("~rate", 50)
    timeout_ticks = rospy.get_param("~timeout_ticks", 2)
    
    r = rospy.Rate(rate)
    then = rospy.Time.now()
    ticks_since_target = timeout_ticks
    
    ###### main loop  ######
    while not rospy.is_shutdown():
        
        while not rospy.is_shutdown() and ticks_since_target < timeout_ticks:
            # dx = (l + r) / 2
            # dr = (r - l) / w
            
            right = 1.0 * dx + dr * w / 2 
            left = 1.0 * dx - dr * w / 2
            # rospy.loginfo("publishing: (%d, %d)", left, right) 
            pub_lmotor.publish(left)
            pub_rmotor.publish(right)
            
            ticks_since_target += 1
            r.sleep()
        r.sleep
        
    
    