#!/usr/bin/env python

"""
   wheel_scaler
   scales the wheel readings (and inverts the sign)
   
"""

import rospy
import roslib

from std_msgs.msg import Int16

def lwheelCallback(msg):
    lscaled_pub.publish( msg.data * -1 * scale)
    
def rwheelCallback(msg):
    rscaled_pub.publish( msg.data * -1 * scale)
    

if __name__ == '__main__':
    """main"""
    rospy.init_node("wheel_scaler")
    rospy.loginfo("wheel_scaler started")
    
    scale = rospy.get_param('distance_scale', 1)
    rospy.loginfo("wheel_scaler scale: %0.2f", scale)
    
    rospy.Subscriber("lwheel", Int16, lwheelCallback)
    rospy.Subscriber("rwheel", Int16, rwheelCallback)
    
    lscaled_pub = rospy.Publisher("lwheel_scaled", Int16)
    rscaled_pub = rospy.Publisher("rwheel_scaled", Int16) 
     
    rospy.spin()