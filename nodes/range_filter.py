#!/usr/bin/env python

""" 
   filters the range finder data - it seems to be rather noisy
"""
   
import rospy
import roslib
roslib.load_manifest('knex_ros')

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from numpy import array

############################################################################## 
############################################################################## 
class RangeFilter():
############################################################################## 
############################################################################## 

    #########################################################################
    def __init__(self):
    #########################################################################
        cur_val = 0

        self.rolling_ave = 0.0
        self.rolling_std = 0.0
        self.rolling_meters = 0.0
        rospy.init_node("range_filter")
        rospy.loginfo("-I- range_filter started")
        self.rolling_pts = rospy.get_param('~rolling_pts',4)
        self.m = rospy.get_param('~exponent', -1.31)
        self.b = rospy.get_param('~coefficient', 266.0)
        self.prev = [0] * self.rolling_pts
    
        rospy.Subscriber("range", Int16, self.inputCallback)
    
        self.filtered_pub = rospy.Publisher("range_filtered", Float32)
        self.std_pub = rospy.Publisher("range_std", Float32)
        while not rospy.is_shutdown():
            rospy.spin()
    

    #########################################################################
    def inputCallback(self, msg):
    #########################################################################
        rospy.loginfo("-D- range_filter inputCallback")
        cur_val = msg.data
    
        if cur_val < 900:
            self.prev.append(cur_val)
            del self.prev[0]
        
            p = array(self.prev)
            self.rolling_ave = p.mean()
            self.rolling_std = p.std()
        
            rolling_meters = self.b * self.rolling_ave ** self.m
        
            self.filtered_pub.publish( rolling_meters )
            self.std_pub.publish( self.rolling_std )
        
    
############################################################################## 
############################################################################## 
if __name__ == '__main__':
############################################################################## 
############################################################################## 
    """ main"""
    rangeFilter = RangeFilter()
    