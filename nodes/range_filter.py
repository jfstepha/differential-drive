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

weight = 0.1  # 1 = this value only 0 = prev value only
cur_val = 0

rolling_ave = 0.0
rolling_std = 0.0
rolling_meters = 0.0

def inputCallback(msg):
    rospy.loginfo("-D- range_filter inputCallback")
    b = 266
    m = -1.31
    cur_val = msg.data
    
    global rolling_ave
    global rolling_std
    global prev
    global std_pub
    
    if cur_val < 900:
        prev.append(cur_val)
        del prev[0]
        
        p = array(prev)
        rolling_ave = p.mean()
        rolling_std = p.std()
        
        p = array(prev)
        rolling_ave = p.mean()
        rolling_std = p.std()
        
        rolling_meters = b * rolling_ave ** m
        
        filtered_pub.publish( rolling_meters )
        std_pub.publish( rolling_std )
        
    
if __name__ == '__main__':
    """ main"""
    rospy.init_node("range_filter")
    rospy.loginfo("-I- range_filter started")
    rolling_pts = rospy.get_param('~rolling_pts',4)
    global prev
    prev = [0] * rolling_pts
    
    rospy.Subscriber("range", Int16, inputCallback)
    
    filtered_pub = rospy.Publisher("range_filtered", Float32)
    std_pub = rospy.Publisher("range_std", Float32)
    
    rospy.spin()