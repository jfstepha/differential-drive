#!/usr/bin/env python

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
"""

import rospy
import roslib
roslib.load_manifest('knex_ros')
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int16

#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        
        # parameters
        self.rate = rospy.get_param('rate',10.0)
        self.timeout = rospy.get_param('timeout',1.0)
        self.ticks_meter = float(rospy.get_param('ticks_meter'))
        self.base_width = float(rospy.get_param('base_width'))
        
        self.base_frame_id = rospy.get_param('base_frame_id','base_link')
        self.odom_frame_id = rospy.get_param('odom_frame_id', 'odom')
        
        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = rospy.Time.now()
        
        
        rospy.loginfo("%s: got rate %0.1f " %(self.nodename, self.rate))
        
        # subscriptions
        rospy.Subscriber("lwheel_scaled", Int16, self.lwheelCallback)
        rospy.Subscriber("rwheel_scaled", Int16, self.rwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry)
        self.odomBroadcaster = TransformBroadcaster()
        while not rospy.is_shutdown():
            self.update()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            
            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right
            
            d = ( d_left + d_right ) / 2
            th = ( d_right - d_left ) / self.base_width
            self.dx = d / elapsed
            self.dr = th / elapsed
            
            if (d != 0):
                x = cos( th ) * d
                y = -sin( th ) * d
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if( th != 0):
                self.th = self.th + th
                
            # publish
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
                )
            
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)
            
            


    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        self.left = msg.data
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        self.right = msg.data

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    diffTf = DiffTf();
    
    
   