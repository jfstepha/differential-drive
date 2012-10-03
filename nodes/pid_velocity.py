#!/usr/bin/env python
"""
   pid_velocity - takes messages on wheel_vtarget 
      target velocities for the wheels and monitors wheel for feedback
"""

import rospy
import roslib

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from numpy import array
wheel_latest=0

######################################################
def wheelCallback(msg):
######################################################
    global wheel_latest
    wheel_latest = 1.0 * msg.data / ticks_per_meter
    
######################################################
def targetCallback(msg):
######################################################
    global target
    global ticks_since_target
    target = msg.data
    ticks_since_target = 0
    
    
if __name__ == '__main__':
    """ main """
    rospy.init_node("pid_velocity")
    nodename = rospy.get_name()
    rospy.loginfo("%s started" % nodename)
    
    ### get parameters #### 
    Kp = rospy.get_param('~Kp',10)
    Ki = rospy.get_param('~Ki',10)
    Kd = rospy.get_param('~Kd',10)
    out_min = rospy.get_param('~out_min',-255)
    out_max = rospy.get_param('~out_max',255)
    rate = rospy.get_param('~rate',20)
    rolling_pts = rospy.get_param('~rolling_pts',20)
    timeout_ticks = rospy.get_param('~timeout_ticks',2)
    ticks_per_meter = rospy.get_param('ticks_meter', 20)
    prev_vel = [0.0] * rolling_pts
    rospy.loginfo("%s got Kp:%0.3f Ki:%0.3f Kd:%0.3f tpm:%0.3f" % (nodename, Kp, Ki, Kd, ticks_per_meter))
    
    #### subscribers/publishsers 
    rospy.Subscriber("wheel", Int16, wheelCallback) 
    rospy.Subscriber("wheel_vtarget", Float32, targetCallback) 
    pub_motor = rospy.Publisher('motor_cmd', Int16)
   
    r = rospy.Rate(rate) 
    then = rospy.Time.now()
    ticks_since_target = timeout_ticks   # to make sure we don't do the PID loop until we get a message
   
    ###### main loop ############################## 
    while not rospy.is_shutdown():
        previous_error = 0
        prev_vel = [0.0] * rolling_pts
        integral = 0
        error = 0
        derivative = 0 
        vel = 0
        wheel_prev = wheel_latest
        
        while not rospy.is_shutdown() and ticks_since_target < timeout_ticks:
            #do_pid()
            dt_duration = rospy.Time.now() - then
            dt = dt_duration.to_sec()
            then = rospy.Time.now()
    
            cur_vel = (wheel_latest - wheel_prev) / dt
            wheel_prev = wheel_latest

            # calculate rolling average
            prev_vel.append(cur_vel)
            del prev_vel[0]
            p=array(prev_vel)
            vel = p.mean()
            
            error = target - vel
            integral = integral + (error*dt)
            derivative = (error - previous_error) / dt
            previous_error = error
    
            motor = (Kp * error) + (Ki * integral) + (Kd * derivative)
    
            if motor > out_max:
                motor = out_max
                integral = integral - (error*dt)
            if motor < out_min:
                motor = out_min
                integral = integral - (error*dt)
        
            if (target == 0):
                motor = 0
    
            rospy.loginfo("vel:%0.2f tar:%0.2f err:%0.2f int:%0.2f der:%0.2f ## motor:%d " % 
                          (vel, target, error, integral, derivative, motor))
    
    
            pub_motor.publish(motor)

 
            r.sleep()
            ticks_since_target += 1
            if ticks_since_target == timeout_ticks:
                pub_motor.publish(0)
        r.sleep()
    