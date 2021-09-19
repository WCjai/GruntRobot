#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


#############################################################################
class DiffTf:
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)
        
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)       
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.v = 0.0
        self.vl = 0.0
        self.vr = 0.0
        
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.r = rospy.Rate(1.0)
        
        rospy.Subscriber("vr", Float64, self.vrCallback)
        rospy.Subscriber("vl", Float64, self.vlCallback)
        rospy.Subscriber("v", Float64, self.vCallback)
        
    #############################################################################
    def spin(self):
    #############################################################################
        #r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
       
     
    #############################################################################
    def update(self):
    #############################################################################
        now = rospy.Time.now()
        self.current_time = rospy.Time.now()

        # compute odometry in a typical way given the velocities of the robot
        dt = (self.current_time - self.last_time).to_sec()
        self.vx = self.v * cos(self.vth)
        self.vy = self.v * sin(self.vth)
        self.vth = ((self.vr + self.vl)/0.25)*10
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
       

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
             (self.x, self.y, 0.),
             odom_quat,
             self.current_time,
             "base_link",
             "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # publish the message
        self.odom_pub.publish(odom)

        self.last_time = self.current_time
        self.r.sleep()
        
    def vrCallback(self, msg):
        self.vr = msg.data

    def vlCallback(self, msg):
        self.vl = msg.data    
    
    def vCallback(self, msg):
        self.v = msg.data
           

            

#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
